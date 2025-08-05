module gshare
# (
    parameter GHR_BITS = 10,     // GHR size
    parameter PHT_ENTRIES = 1 << GHR_BITS
) (
    input   logic                   clk,
    input   logic                   rst,
    
    input   logic [GHR_BITS-1:0]    pc,
    input   logic [GHR_BITS-1:0]    br_jump_pc,

    input   logic                   stall_pc, 
    input   logic                   br_inst, // check opcode in fetch
    
    input   logic                   commit_valid,
    input   logic                   commit_outcome,
    input   logic [GHR_BITS-1:0]    commit_idx,
    input   logic                   mispredict,
    input   logic [GHR_BITS-1:0]    commit_ghr,

    // output  logic [GHR_BITS-1:0]    predict_idx,    // can just use PC & GHR stored in ROB
    output  logic                   predict_direction,  // 1 if taken
    output  logic [GHR_BITS-1:0]    ghr_out
);

    /**
        
        GShare Overview:
            - Predictions are made by indexing into a table (1k size) of 2bit counters
        
        Counter Array:
            - 1k array of dual port SRAM is used for the counter array (with 2bits per entry
            allocated for each counter)
            - Writes take 2-cycles, therefore the write result is latched in a flip-flop and
            checked against the incoming request to ensure counter values are most up to date

        GHR:
            - GHR is updated speculatively, therefore on a misprediction reset it to whatever
            was saved on ROB

        Making a Prediction:
            - Index into the SRAM and the valid array at the same cycle
            - If valid-out is true, use the SRAM output
            - If valid-out is false, simply predict taken

        Updating the Predictor:
            - If not a mispredict, update the predictor to either 2'b11 (strongly-taken) or
            2'b00 (strongly-not-taken) depending on the outcome of the branch
            - If mispredict first read the counter, and the valid array:
                - If index is valid, use SRAM out's hysteresis_bit to make a prediction
                - If index is invalid, write 2'b01 to given index
                **NOTE that we don't need to consider weird edge-cases with having 2-mispredictions
                back-to-back etc. 

    */

    logic [GHR_BITS-1:0] pc_next;

    always_comb begin
        // pc_next = pc; 

        pc_next = pc + 1'b1;

        if (br_inst && predict_direction) begin
            pc_next = br_jump_pc;
        end
    end

    logic [GHR_BITS-1:0] ghr;

    assign ghr_out = ghr;
    
    logic [GHR_BITS-1:0] pht_index;

    always_comb begin : CALCULATE_INDEX
        // if (commit_valid && mispredict) begin
            // pht_index = pc_next[GHR_BITS+1:2] ^ commit_ghr; 
        // end 
        // else 
        if (br_inst) begin
            pht_index = pc_next ^ {ghr[8:0], predict_direction};
        end 
        else begin
            pht_index = pc_next ^ ghr;
        end

        // pht_index = pc_next[GHR_BITS+1:2] ^ ghr;
    end

    // SRAM is 64x32; write size = 2; dual-ported
    logic        ca_csb0, ca_csb1;
    logic [5:0]  ca_addr0, ca_addr1;
    logic [15:0] ca_wmask1;
    logic [31:0] ca_din1;
    logic [31:0] ca_dout0; 

    // valid-array
    logic                   valid_csb0[32], valid_csb1[32];
    logic [GHR_BITS-1-5:0]  valid_addr0[32], valid_addr1[32];
    logic                   valid_dout0[32];

    generate
        for (genvar i = 0; i < 2**5; i++) begin
            sp_ff_array_dp #(
                .S_INDEX(GHR_BITS-5)
            ) ghsare_valid_array (
                .clk        (clk),
                .rst        (rst),

                .csb0       (valid_csb0[i]),
                .web0       (1'b1),
                .addr0      (valid_addr0[i]),
                .din0       ('x), // port 0 is only used for reading
                .dout0      (valid_dout0[i]),

                .csb1       (valid_csb1[i]),
                .web1       (1'b0), // port 1 is only used for writing
                .addr1      (valid_addr1[i]),
                .din1       (1'b1), // won't ever have to invalidate an entry
                .dout1      ()
            ); 
        end
    endgenerate

    logic ca_csb0_latch, ca_csb0_latch_latch;
    logic ca_csb1_latch, ca_csb1_latch_latch;

    always_ff @ (posedge clk) begin
        ca_csb0_latch <= ca_csb0;
        ca_csb0_latch_latch <= ca_csb0_latch;
        ca_csb1_latch <= ca_csb1;
        ca_csb1_latch_latch <= ca_csb1_latch;
    end

    gshare_counter_array counters (
        .clk0(clk),
        .csb0(ca_csb0),
        .web0(1'b1), // port 0 is only used for reading
        .wmask0('x),
        .addr0(ca_addr0),
        .din0('x),  
        .dout0(ca_dout0),
        
        .clk1(clk),
        .csb1(ca_csb1),
        .web1(1'b0), // port 1 is only used for writing
        .wmask1(ca_wmask1),
        .addr1(ca_addr1),
        .din1(ca_din1),
        .dout1()
    );

    logic [GHR_BITS-1:0] pht_index_latch;
    logic [GHR_BITS-1:0] commit_idx_latch;
    logic                mispredict_reg;
    logic                commit_valid_reg;
    logic                commit_valid_reg_reg;

    always_ff @(posedge clk) begin
        pht_index_latch <= pht_index;
        commit_idx_latch <= commit_idx;
        mispredict_reg <= mispredict;
        commit_valid_reg <= commit_valid;
        commit_valid_reg_reg <= commit_valid_reg;
    end

    always_comb begin : PORT_ZERO_SRAM
        ca_csb0 = 1'b1;
        ca_addr0 = 'x;
        
        if (mispredict) begin
            ca_csb0  = 1'b0;
            ca_addr0 = commit_idx[9:4];
        end else begin
            ca_csb0  = 1'b0;
            ca_addr0 = pht_index[9:4];
        end
    end

    logic [15:0] ca_wmask1_expanded;

    always_comb begin : EXPAND_WMASK
        ca_wmask1_expanded = 'x;
        
        if (mispredict_reg) begin            
            unique case (commit_idx_latch[3:0])
                4'b0000: ca_wmask1_expanded = 16'b0000_0000_0000_0001; 
                4'b0001: ca_wmask1_expanded = 16'b0000_0000_0000_0010;
                4'b0010: ca_wmask1_expanded = 16'b0000_0000_0000_0100; 
                4'b0011: ca_wmask1_expanded = 16'b0000_0000_0000_1000;
                4'b0100: ca_wmask1_expanded = 16'b0000_0000_0001_0000;
                4'b0101: ca_wmask1_expanded = 16'b0000_0000_0010_0000;
                4'b0110: ca_wmask1_expanded = 16'b0000_0000_0100_0000;
                4'b0111: ca_wmask1_expanded = 16'b0000_0000_1000_0000;
                4'b1000: ca_wmask1_expanded = 16'b0000_0001_0000_0000;
                4'b1001: ca_wmask1_expanded = 16'b0000_0010_0000_0000;
                4'b1010: ca_wmask1_expanded = 16'b0000_0100_0000_0000;
                4'b1011: ca_wmask1_expanded = 16'b0000_1000_0000_0000;
                4'b1100: ca_wmask1_expanded = 16'b0001_0000_0000_0000;
                4'b1101: ca_wmask1_expanded = 16'b0010_0000_0000_0000;
                4'b1110: ca_wmask1_expanded = 16'b0100_0000_0000_0000;
                4'b1111: ca_wmask1_expanded = 16'b1000_0000_0000_0000;
                default: ca_wmask1_expanded = 'x;
            endcase
        end else if (commit_valid) begin 
            unique case (commit_idx[3:0])
                4'b0000: ca_wmask1_expanded = 16'b0000_0000_0000_0001; 
                4'b0001: ca_wmask1_expanded = 16'b0000_0000_0000_0010;
                4'b0010: ca_wmask1_expanded = 16'b0000_0000_0000_0100; 
                4'b0011: ca_wmask1_expanded = 16'b0000_0000_0000_1000;
                4'b0100: ca_wmask1_expanded = 16'b0000_0000_0001_0000;
                4'b0101: ca_wmask1_expanded = 16'b0000_0000_0010_0000;
                4'b0110: ca_wmask1_expanded = 16'b0000_0000_0100_0000;
                4'b0111: ca_wmask1_expanded = 16'b0000_0000_1000_0000;
                4'b1000: ca_wmask1_expanded = 16'b0000_0001_0000_0000;
                4'b1001: ca_wmask1_expanded = 16'b0000_0010_0000_0000;
                4'b1010: ca_wmask1_expanded = 16'b0000_0100_0000_0000;
                4'b1011: ca_wmask1_expanded = 16'b0000_1000_0000_0000;
                4'b1100: ca_wmask1_expanded = 16'b0001_0000_0000_0000;
                4'b1101: ca_wmask1_expanded = 16'b0010_0000_0000_0000;
                4'b1110: ca_wmask1_expanded = 16'b0100_0000_0000_0000;
                4'b1111: ca_wmask1_expanded = 16'b1000_0000_0000_0000;
                default: ca_wmask1_expanded = 'x;
            endcase
        end
    end

    logic [GHR_BITS-1:0] write_address, write_address_latch;
    logic [1:0]          write_data, write_data_latch;
    logic write_latch_valid, write_latch_valid_latch;

    always_ff @(posedge clk) begin
        write_address_latch <= write_address;
        write_data_latch <= write_data;
        write_latch_valid_latch <= write_latch_valid;
    end

    always_comb begin : PORT_ONE_SRAM
        ca_csb1   = 1'b1;
        ca_wmask1 = 'x;
        ca_addr1  = 'x;
        ca_din1   = 'x;

        write_latch_valid = '0;
        write_address = 'x;
        write_data = 'x;

        if (mispredict_reg) begin
            ca_csb1   = 1'b0;
            ca_wmask1 = ca_wmask1_expanded;
            ca_addr1  = commit_idx_latch[9:4];

            write_address = commit_idx_latch;
            write_latch_valid = 1'b1;


            if (valid_dout0[commit_idx_latch[9:5]]) begin
                // set predict_bit to hysteresis_bit
                write_data = {ca_dout0[{commit_idx_latch[3:0], 1'b0}], ~ca_dout0[{commit_idx_latch[3:0], 1'b0}]};
                ca_din1 = {30'b0, write_data} << {commit_idx_latch[3:0], 1'b0};
            end else begin
                write_data = 2'b10;
                ca_din1 = {30'b0, write_data} << {commit_idx_latch[3:0], 1'b0};
            end
        end else if (commit_valid && !mispredict) begin
            ca_csb1   = 1'b0;
            ca_wmask1 = ca_wmask1_expanded;
            ca_addr1  = commit_idx[9:4];

            write_latch_valid = 1'b1;
            write_address = commit_idx;
            write_data = {commit_outcome, commit_outcome};

            ca_din1   = {30'b0, write_data} << {commit_idx[3:0], 1'b0};
        end
    end

    always_comb begin : VALID_ARRAY
        for (integer i = 0; i < 2**5; i++) begin
            valid_csb0[i]  = 1'b1;
            valid_addr0[i] = 'x;

            valid_csb1[i]  = 1'b1;
            valid_addr1[i] = 'x;
        end
        
        if (mispredict) begin
            valid_csb0[commit_idx[9:5]]  = 1'b0;
            valid_addr0[commit_idx[9:5]] = commit_idx[4:0];
        end else begin
            valid_csb0[pht_index[9:5]]  = 1'b0;
            valid_addr0[pht_index[9:5]] = pht_index[4:0];
        end

        if (mispredict_reg) begin
            valid_csb1[commit_idx_latch[9:5]] = 1'b0;
            valid_addr1[commit_idx_latch[9:5]] = commit_idx_latch[4:0];
        end else if (commit_valid && !mispredict) begin
            valid_csb1[commit_idx[9:5]]  = 1'b0;
            valid_addr1[commit_idx[9:5]] = commit_idx[4:0];
        end
    end

    logic predict_direction_latch;
    logic stall_pc_reg;
    
    always_comb begin : MAKE_PREDICTION
        if (stall_pc_reg) begin
            predict_direction = predict_direction_latch;
        end else if (write_latch_valid_latch && write_address_latch == pht_index_latch) begin
            predict_direction = write_data_latch[1];
        end 
        // else if (write_latch_valid && write_address == pht_index_latch) begin
        //     predict_direction = write_data[1];
        // end 
        else if (mispredict_reg) begin
            predict_direction = 1'b1;
        end 
        else if (valid_dout0[pht_index_latch[9:5]]) begin
            predict_direction = ca_dout0[{pht_index_latch[3:0], 1'b1}];
        end else begin
            predict_direction = 1'b0;
        end
    end

    always_ff @(posedge clk) begin
        stall_pc_reg <= stall_pc;

        if (rst) begin
            predict_direction_latch <= 1'b0;
        end 
        else if (stall_pc) begin
            predict_direction_latch <= predict_direction;
        end 
    end

    always_ff @(posedge clk) begin : UPDATE_GHR 
        if (rst) begin
            ghr <= '0;
        end 
        else if (commit_valid && mispredict) begin
            ghr <= commit_ghr;
        end 
        else if (br_inst) begin
            ghr <= {ghr[8:0], predict_direction};
        end
        // else if (commit_valid) begin
        //     ghr <= commit_ghr;
        // end
    end

endmodule : gshare    
