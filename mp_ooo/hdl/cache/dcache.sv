module dcache
import cache_types::*;
(
    input   logic           clk,
    input   logic           rst,

    // cpu side signals, ufp -> upward facing port
    input   logic   [31:0]  ufp_addr,
    input   logic   [3:0]   ufp_rmask,
    input   logic   [3:0]   ufp_wmask,
    output  logic   [31:0]  ufp_rdata,
    input   logic   [31:0]  ufp_wdata,
    output  logic           ufp_resp,

    // memory side signals, dfp -> downward facing port
    output  logic   [31:0]  dfp_addr,
    output  logic           dfp_read,
    output  logic           dfp_write,
    input   logic   [255:0] dfp_rdata,
    output  logic   [255:0] dfp_wdata,
    input   logic           dfp_resp,

    output  logic           stall_pp_out
);
        localparam       NUM_WAYS = 4;

        cache_req   req, req_next;

        logic RAW;  // load immediately after store to SAME {tag, set} address
        logic WAW;
        logic stall_pp;

        assign stall_pp_out = stall_pp;
        
        logic valid_incoming;
        logic hit;
        logic wb;
        logic alloc;

        // NOTE: The dimension specifies whether
        // it is to be used for reading/writing
        //
        // 0: dedicated for reads, 1: writes

        cache_line_t line[NUM_WAYS][2];
        always_comb for (integer unsigned w = 0; w < NUM_WAYS; w++) // {tag, data} for write port unusued
             {line[w][1].l_tag, line[w][1].l_data} = 'x;

        logic                  valid_csb[NUM_WAYS][2];
        logic                  valid_web[NUM_WAYS][2];
        logic [SET_WIDTH-1:0]  valid_addr[NUM_WAYS][2];
        logic                  valid_din[NUM_WAYS][2];

        logic                  dirty_csb[NUM_WAYS][2];
        logic                  dirty_web[NUM_WAYS][2];
        logic [SET_WIDTH-1:0]  dirty_addr[NUM_WAYS][2];
        logic                  dirty_din[NUM_WAYS][2];

        logic                  tag_csb[NUM_WAYS];
        logic                  tag_web[NUM_WAYS];
        logic [SET_WIDTH-1:0]  tag_addr[NUM_WAYS];
        logic [TAG_WIDTH-1:0]  tag_din[NUM_WAYS];

        logic                       data_csb[NUM_WAYS];
        logic                       data_web[NUM_WAYS];
        logic [DATA_MASK_WIDTH-1:0] data_wmask[NUM_WAYS];
        logic [SET_WIDTH-1:0]       data_addr[NUM_WAYS];
        logic [DATA_WIDTH-1:0]      data_din[NUM_WAYS];

        logic                  plru_csb[2];
        logic                  plru_web[2];
        logic [SET_WIDTH-1:0]  plru_addr[2];
        logic [2:0]            plru_din[2], plru[2], plru_next, plru_next_w;

        logic [TAG_WIDTH-1:0]    tag, tag_reg;
        logic [SET_WIDTH-1:0]    set, set_reg;
        logic [OFFSET_WIDTH-1:0] offset, offset_reg;

        logic [31:0]           ufp_wmask_32;
        logic [31:0]           data_word_in;
        logic [DATA_WIDTH-1:0] data_line_in;

        // Quick facts about dcache
        logic [NUM_WAYS-1:0]         ways_hit; // one-hot encoded
        logic [$clog2(NUM_WAYS)-1:0] way_hit;
        logic [$clog2(NUM_WAYS)-1:0] way_to_evict;

        logic dfp_resp_reg;
        logic dfp_read_reg;
        logic dfp_write_reg;
        logic RAW_reg;
        logic WAW_reg;
    
    assign {tag, set, offset} = ufp_addr;
    assign {tag_reg, set_reg, offset_reg} = req.addr;
    assign ufp_wmask_32 = {{8{req.wmask[3]}}, {8{req.wmask[2]}},
                           {8{req.wmask[1]}}, {8{req.wmask[0]}}};

    // RAW holds when the following is true
    // 1. prev. req and current req are valid
    // 2. prev. req is a store && current req is a load
    // 3. the store has the same {tag, set} as the load
    assign RAW = req.r_valid && |ufp_rmask
              && |req.wmask && ~(|ufp_wmask)
              && hit;

    assign WAW = req.r_valid && |ufp_wmask
             && |req.wmask && hit;

    always_comb begin : HIT_DETECTION
        for (integer unsigned w = 0; w < NUM_WAYS; w++)
            ways_hit[w] = line[w][0].l_valid && (tag_reg == line[w][0].l_tag);

        case (ways_hit)
            4'b0001: way_hit = 2'd0;
            4'b0010: way_hit = 2'd1;
            4'b0100: way_hit = 2'd2;
            4'b1000: way_hit = 2'd3;
            default: way_hit = 'x;
        endcase
    end

    //              plru[0]
    //             /       \
    //         plru[1]    plru[2]
    //          /   \       /   \  
    //       way0   way1  way2  way3
    always_comb begin : PLRU_UPDATE
        unique case ({ways_hit[0], ways_hit[1], ways_hit[2], ways_hit[3]})
            4'b1000: plru_next = {plru[0][2], 2'b00};
            4'b0100: plru_next = {plru[0][2], 2'b10};
            4'b0010: plru_next = {1'b0, plru[0][1], 1'b1};
            4'b0001: plru_next = {1'b1, plru[0][1], 1'b1};
            default: plru_next = plru[0];
        endcase
    end
    
    always_comb begin
        unique case (way_to_evict)
            2'd0: plru_next_w = {plru[0][2], 2'b00};
            2'd1: plru_next_w = {plru[0][2], 2'b10};
            2'd2: plru_next_w = {1'b0, plru[0][1], 1'b1};
            2'd3: plru_next_w = {1'b1, plru[0][1], 1'b1};
            default: plru_next_w = plru[0];
        endcase
    end

    always_comb begin : PLRU_EVICT
        unique casez (plru[0])
            3'b?11:  way_to_evict = 2'd0;
            3'b?01:  way_to_evict = 2'd1;
            3'b1?0:  way_to_evict = 2'd2;
            3'b0?0:  way_to_evict = 2'd3;
            default: way_to_evict = 'x;
        endcase
    end

    always_ff @ (posedge clk) begin
        dfp_resp_reg  <= rst ? '0 : dfp_resp;
        dfp_read_reg  <= rst ? '0 : dfp_read;
        dfp_write_reg <= rst ? '0  : dfp_write;
        RAW_reg       <= rst ? '0  : RAW;
        WAW_reg       <= rst ? '0  : WAW;
    end

    assign valid_incoming = (|ufp_rmask || |ufp_wmask) && ~stall_pp;
    assign stall_pp = (req.r_valid && ~ufp_resp) || RAW_reg;

    always_ff @ (posedge clk) begin : PP_CACHE_REG
        if (rst)
            req.r_valid <= '0;
        else
            req <= req_next;
    end

    assign wb    = req.r_valid && ~hit && line[way_to_evict][0].l_dirty;
    assign alloc = req.r_valid && ~hit && ~line[way_to_evict][0].l_dirty && (~dfp_resp_reg || dfp_write_reg);
    assign hit   = req.r_valid && |ways_hit;

    always_comb begin : UFP
        ufp_resp  = '0;
        ufp_rdata = 'x;

        if (hit && ~RAW_reg) begin
            ufp_resp  = '1;
            ufp_rdata = (|req.rmask) ? line[way_hit][0].l_data[32*offset_reg[4:2] +: 32] : 'x;

            if (req.addr < 32'hAAAAA000)
                ufp_rdata = '0;
        end

        if (|req.wmask && dfp_resp_reg && dfp_read_reg) // write allocate resp
            ufp_resp = '1;
    end

    always_comb begin : DFP
        dfp_read  = alloc;

        dfp_write = wb && !WAW_reg;
        dfp_wdata = wb ? line[way_to_evict][0].l_data : 'x;

        case ({alloc, wb})
            2'b01:   dfp_addr = {line[way_to_evict][0].l_tag, set_reg, OFFSET_WIDTH'(0)};
            2'b10:   dfp_addr = {req.addr[31:SET_LSB], OFFSET_WIDTH'(0)};
            default: dfp_addr = 'x;
        endcase

        if (RAW_reg) begin
            {dfp_read, dfp_write} = '0;
            dfp_addr = 'x;
        end
    end 

    always_comb begin : SRAM
        for (integer unsigned w = 0; w < NUM_WAYS; w++) begin : DEFAULT
            for (integer unsigned i = 0; i < 2; i++) begin
                {valid_csb[w][i], valid_web[w][i]}  = '1;
                {valid_addr[w][i], valid_din[w][i]} = 'x;

                {dirty_csb[w][i], dirty_web[w][i]}  = '1;
                {dirty_addr[w][i], dirty_din[w][i]} = 'x;

                {plru_csb[i], plru_web[i]} = '1;
                {plru_addr[i], plru_din[i]} = 'x;
            end

            {tag_csb[w], tag_web[w]}      = '1;
            {tag_addr[w], tag_din[w]}     = 'x;

            {data_csb[w], data_web[w]}    = '1;
            data_wmask[w]                 = 'x;
            {data_addr[w], data_din[w]}   = 'x;
        end

        if (valid_incoming && WAW) begin // back-back writes
            for (integer unsigned w = 0; w < NUM_WAYS; w++) begin
                {valid_csb[w][0],  dirty_csb[w][0]}  = '0;
                {valid_addr[w][0], dirty_addr[w][0]} = {2{set}};

                {tag_csb[w]}   = '0;
                {tag_addr[w]} = {1{set}};
            end

            plru_csb[0]  = '0;
            plru_addr[0] = set;

            {dirty_csb[way_hit][1], dirty_web[way_hit][1]}  = '0;
            dirty_addr[way_hit][1] = set_reg;
            dirty_din[way_hit][1]  = '1;

            // Write to dcache in appropriate location
            {data_csb[way_hit], data_web[way_hit]}  = '0;
            data_wmask[way_hit] = 32'(req.wmask << (4*offset_reg[4:2]));
            data_word_in = (line[way_hit][0].l_data[32*offset_reg[4:2] +: 32] & ~ufp_wmask_32)
                            | (req.wdata & ufp_wmask_32);
            data_line_in = DATA_WIDTH'(data_word_in << (32*offset_reg[4:2]));
            {data_addr[way_hit], data_din[way_hit]} = {set_reg, data_line_in};

            {plru_csb[1], plru_web[1]}  = '0;
            plru_addr[1] = set_reg;
            plru_din[1]  = plru_next;
        end
        else if (valid_incoming && ~RAW) begin
            for (integer unsigned w = 0; w < NUM_WAYS; w++) begin
                {valid_csb[w][0],  dirty_csb[w][0]}  = '0;
                {valid_addr[w][0], dirty_addr[w][0]} = {2{set}};

                {tag_csb[w], data_csb[w]}   = '0;
                {tag_addr[w], data_addr[w]} = {2{set}};
            end

            plru_csb[0]  = '0;
            plru_addr[0] = set;
        end
        else if (RAW_reg) begin // HIT (RAW edge case, please work, please work ARGHHH)
            for (integer unsigned w = 0; w < NUM_WAYS; w++) begin
                {valid_csb[w][0],  dirty_csb[w][0]}  = '0;
                {valid_addr[w][0], dirty_addr[w][0]} = {2{set_reg}};

                {tag_csb[w], data_csb[w]}   = '0;
                {tag_addr[w], data_addr[w]} = {2{set_reg}};
            end

            plru_csb[0]  = '0;
            plru_addr[0] = set_reg;
        end
        else if (hit) begin            // HIT!
            if (|req.wmask) begin
                {dirty_csb[way_hit][1], dirty_web[way_hit][1]}  = '0;
                dirty_addr[way_hit][1] = set_reg;
                dirty_din[way_hit][1]  = '1;

                // Write to dcache in appropriate location
                {data_csb[way_hit], data_web[way_hit]}  = '0;
                data_wmask[way_hit] = 32'(req.wmask << (4*offset_reg[4:2]));
                data_word_in = (line[way_hit][0].l_data[32*offset_reg[4:2] +: 32] & ~ufp_wmask_32)
                                | (req.wdata & ufp_wmask_32);
                data_line_in = DATA_WIDTH'(data_word_in << (32*offset_reg[4:2]));
                {data_addr[way_hit], data_din[way_hit]} = {set_reg, data_line_in};
            end

            {plru_csb[1], plru_web[1]}  = '0;
            plru_addr[1] = set_reg;
            plru_din[1]  = plru_next;
        end
        else if (~dfp_resp && (wb || alloc || dfp_resp_reg)) begin    // in middle of wb/alloc
            {valid_csb[way_to_evict][0],  dirty_csb[way_to_evict][0]}  = '0;
            {valid_addr[way_to_evict][0], dirty_addr[way_to_evict][0]} = {2{set_reg}};

            {tag_csb[way_to_evict], data_csb[way_to_evict]}   = '0;
            {tag_addr[way_to_evict], data_addr[way_to_evict]} = {2{set_reg}};

            plru_csb[0]  = '0;
            plru_addr[0] = set_reg;
        end
        else if (dfp_resp && wb) begin // wb done!
            valid_csb[way_to_evict][0]  = '0;
            valid_addr[way_to_evict][0] = set_reg;
            dirty_csb[way_to_evict][0]  = '0;
            dirty_addr[way_to_evict][0] = set_reg;

            {tag_csb[way_to_evict], data_csb[way_to_evict]}   = '0;
            {tag_addr[way_to_evict], data_addr[way_to_evict]} = {2{set_reg}};

            plru_csb[0]  = '0;
            plru_addr[0] = set_reg;


            {dirty_csb[way_to_evict][1], dirty_web[way_to_evict][1]}  = '0;
            dirty_addr[way_to_evict][1] = set_reg;
            dirty_din[way_to_evict][1]  = '0;   // clear dirty
            
            {valid_csb[way_to_evict][1], valid_web[way_to_evict][1]}  = '0;
            valid_addr[way_to_evict][1] = set_reg;
            valid_din[way_to_evict][1]  = '0;   // clear valid
        end
        else if (dfp_resp && alloc && |req.rmask) begin // alloc done! (reads)
            {valid_csb[way_to_evict][1], valid_web[way_to_evict][1]}  = '0;
            valid_addr[way_to_evict][1] = set_reg;
            valid_din[way_to_evict][1] = '1;        // mark line as valid

            plru_csb[0]  = '0;
            plru_addr[0] = set_reg;

            {tag_csb[way_to_evict], data_csb[way_to_evict]}   = '0;
            {tag_addr[way_to_evict], data_addr[way_to_evict]} = {2{set_reg}};

            {tag_web[way_to_evict], data_web[way_to_evict]} = '0;
            tag_din[way_to_evict]      = tag_reg;
            data_wmask[way_to_evict]   = '1;
            data_din[way_to_evict]     = dfp_rdata; // put {tag, data} into our cache
        end
        else if (dfp_resp && alloc && |req.wmask) begin // alloc done! (writes)
            {valid_csb[way_to_evict][1], valid_web[way_to_evict][1]}  = '0;
            valid_addr[way_to_evict][1] = set_reg;
            valid_din[way_to_evict][1] = '1;            // mark line as valid

            {dirty_csb[way_to_evict][1], dirty_web[way_to_evict][1]}  = '0;
            dirty_addr[way_to_evict][1] = set_reg;
            dirty_din[way_to_evict][1]  = '1;           // mark that shi dirty

            {tag_csb[way_to_evict], tag_web[way_to_evict]}   = '0;
            tag_addr[way_to_evict] = set_reg;
            tag_din[way_to_evict]  = tag_reg;

            {data_csb[way_to_evict], data_web[way_to_evict]}  = '0;
            data_wmask[way_to_evict] = '1;
            data_word_in = (dfp_rdata[32*offset_reg[4:2] +: 32] & ~ufp_wmask_32)
                         | (req.wdata & ufp_wmask_32);
            data_line_in = DATA_WIDTH'((dfp_rdata & ~({32{1'b1}} << (32*offset_reg[4:2]))) | (data_word_in << (32*offset_reg[4:2])));
            {data_addr[way_to_evict], data_din[way_to_evict]} = {set_reg, data_line_in};

            {plru_csb[1], plru_web[1]}  = '0;
            plru_addr[1] = set_reg;
            plru_din[1]  = plru_next_w;
        end
    end

    always_comb begin : PP_NEXT
        req_next = '{
            r_valid   :  valid_incoming,

            addr      :  ufp_addr,
            rmask     :  ufp_rmask,
            wmask     :  ufp_wmask,
            wdata     :  ufp_wdata
        };

        if (stall_pp) begin
            req_next = '{
                r_valid   :  req.r_valid,

                addr      :  req.addr,
                rmask     :  req.rmask,
                wmask     :  req.wmask,
                wdata     :  req.wdata
            };            
        end
    end

    generate for (genvar k = 0; k < NUM_WAYS; k++) begin : arrays
        dp_ff_array valid_array (
            .clk0       (clk),
            .rst0       (rst),
            .csb0       (valid_csb[k][0]),
            .web0       (valid_web[k][0]),
            .addr0      (valid_addr[k][0]),
            .din0       (valid_din[k][0]),
            .dout0      (line[k][0].l_valid),
            .csb1       (valid_csb[k][1]),
            .web1       (valid_web[k][1]),
            .addr1      (valid_addr[k][1]),
            .din1       (valid_din[k][1]),
            .dout1      (line[k][1].l_valid)
        );
        dp_ff_array dirty_array (
            .clk0       (clk),
            .rst0       (rst),
            .csb0       (dirty_csb[k][0]),
            .web0       (dirty_web[k][0]),
            .addr0      (dirty_addr[k][0]),
            .din0       (dirty_din[k][0]),
            .dout0      (line[k][0].l_dirty),
            .csb1       (dirty_csb[k][1]),
            .web1       (dirty_web[k][1]),
            .addr1      (dirty_addr[k][1]),
            .din1       (dirty_din[k][1]),
            .dout1      (line[k][1].l_dirty)
        );
        mp_cache_tag_array tag_array (
            .clk0       (clk),
            .csb0       (tag_csb[k]),
            .web0       (tag_web[k]),
            .addr0      (tag_addr[k]),
            .din0       (tag_din[k]),
            .dout0      (line[k][0].l_tag)
        );
        mp_cache_data_array data_array (
            .clk0       (clk),
            .csb0       (data_csb[k]),
            .web0       (data_web[k]),
            .wmask0     (data_wmask[k]),
            .addr0      (data_addr[k]),
            .din0       (data_din[k]),
            .dout0      (line[k][0].l_data)
        );
    end endgenerate

    dp_ff_array #(
        .WIDTH      (NUM_WAYS-1)
    ) lru_array (
        .clk0       (clk),
        .rst0       (rst),
        .csb0       (plru_csb[0]),
        .web0       (plru_web[0]),
        .addr0      (plru_addr[0]),
        .din0       (plru_din[0]),
        .dout0      (plru[0]),
        .csb1       (plru_csb[1]),
        .web1       (plru_web[1]),
        .addr1      (plru_addr[1]),
        .din1       (plru_din[1]),
        .dout1      (plru[1])
    );

endmodule : dcache
