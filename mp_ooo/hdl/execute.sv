module execute
import rv32i_types::*;
(
    input   logic             clk,
    input   logic             rst,

    input   logic     [31:0]  rs1_prdata[NUM_RES_STATIONS],
    input   logic     [31:0]  rs2_prdata[NUM_RES_STATIONS],

    input   instr_pkt         br_mul_div_iss,
    input   instr_pkt         alu_iss,
    input   instr_pkt         mem_iss,
    input   logic             is_store,

    output  logic     [2:0]   stall_br_mul_div, // one-hot encoded
    output  logic             stall_alu,
    output  logic             stall_store,
    output  logic             stall_load,
    input   logic             stall_mem_latch,
    output  logic     [31:0]  dfp_addr,
    output  logic             dfp_read,
    output  logic             dfp_write,
    input   logic     [255:0] dfp_rdata,
    output  logic     [255:0] dfp_wdata,
    input   logic             dfp_resp,
    input   logic             br_flush, 

    output  cdb_t             ex_out[CDB_PORTS]
);
        logic       stall_br; 
        logic       stall_div;
        logic       div_in_progress;

        logic stall_pp;

        instr_pkt   br_reg, br_reg_next;
        instr_pkt   mul_reg, mul_reg_next;
        instr_pkt   div_reg, div_reg_next;
        instr_pkt   alu_reg, alu_reg_next;
        instr_pkt   lq_reg[2], lq_reg_next[2];

        instr_pkt   br_out, mul_out, div_out;

        logic [31:0] dcache_addr;
        logic [3:0]  dcache_rmask;
        logic [3:0]  dcache_wmask;
        logic [31:0] dcache_rdata;
        logic [31:0] dcache_wdata;
        logic        dcache_resp;

        logic [31:0] raw_dcache_addr;
        logic [31:0] ld_rd_wdata;

        logic br_flush_latch;

        logic           store_valid;
        logic [31:0]    store_address_in;
        logic [3:0]     store_wmask_in;
        logic [31:0]    store_data_in;
        
        logic           load_valid;
        logic [31:0]    load_address;

        logic           dcache_busy;
        logic           pcsb_dcache_req;
        logic [31:0]    pcsb_dcache_address;
        logic [3:0]     pcsb_dcache_wmask;
        logic [31:0]    pcsb_dcache_data;

        logic           hit;
        logic [31:0]    store_address_out;
        logic [3:0]     store_wmask_out;
        logic [31:0]    store_data_out;

        logic           pcsb_full;

    always_ff @(posedge clk) begin : FLUSH_LATCH
        if (rst)
            br_flush_latch <= '0;
        else if (br_flush && (dfp_read | dfp_write))
            br_flush_latch <= br_flush;
        else if (dcache_resp)
            br_flush_latch <= '0;
    end

    always_ff @ (posedge clk) begin : EX_REGS
        if (rst | br_flush) begin
            br_reg.i_valid  <= '0; 
            mul_reg.i_valid <= '0; 
            div_reg.i_valid <= '0; 
            alu_reg.i_valid <= '0;
        end
        else begin
            br_reg  <= stall_br ? br_reg:br_reg_next;  
            mul_reg <= mul_reg_next;  
            div_reg <= div_reg_next;  
            alu_reg <= alu_reg_next;  
        end
    end

    // logic issued_store;

    // always_ff @(posedge clk) begin
    //     if (pcsb_dcache_req) begin
    //         issued_store <= 1'b1;
    //     end

    //     if (dcache_resp || rst) begin
    //         issued_store <= 1'b0; 
    //     end
    // end

    always_ff @ (posedge clk) begin : MEM_REGS
        if (rst | br_flush)
            {lq_reg[0].i_valid, lq_reg[1].i_valid} <= '0;
        else if (stall_load && !(hit && ((store_wmask_out & lq_reg[1].i_dmem_rmask) == lq_reg[1].i_dmem_rmask))) begin
            lq_reg[0].i_valid <= '0;
            lq_reg[1] <= lq_reg[1];
        end
        else begin
            lq_reg[1] <= lq_reg_next[1];
            lq_reg[0] <= lq_reg_next[0];
        end
    end

    assign stall_br_mul_div = {stall_br, 1'b0, (stall_div | div_in_progress)};

    always_comb begin : REDIRECT_BR_MUL_DIV_ISS
        br_reg_next          = 'x;
        br_reg_next.i_valid  = 1'b0;

        mul_reg_next         = 'x;
        mul_reg_next.i_valid = 1'b0;

        div_reg_next         = 'x;
        div_reg_next.i_valid = 1'b0;


        if (br_mul_div_iss.i_valid) begin
            unique case (br_mul_div_iss.br_mul_div)
                3'b001: begin
                    div_reg_next = br_mul_div_iss;
                    div_reg_next.rs1_data = rs1_prdata[0];
                    div_reg_next.rs2_data = rs2_prdata[0];
                end
                3'b010: begin
                    mul_reg_next = br_mul_div_iss;
                    mul_reg_next.rs1_data = rs1_prdata[0];
                    mul_reg_next.rs2_data = rs2_prdata[0];
                end
                3'b100: begin
                    br_reg_next = br_mul_div_iss;
                    br_reg_next.rs1_data = rs1_prdata[0];
                    br_reg_next.rs2_data = rs2_prdata[0];
                end
                default:;
            endcase
        end
    end

    branch branch_i (
        .clk(clk),
        .rst(rst || br_flush),

        .stall_br(stall_br),
        
        .ex_pkt(br_reg),
        .br_pkt(br_out)
    );

    mul mul_i (
        .clk,
        .rst(br_flush || rst),

        .ex_pkt(mul_reg),
        .mul_pkt(mul_out)
    );

    div div_i (
        .clk,
        .rst(rst),
        .br_flush,

        .stall_div,
        .div_in_progress,

        .ex_pkt(div_reg),
        .div_pkt(div_out)
    );

    // ~~~ CDB_0 bus fixed priority ~~~
    // 1. mul
    // 2. div
    // 3. br
    always_comb begin : PRIORITY_BR_MUL_DIV
        {stall_div, stall_br} = '0;

        unique case ({br_out.i_valid, mul_out.i_valid, div_out.i_valid})
            3'b001: ex_out[0] = div_out;
            3'b010: ex_out[0] = mul_out;
            3'b011: begin // div stall; mul pass
                ex_out[0] = mul_out;
                stall_div = '1;
            end
            3'b100: ex_out[0] = br_out;
            3'b101: begin // br stall; div pass
                ex_out[0] = div_out;
                stall_br = '1;
            end
            3'b110: begin // br stall; mul pass
                ex_out[0] = mul_out;
                stall_br = '1;
            end
            3'b111: begin // br, div stall; mul pass
                ex_out[0] = mul_out;
                {stall_div, stall_br} = '1;
            end
            default: begin
                ex_out[0] = 'x;
                ex_out[0].inst.i_valid = '0;                
            end
        endcase
    end

    alu alu_i (
        .ex_pkt(alu_reg),
        .alu_pkt(ex_out[1].inst)
    );

    assign stall_alu = '0;
    always_comb begin
        alu_reg_next          = alu_iss;
        alu_reg_next.rs1_data = rs1_prdata[1];
        alu_reg_next.rs2_data = rs2_prdata[1];
    end

    always_comb begin : ADDR_CALC
        dcache_addr     = 'x;
        {dcache_rmask, dcache_wmask} = '0;
        dcache_wdata = 'x;

        if (mem_iss.i_valid && !is_store) begin : LOAD
            dcache_addr = lq_reg_next[1].rs1_data + mem_iss.imm_data;

            unique case (mem_iss.i_funct3)
                    load_f3_lb, load_f3_lbu: dcache_rmask = 4'b0001 << dcache_addr[1:0];
                    load_f3_lh, load_f3_lhu: dcache_rmask = 4'b0011 << dcache_addr[1:0];
                    load_f3_lw             : dcache_rmask = 4'b1111;
                    default                : dcache_rmask = 'x;
            endcase
        end

        if (pcsb_dcache_req) begin : STORE
            dcache_addr = pcsb_dcache_address;
            dcache_wmask = pcsb_dcache_wmask;
            dcache_wdata = pcsb_dcache_data;
        end
        dcache_addr[1:0] = 2'd0;  // all memory request has to be naturally aligned
    end
    // assign stall_load = (br_flush_latch && (dfp_read | dfp_write))
    //                 || (lq_reg[1].i_valid && ~dcache_resp);

    assign stall_load = (br_flush_latch)
                        || (lq_reg[1].i_valid && ~dcache_resp)
                        || (stall_pp);

    assign raw_dcache_addr = lq_reg[1].rs1_data + lq_reg[1].imm_data;

    logic [3:0] wmask_sum;

    assign wmask_sum = lq_reg[1].i_dmem_rmask & store_wmask_out;

    logic [31:0] dcache_tot;

    always_comb begin : LOAD_RMASK
        ld_rd_wdata = '0;

        if (lq_reg[1].i_valid && dcache_resp
        && (lq_reg[1].i_opcode == op_b_load)) begin
            unique case (lq_reg[1].i_funct3)
                load_f3_lb : ld_rd_wdata = {{24{dcache_rdata[7 +8 *raw_dcache_addr[1:0]]}}, dcache_rdata[8 *raw_dcache_addr[1:0] +: 8 ]};
                load_f3_lbu: ld_rd_wdata = {{24{1'b0}}, dcache_rdata[8 *raw_dcache_addr[1:0] +: 8 ]};
                load_f3_lh : ld_rd_wdata = {{16{dcache_rdata[15+16*raw_dcache_addr[1]  ]}}, dcache_rdata[16*raw_dcache_addr[1] +: 16]};
                load_f3_lhu: ld_rd_wdata = {{16{1'b0}}, dcache_rdata[16*raw_dcache_addr[1] +: 16]};
                load_f3_lw : ld_rd_wdata = dcache_rdata;
                default:;
            endcase

            if (hit) begin
                for (integer i = 0; i < 4; i++) begin
                    if (wmask_sum[i]) begin
                        ld_rd_wdata[i*8 +: 8] = store_data_out[i*8 +: 8];
                    end
                end
            end
        end
        
        if (hit && ((store_wmask_out & lq_reg[1].i_dmem_rmask) == lq_reg[1].i_dmem_rmask)) begin
            unique case (lq_reg[1].i_funct3)
                load_f3_lb : ld_rd_wdata = {{24{store_data_out[7 +8 *raw_dcache_addr[1:0]]}}, store_data_out[8 *raw_dcache_addr[1:0] +: 8 ]};
                load_f3_lbu: ld_rd_wdata = {{24{1'b0}}, store_data_out[8 *raw_dcache_addr[1:0] +: 8 ]};
                load_f3_lh : ld_rd_wdata = {{16{store_data_out[15+16*raw_dcache_addr[1]  ]}}, store_data_out[16*raw_dcache_addr[1] +: 16]};
                load_f3_lhu: ld_rd_wdata = {{16{1'b0}}, store_data_out[16*raw_dcache_addr[1] +: 16]};
                load_f3_lw : ld_rd_wdata = store_data_out;
                default:;
            endcase
        end    

        dcache_tot = dcache_rdata;

        if (hit) begin
            for (integer i = 0; i < 4; i++) begin
                if (store_wmask_out[i]) begin
                    dcache_tot[i*8 +: 8] = store_data_out[i*8 +: 8];
                end
            end
        end
    end

    always_comb begin : PCSB_IN
        store_valid = '0;

        store_address_in = 'x;
        store_wmask_in   = 'x;
        store_data_in    = 'x;

        if (mem_iss.i_valid && is_store) begin
            store_valid = 1'b1;
        end

        store_address_in = lq_reg_next[1].rs1_data + mem_iss.imm_data;
            
        unique case (mem_iss.i_funct3)
            store_f3_sb: store_wmask_in = 4'b0001 << store_address_in[1:0];
            store_f3_sh: store_wmask_in = 4'b0011 << store_address_in[1:0];
            store_f3_sw: store_wmask_in = 4'b1111;
            default    : store_wmask_in = 'x;
        endcase
        unique case (mem_iss.i_funct3)
            store_f3_sb: store_data_in[8 *store_address_in[1:0] +: 8 ] = lq_reg_next[1].rs2_data[7 :0];
            store_f3_sh: store_data_in[16*store_address_in[1]   +: 16] = lq_reg_next[1].rs2_data[15:0];
            store_f3_sw: store_data_in = lq_reg_next[1].rs2_data;
            default    : store_data_in = 'x;
        endcase
        store_address_in[1:0] = '0;
    end

    always_comb begin : LOAD_IN
        load_valid = '0;
        load_address = 'x;

        if (mem_iss.i_valid && !is_store) begin
            load_valid   = 1'b1;
            load_address = dcache_addr;
        end
    end


    // logic dfp_read_latch;

    // always_ff @(posedge clk) dfp_read_latch <= dfp_read;

    // assign dcache_busy = (lq_reg[1].i_valid) | (mem_iss.i_valid && !is_store) | (issued_store) | hit | br_flush_latch | dfp_read_latch | dfp_read | dfp_write;
    assign dcache_busy = (mem_iss.i_valid && !is_store) | stall_pp;

    pcsb pcsb_i (
        .clk(clk),
        .rst(rst),

        .store_valid(store_valid),
        .store_address_in(store_address_in),
        .store_wmask_in(store_wmask_in),
        .store_data_in(store_data_in),

        .load_valid(load_valid),
        .load_address(load_address),

        .dcache_busy(dcache_busy),
        .dcache_resp(dcache_resp),
        .stall_pp(stall_pp),
        .pcsb_dcache_req(pcsb_dcache_req),
        .pcsb_dcache_address(pcsb_dcache_address),
        .pcsb_dcache_wmask(pcsb_dcache_wmask),
        .pcsb_dcache_data(pcsb_dcache_data),

        .hit(hit),
        .store_address_out(store_address_out),
        .store_wmask_out(store_wmask_out),
        .store_data_out(store_data_out),

        .pcsb_full(pcsb_full)
    );

    assign stall_store = pcsb_full;

    dcache cache_d (
        .clk,
        .rst,
        
        .ufp_addr   (dcache_addr),
        .ufp_rmask  (dcache_rmask),
        .ufp_wmask  (dcache_wmask),
        .ufp_rdata  (dcache_rdata),
        .ufp_wdata  (dcache_wdata),
        .ufp_resp   (dcache_resp),

        .dfp_addr,
        .dfp_read,
        .dfp_write,
        .dfp_rdata,
        .dfp_wdata,
        .dfp_resp,

        .stall_pp_out(stall_pp)
    );

    always_comb begin : MEM_PP_NEXT
        lq_reg_next[1]              = !is_store ? mem_iss : '0;
        lq_reg_next[1].rs1_data     = !stall_mem_latch ? rs1_prdata[2] : mem_iss.rs1_data;
        lq_reg_next[1].rs2_data     = !stall_mem_latch ? rs2_prdata[2] : mem_iss.rs2_data;
        lq_reg_next[1].i_dmem_addr  = dcache_addr;
        lq_reg_next[1].i_dmem_rmask = dcache_rmask;
        lq_reg_next[1].i_dmem_wmask = '0;
        lq_reg_next[1].i_dmem_wdata = dcache_wdata;

        lq_reg_next[0]              = lq_reg[1];
        lq_reg_next[0].i_dmem_rdata = dcache_tot;
        lq_reg_next[0].rd_data      = ld_rd_wdata;
    end

    assign ex_out[2] = lq_reg[0];

endmodule : execute