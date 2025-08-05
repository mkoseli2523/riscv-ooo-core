module fetch
import rv32i_types::*;
(
    input   logic               clk,
    input   logic               rst,
    input   logic               stall_pc,
    input   logic               pc_sel,
    input   logic   [31:0]      br_pc,
    input   logic   [63:0]      br_order,
    
    output  logic   [31:0]      dfp_addr,
    output  logic               dfp_read,
    output  logic               dfp_write,
    input   logic   [255:0]     dfp_rdata,
    output  logic   [255:0]     dfp_wdata,
    input   logic               dfp_resp,

    input   logic               br_commit_valid,   // needed for predictor 
    input   logic               br_commit_result,
    input   logic   [9:0]       br_commit_idx,
    input   logic               br_commit_mispredict,
    input   logic   [9:0]       br_commit_ghr,

    output  logic               iq_enqueue,
    output  iq_entry            iq_in
);
        instr_pkt       iq_pkt;
        logic [31:0]    icache_addr;
        logic [31:0]    icache_rdata;
        logic           icache_resp;

        logic [31:0]    pc;
        logic [31:0]    pc_next;
        logic [63:0]    order;
        logic [63:0]    order_next;
        logic           valid;

        logic           pc_sel_latch;
        logic [31:0]    br_pc_reg;
        logic [63:0]    br_order_reg;

        logic           discard_response;

        // ================GShare Signals================
        logic br_inst, jmp_inst;
        logic predict_direction;
        logic [9:0] ghr_out;

        assign jmp_inst = valid && (icache_rdata[3] == 1'b1);
        assign br_inst = valid && (icache_rdata[6] == 1'b1 && icache_rdata[2] == 1'b0);

        // calculate br target
        logic           br_jump;
        logic   [31:0]  b_imm, br_jump_pc, ir, jmp_pc, j_imm;

        // ==============================================
        gshare gshare_i (
            .clk(clk),
            .rst(rst),
            
            .pc(pc[11:2]),
            .br_jump_pc(br_jump_pc[11:2]),

            .stall_pc(stall_pc),
            // .icache_miss(icache_miss),
            .br_inst(br_inst && !stall_pc),
            
            .commit_valid(br_commit_valid),
            .commit_outcome(br_commit_result),
            .commit_idx(br_commit_idx),
            .mispredict(br_commit_valid ? br_commit_mispredict : '0),
            .commit_ghr({br_commit_ghr[8:0], br_commit_result}),
            // .commit_ghr({br_commit_ghr[9:1], br_commit_result}),

            .predict_direction(predict_direction),
            .ghr_out(ghr_out)
        );

        // ==============================================

    assign ir = icache_rdata;
    assign b_imm  = {{20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0}; // shift-left preapplied
    assign j_imm  = {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0}; // shift-left preapplied

    always_comb begin : BR_ADDR
        br_jump_pc = pc + b_imm;
        jmp_pc = pc + j_imm;
    end

    always_comb begin : BR_JUMP
        br_jump = '0;

        if (br_inst && predict_direction) begin
            br_jump = 1'b1;
        end
    end

    always_ff @(posedge clk) begin : PC_SEL_LATCH
        if (rst)
            pc_sel_latch <= '0;
        else if (pc_sel)
            pc_sel_latch <= pc_sel;
        else if (icache_resp)
            pc_sel_latch <= '0;
    end

    always_ff @(posedge clk) begin : BR_LATCHES
        if (rst) begin
            br_pc_reg <= '0;
            br_order_reg <= '0;
        end
        else if (pc_sel) begin
            br_pc_reg    <= br_pc;
            br_order_reg <= br_order;
        end
    end

    always_ff @(posedge clk) begin : PC_ORDER
        if (rst) begin
            pc <= pc_rst;
            order <= '0;
        end
        else if (!stall_pc && icache_resp) begin
            pc <= pc_next;
            order <= order_next;
        end
    end

    always_comb begin : PC_ORDER_NEXT
        pc_next    = pc + 'd4;
        order_next = order + 1'b1;
        
        if (br_jump) begin
            order_next = order + 'd1;
            pc_next = br_jump_pc;
        end

        if (jmp_inst) begin
            order_next = order + 'd1;
            pc_next = jmp_pc;
        end

        case ({pc_sel, pc_sel_latch})
            2'b10: begin    // if an ongoing req finished when flush received (unlatched)
                order_next = br_order + 1'b1;
                pc_next    = br_pc;
            end
            2'b01: begin    // was in the middle of ongoing req, but finished (latched) 
                order_next = br_order_reg + 1'b1;
                pc_next    = br_pc_reg;
            end
            // default: begin
            //     if (!br_jump && !jmp_inst) begin
            //         pc_next    = pc + 'd4;
            //         order_next = order + 1'b1;
            //     end
            // end
        endcase
    end

    assign discard_response = pc_sel | (pc_sel_latch & icache_resp);
    assign icache_addr      = (~stall_pc && icache_resp) ? pc_next:pc;

    icache cache_i (
        .clk,
        .rst,
        
        .ufp_addr   (icache_addr),
        .ufp_rmask  ({4{~rst}}),
        .ufp_wmask  ('0),
        .ufp_rdata  (icache_rdata),
        .ufp_wdata  ('x),
        .ufp_resp   (icache_resp),

        .dfp_addr,
        .dfp_read,
        .dfp_write,
        .dfp_rdata,
        .dfp_wdata,
        .dfp_resp
    );

    // interfacing instruction queue
    assign valid      = icache_resp && !discard_response;
    assign iq_enqueue = !stall_pc && valid;

    always_comb begin
        iq_pkt = '{
            i         : icache_rdata,
            i_order   : order,
            i_valid   : valid,
            i_pc      : pc,
            i_pc_next : pc + 'd4,

            i_rob_idx : 'x,
            rob_ready : 'x,
            res_id    : 'x,
            
            i_opcode  : 'x,
            i_uses_rd : 'x,
            i_uses_rs1: 'x,
            i_uses_rs2: 'x,
            i_uses_imm: 'x,
            i_funct3  : 'x,
            i_funct7  : 'x,
            br_mul_div: 'x,
            
            rd_addr   : 'x,
            rd_paddr  : 'x,
            rd_data   : 'x,
            rs1_addr  : 'x,
            rs1_paddr : 'x,
            rs1_data  : 'x,
            rs2_addr  : 'x,
            rs2_paddr : 'x,
            rs2_data  : 'x,
            imm_data  : 'x,
            
            i_dmem_addr : 'x,
            i_dmem_rmask: 'x,
            i_dmem_wmask: 'x,
            i_dmem_rdata: 'x,
            i_dmem_wdata: 'x,
            
            alu_op  : 'x,
            cmp_op  : 'x,
            mul_op  : 'x,
            
            br_pred       : jmp_inst ? 1'b1 : (br_inst ? (predict_direction) : '0),
            br_result     : br_inst ? 'x : '0,
            br_commit_ghr : br_inst ? (ghr_out) : 'x,
            br_target     : 'x,
            
            regfile_we: 'x
        };


        iq_in = '{
            inst : iq_pkt
        };
    end

endmodule : fetch