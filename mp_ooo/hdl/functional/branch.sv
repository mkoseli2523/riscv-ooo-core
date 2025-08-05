module branch
import rv32i_types::*;
(   
    input   logic       clk,
    input   logic       rst,

    input   logic       stall_br,

    input   instr_pkt   ex_pkt,
    output  instr_pkt   br_pkt
);

    logic   [31:0]  rd_data;
    logic           br_en;
    logic           br_result;
    logic   [31:0]  br_target;

    logic   [2:0]   cmpop;
    logic   [31:0]  a;
    logic   [31:0]  b;

    logic signed   [31:0] as;
    logic signed   [31:0] bs;
    logic unsigned [31:0] au;
    logic unsigned [31:0] bu;

    assign as =   signed'(a);
    assign bs =   signed'(b);
    assign au = unsigned'(a);
    assign bu = unsigned'(b);

    always_comb begin : OPERANDS
        a = 'x;
        b = 'x;

        if (ex_pkt.i_uses_rs1) // a
            a = ex_pkt.rs1_data;

        if (ex_pkt.i_uses_rs2) // b
            b = ex_pkt.rs2_data;
        else if (ex_pkt.i_uses_imm)
            b = ex_pkt.imm_data;
    end
    
    always_comb begin : BRANCH_CALC
        cmpop     = 'x;
        rd_data   = 'x;
        br_result = 'x;
        br_target = 'x;
        unique case (ex_pkt.i_opcode)
            op_b_jal : begin
                rd_data = ex_pkt.i_pc + 3'd4;

                br_result = '1;
                br_target = ex_pkt.i_pc + ex_pkt.imm_data;
            end
            op_b_jalr : begin
                rd_data = ex_pkt.i_pc + 3'd4;

                br_result = '1;
                br_target = (a + ex_pkt.imm_data) & 32'hfffffffe;
            end
            op_b_br : begin
                cmpop = ex_pkt.i_funct3;

                if (br_en) begin  // branch taken
                    br_result = '1;
                    br_target = ex_pkt.i_pc + ex_pkt.imm_data;
                end else begin
                    br_result = '0;
                    br_target = '0;
                end
            end
            default :;
        endcase
    end

    always_comb begin : CMPOP
        unique case (cmpop)
            branch_f3_beq : br_en = (au == bu);
            branch_f3_bne : br_en = (au != bu);
            branch_f3_blt : br_en = (as <  bs);
            branch_f3_bge : br_en = (as >= bs);
            branch_f3_bltu: br_en = (au <  bu);
            branch_f3_bgeu: br_en = (au >= bu);
            default       : br_en = 1'bx;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!stall_br) begin
            br_pkt.i_valid <= 1'b0;
        end

        br_pkt <= ex_pkt;
        
        br_pkt.i_pc_next <= br_result ? br_target : ex_pkt.i_pc_next;
        br_pkt.br_result <= br_result;
        br_pkt.br_target <= br_target;
        br_pkt.rd_data   <= rd_data;

        if (rst) begin
            br_pkt.i_valid <= '0;
        end 
        else if (stall_br) begin
            br_pkt <= br_pkt;
        end
    end

endmodule : branch