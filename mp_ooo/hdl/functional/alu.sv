module alu
import rv32i_types::*;
(
    input   instr_pkt   ex_pkt,
    output  instr_pkt   alu_pkt
);
            logic   [6:0]   opcode; assign opcode = ex_pkt.i_opcode;

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

            logic   [31:0] aluout;
            logic          slt;

            logic   [31:0] rd_out;

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

    always_comb begin : ALU
        unique case (ex_pkt.alu_op)
            alu_op_add: aluout = au +   bu;
            alu_op_sll: aluout = au <<  bu[4:0];            
            alu_op_sra: aluout = unsigned'(as >>> bu[4:0]); 
            alu_op_sub: aluout = au -   bu;
            alu_op_xor: aluout = au ^   bu;
            alu_op_srl: aluout = au >>  bu[4:0];            
            alu_op_or : aluout = au |   bu;
            alu_op_and: aluout = au &   bu;
            default   : aluout = 'x;
        endcase
    end

    always_comb begin : SLT
        slt = 1'b0;

        case (ex_pkt.cmp_op)
            branch_f3_blt : slt = (as <  bs);
            branch_f3_bltu: slt = (au <  bu);
        endcase
    end

    always_comb begin : RD
        unique case (opcode)
            op_b_lui:              rd_out = ex_pkt.imm_data;
            op_b_auipc:            rd_out = ex_pkt.i_pc + ex_pkt.imm_data;
            op_b_imm, op_b_reg: begin
                unique case (ex_pkt.i_funct3)
                    arith_f3_slt:  rd_out = {31'd0, slt};
                    arith_f3_sltu: rd_out = {31'd0, slt};
                    default:       rd_out = aluout;
                endcase
            end
            default:    rd_out = 'x;
        endcase
    end

    always_comb begin
        alu_pkt = ex_pkt;
        alu_pkt.rd_data = rd_out;
    end

endmodule : alu