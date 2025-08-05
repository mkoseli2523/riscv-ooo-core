module decode
import rv32i_types::*;
(
    input   iq_entry        iq_out,
    output  id_dp_t         id_dp_reg_next
);
            instr_pkt id_pkt;

            logic   [31:0] ir; assign ir = iq_out.inst.i;

            logic   [6:0]   opcode;
            logic           uses_rd, uses_rs1, uses_rs2, uses_imm;
            logic   [2:0]   funct3;
            logic   [6:0]   funct7;

            logic   [31:0]  i_imm;
            logic   [31:0]  s_imm;
            logic   [31:0]  b_imm;
            logic   [31:0]  u_imm;
            logic   [31:0]  j_imm;
            logic   [31:0]  imm;

            logic   [2:0]   aluop;
            logic   [2:0]   cmpop;
            logic   [2:0]   mulop;
            
            logic           regf_we;

            logic   [1:0]   res_station_id;
            logic   [2:0]   br_mul_div_i;

    // Decoding instruction
    assign funct3 = ir[14:12];
    assign funct7 = ir[31:25];
    assign opcode = ir[6:0];
    assign i_imm  = {{21{ir[31]}}, ir[30:20]};
    assign s_imm  = {{21{ir[31]}}, ir[30:25], ir[11:7]};
    assign b_imm  = {{20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0}; // shift-left preapplied
    assign u_imm  = {ir[31:12], 12'h000};
    assign j_imm  = {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0}; // shift-left preapplied

    always_comb begin : RESOURCES
        unique case (opcode)
            op_b_lui   : {uses_rd, uses_rs1, uses_rs2, uses_imm} = 4'b1001;
            op_b_auipc : {uses_rd, uses_rs1, uses_rs2, uses_imm} = 4'b1001;
            op_b_jal   : {uses_rd, uses_rs1, uses_rs2, uses_imm} = 4'b1001;
            op_b_jalr  : {uses_rd, uses_rs1, uses_rs2, uses_imm} = 4'b1101;
            op_b_br    : {uses_rd, uses_rs1, uses_rs2, uses_imm} = 4'b0111;
            op_b_load  : {uses_rd, uses_rs1, uses_rs2, uses_imm} = 4'b1101;
            op_b_store : {uses_rd, uses_rs1, uses_rs2, uses_imm} = 4'b0111;
            op_b_imm   : {uses_rd, uses_rs1, uses_rs2, uses_imm} = 4'b1101;
            op_b_reg   : {uses_rd, uses_rs1, uses_rs2, uses_imm} = 4'b1110;
            default    : {uses_rd, uses_rs1, uses_rs2, uses_imm} = '0;
        endcase
    end

    always_comb begin : IMMEDIATE
        unique case (opcode)
            op_b_lui  : imm = u_imm;
            op_b_auipc: imm = u_imm;
            op_b_jal  : imm = j_imm;
            op_b_jalr : imm = i_imm;
            op_b_br   : imm = b_imm;
            op_b_load : imm = i_imm;
            op_b_store: imm = s_imm;
            op_b_imm  : imm = i_imm;
            default   : imm = 'x;
        endcase
    end

    always_comb begin : ALU_OP
        aluop = 'x;
        
        if (opcode == op_b_imm) begin
            if (funct3 == arith_f3_sr)  aluop = funct7[5] ? alu_op_sra:alu_op_srl;
            else                        aluop = funct3;
        end

        if (opcode == op_b_reg) begin
            unique case (funct3)
                arith_f3_sr:  aluop = funct7[5] ? alu_op_sra:alu_op_srl;
                arith_f3_add: aluop = funct7[5] ? alu_op_sub:alu_op_add;                   
                default:      aluop = funct3;
            endcase
        end
    end

    always_comb begin : CMP_OP
        cmpop = 'x;

        if (opcode == op_b_br)
            cmpop = funct3;
        if ((opcode == op_b_imm) || (opcode == op_b_reg)) begin
            case (funct3)
                arith_f3_slt:   cmpop = branch_f3_blt;
                arith_f3_sltu:  cmpop = branch_f3_bltu;
            endcase
        end
    end

    always_comb begin : RESERVATION_STATION_ID
        unique case (opcode)
            op_b_lui, op_b_auipc, op_b_imm  : res_station_id = alu_res;
            op_b_jal, op_b_jalr, op_b_br    : res_station_id = br_mul_div_res;
            op_b_load, op_b_store           : res_station_id = mem_res;
            op_b_reg                        : res_station_id = funct7[0] ? br_mul_div_res:alu_res;
            default                         : res_station_id = privilege_res;   // all other instructions must be FENCE*, ECALL, EBREAK, and CSRR
        endcase
    end

    assign mulop = funct7[0] ? funct3:'x;

    always_comb begin : REGFILE_WEA
        regf_we = 1'b0;

        if ((opcode == op_b_lui)
        ||  (opcode == op_b_auipc)
        ||  (opcode == op_b_jal)
        ||  (opcode == op_b_jalr)
        ||  (opcode == op_b_load)
        ||  (opcode == op_b_imm)
        ||  (opcode == op_b_reg))
            regf_we = 1'b1;
    end

    always_comb begin : DIFFERENTIATE_BR_MUL_DIV
        unique case (opcode)
            op_b_jal, op_b_jalr, op_b_br : br_mul_div_i = 3'b100;
            op_b_reg                     : br_mul_div_i = (funct3 <= 3'h3) ? 3'b010:3'b001;
            default                      : br_mul_div_i = 'x;
        endcase 
    end

    always_comb begin
        id_pkt = '{
            i         : ir,
            i_order   : iq_out.inst.i_order,
            i_valid   : iq_out.inst.i_valid && (ir != 0) && (res_station_id != privilege_res),
            i_pc      : iq_out.inst.i_pc,
            i_pc_next : iq_out.inst.i_pc_next,

            i_rob_idx : 'x,
            rob_ready : 'x,
            res_id    : res_station_id,
            
            i_opcode  : opcode,
            i_uses_rd : uses_rd,
            i_uses_rs1: uses_rs1,
            i_uses_rs2: uses_rs2,
            i_uses_imm: uses_imm,
            i_funct3  : funct3,
            i_funct7  : funct7,
            br_mul_div: br_mul_div_i,
            
            rd_addr   : uses_rd ? ir[11:7] : '0,
            rd_paddr  : 'x,
            rd_data   : 'x,
            rs1_addr  : uses_rs1 ? ir[19:15] : '0,
            rs1_paddr : 'x,
            rs1_data  : 'x,
            rs2_addr  : uses_rs2 ? ir[24:20] : '0,
            rs2_paddr : 'x,
            rs2_data  : 'x,
            imm_data  : imm,
            
            i_dmem_addr : 'x,
            i_dmem_rmask: 'x,
            i_dmem_wmask: 'x,
            i_dmem_rdata: 'x,
            i_dmem_wdata: 'x,
            
            alu_op  : aluop,
            cmp_op  : cmpop,
            mul_op  : mulop,
            
            br_pred       : iq_out.inst.br_pred,
            br_result     : '0,
            br_commit_ghr : iq_out.inst.br_commit_ghr, 
            br_target     : 'x,
            
            regfile_we: regf_we
        };

        id_dp_reg_next = '{
            inst : id_pkt
        };
    end

endmodule : decode