// This class generates random valid RISC-V instructions to test your
// RISC-V cores.

class RandInst;

    localparam NUM_TYPES = 4;

    rand instr_t instr;
    rand bit [NUM_TYPES-1:0] instr_type;

    // Make sure we have an even distribution of instruction types.
    constraint solve_order_c { solve instr_type before funct3; }

    // to get 100% coverage with 500 calls to .randomize().
    rand logic [2:0] funct3;
    rand logic [6:0] funct7;

    constraint solve_order_funct3_c { 
        solve funct3 before funct7;
    }

    constraint funct3_funct7 {
        instr.r_type.funct3 == funct3;
        instr.r_type.funct7 == funct7;
    }

    // Pick one of the instruction types.
    constraint instr_type_c {
        $countones(instr_type) == 1; // Ensures one-hot.
    }

    // Constraints for actually generating instructions, given the type.
    constraint instr_c {
        // Reg-imm instructions
        instr_type[0] -> {
            instr.i_type.opcode == op_b_imm;

            // Implies syntax: if funct3 is arith_f3_sr, then funct7 must be
            // one of two possibilities.
            instr.i_type.funct3 == arith_f3_sr -> {
                // Use r_type here to be able to constrain funct7.
                instr.r_type.funct7 inside {base, variant};
            }

            // This if syntax is equivalent to the implies syntax above
            // but also supports an else { ... } clause.
            if (instr.i_type.funct3 == arith_f3_sll) {
                instr.r_type.funct7 == base;
            }
        }

        // Reg-reg instructions
        instr_type[1] -> {
            instr.r_type.opcode == op_b_reg;

            if (instr.r_type.funct7 == base) {
                instr.r_type.funct3 inside {arith_f3_add, arith_f3_sll, arith_f3_slt, arith_f3_sltu, arith_f3_xor, arith_f3_sr, arith_f3_or, arith_f3_and};
            } 
            else if (instr.r_type.funct7 == m_ext) {
                instr.r_type.funct3 inside {m_op_mul, m_op_mulh, m_op_mulhsu, m_op_mulhu, m_op_div, m_op_divu, m_op_rem, m_op_remu};
            }
            else {
                instr.r_type.funct3 inside {arith_f3_add, arith_f3_sr};
            }
        }

        // // Store instructions
        // instr_type[2] -> {
        //     instr.s_type.opcode == op_b_store;
        //     instr.s_type.funct3 inside {store_f3_sb, store_f3_sh, store_f3_sw};
        // }

        // // Load instructions
        // instr_type[3] -> {
        //     instr.i_type.opcode == op_b_load;
        //     instr.i_type.funct3 inside {load_f3_lb, load_f3_lh, load_f3_lw, load_f3_lbu, load_f3_lhu};
        // }

        // lui
        instr_type[2] -> {
            instr.j_type.opcode == op_b_lui;
        }

        // auipc
        instr_type[3] -> {
            instr.j_type.opcode == op_b_auipc;
        }

        // // jal
        // instr_type[6] -> {
        //     instr.j_type.opcode == op_b_jal;
        // }

        // // jalr
        // instr_type[7] -> {
        //     instr.i_type.opcode == op_b_jalr;
        //     instr.i_type.funct3 == 3'b000;
        // }

        // // branch
        // instr_type[8] -> {
        //     instr.b_type.opcode == op_b_br;
        //     instr.b_type.funct3 inside {branch_f3_beq, branch_f3_bne, branch_f3_blt, branch_f3_bge, branch_f3_bltu, branch_f3_bgeu};
        // }
    }

    `include "instr_cg.svh"

    // Constructor, make sure we construct the covergroup.
    function new();
        instr_cg = new();
    endfunction : new

    // Whenever randomize() is called, sample the covergroup. This assumes
    // that every generated random instruction are send it into the CPU.
    function void post_randomize();
        instr_cg.sample(this.instr);
    endfunction : post_randomize

    // A nice part of writing constraints is that we get constraint checking
    // for free -- this function will check if a bitvector is a valid RISC-V
    // instruction (assuming you have written all the relevant constraints).
    function bit verify_valid_instr(instr_t inp);
        bit valid = 1'b0;
        this.instr = inp;
        for (int i = 0; i < NUM_TYPES; ++i) begin
            this.instr_type = NUM_TYPES'(1 << i);
            if (this.randomize(null)) begin
                valid = 1'b1;
                break;
            end
        end
        return valid;
    endfunction : verify_valid_instr

endclass : RandInst