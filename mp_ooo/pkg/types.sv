package rv32i_types;

    localparam PROCESSOR_WIDTH = 1;
    localparam pc_rst         = 32'hAAAAA000;

    localparam PHYS_REGS  = 64;
    localparam ARCH_REGS  = 32;
    localparam PHYS_WIDTH = $clog2(PHYS_REGS);
    localparam ARCH_WIDTH = $clog2(ARCH_REGS);

    localparam ROB_DEPTH      = 16;
    localparam ROB_ADDR_WIDTH = $clog2(ROB_DEPTH);

    localparam NUM_LINES = 2;

    localparam NUM_SB = 4;
    localparam SB_DEPTH = 4;
    localparam SB_ADDR_WIDTH = $clog2(SB_DEPTH);

    localparam LSQ_DEPTH      = 8;
    localparam LSQ_ADDR_WIDTH = $clog2(LSQ_DEPTH);

    localparam FL_DEPTH      = PHYS_REGS-ARCH_REGS;
    localparam FL_ADDR_WIDTH = $clog2(FL_DEPTH);

    localparam NUM_RES_STATIONS = 3;

    localparam NUM_BMD_RS_ENTRIES = 8;
    localparam NUM_ALU_RS_ENTRIES = 8;
    localparam NUM_MEM_RS_ENTRIES = LSQ_DEPTH;
    localparam NUM_RES_ENTRIES    = NUM_ALU_RS_ENTRIES + NUM_BMD_RS_ENTRIES + NUM_MEM_RS_ENTRIES;

    localparam CDB_PORTS        = 3;
    
    typedef enum logic [1:0] {
        br_mul_div_res = 2'd0,
        alu_res        = 2'd1,
        mem_res        = 2'd2,
        privilege_res  = 2'd3
    } res_station_id;

    typedef enum logic [6:0] {
        op_b_lui       = 7'b0110111, // load upper immediate (U type)
        op_b_auipc     = 7'b0010111, // add upper immediate PC (U type)
        op_b_jal       = 7'b1101111, // jump and link (J type)
        op_b_jalr      = 7'b1100111, // jump and link register (I type)
        op_b_br        = 7'b1100011, // branch (B type)
        op_b_load      = 7'b0000011, // load (I type)
        op_b_store     = 7'b0100011, // store (S type)
        op_b_imm       = 7'b0010011, // arith ops with register/immediate operands (I type)
        op_b_reg       = 7'b0110011  // arith ops with register operands (R type)
    } rv32i_opcode;

    typedef enum logic [2:0] {
        arith_f3_add   = 3'b000, // check logic 30 for sub if op_reg op
        arith_f3_sll   = 3'b001,
        arith_f3_slt   = 3'b010,
        arith_f3_sltu  = 3'b011,
        arith_f3_xor   = 3'b100,
        arith_f3_sr    = 3'b101, // check logic 30 for logical/arithmetic
        arith_f3_or    = 3'b110,
        arith_f3_and   = 3'b111
    } arith_f3_t;

    typedef enum logic [2:0] {
        load_f3_lb     = 3'b000,
        load_f3_lh     = 3'b001,
        load_f3_lw     = 3'b010,
        load_f3_lbu    = 3'b100,
        load_f3_lhu    = 3'b101
    } load_f3_t;

    typedef enum logic [2:0] {
        store_f3_sb    = 3'b000,
        store_f3_sh    = 3'b001,
        store_f3_sw    = 3'b010
    } store_f3_t;

    typedef enum logic [2:0] {
        branch_f3_beq  = 3'b000,
        branch_f3_bne  = 3'b001,
        branch_f3_blt  = 3'b100,
        branch_f3_bge  = 3'b101,
        branch_f3_bltu = 3'b110,
        branch_f3_bgeu = 3'b111
    } branch_f3_t;

    typedef enum logic [2:0] {
        alu_op_add     = 3'b000,
        alu_op_sll     = 3'b001,
        alu_op_sra     = 3'b010,
        alu_op_sub     = 3'b011,
        alu_op_xor     = 3'b100,
        alu_op_srl     = 3'b101,
        alu_op_or      = 3'b110,
        alu_op_and     = 3'b111
    } alu_ops;
    
    typedef enum logic [2:0] {
        m_op_mul      = 3'b000,
        m_op_mulh     = 3'b001,
        m_op_mulhsu   = 3'b010,
        m_op_mulhu    = 3'b011,
        m_op_div      = 3'b100,
        m_op_divu     = 3'b101,
        m_op_rem      = 3'b110,
        m_op_remu     = 3'b111
    } m_ops;

    typedef enum logic [2:0] {
        jalr_f3 = 3'b000
    } jalr_f3_t;

    typedef enum logic [6:0] {
        base           = 7'b0000000,
        variant        = 7'b0100000,
        m_ext          = 7'b0000001
    } funct7_t;

    // Various ways RISC-V instruction words can be interpreted.
    typedef union packed {
        logic [31:0] word;

        struct packed {
            logic [11:0] i_imm;
            logic [4:0]  rs1;
            logic [2:0]  funct3;
            logic [4:0]  rd;
            rv32i_opcode opcode;
        } i_type;

        struct packed {
            logic [6:0]  funct7;
            logic [4:0]  rs2;
            logic [4:0]  rs1;
            logic [2:0]  funct3;
            logic [4:0]  rd;
            rv32i_opcode opcode;
        } r_type;

        struct packed {
            logic [11:5] imm_s_top;
            logic [4:0]  rs2;
            logic [4:0]  rs1;
            logic [2:0]  funct3;
            logic [4:0]  imm_s_bot;
            rv32i_opcode opcode;
        } s_type;

        struct packed {
            logic        imm_12;
            logic [10:5] imm_10_5;
            logic [4:0]  rs2;
            logic [4:0]  rs1;
            logic [2:0]  funct3;
            logic [4:1]  imm_4_1;
            logic        imm_11;
            rv32i_opcode opcode;
        } b_type;

        struct packed {
            logic [31:12] imm;
            logic [4:0]   rd;
            rv32i_opcode  opcode;
        } j_type;
    } instr_t;

    // ~~~~~~~~~~~~ CONTROL SIGNALS ~~~~~~~~~~~~
    typedef enum logic {
        PC_increment = 1'b0,
        PC_br = 1'b1
    } pc_m1_sel_t;

    // ~~~~~~~~~~~~ INSTRUCTION METADATA ~~~~~~~~~~~~
    typedef struct packed {
        logic [31:0]               i;
        logic [63:0]               i_order;
        logic                      i_valid;
        logic [31:0]               i_pc;
        logic [31:0]               i_pc_next;

        logic [ROB_ADDR_WIDTH-1:0] i_rob_idx;
        logic                      rob_ready;
        logic [1:0]                res_id;

        logic [6:0]  i_opcode;
        logic        i_uses_rd;
        logic        i_uses_rs1;
        logic        i_uses_rs2;
        logic        i_uses_imm;
        logic [2:0]  i_funct3;
        logic [6:0]  i_funct7;
        logic [2:0]  br_mul_div;

        logic [4:0]             rd_addr;
        logic [PHYS_WIDTH-1:0]  rd_paddr;
        logic [31:0]            rd_data;
        logic [4:0]             rs1_addr;
        logic [PHYS_WIDTH-1:0]  rs1_paddr;
        logic [31:0]            rs1_data;
        logic [4:0]             rs2_addr;
        logic [PHYS_WIDTH-1:0]  rs2_paddr;
        logic [31:0]            rs2_data;
        logic [31:0]            imm_data;

        logic [31:0] i_dmem_addr;
        logic [3:0]  i_dmem_rmask;
        logic [3:0]  i_dmem_wmask;
        logic [31:0] i_dmem_rdata;
        logic [31:0] i_dmem_wdata;

        logic [2:0]  alu_op;
        logic [2:0]  cmp_op;
        logic [2:0]  mul_op;

        logic        br_pred;
        logic        br_result;
        logic [9:0]  br_commit_ghr;
        logic [31:0] br_target;

        logic        regfile_we;
    } instr_pkt;

    // ~~~~~~~~~~~~ PIPELINE REG STRUCTURES ~~~~~~~~~~~~
    typedef struct packed {
        instr_pkt    inst;
    } iq_entry;

    typedef struct packed {
        instr_pkt    inst;
    } id_dp_t;

    // assume that both queues are the same depth
    // parameter AGE_COUNTER_WIDTH = $clog2(LSQ_DEPTH*2);

    typedef struct packed {
        instr_pkt    inst;
        
        logic        valid;
        logic        stall;
    } res_entry;

    typedef struct packed {
        instr_pkt    inst;
    } iss_ex_t;

    typedef struct packed {
        instr_pkt    inst;
    } cdb_t;

    localparam IQ_DEPTH       = 8;//TBDCP2
    localparam IQ_WIDTH       = $bits(iq_entry);

endpackage : rv32i_types

package cache_types;

    localparam NUM_ICACHE_SETS = 16;

    localparam ICACHE_SET_WIDTH = $clog2(NUM_ICACHE_SETS);

    localparam TAG_MSB    = 31;
    localparam TAG_LSB    = 9;
    
    localparam SET_MSB    = 8;
    localparam SET_LSB    = 5;
    
    localparam OFFSET_MSB = 4;
    localparam OFFSET_LSB = 0;

    localparam TAG_WIDTH    = TAG_MSB-TAG_LSB+1; // (in bits)
    localparam SET_WIDTH    = SET_MSB-SET_LSB+1; // index
    localparam OFFSET_WIDTH = OFFSET_MSB-OFFSET_LSB+1;
    localparam DATA_WIDTH   = 32*8;
    localparam DATA_MASK_WIDTH = 32;

    typedef struct packed {
        logic                  l_valid;
        logic                  l_dirty;
        logic [TAG_WIDTH-1:0]  l_tag;
        logic [DATA_WIDTH-1:0] l_data;
    } cache_line_t;

    typedef struct packed {
        logic         r_valid;

        logic [31:0]  addr;
        logic [3:0]   rmask;
        logic [3:0]   wmask;
        logic [31:0]  wdata;
    } cache_req;

endpackage : cache_types