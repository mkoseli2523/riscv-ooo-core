module dispatch 
import rv32i_types::*;
(   
    // instruction coming from decode
    input   id_dp_t                 id_dp_reg,

    // register mappings to/from RAT
    input   logic [PHYS_WIDTH-1:0]  rs1_mapping,
    input   logic [PHYS_WIDTH-1:0]  rs2_mapping,
    output  logic [ARCH_WIDTH-1:0]  rs1_archreg,
    output  logic [ARCH_WIDTH-1:0]  rs2_archreg,
    output  logic                   rename_en, // control signal
    output  logic [ARCH_WIDTH-1:0]  rat_arch_dst, // arch register being renamed
    output  logic [PHYS_WIDTH-1:0]  rat_phys_dst, // also goes into valid array

    // signals to/from free list
    input   logic                   fl_empty,
    input   logic [PHYS_WIDTH-1:0]  preg_addr,
    output  logic                   preg_request, // control signal

    // ROB signals
    input   logic [ROB_ADDR_WIDTH-1:0] rob_idx,
    input   logic                      rob_full,
    output  instr_pkt                  rob_pkt, 
    
    // signals coming from RS
    input   logic                   res_full[NUM_RES_STATIONS],
    output  res_entry               res_entry_out,

    output  logic                   stall_dispatch
);

    /**

        INTERACTING with RAT:
            - send source operands to RAT, get the mappings same cycle (RAT is combinational read)
            - also send the new destination reg mapping to RAT in order to update the mapping

        INTERACTING with ROB:
            - if ROB is not full:
                - send instruction packet to ROB
                - ROB will return an index immediately

        INTERACTING with free-list:
            - if free-list isn't empty
                - send request to free-list
                - free-list will return a free-reg same cycle

        INTERACTING with valid-array:
            - send new mapping of arch reg to valid-array
                - valid-array should invalidate the given mapping
                - phys_reg signal goes to both RAT and valid-array

        RENAME:
            - For passed in instruction:
                - get rs1/rs2 mappings from RAT
                - if instruction uses rd, get new reg from free-list
                    - invalidate this phys reg
                    - send new mapping to RAT
                - get idx from ROB
                - select which res station to send the instruction to

        WRITING to RS:
            - fill res_entry_out struct
            - check opcode of inst to output and set res_station_index
            - check if res_full fo res station to insert
                - if full, stall
                - if not full, send res_entry_out to res station

    */

    assign stall_dispatch = id_dp_reg.inst.i_valid && (res_full[id_dp_reg.inst.res_id] | rob_full | fl_empty);
    
    always_comb begin : RAT_REQUEST
        rs1_archreg = id_dp_reg.inst.rs1_addr;
        rs2_archreg = id_dp_reg.inst.rs2_addr;

        rat_arch_dst = id_dp_reg.inst.rd_addr;
        rat_phys_dst = preg_addr;   // if free-list empty then don't invalidate anything
        rename_en = (id_dp_reg.inst.i_valid && rat_arch_dst != '0 && !stall_dispatch) ? id_dp_reg.inst.i_uses_rd : '0;
    end

    always_comb begin : FREE_LIST_REQUEST
        preg_request = !stall_dispatch && id_dp_reg.inst.i_valid && id_dp_reg.inst.rd_addr != '0;
    end
    
    always_comb begin : UPDATE_ROB_PKT
        rob_pkt = id_dp_reg.inst;
        rob_pkt.i_valid = id_dp_reg.inst.i_valid && !stall_dispatch;
        rob_pkt.rs1_paddr = rs1_mapping;
        rob_pkt.rs2_paddr = rs2_mapping;
        rob_pkt.rd_paddr  = preg_addr;

        rob_pkt.i_rob_idx = rob_idx;

        res_entry_out = '{
            inst    : rob_pkt, 
            valid   : 1'b0,
            stall   : 1'b0
        };
    end

endmodule : dispatch