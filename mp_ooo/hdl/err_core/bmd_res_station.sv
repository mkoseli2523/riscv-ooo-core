module bmd_res_station 
import rv32i_types::*;
#(
    parameter  DEPTH = 8,  // number of elements in res station
    localparam ADDR_WIDTH = $clog2(DEPTH),
    parameter  STALL_WIDTH = 3 // how many FUs the RS accomadates for
)
(
    input   logic                       clk,
    input   logic                       rst,

    // valid array inputs/outputs
    output  logic [PHYS_WIDTH-1:0]  rs1_out[DEPTH],
    output  logic [PHYS_WIDTH-1:0]  rs2_out[DEPTH],
    input   logic                   rs1_valid[DEPTH],
    input   logic                   rs2_valid[DEPTH],

    
    // inputs coming from dispatch
    input   logic                       enable,        // high - new inst to insert
    input   res_entry                   res_entry_in,  // inst from dispatch
    
    // stall signal coming from execute
    input   logic [STALL_WIDTH-1:0]     stall_fu,
    input   logic                       br_flush,

    // output signals
    output  logic                       res_full,      // stall prev stages if high
    output  instr_pkt                   instr_pkt_out,

    // read from PRF
    output  logic [PHYS_WIDTH-1:0]      rs1_paddr,
    output  logic [PHYS_WIDTH-1:0]      rs2_paddr
);
    
    /**
    
        Functionality of the collabsable queue:

            Insert at the lowest priority (ie level 0).

            Adding an instruction to the queue:
                - check if lowest priority level is taken
                - check if the lowest priority level is stalling
                - if not, insert to lowest priority

            Removing (issuing) an instruction from the queue:
                - for loop to go thru each instruction in the queue
                - starting from lowest (priority level 0) to highest priority level (DEPTH-1)
                - check if the FU is stalling AND rs1 && rs2 ready (if used)
                    - if true, set valid b!t of that entry to 0 
                    - NOTE: this needs to happen before collapsing mechanism due to way priority works

            Collapsing mechanism:
                - for each idx in queue if idx is not the selected-idx AND idx is valid collapse if
                    - next priority-level is not valid
                    - next priority-level is not stalling
                    - next priority-level is selected && fu is not stalling (ie selected_idx will get booted out)

            Inter-stalling mechanism:
                - if index is not selected AND index after is stalling (always true for highest priority)
                current index is also stalling
    
    */

    res_entry res_queue [DEPTH];

    assign res_full = res_queue[0].valid && res_queue[0].stall;

    generate 
        for (genvar i = 0; i < DEPTH; i++) begin
            assign rs1_out[i] = res_queue[i].inst.rs1_paddr;
            assign rs2_out[i] = res_queue[i].inst.rs2_paddr;
        end
    endgenerate

    logic [ADDR_WIDTH-1:0] select_idx;
    logic                  select_valid;
    logic                  out_stalling;

    always_comb begin : SELECT_IDX_LOGIC
        select_idx = ADDR_WIDTH'(0);
        select_valid = 1'b0;

        for (integer unsigned i = 0; i < DEPTH; i++) begin
            // choose oldest ready inst

            // stall_fu one hot incoded for bmd res station/fu
            // stall_fu = {br,mul,div}
            
            if (!out_stalling && rs1_valid[i] && rs2_valid[i] && res_queue[i].valid
                && !stall_fu[res_queue[i].inst.br_mul_div[2:1]]) begin
                
                select_idx = ADDR_WIDTH'(unsigned'(i));
                select_valid = 1'b1;
                
                // unique case (res_queue[i].inst.br_mul_div)
                    // 3'b100 : begin  // branch instruction
                        // if (!stall_fu[2]) begin
                            // select_idx = ADDR_WIDTH'(unsigned'(i));
                            // select_valid = 1'b1;
                        // end
                    // end
                    // 3'b010 : begin  // mul stalling
                        // if (!stall_fu[1]) begin
                            // select_idx = ADDR_WIDTH'(unsigned'(i));
                            // select_valid = 1'b1;
                        // end
                    // end
                    // 3'b001 : begin  // divide instruction
                        // if (!stall_fu[0]) begin
                            // select_idx = ADDR_WIDTH'(unsigned'(i));
                            // select_valid = 1'b1;
                        // end
                    // end
                    // default: begin
                        // select_idx = 'x;
                        // select_valid = 1'b0;
                    // end
                // endcase
            end
        end
    end

    always_comb begin : INTER_STALL_LOGIC
        for (integer i = DEPTH-1; i > -1; i--) begin
            if (res_queue[i].valid) begin
                if (select_valid && select_idx == ADDR_WIDTH'(unsigned'(i))) begin
                    res_queue[i].stall = 1'b0; // check if the index will get booted out
                end else if (i == DEPTH-1) begin
                    res_queue[i].stall = 1'b1; // highest priority and not selected
                end else if (res_queue[i + 1].stall) begin
                    res_queue[i].stall = 1'b1; // if the next stage stalls; stall all prior insts
                end else begin
                    res_queue[i].stall = 1'b0; // else don't stall
                end
            end else begin
                res_queue[i].stall = 1'b0;
            end
        end
    end

    always_ff @ (posedge clk) begin : COLLAPSING_QUEUE
        if (rst | br_flush) begin
            for (integer i = 0; i < DEPTH; i++) begin
                res_queue[i].valid <= 1'b0;
            end
        end else begin
            // invalidanting an entry upon selecting
            if (select_valid) begin : INVALIDATE
                res_queue[select_idx].valid <= 1'b0;
            end 

            for (integer i = DEPTH-2; i > -1; i--) begin : COLLAPSE
                if (!res_queue[i+1].stall && res_queue[i].valid
                 && !(ADDR_WIDTH'(unsigned'(i)) == select_idx && select_valid)) begin
                    res_queue[i+1].inst <= res_queue[i].inst;
                    res_queue[i+1].valid <= res_queue[i].valid;
                    res_queue[i].valid <= '0;
                end
            end

            // add insert logic
            if (!res_queue[0].stall && enable) begin : INSERT
                res_queue[0].inst <= res_entry_in.inst;
                res_queue[0].valid <= 1'b1;
            end
        end
    end
    
    assign out_stalling = instr_pkt_out.i_valid && stall_fu[instr_pkt_out.br_mul_div >> 1];

    always_ff @(posedge clk) begin : LATCH_OUTPUT
        if (rst | br_flush) begin
            instr_pkt_out.i_valid <= '0;
        end else if (select_valid) begin
            instr_pkt_out <= res_queue[select_idx].inst;
        end else if (!out_stalling) begin
            instr_pkt_out.i_valid <= '0;
        end
    end

    assign rs1_paddr = out_stalling ? instr_pkt_out.rs1_paddr : res_queue[select_idx].inst.rs1_paddr;
    assign rs2_paddr = out_stalling ? instr_pkt_out.rs2_paddr : res_queue[select_idx].inst.rs2_paddr;

endmodule : bmd_res_station