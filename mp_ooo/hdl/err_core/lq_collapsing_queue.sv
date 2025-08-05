module lq_collapsing_queue 
import rv32i_types::*;
#(
    parameter  DEPTH = LSQ_DEPTH,  // number of elements in res station
    localparam ADDR_WIDTH = $clog2(DEPTH),
    parameter  STALL_WIDTH = 1 // how many FUs the RS accomadates for
)
(
    input   logic                       clk,
    input   logic                       rst,

    // valid array inputs/outputs
    output  logic [PHYS_WIDTH-1:0]  rs1_out[DEPTH],
    // output  logic [PHYS_WIDTH-1:0]  rs2_out[DEPTH],
    input   logic                   rs1_valid[DEPTH],
    // input   logic                   rs2_valid[DEPTH],

    
    // inputs coming from dispatch
    input   logic                       enable,        // high - new inst to insert
    input   res_entry                   res_entry_in,  // inst from dispatch
    
    // stall signal coming from execute
    input   logic [STALL_WIDTH-1:0]     stall_fu,
    input   logic                       br_flush,

    // output signals
    output  logic                       res_full,      // stall prev stages if high
    output  logic                       res_empty,
    output  instr_pkt                   instr_pkt_out,

    // read from PRF
    output  logic [PHYS_WIDTH-1:0]      rs1_paddr,
    // output  logic [PHYS_WIDTH-1:0]      rs2_paddr,

    input   logic                       store_selected,
    input   logic                       store_inserted,
    input   logic [ADDR_WIDTH-1:0]      store_selected_index,
    input   logic [ADDR_WIDTH-1:0]      store_inserted_index,
    input   logic [DEPTH-1:0]           store_valid
);

    res_entry queue [DEPTH];

    generate 
        for (genvar i = 0; i < DEPTH; i++) begin
            assign rs1_out[i] = queue[i].inst.rs1_paddr;
            // assign rs2_out[i] = res_queue[i].inst.rs2_paddr;
            // assign lq_age[i]  = res_queue[i].inst.age;
        end
    endgenerate

    logic [ADDR_WIDTH:0] head, head_next;
    logic [ADDR_WIDTH:0] tail;

    logic [DEPTH-1:0] valid_matrix[DEPTH], valid_matrix_next[DEPTH];

    logic issue;
    logic [ADDR_WIDTH-1:0] issue_index;
    logic can_issue[DEPTH];

    always_ff @(posedge clk) begin
        if (rst | br_flush) begin
            head <= '0;
            tail <= '0;

            for (integer i = 0; i < LSQ_DEPTH; i++) begin
                queue[i].valid <= '0;
                valid_matrix[i] <= '0;
                queue[i].stall <= 'x;
            end
        end
        else begin
            valid_matrix <= valid_matrix_next;

            if (!res_empty && issue) begin
                queue[issue_index].valid   <= 1'b0;
            
                head <= head_next;
            end

            if (!res_full && enable) begin
                queue[tail[ADDR_WIDTH-1:0]].inst  <= res_entry_in.inst;
                queue[tail[ADDR_WIDTH-1:0]].valid <= 1'b1;
                
                tail <= tail + 1'b1;
            end
        end
    end

    generate
        for (genvar i = 0; i < DEPTH; i++) begin
            assign can_issue[i] = !stall_fu && rs1_valid[i] && queue[i].valid && !store_selected && valid_matrix[i] == '1;
        end
    endgenerate

    always_comb begin : VALID_MATRIX
        valid_matrix_next = valid_matrix;

        if (issue) begin
            valid_matrix_next[issue_index] = '0;
        end

        if (enable) begin
            for (integer i = 0; i < DEPTH; i++) begin
                if (store_valid[i]) begin
                    valid_matrix_next[tail[ADDR_WIDTH-1:0]][i] = 1'b0;
                end 
                else begin
                    valid_matrix_next[tail[ADDR_WIDTH-1:0]][i] = 1'b1;
                end
            end
        end

        if (store_selected) begin
            for (integer i = 0; i < DEPTH; i++) begin
                valid_matrix_next[i][store_selected_index] = 1'b1;
            end
        end

        if (store_inserted) begin
            for (integer i = 0; i < DEPTH; i++) begin
                valid_matrix_next[i][store_inserted_index] = 1'b1;
            end
        end
    end

    always_comb begin : HEAD_NEXT
        head_next = head;

        // if (issue && issue_index == head[ADDR_WIDTH-1:0]) begin
        //     head_next = tail;

        //     for (integer i = DEPTH-1; i > 0; i--) begin
        //         if (queue[head[ADDR_WIDTH-1:0] + ADDR_WIDTH'(i)].valid) begin
        //             head_next = head + ADDR_WIDTH'(i);
        //         end
        //     end
        // end

        head_next = head + 1'b1;
    end

    always_comb begin : ISSUE
        issue = '0;
        issue_index = 'x;

        // for (integer i = DEPTH-1; i > -1; i--) begin
        //     if (can_issue[i]) begin
        //         issue = 1'b1;
        //         issue_index = ADDR_WIDTH'(i);
        //     end
        // end

        if (can_issue[head[ADDR_WIDTH-1:0]]) begin
            issue = 1'b1;
            issue_index = head[ADDR_WIDTH-1:0];
        end
    end

    always_comb begin : ASSIGN_DOUT
        instr_pkt_out = 'x;

        if (issue) begin
            instr_pkt_out = queue[issue_index].inst;
        end else begin
            instr_pkt_out.i_valid = '0;
        end
    end

    assign res_empty = (head == tail && !res_full);
    assign res_full  = (head[ADDR_WIDTH-1:0] == tail[ADDR_WIDTH-1:0]) && (head[ADDR_WIDTH] != tail[ADDR_WIDTH]);

    assign rs1_paddr = queue[issue_index].inst.rs1_paddr;
























































    // /**
    
    //     Functionality of the collabsable queue:

    //         Insert at the lowest priority (ie level 0).

    //         Adding an instruction to the queue:
    //             - check if lowest priority level is taken
    //             - check if the lowest priority level is stalling
    //             - if not, insert to lowest priority

    //         Removing (issuing) an instruction from the queue:
    //             - for loop to go thru each instruction in the queue
    //             - starting from lowest (priority level 0) to highest priority level (DEPTH-1)
    //             - check if the FU is stalling AND rs1 && rs2 ready (if used)
    //                 - if true, set valid b!t of that entry to 0 
    //                 - NOTE: this needs to happen before collapsing mechanism due to way priority works

    //         Collapsing mechanism:
    //             - for each idx in queue if idx is not the selected-idx AND idx is valid collapse if
    //                 - next priority-level is not valid
    //                 - next priority-level is not stalling
    //                 - next priority-level is selected && fu is not stalling (ie selected_idx will get booted out)

    //         Inter-stalling mechanism:
    //             - if index is not selected AND index after is stalling (always true for highest priority)
    //             current index is also stalling
    
    // */

    // res_entry res_queue [DEPTH];

    // assign res_full = res_queue[0].valid && res_queue[0].stall;

    // generate 
    //     for (genvar i = 0; i < DEPTH; i++) begin
    //         assign rs1_out[i] = res_queue[i].inst.rs1_paddr;
    //         // assign rs2_out[i] = res_queue[i].inst.rs2_paddr;
    //         // assign lq_age[i]  = res_queue[i].inst.age;
    //     end
    // endgenerate

    // logic [ADDR_WIDTH-1:0] select_idx;
    // logic                  select_valid;

    // always_comb begin : SELECT_IDX_LOGIC
    //     select_idx = ADDR_WIDTH'(0);
    //     select_valid = 1'b0;

    //     for (integer unsigned i = 0; i < DEPTH; i++) begin
    //         // choose oldest ready inst
    //         if (!stall_fu && rs1_valid[i] && res_queue[i].valid && res_queue[i].age - base_age < sq_age - base_age && !store_selected) begin
    //             select_idx = ADDR_WIDTH'(unsigned'(i));
    //             select_valid = 1'b1;
    //         end
    //     end
    // end

    // always_comb begin : OLDEST_LOGIC
    //     res_empty = 1'b1;
    //     lq_oldest_age = 'x;

    //     for (integer i = 0; i < DEPTH; i++) begin
    //         if (res_queue[i].valid) begin
    //             lq_oldest_age = res_queue[i].age;
    //             res_empty = 1'b0;
    //         end
    //     end
    // end

    // always_comb begin : INTER_STALL_LOGIC
    //     for (integer i = DEPTH-1; i > -1; i--) begin
    //         if (res_queue[i].valid) begin
    //             if (select_valid && select_idx == ADDR_WIDTH'(unsigned'(i))) begin
    //                 res_queue[i].stall = 1'b0; // check if the index will get booted out
    //             end else if (i == DEPTH-1) begin
    //                 res_queue[i].stall = 1'b1; // highest priority and not selected
    //             end else if (res_queue[i + 1].stall) begin
    //                 res_queue[i].stall = 1'b1; // if the next stage stalls; stall all prior insts
    //             end else begin
    //                 res_queue[i].stall = 1'b0; // else don't stall
    //             end
    //         end else begin
    //             res_queue[i].stall = 1'b0;
    //         end
    //     end
    // end

    // always_ff @ (posedge clk) begin : COLLAPSING_QUEUE
    //     if (rst | br_flush) begin
    //         for (integer i = 0; i < DEPTH; i++) begin
    //             res_queue[i].valid <= 1'b0;
    //             // queue[i].age       <= 2**AGE_COUNTER_WIDTH-1;
    //         end
    //     end else begin
    //         // invalidanting an entry upon selecting
    //         if (select_valid) begin : INVALIDATE
    //             res_queue[select_idx].valid <= 1'b0;
    //         end 

    //         for (integer i = DEPTH-2; i > -1; i--) begin : COLLAPSE
    //             if (!res_queue[i+1].stall && res_queue[i].valid
    //              && !(ADDR_WIDTH'(unsigned'(i)) == select_idx && select_valid)) begin
    //                 res_queue[i+1].inst <= res_queue[i].inst;
    //                 res_queue[i+1].valid <= res_queue[i].valid;
    //                 res_queue[i+1].age  <= res_queue[i].age;
    //                 res_queue[i].valid <= '0;
    //             end
    //         end

    //         // add insert logic
    //         if (!res_queue[0].stall && enable) begin : INSERT
    //             res_queue[0].inst <= res_entry_in.inst;
    //             res_queue[0].age  <= age;

    //             res_queue[0].valid <= 1'b1;
    //         end
    //     end
    // end

    // always_comb begin : ASSIGN_DOUT
    //     instr_pkt_out = 'x;

    //     if (select_valid) begin
    //         instr_pkt_out = res_queue[select_idx].inst;
    //     end else begin
    //         instr_pkt_out.i_valid = '0;
    //     end
    // end

    // // always_ff @(posedge clk) begin : LATCH_OUTPUT
    // //     if (rst | br_flush) begin
    // //         instr_pkt_out.i_valid <= '0;
    // //     end else if (select_valid) begin
    // //         instr_pkt_out <= res_queue[select_idx].inst;
    // //     end else begin
    // //         instr_pkt_out.i_valid <= '0;
    // //     end
    // // end

    // assign rs1_paddr = res_queue[select_idx].inst.rs1_paddr;
    // // assign rs2_paddr = res_queue[select_idx].inst.rs2_paddr;

endmodule : lq_collapsing_queue