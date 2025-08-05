module free_list
import rv32i_types::*;
#(
    localparam DEPTH = FL_DEPTH,  // number of elements in fifo
    localparam ADDR_WIDTH = $clog2(DEPTH)    // defines head/tail pointer widths
) (
    input  logic                   clk,
    input  logic                   rst,

    // from rrf
    input  logic                   enqueue,
    input  logic [PHYS_WIDTH-1:0]  preg_addr_in,

    input  logic                   br_flush, // branch mispredict
    input  logic [FL_ADDR_WIDTH:0] rollback_fl_head,

    input  logic                   preg_request,  // dequeue signal (from dispatch)
    output logic [PHYS_WIDTH-1:0]  preg_addr_out,
    output logic                   fl_full,
    output logic                   fl_empty
);

    // free list queue entries are numerical values corresponding to 
    // free physical register values
    logic [PHYS_WIDTH-1:0] queue [0:DEPTH-1];
    logic [ADDR_WIDTH:0]   head;
    logic [ADDR_WIDTH:0]   tail;

    /*** enq/deq logic ***/
    always_ff @ (posedge clk) begin : ENQUEUE_LOGIC
        if (rst) begin
            head <= '0;
            tail <= DEPTH[ADDR_WIDTH:0];

            // fill queue on rst
            for (integer unsigned i = 0; i < DEPTH; i++)
                queue[i] <= PHYS_WIDTH'(32+i);
        end else begin
            if (enqueue && !fl_full) begin
                queue[tail[ADDR_WIDTH-1:0]] <= preg_addr_in;  // % depth
                tail <= tail + 1'b1;
            end

            if (preg_request && !fl_empty) begin
                head <= head + 1'b1;
            end
            
            if (br_flush) begin
                head <= rollback_fl_head;
            end
        end
    end

    assign preg_addr_out = (preg_request && !fl_empty) ? queue[head[ADDR_WIDTH-1:0]] : '0;
    assign fl_empty = (head == tail && !fl_full);
    assign fl_full  = (head[ADDR_WIDTH-1:0] == tail[ADDR_WIDTH-1:0]) && (head[ADDR_WIDTH] != tail[ADDR_WIDTH]);

endmodule : free_list