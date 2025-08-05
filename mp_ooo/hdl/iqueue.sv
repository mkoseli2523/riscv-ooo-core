module iqueue 
import rv32i_types::*;
#(
    localparam ADDR_WIDTH = $clog2(IQ_DEPTH)  // defines head/tail pointer widths
) (
    input  logic                 clk,
    input  logic                 rst,

    input  logic                 enqueue,
    input  logic                 dequeue,
    input  iq_entry              din,   // element to store in iqueue

    output iq_entry              dout,
    output logic                 full,
    output logic                 empty,

    input logic                  stall_dispatch
);

    iq_entry queue [0:IQ_DEPTH-1];
    logic [ADDR_WIDTH:0] head;  // if depth == 8, addr_width == 3; [ADDR_WIDTH:0] == [3:0] == 15 bits. 
                                // only need lower 3 bits for indexing; msb is used for overflow detection
    logic [ADDR_WIDTH:0] tail;

    /*** enq/deq logic ***/
    always_ff @(posedge clk) begin
        if (rst) begin
            head <= '0;
            tail <= '0;
            dout <= '0;

            // clear queue
            for (integer i = 0; i < IQ_DEPTH; i++) begin
                queue[i] <= 'x;
            end
        end else begin
            if (enqueue && !full) begin
                queue[tail[ADDR_WIDTH-1:0]] <= din;  // % depth
                tail <= tail + 1'b1;
            end

            // Dealing with duplicated / Backpressure
            if (dout.inst.i_valid && !stall_dispatch) begin
                dout.inst.i_valid <= 1'b0;
            end

            if (dequeue && !empty) begin
                dout <= queue[head[ADDR_WIDTH-1:0]];
                head <= head + 1'b1;
            end
        end
    end

    assign empty = (head == tail && !full);
    assign full  = (head[ADDR_WIDTH-1:0] == tail[ADDR_WIDTH-1:0]) && (head[ADDR_WIDTH] != tail[ADDR_WIDTH]);

endmodule : iqueue