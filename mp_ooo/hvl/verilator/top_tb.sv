module top_tb
(
    input   logic   clk,
    input   logic   rst,
    output  logic   dump_on
);

    // initial begin
    //     $dumpfile("dump.fst");
    //     if ($test$plusargs("NO_DUMP_ALL_ECE411")) begin
    //         $dumpvars(0, dut);
    //         $dumpoff();
    //     end else begin
    //         $dumpvars();
    //     end
    // end

    assign dump_on = monitor.dump_on;

    `include "top_tb.svh"

endmodule
