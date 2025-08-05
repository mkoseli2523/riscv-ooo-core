module sp_ff_array #(
            parameter               S_INDEX     = 4,
            parameter               WIDTH       = 1
)(
    input   logic                   clk0,
    input   logic                   rst0,
    input   logic                   csb0,
    input   logic                   web0,
    input   logic   [S_INDEX-1:0]   addr0,
    input   logic   [WIDTH-1:0]     din0,
    output  logic   [WIDTH-1:0]     dout0
);

            localparam              NUM_SETS    = 2**S_INDEX;

            logic                   web0_reg;
            logic   [S_INDEX-1:0]   addr0_reg;
            logic   [WIDTH-1:0]     din0_reg;

            logic   [WIDTH-1:0]     internal_array [NUM_SETS];

    always_ff @(posedge clk0) begin
        if (rst0) begin
            web0_reg  <= 1'b1;
            addr0_reg <= 'x;
            din0_reg  <= 'x;
        end else begin
            if (!csb0) begin
                web0_reg  <= web0;
                addr0_reg <= addr0;
                din0_reg  <= din0;
            end
        end
    end

    always_ff @(posedge clk0) begin
        if (rst0) begin
            for (integer i = 0; i < NUM_SETS; i++) begin
                internal_array[i] <= '0;
            end
        end else begin
            if (!web0_reg) begin
                internal_array[addr0_reg] <= din0_reg;
            end
        end
    end

    always_comb begin
        dout0 = internal_array[addr0_reg];
    end

endmodule : sp_ff_array
