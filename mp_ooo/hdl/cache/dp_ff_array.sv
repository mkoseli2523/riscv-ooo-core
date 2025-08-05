module dp_ff_array #(
            parameter               S_INDEX     = 4,
            parameter               WIDTH       = 1
)(
    input   logic                   clk0,
    input   logic                   rst0,

    input   logic                   csb0,
    input   logic                   web0,
    input   logic   [S_INDEX-1:0]   addr0,
    input   logic   [WIDTH-1:0]     din0,
    output  logic   [WIDTH-1:0]     dout0,

    input   logic                   csb1,
    input   logic                   web1,
    input   logic   [S_INDEX-1:0]   addr1,
    input   logic   [WIDTH-1:0]     din1,
    output  logic   [WIDTH-1:0]     dout1
);

            localparam              NUM_SETS    = 2**S_INDEX;

            logic                   csb0_reg;
            logic                   web0_reg;
            logic   [S_INDEX-1:0]   addr0_reg;
            logic   [WIDTH-1:0]     din0_reg;

            logic                   csb1_reg;
            logic                   web1_reg;
            logic   [S_INDEX-1:0]   addr1_reg;
            logic   [WIDTH-1:0]     din1_reg;

            logic   [WIDTH-1:0]     internal_array [NUM_SETS];

    always_ff @(posedge clk0) begin
        if (rst0) begin
            web0_reg  <= 1'b1;
            addr0_reg <= 'x;
            din0_reg  <= 'x;
            web1_reg  <= 1'b1;
            addr1_reg <= 'x;
            din1_reg  <= 'x;
            {csb0_reg, csb1_reg} <= '1;
        end else begin
            if (!csb0) begin
                web0_reg  <= web0;
                addr0_reg <= addr0;
                din0_reg  <= din0;
            end
            if (!csb1) begin
                web1_reg  <= web1;
                addr1_reg <= addr1;
                din1_reg  <= din1;
            end

            {csb0_reg, csb1_reg} <= {csb0, csb1};
        end
    end

    always_ff @(posedge clk0) begin
        if (rst0) begin
            for (integer i = 0; i < NUM_SETS; i++) begin
                internal_array[i] <= '0;
            end
        end else begin
            case ({web1_reg, web0_reg})
                2'b00: begin
                    if (addr0_reg == addr1_reg) begin
                        internal_array[addr0_reg] <= 'x;
                    end else begin
                        internal_array[addr1_reg] <= din1_reg;
                        internal_array[addr0_reg] <= din0_reg;
                    end
                end
                2'b01: internal_array[addr1_reg] <= din1_reg;
                2'b10: internal_array[addr0_reg] <= din0_reg;
            endcase
        end
    end

    always_comb begin//transparent read on dout0
        dout0 = (~csb0_reg && ~web1_reg && (addr1_reg == addr0_reg)) ? din1_reg : internal_array[addr0_reg];
        dout1 = internal_array[addr1_reg];
    end

endmodule : dp_ff_array
