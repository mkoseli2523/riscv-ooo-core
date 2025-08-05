// OpenRAM SRAM model
// Words: 64
// Word size: 32
// Write size: 2

module gshare_counter_array(
`ifdef USE_POWER_PINS
    vdd,
    gnd,
`endif
// Port 0: RW
    clk0,csb0,web0,wmask0,addr0,din0,dout0,
// Port 1: RW
    clk1,csb1,web1,wmask1,addr1,din1,dout1
  );

  parameter NUM_WMASKS = 16 ;
  parameter DATA_WIDTH = 32 ;
  parameter ADDR_WIDTH = 6 ;
  parameter RAM_DEPTH = 1 << ADDR_WIDTH;

`ifdef USE_POWER_PINS
    inout vdd;
    inout gnd;
`endif
  input  clk0; // clock
  input   csb0; // active low chip select
  input  web0; // active low write control
  input [ADDR_WIDTH-1:0]  addr0;
  input [NUM_WMASKS-1:0]   wmask0; // write mask
  input [DATA_WIDTH-1:0]  din0;
  output [DATA_WIDTH-1:0] dout0;
  input  clk1; // clock
  input   csb1; // active low chip select
  input  web1; // active low write control
  input [ADDR_WIDTH-1:0]  addr1;
  input [NUM_WMASKS-1:0]   wmask1; // write mask
  input [DATA_WIDTH-1:0]  din1;
  output [DATA_WIDTH-1:0] dout1;

  reg [DATA_WIDTH-1:0]    mem [0:RAM_DEPTH-1];

  reg  web0_reg;
  initial web0_reg = 1'b1;
  reg [NUM_WMASKS-1:0]   wmask0_reg;
  reg [ADDR_WIDTH-1:0]  addr0_reg;
  reg [DATA_WIDTH-1:0]  din0_reg;
  reg [DATA_WIDTH-1:0]  dout0;

  always @(posedge clk0)
  begin
    if( !csb0 ) begin
      web0_reg <= web0;
      wmask0_reg <= wmask0;
      addr0_reg <= addr0;
      din0_reg <= din0;
    end
  end

  reg  web1_reg;
  initial web1_reg = 1'b1;
  reg [NUM_WMASKS-1:0]   wmask1_reg;
  reg [ADDR_WIDTH-1:0]  addr1_reg;
  reg [DATA_WIDTH-1:0]  din1_reg;
  reg [DATA_WIDTH-1:0]  dout1;

  always @(posedge clk1)
  begin
    if( !csb1 ) begin
      web1_reg <= web1;
      wmask1_reg <= wmask1;
      addr1_reg <= addr1;
      din1_reg <= din1;
    end
  end


  always @ (posedge clk0)
  begin : MEM_WRITE0
    if ( !web0_reg ) begin
        if (wmask0_reg[0])
                mem[addr0_reg][1:0] <= din0_reg[1:0];
        if (wmask0_reg[1])
                mem[addr0_reg][3:2] <= din0_reg[3:2];
        if (wmask0_reg[2])
                mem[addr0_reg][5:4] <= din0_reg[5:4];
        if (wmask0_reg[3])
                mem[addr0_reg][7:6] <= din0_reg[7:6];
        if (wmask0_reg[4])
                mem[addr0_reg][9:8] <= din0_reg[9:8];
        if (wmask0_reg[5])
                mem[addr0_reg][11:10] <= din0_reg[11:10];
        if (wmask0_reg[6])
                mem[addr0_reg][13:12] <= din0_reg[13:12];
        if (wmask0_reg[7])
                mem[addr0_reg][15:14] <= din0_reg[15:14];
        if (wmask0_reg[8])
                mem[addr0_reg][17:16] <= din0_reg[17:16];
        if (wmask0_reg[9])
                mem[addr0_reg][19:18] <= din0_reg[19:18];
        if (wmask0_reg[10])
                mem[addr0_reg][21:20] <= din0_reg[21:20];
        if (wmask0_reg[11])
                mem[addr0_reg][23:22] <= din0_reg[23:22];
        if (wmask0_reg[12])
                mem[addr0_reg][25:24] <= din0_reg[25:24];
        if (wmask0_reg[13])
                mem[addr0_reg][27:26] <= din0_reg[27:26];
        if (wmask0_reg[14])
                mem[addr0_reg][29:28] <= din0_reg[29:28];
        if (wmask0_reg[15])
                mem[addr0_reg][31:30] <= din0_reg[31:30];
    end
  end

  always @ (*)
  begin : MEM_READ0
    dout0 = mem[addr0_reg];
  end

  always @ (posedge clk1)
  begin : MEM_WRITE1
    if ( !web1_reg ) begin
        if (wmask1_reg[0])
                mem[addr1_reg][1:0] <= din1_reg[1:0];
        if (wmask1_reg[1])
                mem[addr1_reg][3:2] <= din1_reg[3:2];
        if (wmask1_reg[2])
                mem[addr1_reg][5:4] <= din1_reg[5:4];
        if (wmask1_reg[3])
                mem[addr1_reg][7:6] <= din1_reg[7:6];
        if (wmask1_reg[4])
                mem[addr1_reg][9:8] <= din1_reg[9:8];
        if (wmask1_reg[5])
                mem[addr1_reg][11:10] <= din1_reg[11:10];
        if (wmask1_reg[6])
                mem[addr1_reg][13:12] <= din1_reg[13:12];
        if (wmask1_reg[7])
                mem[addr1_reg][15:14] <= din1_reg[15:14];
        if (wmask1_reg[8])
                mem[addr1_reg][17:16] <= din1_reg[17:16];
        if (wmask1_reg[9])
                mem[addr1_reg][19:18] <= din1_reg[19:18];
        if (wmask1_reg[10])
                mem[addr1_reg][21:20] <= din1_reg[21:20];
        if (wmask1_reg[11])
                mem[addr1_reg][23:22] <= din1_reg[23:22];
        if (wmask1_reg[12])
                mem[addr1_reg][25:24] <= din1_reg[25:24];
        if (wmask1_reg[13])
                mem[addr1_reg][27:26] <= din1_reg[27:26];
        if (wmask1_reg[14])
                mem[addr1_reg][29:28] <= din1_reg[29:28];
        if (wmask1_reg[15])
                mem[addr1_reg][31:30] <= din1_reg[31:30];
    end
  end

  always @ (*)
  begin : MEM_READ1
    dout1 = mem[addr1_reg];
  end

endmodule
