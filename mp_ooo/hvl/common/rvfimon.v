// DO NOT EDIT -- auto-generated from riscv-formal/monitor/generate.py
//
// Command line options: -i rv32imc -a -r 16 -c 8

module riscv_formal_monitor_rv32imc (
  input clock,
  input reset,
  input [7:0] rvfi_valid,
  input [511:0] rvfi_order,
  input [255:0] rvfi_insn,
  input [7:0] rvfi_trap,
  input [7:0] rvfi_halt,
  input [7:0] rvfi_intr,
  input [15:0] rvfi_mode,
  input [39:0] rvfi_rs1_addr,
  input [39:0] rvfi_rs2_addr,
  input [255:0] rvfi_rs1_rdata,
  input [255:0] rvfi_rs2_rdata,
  input [39:0] rvfi_rd_addr,
  input [255:0] rvfi_rd_wdata,
  input [255:0] rvfi_pc_rdata,
  input [255:0] rvfi_pc_wdata,
  input [255:0] rvfi_mem_addr,
  input [31:0] rvfi_mem_rmask,
  input [31:0] rvfi_mem_wmask,
  input [255:0] rvfi_mem_rdata,
  input [255:0] rvfi_mem_wdata,
  input [7:0] rvfi_mem_extamo,
  output reg [15:0] errcode
);
  wire ch0_rvfi_valid = rvfi_valid[0];
  wire [63:0] ch0_rvfi_order = rvfi_order[63:0];
  wire [31:0] ch0_rvfi_insn = rvfi_insn[31:0];
  wire ch0_rvfi_trap = rvfi_trap[0];
  wire ch0_rvfi_halt = rvfi_halt[0];
  wire ch0_rvfi_intr = rvfi_intr[0];
  wire [4:0] ch0_rvfi_rs1_addr = rvfi_rs1_addr[4:0];
  wire [4:0] ch0_rvfi_rs2_addr = rvfi_rs2_addr[4:0];
  wire [31:0] ch0_rvfi_rs1_rdata = rvfi_rs1_rdata[31:0];
  wire [31:0] ch0_rvfi_rs2_rdata = rvfi_rs2_rdata[31:0];
  wire [4:0] ch0_rvfi_rd_addr = rvfi_rd_addr[4:0];
  wire [31:0] ch0_rvfi_rd_wdata = rvfi_rd_wdata[31:0];
  wire [31:0] ch0_rvfi_pc_rdata = rvfi_pc_rdata[31:0];
  wire [31:0] ch0_rvfi_pc_wdata = rvfi_pc_wdata[31:0];
  wire [31:0] ch0_rvfi_mem_addr = rvfi_mem_addr[31:0];
  wire [3:0] ch0_rvfi_mem_rmask = rvfi_mem_rmask[3:0];
  wire [3:0] ch0_rvfi_mem_wmask = rvfi_mem_wmask[3:0];
  wire [31:0] ch0_rvfi_mem_rdata = rvfi_mem_rdata[31:0];
  wire [31:0] ch0_rvfi_mem_wdata = rvfi_mem_wdata[31:0];
  wire ch0_rvfi_mem_extamo = rvfi_mem_extamo[0];

  wire ch0_spec_valid;
  wire ch0_spec_trap;
  wire [4:0] ch0_spec_rs1_addr;
  wire [4:0] ch0_spec_rs2_addr;
  wire [4:0] ch0_spec_rd_addr;
  wire [31:0] ch0_spec_rd_wdata;
  wire [31:0] ch0_spec_pc_wdata;
  wire [31:0] ch0_spec_mem_addr;
  wire [3:0] ch0_spec_mem_rmask;
  wire [3:0] ch0_spec_mem_wmask;
  wire [31:0] ch0_spec_mem_wdata;

  riscv_formal_monitor_rv32imc_isa_spec ch0_isa_spec (
    .rvfi_valid(ch0_rvfi_valid),
    .rvfi_insn(ch0_rvfi_insn),
    .rvfi_pc_rdata(ch0_rvfi_pc_rdata),
    .rvfi_rs1_rdata(ch0_rvfi_rs1_rdata),
    .rvfi_rs2_rdata(ch0_rvfi_rs2_rdata),
    .rvfi_mem_rdata(ch0_rvfi_mem_rdata),
    .spec_valid(ch0_spec_valid),
    .spec_trap(ch0_spec_trap),
    .spec_rs1_addr(ch0_spec_rs1_addr),
    .spec_rs2_addr(ch0_spec_rs2_addr),
    .spec_rd_addr(ch0_spec_rd_addr),
    .spec_rd_wdata(ch0_spec_rd_wdata),
    .spec_pc_wdata(ch0_spec_pc_wdata),
    .spec_mem_addr(ch0_spec_mem_addr),
    .spec_mem_rmask(ch0_spec_mem_rmask),
    .spec_mem_wmask(ch0_spec_mem_wmask),
    .spec_mem_wdata(ch0_spec_mem_wdata)
  );

  reg [15:0] ch0_errcode;

  task ch0_handle_error;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in channel 0: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ch0_rvfi_valid);
      $display("rvfi_order = %x", ch0_rvfi_order);
      $display("rvfi_insn = %x", ch0_rvfi_insn);
      $display("rvfi_trap = %x", ch0_rvfi_trap);
      $display("rvfi_halt = %x", ch0_rvfi_halt);
      $display("rvfi_intr = %x", ch0_rvfi_intr);
      $display("rvfi_rs1_addr = %x", ch0_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ch0_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ch0_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ch0_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ch0_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ch0_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ch0_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ch0_rvfi_pc_wdata);
      $display("rvfi_mem_addr = %x", ch0_rvfi_mem_addr);
      $display("rvfi_mem_rmask = %x", ch0_rvfi_mem_rmask);
      $display("rvfi_mem_wmask = %x", ch0_rvfi_mem_wmask);
      $display("rvfi_mem_rdata = %x", ch0_rvfi_mem_rdata);
      $display("rvfi_mem_wdata = %x", ch0_rvfi_mem_wdata);
      $display("spec_valid = %x", ch0_spec_valid);
      $display("spec_trap = %x", ch0_spec_trap);
      $display("spec_rs1_addr = %x", ch0_spec_rs1_addr);
      $display("spec_rs2_addr = %x", ch0_spec_rs2_addr);
      $display("spec_rd_addr = %x", ch0_spec_rd_addr);
      $display("spec_rd_wdata = %x", ch0_spec_rd_wdata);
      $display("spec_pc_wdata = %x", ch0_spec_pc_wdata);
      $display("spec_mem_addr = %x", ch0_spec_mem_addr);
      $display("spec_mem_rmask = %x", ch0_spec_mem_rmask);
      $display("spec_mem_wmask = %x", ch0_spec_mem_wmask);
      $display("spec_mem_wdata = %x", ch0_spec_mem_wdata);
      ch0_errcode <= code;
    end
  endtask

  always @(posedge clock) begin
    ch0_errcode <= 0;
    if (!reset && ch0_rvfi_valid) begin
      if (ch0_spec_valid) begin
        if (ch0_rvfi_trap != ch0_spec_trap) begin
          ch0_handle_error(101, "mismatch in trap");
        end
        if (ch0_rvfi_rs1_addr != ch0_spec_rs1_addr && ch0_spec_rs1_addr != 0) begin
          ch0_handle_error(102, "mismatch in rs1_addr");
        end
        if (ch0_rvfi_rs2_addr != ch0_spec_rs2_addr && ch0_spec_rs2_addr != 0) begin
          ch0_handle_error(103, "mismatch in rs2_addr");
        end
        if (ch0_rvfi_rd_addr != ch0_spec_rd_addr) begin
          ch0_handle_error(104, "mismatch in rd_addr");
        end
        if (ch0_rvfi_rd_wdata != ch0_spec_rd_wdata) begin
          ch0_handle_error(105, "mismatch in rd_wdata");
        end
        if (ch0_rvfi_pc_wdata != ch0_spec_pc_wdata) begin
          ch0_handle_error(106, "mismatch in pc_wdata");
        end
        if (ch0_rvfi_mem_wmask != ch0_spec_mem_wmask) begin
          ch0_handle_error(108, "mismatch in mem_wmask");
        end
        if (!ch0_rvfi_mem_rmask[0] && ch0_spec_mem_rmask[0]) begin
          ch0_handle_error(110, "mismatch in mem_rmask[0]");
        end
        if (ch0_rvfi_mem_wmask[0] && ch0_rvfi_mem_wdata[7:0] != ch0_spec_mem_wdata[7:0]) begin
          ch0_handle_error(120, "mismatch in mem_wdata[7:0]");
        end
        if (!ch0_rvfi_mem_rmask[1] && ch0_spec_mem_rmask[1]) begin
          ch0_handle_error(111, "mismatch in mem_rmask[1]");
        end
        if (ch0_rvfi_mem_wmask[1] && ch0_rvfi_mem_wdata[15:8] != ch0_spec_mem_wdata[15:8]) begin
          ch0_handle_error(121, "mismatch in mem_wdata[15:8]");
        end
        if (!ch0_rvfi_mem_rmask[2] && ch0_spec_mem_rmask[2]) begin
          ch0_handle_error(112, "mismatch in mem_rmask[2]");
        end
        if (ch0_rvfi_mem_wmask[2] && ch0_rvfi_mem_wdata[23:16] != ch0_spec_mem_wdata[23:16]) begin
          ch0_handle_error(122, "mismatch in mem_wdata[23:16]");
        end
        if (!ch0_rvfi_mem_rmask[3] && ch0_spec_mem_rmask[3]) begin
          ch0_handle_error(113, "mismatch in mem_rmask[3]");
        end
        if (ch0_rvfi_mem_wmask[3] && ch0_rvfi_mem_wdata[31:24] != ch0_spec_mem_wdata[31:24]) begin
          ch0_handle_error(123, "mismatch in mem_wdata[31:24]");
        end
        if (ch0_rvfi_mem_addr != ch0_spec_mem_addr && (ch0_rvfi_mem_wmask || ch0_rvfi_mem_rmask)) begin
          ch0_handle_error(107, "mismatch in mem_addr");
        end
      end
    end
  end

  wire ch1_rvfi_valid = rvfi_valid[1];
  wire [63:0] ch1_rvfi_order = rvfi_order[127:64];
  wire [31:0] ch1_rvfi_insn = rvfi_insn[63:32];
  wire ch1_rvfi_trap = rvfi_trap[1];
  wire ch1_rvfi_halt = rvfi_halt[1];
  wire ch1_rvfi_intr = rvfi_intr[1];
  wire [4:0] ch1_rvfi_rs1_addr = rvfi_rs1_addr[9:5];
  wire [4:0] ch1_rvfi_rs2_addr = rvfi_rs2_addr[9:5];
  wire [31:0] ch1_rvfi_rs1_rdata = rvfi_rs1_rdata[63:32];
  wire [31:0] ch1_rvfi_rs2_rdata = rvfi_rs2_rdata[63:32];
  wire [4:0] ch1_rvfi_rd_addr = rvfi_rd_addr[9:5];
  wire [31:0] ch1_rvfi_rd_wdata = rvfi_rd_wdata[63:32];
  wire [31:0] ch1_rvfi_pc_rdata = rvfi_pc_rdata[63:32];
  wire [31:0] ch1_rvfi_pc_wdata = rvfi_pc_wdata[63:32];
  wire [31:0] ch1_rvfi_mem_addr = rvfi_mem_addr[63:32];
  wire [3:0] ch1_rvfi_mem_rmask = rvfi_mem_rmask[7:4];
  wire [3:0] ch1_rvfi_mem_wmask = rvfi_mem_wmask[7:4];
  wire [31:0] ch1_rvfi_mem_rdata = rvfi_mem_rdata[63:32];
  wire [31:0] ch1_rvfi_mem_wdata = rvfi_mem_wdata[63:32];
  wire ch1_rvfi_mem_extamo = rvfi_mem_extamo[1];

  wire ch1_spec_valid;
  wire ch1_spec_trap;
  wire [4:0] ch1_spec_rs1_addr;
  wire [4:0] ch1_spec_rs2_addr;
  wire [4:0] ch1_spec_rd_addr;
  wire [31:0] ch1_spec_rd_wdata;
  wire [31:0] ch1_spec_pc_wdata;
  wire [31:0] ch1_spec_mem_addr;
  wire [3:0] ch1_spec_mem_rmask;
  wire [3:0] ch1_spec_mem_wmask;
  wire [31:0] ch1_spec_mem_wdata;

  riscv_formal_monitor_rv32imc_isa_spec ch1_isa_spec (
    .rvfi_valid(ch1_rvfi_valid),
    .rvfi_insn(ch1_rvfi_insn),
    .rvfi_pc_rdata(ch1_rvfi_pc_rdata),
    .rvfi_rs1_rdata(ch1_rvfi_rs1_rdata),
    .rvfi_rs2_rdata(ch1_rvfi_rs2_rdata),
    .rvfi_mem_rdata(ch1_rvfi_mem_rdata),
    .spec_valid(ch1_spec_valid),
    .spec_trap(ch1_spec_trap),
    .spec_rs1_addr(ch1_spec_rs1_addr),
    .spec_rs2_addr(ch1_spec_rs2_addr),
    .spec_rd_addr(ch1_spec_rd_addr),
    .spec_rd_wdata(ch1_spec_rd_wdata),
    .spec_pc_wdata(ch1_spec_pc_wdata),
    .spec_mem_addr(ch1_spec_mem_addr),
    .spec_mem_rmask(ch1_spec_mem_rmask),
    .spec_mem_wmask(ch1_spec_mem_wmask),
    .spec_mem_wdata(ch1_spec_mem_wdata)
  );

  reg [15:0] ch1_errcode;

  task ch1_handle_error;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in channel 1: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ch1_rvfi_valid);
      $display("rvfi_order = %x", ch1_rvfi_order);
      $display("rvfi_insn = %x", ch1_rvfi_insn);
      $display("rvfi_trap = %x", ch1_rvfi_trap);
      $display("rvfi_halt = %x", ch1_rvfi_halt);
      $display("rvfi_intr = %x", ch1_rvfi_intr);
      $display("rvfi_rs1_addr = %x", ch1_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ch1_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ch1_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ch1_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ch1_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ch1_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ch1_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ch1_rvfi_pc_wdata);
      $display("rvfi_mem_addr = %x", ch1_rvfi_mem_addr);
      $display("rvfi_mem_rmask = %x", ch1_rvfi_mem_rmask);
      $display("rvfi_mem_wmask = %x", ch1_rvfi_mem_wmask);
      $display("rvfi_mem_rdata = %x", ch1_rvfi_mem_rdata);
      $display("rvfi_mem_wdata = %x", ch1_rvfi_mem_wdata);
      $display("spec_valid = %x", ch1_spec_valid);
      $display("spec_trap = %x", ch1_spec_trap);
      $display("spec_rs1_addr = %x", ch1_spec_rs1_addr);
      $display("spec_rs2_addr = %x", ch1_spec_rs2_addr);
      $display("spec_rd_addr = %x", ch1_spec_rd_addr);
      $display("spec_rd_wdata = %x", ch1_spec_rd_wdata);
      $display("spec_pc_wdata = %x", ch1_spec_pc_wdata);
      $display("spec_mem_addr = %x", ch1_spec_mem_addr);
      $display("spec_mem_rmask = %x", ch1_spec_mem_rmask);
      $display("spec_mem_wmask = %x", ch1_spec_mem_wmask);
      $display("spec_mem_wdata = %x", ch1_spec_mem_wdata);
      ch1_errcode <= code;
    end
  endtask

  always @(posedge clock) begin
    ch1_errcode <= 0;
    if (!reset && ch1_rvfi_valid) begin
      if (ch1_spec_valid) begin
        if (ch1_rvfi_trap != ch1_spec_trap) begin
          ch1_handle_error(201, "mismatch in trap");
        end
        if (ch1_rvfi_rs1_addr != ch1_spec_rs1_addr && ch1_spec_rs1_addr != 0) begin
          ch1_handle_error(202, "mismatch in rs1_addr");
        end
        if (ch1_rvfi_rs2_addr != ch1_spec_rs2_addr && ch1_spec_rs2_addr != 0) begin
          ch1_handle_error(203, "mismatch in rs2_addr");
        end
        if (ch1_rvfi_rd_addr != ch1_spec_rd_addr) begin
          ch1_handle_error(204, "mismatch in rd_addr");
        end
        if (ch1_rvfi_rd_wdata != ch1_spec_rd_wdata) begin
          ch1_handle_error(205, "mismatch in rd_wdata");
        end
        if (ch1_rvfi_pc_wdata != ch1_spec_pc_wdata) begin
          ch1_handle_error(206, "mismatch in pc_wdata");
        end
        if (ch1_rvfi_mem_wmask != ch1_spec_mem_wmask) begin
          ch1_handle_error(208, "mismatch in mem_wmask");
        end
        if (!ch1_rvfi_mem_rmask[0] && ch1_spec_mem_rmask[0]) begin
          ch1_handle_error(210, "mismatch in mem_rmask[0]");
        end
        if (ch1_rvfi_mem_wmask[0] && ch1_rvfi_mem_wdata[7:0] != ch1_spec_mem_wdata[7:0]) begin
          ch1_handle_error(220, "mismatch in mem_wdata[7:0]");
        end
        if (!ch1_rvfi_mem_rmask[1] && ch1_spec_mem_rmask[1]) begin
          ch1_handle_error(211, "mismatch in mem_rmask[1]");
        end
        if (ch1_rvfi_mem_wmask[1] && ch1_rvfi_mem_wdata[15:8] != ch1_spec_mem_wdata[15:8]) begin
          ch1_handle_error(221, "mismatch in mem_wdata[15:8]");
        end
        if (!ch1_rvfi_mem_rmask[2] && ch1_spec_mem_rmask[2]) begin
          ch1_handle_error(212, "mismatch in mem_rmask[2]");
        end
        if (ch1_rvfi_mem_wmask[2] && ch1_rvfi_mem_wdata[23:16] != ch1_spec_mem_wdata[23:16]) begin
          ch1_handle_error(222, "mismatch in mem_wdata[23:16]");
        end
        if (!ch1_rvfi_mem_rmask[3] && ch1_spec_mem_rmask[3]) begin
          ch1_handle_error(213, "mismatch in mem_rmask[3]");
        end
        if (ch1_rvfi_mem_wmask[3] && ch1_rvfi_mem_wdata[31:24] != ch1_spec_mem_wdata[31:24]) begin
          ch1_handle_error(223, "mismatch in mem_wdata[31:24]");
        end
        if (ch1_rvfi_mem_addr != ch1_spec_mem_addr && (ch1_rvfi_mem_wmask || ch1_rvfi_mem_rmask)) begin
          ch1_handle_error(207, "mismatch in mem_addr");
        end
      end
    end
  end

  wire ch2_rvfi_valid = rvfi_valid[2];
  wire [63:0] ch2_rvfi_order = rvfi_order[191:128];
  wire [31:0] ch2_rvfi_insn = rvfi_insn[95:64];
  wire ch2_rvfi_trap = rvfi_trap[2];
  wire ch2_rvfi_halt = rvfi_halt[2];
  wire ch2_rvfi_intr = rvfi_intr[2];
  wire [4:0] ch2_rvfi_rs1_addr = rvfi_rs1_addr[14:10];
  wire [4:0] ch2_rvfi_rs2_addr = rvfi_rs2_addr[14:10];
  wire [31:0] ch2_rvfi_rs1_rdata = rvfi_rs1_rdata[95:64];
  wire [31:0] ch2_rvfi_rs2_rdata = rvfi_rs2_rdata[95:64];
  wire [4:0] ch2_rvfi_rd_addr = rvfi_rd_addr[14:10];
  wire [31:0] ch2_rvfi_rd_wdata = rvfi_rd_wdata[95:64];
  wire [31:0] ch2_rvfi_pc_rdata = rvfi_pc_rdata[95:64];
  wire [31:0] ch2_rvfi_pc_wdata = rvfi_pc_wdata[95:64];
  wire [31:0] ch2_rvfi_mem_addr = rvfi_mem_addr[95:64];
  wire [3:0] ch2_rvfi_mem_rmask = rvfi_mem_rmask[11:8];
  wire [3:0] ch2_rvfi_mem_wmask = rvfi_mem_wmask[11:8];
  wire [31:0] ch2_rvfi_mem_rdata = rvfi_mem_rdata[95:64];
  wire [31:0] ch2_rvfi_mem_wdata = rvfi_mem_wdata[95:64];
  wire ch2_rvfi_mem_extamo = rvfi_mem_extamo[2];

  wire ch2_spec_valid;
  wire ch2_spec_trap;
  wire [4:0] ch2_spec_rs1_addr;
  wire [4:0] ch2_spec_rs2_addr;
  wire [4:0] ch2_spec_rd_addr;
  wire [31:0] ch2_spec_rd_wdata;
  wire [31:0] ch2_spec_pc_wdata;
  wire [31:0] ch2_spec_mem_addr;
  wire [3:0] ch2_spec_mem_rmask;
  wire [3:0] ch2_spec_mem_wmask;
  wire [31:0] ch2_spec_mem_wdata;

  riscv_formal_monitor_rv32imc_isa_spec ch2_isa_spec (
    .rvfi_valid(ch2_rvfi_valid),
    .rvfi_insn(ch2_rvfi_insn),
    .rvfi_pc_rdata(ch2_rvfi_pc_rdata),
    .rvfi_rs1_rdata(ch2_rvfi_rs1_rdata),
    .rvfi_rs2_rdata(ch2_rvfi_rs2_rdata),
    .rvfi_mem_rdata(ch2_rvfi_mem_rdata),
    .spec_valid(ch2_spec_valid),
    .spec_trap(ch2_spec_trap),
    .spec_rs1_addr(ch2_spec_rs1_addr),
    .spec_rs2_addr(ch2_spec_rs2_addr),
    .spec_rd_addr(ch2_spec_rd_addr),
    .spec_rd_wdata(ch2_spec_rd_wdata),
    .spec_pc_wdata(ch2_spec_pc_wdata),
    .spec_mem_addr(ch2_spec_mem_addr),
    .spec_mem_rmask(ch2_spec_mem_rmask),
    .spec_mem_wmask(ch2_spec_mem_wmask),
    .spec_mem_wdata(ch2_spec_mem_wdata)
  );

  reg [15:0] ch2_errcode;

  task ch2_handle_error;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in channel 2: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ch2_rvfi_valid);
      $display("rvfi_order = %x", ch2_rvfi_order);
      $display("rvfi_insn = %x", ch2_rvfi_insn);
      $display("rvfi_trap = %x", ch2_rvfi_trap);
      $display("rvfi_halt = %x", ch2_rvfi_halt);
      $display("rvfi_intr = %x", ch2_rvfi_intr);
      $display("rvfi_rs1_addr = %x", ch2_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ch2_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ch2_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ch2_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ch2_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ch2_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ch2_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ch2_rvfi_pc_wdata);
      $display("rvfi_mem_addr = %x", ch2_rvfi_mem_addr);
      $display("rvfi_mem_rmask = %x", ch2_rvfi_mem_rmask);
      $display("rvfi_mem_wmask = %x", ch2_rvfi_mem_wmask);
      $display("rvfi_mem_rdata = %x", ch2_rvfi_mem_rdata);
      $display("rvfi_mem_wdata = %x", ch2_rvfi_mem_wdata);
      $display("spec_valid = %x", ch2_spec_valid);
      $display("spec_trap = %x", ch2_spec_trap);
      $display("spec_rs1_addr = %x", ch2_spec_rs1_addr);
      $display("spec_rs2_addr = %x", ch2_spec_rs2_addr);
      $display("spec_rd_addr = %x", ch2_spec_rd_addr);
      $display("spec_rd_wdata = %x", ch2_spec_rd_wdata);
      $display("spec_pc_wdata = %x", ch2_spec_pc_wdata);
      $display("spec_mem_addr = %x", ch2_spec_mem_addr);
      $display("spec_mem_rmask = %x", ch2_spec_mem_rmask);
      $display("spec_mem_wmask = %x", ch2_spec_mem_wmask);
      $display("spec_mem_wdata = %x", ch2_spec_mem_wdata);
      ch2_errcode <= code;
    end
  endtask

  always @(posedge clock) begin
    ch2_errcode <= 0;
    if (!reset && ch2_rvfi_valid) begin
      if (ch2_spec_valid) begin
        if (ch2_rvfi_trap != ch2_spec_trap) begin
          ch2_handle_error(301, "mismatch in trap");
        end
        if (ch2_rvfi_rs1_addr != ch2_spec_rs1_addr && ch2_spec_rs1_addr != 0) begin
          ch2_handle_error(302, "mismatch in rs1_addr");
        end
        if (ch2_rvfi_rs2_addr != ch2_spec_rs2_addr && ch2_spec_rs2_addr != 0) begin
          ch2_handle_error(303, "mismatch in rs2_addr");
        end
        if (ch2_rvfi_rd_addr != ch2_spec_rd_addr) begin
          ch2_handle_error(304, "mismatch in rd_addr");
        end
        if (ch2_rvfi_rd_wdata != ch2_spec_rd_wdata) begin
          ch2_handle_error(305, "mismatch in rd_wdata");
        end
        if (ch2_rvfi_pc_wdata != ch2_spec_pc_wdata) begin
          ch2_handle_error(306, "mismatch in pc_wdata");
        end
        if (ch2_rvfi_mem_wmask != ch2_spec_mem_wmask) begin
          ch2_handle_error(308, "mismatch in mem_wmask");
        end
        if (!ch2_rvfi_mem_rmask[0] && ch2_spec_mem_rmask[0]) begin
          ch2_handle_error(310, "mismatch in mem_rmask[0]");
        end
        if (ch2_rvfi_mem_wmask[0] && ch2_rvfi_mem_wdata[7:0] != ch2_spec_mem_wdata[7:0]) begin
          ch2_handle_error(320, "mismatch in mem_wdata[7:0]");
        end
        if (!ch2_rvfi_mem_rmask[1] && ch2_spec_mem_rmask[1]) begin
          ch2_handle_error(311, "mismatch in mem_rmask[1]");
        end
        if (ch2_rvfi_mem_wmask[1] && ch2_rvfi_mem_wdata[15:8] != ch2_spec_mem_wdata[15:8]) begin
          ch2_handle_error(321, "mismatch in mem_wdata[15:8]");
        end
        if (!ch2_rvfi_mem_rmask[2] && ch2_spec_mem_rmask[2]) begin
          ch2_handle_error(312, "mismatch in mem_rmask[2]");
        end
        if (ch2_rvfi_mem_wmask[2] && ch2_rvfi_mem_wdata[23:16] != ch2_spec_mem_wdata[23:16]) begin
          ch2_handle_error(322, "mismatch in mem_wdata[23:16]");
        end
        if (!ch2_rvfi_mem_rmask[3] && ch2_spec_mem_rmask[3]) begin
          ch2_handle_error(313, "mismatch in mem_rmask[3]");
        end
        if (ch2_rvfi_mem_wmask[3] && ch2_rvfi_mem_wdata[31:24] != ch2_spec_mem_wdata[31:24]) begin
          ch2_handle_error(323, "mismatch in mem_wdata[31:24]");
        end
        if (ch2_rvfi_mem_addr != ch2_spec_mem_addr && (ch2_rvfi_mem_wmask || ch2_rvfi_mem_rmask)) begin
          ch2_handle_error(307, "mismatch in mem_addr");
        end
      end
    end
  end

  wire ch3_rvfi_valid = rvfi_valid[3];
  wire [63:0] ch3_rvfi_order = rvfi_order[255:192];
  wire [31:0] ch3_rvfi_insn = rvfi_insn[127:96];
  wire ch3_rvfi_trap = rvfi_trap[3];
  wire ch3_rvfi_halt = rvfi_halt[3];
  wire ch3_rvfi_intr = rvfi_intr[3];
  wire [4:0] ch3_rvfi_rs1_addr = rvfi_rs1_addr[19:15];
  wire [4:0] ch3_rvfi_rs2_addr = rvfi_rs2_addr[19:15];
  wire [31:0] ch3_rvfi_rs1_rdata = rvfi_rs1_rdata[127:96];
  wire [31:0] ch3_rvfi_rs2_rdata = rvfi_rs2_rdata[127:96];
  wire [4:0] ch3_rvfi_rd_addr = rvfi_rd_addr[19:15];
  wire [31:0] ch3_rvfi_rd_wdata = rvfi_rd_wdata[127:96];
  wire [31:0] ch3_rvfi_pc_rdata = rvfi_pc_rdata[127:96];
  wire [31:0] ch3_rvfi_pc_wdata = rvfi_pc_wdata[127:96];
  wire [31:0] ch3_rvfi_mem_addr = rvfi_mem_addr[127:96];
  wire [3:0] ch3_rvfi_mem_rmask = rvfi_mem_rmask[15:12];
  wire [3:0] ch3_rvfi_mem_wmask = rvfi_mem_wmask[15:12];
  wire [31:0] ch3_rvfi_mem_rdata = rvfi_mem_rdata[127:96];
  wire [31:0] ch3_rvfi_mem_wdata = rvfi_mem_wdata[127:96];
  wire ch3_rvfi_mem_extamo = rvfi_mem_extamo[3];

  wire ch3_spec_valid;
  wire ch3_spec_trap;
  wire [4:0] ch3_spec_rs1_addr;
  wire [4:0] ch3_spec_rs2_addr;
  wire [4:0] ch3_spec_rd_addr;
  wire [31:0] ch3_spec_rd_wdata;
  wire [31:0] ch3_spec_pc_wdata;
  wire [31:0] ch3_spec_mem_addr;
  wire [3:0] ch3_spec_mem_rmask;
  wire [3:0] ch3_spec_mem_wmask;
  wire [31:0] ch3_spec_mem_wdata;

  riscv_formal_monitor_rv32imc_isa_spec ch3_isa_spec (
    .rvfi_valid(ch3_rvfi_valid),
    .rvfi_insn(ch3_rvfi_insn),
    .rvfi_pc_rdata(ch3_rvfi_pc_rdata),
    .rvfi_rs1_rdata(ch3_rvfi_rs1_rdata),
    .rvfi_rs2_rdata(ch3_rvfi_rs2_rdata),
    .rvfi_mem_rdata(ch3_rvfi_mem_rdata),
    .spec_valid(ch3_spec_valid),
    .spec_trap(ch3_spec_trap),
    .spec_rs1_addr(ch3_spec_rs1_addr),
    .spec_rs2_addr(ch3_spec_rs2_addr),
    .spec_rd_addr(ch3_spec_rd_addr),
    .spec_rd_wdata(ch3_spec_rd_wdata),
    .spec_pc_wdata(ch3_spec_pc_wdata),
    .spec_mem_addr(ch3_spec_mem_addr),
    .spec_mem_rmask(ch3_spec_mem_rmask),
    .spec_mem_wmask(ch3_spec_mem_wmask),
    .spec_mem_wdata(ch3_spec_mem_wdata)
  );

  reg [15:0] ch3_errcode;

  task ch3_handle_error;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in channel 3: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ch3_rvfi_valid);
      $display("rvfi_order = %x", ch3_rvfi_order);
      $display("rvfi_insn = %x", ch3_rvfi_insn);
      $display("rvfi_trap = %x", ch3_rvfi_trap);
      $display("rvfi_halt = %x", ch3_rvfi_halt);
      $display("rvfi_intr = %x", ch3_rvfi_intr);
      $display("rvfi_rs1_addr = %x", ch3_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ch3_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ch3_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ch3_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ch3_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ch3_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ch3_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ch3_rvfi_pc_wdata);
      $display("rvfi_mem_addr = %x", ch3_rvfi_mem_addr);
      $display("rvfi_mem_rmask = %x", ch3_rvfi_mem_rmask);
      $display("rvfi_mem_wmask = %x", ch3_rvfi_mem_wmask);
      $display("rvfi_mem_rdata = %x", ch3_rvfi_mem_rdata);
      $display("rvfi_mem_wdata = %x", ch3_rvfi_mem_wdata);
      $display("spec_valid = %x", ch3_spec_valid);
      $display("spec_trap = %x", ch3_spec_trap);
      $display("spec_rs1_addr = %x", ch3_spec_rs1_addr);
      $display("spec_rs2_addr = %x", ch3_spec_rs2_addr);
      $display("spec_rd_addr = %x", ch3_spec_rd_addr);
      $display("spec_rd_wdata = %x", ch3_spec_rd_wdata);
      $display("spec_pc_wdata = %x", ch3_spec_pc_wdata);
      $display("spec_mem_addr = %x", ch3_spec_mem_addr);
      $display("spec_mem_rmask = %x", ch3_spec_mem_rmask);
      $display("spec_mem_wmask = %x", ch3_spec_mem_wmask);
      $display("spec_mem_wdata = %x", ch3_spec_mem_wdata);
      ch3_errcode <= code;
    end
  endtask

  always @(posedge clock) begin
    ch3_errcode <= 0;
    if (!reset && ch3_rvfi_valid) begin
      if (ch3_spec_valid) begin
        if (ch3_rvfi_trap != ch3_spec_trap) begin
          ch3_handle_error(401, "mismatch in trap");
        end
        if (ch3_rvfi_rs1_addr != ch3_spec_rs1_addr && ch3_spec_rs1_addr != 0) begin
          ch3_handle_error(402, "mismatch in rs1_addr");
        end
        if (ch3_rvfi_rs2_addr != ch3_spec_rs2_addr && ch3_spec_rs2_addr != 0) begin
          ch3_handle_error(403, "mismatch in rs2_addr");
        end
        if (ch3_rvfi_rd_addr != ch3_spec_rd_addr) begin
          ch3_handle_error(404, "mismatch in rd_addr");
        end
        if (ch3_rvfi_rd_wdata != ch3_spec_rd_wdata) begin
          ch3_handle_error(405, "mismatch in rd_wdata");
        end
        if (ch3_rvfi_pc_wdata != ch3_spec_pc_wdata) begin
          ch3_handle_error(406, "mismatch in pc_wdata");
        end
        if (ch3_rvfi_mem_wmask != ch3_spec_mem_wmask) begin
          ch3_handle_error(408, "mismatch in mem_wmask");
        end
        if (!ch3_rvfi_mem_rmask[0] && ch3_spec_mem_rmask[0]) begin
          ch3_handle_error(410, "mismatch in mem_rmask[0]");
        end
        if (ch3_rvfi_mem_wmask[0] && ch3_rvfi_mem_wdata[7:0] != ch3_spec_mem_wdata[7:0]) begin
          ch3_handle_error(420, "mismatch in mem_wdata[7:0]");
        end
        if (!ch3_rvfi_mem_rmask[1] && ch3_spec_mem_rmask[1]) begin
          ch3_handle_error(411, "mismatch in mem_rmask[1]");
        end
        if (ch3_rvfi_mem_wmask[1] && ch3_rvfi_mem_wdata[15:8] != ch3_spec_mem_wdata[15:8]) begin
          ch3_handle_error(421, "mismatch in mem_wdata[15:8]");
        end
        if (!ch3_rvfi_mem_rmask[2] && ch3_spec_mem_rmask[2]) begin
          ch3_handle_error(412, "mismatch in mem_rmask[2]");
        end
        if (ch3_rvfi_mem_wmask[2] && ch3_rvfi_mem_wdata[23:16] != ch3_spec_mem_wdata[23:16]) begin
          ch3_handle_error(422, "mismatch in mem_wdata[23:16]");
        end
        if (!ch3_rvfi_mem_rmask[3] && ch3_spec_mem_rmask[3]) begin
          ch3_handle_error(413, "mismatch in mem_rmask[3]");
        end
        if (ch3_rvfi_mem_wmask[3] && ch3_rvfi_mem_wdata[31:24] != ch3_spec_mem_wdata[31:24]) begin
          ch3_handle_error(423, "mismatch in mem_wdata[31:24]");
        end
        if (ch3_rvfi_mem_addr != ch3_spec_mem_addr && (ch3_rvfi_mem_wmask || ch3_rvfi_mem_rmask)) begin
          ch3_handle_error(407, "mismatch in mem_addr");
        end
      end
    end
  end

  wire ch4_rvfi_valid = rvfi_valid[4];
  wire [63:0] ch4_rvfi_order = rvfi_order[319:256];
  wire [31:0] ch4_rvfi_insn = rvfi_insn[159:128];
  wire ch4_rvfi_trap = rvfi_trap[4];
  wire ch4_rvfi_halt = rvfi_halt[4];
  wire ch4_rvfi_intr = rvfi_intr[4];
  wire [4:0] ch4_rvfi_rs1_addr = rvfi_rs1_addr[24:20];
  wire [4:0] ch4_rvfi_rs2_addr = rvfi_rs2_addr[24:20];
  wire [31:0] ch4_rvfi_rs1_rdata = rvfi_rs1_rdata[159:128];
  wire [31:0] ch4_rvfi_rs2_rdata = rvfi_rs2_rdata[159:128];
  wire [4:0] ch4_rvfi_rd_addr = rvfi_rd_addr[24:20];
  wire [31:0] ch4_rvfi_rd_wdata = rvfi_rd_wdata[159:128];
  wire [31:0] ch4_rvfi_pc_rdata = rvfi_pc_rdata[159:128];
  wire [31:0] ch4_rvfi_pc_wdata = rvfi_pc_wdata[159:128];
  wire [31:0] ch4_rvfi_mem_addr = rvfi_mem_addr[159:128];
  wire [3:0] ch4_rvfi_mem_rmask = rvfi_mem_rmask[19:16];
  wire [3:0] ch4_rvfi_mem_wmask = rvfi_mem_wmask[19:16];
  wire [31:0] ch4_rvfi_mem_rdata = rvfi_mem_rdata[159:128];
  wire [31:0] ch4_rvfi_mem_wdata = rvfi_mem_wdata[159:128];
  wire ch4_rvfi_mem_extamo = rvfi_mem_extamo[4];

  wire ch4_spec_valid;
  wire ch4_spec_trap;
  wire [4:0] ch4_spec_rs1_addr;
  wire [4:0] ch4_spec_rs2_addr;
  wire [4:0] ch4_spec_rd_addr;
  wire [31:0] ch4_spec_rd_wdata;
  wire [31:0] ch4_spec_pc_wdata;
  wire [31:0] ch4_spec_mem_addr;
  wire [3:0] ch4_spec_mem_rmask;
  wire [3:0] ch4_spec_mem_wmask;
  wire [31:0] ch4_spec_mem_wdata;

  riscv_formal_monitor_rv32imc_isa_spec ch4_isa_spec (
    .rvfi_valid(ch4_rvfi_valid),
    .rvfi_insn(ch4_rvfi_insn),
    .rvfi_pc_rdata(ch4_rvfi_pc_rdata),
    .rvfi_rs1_rdata(ch4_rvfi_rs1_rdata),
    .rvfi_rs2_rdata(ch4_rvfi_rs2_rdata),
    .rvfi_mem_rdata(ch4_rvfi_mem_rdata),
    .spec_valid(ch4_spec_valid),
    .spec_trap(ch4_spec_trap),
    .spec_rs1_addr(ch4_spec_rs1_addr),
    .spec_rs2_addr(ch4_spec_rs2_addr),
    .spec_rd_addr(ch4_spec_rd_addr),
    .spec_rd_wdata(ch4_spec_rd_wdata),
    .spec_pc_wdata(ch4_spec_pc_wdata),
    .spec_mem_addr(ch4_spec_mem_addr),
    .spec_mem_rmask(ch4_spec_mem_rmask),
    .spec_mem_wmask(ch4_spec_mem_wmask),
    .spec_mem_wdata(ch4_spec_mem_wdata)
  );

  reg [15:0] ch4_errcode;

  task ch4_handle_error;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in channel 4: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ch4_rvfi_valid);
      $display("rvfi_order = %x", ch4_rvfi_order);
      $display("rvfi_insn = %x", ch4_rvfi_insn);
      $display("rvfi_trap = %x", ch4_rvfi_trap);
      $display("rvfi_halt = %x", ch4_rvfi_halt);
      $display("rvfi_intr = %x", ch4_rvfi_intr);
      $display("rvfi_rs1_addr = %x", ch4_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ch4_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ch4_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ch4_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ch4_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ch4_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ch4_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ch4_rvfi_pc_wdata);
      $display("rvfi_mem_addr = %x", ch4_rvfi_mem_addr);
      $display("rvfi_mem_rmask = %x", ch4_rvfi_mem_rmask);
      $display("rvfi_mem_wmask = %x", ch4_rvfi_mem_wmask);
      $display("rvfi_mem_rdata = %x", ch4_rvfi_mem_rdata);
      $display("rvfi_mem_wdata = %x", ch4_rvfi_mem_wdata);
      $display("spec_valid = %x", ch4_spec_valid);
      $display("spec_trap = %x", ch4_spec_trap);
      $display("spec_rs1_addr = %x", ch4_spec_rs1_addr);
      $display("spec_rs2_addr = %x", ch4_spec_rs2_addr);
      $display("spec_rd_addr = %x", ch4_spec_rd_addr);
      $display("spec_rd_wdata = %x", ch4_spec_rd_wdata);
      $display("spec_pc_wdata = %x", ch4_spec_pc_wdata);
      $display("spec_mem_addr = %x", ch4_spec_mem_addr);
      $display("spec_mem_rmask = %x", ch4_spec_mem_rmask);
      $display("spec_mem_wmask = %x", ch4_spec_mem_wmask);
      $display("spec_mem_wdata = %x", ch4_spec_mem_wdata);
      ch4_errcode <= code;
    end
  endtask

  always @(posedge clock) begin
    ch4_errcode <= 0;
    if (!reset && ch4_rvfi_valid) begin
      if (ch4_spec_valid) begin
        if (ch4_rvfi_trap != ch4_spec_trap) begin
          ch4_handle_error(501, "mismatch in trap");
        end
        if (ch4_rvfi_rs1_addr != ch4_spec_rs1_addr && ch4_spec_rs1_addr != 0) begin
          ch4_handle_error(502, "mismatch in rs1_addr");
        end
        if (ch4_rvfi_rs2_addr != ch4_spec_rs2_addr && ch4_spec_rs2_addr != 0) begin
          ch4_handle_error(503, "mismatch in rs2_addr");
        end
        if (ch4_rvfi_rd_addr != ch4_spec_rd_addr) begin
          ch4_handle_error(504, "mismatch in rd_addr");
        end
        if (ch4_rvfi_rd_wdata != ch4_spec_rd_wdata) begin
          ch4_handle_error(505, "mismatch in rd_wdata");
        end
        if (ch4_rvfi_pc_wdata != ch4_spec_pc_wdata) begin
          ch4_handle_error(506, "mismatch in pc_wdata");
        end
        if (ch4_rvfi_mem_wmask != ch4_spec_mem_wmask) begin
          ch4_handle_error(508, "mismatch in mem_wmask");
        end
        if (!ch4_rvfi_mem_rmask[0] && ch4_spec_mem_rmask[0]) begin
          ch4_handle_error(510, "mismatch in mem_rmask[0]");
        end
        if (ch4_rvfi_mem_wmask[0] && ch4_rvfi_mem_wdata[7:0] != ch4_spec_mem_wdata[7:0]) begin
          ch4_handle_error(520, "mismatch in mem_wdata[7:0]");
        end
        if (!ch4_rvfi_mem_rmask[1] && ch4_spec_mem_rmask[1]) begin
          ch4_handle_error(511, "mismatch in mem_rmask[1]");
        end
        if (ch4_rvfi_mem_wmask[1] && ch4_rvfi_mem_wdata[15:8] != ch4_spec_mem_wdata[15:8]) begin
          ch4_handle_error(521, "mismatch in mem_wdata[15:8]");
        end
        if (!ch4_rvfi_mem_rmask[2] && ch4_spec_mem_rmask[2]) begin
          ch4_handle_error(512, "mismatch in mem_rmask[2]");
        end
        if (ch4_rvfi_mem_wmask[2] && ch4_rvfi_mem_wdata[23:16] != ch4_spec_mem_wdata[23:16]) begin
          ch4_handle_error(522, "mismatch in mem_wdata[23:16]");
        end
        if (!ch4_rvfi_mem_rmask[3] && ch4_spec_mem_rmask[3]) begin
          ch4_handle_error(513, "mismatch in mem_rmask[3]");
        end
        if (ch4_rvfi_mem_wmask[3] && ch4_rvfi_mem_wdata[31:24] != ch4_spec_mem_wdata[31:24]) begin
          ch4_handle_error(523, "mismatch in mem_wdata[31:24]");
        end
        if (ch4_rvfi_mem_addr != ch4_spec_mem_addr && (ch4_rvfi_mem_wmask || ch4_rvfi_mem_rmask)) begin
          ch4_handle_error(507, "mismatch in mem_addr");
        end
      end
    end
  end

  wire ch5_rvfi_valid = rvfi_valid[5];
  wire [63:0] ch5_rvfi_order = rvfi_order[383:320];
  wire [31:0] ch5_rvfi_insn = rvfi_insn[191:160];
  wire ch5_rvfi_trap = rvfi_trap[5];
  wire ch5_rvfi_halt = rvfi_halt[5];
  wire ch5_rvfi_intr = rvfi_intr[5];
  wire [4:0] ch5_rvfi_rs1_addr = rvfi_rs1_addr[29:25];
  wire [4:0] ch5_rvfi_rs2_addr = rvfi_rs2_addr[29:25];
  wire [31:0] ch5_rvfi_rs1_rdata = rvfi_rs1_rdata[191:160];
  wire [31:0] ch5_rvfi_rs2_rdata = rvfi_rs2_rdata[191:160];
  wire [4:0] ch5_rvfi_rd_addr = rvfi_rd_addr[29:25];
  wire [31:0] ch5_rvfi_rd_wdata = rvfi_rd_wdata[191:160];
  wire [31:0] ch5_rvfi_pc_rdata = rvfi_pc_rdata[191:160];
  wire [31:0] ch5_rvfi_pc_wdata = rvfi_pc_wdata[191:160];
  wire [31:0] ch5_rvfi_mem_addr = rvfi_mem_addr[191:160];
  wire [3:0] ch5_rvfi_mem_rmask = rvfi_mem_rmask[23:20];
  wire [3:0] ch5_rvfi_mem_wmask = rvfi_mem_wmask[23:20];
  wire [31:0] ch5_rvfi_mem_rdata = rvfi_mem_rdata[191:160];
  wire [31:0] ch5_rvfi_mem_wdata = rvfi_mem_wdata[191:160];
  wire ch5_rvfi_mem_extamo = rvfi_mem_extamo[5];

  wire ch5_spec_valid;
  wire ch5_spec_trap;
  wire [4:0] ch5_spec_rs1_addr;
  wire [4:0] ch5_spec_rs2_addr;
  wire [4:0] ch5_spec_rd_addr;
  wire [31:0] ch5_spec_rd_wdata;
  wire [31:0] ch5_spec_pc_wdata;
  wire [31:0] ch5_spec_mem_addr;
  wire [3:0] ch5_spec_mem_rmask;
  wire [3:0] ch5_spec_mem_wmask;
  wire [31:0] ch5_spec_mem_wdata;

  riscv_formal_monitor_rv32imc_isa_spec ch5_isa_spec (
    .rvfi_valid(ch5_rvfi_valid),
    .rvfi_insn(ch5_rvfi_insn),
    .rvfi_pc_rdata(ch5_rvfi_pc_rdata),
    .rvfi_rs1_rdata(ch5_rvfi_rs1_rdata),
    .rvfi_rs2_rdata(ch5_rvfi_rs2_rdata),
    .rvfi_mem_rdata(ch5_rvfi_mem_rdata),
    .spec_valid(ch5_spec_valid),
    .spec_trap(ch5_spec_trap),
    .spec_rs1_addr(ch5_spec_rs1_addr),
    .spec_rs2_addr(ch5_spec_rs2_addr),
    .spec_rd_addr(ch5_spec_rd_addr),
    .spec_rd_wdata(ch5_spec_rd_wdata),
    .spec_pc_wdata(ch5_spec_pc_wdata),
    .spec_mem_addr(ch5_spec_mem_addr),
    .spec_mem_rmask(ch5_spec_mem_rmask),
    .spec_mem_wmask(ch5_spec_mem_wmask),
    .spec_mem_wdata(ch5_spec_mem_wdata)
  );

  reg [15:0] ch5_errcode;

  task ch5_handle_error;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in channel 5: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ch5_rvfi_valid);
      $display("rvfi_order = %x", ch5_rvfi_order);
      $display("rvfi_insn = %x", ch5_rvfi_insn);
      $display("rvfi_trap = %x", ch5_rvfi_trap);
      $display("rvfi_halt = %x", ch5_rvfi_halt);
      $display("rvfi_intr = %x", ch5_rvfi_intr);
      $display("rvfi_rs1_addr = %x", ch5_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ch5_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ch5_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ch5_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ch5_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ch5_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ch5_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ch5_rvfi_pc_wdata);
      $display("rvfi_mem_addr = %x", ch5_rvfi_mem_addr);
      $display("rvfi_mem_rmask = %x", ch5_rvfi_mem_rmask);
      $display("rvfi_mem_wmask = %x", ch5_rvfi_mem_wmask);
      $display("rvfi_mem_rdata = %x", ch5_rvfi_mem_rdata);
      $display("rvfi_mem_wdata = %x", ch5_rvfi_mem_wdata);
      $display("spec_valid = %x", ch5_spec_valid);
      $display("spec_trap = %x", ch5_spec_trap);
      $display("spec_rs1_addr = %x", ch5_spec_rs1_addr);
      $display("spec_rs2_addr = %x", ch5_spec_rs2_addr);
      $display("spec_rd_addr = %x", ch5_spec_rd_addr);
      $display("spec_rd_wdata = %x", ch5_spec_rd_wdata);
      $display("spec_pc_wdata = %x", ch5_spec_pc_wdata);
      $display("spec_mem_addr = %x", ch5_spec_mem_addr);
      $display("spec_mem_rmask = %x", ch5_spec_mem_rmask);
      $display("spec_mem_wmask = %x", ch5_spec_mem_wmask);
      $display("spec_mem_wdata = %x", ch5_spec_mem_wdata);
      ch5_errcode <= code;
    end
  endtask

  always @(posedge clock) begin
    ch5_errcode <= 0;
    if (!reset && ch5_rvfi_valid) begin
      if (ch5_spec_valid) begin
        if (ch5_rvfi_trap != ch5_spec_trap) begin
          ch5_handle_error(601, "mismatch in trap");
        end
        if (ch5_rvfi_rs1_addr != ch5_spec_rs1_addr && ch5_spec_rs1_addr != 0) begin
          ch5_handle_error(602, "mismatch in rs1_addr");
        end
        if (ch5_rvfi_rs2_addr != ch5_spec_rs2_addr && ch5_spec_rs2_addr != 0) begin
          ch5_handle_error(603, "mismatch in rs2_addr");
        end
        if (ch5_rvfi_rd_addr != ch5_spec_rd_addr) begin
          ch5_handle_error(604, "mismatch in rd_addr");
        end
        if (ch5_rvfi_rd_wdata != ch5_spec_rd_wdata) begin
          ch5_handle_error(605, "mismatch in rd_wdata");
        end
        if (ch5_rvfi_pc_wdata != ch5_spec_pc_wdata) begin
          ch5_handle_error(606, "mismatch in pc_wdata");
        end
        if (ch5_rvfi_mem_wmask != ch5_spec_mem_wmask) begin
          ch5_handle_error(608, "mismatch in mem_wmask");
        end
        if (!ch5_rvfi_mem_rmask[0] && ch5_spec_mem_rmask[0]) begin
          ch5_handle_error(610, "mismatch in mem_rmask[0]");
        end
        if (ch5_rvfi_mem_wmask[0] && ch5_rvfi_mem_wdata[7:0] != ch5_spec_mem_wdata[7:0]) begin
          ch5_handle_error(620, "mismatch in mem_wdata[7:0]");
        end
        if (!ch5_rvfi_mem_rmask[1] && ch5_spec_mem_rmask[1]) begin
          ch5_handle_error(611, "mismatch in mem_rmask[1]");
        end
        if (ch5_rvfi_mem_wmask[1] && ch5_rvfi_mem_wdata[15:8] != ch5_spec_mem_wdata[15:8]) begin
          ch5_handle_error(621, "mismatch in mem_wdata[15:8]");
        end
        if (!ch5_rvfi_mem_rmask[2] && ch5_spec_mem_rmask[2]) begin
          ch5_handle_error(612, "mismatch in mem_rmask[2]");
        end
        if (ch5_rvfi_mem_wmask[2] && ch5_rvfi_mem_wdata[23:16] != ch5_spec_mem_wdata[23:16]) begin
          ch5_handle_error(622, "mismatch in mem_wdata[23:16]");
        end
        if (!ch5_rvfi_mem_rmask[3] && ch5_spec_mem_rmask[3]) begin
          ch5_handle_error(613, "mismatch in mem_rmask[3]");
        end
        if (ch5_rvfi_mem_wmask[3] && ch5_rvfi_mem_wdata[31:24] != ch5_spec_mem_wdata[31:24]) begin
          ch5_handle_error(623, "mismatch in mem_wdata[31:24]");
        end
        if (ch5_rvfi_mem_addr != ch5_spec_mem_addr && (ch5_rvfi_mem_wmask || ch5_rvfi_mem_rmask)) begin
          ch5_handle_error(607, "mismatch in mem_addr");
        end
      end
    end
  end

  wire ch6_rvfi_valid = rvfi_valid[6];
  wire [63:0] ch6_rvfi_order = rvfi_order[447:384];
  wire [31:0] ch6_rvfi_insn = rvfi_insn[223:192];
  wire ch6_rvfi_trap = rvfi_trap[6];
  wire ch6_rvfi_halt = rvfi_halt[6];
  wire ch6_rvfi_intr = rvfi_intr[6];
  wire [4:0] ch6_rvfi_rs1_addr = rvfi_rs1_addr[34:30];
  wire [4:0] ch6_rvfi_rs2_addr = rvfi_rs2_addr[34:30];
  wire [31:0] ch6_rvfi_rs1_rdata = rvfi_rs1_rdata[223:192];
  wire [31:0] ch6_rvfi_rs2_rdata = rvfi_rs2_rdata[223:192];
  wire [4:0] ch6_rvfi_rd_addr = rvfi_rd_addr[34:30];
  wire [31:0] ch6_rvfi_rd_wdata = rvfi_rd_wdata[223:192];
  wire [31:0] ch6_rvfi_pc_rdata = rvfi_pc_rdata[223:192];
  wire [31:0] ch6_rvfi_pc_wdata = rvfi_pc_wdata[223:192];
  wire [31:0] ch6_rvfi_mem_addr = rvfi_mem_addr[223:192];
  wire [3:0] ch6_rvfi_mem_rmask = rvfi_mem_rmask[27:24];
  wire [3:0] ch6_rvfi_mem_wmask = rvfi_mem_wmask[27:24];
  wire [31:0] ch6_rvfi_mem_rdata = rvfi_mem_rdata[223:192];
  wire [31:0] ch6_rvfi_mem_wdata = rvfi_mem_wdata[223:192];
  wire ch6_rvfi_mem_extamo = rvfi_mem_extamo[6];

  wire ch6_spec_valid;
  wire ch6_spec_trap;
  wire [4:0] ch6_spec_rs1_addr;
  wire [4:0] ch6_spec_rs2_addr;
  wire [4:0] ch6_spec_rd_addr;
  wire [31:0] ch6_spec_rd_wdata;
  wire [31:0] ch6_spec_pc_wdata;
  wire [31:0] ch6_spec_mem_addr;
  wire [3:0] ch6_spec_mem_rmask;
  wire [3:0] ch6_spec_mem_wmask;
  wire [31:0] ch6_spec_mem_wdata;

  riscv_formal_monitor_rv32imc_isa_spec ch6_isa_spec (
    .rvfi_valid(ch6_rvfi_valid),
    .rvfi_insn(ch6_rvfi_insn),
    .rvfi_pc_rdata(ch6_rvfi_pc_rdata),
    .rvfi_rs1_rdata(ch6_rvfi_rs1_rdata),
    .rvfi_rs2_rdata(ch6_rvfi_rs2_rdata),
    .rvfi_mem_rdata(ch6_rvfi_mem_rdata),
    .spec_valid(ch6_spec_valid),
    .spec_trap(ch6_spec_trap),
    .spec_rs1_addr(ch6_spec_rs1_addr),
    .spec_rs2_addr(ch6_spec_rs2_addr),
    .spec_rd_addr(ch6_spec_rd_addr),
    .spec_rd_wdata(ch6_spec_rd_wdata),
    .spec_pc_wdata(ch6_spec_pc_wdata),
    .spec_mem_addr(ch6_spec_mem_addr),
    .spec_mem_rmask(ch6_spec_mem_rmask),
    .spec_mem_wmask(ch6_spec_mem_wmask),
    .spec_mem_wdata(ch6_spec_mem_wdata)
  );

  reg [15:0] ch6_errcode;

  task ch6_handle_error;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in channel 6: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ch6_rvfi_valid);
      $display("rvfi_order = %x", ch6_rvfi_order);
      $display("rvfi_insn = %x", ch6_rvfi_insn);
      $display("rvfi_trap = %x", ch6_rvfi_trap);
      $display("rvfi_halt = %x", ch6_rvfi_halt);
      $display("rvfi_intr = %x", ch6_rvfi_intr);
      $display("rvfi_rs1_addr = %x", ch6_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ch6_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ch6_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ch6_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ch6_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ch6_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ch6_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ch6_rvfi_pc_wdata);
      $display("rvfi_mem_addr = %x", ch6_rvfi_mem_addr);
      $display("rvfi_mem_rmask = %x", ch6_rvfi_mem_rmask);
      $display("rvfi_mem_wmask = %x", ch6_rvfi_mem_wmask);
      $display("rvfi_mem_rdata = %x", ch6_rvfi_mem_rdata);
      $display("rvfi_mem_wdata = %x", ch6_rvfi_mem_wdata);
      $display("spec_valid = %x", ch6_spec_valid);
      $display("spec_trap = %x", ch6_spec_trap);
      $display("spec_rs1_addr = %x", ch6_spec_rs1_addr);
      $display("spec_rs2_addr = %x", ch6_spec_rs2_addr);
      $display("spec_rd_addr = %x", ch6_spec_rd_addr);
      $display("spec_rd_wdata = %x", ch6_spec_rd_wdata);
      $display("spec_pc_wdata = %x", ch6_spec_pc_wdata);
      $display("spec_mem_addr = %x", ch6_spec_mem_addr);
      $display("spec_mem_rmask = %x", ch6_spec_mem_rmask);
      $display("spec_mem_wmask = %x", ch6_spec_mem_wmask);
      $display("spec_mem_wdata = %x", ch6_spec_mem_wdata);
      ch6_errcode <= code;
    end
  endtask

  always @(posedge clock) begin
    ch6_errcode <= 0;
    if (!reset && ch6_rvfi_valid) begin
      if (ch6_spec_valid) begin
        if (ch6_rvfi_trap != ch6_spec_trap) begin
          ch6_handle_error(701, "mismatch in trap");
        end
        if (ch6_rvfi_rs1_addr != ch6_spec_rs1_addr && ch6_spec_rs1_addr != 0) begin
          ch6_handle_error(702, "mismatch in rs1_addr");
        end
        if (ch6_rvfi_rs2_addr != ch6_spec_rs2_addr && ch6_spec_rs2_addr != 0) begin
          ch6_handle_error(703, "mismatch in rs2_addr");
        end
        if (ch6_rvfi_rd_addr != ch6_spec_rd_addr) begin
          ch6_handle_error(704, "mismatch in rd_addr");
        end
        if (ch6_rvfi_rd_wdata != ch6_spec_rd_wdata) begin
          ch6_handle_error(705, "mismatch in rd_wdata");
        end
        if (ch6_rvfi_pc_wdata != ch6_spec_pc_wdata) begin
          ch6_handle_error(706, "mismatch in pc_wdata");
        end
        if (ch6_rvfi_mem_wmask != ch6_spec_mem_wmask) begin
          ch6_handle_error(708, "mismatch in mem_wmask");
        end
        if (!ch6_rvfi_mem_rmask[0] && ch6_spec_mem_rmask[0]) begin
          ch6_handle_error(710, "mismatch in mem_rmask[0]");
        end
        if (ch6_rvfi_mem_wmask[0] && ch6_rvfi_mem_wdata[7:0] != ch6_spec_mem_wdata[7:0]) begin
          ch6_handle_error(720, "mismatch in mem_wdata[7:0]");
        end
        if (!ch6_rvfi_mem_rmask[1] && ch6_spec_mem_rmask[1]) begin
          ch6_handle_error(711, "mismatch in mem_rmask[1]");
        end
        if (ch6_rvfi_mem_wmask[1] && ch6_rvfi_mem_wdata[15:8] != ch6_spec_mem_wdata[15:8]) begin
          ch6_handle_error(721, "mismatch in mem_wdata[15:8]");
        end
        if (!ch6_rvfi_mem_rmask[2] && ch6_spec_mem_rmask[2]) begin
          ch6_handle_error(712, "mismatch in mem_rmask[2]");
        end
        if (ch6_rvfi_mem_wmask[2] && ch6_rvfi_mem_wdata[23:16] != ch6_spec_mem_wdata[23:16]) begin
          ch6_handle_error(722, "mismatch in mem_wdata[23:16]");
        end
        if (!ch6_rvfi_mem_rmask[3] && ch6_spec_mem_rmask[3]) begin
          ch6_handle_error(713, "mismatch in mem_rmask[3]");
        end
        if (ch6_rvfi_mem_wmask[3] && ch6_rvfi_mem_wdata[31:24] != ch6_spec_mem_wdata[31:24]) begin
          ch6_handle_error(723, "mismatch in mem_wdata[31:24]");
        end
        if (ch6_rvfi_mem_addr != ch6_spec_mem_addr && (ch6_rvfi_mem_wmask || ch6_rvfi_mem_rmask)) begin
          ch6_handle_error(707, "mismatch in mem_addr");
        end
      end
    end
  end

  wire ch7_rvfi_valid = rvfi_valid[7];
  wire [63:0] ch7_rvfi_order = rvfi_order[511:448];
  wire [31:0] ch7_rvfi_insn = rvfi_insn[255:224];
  wire ch7_rvfi_trap = rvfi_trap[7];
  wire ch7_rvfi_halt = rvfi_halt[7];
  wire ch7_rvfi_intr = rvfi_intr[7];
  wire [4:0] ch7_rvfi_rs1_addr = rvfi_rs1_addr[39:35];
  wire [4:0] ch7_rvfi_rs2_addr = rvfi_rs2_addr[39:35];
  wire [31:0] ch7_rvfi_rs1_rdata = rvfi_rs1_rdata[255:224];
  wire [31:0] ch7_rvfi_rs2_rdata = rvfi_rs2_rdata[255:224];
  wire [4:0] ch7_rvfi_rd_addr = rvfi_rd_addr[39:35];
  wire [31:0] ch7_rvfi_rd_wdata = rvfi_rd_wdata[255:224];
  wire [31:0] ch7_rvfi_pc_rdata = rvfi_pc_rdata[255:224];
  wire [31:0] ch7_rvfi_pc_wdata = rvfi_pc_wdata[255:224];
  wire [31:0] ch7_rvfi_mem_addr = rvfi_mem_addr[255:224];
  wire [3:0] ch7_rvfi_mem_rmask = rvfi_mem_rmask[31:28];
  wire [3:0] ch7_rvfi_mem_wmask = rvfi_mem_wmask[31:28];
  wire [31:0] ch7_rvfi_mem_rdata = rvfi_mem_rdata[255:224];
  wire [31:0] ch7_rvfi_mem_wdata = rvfi_mem_wdata[255:224];
  wire ch7_rvfi_mem_extamo = rvfi_mem_extamo[7];

  wire ch7_spec_valid;
  wire ch7_spec_trap;
  wire [4:0] ch7_spec_rs1_addr;
  wire [4:0] ch7_spec_rs2_addr;
  wire [4:0] ch7_spec_rd_addr;
  wire [31:0] ch7_spec_rd_wdata;
  wire [31:0] ch7_spec_pc_wdata;
  wire [31:0] ch7_spec_mem_addr;
  wire [3:0] ch7_spec_mem_rmask;
  wire [3:0] ch7_spec_mem_wmask;
  wire [31:0] ch7_spec_mem_wdata;

  riscv_formal_monitor_rv32imc_isa_spec ch7_isa_spec (
    .rvfi_valid(ch7_rvfi_valid),
    .rvfi_insn(ch7_rvfi_insn),
    .rvfi_pc_rdata(ch7_rvfi_pc_rdata),
    .rvfi_rs1_rdata(ch7_rvfi_rs1_rdata),
    .rvfi_rs2_rdata(ch7_rvfi_rs2_rdata),
    .rvfi_mem_rdata(ch7_rvfi_mem_rdata),
    .spec_valid(ch7_spec_valid),
    .spec_trap(ch7_spec_trap),
    .spec_rs1_addr(ch7_spec_rs1_addr),
    .spec_rs2_addr(ch7_spec_rs2_addr),
    .spec_rd_addr(ch7_spec_rd_addr),
    .spec_rd_wdata(ch7_spec_rd_wdata),
    .spec_pc_wdata(ch7_spec_pc_wdata),
    .spec_mem_addr(ch7_spec_mem_addr),
    .spec_mem_rmask(ch7_spec_mem_rmask),
    .spec_mem_wmask(ch7_spec_mem_wmask),
    .spec_mem_wdata(ch7_spec_mem_wdata)
  );

  reg [15:0] ch7_errcode;

  task ch7_handle_error;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in channel 7: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ch7_rvfi_valid);
      $display("rvfi_order = %x", ch7_rvfi_order);
      $display("rvfi_insn = %x", ch7_rvfi_insn);
      $display("rvfi_trap = %x", ch7_rvfi_trap);
      $display("rvfi_halt = %x", ch7_rvfi_halt);
      $display("rvfi_intr = %x", ch7_rvfi_intr);
      $display("rvfi_rs1_addr = %x", ch7_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ch7_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ch7_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ch7_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ch7_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ch7_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ch7_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ch7_rvfi_pc_wdata);
      $display("rvfi_mem_addr = %x", ch7_rvfi_mem_addr);
      $display("rvfi_mem_rmask = %x", ch7_rvfi_mem_rmask);
      $display("rvfi_mem_wmask = %x", ch7_rvfi_mem_wmask);
      $display("rvfi_mem_rdata = %x", ch7_rvfi_mem_rdata);
      $display("rvfi_mem_wdata = %x", ch7_rvfi_mem_wdata);
      $display("spec_valid = %x", ch7_spec_valid);
      $display("spec_trap = %x", ch7_spec_trap);
      $display("spec_rs1_addr = %x", ch7_spec_rs1_addr);
      $display("spec_rs2_addr = %x", ch7_spec_rs2_addr);
      $display("spec_rd_addr = %x", ch7_spec_rd_addr);
      $display("spec_rd_wdata = %x", ch7_spec_rd_wdata);
      $display("spec_pc_wdata = %x", ch7_spec_pc_wdata);
      $display("spec_mem_addr = %x", ch7_spec_mem_addr);
      $display("spec_mem_rmask = %x", ch7_spec_mem_rmask);
      $display("spec_mem_wmask = %x", ch7_spec_mem_wmask);
      $display("spec_mem_wdata = %x", ch7_spec_mem_wdata);
      ch7_errcode <= code;
    end
  endtask

  always @(posedge clock) begin
    ch7_errcode <= 0;
    if (!reset && ch7_rvfi_valid) begin
      if (ch7_spec_valid) begin
        if (ch7_rvfi_trap != ch7_spec_trap) begin
          ch7_handle_error(801, "mismatch in trap");
        end
        if (ch7_rvfi_rs1_addr != ch7_spec_rs1_addr && ch7_spec_rs1_addr != 0) begin
          ch7_handle_error(802, "mismatch in rs1_addr");
        end
        if (ch7_rvfi_rs2_addr != ch7_spec_rs2_addr && ch7_spec_rs2_addr != 0) begin
          ch7_handle_error(803, "mismatch in rs2_addr");
        end
        if (ch7_rvfi_rd_addr != ch7_spec_rd_addr) begin
          ch7_handle_error(804, "mismatch in rd_addr");
        end
        if (ch7_rvfi_rd_wdata != ch7_spec_rd_wdata) begin
          ch7_handle_error(805, "mismatch in rd_wdata");
        end
        if (ch7_rvfi_pc_wdata != ch7_spec_pc_wdata) begin
          ch7_handle_error(806, "mismatch in pc_wdata");
        end
        if (ch7_rvfi_mem_wmask != ch7_spec_mem_wmask) begin
          ch7_handle_error(808, "mismatch in mem_wmask");
        end
        if (!ch7_rvfi_mem_rmask[0] && ch7_spec_mem_rmask[0]) begin
          ch7_handle_error(810, "mismatch in mem_rmask[0]");
        end
        if (ch7_rvfi_mem_wmask[0] && ch7_rvfi_mem_wdata[7:0] != ch7_spec_mem_wdata[7:0]) begin
          ch7_handle_error(820, "mismatch in mem_wdata[7:0]");
        end
        if (!ch7_rvfi_mem_rmask[1] && ch7_spec_mem_rmask[1]) begin
          ch7_handle_error(811, "mismatch in mem_rmask[1]");
        end
        if (ch7_rvfi_mem_wmask[1] && ch7_rvfi_mem_wdata[15:8] != ch7_spec_mem_wdata[15:8]) begin
          ch7_handle_error(821, "mismatch in mem_wdata[15:8]");
        end
        if (!ch7_rvfi_mem_rmask[2] && ch7_spec_mem_rmask[2]) begin
          ch7_handle_error(812, "mismatch in mem_rmask[2]");
        end
        if (ch7_rvfi_mem_wmask[2] && ch7_rvfi_mem_wdata[23:16] != ch7_spec_mem_wdata[23:16]) begin
          ch7_handle_error(822, "mismatch in mem_wdata[23:16]");
        end
        if (!ch7_rvfi_mem_rmask[3] && ch7_spec_mem_rmask[3]) begin
          ch7_handle_error(813, "mismatch in mem_rmask[3]");
        end
        if (ch7_rvfi_mem_wmask[3] && ch7_rvfi_mem_wdata[31:24] != ch7_spec_mem_wdata[31:24]) begin
          ch7_handle_error(823, "mismatch in mem_wdata[31:24]");
        end
        if (ch7_rvfi_mem_addr != ch7_spec_mem_addr && (ch7_rvfi_mem_wmask || ch7_rvfi_mem_rmask)) begin
          ch7_handle_error(807, "mismatch in mem_addr");
        end
      end
    end
  end

  wire rob_i0_valid;
  wire [63:0] rob_i0_order;
  wire [208:0] rob_i0_data;

  wire rob_o0_valid;
  wire [63:0] rob_o0_order;
  wire [208:0] rob_o0_data;

  wire ro0_rvfi_valid = rob_o0_valid;
  assign rob_i0_valid = ch0_rvfi_valid;

  wire [63:0] ro0_rvfi_order = rob_o0_order;
  assign rob_i0_order = ch0_rvfi_order;

  wire [31:0] ro0_rvfi_insn = rob_o0_data[31:0];
  assign rob_i0_data[31:0] = ch0_rvfi_insn;

  wire [4:0] ro0_rvfi_rs1_addr = rob_o0_data[36:32];
  assign rob_i0_data[36:32] = ch0_rvfi_rs1_addr;

  wire [4:0] ro0_rvfi_rs2_addr = rob_o0_data[41:37];
  assign rob_i0_data[41:37] = ch0_rvfi_rs2_addr;

  wire [4:0] ro0_rvfi_rd_addr = rob_o0_data[46:42];
  assign rob_i0_data[46:42] = ch0_rvfi_rd_addr;

  wire [31:0] ro0_rvfi_rs1_rdata = rob_o0_data[78:47];
  assign rob_i0_data[78:47] = ch0_rvfi_rs1_rdata;

  wire [31:0] ro0_rvfi_rs2_rdata = rob_o0_data[110:79];
  assign rob_i0_data[110:79] = ch0_rvfi_rs2_rdata;

  wire [31:0] ro0_rvfi_rd_wdata = rob_o0_data[142:111];
  assign rob_i0_data[142:111] = ch0_rvfi_rd_wdata;

  wire [31:0] ro0_rvfi_pc_rdata = rob_o0_data[174:143];
  assign rob_i0_data[174:143] = ch0_rvfi_pc_rdata;

  wire [31:0] ro0_rvfi_pc_wdata = rob_o0_data[206:175];
  assign rob_i0_data[206:175] = ch0_rvfi_pc_wdata;

  wire [0:0] ro0_rvfi_intr = rob_o0_data[207:207];
  assign rob_i0_data[207:207] = ch0_rvfi_intr;

  wire [0:0] ro0_rvfi_trap = rob_o0_data[208:208];
  assign rob_i0_data[208:208] = ch0_rvfi_trap;

  wire rob_i1_valid;
  wire [63:0] rob_i1_order;
  wire [208:0] rob_i1_data;

  wire rob_o1_valid;
  wire [63:0] rob_o1_order;
  wire [208:0] rob_o1_data;

  wire ro1_rvfi_valid = rob_o1_valid;
  assign rob_i1_valid = ch1_rvfi_valid;

  wire [63:0] ro1_rvfi_order = rob_o1_order;
  assign rob_i1_order = ch1_rvfi_order;

  wire [31:0] ro1_rvfi_insn = rob_o1_data[31:0];
  assign rob_i1_data[31:0] = ch1_rvfi_insn;

  wire [4:0] ro1_rvfi_rs1_addr = rob_o1_data[36:32];
  assign rob_i1_data[36:32] = ch1_rvfi_rs1_addr;

  wire [4:0] ro1_rvfi_rs2_addr = rob_o1_data[41:37];
  assign rob_i1_data[41:37] = ch1_rvfi_rs2_addr;

  wire [4:0] ro1_rvfi_rd_addr = rob_o1_data[46:42];
  assign rob_i1_data[46:42] = ch1_rvfi_rd_addr;

  wire [31:0] ro1_rvfi_rs1_rdata = rob_o1_data[78:47];
  assign rob_i1_data[78:47] = ch1_rvfi_rs1_rdata;

  wire [31:0] ro1_rvfi_rs2_rdata = rob_o1_data[110:79];
  assign rob_i1_data[110:79] = ch1_rvfi_rs2_rdata;

  wire [31:0] ro1_rvfi_rd_wdata = rob_o1_data[142:111];
  assign rob_i1_data[142:111] = ch1_rvfi_rd_wdata;

  wire [31:0] ro1_rvfi_pc_rdata = rob_o1_data[174:143];
  assign rob_i1_data[174:143] = ch1_rvfi_pc_rdata;

  wire [31:0] ro1_rvfi_pc_wdata = rob_o1_data[206:175];
  assign rob_i1_data[206:175] = ch1_rvfi_pc_wdata;

  wire [0:0] ro1_rvfi_intr = rob_o1_data[207:207];
  assign rob_i1_data[207:207] = ch1_rvfi_intr;

  wire [0:0] ro1_rvfi_trap = rob_o1_data[208:208];
  assign rob_i1_data[208:208] = ch1_rvfi_trap;

  wire rob_i2_valid;
  wire [63:0] rob_i2_order;
  wire [208:0] rob_i2_data;

  wire rob_o2_valid;
  wire [63:0] rob_o2_order;
  wire [208:0] rob_o2_data;

  wire ro2_rvfi_valid = rob_o2_valid;
  assign rob_i2_valid = ch2_rvfi_valid;

  wire [63:0] ro2_rvfi_order = rob_o2_order;
  assign rob_i2_order = ch2_rvfi_order;

  wire [31:0] ro2_rvfi_insn = rob_o2_data[31:0];
  assign rob_i2_data[31:0] = ch2_rvfi_insn;

  wire [4:0] ro2_rvfi_rs1_addr = rob_o2_data[36:32];
  assign rob_i2_data[36:32] = ch2_rvfi_rs1_addr;

  wire [4:0] ro2_rvfi_rs2_addr = rob_o2_data[41:37];
  assign rob_i2_data[41:37] = ch2_rvfi_rs2_addr;

  wire [4:0] ro2_rvfi_rd_addr = rob_o2_data[46:42];
  assign rob_i2_data[46:42] = ch2_rvfi_rd_addr;

  wire [31:0] ro2_rvfi_rs1_rdata = rob_o2_data[78:47];
  assign rob_i2_data[78:47] = ch2_rvfi_rs1_rdata;

  wire [31:0] ro2_rvfi_rs2_rdata = rob_o2_data[110:79];
  assign rob_i2_data[110:79] = ch2_rvfi_rs2_rdata;

  wire [31:0] ro2_rvfi_rd_wdata = rob_o2_data[142:111];
  assign rob_i2_data[142:111] = ch2_rvfi_rd_wdata;

  wire [31:0] ro2_rvfi_pc_rdata = rob_o2_data[174:143];
  assign rob_i2_data[174:143] = ch2_rvfi_pc_rdata;

  wire [31:0] ro2_rvfi_pc_wdata = rob_o2_data[206:175];
  assign rob_i2_data[206:175] = ch2_rvfi_pc_wdata;

  wire [0:0] ro2_rvfi_intr = rob_o2_data[207:207];
  assign rob_i2_data[207:207] = ch2_rvfi_intr;

  wire [0:0] ro2_rvfi_trap = rob_o2_data[208:208];
  assign rob_i2_data[208:208] = ch2_rvfi_trap;

  wire rob_i3_valid;
  wire [63:0] rob_i3_order;
  wire [208:0] rob_i3_data;

  wire rob_o3_valid;
  wire [63:0] rob_o3_order;
  wire [208:0] rob_o3_data;

  wire ro3_rvfi_valid = rob_o3_valid;
  assign rob_i3_valid = ch3_rvfi_valid;

  wire [63:0] ro3_rvfi_order = rob_o3_order;
  assign rob_i3_order = ch3_rvfi_order;

  wire [31:0] ro3_rvfi_insn = rob_o3_data[31:0];
  assign rob_i3_data[31:0] = ch3_rvfi_insn;

  wire [4:0] ro3_rvfi_rs1_addr = rob_o3_data[36:32];
  assign rob_i3_data[36:32] = ch3_rvfi_rs1_addr;

  wire [4:0] ro3_rvfi_rs2_addr = rob_o3_data[41:37];
  assign rob_i3_data[41:37] = ch3_rvfi_rs2_addr;

  wire [4:0] ro3_rvfi_rd_addr = rob_o3_data[46:42];
  assign rob_i3_data[46:42] = ch3_rvfi_rd_addr;

  wire [31:0] ro3_rvfi_rs1_rdata = rob_o3_data[78:47];
  assign rob_i3_data[78:47] = ch3_rvfi_rs1_rdata;

  wire [31:0] ro3_rvfi_rs2_rdata = rob_o3_data[110:79];
  assign rob_i3_data[110:79] = ch3_rvfi_rs2_rdata;

  wire [31:0] ro3_rvfi_rd_wdata = rob_o3_data[142:111];
  assign rob_i3_data[142:111] = ch3_rvfi_rd_wdata;

  wire [31:0] ro3_rvfi_pc_rdata = rob_o3_data[174:143];
  assign rob_i3_data[174:143] = ch3_rvfi_pc_rdata;

  wire [31:0] ro3_rvfi_pc_wdata = rob_o3_data[206:175];
  assign rob_i3_data[206:175] = ch3_rvfi_pc_wdata;

  wire [0:0] ro3_rvfi_intr = rob_o3_data[207:207];
  assign rob_i3_data[207:207] = ch3_rvfi_intr;

  wire [0:0] ro3_rvfi_trap = rob_o3_data[208:208];
  assign rob_i3_data[208:208] = ch3_rvfi_trap;

  wire rob_i4_valid;
  wire [63:0] rob_i4_order;
  wire [208:0] rob_i4_data;

  wire rob_o4_valid;
  wire [63:0] rob_o4_order;
  wire [208:0] rob_o4_data;

  wire ro4_rvfi_valid = rob_o4_valid;
  assign rob_i4_valid = ch4_rvfi_valid;

  wire [63:0] ro4_rvfi_order = rob_o4_order;
  assign rob_i4_order = ch4_rvfi_order;

  wire [31:0] ro4_rvfi_insn = rob_o4_data[31:0];
  assign rob_i4_data[31:0] = ch4_rvfi_insn;

  wire [4:0] ro4_rvfi_rs1_addr = rob_o4_data[36:32];
  assign rob_i4_data[36:32] = ch4_rvfi_rs1_addr;

  wire [4:0] ro4_rvfi_rs2_addr = rob_o4_data[41:37];
  assign rob_i4_data[41:37] = ch4_rvfi_rs2_addr;

  wire [4:0] ro4_rvfi_rd_addr = rob_o4_data[46:42];
  assign rob_i4_data[46:42] = ch4_rvfi_rd_addr;

  wire [31:0] ro4_rvfi_rs1_rdata = rob_o4_data[78:47];
  assign rob_i4_data[78:47] = ch4_rvfi_rs1_rdata;

  wire [31:0] ro4_rvfi_rs2_rdata = rob_o4_data[110:79];
  assign rob_i4_data[110:79] = ch4_rvfi_rs2_rdata;

  wire [31:0] ro4_rvfi_rd_wdata = rob_o4_data[142:111];
  assign rob_i4_data[142:111] = ch4_rvfi_rd_wdata;

  wire [31:0] ro4_rvfi_pc_rdata = rob_o4_data[174:143];
  assign rob_i4_data[174:143] = ch4_rvfi_pc_rdata;

  wire [31:0] ro4_rvfi_pc_wdata = rob_o4_data[206:175];
  assign rob_i4_data[206:175] = ch4_rvfi_pc_wdata;

  wire [0:0] ro4_rvfi_intr = rob_o4_data[207:207];
  assign rob_i4_data[207:207] = ch4_rvfi_intr;

  wire [0:0] ro4_rvfi_trap = rob_o4_data[208:208];
  assign rob_i4_data[208:208] = ch4_rvfi_trap;

  wire rob_i5_valid;
  wire [63:0] rob_i5_order;
  wire [208:0] rob_i5_data;

  wire rob_o5_valid;
  wire [63:0] rob_o5_order;
  wire [208:0] rob_o5_data;

  wire ro5_rvfi_valid = rob_o5_valid;
  assign rob_i5_valid = ch5_rvfi_valid;

  wire [63:0] ro5_rvfi_order = rob_o5_order;
  assign rob_i5_order = ch5_rvfi_order;

  wire [31:0] ro5_rvfi_insn = rob_o5_data[31:0];
  assign rob_i5_data[31:0] = ch5_rvfi_insn;

  wire [4:0] ro5_rvfi_rs1_addr = rob_o5_data[36:32];
  assign rob_i5_data[36:32] = ch5_rvfi_rs1_addr;

  wire [4:0] ro5_rvfi_rs2_addr = rob_o5_data[41:37];
  assign rob_i5_data[41:37] = ch5_rvfi_rs2_addr;

  wire [4:0] ro5_rvfi_rd_addr = rob_o5_data[46:42];
  assign rob_i5_data[46:42] = ch5_rvfi_rd_addr;

  wire [31:0] ro5_rvfi_rs1_rdata = rob_o5_data[78:47];
  assign rob_i5_data[78:47] = ch5_rvfi_rs1_rdata;

  wire [31:0] ro5_rvfi_rs2_rdata = rob_o5_data[110:79];
  assign rob_i5_data[110:79] = ch5_rvfi_rs2_rdata;

  wire [31:0] ro5_rvfi_rd_wdata = rob_o5_data[142:111];
  assign rob_i5_data[142:111] = ch5_rvfi_rd_wdata;

  wire [31:0] ro5_rvfi_pc_rdata = rob_o5_data[174:143];
  assign rob_i5_data[174:143] = ch5_rvfi_pc_rdata;

  wire [31:0] ro5_rvfi_pc_wdata = rob_o5_data[206:175];
  assign rob_i5_data[206:175] = ch5_rvfi_pc_wdata;

  wire [0:0] ro5_rvfi_intr = rob_o5_data[207:207];
  assign rob_i5_data[207:207] = ch5_rvfi_intr;

  wire [0:0] ro5_rvfi_trap = rob_o5_data[208:208];
  assign rob_i5_data[208:208] = ch5_rvfi_trap;

  wire rob_i6_valid;
  wire [63:0] rob_i6_order;
  wire [208:0] rob_i6_data;

  wire rob_o6_valid;
  wire [63:0] rob_o6_order;
  wire [208:0] rob_o6_data;

  wire ro6_rvfi_valid = rob_o6_valid;
  assign rob_i6_valid = ch6_rvfi_valid;

  wire [63:0] ro6_rvfi_order = rob_o6_order;
  assign rob_i6_order = ch6_rvfi_order;

  wire [31:0] ro6_rvfi_insn = rob_o6_data[31:0];
  assign rob_i6_data[31:0] = ch6_rvfi_insn;

  wire [4:0] ro6_rvfi_rs1_addr = rob_o6_data[36:32];
  assign rob_i6_data[36:32] = ch6_rvfi_rs1_addr;

  wire [4:0] ro6_rvfi_rs2_addr = rob_o6_data[41:37];
  assign rob_i6_data[41:37] = ch6_rvfi_rs2_addr;

  wire [4:0] ro6_rvfi_rd_addr = rob_o6_data[46:42];
  assign rob_i6_data[46:42] = ch6_rvfi_rd_addr;

  wire [31:0] ro6_rvfi_rs1_rdata = rob_o6_data[78:47];
  assign rob_i6_data[78:47] = ch6_rvfi_rs1_rdata;

  wire [31:0] ro6_rvfi_rs2_rdata = rob_o6_data[110:79];
  assign rob_i6_data[110:79] = ch6_rvfi_rs2_rdata;

  wire [31:0] ro6_rvfi_rd_wdata = rob_o6_data[142:111];
  assign rob_i6_data[142:111] = ch6_rvfi_rd_wdata;

  wire [31:0] ro6_rvfi_pc_rdata = rob_o6_data[174:143];
  assign rob_i6_data[174:143] = ch6_rvfi_pc_rdata;

  wire [31:0] ro6_rvfi_pc_wdata = rob_o6_data[206:175];
  assign rob_i6_data[206:175] = ch6_rvfi_pc_wdata;

  wire [0:0] ro6_rvfi_intr = rob_o6_data[207:207];
  assign rob_i6_data[207:207] = ch6_rvfi_intr;

  wire [0:0] ro6_rvfi_trap = rob_o6_data[208:208];
  assign rob_i6_data[208:208] = ch6_rvfi_trap;

  wire rob_i7_valid;
  wire [63:0] rob_i7_order;
  wire [208:0] rob_i7_data;

  wire rob_o7_valid;
  wire [63:0] rob_o7_order;
  wire [208:0] rob_o7_data;

  wire ro7_rvfi_valid = rob_o7_valid;
  assign rob_i7_valid = ch7_rvfi_valid;

  wire [63:0] ro7_rvfi_order = rob_o7_order;
  assign rob_i7_order = ch7_rvfi_order;

  wire [31:0] ro7_rvfi_insn = rob_o7_data[31:0];
  assign rob_i7_data[31:0] = ch7_rvfi_insn;

  wire [4:0] ro7_rvfi_rs1_addr = rob_o7_data[36:32];
  assign rob_i7_data[36:32] = ch7_rvfi_rs1_addr;

  wire [4:0] ro7_rvfi_rs2_addr = rob_o7_data[41:37];
  assign rob_i7_data[41:37] = ch7_rvfi_rs2_addr;

  wire [4:0] ro7_rvfi_rd_addr = rob_o7_data[46:42];
  assign rob_i7_data[46:42] = ch7_rvfi_rd_addr;

  wire [31:0] ro7_rvfi_rs1_rdata = rob_o7_data[78:47];
  assign rob_i7_data[78:47] = ch7_rvfi_rs1_rdata;

  wire [31:0] ro7_rvfi_rs2_rdata = rob_o7_data[110:79];
  assign rob_i7_data[110:79] = ch7_rvfi_rs2_rdata;

  wire [31:0] ro7_rvfi_rd_wdata = rob_o7_data[142:111];
  assign rob_i7_data[142:111] = ch7_rvfi_rd_wdata;

  wire [31:0] ro7_rvfi_pc_rdata = rob_o7_data[174:143];
  assign rob_i7_data[174:143] = ch7_rvfi_pc_rdata;

  wire [31:0] ro7_rvfi_pc_wdata = rob_o7_data[206:175];
  assign rob_i7_data[206:175] = ch7_rvfi_pc_wdata;

  wire [0:0] ro7_rvfi_intr = rob_o7_data[207:207];
  assign rob_i7_data[207:207] = ch7_rvfi_intr;

  wire [0:0] ro7_rvfi_trap = rob_o7_data[208:208];
  assign rob_i7_data[208:208] = ch7_rvfi_trap;

  wire [15:0] rob_errcode;

  riscv_formal_monitor_rv32imc_rob rob (
    .clock(clock),
    .reset(reset),
    .i0_valid(rob_i0_valid),
    .i0_order(rob_i0_order),
    .i0_data(rob_i0_data),
    .o0_valid(rob_o0_valid),
    .o0_order(rob_o0_order),
    .o0_data(rob_o0_data),
    .i1_valid(rob_i1_valid),
    .i1_order(rob_i1_order),
    .i1_data(rob_i1_data),
    .o1_valid(rob_o1_valid),
    .o1_order(rob_o1_order),
    .o1_data(rob_o1_data),
    .i2_valid(rob_i2_valid),
    .i2_order(rob_i2_order),
    .i2_data(rob_i2_data),
    .o2_valid(rob_o2_valid),
    .o2_order(rob_o2_order),
    .o2_data(rob_o2_data),
    .i3_valid(rob_i3_valid),
    .i3_order(rob_i3_order),
    .i3_data(rob_i3_data),
    .o3_valid(rob_o3_valid),
    .o3_order(rob_o3_order),
    .o3_data(rob_o3_data),
    .i4_valid(rob_i4_valid),
    .i4_order(rob_i4_order),
    .i4_data(rob_i4_data),
    .o4_valid(rob_o4_valid),
    .o4_order(rob_o4_order),
    .o4_data(rob_o4_data),
    .i5_valid(rob_i5_valid),
    .i5_order(rob_i5_order),
    .i5_data(rob_i5_data),
    .o5_valid(rob_o5_valid),
    .o5_order(rob_o5_order),
    .o5_data(rob_o5_data),
    .i6_valid(rob_i6_valid),
    .i6_order(rob_i6_order),
    .i6_data(rob_i6_data),
    .o6_valid(rob_o6_valid),
    .o6_order(rob_o6_order),
    .o6_data(rob_o6_data),
    .i7_valid(rob_i7_valid),
    .i7_order(rob_i7_order),
    .i7_data(rob_i7_data),
    .o7_valid(rob_o7_valid),
    .o7_order(rob_o7_order),
    .o7_data(rob_o7_data),
    .errcode(rob_errcode)
  );

  always @(posedge clock) begin
    if (!reset && rob_errcode) begin
      $display("-------- RVFI Monitor ROB error %0d: %m at time %0t --------", rob_errcode, $time);
      $display("No details on ROB errors available.");
    end
  end

  reg shadow_pc_valid;
  reg shadow_pc_trap;
  reg [31:0] shadow_pc;

  reg shadow0_pc_valid;
  reg shadow0_pc_trap;
  reg [31:0] shadow0_pc_rdata;

  reg [15:0] ro0_errcode_p;

  task ro0_handle_error_p;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 0: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro0_rvfi_valid);
      $display("rvfi_order = %x", ro0_rvfi_order);
      $display("rvfi_insn = %x", ro0_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro0_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro0_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro0_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro0_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro0_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro0_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro0_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro0_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro0_rvfi_intr);
      $display("rvfi_trap = %x", ro0_rvfi_trap);
      $display("shadow_pc_valid = %x", shadow0_pc_valid);
      $display("shadow_pc_rdata = %x", shadow0_pc_rdata);
      ro0_errcode_p <= code;
    end
  endtask

  always @* begin
    shadow0_pc_valid = shadow_pc_valid;
    shadow0_pc_trap = shadow_pc_trap;
    shadow0_pc_rdata = shadow_pc;
  end

  reg shadow1_pc_valid;
  reg shadow1_pc_trap;
  reg [31:0] shadow1_pc_rdata;

  reg [15:0] ro1_errcode_p;

  task ro1_handle_error_p;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 1: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro1_rvfi_valid);
      $display("rvfi_order = %x", ro1_rvfi_order);
      $display("rvfi_insn = %x", ro1_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro1_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro1_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro1_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro1_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro1_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro1_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro1_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro1_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro1_rvfi_intr);
      $display("rvfi_trap = %x", ro1_rvfi_trap);
      $display("shadow_pc_valid = %x", shadow1_pc_valid);
      $display("shadow_pc_rdata = %x", shadow1_pc_rdata);
      ro1_errcode_p <= code;
    end
  endtask

  always @* begin
    shadow1_pc_valid = shadow_pc_valid;
    shadow1_pc_trap = shadow_pc_trap;
    shadow1_pc_rdata = shadow_pc;
    if (!reset && ro0_rvfi_valid) begin
      shadow1_pc_valid = !ro1_rvfi_trap;
      shadow1_pc_trap = ro1_rvfi_trap;
      shadow1_pc_rdata = ro0_rvfi_pc_wdata;
    end
  end

  reg shadow2_pc_valid;
  reg shadow2_pc_trap;
  reg [31:0] shadow2_pc_rdata;

  reg [15:0] ro2_errcode_p;

  task ro2_handle_error_p;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 2: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro2_rvfi_valid);
      $display("rvfi_order = %x", ro2_rvfi_order);
      $display("rvfi_insn = %x", ro2_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro2_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro2_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro2_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro2_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro2_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro2_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro2_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro2_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro2_rvfi_intr);
      $display("rvfi_trap = %x", ro2_rvfi_trap);
      $display("shadow_pc_valid = %x", shadow2_pc_valid);
      $display("shadow_pc_rdata = %x", shadow2_pc_rdata);
      ro2_errcode_p <= code;
    end
  endtask

  always @* begin
    shadow2_pc_valid = shadow_pc_valid;
    shadow2_pc_trap = shadow_pc_trap;
    shadow2_pc_rdata = shadow_pc;
    if (!reset && ro0_rvfi_valid) begin
      shadow2_pc_valid = !ro2_rvfi_trap;
      shadow2_pc_trap = ro2_rvfi_trap;
      shadow2_pc_rdata = ro0_rvfi_pc_wdata;
    end
    if (!reset && ro1_rvfi_valid) begin
      shadow2_pc_valid = !ro2_rvfi_trap;
      shadow2_pc_trap = ro2_rvfi_trap;
      shadow2_pc_rdata = ro1_rvfi_pc_wdata;
    end
  end

  reg shadow3_pc_valid;
  reg shadow3_pc_trap;
  reg [31:0] shadow3_pc_rdata;

  reg [15:0] ro3_errcode_p;

  task ro3_handle_error_p;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 3: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro3_rvfi_valid);
      $display("rvfi_order = %x", ro3_rvfi_order);
      $display("rvfi_insn = %x", ro3_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro3_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro3_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro3_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro3_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro3_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro3_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro3_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro3_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro3_rvfi_intr);
      $display("rvfi_trap = %x", ro3_rvfi_trap);
      $display("shadow_pc_valid = %x", shadow3_pc_valid);
      $display("shadow_pc_rdata = %x", shadow3_pc_rdata);
      ro3_errcode_p <= code;
    end
  endtask

  always @* begin
    shadow3_pc_valid = shadow_pc_valid;
    shadow3_pc_trap = shadow_pc_trap;
    shadow3_pc_rdata = shadow_pc;
    if (!reset && ro0_rvfi_valid) begin
      shadow3_pc_valid = !ro3_rvfi_trap;
      shadow3_pc_trap = ro3_rvfi_trap;
      shadow3_pc_rdata = ro0_rvfi_pc_wdata;
    end
    if (!reset && ro1_rvfi_valid) begin
      shadow3_pc_valid = !ro3_rvfi_trap;
      shadow3_pc_trap = ro3_rvfi_trap;
      shadow3_pc_rdata = ro1_rvfi_pc_wdata;
    end
    if (!reset && ro2_rvfi_valid) begin
      shadow3_pc_valid = !ro3_rvfi_trap;
      shadow3_pc_trap = ro3_rvfi_trap;
      shadow3_pc_rdata = ro2_rvfi_pc_wdata;
    end
  end

  reg shadow4_pc_valid;
  reg shadow4_pc_trap;
  reg [31:0] shadow4_pc_rdata;

  reg [15:0] ro4_errcode_p;

  task ro4_handle_error_p;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 4: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro4_rvfi_valid);
      $display("rvfi_order = %x", ro4_rvfi_order);
      $display("rvfi_insn = %x", ro4_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro4_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro4_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro4_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro4_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro4_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro4_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro4_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro4_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro4_rvfi_intr);
      $display("rvfi_trap = %x", ro4_rvfi_trap);
      $display("shadow_pc_valid = %x", shadow4_pc_valid);
      $display("shadow_pc_rdata = %x", shadow4_pc_rdata);
      ro4_errcode_p <= code;
    end
  endtask

  always @* begin
    shadow4_pc_valid = shadow_pc_valid;
    shadow4_pc_trap = shadow_pc_trap;
    shadow4_pc_rdata = shadow_pc;
    if (!reset && ro0_rvfi_valid) begin
      shadow4_pc_valid = !ro4_rvfi_trap;
      shadow4_pc_trap = ro4_rvfi_trap;
      shadow4_pc_rdata = ro0_rvfi_pc_wdata;
    end
    if (!reset && ro1_rvfi_valid) begin
      shadow4_pc_valid = !ro4_rvfi_trap;
      shadow4_pc_trap = ro4_rvfi_trap;
      shadow4_pc_rdata = ro1_rvfi_pc_wdata;
    end
    if (!reset && ro2_rvfi_valid) begin
      shadow4_pc_valid = !ro4_rvfi_trap;
      shadow4_pc_trap = ro4_rvfi_trap;
      shadow4_pc_rdata = ro2_rvfi_pc_wdata;
    end
    if (!reset && ro3_rvfi_valid) begin
      shadow4_pc_valid = !ro4_rvfi_trap;
      shadow4_pc_trap = ro4_rvfi_trap;
      shadow4_pc_rdata = ro3_rvfi_pc_wdata;
    end
  end

  reg shadow5_pc_valid;
  reg shadow5_pc_trap;
  reg [31:0] shadow5_pc_rdata;

  reg [15:0] ro5_errcode_p;

  task ro5_handle_error_p;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 5: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro5_rvfi_valid);
      $display("rvfi_order = %x", ro5_rvfi_order);
      $display("rvfi_insn = %x", ro5_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro5_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro5_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro5_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro5_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro5_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro5_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro5_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro5_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro5_rvfi_intr);
      $display("rvfi_trap = %x", ro5_rvfi_trap);
      $display("shadow_pc_valid = %x", shadow5_pc_valid);
      $display("shadow_pc_rdata = %x", shadow5_pc_rdata);
      ro5_errcode_p <= code;
    end
  endtask

  always @* begin
    shadow5_pc_valid = shadow_pc_valid;
    shadow5_pc_trap = shadow_pc_trap;
    shadow5_pc_rdata = shadow_pc;
    if (!reset && ro0_rvfi_valid) begin
      shadow5_pc_valid = !ro5_rvfi_trap;
      shadow5_pc_trap = ro5_rvfi_trap;
      shadow5_pc_rdata = ro0_rvfi_pc_wdata;
    end
    if (!reset && ro1_rvfi_valid) begin
      shadow5_pc_valid = !ro5_rvfi_trap;
      shadow5_pc_trap = ro5_rvfi_trap;
      shadow5_pc_rdata = ro1_rvfi_pc_wdata;
    end
    if (!reset && ro2_rvfi_valid) begin
      shadow5_pc_valid = !ro5_rvfi_trap;
      shadow5_pc_trap = ro5_rvfi_trap;
      shadow5_pc_rdata = ro2_rvfi_pc_wdata;
    end
    if (!reset && ro3_rvfi_valid) begin
      shadow5_pc_valid = !ro5_rvfi_trap;
      shadow5_pc_trap = ro5_rvfi_trap;
      shadow5_pc_rdata = ro3_rvfi_pc_wdata;
    end
    if (!reset && ro4_rvfi_valid) begin
      shadow5_pc_valid = !ro5_rvfi_trap;
      shadow5_pc_trap = ro5_rvfi_trap;
      shadow5_pc_rdata = ro4_rvfi_pc_wdata;
    end
  end

  reg shadow6_pc_valid;
  reg shadow6_pc_trap;
  reg [31:0] shadow6_pc_rdata;

  reg [15:0] ro6_errcode_p;

  task ro6_handle_error_p;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 6: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro6_rvfi_valid);
      $display("rvfi_order = %x", ro6_rvfi_order);
      $display("rvfi_insn = %x", ro6_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro6_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro6_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro6_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro6_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro6_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro6_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro6_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro6_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro6_rvfi_intr);
      $display("rvfi_trap = %x", ro6_rvfi_trap);
      $display("shadow_pc_valid = %x", shadow6_pc_valid);
      $display("shadow_pc_rdata = %x", shadow6_pc_rdata);
      ro6_errcode_p <= code;
    end
  endtask

  always @* begin
    shadow6_pc_valid = shadow_pc_valid;
    shadow6_pc_trap = shadow_pc_trap;
    shadow6_pc_rdata = shadow_pc;
    if (!reset && ro0_rvfi_valid) begin
      shadow6_pc_valid = !ro6_rvfi_trap;
      shadow6_pc_trap = ro6_rvfi_trap;
      shadow6_pc_rdata = ro0_rvfi_pc_wdata;
    end
    if (!reset && ro1_rvfi_valid) begin
      shadow6_pc_valid = !ro6_rvfi_trap;
      shadow6_pc_trap = ro6_rvfi_trap;
      shadow6_pc_rdata = ro1_rvfi_pc_wdata;
    end
    if (!reset && ro2_rvfi_valid) begin
      shadow6_pc_valid = !ro6_rvfi_trap;
      shadow6_pc_trap = ro6_rvfi_trap;
      shadow6_pc_rdata = ro2_rvfi_pc_wdata;
    end
    if (!reset && ro3_rvfi_valid) begin
      shadow6_pc_valid = !ro6_rvfi_trap;
      shadow6_pc_trap = ro6_rvfi_trap;
      shadow6_pc_rdata = ro3_rvfi_pc_wdata;
    end
    if (!reset && ro4_rvfi_valid) begin
      shadow6_pc_valid = !ro6_rvfi_trap;
      shadow6_pc_trap = ro6_rvfi_trap;
      shadow6_pc_rdata = ro4_rvfi_pc_wdata;
    end
    if (!reset && ro5_rvfi_valid) begin
      shadow6_pc_valid = !ro6_rvfi_trap;
      shadow6_pc_trap = ro6_rvfi_trap;
      shadow6_pc_rdata = ro5_rvfi_pc_wdata;
    end
  end

  reg shadow7_pc_valid;
  reg shadow7_pc_trap;
  reg [31:0] shadow7_pc_rdata;

  reg [15:0] ro7_errcode_p;

  task ro7_handle_error_p;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 7: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro7_rvfi_valid);
      $display("rvfi_order = %x", ro7_rvfi_order);
      $display("rvfi_insn = %x", ro7_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro7_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro7_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro7_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro7_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro7_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro7_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro7_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro7_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro7_rvfi_intr);
      $display("rvfi_trap = %x", ro7_rvfi_trap);
      $display("shadow_pc_valid = %x", shadow7_pc_valid);
      $display("shadow_pc_rdata = %x", shadow7_pc_rdata);
      ro7_errcode_p <= code;
    end
  endtask

  always @* begin
    shadow7_pc_valid = shadow_pc_valid;
    shadow7_pc_trap = shadow_pc_trap;
    shadow7_pc_rdata = shadow_pc;
    if (!reset && ro0_rvfi_valid) begin
      shadow7_pc_valid = !ro7_rvfi_trap;
      shadow7_pc_trap = ro7_rvfi_trap;
      shadow7_pc_rdata = ro0_rvfi_pc_wdata;
    end
    if (!reset && ro1_rvfi_valid) begin
      shadow7_pc_valid = !ro7_rvfi_trap;
      shadow7_pc_trap = ro7_rvfi_trap;
      shadow7_pc_rdata = ro1_rvfi_pc_wdata;
    end
    if (!reset && ro2_rvfi_valid) begin
      shadow7_pc_valid = !ro7_rvfi_trap;
      shadow7_pc_trap = ro7_rvfi_trap;
      shadow7_pc_rdata = ro2_rvfi_pc_wdata;
    end
    if (!reset && ro3_rvfi_valid) begin
      shadow7_pc_valid = !ro7_rvfi_trap;
      shadow7_pc_trap = ro7_rvfi_trap;
      shadow7_pc_rdata = ro3_rvfi_pc_wdata;
    end
    if (!reset && ro4_rvfi_valid) begin
      shadow7_pc_valid = !ro7_rvfi_trap;
      shadow7_pc_trap = ro7_rvfi_trap;
      shadow7_pc_rdata = ro4_rvfi_pc_wdata;
    end
    if (!reset && ro5_rvfi_valid) begin
      shadow7_pc_valid = !ro7_rvfi_trap;
      shadow7_pc_trap = ro7_rvfi_trap;
      shadow7_pc_rdata = ro5_rvfi_pc_wdata;
    end
    if (!reset && ro6_rvfi_valid) begin
      shadow7_pc_valid = !ro7_rvfi_trap;
      shadow7_pc_trap = ro7_rvfi_trap;
      shadow7_pc_rdata = ro6_rvfi_pc_wdata;
    end
  end

  always @(posedge clock) begin
    ro0_errcode_p <= 0;
    ro1_errcode_p <= 0;
    ro2_errcode_p <= 0;
    ro3_errcode_p <= 0;
    ro4_errcode_p <= 0;
    ro5_errcode_p <= 0;
    ro6_errcode_p <= 0;
    ro7_errcode_p <= 0;
    if (reset) begin
      shadow_pc_valid <= 0;
      shadow_pc_trap <= 0;
    end
    if (!reset && ro0_rvfi_valid) begin
      if (shadow0_pc_valid && shadow0_pc_rdata != ro0_rvfi_pc_rdata && !ro0_rvfi_intr) begin
        ro0_handle_error_p(130, "mismatch with shadow pc");
      end
      if (shadow0_pc_valid && shadow0_pc_trap && !ro0_rvfi_intr) begin
        ro0_handle_error_p(133, "expected intr after trap");
      end
      shadow_pc_valid <= !ro0_rvfi_trap;
      shadow_pc_trap <= ro0_rvfi_trap;
      shadow_pc <= ro0_rvfi_pc_wdata;
    end
    if (!reset && ro1_rvfi_valid) begin
      if (shadow1_pc_valid && shadow1_pc_rdata != ro1_rvfi_pc_rdata && !ro1_rvfi_intr) begin
        ro1_handle_error_p(230, "mismatch with shadow pc");
      end
      if (shadow1_pc_valid && shadow1_pc_trap && !ro1_rvfi_intr) begin
        ro1_handle_error_p(233, "expected intr after trap");
      end
      shadow_pc_valid <= !ro1_rvfi_trap;
      shadow_pc_trap <= ro1_rvfi_trap;
      shadow_pc <= ro1_rvfi_pc_wdata;
    end
    if (!reset && ro2_rvfi_valid) begin
      if (shadow2_pc_valid && shadow2_pc_rdata != ro2_rvfi_pc_rdata && !ro2_rvfi_intr) begin
        ro2_handle_error_p(330, "mismatch with shadow pc");
      end
      if (shadow2_pc_valid && shadow2_pc_trap && !ro2_rvfi_intr) begin
        ro2_handle_error_p(333, "expected intr after trap");
      end
      shadow_pc_valid <= !ro2_rvfi_trap;
      shadow_pc_trap <= ro2_rvfi_trap;
      shadow_pc <= ro2_rvfi_pc_wdata;
    end
    if (!reset && ro3_rvfi_valid) begin
      if (shadow3_pc_valid && shadow3_pc_rdata != ro3_rvfi_pc_rdata && !ro3_rvfi_intr) begin
        ro3_handle_error_p(430, "mismatch with shadow pc");
      end
      if (shadow3_pc_valid && shadow3_pc_trap && !ro3_rvfi_intr) begin
        ro3_handle_error_p(433, "expected intr after trap");
      end
      shadow_pc_valid <= !ro3_rvfi_trap;
      shadow_pc_trap <= ro3_rvfi_trap;
      shadow_pc <= ro3_rvfi_pc_wdata;
    end
    if (!reset && ro4_rvfi_valid) begin
      if (shadow4_pc_valid && shadow4_pc_rdata != ro4_rvfi_pc_rdata && !ro4_rvfi_intr) begin
        ro4_handle_error_p(530, "mismatch with shadow pc");
      end
      if (shadow4_pc_valid && shadow4_pc_trap && !ro4_rvfi_intr) begin
        ro4_handle_error_p(533, "expected intr after trap");
      end
      shadow_pc_valid <= !ro4_rvfi_trap;
      shadow_pc_trap <= ro4_rvfi_trap;
      shadow_pc <= ro4_rvfi_pc_wdata;
    end
    if (!reset && ro5_rvfi_valid) begin
      if (shadow5_pc_valid && shadow5_pc_rdata != ro5_rvfi_pc_rdata && !ro5_rvfi_intr) begin
        ro5_handle_error_p(630, "mismatch with shadow pc");
      end
      if (shadow5_pc_valid && shadow5_pc_trap && !ro5_rvfi_intr) begin
        ro5_handle_error_p(633, "expected intr after trap");
      end
      shadow_pc_valid <= !ro5_rvfi_trap;
      shadow_pc_trap <= ro5_rvfi_trap;
      shadow_pc <= ro5_rvfi_pc_wdata;
    end
    if (!reset && ro6_rvfi_valid) begin
      if (shadow6_pc_valid && shadow6_pc_rdata != ro6_rvfi_pc_rdata && !ro6_rvfi_intr) begin
        ro6_handle_error_p(730, "mismatch with shadow pc");
      end
      if (shadow6_pc_valid && shadow6_pc_trap && !ro6_rvfi_intr) begin
        ro6_handle_error_p(733, "expected intr after trap");
      end
      shadow_pc_valid <= !ro6_rvfi_trap;
      shadow_pc_trap <= ro6_rvfi_trap;
      shadow_pc <= ro6_rvfi_pc_wdata;
    end
    if (!reset && ro7_rvfi_valid) begin
      if (shadow7_pc_valid && shadow7_pc_rdata != ro7_rvfi_pc_rdata && !ro7_rvfi_intr) begin
        ro7_handle_error_p(830, "mismatch with shadow pc");
      end
      if (shadow7_pc_valid && shadow7_pc_trap && !ro7_rvfi_intr) begin
        ro7_handle_error_p(833, "expected intr after trap");
      end
      shadow_pc_valid <= !ro7_rvfi_trap;
      shadow_pc_trap <= ro7_rvfi_trap;
      shadow_pc <= ro7_rvfi_pc_wdata;
    end
  end

  reg [31:0] shadow_xregs_valid;
  reg [31:0] shadow_xregs [0:31];

  reg shadow0_rs1_valid;
  reg shadow0_rs2_valid;
  reg [31:0] shadow0_rs1_rdata;
  reg [31:0] shadow0_rs2_rdata;

  reg [15:0] ro0_errcode_r;

  task ro0_handle_error_r;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 0: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro0_rvfi_valid);
      $display("rvfi_order = %x", ro0_rvfi_order);
      $display("rvfi_insn = %x", ro0_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro0_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro0_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro0_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro0_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro0_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro0_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro0_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro0_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro0_rvfi_intr);
      $display("rvfi_trap = %x", ro0_rvfi_trap);
      $display("shadow_rs1_valid = %x", shadow0_rs1_valid);
      $display("shadow_rs1_rdata = %x", shadow0_rs1_rdata);
      $display("shadow_rs2_valid = %x", shadow0_rs2_valid);
      $display("shadow_rs2_rdata = %x", shadow0_rs2_rdata);
      ro0_errcode_r <= code;
    end
  endtask

  always @* begin
    shadow0_rs1_valid = 0;
    shadow0_rs1_rdata = 0;
    if (!reset && ro0_rvfi_valid) begin
      shadow0_rs1_valid = shadow_xregs_valid[ro0_rvfi_rs1_addr];
      shadow0_rs1_rdata = shadow_xregs[ro0_rvfi_rs1_addr];
    end
  end

  always @* begin
    shadow0_rs2_valid = 0;
    shadow0_rs2_rdata = 0;
    if (!reset && ro0_rvfi_valid) begin
      shadow0_rs2_valid = shadow_xregs_valid[ro0_rvfi_rs2_addr];
      shadow0_rs2_rdata = shadow_xregs[ro0_rvfi_rs2_addr];
    end
  end

  reg shadow1_rs1_valid;
  reg shadow1_rs2_valid;
  reg [31:0] shadow1_rs1_rdata;
  reg [31:0] shadow1_rs2_rdata;

  reg [15:0] ro1_errcode_r;

  task ro1_handle_error_r;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 1: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro1_rvfi_valid);
      $display("rvfi_order = %x", ro1_rvfi_order);
      $display("rvfi_insn = %x", ro1_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro1_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro1_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro1_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro1_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro1_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro1_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro1_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro1_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro1_rvfi_intr);
      $display("rvfi_trap = %x", ro1_rvfi_trap);
      $display("shadow_rs1_valid = %x", shadow1_rs1_valid);
      $display("shadow_rs1_rdata = %x", shadow1_rs1_rdata);
      $display("shadow_rs2_valid = %x", shadow1_rs2_valid);
      $display("shadow_rs2_rdata = %x", shadow1_rs2_rdata);
      ro1_errcode_r <= code;
    end
  endtask

  always @* begin
    shadow1_rs1_valid = 0;
    shadow1_rs1_rdata = 0;
    if (!reset && ro1_rvfi_valid) begin
      shadow1_rs1_valid = shadow_xregs_valid[ro1_rvfi_rs1_addr];
      shadow1_rs1_rdata = shadow_xregs[ro1_rvfi_rs1_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro1_rvfi_rs1_addr) begin
        shadow1_rs1_valid = 1;
        shadow1_rs1_rdata = ro0_rvfi_rd_wdata;
      end
    end
  end

  always @* begin
    shadow1_rs2_valid = 0;
    shadow1_rs2_rdata = 0;
    if (!reset && ro1_rvfi_valid) begin
      shadow1_rs2_valid = shadow_xregs_valid[ro1_rvfi_rs2_addr];
      shadow1_rs2_rdata = shadow_xregs[ro1_rvfi_rs2_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro1_rvfi_rs2_addr) begin
        shadow1_rs2_valid = 1;
        shadow1_rs2_rdata = ro0_rvfi_rd_wdata;
      end
    end
  end

  reg shadow2_rs1_valid;
  reg shadow2_rs2_valid;
  reg [31:0] shadow2_rs1_rdata;
  reg [31:0] shadow2_rs2_rdata;

  reg [15:0] ro2_errcode_r;

  task ro2_handle_error_r;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 2: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro2_rvfi_valid);
      $display("rvfi_order = %x", ro2_rvfi_order);
      $display("rvfi_insn = %x", ro2_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro2_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro2_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro2_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro2_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro2_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro2_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro2_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro2_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro2_rvfi_intr);
      $display("rvfi_trap = %x", ro2_rvfi_trap);
      $display("shadow_rs1_valid = %x", shadow2_rs1_valid);
      $display("shadow_rs1_rdata = %x", shadow2_rs1_rdata);
      $display("shadow_rs2_valid = %x", shadow2_rs2_valid);
      $display("shadow_rs2_rdata = %x", shadow2_rs2_rdata);
      ro2_errcode_r <= code;
    end
  endtask

  always @* begin
    shadow2_rs1_valid = 0;
    shadow2_rs1_rdata = 0;
    if (!reset && ro2_rvfi_valid) begin
      shadow2_rs1_valid = shadow_xregs_valid[ro2_rvfi_rs1_addr];
      shadow2_rs1_rdata = shadow_xregs[ro2_rvfi_rs1_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro2_rvfi_rs1_addr) begin
        shadow2_rs1_valid = 1;
        shadow2_rs1_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro2_rvfi_rs1_addr) begin
        shadow2_rs1_valid = 1;
        shadow2_rs1_rdata = ro1_rvfi_rd_wdata;
      end
    end
  end

  always @* begin
    shadow2_rs2_valid = 0;
    shadow2_rs2_rdata = 0;
    if (!reset && ro2_rvfi_valid) begin
      shadow2_rs2_valid = shadow_xregs_valid[ro2_rvfi_rs2_addr];
      shadow2_rs2_rdata = shadow_xregs[ro2_rvfi_rs2_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro2_rvfi_rs2_addr) begin
        shadow2_rs2_valid = 1;
        shadow2_rs2_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro2_rvfi_rs2_addr) begin
        shadow2_rs2_valid = 1;
        shadow2_rs2_rdata = ro1_rvfi_rd_wdata;
      end
    end
  end

  reg shadow3_rs1_valid;
  reg shadow3_rs2_valid;
  reg [31:0] shadow3_rs1_rdata;
  reg [31:0] shadow3_rs2_rdata;

  reg [15:0] ro3_errcode_r;

  task ro3_handle_error_r;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 3: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro3_rvfi_valid);
      $display("rvfi_order = %x", ro3_rvfi_order);
      $display("rvfi_insn = %x", ro3_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro3_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro3_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro3_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro3_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro3_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro3_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro3_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro3_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro3_rvfi_intr);
      $display("rvfi_trap = %x", ro3_rvfi_trap);
      $display("shadow_rs1_valid = %x", shadow3_rs1_valid);
      $display("shadow_rs1_rdata = %x", shadow3_rs1_rdata);
      $display("shadow_rs2_valid = %x", shadow3_rs2_valid);
      $display("shadow_rs2_rdata = %x", shadow3_rs2_rdata);
      ro3_errcode_r <= code;
    end
  endtask

  always @* begin
    shadow3_rs1_valid = 0;
    shadow3_rs1_rdata = 0;
    if (!reset && ro3_rvfi_valid) begin
      shadow3_rs1_valid = shadow_xregs_valid[ro3_rvfi_rs1_addr];
      shadow3_rs1_rdata = shadow_xregs[ro3_rvfi_rs1_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro3_rvfi_rs1_addr) begin
        shadow3_rs1_valid = 1;
        shadow3_rs1_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro3_rvfi_rs1_addr) begin
        shadow3_rs1_valid = 1;
        shadow3_rs1_rdata = ro1_rvfi_rd_wdata;
      end
      if (ro2_rvfi_valid && ro2_rvfi_rd_addr == ro3_rvfi_rs1_addr) begin
        shadow3_rs1_valid = 1;
        shadow3_rs1_rdata = ro2_rvfi_rd_wdata;
      end
    end
  end

  always @* begin
    shadow3_rs2_valid = 0;
    shadow3_rs2_rdata = 0;
    if (!reset && ro3_rvfi_valid) begin
      shadow3_rs2_valid = shadow_xregs_valid[ro3_rvfi_rs2_addr];
      shadow3_rs2_rdata = shadow_xregs[ro3_rvfi_rs2_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro3_rvfi_rs2_addr) begin
        shadow3_rs2_valid = 1;
        shadow3_rs2_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro3_rvfi_rs2_addr) begin
        shadow3_rs2_valid = 1;
        shadow3_rs2_rdata = ro1_rvfi_rd_wdata;
      end
      if (ro2_rvfi_valid && ro2_rvfi_rd_addr == ro3_rvfi_rs2_addr) begin
        shadow3_rs2_valid = 1;
        shadow3_rs2_rdata = ro2_rvfi_rd_wdata;
      end
    end
  end

  reg shadow4_rs1_valid;
  reg shadow4_rs2_valid;
  reg [31:0] shadow4_rs1_rdata;
  reg [31:0] shadow4_rs2_rdata;

  reg [15:0] ro4_errcode_r;

  task ro4_handle_error_r;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 4: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro4_rvfi_valid);
      $display("rvfi_order = %x", ro4_rvfi_order);
      $display("rvfi_insn = %x", ro4_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro4_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro4_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro4_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro4_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro4_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro4_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro4_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro4_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro4_rvfi_intr);
      $display("rvfi_trap = %x", ro4_rvfi_trap);
      $display("shadow_rs1_valid = %x", shadow4_rs1_valid);
      $display("shadow_rs1_rdata = %x", shadow4_rs1_rdata);
      $display("shadow_rs2_valid = %x", shadow4_rs2_valid);
      $display("shadow_rs2_rdata = %x", shadow4_rs2_rdata);
      ro4_errcode_r <= code;
    end
  endtask

  always @* begin
    shadow4_rs1_valid = 0;
    shadow4_rs1_rdata = 0;
    if (!reset && ro4_rvfi_valid) begin
      shadow4_rs1_valid = shadow_xregs_valid[ro4_rvfi_rs1_addr];
      shadow4_rs1_rdata = shadow_xregs[ro4_rvfi_rs1_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro4_rvfi_rs1_addr) begin
        shadow4_rs1_valid = 1;
        shadow4_rs1_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro4_rvfi_rs1_addr) begin
        shadow4_rs1_valid = 1;
        shadow4_rs1_rdata = ro1_rvfi_rd_wdata;
      end
      if (ro2_rvfi_valid && ro2_rvfi_rd_addr == ro4_rvfi_rs1_addr) begin
        shadow4_rs1_valid = 1;
        shadow4_rs1_rdata = ro2_rvfi_rd_wdata;
      end
      if (ro3_rvfi_valid && ro3_rvfi_rd_addr == ro4_rvfi_rs1_addr) begin
        shadow4_rs1_valid = 1;
        shadow4_rs1_rdata = ro3_rvfi_rd_wdata;
      end
    end
  end

  always @* begin
    shadow4_rs2_valid = 0;
    shadow4_rs2_rdata = 0;
    if (!reset && ro4_rvfi_valid) begin
      shadow4_rs2_valid = shadow_xregs_valid[ro4_rvfi_rs2_addr];
      shadow4_rs2_rdata = shadow_xregs[ro4_rvfi_rs2_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro4_rvfi_rs2_addr) begin
        shadow4_rs2_valid = 1;
        shadow4_rs2_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro4_rvfi_rs2_addr) begin
        shadow4_rs2_valid = 1;
        shadow4_rs2_rdata = ro1_rvfi_rd_wdata;
      end
      if (ro2_rvfi_valid && ro2_rvfi_rd_addr == ro4_rvfi_rs2_addr) begin
        shadow4_rs2_valid = 1;
        shadow4_rs2_rdata = ro2_rvfi_rd_wdata;
      end
      if (ro3_rvfi_valid && ro3_rvfi_rd_addr == ro4_rvfi_rs2_addr) begin
        shadow4_rs2_valid = 1;
        shadow4_rs2_rdata = ro3_rvfi_rd_wdata;
      end
    end
  end

  reg shadow5_rs1_valid;
  reg shadow5_rs2_valid;
  reg [31:0] shadow5_rs1_rdata;
  reg [31:0] shadow5_rs2_rdata;

  reg [15:0] ro5_errcode_r;

  task ro5_handle_error_r;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 5: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro5_rvfi_valid);
      $display("rvfi_order = %x", ro5_rvfi_order);
      $display("rvfi_insn = %x", ro5_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro5_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro5_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro5_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro5_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro5_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro5_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro5_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro5_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro5_rvfi_intr);
      $display("rvfi_trap = %x", ro5_rvfi_trap);
      $display("shadow_rs1_valid = %x", shadow5_rs1_valid);
      $display("shadow_rs1_rdata = %x", shadow5_rs1_rdata);
      $display("shadow_rs2_valid = %x", shadow5_rs2_valid);
      $display("shadow_rs2_rdata = %x", shadow5_rs2_rdata);
      ro5_errcode_r <= code;
    end
  endtask

  always @* begin
    shadow5_rs1_valid = 0;
    shadow5_rs1_rdata = 0;
    if (!reset && ro5_rvfi_valid) begin
      shadow5_rs1_valid = shadow_xregs_valid[ro5_rvfi_rs1_addr];
      shadow5_rs1_rdata = shadow_xregs[ro5_rvfi_rs1_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro5_rvfi_rs1_addr) begin
        shadow5_rs1_valid = 1;
        shadow5_rs1_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro5_rvfi_rs1_addr) begin
        shadow5_rs1_valid = 1;
        shadow5_rs1_rdata = ro1_rvfi_rd_wdata;
      end
      if (ro2_rvfi_valid && ro2_rvfi_rd_addr == ro5_rvfi_rs1_addr) begin
        shadow5_rs1_valid = 1;
        shadow5_rs1_rdata = ro2_rvfi_rd_wdata;
      end
      if (ro3_rvfi_valid && ro3_rvfi_rd_addr == ro5_rvfi_rs1_addr) begin
        shadow5_rs1_valid = 1;
        shadow5_rs1_rdata = ro3_rvfi_rd_wdata;
      end
      if (ro4_rvfi_valid && ro4_rvfi_rd_addr == ro5_rvfi_rs1_addr) begin
        shadow5_rs1_valid = 1;
        shadow5_rs1_rdata = ro4_rvfi_rd_wdata;
      end
    end
  end

  always @* begin
    shadow5_rs2_valid = 0;
    shadow5_rs2_rdata = 0;
    if (!reset && ro5_rvfi_valid) begin
      shadow5_rs2_valid = shadow_xregs_valid[ro5_rvfi_rs2_addr];
      shadow5_rs2_rdata = shadow_xregs[ro5_rvfi_rs2_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro5_rvfi_rs2_addr) begin
        shadow5_rs2_valid = 1;
        shadow5_rs2_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro5_rvfi_rs2_addr) begin
        shadow5_rs2_valid = 1;
        shadow5_rs2_rdata = ro1_rvfi_rd_wdata;
      end
      if (ro2_rvfi_valid && ro2_rvfi_rd_addr == ro5_rvfi_rs2_addr) begin
        shadow5_rs2_valid = 1;
        shadow5_rs2_rdata = ro2_rvfi_rd_wdata;
      end
      if (ro3_rvfi_valid && ro3_rvfi_rd_addr == ro5_rvfi_rs2_addr) begin
        shadow5_rs2_valid = 1;
        shadow5_rs2_rdata = ro3_rvfi_rd_wdata;
      end
      if (ro4_rvfi_valid && ro4_rvfi_rd_addr == ro5_rvfi_rs2_addr) begin
        shadow5_rs2_valid = 1;
        shadow5_rs2_rdata = ro4_rvfi_rd_wdata;
      end
    end
  end

  reg shadow6_rs1_valid;
  reg shadow6_rs2_valid;
  reg [31:0] shadow6_rs1_rdata;
  reg [31:0] shadow6_rs2_rdata;

  reg [15:0] ro6_errcode_r;

  task ro6_handle_error_r;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 6: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro6_rvfi_valid);
      $display("rvfi_order = %x", ro6_rvfi_order);
      $display("rvfi_insn = %x", ro6_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro6_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro6_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro6_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro6_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro6_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro6_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro6_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro6_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro6_rvfi_intr);
      $display("rvfi_trap = %x", ro6_rvfi_trap);
      $display("shadow_rs1_valid = %x", shadow6_rs1_valid);
      $display("shadow_rs1_rdata = %x", shadow6_rs1_rdata);
      $display("shadow_rs2_valid = %x", shadow6_rs2_valid);
      $display("shadow_rs2_rdata = %x", shadow6_rs2_rdata);
      ro6_errcode_r <= code;
    end
  endtask

  always @* begin
    shadow6_rs1_valid = 0;
    shadow6_rs1_rdata = 0;
    if (!reset && ro6_rvfi_valid) begin
      shadow6_rs1_valid = shadow_xregs_valid[ro6_rvfi_rs1_addr];
      shadow6_rs1_rdata = shadow_xregs[ro6_rvfi_rs1_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro6_rvfi_rs1_addr) begin
        shadow6_rs1_valid = 1;
        shadow6_rs1_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro6_rvfi_rs1_addr) begin
        shadow6_rs1_valid = 1;
        shadow6_rs1_rdata = ro1_rvfi_rd_wdata;
      end
      if (ro2_rvfi_valid && ro2_rvfi_rd_addr == ro6_rvfi_rs1_addr) begin
        shadow6_rs1_valid = 1;
        shadow6_rs1_rdata = ro2_rvfi_rd_wdata;
      end
      if (ro3_rvfi_valid && ro3_rvfi_rd_addr == ro6_rvfi_rs1_addr) begin
        shadow6_rs1_valid = 1;
        shadow6_rs1_rdata = ro3_rvfi_rd_wdata;
      end
      if (ro4_rvfi_valid && ro4_rvfi_rd_addr == ro6_rvfi_rs1_addr) begin
        shadow6_rs1_valid = 1;
        shadow6_rs1_rdata = ro4_rvfi_rd_wdata;
      end
      if (ro5_rvfi_valid && ro5_rvfi_rd_addr == ro6_rvfi_rs1_addr) begin
        shadow6_rs1_valid = 1;
        shadow6_rs1_rdata = ro5_rvfi_rd_wdata;
      end
    end
  end

  always @* begin
    shadow6_rs2_valid = 0;
    shadow6_rs2_rdata = 0;
    if (!reset && ro6_rvfi_valid) begin
      shadow6_rs2_valid = shadow_xregs_valid[ro6_rvfi_rs2_addr];
      shadow6_rs2_rdata = shadow_xregs[ro6_rvfi_rs2_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro6_rvfi_rs2_addr) begin
        shadow6_rs2_valid = 1;
        shadow6_rs2_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro6_rvfi_rs2_addr) begin
        shadow6_rs2_valid = 1;
        shadow6_rs2_rdata = ro1_rvfi_rd_wdata;
      end
      if (ro2_rvfi_valid && ro2_rvfi_rd_addr == ro6_rvfi_rs2_addr) begin
        shadow6_rs2_valid = 1;
        shadow6_rs2_rdata = ro2_rvfi_rd_wdata;
      end
      if (ro3_rvfi_valid && ro3_rvfi_rd_addr == ro6_rvfi_rs2_addr) begin
        shadow6_rs2_valid = 1;
        shadow6_rs2_rdata = ro3_rvfi_rd_wdata;
      end
      if (ro4_rvfi_valid && ro4_rvfi_rd_addr == ro6_rvfi_rs2_addr) begin
        shadow6_rs2_valid = 1;
        shadow6_rs2_rdata = ro4_rvfi_rd_wdata;
      end
      if (ro5_rvfi_valid && ro5_rvfi_rd_addr == ro6_rvfi_rs2_addr) begin
        shadow6_rs2_valid = 1;
        shadow6_rs2_rdata = ro5_rvfi_rd_wdata;
      end
    end
  end

  reg shadow7_rs1_valid;
  reg shadow7_rs2_valid;
  reg [31:0] shadow7_rs1_rdata;
  reg [31:0] shadow7_rs2_rdata;

  reg [15:0] ro7_errcode_r;

  task ro7_handle_error_r;
    input [15:0] code;
    input [511:0] msg;
    begin
      $display("-------- RVFI Monitor error %0d in reordered channel 7: %m at time %0t --------", code, $time);
      $display("Error message: %0s", msg);
      $display("rvfi_valid = %x", ro7_rvfi_valid);
      $display("rvfi_order = %x", ro7_rvfi_order);
      $display("rvfi_insn = %x", ro7_rvfi_insn);
      $display("rvfi_rs1_addr = %x", ro7_rvfi_rs1_addr);
      $display("rvfi_rs2_addr = %x", ro7_rvfi_rs2_addr);
      $display("rvfi_rs1_rdata = %x", ro7_rvfi_rs1_rdata);
      $display("rvfi_rs2_rdata = %x", ro7_rvfi_rs2_rdata);
      $display("rvfi_rd_addr = %x", ro7_rvfi_rd_addr);
      $display("rvfi_rd_wdata = %x", ro7_rvfi_rd_wdata);
      $display("rvfi_pc_rdata = %x", ro7_rvfi_pc_rdata);
      $display("rvfi_pc_wdata = %x", ro7_rvfi_pc_wdata);
      $display("rvfi_intr = %x", ro7_rvfi_intr);
      $display("rvfi_trap = %x", ro7_rvfi_trap);
      $display("shadow_rs1_valid = %x", shadow7_rs1_valid);
      $display("shadow_rs1_rdata = %x", shadow7_rs1_rdata);
      $display("shadow_rs2_valid = %x", shadow7_rs2_valid);
      $display("shadow_rs2_rdata = %x", shadow7_rs2_rdata);
      ro7_errcode_r <= code;
    end
  endtask

  always @* begin
    shadow7_rs1_valid = 0;
    shadow7_rs1_rdata = 0;
    if (!reset && ro7_rvfi_valid) begin
      shadow7_rs1_valid = shadow_xregs_valid[ro7_rvfi_rs1_addr];
      shadow7_rs1_rdata = shadow_xregs[ro7_rvfi_rs1_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro7_rvfi_rs1_addr) begin
        shadow7_rs1_valid = 1;
        shadow7_rs1_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro7_rvfi_rs1_addr) begin
        shadow7_rs1_valid = 1;
        shadow7_rs1_rdata = ro1_rvfi_rd_wdata;
      end
      if (ro2_rvfi_valid && ro2_rvfi_rd_addr == ro7_rvfi_rs1_addr) begin
        shadow7_rs1_valid = 1;
        shadow7_rs1_rdata = ro2_rvfi_rd_wdata;
      end
      if (ro3_rvfi_valid && ro3_rvfi_rd_addr == ro7_rvfi_rs1_addr) begin
        shadow7_rs1_valid = 1;
        shadow7_rs1_rdata = ro3_rvfi_rd_wdata;
      end
      if (ro4_rvfi_valid && ro4_rvfi_rd_addr == ro7_rvfi_rs1_addr) begin
        shadow7_rs1_valid = 1;
        shadow7_rs1_rdata = ro4_rvfi_rd_wdata;
      end
      if (ro5_rvfi_valid && ro5_rvfi_rd_addr == ro7_rvfi_rs1_addr) begin
        shadow7_rs1_valid = 1;
        shadow7_rs1_rdata = ro5_rvfi_rd_wdata;
      end
      if (ro6_rvfi_valid && ro6_rvfi_rd_addr == ro7_rvfi_rs1_addr) begin
        shadow7_rs1_valid = 1;
        shadow7_rs1_rdata = ro6_rvfi_rd_wdata;
      end
    end
  end

  always @* begin
    shadow7_rs2_valid = 0;
    shadow7_rs2_rdata = 0;
    if (!reset && ro7_rvfi_valid) begin
      shadow7_rs2_valid = shadow_xregs_valid[ro7_rvfi_rs2_addr];
      shadow7_rs2_rdata = shadow_xregs[ro7_rvfi_rs2_addr];
      if (ro0_rvfi_valid && ro0_rvfi_rd_addr == ro7_rvfi_rs2_addr) begin
        shadow7_rs2_valid = 1;
        shadow7_rs2_rdata = ro0_rvfi_rd_wdata;
      end
      if (ro1_rvfi_valid && ro1_rvfi_rd_addr == ro7_rvfi_rs2_addr) begin
        shadow7_rs2_valid = 1;
        shadow7_rs2_rdata = ro1_rvfi_rd_wdata;
      end
      if (ro2_rvfi_valid && ro2_rvfi_rd_addr == ro7_rvfi_rs2_addr) begin
        shadow7_rs2_valid = 1;
        shadow7_rs2_rdata = ro2_rvfi_rd_wdata;
      end
      if (ro3_rvfi_valid && ro3_rvfi_rd_addr == ro7_rvfi_rs2_addr) begin
        shadow7_rs2_valid = 1;
        shadow7_rs2_rdata = ro3_rvfi_rd_wdata;
      end
      if (ro4_rvfi_valid && ro4_rvfi_rd_addr == ro7_rvfi_rs2_addr) begin
        shadow7_rs2_valid = 1;
        shadow7_rs2_rdata = ro4_rvfi_rd_wdata;
      end
      if (ro5_rvfi_valid && ro5_rvfi_rd_addr == ro7_rvfi_rs2_addr) begin
        shadow7_rs2_valid = 1;
        shadow7_rs2_rdata = ro5_rvfi_rd_wdata;
      end
      if (ro6_rvfi_valid && ro6_rvfi_rd_addr == ro7_rvfi_rs2_addr) begin
        shadow7_rs2_valid = 1;
        shadow7_rs2_rdata = ro6_rvfi_rd_wdata;
      end
    end
  end

  always @(posedge clock) begin
    ro0_errcode_r <= 0;
    ro1_errcode_r <= 0;
    ro2_errcode_r <= 0;
    ro3_errcode_r <= 0;
    ro4_errcode_r <= 0;
    ro5_errcode_r <= 0;
    ro6_errcode_r <= 0;
    ro7_errcode_r <= 0;
    if (reset) begin
      shadow_xregs_valid <= 1;
      shadow_xregs[0] <= 0;
    end
    if (!reset && ro0_rvfi_valid) begin
      if (shadow0_rs1_valid && shadow0_rs1_rdata != ro0_rvfi_rs1_rdata) begin
        ro0_handle_error_r(131, "mismatch with shadow rs1");
      end
      if (shadow0_rs2_valid && shadow0_rs2_rdata != ro0_rvfi_rs2_rdata) begin
        ro0_handle_error_r(132, "mismatch with shadow rs2");
      end
      shadow_xregs_valid[ro0_rvfi_rd_addr] <= 1;
      shadow_xregs[ro0_rvfi_rd_addr] <= ro0_rvfi_rd_wdata;
    end
    if (!reset && ro1_rvfi_valid) begin
      if (shadow1_rs1_valid && shadow1_rs1_rdata != ro1_rvfi_rs1_rdata) begin
        ro1_handle_error_r(231, "mismatch with shadow rs1");
      end
      if (shadow1_rs2_valid && shadow1_rs2_rdata != ro1_rvfi_rs2_rdata) begin
        ro1_handle_error_r(232, "mismatch with shadow rs2");
      end
      shadow_xregs_valid[ro1_rvfi_rd_addr] <= 1;
      shadow_xregs[ro1_rvfi_rd_addr] <= ro1_rvfi_rd_wdata;
    end
    if (!reset && ro2_rvfi_valid) begin
      if (shadow2_rs1_valid && shadow2_rs1_rdata != ro2_rvfi_rs1_rdata) begin
        ro2_handle_error_r(331, "mismatch with shadow rs1");
      end
      if (shadow2_rs2_valid && shadow2_rs2_rdata != ro2_rvfi_rs2_rdata) begin
        ro2_handle_error_r(332, "mismatch with shadow rs2");
      end
      shadow_xregs_valid[ro2_rvfi_rd_addr] <= 1;
      shadow_xregs[ro2_rvfi_rd_addr] <= ro2_rvfi_rd_wdata;
    end
    if (!reset && ro3_rvfi_valid) begin
      if (shadow3_rs1_valid && shadow3_rs1_rdata != ro3_rvfi_rs1_rdata) begin
        ro3_handle_error_r(431, "mismatch with shadow rs1");
      end
      if (shadow3_rs2_valid && shadow3_rs2_rdata != ro3_rvfi_rs2_rdata) begin
        ro3_handle_error_r(432, "mismatch with shadow rs2");
      end
      shadow_xregs_valid[ro3_rvfi_rd_addr] <= 1;
      shadow_xregs[ro3_rvfi_rd_addr] <= ro3_rvfi_rd_wdata;
    end
    if (!reset && ro4_rvfi_valid) begin
      if (shadow4_rs1_valid && shadow4_rs1_rdata != ro4_rvfi_rs1_rdata) begin
        ro4_handle_error_r(531, "mismatch with shadow rs1");
      end
      if (shadow4_rs2_valid && shadow4_rs2_rdata != ro4_rvfi_rs2_rdata) begin
        ro4_handle_error_r(532, "mismatch with shadow rs2");
      end
      shadow_xregs_valid[ro4_rvfi_rd_addr] <= 1;
      shadow_xregs[ro4_rvfi_rd_addr] <= ro4_rvfi_rd_wdata;
    end
    if (!reset && ro5_rvfi_valid) begin
      if (shadow5_rs1_valid && shadow5_rs1_rdata != ro5_rvfi_rs1_rdata) begin
        ro5_handle_error_r(631, "mismatch with shadow rs1");
      end
      if (shadow5_rs2_valid && shadow5_rs2_rdata != ro5_rvfi_rs2_rdata) begin
        ro5_handle_error_r(632, "mismatch with shadow rs2");
      end
      shadow_xregs_valid[ro5_rvfi_rd_addr] <= 1;
      shadow_xregs[ro5_rvfi_rd_addr] <= ro5_rvfi_rd_wdata;
    end
    if (!reset && ro6_rvfi_valid) begin
      if (shadow6_rs1_valid && shadow6_rs1_rdata != ro6_rvfi_rs1_rdata) begin
        ro6_handle_error_r(731, "mismatch with shadow rs1");
      end
      if (shadow6_rs2_valid && shadow6_rs2_rdata != ro6_rvfi_rs2_rdata) begin
        ro6_handle_error_r(732, "mismatch with shadow rs2");
      end
      shadow_xregs_valid[ro6_rvfi_rd_addr] <= 1;
      shadow_xregs[ro6_rvfi_rd_addr] <= ro6_rvfi_rd_wdata;
    end
    if (!reset && ro7_rvfi_valid) begin
      if (shadow7_rs1_valid && shadow7_rs1_rdata != ro7_rvfi_rs1_rdata) begin
        ro7_handle_error_r(831, "mismatch with shadow rs1");
      end
      if (shadow7_rs2_valid && shadow7_rs2_rdata != ro7_rvfi_rs2_rdata) begin
        ro7_handle_error_r(832, "mismatch with shadow rs2");
      end
      shadow_xregs_valid[ro7_rvfi_rd_addr] <= 1;
      shadow_xregs[ro7_rvfi_rd_addr] <= ro7_rvfi_rd_wdata;
    end
  end

  always @(posedge clock) begin
    errcode <= 0;
    if (!reset) begin
      if (ch0_errcode) errcode <= ch0_errcode;
      if (ch1_errcode) errcode <= ch1_errcode;
      if (ch2_errcode) errcode <= ch2_errcode;
      if (ch3_errcode) errcode <= ch3_errcode;
      if (ch4_errcode) errcode <= ch4_errcode;
      if (ch5_errcode) errcode <= ch5_errcode;
      if (ch6_errcode) errcode <= ch6_errcode;
      if (ch7_errcode) errcode <= ch7_errcode;
      if (rob_errcode) errcode <= rob_errcode;
      if (ro0_errcode_p) errcode <= ro0_errcode_p;
      if (ro1_errcode_p) errcode <= ro1_errcode_p;
      if (ro2_errcode_p) errcode <= ro2_errcode_p;
      if (ro3_errcode_p) errcode <= ro3_errcode_p;
      if (ro4_errcode_p) errcode <= ro4_errcode_p;
      if (ro5_errcode_p) errcode <= ro5_errcode_p;
      if (ro6_errcode_p) errcode <= ro6_errcode_p;
      if (ro7_errcode_p) errcode <= ro7_errcode_p;
      if (ro0_errcode_r) errcode <= ro0_errcode_r;
      if (ro1_errcode_r) errcode <= ro1_errcode_r;
      if (ro2_errcode_r) errcode <= ro2_errcode_r;
      if (ro3_errcode_r) errcode <= ro3_errcode_r;
      if (ro4_errcode_r) errcode <= ro4_errcode_r;
      if (ro5_errcode_r) errcode <= ro5_errcode_r;
      if (ro6_errcode_r) errcode <= ro6_errcode_r;
      if (ro7_errcode_r) errcode <= ro7_errcode_r;
    end
  end
endmodule

module riscv_formal_monitor_rv32imc_rob (
  input clock,
  input reset,
    input i0_valid,
    input [63:0] i0_order,
    input [208:0] i0_data,
    output reg o0_valid,
    output reg [63:0] o0_order,
    output reg [208:0] o0_data,
    input i1_valid,
    input [63:0] i1_order,
    input [208:0] i1_data,
    output reg o1_valid,
    output reg [63:0] o1_order,
    output reg [208:0] o1_data,
    input i2_valid,
    input [63:0] i2_order,
    input [208:0] i2_data,
    output reg o2_valid,
    output reg [63:0] o2_order,
    output reg [208:0] o2_data,
    input i3_valid,
    input [63:0] i3_order,
    input [208:0] i3_data,
    output reg o3_valid,
    output reg [63:0] o3_order,
    output reg [208:0] o3_data,
    input i4_valid,
    input [63:0] i4_order,
    input [208:0] i4_data,
    output reg o4_valid,
    output reg [63:0] o4_order,
    output reg [208:0] o4_data,
    input i5_valid,
    input [63:0] i5_order,
    input [208:0] i5_data,
    output reg o5_valid,
    output reg [63:0] o5_order,
    output reg [208:0] o5_data,
    input i6_valid,
    input [63:0] i6_order,
    input [208:0] i6_data,
    output reg o6_valid,
    output reg [63:0] o6_order,
    output reg [208:0] o6_data,
    input i7_valid,
    input [63:0] i7_order,
    input [208:0] i7_data,
    output reg o7_valid,
    output reg [63:0] o7_order,
    output reg [208:0] o7_data,
  output reg [15:0] errcode
);
  reg [272:0] buffer [0:15];
  reg [15:0] valid;
  reg [63:0] cursor;
  reg continue_flag;

  always @(posedge clock) begin
    o0_valid <= 0;
    o1_valid <= 0;
    o2_valid <= 0;
    o3_valid <= 0;
    o4_valid <= 0;
    o5_valid <= 0;
    o6_valid <= 0;
    o7_valid <= 0;
    errcode <= 0;
    continue_flag = 1;
    if (reset) begin
      valid <= 0;
      cursor = 0;
    end else begin
      if (i0_valid) begin
        if (valid[i0_order[3:0]])
          errcode <= 60000 + i0_order[7:0];
        buffer[i0_order[3:0]] <= {i0_data, i0_order};
        valid[i0_order[3:0]] <= 1;
      end
      if (i1_valid) begin
        if (valid[i1_order[3:0]])
          errcode <= 60000 + i1_order[7:0];
        buffer[i1_order[3:0]] <= {i1_data, i1_order};
        valid[i1_order[3:0]] <= 1;
      end
      if (i2_valid) begin
        if (valid[i2_order[3:0]])
          errcode <= 60000 + i2_order[7:0];
        buffer[i2_order[3:0]] <= {i2_data, i2_order};
        valid[i2_order[3:0]] <= 1;
      end
      if (i3_valid) begin
        if (valid[i3_order[3:0]])
          errcode <= 60000 + i3_order[7:0];
        buffer[i3_order[3:0]] <= {i3_data, i3_order};
        valid[i3_order[3:0]] <= 1;
      end
      if (i4_valid) begin
        if (valid[i4_order[3:0]])
          errcode <= 60000 + i4_order[7:0];
        buffer[i4_order[3:0]] <= {i4_data, i4_order};
        valid[i4_order[3:0]] <= 1;
      end
      if (i5_valid) begin
        if (valid[i5_order[3:0]])
          errcode <= 60000 + i5_order[7:0];
        buffer[i5_order[3:0]] <= {i5_data, i5_order};
        valid[i5_order[3:0]] <= 1;
      end
      if (i6_valid) begin
        if (valid[i6_order[3:0]])
          errcode <= 60000 + i6_order[7:0];
        buffer[i6_order[3:0]] <= {i6_data, i6_order};
        valid[i6_order[3:0]] <= 1;
      end
      if (i7_valid) begin
        if (valid[i7_order[3:0]])
          errcode <= 60000 + i7_order[7:0];
        buffer[i7_order[3:0]] <= {i7_data, i7_order};
        valid[i7_order[3:0]] <= 1;
      end
      if (continue_flag && valid[cursor[3:0]]) begin
        if (buffer[cursor[3:0]][63:0] != cursor)
          errcode <= 61000 + cursor[7:0];
        o0_valid <= 1;
        o0_order <= buffer[cursor[3:0]][63:0];
        o0_data <= buffer[cursor[3:0]][272:64];
        valid[cursor[3:0]] <= 0;
        cursor = cursor + 1;
      end else begin
        continue_flag = 0;
      end
      if (continue_flag && valid[cursor[3:0]]) begin
        if (buffer[cursor[3:0]][63:0] != cursor)
          errcode <= 61000 + cursor[7:0];
        o1_valid <= 1;
        o1_order <= buffer[cursor[3:0]][63:0];
        o1_data <= buffer[cursor[3:0]][272:64];
        valid[cursor[3:0]] <= 0;
        cursor = cursor + 1;
      end else begin
        continue_flag = 0;
      end
      if (continue_flag && valid[cursor[3:0]]) begin
        if (buffer[cursor[3:0]][63:0] != cursor)
          errcode <= 61000 + cursor[7:0];
        o2_valid <= 1;
        o2_order <= buffer[cursor[3:0]][63:0];
        o2_data <= buffer[cursor[3:0]][272:64];
        valid[cursor[3:0]] <= 0;
        cursor = cursor + 1;
      end else begin
        continue_flag = 0;
      end
      if (continue_flag && valid[cursor[3:0]]) begin
        if (buffer[cursor[3:0]][63:0] != cursor)
          errcode <= 61000 + cursor[7:0];
        o3_valid <= 1;
        o3_order <= buffer[cursor[3:0]][63:0];
        o3_data <= buffer[cursor[3:0]][272:64];
        valid[cursor[3:0]] <= 0;
        cursor = cursor + 1;
      end else begin
        continue_flag = 0;
      end
      if (continue_flag && valid[cursor[3:0]]) begin
        if (buffer[cursor[3:0]][63:0] != cursor)
          errcode <= 61000 + cursor[7:0];
        o4_valid <= 1;
        o4_order <= buffer[cursor[3:0]][63:0];
        o4_data <= buffer[cursor[3:0]][272:64];
        valid[cursor[3:0]] <= 0;
        cursor = cursor + 1;
      end else begin
        continue_flag = 0;
      end
      if (continue_flag && valid[cursor[3:0]]) begin
        if (buffer[cursor[3:0]][63:0] != cursor)
          errcode <= 61000 + cursor[7:0];
        o5_valid <= 1;
        o5_order <= buffer[cursor[3:0]][63:0];
        o5_data <= buffer[cursor[3:0]][272:64];
        valid[cursor[3:0]] <= 0;
        cursor = cursor + 1;
      end else begin
        continue_flag = 0;
      end
      if (continue_flag && valid[cursor[3:0]]) begin
        if (buffer[cursor[3:0]][63:0] != cursor)
          errcode <= 61000 + cursor[7:0];
        o6_valid <= 1;
        o6_order <= buffer[cursor[3:0]][63:0];
        o6_data <= buffer[cursor[3:0]][272:64];
        valid[cursor[3:0]] <= 0;
        cursor = cursor + 1;
      end else begin
        continue_flag = 0;
      end
      if (continue_flag && valid[cursor[3:0]]) begin
        if (buffer[cursor[3:0]][63:0] != cursor)
          errcode <= 61000 + cursor[7:0];
        o7_valid <= 1;
        o7_order <= buffer[cursor[3:0]][63:0];
        o7_data <= buffer[cursor[3:0]][272:64];
        valid[cursor[3:0]] <= 0;
        cursor = cursor + 1;
      end else begin
        continue_flag = 0;
      end
    end
  end
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_isa_spec (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);
  wire                                spec_insn_add_valid;
  wire                                spec_insn_add_trap;
  wire [                       4 : 0] spec_insn_add_rs1_addr;
  wire [                       4 : 0] spec_insn_add_rs2_addr;
  wire [                       4 : 0] spec_insn_add_rd_addr;
  wire [32   - 1 : 0] spec_insn_add_rd_wdata;
  wire [32   - 1 : 0] spec_insn_add_pc_wdata;
  wire [32   - 1 : 0] spec_insn_add_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_add_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_add_mem_wmask;
  wire [32   - 1 : 0] spec_insn_add_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_add insn_add (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_add_valid),
    .spec_trap(spec_insn_add_trap),
    .spec_rs1_addr(spec_insn_add_rs1_addr),
    .spec_rs2_addr(spec_insn_add_rs2_addr),
    .spec_rd_addr(spec_insn_add_rd_addr),
    .spec_rd_wdata(spec_insn_add_rd_wdata),
    .spec_pc_wdata(spec_insn_add_pc_wdata),
    .spec_mem_addr(spec_insn_add_mem_addr),
    .spec_mem_rmask(spec_insn_add_mem_rmask),
    .spec_mem_wmask(spec_insn_add_mem_wmask),
    .spec_mem_wdata(spec_insn_add_mem_wdata)
  );

  wire                                spec_insn_addi_valid;
  wire                                spec_insn_addi_trap;
  wire [                       4 : 0] spec_insn_addi_rs1_addr;
  wire [                       4 : 0] spec_insn_addi_rs2_addr;
  wire [                       4 : 0] spec_insn_addi_rd_addr;
  wire [32   - 1 : 0] spec_insn_addi_rd_wdata;
  wire [32   - 1 : 0] spec_insn_addi_pc_wdata;
  wire [32   - 1 : 0] spec_insn_addi_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_addi_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_addi_mem_wmask;
  wire [32   - 1 : 0] spec_insn_addi_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_addi insn_addi (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_addi_valid),
    .spec_trap(spec_insn_addi_trap),
    .spec_rs1_addr(spec_insn_addi_rs1_addr),
    .spec_rs2_addr(spec_insn_addi_rs2_addr),
    .spec_rd_addr(spec_insn_addi_rd_addr),
    .spec_rd_wdata(spec_insn_addi_rd_wdata),
    .spec_pc_wdata(spec_insn_addi_pc_wdata),
    .spec_mem_addr(spec_insn_addi_mem_addr),
    .spec_mem_rmask(spec_insn_addi_mem_rmask),
    .spec_mem_wmask(spec_insn_addi_mem_wmask),
    .spec_mem_wdata(spec_insn_addi_mem_wdata)
  );

  wire                                spec_insn_and_valid;
  wire                                spec_insn_and_trap;
  wire [                       4 : 0] spec_insn_and_rs1_addr;
  wire [                       4 : 0] spec_insn_and_rs2_addr;
  wire [                       4 : 0] spec_insn_and_rd_addr;
  wire [32   - 1 : 0] spec_insn_and_rd_wdata;
  wire [32   - 1 : 0] spec_insn_and_pc_wdata;
  wire [32   - 1 : 0] spec_insn_and_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_and_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_and_mem_wmask;
  wire [32   - 1 : 0] spec_insn_and_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_and insn_and (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_and_valid),
    .spec_trap(spec_insn_and_trap),
    .spec_rs1_addr(spec_insn_and_rs1_addr),
    .spec_rs2_addr(spec_insn_and_rs2_addr),
    .spec_rd_addr(spec_insn_and_rd_addr),
    .spec_rd_wdata(spec_insn_and_rd_wdata),
    .spec_pc_wdata(spec_insn_and_pc_wdata),
    .spec_mem_addr(spec_insn_and_mem_addr),
    .spec_mem_rmask(spec_insn_and_mem_rmask),
    .spec_mem_wmask(spec_insn_and_mem_wmask),
    .spec_mem_wdata(spec_insn_and_mem_wdata)
  );

  wire                                spec_insn_andi_valid;
  wire                                spec_insn_andi_trap;
  wire [                       4 : 0] spec_insn_andi_rs1_addr;
  wire [                       4 : 0] spec_insn_andi_rs2_addr;
  wire [                       4 : 0] spec_insn_andi_rd_addr;
  wire [32   - 1 : 0] spec_insn_andi_rd_wdata;
  wire [32   - 1 : 0] spec_insn_andi_pc_wdata;
  wire [32   - 1 : 0] spec_insn_andi_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_andi_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_andi_mem_wmask;
  wire [32   - 1 : 0] spec_insn_andi_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_andi insn_andi (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_andi_valid),
    .spec_trap(spec_insn_andi_trap),
    .spec_rs1_addr(spec_insn_andi_rs1_addr),
    .spec_rs2_addr(spec_insn_andi_rs2_addr),
    .spec_rd_addr(spec_insn_andi_rd_addr),
    .spec_rd_wdata(spec_insn_andi_rd_wdata),
    .spec_pc_wdata(spec_insn_andi_pc_wdata),
    .spec_mem_addr(spec_insn_andi_mem_addr),
    .spec_mem_rmask(spec_insn_andi_mem_rmask),
    .spec_mem_wmask(spec_insn_andi_mem_wmask),
    .spec_mem_wdata(spec_insn_andi_mem_wdata)
  );

  wire                                spec_insn_auipc_valid;
  wire                                spec_insn_auipc_trap;
  wire [                       4 : 0] spec_insn_auipc_rs1_addr;
  wire [                       4 : 0] spec_insn_auipc_rs2_addr;
  wire [                       4 : 0] spec_insn_auipc_rd_addr;
  wire [32   - 1 : 0] spec_insn_auipc_rd_wdata;
  wire [32   - 1 : 0] spec_insn_auipc_pc_wdata;
  wire [32   - 1 : 0] spec_insn_auipc_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_auipc_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_auipc_mem_wmask;
  wire [32   - 1 : 0] spec_insn_auipc_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_auipc insn_auipc (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_auipc_valid),
    .spec_trap(spec_insn_auipc_trap),
    .spec_rs1_addr(spec_insn_auipc_rs1_addr),
    .spec_rs2_addr(spec_insn_auipc_rs2_addr),
    .spec_rd_addr(spec_insn_auipc_rd_addr),
    .spec_rd_wdata(spec_insn_auipc_rd_wdata),
    .spec_pc_wdata(spec_insn_auipc_pc_wdata),
    .spec_mem_addr(spec_insn_auipc_mem_addr),
    .spec_mem_rmask(spec_insn_auipc_mem_rmask),
    .spec_mem_wmask(spec_insn_auipc_mem_wmask),
    .spec_mem_wdata(spec_insn_auipc_mem_wdata)
  );

  wire                                spec_insn_beq_valid;
  wire                                spec_insn_beq_trap;
  wire [                       4 : 0] spec_insn_beq_rs1_addr;
  wire [                       4 : 0] spec_insn_beq_rs2_addr;
  wire [                       4 : 0] spec_insn_beq_rd_addr;
  wire [32   - 1 : 0] spec_insn_beq_rd_wdata;
  wire [32   - 1 : 0] spec_insn_beq_pc_wdata;
  wire [32   - 1 : 0] spec_insn_beq_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_beq_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_beq_mem_wmask;
  wire [32   - 1 : 0] spec_insn_beq_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_beq insn_beq (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_beq_valid),
    .spec_trap(spec_insn_beq_trap),
    .spec_rs1_addr(spec_insn_beq_rs1_addr),
    .spec_rs2_addr(spec_insn_beq_rs2_addr),
    .spec_rd_addr(spec_insn_beq_rd_addr),
    .spec_rd_wdata(spec_insn_beq_rd_wdata),
    .spec_pc_wdata(spec_insn_beq_pc_wdata),
    .spec_mem_addr(spec_insn_beq_mem_addr),
    .spec_mem_rmask(spec_insn_beq_mem_rmask),
    .spec_mem_wmask(spec_insn_beq_mem_wmask),
    .spec_mem_wdata(spec_insn_beq_mem_wdata)
  );

  wire                                spec_insn_bge_valid;
  wire                                spec_insn_bge_trap;
  wire [                       4 : 0] spec_insn_bge_rs1_addr;
  wire [                       4 : 0] spec_insn_bge_rs2_addr;
  wire [                       4 : 0] spec_insn_bge_rd_addr;
  wire [32   - 1 : 0] spec_insn_bge_rd_wdata;
  wire [32   - 1 : 0] spec_insn_bge_pc_wdata;
  wire [32   - 1 : 0] spec_insn_bge_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_bge_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_bge_mem_wmask;
  wire [32   - 1 : 0] spec_insn_bge_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_bge insn_bge (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_bge_valid),
    .spec_trap(spec_insn_bge_trap),
    .spec_rs1_addr(spec_insn_bge_rs1_addr),
    .spec_rs2_addr(spec_insn_bge_rs2_addr),
    .spec_rd_addr(spec_insn_bge_rd_addr),
    .spec_rd_wdata(spec_insn_bge_rd_wdata),
    .spec_pc_wdata(spec_insn_bge_pc_wdata),
    .spec_mem_addr(spec_insn_bge_mem_addr),
    .spec_mem_rmask(spec_insn_bge_mem_rmask),
    .spec_mem_wmask(spec_insn_bge_mem_wmask),
    .spec_mem_wdata(spec_insn_bge_mem_wdata)
  );

  wire                                spec_insn_bgeu_valid;
  wire                                spec_insn_bgeu_trap;
  wire [                       4 : 0] spec_insn_bgeu_rs1_addr;
  wire [                       4 : 0] spec_insn_bgeu_rs2_addr;
  wire [                       4 : 0] spec_insn_bgeu_rd_addr;
  wire [32   - 1 : 0] spec_insn_bgeu_rd_wdata;
  wire [32   - 1 : 0] spec_insn_bgeu_pc_wdata;
  wire [32   - 1 : 0] spec_insn_bgeu_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_bgeu_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_bgeu_mem_wmask;
  wire [32   - 1 : 0] spec_insn_bgeu_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_bgeu insn_bgeu (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_bgeu_valid),
    .spec_trap(spec_insn_bgeu_trap),
    .spec_rs1_addr(spec_insn_bgeu_rs1_addr),
    .spec_rs2_addr(spec_insn_bgeu_rs2_addr),
    .spec_rd_addr(spec_insn_bgeu_rd_addr),
    .spec_rd_wdata(spec_insn_bgeu_rd_wdata),
    .spec_pc_wdata(spec_insn_bgeu_pc_wdata),
    .spec_mem_addr(spec_insn_bgeu_mem_addr),
    .spec_mem_rmask(spec_insn_bgeu_mem_rmask),
    .spec_mem_wmask(spec_insn_bgeu_mem_wmask),
    .spec_mem_wdata(spec_insn_bgeu_mem_wdata)
  );

  wire                                spec_insn_blt_valid;
  wire                                spec_insn_blt_trap;
  wire [                       4 : 0] spec_insn_blt_rs1_addr;
  wire [                       4 : 0] spec_insn_blt_rs2_addr;
  wire [                       4 : 0] spec_insn_blt_rd_addr;
  wire [32   - 1 : 0] spec_insn_blt_rd_wdata;
  wire [32   - 1 : 0] spec_insn_blt_pc_wdata;
  wire [32   - 1 : 0] spec_insn_blt_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_blt_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_blt_mem_wmask;
  wire [32   - 1 : 0] spec_insn_blt_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_blt insn_blt (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_blt_valid),
    .spec_trap(spec_insn_blt_trap),
    .spec_rs1_addr(spec_insn_blt_rs1_addr),
    .spec_rs2_addr(spec_insn_blt_rs2_addr),
    .spec_rd_addr(spec_insn_blt_rd_addr),
    .spec_rd_wdata(spec_insn_blt_rd_wdata),
    .spec_pc_wdata(spec_insn_blt_pc_wdata),
    .spec_mem_addr(spec_insn_blt_mem_addr),
    .spec_mem_rmask(spec_insn_blt_mem_rmask),
    .spec_mem_wmask(spec_insn_blt_mem_wmask),
    .spec_mem_wdata(spec_insn_blt_mem_wdata)
  );

  wire                                spec_insn_bltu_valid;
  wire                                spec_insn_bltu_trap;
  wire [                       4 : 0] spec_insn_bltu_rs1_addr;
  wire [                       4 : 0] spec_insn_bltu_rs2_addr;
  wire [                       4 : 0] spec_insn_bltu_rd_addr;
  wire [32   - 1 : 0] spec_insn_bltu_rd_wdata;
  wire [32   - 1 : 0] spec_insn_bltu_pc_wdata;
  wire [32   - 1 : 0] spec_insn_bltu_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_bltu_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_bltu_mem_wmask;
  wire [32   - 1 : 0] spec_insn_bltu_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_bltu insn_bltu (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_bltu_valid),
    .spec_trap(spec_insn_bltu_trap),
    .spec_rs1_addr(spec_insn_bltu_rs1_addr),
    .spec_rs2_addr(spec_insn_bltu_rs2_addr),
    .spec_rd_addr(spec_insn_bltu_rd_addr),
    .spec_rd_wdata(spec_insn_bltu_rd_wdata),
    .spec_pc_wdata(spec_insn_bltu_pc_wdata),
    .spec_mem_addr(spec_insn_bltu_mem_addr),
    .spec_mem_rmask(spec_insn_bltu_mem_rmask),
    .spec_mem_wmask(spec_insn_bltu_mem_wmask),
    .spec_mem_wdata(spec_insn_bltu_mem_wdata)
  );

  wire                                spec_insn_bne_valid;
  wire                                spec_insn_bne_trap;
  wire [                       4 : 0] spec_insn_bne_rs1_addr;
  wire [                       4 : 0] spec_insn_bne_rs2_addr;
  wire [                       4 : 0] spec_insn_bne_rd_addr;
  wire [32   - 1 : 0] spec_insn_bne_rd_wdata;
  wire [32   - 1 : 0] spec_insn_bne_pc_wdata;
  wire [32   - 1 : 0] spec_insn_bne_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_bne_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_bne_mem_wmask;
  wire [32   - 1 : 0] spec_insn_bne_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_bne insn_bne (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_bne_valid),
    .spec_trap(spec_insn_bne_trap),
    .spec_rs1_addr(spec_insn_bne_rs1_addr),
    .spec_rs2_addr(spec_insn_bne_rs2_addr),
    .spec_rd_addr(spec_insn_bne_rd_addr),
    .spec_rd_wdata(spec_insn_bne_rd_wdata),
    .spec_pc_wdata(spec_insn_bne_pc_wdata),
    .spec_mem_addr(spec_insn_bne_mem_addr),
    .spec_mem_rmask(spec_insn_bne_mem_rmask),
    .spec_mem_wmask(spec_insn_bne_mem_wmask),
    .spec_mem_wdata(spec_insn_bne_mem_wdata)
  );

  wire                                spec_insn_c_add_valid;
  wire                                spec_insn_c_add_trap;
  wire [                       4 : 0] spec_insn_c_add_rs1_addr;
  wire [                       4 : 0] spec_insn_c_add_rs2_addr;
  wire [                       4 : 0] spec_insn_c_add_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_add_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_add_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_add_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_add_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_add_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_add_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_add insn_c_add (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_add_valid),
    .spec_trap(spec_insn_c_add_trap),
    .spec_rs1_addr(spec_insn_c_add_rs1_addr),
    .spec_rs2_addr(spec_insn_c_add_rs2_addr),
    .spec_rd_addr(spec_insn_c_add_rd_addr),
    .spec_rd_wdata(spec_insn_c_add_rd_wdata),
    .spec_pc_wdata(spec_insn_c_add_pc_wdata),
    .spec_mem_addr(spec_insn_c_add_mem_addr),
    .spec_mem_rmask(spec_insn_c_add_mem_rmask),
    .spec_mem_wmask(spec_insn_c_add_mem_wmask),
    .spec_mem_wdata(spec_insn_c_add_mem_wdata)
  );

  wire                                spec_insn_c_addi_valid;
  wire                                spec_insn_c_addi_trap;
  wire [                       4 : 0] spec_insn_c_addi_rs1_addr;
  wire [                       4 : 0] spec_insn_c_addi_rs2_addr;
  wire [                       4 : 0] spec_insn_c_addi_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_addi_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_addi_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_addi_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_addi_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_addi_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_addi_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_addi insn_c_addi (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_addi_valid),
    .spec_trap(spec_insn_c_addi_trap),
    .spec_rs1_addr(spec_insn_c_addi_rs1_addr),
    .spec_rs2_addr(spec_insn_c_addi_rs2_addr),
    .spec_rd_addr(spec_insn_c_addi_rd_addr),
    .spec_rd_wdata(spec_insn_c_addi_rd_wdata),
    .spec_pc_wdata(spec_insn_c_addi_pc_wdata),
    .spec_mem_addr(spec_insn_c_addi_mem_addr),
    .spec_mem_rmask(spec_insn_c_addi_mem_rmask),
    .spec_mem_wmask(spec_insn_c_addi_mem_wmask),
    .spec_mem_wdata(spec_insn_c_addi_mem_wdata)
  );

  wire                                spec_insn_c_addi16sp_valid;
  wire                                spec_insn_c_addi16sp_trap;
  wire [                       4 : 0] spec_insn_c_addi16sp_rs1_addr;
  wire [                       4 : 0] spec_insn_c_addi16sp_rs2_addr;
  wire [                       4 : 0] spec_insn_c_addi16sp_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_addi16sp_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_addi16sp_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_addi16sp_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_addi16sp_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_addi16sp_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_addi16sp_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_addi16sp insn_c_addi16sp (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_addi16sp_valid),
    .spec_trap(spec_insn_c_addi16sp_trap),
    .spec_rs1_addr(spec_insn_c_addi16sp_rs1_addr),
    .spec_rs2_addr(spec_insn_c_addi16sp_rs2_addr),
    .spec_rd_addr(spec_insn_c_addi16sp_rd_addr),
    .spec_rd_wdata(spec_insn_c_addi16sp_rd_wdata),
    .spec_pc_wdata(spec_insn_c_addi16sp_pc_wdata),
    .spec_mem_addr(spec_insn_c_addi16sp_mem_addr),
    .spec_mem_rmask(spec_insn_c_addi16sp_mem_rmask),
    .spec_mem_wmask(spec_insn_c_addi16sp_mem_wmask),
    .spec_mem_wdata(spec_insn_c_addi16sp_mem_wdata)
  );

  wire                                spec_insn_c_addi4spn_valid;
  wire                                spec_insn_c_addi4spn_trap;
  wire [                       4 : 0] spec_insn_c_addi4spn_rs1_addr;
  wire [                       4 : 0] spec_insn_c_addi4spn_rs2_addr;
  wire [                       4 : 0] spec_insn_c_addi4spn_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_addi4spn_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_addi4spn_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_addi4spn_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_addi4spn_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_addi4spn_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_addi4spn_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_addi4spn insn_c_addi4spn (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_addi4spn_valid),
    .spec_trap(spec_insn_c_addi4spn_trap),
    .spec_rs1_addr(spec_insn_c_addi4spn_rs1_addr),
    .spec_rs2_addr(spec_insn_c_addi4spn_rs2_addr),
    .spec_rd_addr(spec_insn_c_addi4spn_rd_addr),
    .spec_rd_wdata(spec_insn_c_addi4spn_rd_wdata),
    .spec_pc_wdata(spec_insn_c_addi4spn_pc_wdata),
    .spec_mem_addr(spec_insn_c_addi4spn_mem_addr),
    .spec_mem_rmask(spec_insn_c_addi4spn_mem_rmask),
    .spec_mem_wmask(spec_insn_c_addi4spn_mem_wmask),
    .spec_mem_wdata(spec_insn_c_addi4spn_mem_wdata)
  );

  wire                                spec_insn_c_and_valid;
  wire                                spec_insn_c_and_trap;
  wire [                       4 : 0] spec_insn_c_and_rs1_addr;
  wire [                       4 : 0] spec_insn_c_and_rs2_addr;
  wire [                       4 : 0] spec_insn_c_and_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_and_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_and_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_and_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_and_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_and_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_and_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_and insn_c_and (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_and_valid),
    .spec_trap(spec_insn_c_and_trap),
    .spec_rs1_addr(spec_insn_c_and_rs1_addr),
    .spec_rs2_addr(spec_insn_c_and_rs2_addr),
    .spec_rd_addr(spec_insn_c_and_rd_addr),
    .spec_rd_wdata(spec_insn_c_and_rd_wdata),
    .spec_pc_wdata(spec_insn_c_and_pc_wdata),
    .spec_mem_addr(spec_insn_c_and_mem_addr),
    .spec_mem_rmask(spec_insn_c_and_mem_rmask),
    .spec_mem_wmask(spec_insn_c_and_mem_wmask),
    .spec_mem_wdata(spec_insn_c_and_mem_wdata)
  );

  wire                                spec_insn_c_andi_valid;
  wire                                spec_insn_c_andi_trap;
  wire [                       4 : 0] spec_insn_c_andi_rs1_addr;
  wire [                       4 : 0] spec_insn_c_andi_rs2_addr;
  wire [                       4 : 0] spec_insn_c_andi_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_andi_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_andi_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_andi_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_andi_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_andi_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_andi_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_andi insn_c_andi (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_andi_valid),
    .spec_trap(spec_insn_c_andi_trap),
    .spec_rs1_addr(spec_insn_c_andi_rs1_addr),
    .spec_rs2_addr(spec_insn_c_andi_rs2_addr),
    .spec_rd_addr(spec_insn_c_andi_rd_addr),
    .spec_rd_wdata(spec_insn_c_andi_rd_wdata),
    .spec_pc_wdata(spec_insn_c_andi_pc_wdata),
    .spec_mem_addr(spec_insn_c_andi_mem_addr),
    .spec_mem_rmask(spec_insn_c_andi_mem_rmask),
    .spec_mem_wmask(spec_insn_c_andi_mem_wmask),
    .spec_mem_wdata(spec_insn_c_andi_mem_wdata)
  );

  wire                                spec_insn_c_beqz_valid;
  wire                                spec_insn_c_beqz_trap;
  wire [                       4 : 0] spec_insn_c_beqz_rs1_addr;
  wire [                       4 : 0] spec_insn_c_beqz_rs2_addr;
  wire [                       4 : 0] spec_insn_c_beqz_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_beqz_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_beqz_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_beqz_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_beqz_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_beqz_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_beqz_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_beqz insn_c_beqz (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_beqz_valid),
    .spec_trap(spec_insn_c_beqz_trap),
    .spec_rs1_addr(spec_insn_c_beqz_rs1_addr),
    .spec_rs2_addr(spec_insn_c_beqz_rs2_addr),
    .spec_rd_addr(spec_insn_c_beqz_rd_addr),
    .spec_rd_wdata(spec_insn_c_beqz_rd_wdata),
    .spec_pc_wdata(spec_insn_c_beqz_pc_wdata),
    .spec_mem_addr(spec_insn_c_beqz_mem_addr),
    .spec_mem_rmask(spec_insn_c_beqz_mem_rmask),
    .spec_mem_wmask(spec_insn_c_beqz_mem_wmask),
    .spec_mem_wdata(spec_insn_c_beqz_mem_wdata)
  );

  wire                                spec_insn_c_bnez_valid;
  wire                                spec_insn_c_bnez_trap;
  wire [                       4 : 0] spec_insn_c_bnez_rs1_addr;
  wire [                       4 : 0] spec_insn_c_bnez_rs2_addr;
  wire [                       4 : 0] spec_insn_c_bnez_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_bnez_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_bnez_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_bnez_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_bnez_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_bnez_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_bnez_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_bnez insn_c_bnez (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_bnez_valid),
    .spec_trap(spec_insn_c_bnez_trap),
    .spec_rs1_addr(spec_insn_c_bnez_rs1_addr),
    .spec_rs2_addr(spec_insn_c_bnez_rs2_addr),
    .spec_rd_addr(spec_insn_c_bnez_rd_addr),
    .spec_rd_wdata(spec_insn_c_bnez_rd_wdata),
    .spec_pc_wdata(spec_insn_c_bnez_pc_wdata),
    .spec_mem_addr(spec_insn_c_bnez_mem_addr),
    .spec_mem_rmask(spec_insn_c_bnez_mem_rmask),
    .spec_mem_wmask(spec_insn_c_bnez_mem_wmask),
    .spec_mem_wdata(spec_insn_c_bnez_mem_wdata)
  );

  wire                                spec_insn_c_j_valid;
  wire                                spec_insn_c_j_trap;
  wire [                       4 : 0] spec_insn_c_j_rs1_addr;
  wire [                       4 : 0] spec_insn_c_j_rs2_addr;
  wire [                       4 : 0] spec_insn_c_j_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_j_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_j_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_j_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_j_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_j_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_j_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_j insn_c_j (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_j_valid),
    .spec_trap(spec_insn_c_j_trap),
    .spec_rs1_addr(spec_insn_c_j_rs1_addr),
    .spec_rs2_addr(spec_insn_c_j_rs2_addr),
    .spec_rd_addr(spec_insn_c_j_rd_addr),
    .spec_rd_wdata(spec_insn_c_j_rd_wdata),
    .spec_pc_wdata(spec_insn_c_j_pc_wdata),
    .spec_mem_addr(spec_insn_c_j_mem_addr),
    .spec_mem_rmask(spec_insn_c_j_mem_rmask),
    .spec_mem_wmask(spec_insn_c_j_mem_wmask),
    .spec_mem_wdata(spec_insn_c_j_mem_wdata)
  );

  wire                                spec_insn_c_jal_valid;
  wire                                spec_insn_c_jal_trap;
  wire [                       4 : 0] spec_insn_c_jal_rs1_addr;
  wire [                       4 : 0] spec_insn_c_jal_rs2_addr;
  wire [                       4 : 0] spec_insn_c_jal_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_jal_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_jal_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_jal_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_jal_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_jal_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_jal_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_jal insn_c_jal (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_jal_valid),
    .spec_trap(spec_insn_c_jal_trap),
    .spec_rs1_addr(spec_insn_c_jal_rs1_addr),
    .spec_rs2_addr(spec_insn_c_jal_rs2_addr),
    .spec_rd_addr(spec_insn_c_jal_rd_addr),
    .spec_rd_wdata(spec_insn_c_jal_rd_wdata),
    .spec_pc_wdata(spec_insn_c_jal_pc_wdata),
    .spec_mem_addr(spec_insn_c_jal_mem_addr),
    .spec_mem_rmask(spec_insn_c_jal_mem_rmask),
    .spec_mem_wmask(spec_insn_c_jal_mem_wmask),
    .spec_mem_wdata(spec_insn_c_jal_mem_wdata)
  );

  wire                                spec_insn_c_jalr_valid;
  wire                                spec_insn_c_jalr_trap;
  wire [                       4 : 0] spec_insn_c_jalr_rs1_addr;
  wire [                       4 : 0] spec_insn_c_jalr_rs2_addr;
  wire [                       4 : 0] spec_insn_c_jalr_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_jalr_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_jalr_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_jalr_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_jalr_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_jalr_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_jalr_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_jalr insn_c_jalr (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_jalr_valid),
    .spec_trap(spec_insn_c_jalr_trap),
    .spec_rs1_addr(spec_insn_c_jalr_rs1_addr),
    .spec_rs2_addr(spec_insn_c_jalr_rs2_addr),
    .spec_rd_addr(spec_insn_c_jalr_rd_addr),
    .spec_rd_wdata(spec_insn_c_jalr_rd_wdata),
    .spec_pc_wdata(spec_insn_c_jalr_pc_wdata),
    .spec_mem_addr(spec_insn_c_jalr_mem_addr),
    .spec_mem_rmask(spec_insn_c_jalr_mem_rmask),
    .spec_mem_wmask(spec_insn_c_jalr_mem_wmask),
    .spec_mem_wdata(spec_insn_c_jalr_mem_wdata)
  );

  wire                                spec_insn_c_jr_valid;
  wire                                spec_insn_c_jr_trap;
  wire [                       4 : 0] spec_insn_c_jr_rs1_addr;
  wire [                       4 : 0] spec_insn_c_jr_rs2_addr;
  wire [                       4 : 0] spec_insn_c_jr_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_jr_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_jr_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_jr_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_jr_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_jr_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_jr_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_jr insn_c_jr (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_jr_valid),
    .spec_trap(spec_insn_c_jr_trap),
    .spec_rs1_addr(spec_insn_c_jr_rs1_addr),
    .spec_rs2_addr(spec_insn_c_jr_rs2_addr),
    .spec_rd_addr(spec_insn_c_jr_rd_addr),
    .spec_rd_wdata(spec_insn_c_jr_rd_wdata),
    .spec_pc_wdata(spec_insn_c_jr_pc_wdata),
    .spec_mem_addr(spec_insn_c_jr_mem_addr),
    .spec_mem_rmask(spec_insn_c_jr_mem_rmask),
    .spec_mem_wmask(spec_insn_c_jr_mem_wmask),
    .spec_mem_wdata(spec_insn_c_jr_mem_wdata)
  );

  wire                                spec_insn_c_li_valid;
  wire                                spec_insn_c_li_trap;
  wire [                       4 : 0] spec_insn_c_li_rs1_addr;
  wire [                       4 : 0] spec_insn_c_li_rs2_addr;
  wire [                       4 : 0] spec_insn_c_li_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_li_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_li_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_li_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_li_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_li_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_li_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_li insn_c_li (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_li_valid),
    .spec_trap(spec_insn_c_li_trap),
    .spec_rs1_addr(spec_insn_c_li_rs1_addr),
    .spec_rs2_addr(spec_insn_c_li_rs2_addr),
    .spec_rd_addr(spec_insn_c_li_rd_addr),
    .spec_rd_wdata(spec_insn_c_li_rd_wdata),
    .spec_pc_wdata(spec_insn_c_li_pc_wdata),
    .spec_mem_addr(spec_insn_c_li_mem_addr),
    .spec_mem_rmask(spec_insn_c_li_mem_rmask),
    .spec_mem_wmask(spec_insn_c_li_mem_wmask),
    .spec_mem_wdata(spec_insn_c_li_mem_wdata)
  );

  wire                                spec_insn_c_lui_valid;
  wire                                spec_insn_c_lui_trap;
  wire [                       4 : 0] spec_insn_c_lui_rs1_addr;
  wire [                       4 : 0] spec_insn_c_lui_rs2_addr;
  wire [                       4 : 0] spec_insn_c_lui_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_lui_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_lui_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_lui_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_lui_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_lui_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_lui_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_lui insn_c_lui (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_lui_valid),
    .spec_trap(spec_insn_c_lui_trap),
    .spec_rs1_addr(spec_insn_c_lui_rs1_addr),
    .spec_rs2_addr(spec_insn_c_lui_rs2_addr),
    .spec_rd_addr(spec_insn_c_lui_rd_addr),
    .spec_rd_wdata(spec_insn_c_lui_rd_wdata),
    .spec_pc_wdata(spec_insn_c_lui_pc_wdata),
    .spec_mem_addr(spec_insn_c_lui_mem_addr),
    .spec_mem_rmask(spec_insn_c_lui_mem_rmask),
    .spec_mem_wmask(spec_insn_c_lui_mem_wmask),
    .spec_mem_wdata(spec_insn_c_lui_mem_wdata)
  );

  wire                                spec_insn_c_lw_valid;
  wire                                spec_insn_c_lw_trap;
  wire [                       4 : 0] spec_insn_c_lw_rs1_addr;
  wire [                       4 : 0] spec_insn_c_lw_rs2_addr;
  wire [                       4 : 0] spec_insn_c_lw_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_lw_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_lw_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_lw_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_lw_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_lw_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_lw_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_lw insn_c_lw (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_lw_valid),
    .spec_trap(spec_insn_c_lw_trap),
    .spec_rs1_addr(spec_insn_c_lw_rs1_addr),
    .spec_rs2_addr(spec_insn_c_lw_rs2_addr),
    .spec_rd_addr(spec_insn_c_lw_rd_addr),
    .spec_rd_wdata(spec_insn_c_lw_rd_wdata),
    .spec_pc_wdata(spec_insn_c_lw_pc_wdata),
    .spec_mem_addr(spec_insn_c_lw_mem_addr),
    .spec_mem_rmask(spec_insn_c_lw_mem_rmask),
    .spec_mem_wmask(spec_insn_c_lw_mem_wmask),
    .spec_mem_wdata(spec_insn_c_lw_mem_wdata)
  );

  wire                                spec_insn_c_lwsp_valid;
  wire                                spec_insn_c_lwsp_trap;
  wire [                       4 : 0] spec_insn_c_lwsp_rs1_addr;
  wire [                       4 : 0] spec_insn_c_lwsp_rs2_addr;
  wire [                       4 : 0] spec_insn_c_lwsp_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_lwsp_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_lwsp_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_lwsp_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_lwsp_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_lwsp_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_lwsp_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_lwsp insn_c_lwsp (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_lwsp_valid),
    .spec_trap(spec_insn_c_lwsp_trap),
    .spec_rs1_addr(spec_insn_c_lwsp_rs1_addr),
    .spec_rs2_addr(spec_insn_c_lwsp_rs2_addr),
    .spec_rd_addr(spec_insn_c_lwsp_rd_addr),
    .spec_rd_wdata(spec_insn_c_lwsp_rd_wdata),
    .spec_pc_wdata(spec_insn_c_lwsp_pc_wdata),
    .spec_mem_addr(spec_insn_c_lwsp_mem_addr),
    .spec_mem_rmask(spec_insn_c_lwsp_mem_rmask),
    .spec_mem_wmask(spec_insn_c_lwsp_mem_wmask),
    .spec_mem_wdata(spec_insn_c_lwsp_mem_wdata)
  );

  wire                                spec_insn_c_mv_valid;
  wire                                spec_insn_c_mv_trap;
  wire [                       4 : 0] spec_insn_c_mv_rs1_addr;
  wire [                       4 : 0] spec_insn_c_mv_rs2_addr;
  wire [                       4 : 0] spec_insn_c_mv_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_mv_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_mv_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_mv_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_mv_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_mv_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_mv_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_mv insn_c_mv (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_mv_valid),
    .spec_trap(spec_insn_c_mv_trap),
    .spec_rs1_addr(spec_insn_c_mv_rs1_addr),
    .spec_rs2_addr(spec_insn_c_mv_rs2_addr),
    .spec_rd_addr(spec_insn_c_mv_rd_addr),
    .spec_rd_wdata(spec_insn_c_mv_rd_wdata),
    .spec_pc_wdata(spec_insn_c_mv_pc_wdata),
    .spec_mem_addr(spec_insn_c_mv_mem_addr),
    .spec_mem_rmask(spec_insn_c_mv_mem_rmask),
    .spec_mem_wmask(spec_insn_c_mv_mem_wmask),
    .spec_mem_wdata(spec_insn_c_mv_mem_wdata)
  );

  wire                                spec_insn_c_or_valid;
  wire                                spec_insn_c_or_trap;
  wire [                       4 : 0] spec_insn_c_or_rs1_addr;
  wire [                       4 : 0] spec_insn_c_or_rs2_addr;
  wire [                       4 : 0] spec_insn_c_or_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_or_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_or_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_or_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_or_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_or_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_or_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_or insn_c_or (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_or_valid),
    .spec_trap(spec_insn_c_or_trap),
    .spec_rs1_addr(spec_insn_c_or_rs1_addr),
    .spec_rs2_addr(spec_insn_c_or_rs2_addr),
    .spec_rd_addr(spec_insn_c_or_rd_addr),
    .spec_rd_wdata(spec_insn_c_or_rd_wdata),
    .spec_pc_wdata(spec_insn_c_or_pc_wdata),
    .spec_mem_addr(spec_insn_c_or_mem_addr),
    .spec_mem_rmask(spec_insn_c_or_mem_rmask),
    .spec_mem_wmask(spec_insn_c_or_mem_wmask),
    .spec_mem_wdata(spec_insn_c_or_mem_wdata)
  );

  wire                                spec_insn_c_slli_valid;
  wire                                spec_insn_c_slli_trap;
  wire [                       4 : 0] spec_insn_c_slli_rs1_addr;
  wire [                       4 : 0] spec_insn_c_slli_rs2_addr;
  wire [                       4 : 0] spec_insn_c_slli_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_slli_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_slli_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_slli_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_slli_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_slli_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_slli_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_slli insn_c_slli (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_slli_valid),
    .spec_trap(spec_insn_c_slli_trap),
    .spec_rs1_addr(spec_insn_c_slli_rs1_addr),
    .spec_rs2_addr(spec_insn_c_slli_rs2_addr),
    .spec_rd_addr(spec_insn_c_slli_rd_addr),
    .spec_rd_wdata(spec_insn_c_slli_rd_wdata),
    .spec_pc_wdata(spec_insn_c_slli_pc_wdata),
    .spec_mem_addr(spec_insn_c_slli_mem_addr),
    .spec_mem_rmask(spec_insn_c_slli_mem_rmask),
    .spec_mem_wmask(spec_insn_c_slli_mem_wmask),
    .spec_mem_wdata(spec_insn_c_slli_mem_wdata)
  );

  wire                                spec_insn_c_srai_valid;
  wire                                spec_insn_c_srai_trap;
  wire [                       4 : 0] spec_insn_c_srai_rs1_addr;
  wire [                       4 : 0] spec_insn_c_srai_rs2_addr;
  wire [                       4 : 0] spec_insn_c_srai_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_srai_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_srai_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_srai_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_srai_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_srai_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_srai_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_srai insn_c_srai (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_srai_valid),
    .spec_trap(spec_insn_c_srai_trap),
    .spec_rs1_addr(spec_insn_c_srai_rs1_addr),
    .spec_rs2_addr(spec_insn_c_srai_rs2_addr),
    .spec_rd_addr(spec_insn_c_srai_rd_addr),
    .spec_rd_wdata(spec_insn_c_srai_rd_wdata),
    .spec_pc_wdata(spec_insn_c_srai_pc_wdata),
    .spec_mem_addr(spec_insn_c_srai_mem_addr),
    .spec_mem_rmask(spec_insn_c_srai_mem_rmask),
    .spec_mem_wmask(spec_insn_c_srai_mem_wmask),
    .spec_mem_wdata(spec_insn_c_srai_mem_wdata)
  );

  wire                                spec_insn_c_srli_valid;
  wire                                spec_insn_c_srli_trap;
  wire [                       4 : 0] spec_insn_c_srli_rs1_addr;
  wire [                       4 : 0] spec_insn_c_srli_rs2_addr;
  wire [                       4 : 0] spec_insn_c_srli_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_srli_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_srli_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_srli_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_srli_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_srli_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_srli_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_srli insn_c_srli (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_srli_valid),
    .spec_trap(spec_insn_c_srli_trap),
    .spec_rs1_addr(spec_insn_c_srli_rs1_addr),
    .spec_rs2_addr(spec_insn_c_srli_rs2_addr),
    .spec_rd_addr(spec_insn_c_srli_rd_addr),
    .spec_rd_wdata(spec_insn_c_srli_rd_wdata),
    .spec_pc_wdata(spec_insn_c_srli_pc_wdata),
    .spec_mem_addr(spec_insn_c_srli_mem_addr),
    .spec_mem_rmask(spec_insn_c_srli_mem_rmask),
    .spec_mem_wmask(spec_insn_c_srli_mem_wmask),
    .spec_mem_wdata(spec_insn_c_srli_mem_wdata)
  );

  wire                                spec_insn_c_sub_valid;
  wire                                spec_insn_c_sub_trap;
  wire [                       4 : 0] spec_insn_c_sub_rs1_addr;
  wire [                       4 : 0] spec_insn_c_sub_rs2_addr;
  wire [                       4 : 0] spec_insn_c_sub_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_sub_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_sub_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_sub_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_sub_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_sub_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_sub_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_sub insn_c_sub (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_sub_valid),
    .spec_trap(spec_insn_c_sub_trap),
    .spec_rs1_addr(spec_insn_c_sub_rs1_addr),
    .spec_rs2_addr(spec_insn_c_sub_rs2_addr),
    .spec_rd_addr(spec_insn_c_sub_rd_addr),
    .spec_rd_wdata(spec_insn_c_sub_rd_wdata),
    .spec_pc_wdata(spec_insn_c_sub_pc_wdata),
    .spec_mem_addr(spec_insn_c_sub_mem_addr),
    .spec_mem_rmask(spec_insn_c_sub_mem_rmask),
    .spec_mem_wmask(spec_insn_c_sub_mem_wmask),
    .spec_mem_wdata(spec_insn_c_sub_mem_wdata)
  );

  wire                                spec_insn_c_sw_valid;
  wire                                spec_insn_c_sw_trap;
  wire [                       4 : 0] spec_insn_c_sw_rs1_addr;
  wire [                       4 : 0] spec_insn_c_sw_rs2_addr;
  wire [                       4 : 0] spec_insn_c_sw_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_sw_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_sw_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_sw_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_sw_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_sw_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_sw_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_sw insn_c_sw (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_sw_valid),
    .spec_trap(spec_insn_c_sw_trap),
    .spec_rs1_addr(spec_insn_c_sw_rs1_addr),
    .spec_rs2_addr(spec_insn_c_sw_rs2_addr),
    .spec_rd_addr(spec_insn_c_sw_rd_addr),
    .spec_rd_wdata(spec_insn_c_sw_rd_wdata),
    .spec_pc_wdata(spec_insn_c_sw_pc_wdata),
    .spec_mem_addr(spec_insn_c_sw_mem_addr),
    .spec_mem_rmask(spec_insn_c_sw_mem_rmask),
    .spec_mem_wmask(spec_insn_c_sw_mem_wmask),
    .spec_mem_wdata(spec_insn_c_sw_mem_wdata)
  );

  wire                                spec_insn_c_swsp_valid;
  wire                                spec_insn_c_swsp_trap;
  wire [                       4 : 0] spec_insn_c_swsp_rs1_addr;
  wire [                       4 : 0] spec_insn_c_swsp_rs2_addr;
  wire [                       4 : 0] spec_insn_c_swsp_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_swsp_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_swsp_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_swsp_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_swsp_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_swsp_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_swsp_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_swsp insn_c_swsp (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_swsp_valid),
    .spec_trap(spec_insn_c_swsp_trap),
    .spec_rs1_addr(spec_insn_c_swsp_rs1_addr),
    .spec_rs2_addr(spec_insn_c_swsp_rs2_addr),
    .spec_rd_addr(spec_insn_c_swsp_rd_addr),
    .spec_rd_wdata(spec_insn_c_swsp_rd_wdata),
    .spec_pc_wdata(spec_insn_c_swsp_pc_wdata),
    .spec_mem_addr(spec_insn_c_swsp_mem_addr),
    .spec_mem_rmask(spec_insn_c_swsp_mem_rmask),
    .spec_mem_wmask(spec_insn_c_swsp_mem_wmask),
    .spec_mem_wdata(spec_insn_c_swsp_mem_wdata)
  );

  wire                                spec_insn_c_xor_valid;
  wire                                spec_insn_c_xor_trap;
  wire [                       4 : 0] spec_insn_c_xor_rs1_addr;
  wire [                       4 : 0] spec_insn_c_xor_rs2_addr;
  wire [                       4 : 0] spec_insn_c_xor_rd_addr;
  wire [32   - 1 : 0] spec_insn_c_xor_rd_wdata;
  wire [32   - 1 : 0] spec_insn_c_xor_pc_wdata;
  wire [32   - 1 : 0] spec_insn_c_xor_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_c_xor_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_c_xor_mem_wmask;
  wire [32   - 1 : 0] spec_insn_c_xor_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_c_xor insn_c_xor (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_c_xor_valid),
    .spec_trap(spec_insn_c_xor_trap),
    .spec_rs1_addr(spec_insn_c_xor_rs1_addr),
    .spec_rs2_addr(spec_insn_c_xor_rs2_addr),
    .spec_rd_addr(spec_insn_c_xor_rd_addr),
    .spec_rd_wdata(spec_insn_c_xor_rd_wdata),
    .spec_pc_wdata(spec_insn_c_xor_pc_wdata),
    .spec_mem_addr(spec_insn_c_xor_mem_addr),
    .spec_mem_rmask(spec_insn_c_xor_mem_rmask),
    .spec_mem_wmask(spec_insn_c_xor_mem_wmask),
    .spec_mem_wdata(spec_insn_c_xor_mem_wdata)
  );

  wire                                spec_insn_div_valid;
  wire                                spec_insn_div_trap;
  wire [                       4 : 0] spec_insn_div_rs1_addr;
  wire [                       4 : 0] spec_insn_div_rs2_addr;
  wire [                       4 : 0] spec_insn_div_rd_addr;
  wire [32   - 1 : 0] spec_insn_div_rd_wdata;
  wire [32   - 1 : 0] spec_insn_div_pc_wdata;
  wire [32   - 1 : 0] spec_insn_div_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_div_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_div_mem_wmask;
  wire [32   - 1 : 0] spec_insn_div_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_div insn_div (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_div_valid),
    .spec_trap(spec_insn_div_trap),
    .spec_rs1_addr(spec_insn_div_rs1_addr),
    .spec_rs2_addr(spec_insn_div_rs2_addr),
    .spec_rd_addr(spec_insn_div_rd_addr),
    .spec_rd_wdata(spec_insn_div_rd_wdata),
    .spec_pc_wdata(spec_insn_div_pc_wdata),
    .spec_mem_addr(spec_insn_div_mem_addr),
    .spec_mem_rmask(spec_insn_div_mem_rmask),
    .spec_mem_wmask(spec_insn_div_mem_wmask),
    .spec_mem_wdata(spec_insn_div_mem_wdata)
  );

  wire                                spec_insn_divu_valid;
  wire                                spec_insn_divu_trap;
  wire [                       4 : 0] spec_insn_divu_rs1_addr;
  wire [                       4 : 0] spec_insn_divu_rs2_addr;
  wire [                       4 : 0] spec_insn_divu_rd_addr;
  wire [32   - 1 : 0] spec_insn_divu_rd_wdata;
  wire [32   - 1 : 0] spec_insn_divu_pc_wdata;
  wire [32   - 1 : 0] spec_insn_divu_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_divu_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_divu_mem_wmask;
  wire [32   - 1 : 0] spec_insn_divu_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_divu insn_divu (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_divu_valid),
    .spec_trap(spec_insn_divu_trap),
    .spec_rs1_addr(spec_insn_divu_rs1_addr),
    .spec_rs2_addr(spec_insn_divu_rs2_addr),
    .spec_rd_addr(spec_insn_divu_rd_addr),
    .spec_rd_wdata(spec_insn_divu_rd_wdata),
    .spec_pc_wdata(spec_insn_divu_pc_wdata),
    .spec_mem_addr(spec_insn_divu_mem_addr),
    .spec_mem_rmask(spec_insn_divu_mem_rmask),
    .spec_mem_wmask(spec_insn_divu_mem_wmask),
    .spec_mem_wdata(spec_insn_divu_mem_wdata)
  );

  wire                                spec_insn_jal_valid;
  wire                                spec_insn_jal_trap;
  wire [                       4 : 0] spec_insn_jal_rs1_addr;
  wire [                       4 : 0] spec_insn_jal_rs2_addr;
  wire [                       4 : 0] spec_insn_jal_rd_addr;
  wire [32   - 1 : 0] spec_insn_jal_rd_wdata;
  wire [32   - 1 : 0] spec_insn_jal_pc_wdata;
  wire [32   - 1 : 0] spec_insn_jal_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_jal_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_jal_mem_wmask;
  wire [32   - 1 : 0] spec_insn_jal_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_jal insn_jal (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_jal_valid),
    .spec_trap(spec_insn_jal_trap),
    .spec_rs1_addr(spec_insn_jal_rs1_addr),
    .spec_rs2_addr(spec_insn_jal_rs2_addr),
    .spec_rd_addr(spec_insn_jal_rd_addr),
    .spec_rd_wdata(spec_insn_jal_rd_wdata),
    .spec_pc_wdata(spec_insn_jal_pc_wdata),
    .spec_mem_addr(spec_insn_jal_mem_addr),
    .spec_mem_rmask(spec_insn_jal_mem_rmask),
    .spec_mem_wmask(spec_insn_jal_mem_wmask),
    .spec_mem_wdata(spec_insn_jal_mem_wdata)
  );

  wire                                spec_insn_jalr_valid;
  wire                                spec_insn_jalr_trap;
  wire [                       4 : 0] spec_insn_jalr_rs1_addr;
  wire [                       4 : 0] spec_insn_jalr_rs2_addr;
  wire [                       4 : 0] spec_insn_jalr_rd_addr;
  wire [32   - 1 : 0] spec_insn_jalr_rd_wdata;
  wire [32   - 1 : 0] spec_insn_jalr_pc_wdata;
  wire [32   - 1 : 0] spec_insn_jalr_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_jalr_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_jalr_mem_wmask;
  wire [32   - 1 : 0] spec_insn_jalr_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_jalr insn_jalr (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_jalr_valid),
    .spec_trap(spec_insn_jalr_trap),
    .spec_rs1_addr(spec_insn_jalr_rs1_addr),
    .spec_rs2_addr(spec_insn_jalr_rs2_addr),
    .spec_rd_addr(spec_insn_jalr_rd_addr),
    .spec_rd_wdata(spec_insn_jalr_rd_wdata),
    .spec_pc_wdata(spec_insn_jalr_pc_wdata),
    .spec_mem_addr(spec_insn_jalr_mem_addr),
    .spec_mem_rmask(spec_insn_jalr_mem_rmask),
    .spec_mem_wmask(spec_insn_jalr_mem_wmask),
    .spec_mem_wdata(spec_insn_jalr_mem_wdata)
  );

  wire                                spec_insn_lb_valid;
  wire                                spec_insn_lb_trap;
  wire [                       4 : 0] spec_insn_lb_rs1_addr;
  wire [                       4 : 0] spec_insn_lb_rs2_addr;
  wire [                       4 : 0] spec_insn_lb_rd_addr;
  wire [32   - 1 : 0] spec_insn_lb_rd_wdata;
  wire [32   - 1 : 0] spec_insn_lb_pc_wdata;
  wire [32   - 1 : 0] spec_insn_lb_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_lb_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_lb_mem_wmask;
  wire [32   - 1 : 0] spec_insn_lb_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_lb insn_lb (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_lb_valid),
    .spec_trap(spec_insn_lb_trap),
    .spec_rs1_addr(spec_insn_lb_rs1_addr),
    .spec_rs2_addr(spec_insn_lb_rs2_addr),
    .spec_rd_addr(spec_insn_lb_rd_addr),
    .spec_rd_wdata(spec_insn_lb_rd_wdata),
    .spec_pc_wdata(spec_insn_lb_pc_wdata),
    .spec_mem_addr(spec_insn_lb_mem_addr),
    .spec_mem_rmask(spec_insn_lb_mem_rmask),
    .spec_mem_wmask(spec_insn_lb_mem_wmask),
    .spec_mem_wdata(spec_insn_lb_mem_wdata)
  );

  wire                                spec_insn_lbu_valid;
  wire                                spec_insn_lbu_trap;
  wire [                       4 : 0] spec_insn_lbu_rs1_addr;
  wire [                       4 : 0] spec_insn_lbu_rs2_addr;
  wire [                       4 : 0] spec_insn_lbu_rd_addr;
  wire [32   - 1 : 0] spec_insn_lbu_rd_wdata;
  wire [32   - 1 : 0] spec_insn_lbu_pc_wdata;
  wire [32   - 1 : 0] spec_insn_lbu_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_lbu_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_lbu_mem_wmask;
  wire [32   - 1 : 0] spec_insn_lbu_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_lbu insn_lbu (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_lbu_valid),
    .spec_trap(spec_insn_lbu_trap),
    .spec_rs1_addr(spec_insn_lbu_rs1_addr),
    .spec_rs2_addr(spec_insn_lbu_rs2_addr),
    .spec_rd_addr(spec_insn_lbu_rd_addr),
    .spec_rd_wdata(spec_insn_lbu_rd_wdata),
    .spec_pc_wdata(spec_insn_lbu_pc_wdata),
    .spec_mem_addr(spec_insn_lbu_mem_addr),
    .spec_mem_rmask(spec_insn_lbu_mem_rmask),
    .spec_mem_wmask(spec_insn_lbu_mem_wmask),
    .spec_mem_wdata(spec_insn_lbu_mem_wdata)
  );

  wire                                spec_insn_lh_valid;
  wire                                spec_insn_lh_trap;
  wire [                       4 : 0] spec_insn_lh_rs1_addr;
  wire [                       4 : 0] spec_insn_lh_rs2_addr;
  wire [                       4 : 0] spec_insn_lh_rd_addr;
  wire [32   - 1 : 0] spec_insn_lh_rd_wdata;
  wire [32   - 1 : 0] spec_insn_lh_pc_wdata;
  wire [32   - 1 : 0] spec_insn_lh_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_lh_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_lh_mem_wmask;
  wire [32   - 1 : 0] spec_insn_lh_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_lh insn_lh (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_lh_valid),
    .spec_trap(spec_insn_lh_trap),
    .spec_rs1_addr(spec_insn_lh_rs1_addr),
    .spec_rs2_addr(spec_insn_lh_rs2_addr),
    .spec_rd_addr(spec_insn_lh_rd_addr),
    .spec_rd_wdata(spec_insn_lh_rd_wdata),
    .spec_pc_wdata(spec_insn_lh_pc_wdata),
    .spec_mem_addr(spec_insn_lh_mem_addr),
    .spec_mem_rmask(spec_insn_lh_mem_rmask),
    .spec_mem_wmask(spec_insn_lh_mem_wmask),
    .spec_mem_wdata(spec_insn_lh_mem_wdata)
  );

  wire                                spec_insn_lhu_valid;
  wire                                spec_insn_lhu_trap;
  wire [                       4 : 0] spec_insn_lhu_rs1_addr;
  wire [                       4 : 0] spec_insn_lhu_rs2_addr;
  wire [                       4 : 0] spec_insn_lhu_rd_addr;
  wire [32   - 1 : 0] spec_insn_lhu_rd_wdata;
  wire [32   - 1 : 0] spec_insn_lhu_pc_wdata;
  wire [32   - 1 : 0] spec_insn_lhu_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_lhu_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_lhu_mem_wmask;
  wire [32   - 1 : 0] spec_insn_lhu_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_lhu insn_lhu (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_lhu_valid),
    .spec_trap(spec_insn_lhu_trap),
    .spec_rs1_addr(spec_insn_lhu_rs1_addr),
    .spec_rs2_addr(spec_insn_lhu_rs2_addr),
    .spec_rd_addr(spec_insn_lhu_rd_addr),
    .spec_rd_wdata(spec_insn_lhu_rd_wdata),
    .spec_pc_wdata(spec_insn_lhu_pc_wdata),
    .spec_mem_addr(spec_insn_lhu_mem_addr),
    .spec_mem_rmask(spec_insn_lhu_mem_rmask),
    .spec_mem_wmask(spec_insn_lhu_mem_wmask),
    .spec_mem_wdata(spec_insn_lhu_mem_wdata)
  );

  wire                                spec_insn_lui_valid;
  wire                                spec_insn_lui_trap;
  wire [                       4 : 0] spec_insn_lui_rs1_addr;
  wire [                       4 : 0] spec_insn_lui_rs2_addr;
  wire [                       4 : 0] spec_insn_lui_rd_addr;
  wire [32   - 1 : 0] spec_insn_lui_rd_wdata;
  wire [32   - 1 : 0] spec_insn_lui_pc_wdata;
  wire [32   - 1 : 0] spec_insn_lui_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_lui_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_lui_mem_wmask;
  wire [32   - 1 : 0] spec_insn_lui_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_lui insn_lui (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_lui_valid),
    .spec_trap(spec_insn_lui_trap),
    .spec_rs1_addr(spec_insn_lui_rs1_addr),
    .spec_rs2_addr(spec_insn_lui_rs2_addr),
    .spec_rd_addr(spec_insn_lui_rd_addr),
    .spec_rd_wdata(spec_insn_lui_rd_wdata),
    .spec_pc_wdata(spec_insn_lui_pc_wdata),
    .spec_mem_addr(spec_insn_lui_mem_addr),
    .spec_mem_rmask(spec_insn_lui_mem_rmask),
    .spec_mem_wmask(spec_insn_lui_mem_wmask),
    .spec_mem_wdata(spec_insn_lui_mem_wdata)
  );

  wire                                spec_insn_lw_valid;
  wire                                spec_insn_lw_trap;
  wire [                       4 : 0] spec_insn_lw_rs1_addr;
  wire [                       4 : 0] spec_insn_lw_rs2_addr;
  wire [                       4 : 0] spec_insn_lw_rd_addr;
  wire [32   - 1 : 0] spec_insn_lw_rd_wdata;
  wire [32   - 1 : 0] spec_insn_lw_pc_wdata;
  wire [32   - 1 : 0] spec_insn_lw_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_lw_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_lw_mem_wmask;
  wire [32   - 1 : 0] spec_insn_lw_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_lw insn_lw (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_lw_valid),
    .spec_trap(spec_insn_lw_trap),
    .spec_rs1_addr(spec_insn_lw_rs1_addr),
    .spec_rs2_addr(spec_insn_lw_rs2_addr),
    .spec_rd_addr(spec_insn_lw_rd_addr),
    .spec_rd_wdata(spec_insn_lw_rd_wdata),
    .spec_pc_wdata(spec_insn_lw_pc_wdata),
    .spec_mem_addr(spec_insn_lw_mem_addr),
    .spec_mem_rmask(spec_insn_lw_mem_rmask),
    .spec_mem_wmask(spec_insn_lw_mem_wmask),
    .spec_mem_wdata(spec_insn_lw_mem_wdata)
  );

  wire                                spec_insn_mul_valid;
  wire                                spec_insn_mul_trap;
  wire [                       4 : 0] spec_insn_mul_rs1_addr;
  wire [                       4 : 0] spec_insn_mul_rs2_addr;
  wire [                       4 : 0] spec_insn_mul_rd_addr;
  wire [32   - 1 : 0] spec_insn_mul_rd_wdata;
  wire [32   - 1 : 0] spec_insn_mul_pc_wdata;
  wire [32   - 1 : 0] spec_insn_mul_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_mul_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_mul_mem_wmask;
  wire [32   - 1 : 0] spec_insn_mul_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_mul insn_mul (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_mul_valid),
    .spec_trap(spec_insn_mul_trap),
    .spec_rs1_addr(spec_insn_mul_rs1_addr),
    .spec_rs2_addr(spec_insn_mul_rs2_addr),
    .spec_rd_addr(spec_insn_mul_rd_addr),
    .spec_rd_wdata(spec_insn_mul_rd_wdata),
    .spec_pc_wdata(spec_insn_mul_pc_wdata),
    .spec_mem_addr(spec_insn_mul_mem_addr),
    .spec_mem_rmask(spec_insn_mul_mem_rmask),
    .spec_mem_wmask(spec_insn_mul_mem_wmask),
    .spec_mem_wdata(spec_insn_mul_mem_wdata)
  );

  wire                                spec_insn_mulh_valid;
  wire                                spec_insn_mulh_trap;
  wire [                       4 : 0] spec_insn_mulh_rs1_addr;
  wire [                       4 : 0] spec_insn_mulh_rs2_addr;
  wire [                       4 : 0] spec_insn_mulh_rd_addr;
  wire [32   - 1 : 0] spec_insn_mulh_rd_wdata;
  wire [32   - 1 : 0] spec_insn_mulh_pc_wdata;
  wire [32   - 1 : 0] spec_insn_mulh_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_mulh_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_mulh_mem_wmask;
  wire [32   - 1 : 0] spec_insn_mulh_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_mulh insn_mulh (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_mulh_valid),
    .spec_trap(spec_insn_mulh_trap),
    .spec_rs1_addr(spec_insn_mulh_rs1_addr),
    .spec_rs2_addr(spec_insn_mulh_rs2_addr),
    .spec_rd_addr(spec_insn_mulh_rd_addr),
    .spec_rd_wdata(spec_insn_mulh_rd_wdata),
    .spec_pc_wdata(spec_insn_mulh_pc_wdata),
    .spec_mem_addr(spec_insn_mulh_mem_addr),
    .spec_mem_rmask(spec_insn_mulh_mem_rmask),
    .spec_mem_wmask(spec_insn_mulh_mem_wmask),
    .spec_mem_wdata(spec_insn_mulh_mem_wdata)
  );

  wire                                spec_insn_mulhsu_valid;
  wire                                spec_insn_mulhsu_trap;
  wire [                       4 : 0] spec_insn_mulhsu_rs1_addr;
  wire [                       4 : 0] spec_insn_mulhsu_rs2_addr;
  wire [                       4 : 0] spec_insn_mulhsu_rd_addr;
  wire [32   - 1 : 0] spec_insn_mulhsu_rd_wdata;
  wire [32   - 1 : 0] spec_insn_mulhsu_pc_wdata;
  wire [32   - 1 : 0] spec_insn_mulhsu_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_mulhsu_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_mulhsu_mem_wmask;
  wire [32   - 1 : 0] spec_insn_mulhsu_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_mulhsu insn_mulhsu (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_mulhsu_valid),
    .spec_trap(spec_insn_mulhsu_trap),
    .spec_rs1_addr(spec_insn_mulhsu_rs1_addr),
    .spec_rs2_addr(spec_insn_mulhsu_rs2_addr),
    .spec_rd_addr(spec_insn_mulhsu_rd_addr),
    .spec_rd_wdata(spec_insn_mulhsu_rd_wdata),
    .spec_pc_wdata(spec_insn_mulhsu_pc_wdata),
    .spec_mem_addr(spec_insn_mulhsu_mem_addr),
    .spec_mem_rmask(spec_insn_mulhsu_mem_rmask),
    .spec_mem_wmask(spec_insn_mulhsu_mem_wmask),
    .spec_mem_wdata(spec_insn_mulhsu_mem_wdata)
  );

  wire                                spec_insn_mulhu_valid;
  wire                                spec_insn_mulhu_trap;
  wire [                       4 : 0] spec_insn_mulhu_rs1_addr;
  wire [                       4 : 0] spec_insn_mulhu_rs2_addr;
  wire [                       4 : 0] spec_insn_mulhu_rd_addr;
  wire [32   - 1 : 0] spec_insn_mulhu_rd_wdata;
  wire [32   - 1 : 0] spec_insn_mulhu_pc_wdata;
  wire [32   - 1 : 0] spec_insn_mulhu_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_mulhu_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_mulhu_mem_wmask;
  wire [32   - 1 : 0] spec_insn_mulhu_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_mulhu insn_mulhu (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_mulhu_valid),
    .spec_trap(spec_insn_mulhu_trap),
    .spec_rs1_addr(spec_insn_mulhu_rs1_addr),
    .spec_rs2_addr(spec_insn_mulhu_rs2_addr),
    .spec_rd_addr(spec_insn_mulhu_rd_addr),
    .spec_rd_wdata(spec_insn_mulhu_rd_wdata),
    .spec_pc_wdata(spec_insn_mulhu_pc_wdata),
    .spec_mem_addr(spec_insn_mulhu_mem_addr),
    .spec_mem_rmask(spec_insn_mulhu_mem_rmask),
    .spec_mem_wmask(spec_insn_mulhu_mem_wmask),
    .spec_mem_wdata(spec_insn_mulhu_mem_wdata)
  );

  wire                                spec_insn_or_valid;
  wire                                spec_insn_or_trap;
  wire [                       4 : 0] spec_insn_or_rs1_addr;
  wire [                       4 : 0] spec_insn_or_rs2_addr;
  wire [                       4 : 0] spec_insn_or_rd_addr;
  wire [32   - 1 : 0] spec_insn_or_rd_wdata;
  wire [32   - 1 : 0] spec_insn_or_pc_wdata;
  wire [32   - 1 : 0] spec_insn_or_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_or_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_or_mem_wmask;
  wire [32   - 1 : 0] spec_insn_or_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_or insn_or (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_or_valid),
    .spec_trap(spec_insn_or_trap),
    .spec_rs1_addr(spec_insn_or_rs1_addr),
    .spec_rs2_addr(spec_insn_or_rs2_addr),
    .spec_rd_addr(spec_insn_or_rd_addr),
    .spec_rd_wdata(spec_insn_or_rd_wdata),
    .spec_pc_wdata(spec_insn_or_pc_wdata),
    .spec_mem_addr(spec_insn_or_mem_addr),
    .spec_mem_rmask(spec_insn_or_mem_rmask),
    .spec_mem_wmask(spec_insn_or_mem_wmask),
    .spec_mem_wdata(spec_insn_or_mem_wdata)
  );

  wire                                spec_insn_ori_valid;
  wire                                spec_insn_ori_trap;
  wire [                       4 : 0] spec_insn_ori_rs1_addr;
  wire [                       4 : 0] spec_insn_ori_rs2_addr;
  wire [                       4 : 0] spec_insn_ori_rd_addr;
  wire [32   - 1 : 0] spec_insn_ori_rd_wdata;
  wire [32   - 1 : 0] spec_insn_ori_pc_wdata;
  wire [32   - 1 : 0] spec_insn_ori_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_ori_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_ori_mem_wmask;
  wire [32   - 1 : 0] spec_insn_ori_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_ori insn_ori (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_ori_valid),
    .spec_trap(spec_insn_ori_trap),
    .spec_rs1_addr(spec_insn_ori_rs1_addr),
    .spec_rs2_addr(spec_insn_ori_rs2_addr),
    .spec_rd_addr(spec_insn_ori_rd_addr),
    .spec_rd_wdata(spec_insn_ori_rd_wdata),
    .spec_pc_wdata(spec_insn_ori_pc_wdata),
    .spec_mem_addr(spec_insn_ori_mem_addr),
    .spec_mem_rmask(spec_insn_ori_mem_rmask),
    .spec_mem_wmask(spec_insn_ori_mem_wmask),
    .spec_mem_wdata(spec_insn_ori_mem_wdata)
  );

  wire                                spec_insn_rem_valid;
  wire                                spec_insn_rem_trap;
  wire [                       4 : 0] spec_insn_rem_rs1_addr;
  wire [                       4 : 0] spec_insn_rem_rs2_addr;
  wire [                       4 : 0] spec_insn_rem_rd_addr;
  wire [32   - 1 : 0] spec_insn_rem_rd_wdata;
  wire [32   - 1 : 0] spec_insn_rem_pc_wdata;
  wire [32   - 1 : 0] spec_insn_rem_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_rem_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_rem_mem_wmask;
  wire [32   - 1 : 0] spec_insn_rem_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_rem insn_rem (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_rem_valid),
    .spec_trap(spec_insn_rem_trap),
    .spec_rs1_addr(spec_insn_rem_rs1_addr),
    .spec_rs2_addr(spec_insn_rem_rs2_addr),
    .spec_rd_addr(spec_insn_rem_rd_addr),
    .spec_rd_wdata(spec_insn_rem_rd_wdata),
    .spec_pc_wdata(spec_insn_rem_pc_wdata),
    .spec_mem_addr(spec_insn_rem_mem_addr),
    .spec_mem_rmask(spec_insn_rem_mem_rmask),
    .spec_mem_wmask(spec_insn_rem_mem_wmask),
    .spec_mem_wdata(spec_insn_rem_mem_wdata)
  );

  wire                                spec_insn_remu_valid;
  wire                                spec_insn_remu_trap;
  wire [                       4 : 0] spec_insn_remu_rs1_addr;
  wire [                       4 : 0] spec_insn_remu_rs2_addr;
  wire [                       4 : 0] spec_insn_remu_rd_addr;
  wire [32   - 1 : 0] spec_insn_remu_rd_wdata;
  wire [32   - 1 : 0] spec_insn_remu_pc_wdata;
  wire [32   - 1 : 0] spec_insn_remu_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_remu_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_remu_mem_wmask;
  wire [32   - 1 : 0] spec_insn_remu_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_remu insn_remu (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_remu_valid),
    .spec_trap(spec_insn_remu_trap),
    .spec_rs1_addr(spec_insn_remu_rs1_addr),
    .spec_rs2_addr(spec_insn_remu_rs2_addr),
    .spec_rd_addr(spec_insn_remu_rd_addr),
    .spec_rd_wdata(spec_insn_remu_rd_wdata),
    .spec_pc_wdata(spec_insn_remu_pc_wdata),
    .spec_mem_addr(spec_insn_remu_mem_addr),
    .spec_mem_rmask(spec_insn_remu_mem_rmask),
    .spec_mem_wmask(spec_insn_remu_mem_wmask),
    .spec_mem_wdata(spec_insn_remu_mem_wdata)
  );

  wire                                spec_insn_sb_valid;
  wire                                spec_insn_sb_trap;
  wire [                       4 : 0] spec_insn_sb_rs1_addr;
  wire [                       4 : 0] spec_insn_sb_rs2_addr;
  wire [                       4 : 0] spec_insn_sb_rd_addr;
  wire [32   - 1 : 0] spec_insn_sb_rd_wdata;
  wire [32   - 1 : 0] spec_insn_sb_pc_wdata;
  wire [32   - 1 : 0] spec_insn_sb_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_sb_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_sb_mem_wmask;
  wire [32   - 1 : 0] spec_insn_sb_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_sb insn_sb (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_sb_valid),
    .spec_trap(spec_insn_sb_trap),
    .spec_rs1_addr(spec_insn_sb_rs1_addr),
    .spec_rs2_addr(spec_insn_sb_rs2_addr),
    .spec_rd_addr(spec_insn_sb_rd_addr),
    .spec_rd_wdata(spec_insn_sb_rd_wdata),
    .spec_pc_wdata(spec_insn_sb_pc_wdata),
    .spec_mem_addr(spec_insn_sb_mem_addr),
    .spec_mem_rmask(spec_insn_sb_mem_rmask),
    .spec_mem_wmask(spec_insn_sb_mem_wmask),
    .spec_mem_wdata(spec_insn_sb_mem_wdata)
  );

  wire                                spec_insn_sh_valid;
  wire                                spec_insn_sh_trap;
  wire [                       4 : 0] spec_insn_sh_rs1_addr;
  wire [                       4 : 0] spec_insn_sh_rs2_addr;
  wire [                       4 : 0] spec_insn_sh_rd_addr;
  wire [32   - 1 : 0] spec_insn_sh_rd_wdata;
  wire [32   - 1 : 0] spec_insn_sh_pc_wdata;
  wire [32   - 1 : 0] spec_insn_sh_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_sh_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_sh_mem_wmask;
  wire [32   - 1 : 0] spec_insn_sh_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_sh insn_sh (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_sh_valid),
    .spec_trap(spec_insn_sh_trap),
    .spec_rs1_addr(spec_insn_sh_rs1_addr),
    .spec_rs2_addr(spec_insn_sh_rs2_addr),
    .spec_rd_addr(spec_insn_sh_rd_addr),
    .spec_rd_wdata(spec_insn_sh_rd_wdata),
    .spec_pc_wdata(spec_insn_sh_pc_wdata),
    .spec_mem_addr(spec_insn_sh_mem_addr),
    .spec_mem_rmask(spec_insn_sh_mem_rmask),
    .spec_mem_wmask(spec_insn_sh_mem_wmask),
    .spec_mem_wdata(spec_insn_sh_mem_wdata)
  );

  wire                                spec_insn_sll_valid;
  wire                                spec_insn_sll_trap;
  wire [                       4 : 0] spec_insn_sll_rs1_addr;
  wire [                       4 : 0] spec_insn_sll_rs2_addr;
  wire [                       4 : 0] spec_insn_sll_rd_addr;
  wire [32   - 1 : 0] spec_insn_sll_rd_wdata;
  wire [32   - 1 : 0] spec_insn_sll_pc_wdata;
  wire [32   - 1 : 0] spec_insn_sll_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_sll_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_sll_mem_wmask;
  wire [32   - 1 : 0] spec_insn_sll_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_sll insn_sll (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_sll_valid),
    .spec_trap(spec_insn_sll_trap),
    .spec_rs1_addr(spec_insn_sll_rs1_addr),
    .spec_rs2_addr(spec_insn_sll_rs2_addr),
    .spec_rd_addr(spec_insn_sll_rd_addr),
    .spec_rd_wdata(spec_insn_sll_rd_wdata),
    .spec_pc_wdata(spec_insn_sll_pc_wdata),
    .spec_mem_addr(spec_insn_sll_mem_addr),
    .spec_mem_rmask(spec_insn_sll_mem_rmask),
    .spec_mem_wmask(spec_insn_sll_mem_wmask),
    .spec_mem_wdata(spec_insn_sll_mem_wdata)
  );

  wire                                spec_insn_slli_valid;
  wire                                spec_insn_slli_trap;
  wire [                       4 : 0] spec_insn_slli_rs1_addr;
  wire [                       4 : 0] spec_insn_slli_rs2_addr;
  wire [                       4 : 0] spec_insn_slli_rd_addr;
  wire [32   - 1 : 0] spec_insn_slli_rd_wdata;
  wire [32   - 1 : 0] spec_insn_slli_pc_wdata;
  wire [32   - 1 : 0] spec_insn_slli_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_slli_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_slli_mem_wmask;
  wire [32   - 1 : 0] spec_insn_slli_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_slli insn_slli (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_slli_valid),
    .spec_trap(spec_insn_slli_trap),
    .spec_rs1_addr(spec_insn_slli_rs1_addr),
    .spec_rs2_addr(spec_insn_slli_rs2_addr),
    .spec_rd_addr(spec_insn_slli_rd_addr),
    .spec_rd_wdata(spec_insn_slli_rd_wdata),
    .spec_pc_wdata(spec_insn_slli_pc_wdata),
    .spec_mem_addr(spec_insn_slli_mem_addr),
    .spec_mem_rmask(spec_insn_slli_mem_rmask),
    .spec_mem_wmask(spec_insn_slli_mem_wmask),
    .spec_mem_wdata(spec_insn_slli_mem_wdata)
  );

  wire                                spec_insn_slt_valid;
  wire                                spec_insn_slt_trap;
  wire [                       4 : 0] spec_insn_slt_rs1_addr;
  wire [                       4 : 0] spec_insn_slt_rs2_addr;
  wire [                       4 : 0] spec_insn_slt_rd_addr;
  wire [32   - 1 : 0] spec_insn_slt_rd_wdata;
  wire [32   - 1 : 0] spec_insn_slt_pc_wdata;
  wire [32   - 1 : 0] spec_insn_slt_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_slt_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_slt_mem_wmask;
  wire [32   - 1 : 0] spec_insn_slt_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_slt insn_slt (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_slt_valid),
    .spec_trap(spec_insn_slt_trap),
    .spec_rs1_addr(spec_insn_slt_rs1_addr),
    .spec_rs2_addr(spec_insn_slt_rs2_addr),
    .spec_rd_addr(spec_insn_slt_rd_addr),
    .spec_rd_wdata(spec_insn_slt_rd_wdata),
    .spec_pc_wdata(spec_insn_slt_pc_wdata),
    .spec_mem_addr(spec_insn_slt_mem_addr),
    .spec_mem_rmask(spec_insn_slt_mem_rmask),
    .spec_mem_wmask(spec_insn_slt_mem_wmask),
    .spec_mem_wdata(spec_insn_slt_mem_wdata)
  );

  wire                                spec_insn_slti_valid;
  wire                                spec_insn_slti_trap;
  wire [                       4 : 0] spec_insn_slti_rs1_addr;
  wire [                       4 : 0] spec_insn_slti_rs2_addr;
  wire [                       4 : 0] spec_insn_slti_rd_addr;
  wire [32   - 1 : 0] spec_insn_slti_rd_wdata;
  wire [32   - 1 : 0] spec_insn_slti_pc_wdata;
  wire [32   - 1 : 0] spec_insn_slti_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_slti_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_slti_mem_wmask;
  wire [32   - 1 : 0] spec_insn_slti_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_slti insn_slti (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_slti_valid),
    .spec_trap(spec_insn_slti_trap),
    .spec_rs1_addr(spec_insn_slti_rs1_addr),
    .spec_rs2_addr(spec_insn_slti_rs2_addr),
    .spec_rd_addr(spec_insn_slti_rd_addr),
    .spec_rd_wdata(spec_insn_slti_rd_wdata),
    .spec_pc_wdata(spec_insn_slti_pc_wdata),
    .spec_mem_addr(spec_insn_slti_mem_addr),
    .spec_mem_rmask(spec_insn_slti_mem_rmask),
    .spec_mem_wmask(spec_insn_slti_mem_wmask),
    .spec_mem_wdata(spec_insn_slti_mem_wdata)
  );

  wire                                spec_insn_sltiu_valid;
  wire                                spec_insn_sltiu_trap;
  wire [                       4 : 0] spec_insn_sltiu_rs1_addr;
  wire [                       4 : 0] spec_insn_sltiu_rs2_addr;
  wire [                       4 : 0] spec_insn_sltiu_rd_addr;
  wire [32   - 1 : 0] spec_insn_sltiu_rd_wdata;
  wire [32   - 1 : 0] spec_insn_sltiu_pc_wdata;
  wire [32   - 1 : 0] spec_insn_sltiu_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_sltiu_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_sltiu_mem_wmask;
  wire [32   - 1 : 0] spec_insn_sltiu_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_sltiu insn_sltiu (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_sltiu_valid),
    .spec_trap(spec_insn_sltiu_trap),
    .spec_rs1_addr(spec_insn_sltiu_rs1_addr),
    .spec_rs2_addr(spec_insn_sltiu_rs2_addr),
    .spec_rd_addr(spec_insn_sltiu_rd_addr),
    .spec_rd_wdata(spec_insn_sltiu_rd_wdata),
    .spec_pc_wdata(spec_insn_sltiu_pc_wdata),
    .spec_mem_addr(spec_insn_sltiu_mem_addr),
    .spec_mem_rmask(spec_insn_sltiu_mem_rmask),
    .spec_mem_wmask(spec_insn_sltiu_mem_wmask),
    .spec_mem_wdata(spec_insn_sltiu_mem_wdata)
  );

  wire                                spec_insn_sltu_valid;
  wire                                spec_insn_sltu_trap;
  wire [                       4 : 0] spec_insn_sltu_rs1_addr;
  wire [                       4 : 0] spec_insn_sltu_rs2_addr;
  wire [                       4 : 0] spec_insn_sltu_rd_addr;
  wire [32   - 1 : 0] spec_insn_sltu_rd_wdata;
  wire [32   - 1 : 0] spec_insn_sltu_pc_wdata;
  wire [32   - 1 : 0] spec_insn_sltu_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_sltu_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_sltu_mem_wmask;
  wire [32   - 1 : 0] spec_insn_sltu_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_sltu insn_sltu (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_sltu_valid),
    .spec_trap(spec_insn_sltu_trap),
    .spec_rs1_addr(spec_insn_sltu_rs1_addr),
    .spec_rs2_addr(spec_insn_sltu_rs2_addr),
    .spec_rd_addr(spec_insn_sltu_rd_addr),
    .spec_rd_wdata(spec_insn_sltu_rd_wdata),
    .spec_pc_wdata(spec_insn_sltu_pc_wdata),
    .spec_mem_addr(spec_insn_sltu_mem_addr),
    .spec_mem_rmask(spec_insn_sltu_mem_rmask),
    .spec_mem_wmask(spec_insn_sltu_mem_wmask),
    .spec_mem_wdata(spec_insn_sltu_mem_wdata)
  );

  wire                                spec_insn_sra_valid;
  wire                                spec_insn_sra_trap;
  wire [                       4 : 0] spec_insn_sra_rs1_addr;
  wire [                       4 : 0] spec_insn_sra_rs2_addr;
  wire [                       4 : 0] spec_insn_sra_rd_addr;
  wire [32   - 1 : 0] spec_insn_sra_rd_wdata;
  wire [32   - 1 : 0] spec_insn_sra_pc_wdata;
  wire [32   - 1 : 0] spec_insn_sra_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_sra_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_sra_mem_wmask;
  wire [32   - 1 : 0] spec_insn_sra_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_sra insn_sra (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_sra_valid),
    .spec_trap(spec_insn_sra_trap),
    .spec_rs1_addr(spec_insn_sra_rs1_addr),
    .spec_rs2_addr(spec_insn_sra_rs2_addr),
    .spec_rd_addr(spec_insn_sra_rd_addr),
    .spec_rd_wdata(spec_insn_sra_rd_wdata),
    .spec_pc_wdata(spec_insn_sra_pc_wdata),
    .spec_mem_addr(spec_insn_sra_mem_addr),
    .spec_mem_rmask(spec_insn_sra_mem_rmask),
    .spec_mem_wmask(spec_insn_sra_mem_wmask),
    .spec_mem_wdata(spec_insn_sra_mem_wdata)
  );

  wire                                spec_insn_srai_valid;
  wire                                spec_insn_srai_trap;
  wire [                       4 : 0] spec_insn_srai_rs1_addr;
  wire [                       4 : 0] spec_insn_srai_rs2_addr;
  wire [                       4 : 0] spec_insn_srai_rd_addr;
  wire [32   - 1 : 0] spec_insn_srai_rd_wdata;
  wire [32   - 1 : 0] spec_insn_srai_pc_wdata;
  wire [32   - 1 : 0] spec_insn_srai_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_srai_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_srai_mem_wmask;
  wire [32   - 1 : 0] spec_insn_srai_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_srai insn_srai (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_srai_valid),
    .spec_trap(spec_insn_srai_trap),
    .spec_rs1_addr(spec_insn_srai_rs1_addr),
    .spec_rs2_addr(spec_insn_srai_rs2_addr),
    .spec_rd_addr(spec_insn_srai_rd_addr),
    .spec_rd_wdata(spec_insn_srai_rd_wdata),
    .spec_pc_wdata(spec_insn_srai_pc_wdata),
    .spec_mem_addr(spec_insn_srai_mem_addr),
    .spec_mem_rmask(spec_insn_srai_mem_rmask),
    .spec_mem_wmask(spec_insn_srai_mem_wmask),
    .spec_mem_wdata(spec_insn_srai_mem_wdata)
  );

  wire                                spec_insn_srl_valid;
  wire                                spec_insn_srl_trap;
  wire [                       4 : 0] spec_insn_srl_rs1_addr;
  wire [                       4 : 0] spec_insn_srl_rs2_addr;
  wire [                       4 : 0] spec_insn_srl_rd_addr;
  wire [32   - 1 : 0] spec_insn_srl_rd_wdata;
  wire [32   - 1 : 0] spec_insn_srl_pc_wdata;
  wire [32   - 1 : 0] spec_insn_srl_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_srl_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_srl_mem_wmask;
  wire [32   - 1 : 0] spec_insn_srl_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_srl insn_srl (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_srl_valid),
    .spec_trap(spec_insn_srl_trap),
    .spec_rs1_addr(spec_insn_srl_rs1_addr),
    .spec_rs2_addr(spec_insn_srl_rs2_addr),
    .spec_rd_addr(spec_insn_srl_rd_addr),
    .spec_rd_wdata(spec_insn_srl_rd_wdata),
    .spec_pc_wdata(spec_insn_srl_pc_wdata),
    .spec_mem_addr(spec_insn_srl_mem_addr),
    .spec_mem_rmask(spec_insn_srl_mem_rmask),
    .spec_mem_wmask(spec_insn_srl_mem_wmask),
    .spec_mem_wdata(spec_insn_srl_mem_wdata)
  );

  wire                                spec_insn_srli_valid;
  wire                                spec_insn_srli_trap;
  wire [                       4 : 0] spec_insn_srli_rs1_addr;
  wire [                       4 : 0] spec_insn_srli_rs2_addr;
  wire [                       4 : 0] spec_insn_srli_rd_addr;
  wire [32   - 1 : 0] spec_insn_srli_rd_wdata;
  wire [32   - 1 : 0] spec_insn_srli_pc_wdata;
  wire [32   - 1 : 0] spec_insn_srli_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_srli_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_srli_mem_wmask;
  wire [32   - 1 : 0] spec_insn_srli_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_srli insn_srli (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_srli_valid),
    .spec_trap(spec_insn_srli_trap),
    .spec_rs1_addr(spec_insn_srli_rs1_addr),
    .spec_rs2_addr(spec_insn_srli_rs2_addr),
    .spec_rd_addr(spec_insn_srli_rd_addr),
    .spec_rd_wdata(spec_insn_srli_rd_wdata),
    .spec_pc_wdata(spec_insn_srli_pc_wdata),
    .spec_mem_addr(spec_insn_srli_mem_addr),
    .spec_mem_rmask(spec_insn_srli_mem_rmask),
    .spec_mem_wmask(spec_insn_srli_mem_wmask),
    .spec_mem_wdata(spec_insn_srli_mem_wdata)
  );

  wire                                spec_insn_sub_valid;
  wire                                spec_insn_sub_trap;
  wire [                       4 : 0] spec_insn_sub_rs1_addr;
  wire [                       4 : 0] spec_insn_sub_rs2_addr;
  wire [                       4 : 0] spec_insn_sub_rd_addr;
  wire [32   - 1 : 0] spec_insn_sub_rd_wdata;
  wire [32   - 1 : 0] spec_insn_sub_pc_wdata;
  wire [32   - 1 : 0] spec_insn_sub_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_sub_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_sub_mem_wmask;
  wire [32   - 1 : 0] spec_insn_sub_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_sub insn_sub (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_sub_valid),
    .spec_trap(spec_insn_sub_trap),
    .spec_rs1_addr(spec_insn_sub_rs1_addr),
    .spec_rs2_addr(spec_insn_sub_rs2_addr),
    .spec_rd_addr(spec_insn_sub_rd_addr),
    .spec_rd_wdata(spec_insn_sub_rd_wdata),
    .spec_pc_wdata(spec_insn_sub_pc_wdata),
    .spec_mem_addr(spec_insn_sub_mem_addr),
    .spec_mem_rmask(spec_insn_sub_mem_rmask),
    .spec_mem_wmask(spec_insn_sub_mem_wmask),
    .spec_mem_wdata(spec_insn_sub_mem_wdata)
  );

  wire                                spec_insn_sw_valid;
  wire                                spec_insn_sw_trap;
  wire [                       4 : 0] spec_insn_sw_rs1_addr;
  wire [                       4 : 0] spec_insn_sw_rs2_addr;
  wire [                       4 : 0] spec_insn_sw_rd_addr;
  wire [32   - 1 : 0] spec_insn_sw_rd_wdata;
  wire [32   - 1 : 0] spec_insn_sw_pc_wdata;
  wire [32   - 1 : 0] spec_insn_sw_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_sw_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_sw_mem_wmask;
  wire [32   - 1 : 0] spec_insn_sw_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_sw insn_sw (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_sw_valid),
    .spec_trap(spec_insn_sw_trap),
    .spec_rs1_addr(spec_insn_sw_rs1_addr),
    .spec_rs2_addr(spec_insn_sw_rs2_addr),
    .spec_rd_addr(spec_insn_sw_rd_addr),
    .spec_rd_wdata(spec_insn_sw_rd_wdata),
    .spec_pc_wdata(spec_insn_sw_pc_wdata),
    .spec_mem_addr(spec_insn_sw_mem_addr),
    .spec_mem_rmask(spec_insn_sw_mem_rmask),
    .spec_mem_wmask(spec_insn_sw_mem_wmask),
    .spec_mem_wdata(spec_insn_sw_mem_wdata)
  );

  wire                                spec_insn_xor_valid;
  wire                                spec_insn_xor_trap;
  wire [                       4 : 0] spec_insn_xor_rs1_addr;
  wire [                       4 : 0] spec_insn_xor_rs2_addr;
  wire [                       4 : 0] spec_insn_xor_rd_addr;
  wire [32   - 1 : 0] spec_insn_xor_rd_wdata;
  wire [32   - 1 : 0] spec_insn_xor_pc_wdata;
  wire [32   - 1 : 0] spec_insn_xor_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_xor_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_xor_mem_wmask;
  wire [32   - 1 : 0] spec_insn_xor_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_xor insn_xor (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_xor_valid),
    .spec_trap(spec_insn_xor_trap),
    .spec_rs1_addr(spec_insn_xor_rs1_addr),
    .spec_rs2_addr(spec_insn_xor_rs2_addr),
    .spec_rd_addr(spec_insn_xor_rd_addr),
    .spec_rd_wdata(spec_insn_xor_rd_wdata),
    .spec_pc_wdata(spec_insn_xor_pc_wdata),
    .spec_mem_addr(spec_insn_xor_mem_addr),
    .spec_mem_rmask(spec_insn_xor_mem_rmask),
    .spec_mem_wmask(spec_insn_xor_mem_wmask),
    .spec_mem_wdata(spec_insn_xor_mem_wdata)
  );

  wire                                spec_insn_xori_valid;
  wire                                spec_insn_xori_trap;
  wire [                       4 : 0] spec_insn_xori_rs1_addr;
  wire [                       4 : 0] spec_insn_xori_rs2_addr;
  wire [                       4 : 0] spec_insn_xori_rd_addr;
  wire [32   - 1 : 0] spec_insn_xori_rd_wdata;
  wire [32   - 1 : 0] spec_insn_xori_pc_wdata;
  wire [32   - 1 : 0] spec_insn_xori_mem_addr;
  wire [32/8 - 1 : 0] spec_insn_xori_mem_rmask;
  wire [32/8 - 1 : 0] spec_insn_xori_mem_wmask;
  wire [32   - 1 : 0] spec_insn_xori_mem_wdata;

  riscv_formal_monitor_rv32imc_insn_xori insn_xori (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_insn_xori_valid),
    .spec_trap(spec_insn_xori_trap),
    .spec_rs1_addr(spec_insn_xori_rs1_addr),
    .spec_rs2_addr(spec_insn_xori_rs2_addr),
    .spec_rd_addr(spec_insn_xori_rd_addr),
    .spec_rd_wdata(spec_insn_xori_rd_wdata),
    .spec_pc_wdata(spec_insn_xori_pc_wdata),
    .spec_mem_addr(spec_insn_xori_mem_addr),
    .spec_mem_rmask(spec_insn_xori_mem_rmask),
    .spec_mem_wmask(spec_insn_xori_mem_wmask),
    .spec_mem_wdata(spec_insn_xori_mem_wdata)
  );

  assign spec_valid =
		spec_insn_add_valid ? spec_insn_add_valid :
		spec_insn_addi_valid ? spec_insn_addi_valid :
		spec_insn_and_valid ? spec_insn_and_valid :
		spec_insn_andi_valid ? spec_insn_andi_valid :
		spec_insn_auipc_valid ? spec_insn_auipc_valid :
		spec_insn_beq_valid ? spec_insn_beq_valid :
		spec_insn_bge_valid ? spec_insn_bge_valid :
		spec_insn_bgeu_valid ? spec_insn_bgeu_valid :
		spec_insn_blt_valid ? spec_insn_blt_valid :
		spec_insn_bltu_valid ? spec_insn_bltu_valid :
		spec_insn_bne_valid ? spec_insn_bne_valid :
		spec_insn_c_add_valid ? spec_insn_c_add_valid :
		spec_insn_c_addi_valid ? spec_insn_c_addi_valid :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_valid :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_valid :
		spec_insn_c_and_valid ? spec_insn_c_and_valid :
		spec_insn_c_andi_valid ? spec_insn_c_andi_valid :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_valid :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_valid :
		spec_insn_c_j_valid ? spec_insn_c_j_valid :
		spec_insn_c_jal_valid ? spec_insn_c_jal_valid :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_valid :
		spec_insn_c_jr_valid ? spec_insn_c_jr_valid :
		spec_insn_c_li_valid ? spec_insn_c_li_valid :
		spec_insn_c_lui_valid ? spec_insn_c_lui_valid :
		spec_insn_c_lw_valid ? spec_insn_c_lw_valid :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_valid :
		spec_insn_c_mv_valid ? spec_insn_c_mv_valid :
		spec_insn_c_or_valid ? spec_insn_c_or_valid :
		spec_insn_c_slli_valid ? spec_insn_c_slli_valid :
		spec_insn_c_srai_valid ? spec_insn_c_srai_valid :
		spec_insn_c_srli_valid ? spec_insn_c_srli_valid :
		spec_insn_c_sub_valid ? spec_insn_c_sub_valid :
		spec_insn_c_sw_valid ? spec_insn_c_sw_valid :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_valid :
		spec_insn_c_xor_valid ? spec_insn_c_xor_valid :
		spec_insn_div_valid ? spec_insn_div_valid :
		spec_insn_divu_valid ? spec_insn_divu_valid :
		spec_insn_jal_valid ? spec_insn_jal_valid :
		spec_insn_jalr_valid ? spec_insn_jalr_valid :
		spec_insn_lb_valid ? spec_insn_lb_valid :
		spec_insn_lbu_valid ? spec_insn_lbu_valid :
		spec_insn_lh_valid ? spec_insn_lh_valid :
		spec_insn_lhu_valid ? spec_insn_lhu_valid :
		spec_insn_lui_valid ? spec_insn_lui_valid :
		spec_insn_lw_valid ? spec_insn_lw_valid :
		spec_insn_mul_valid ? spec_insn_mul_valid :
		spec_insn_mulh_valid ? spec_insn_mulh_valid :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_valid :
		spec_insn_mulhu_valid ? spec_insn_mulhu_valid :
		spec_insn_or_valid ? spec_insn_or_valid :
		spec_insn_ori_valid ? spec_insn_ori_valid :
		spec_insn_rem_valid ? spec_insn_rem_valid :
		spec_insn_remu_valid ? spec_insn_remu_valid :
		spec_insn_sb_valid ? spec_insn_sb_valid :
		spec_insn_sh_valid ? spec_insn_sh_valid :
		spec_insn_sll_valid ? spec_insn_sll_valid :
		spec_insn_slli_valid ? spec_insn_slli_valid :
		spec_insn_slt_valid ? spec_insn_slt_valid :
		spec_insn_slti_valid ? spec_insn_slti_valid :
		spec_insn_sltiu_valid ? spec_insn_sltiu_valid :
		spec_insn_sltu_valid ? spec_insn_sltu_valid :
		spec_insn_sra_valid ? spec_insn_sra_valid :
		spec_insn_srai_valid ? spec_insn_srai_valid :
		spec_insn_srl_valid ? spec_insn_srl_valid :
		spec_insn_srli_valid ? spec_insn_srli_valid :
		spec_insn_sub_valid ? spec_insn_sub_valid :
		spec_insn_sw_valid ? spec_insn_sw_valid :
		spec_insn_xor_valid ? spec_insn_xor_valid :
		spec_insn_xori_valid ? spec_insn_xori_valid : 0;
  assign spec_trap =
		spec_insn_add_valid ? spec_insn_add_trap :
		spec_insn_addi_valid ? spec_insn_addi_trap :
		spec_insn_and_valid ? spec_insn_and_trap :
		spec_insn_andi_valid ? spec_insn_andi_trap :
		spec_insn_auipc_valid ? spec_insn_auipc_trap :
		spec_insn_beq_valid ? spec_insn_beq_trap :
		spec_insn_bge_valid ? spec_insn_bge_trap :
		spec_insn_bgeu_valid ? spec_insn_bgeu_trap :
		spec_insn_blt_valid ? spec_insn_blt_trap :
		spec_insn_bltu_valid ? spec_insn_bltu_trap :
		spec_insn_bne_valid ? spec_insn_bne_trap :
		spec_insn_c_add_valid ? spec_insn_c_add_trap :
		spec_insn_c_addi_valid ? spec_insn_c_addi_trap :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_trap :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_trap :
		spec_insn_c_and_valid ? spec_insn_c_and_trap :
		spec_insn_c_andi_valid ? spec_insn_c_andi_trap :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_trap :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_trap :
		spec_insn_c_j_valid ? spec_insn_c_j_trap :
		spec_insn_c_jal_valid ? spec_insn_c_jal_trap :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_trap :
		spec_insn_c_jr_valid ? spec_insn_c_jr_trap :
		spec_insn_c_li_valid ? spec_insn_c_li_trap :
		spec_insn_c_lui_valid ? spec_insn_c_lui_trap :
		spec_insn_c_lw_valid ? spec_insn_c_lw_trap :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_trap :
		spec_insn_c_mv_valid ? spec_insn_c_mv_trap :
		spec_insn_c_or_valid ? spec_insn_c_or_trap :
		spec_insn_c_slli_valid ? spec_insn_c_slli_trap :
		spec_insn_c_srai_valid ? spec_insn_c_srai_trap :
		spec_insn_c_srli_valid ? spec_insn_c_srli_trap :
		spec_insn_c_sub_valid ? spec_insn_c_sub_trap :
		spec_insn_c_sw_valid ? spec_insn_c_sw_trap :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_trap :
		spec_insn_c_xor_valid ? spec_insn_c_xor_trap :
		spec_insn_div_valid ? spec_insn_div_trap :
		spec_insn_divu_valid ? spec_insn_divu_trap :
		spec_insn_jal_valid ? spec_insn_jal_trap :
		spec_insn_jalr_valid ? spec_insn_jalr_trap :
		spec_insn_lb_valid ? spec_insn_lb_trap :
		spec_insn_lbu_valid ? spec_insn_lbu_trap :
		spec_insn_lh_valid ? spec_insn_lh_trap :
		spec_insn_lhu_valid ? spec_insn_lhu_trap :
		spec_insn_lui_valid ? spec_insn_lui_trap :
		spec_insn_lw_valid ? spec_insn_lw_trap :
		spec_insn_mul_valid ? spec_insn_mul_trap :
		spec_insn_mulh_valid ? spec_insn_mulh_trap :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_trap :
		spec_insn_mulhu_valid ? spec_insn_mulhu_trap :
		spec_insn_or_valid ? spec_insn_or_trap :
		spec_insn_ori_valid ? spec_insn_ori_trap :
		spec_insn_rem_valid ? spec_insn_rem_trap :
		spec_insn_remu_valid ? spec_insn_remu_trap :
		spec_insn_sb_valid ? spec_insn_sb_trap :
		spec_insn_sh_valid ? spec_insn_sh_trap :
		spec_insn_sll_valid ? spec_insn_sll_trap :
		spec_insn_slli_valid ? spec_insn_slli_trap :
		spec_insn_slt_valid ? spec_insn_slt_trap :
		spec_insn_slti_valid ? spec_insn_slti_trap :
		spec_insn_sltiu_valid ? spec_insn_sltiu_trap :
		spec_insn_sltu_valid ? spec_insn_sltu_trap :
		spec_insn_sra_valid ? spec_insn_sra_trap :
		spec_insn_srai_valid ? spec_insn_srai_trap :
		spec_insn_srl_valid ? spec_insn_srl_trap :
		spec_insn_srli_valid ? spec_insn_srli_trap :
		spec_insn_sub_valid ? spec_insn_sub_trap :
		spec_insn_sw_valid ? spec_insn_sw_trap :
		spec_insn_xor_valid ? spec_insn_xor_trap :
		spec_insn_xori_valid ? spec_insn_xori_trap : 0;
  assign spec_rs1_addr =
		spec_insn_add_valid ? spec_insn_add_rs1_addr :
		spec_insn_addi_valid ? spec_insn_addi_rs1_addr :
		spec_insn_and_valid ? spec_insn_and_rs1_addr :
		spec_insn_andi_valid ? spec_insn_andi_rs1_addr :
		spec_insn_auipc_valid ? spec_insn_auipc_rs1_addr :
		spec_insn_beq_valid ? spec_insn_beq_rs1_addr :
		spec_insn_bge_valid ? spec_insn_bge_rs1_addr :
		spec_insn_bgeu_valid ? spec_insn_bgeu_rs1_addr :
		spec_insn_blt_valid ? spec_insn_blt_rs1_addr :
		spec_insn_bltu_valid ? spec_insn_bltu_rs1_addr :
		spec_insn_bne_valid ? spec_insn_bne_rs1_addr :
		spec_insn_c_add_valid ? spec_insn_c_add_rs1_addr :
		spec_insn_c_addi_valid ? spec_insn_c_addi_rs1_addr :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_rs1_addr :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_rs1_addr :
		spec_insn_c_and_valid ? spec_insn_c_and_rs1_addr :
		spec_insn_c_andi_valid ? spec_insn_c_andi_rs1_addr :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_rs1_addr :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_rs1_addr :
		spec_insn_c_j_valid ? spec_insn_c_j_rs1_addr :
		spec_insn_c_jal_valid ? spec_insn_c_jal_rs1_addr :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_rs1_addr :
		spec_insn_c_jr_valid ? spec_insn_c_jr_rs1_addr :
		spec_insn_c_li_valid ? spec_insn_c_li_rs1_addr :
		spec_insn_c_lui_valid ? spec_insn_c_lui_rs1_addr :
		spec_insn_c_lw_valid ? spec_insn_c_lw_rs1_addr :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_rs1_addr :
		spec_insn_c_mv_valid ? spec_insn_c_mv_rs1_addr :
		spec_insn_c_or_valid ? spec_insn_c_or_rs1_addr :
		spec_insn_c_slli_valid ? spec_insn_c_slli_rs1_addr :
		spec_insn_c_srai_valid ? spec_insn_c_srai_rs1_addr :
		spec_insn_c_srli_valid ? spec_insn_c_srli_rs1_addr :
		spec_insn_c_sub_valid ? spec_insn_c_sub_rs1_addr :
		spec_insn_c_sw_valid ? spec_insn_c_sw_rs1_addr :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_rs1_addr :
		spec_insn_c_xor_valid ? spec_insn_c_xor_rs1_addr :
		spec_insn_div_valid ? spec_insn_div_rs1_addr :
		spec_insn_divu_valid ? spec_insn_divu_rs1_addr :
		spec_insn_jal_valid ? spec_insn_jal_rs1_addr :
		spec_insn_jalr_valid ? spec_insn_jalr_rs1_addr :
		spec_insn_lb_valid ? spec_insn_lb_rs1_addr :
		spec_insn_lbu_valid ? spec_insn_lbu_rs1_addr :
		spec_insn_lh_valid ? spec_insn_lh_rs1_addr :
		spec_insn_lhu_valid ? spec_insn_lhu_rs1_addr :
		spec_insn_lui_valid ? spec_insn_lui_rs1_addr :
		spec_insn_lw_valid ? spec_insn_lw_rs1_addr :
		spec_insn_mul_valid ? spec_insn_mul_rs1_addr :
		spec_insn_mulh_valid ? spec_insn_mulh_rs1_addr :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_rs1_addr :
		spec_insn_mulhu_valid ? spec_insn_mulhu_rs1_addr :
		spec_insn_or_valid ? spec_insn_or_rs1_addr :
		spec_insn_ori_valid ? spec_insn_ori_rs1_addr :
		spec_insn_rem_valid ? spec_insn_rem_rs1_addr :
		spec_insn_remu_valid ? spec_insn_remu_rs1_addr :
		spec_insn_sb_valid ? spec_insn_sb_rs1_addr :
		spec_insn_sh_valid ? spec_insn_sh_rs1_addr :
		spec_insn_sll_valid ? spec_insn_sll_rs1_addr :
		spec_insn_slli_valid ? spec_insn_slli_rs1_addr :
		spec_insn_slt_valid ? spec_insn_slt_rs1_addr :
		spec_insn_slti_valid ? spec_insn_slti_rs1_addr :
		spec_insn_sltiu_valid ? spec_insn_sltiu_rs1_addr :
		spec_insn_sltu_valid ? spec_insn_sltu_rs1_addr :
		spec_insn_sra_valid ? spec_insn_sra_rs1_addr :
		spec_insn_srai_valid ? spec_insn_srai_rs1_addr :
		spec_insn_srl_valid ? spec_insn_srl_rs1_addr :
		spec_insn_srli_valid ? spec_insn_srli_rs1_addr :
		spec_insn_sub_valid ? spec_insn_sub_rs1_addr :
		spec_insn_sw_valid ? spec_insn_sw_rs1_addr :
		spec_insn_xor_valid ? spec_insn_xor_rs1_addr :
		spec_insn_xori_valid ? spec_insn_xori_rs1_addr : 0;
  assign spec_rs2_addr =
		spec_insn_add_valid ? spec_insn_add_rs2_addr :
		spec_insn_addi_valid ? spec_insn_addi_rs2_addr :
		spec_insn_and_valid ? spec_insn_and_rs2_addr :
		spec_insn_andi_valid ? spec_insn_andi_rs2_addr :
		spec_insn_auipc_valid ? spec_insn_auipc_rs2_addr :
		spec_insn_beq_valid ? spec_insn_beq_rs2_addr :
		spec_insn_bge_valid ? spec_insn_bge_rs2_addr :
		spec_insn_bgeu_valid ? spec_insn_bgeu_rs2_addr :
		spec_insn_blt_valid ? spec_insn_blt_rs2_addr :
		spec_insn_bltu_valid ? spec_insn_bltu_rs2_addr :
		spec_insn_bne_valid ? spec_insn_bne_rs2_addr :
		spec_insn_c_add_valid ? spec_insn_c_add_rs2_addr :
		spec_insn_c_addi_valid ? spec_insn_c_addi_rs2_addr :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_rs2_addr :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_rs2_addr :
		spec_insn_c_and_valid ? spec_insn_c_and_rs2_addr :
		spec_insn_c_andi_valid ? spec_insn_c_andi_rs2_addr :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_rs2_addr :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_rs2_addr :
		spec_insn_c_j_valid ? spec_insn_c_j_rs2_addr :
		spec_insn_c_jal_valid ? spec_insn_c_jal_rs2_addr :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_rs2_addr :
		spec_insn_c_jr_valid ? spec_insn_c_jr_rs2_addr :
		spec_insn_c_li_valid ? spec_insn_c_li_rs2_addr :
		spec_insn_c_lui_valid ? spec_insn_c_lui_rs2_addr :
		spec_insn_c_lw_valid ? spec_insn_c_lw_rs2_addr :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_rs2_addr :
		spec_insn_c_mv_valid ? spec_insn_c_mv_rs2_addr :
		spec_insn_c_or_valid ? spec_insn_c_or_rs2_addr :
		spec_insn_c_slli_valid ? spec_insn_c_slli_rs2_addr :
		spec_insn_c_srai_valid ? spec_insn_c_srai_rs2_addr :
		spec_insn_c_srli_valid ? spec_insn_c_srli_rs2_addr :
		spec_insn_c_sub_valid ? spec_insn_c_sub_rs2_addr :
		spec_insn_c_sw_valid ? spec_insn_c_sw_rs2_addr :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_rs2_addr :
		spec_insn_c_xor_valid ? spec_insn_c_xor_rs2_addr :
		spec_insn_div_valid ? spec_insn_div_rs2_addr :
		spec_insn_divu_valid ? spec_insn_divu_rs2_addr :
		spec_insn_jal_valid ? spec_insn_jal_rs2_addr :
		spec_insn_jalr_valid ? spec_insn_jalr_rs2_addr :
		spec_insn_lb_valid ? spec_insn_lb_rs2_addr :
		spec_insn_lbu_valid ? spec_insn_lbu_rs2_addr :
		spec_insn_lh_valid ? spec_insn_lh_rs2_addr :
		spec_insn_lhu_valid ? spec_insn_lhu_rs2_addr :
		spec_insn_lui_valid ? spec_insn_lui_rs2_addr :
		spec_insn_lw_valid ? spec_insn_lw_rs2_addr :
		spec_insn_mul_valid ? spec_insn_mul_rs2_addr :
		spec_insn_mulh_valid ? spec_insn_mulh_rs2_addr :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_rs2_addr :
		spec_insn_mulhu_valid ? spec_insn_mulhu_rs2_addr :
		spec_insn_or_valid ? spec_insn_or_rs2_addr :
		spec_insn_ori_valid ? spec_insn_ori_rs2_addr :
		spec_insn_rem_valid ? spec_insn_rem_rs2_addr :
		spec_insn_remu_valid ? spec_insn_remu_rs2_addr :
		spec_insn_sb_valid ? spec_insn_sb_rs2_addr :
		spec_insn_sh_valid ? spec_insn_sh_rs2_addr :
		spec_insn_sll_valid ? spec_insn_sll_rs2_addr :
		spec_insn_slli_valid ? spec_insn_slli_rs2_addr :
		spec_insn_slt_valid ? spec_insn_slt_rs2_addr :
		spec_insn_slti_valid ? spec_insn_slti_rs2_addr :
		spec_insn_sltiu_valid ? spec_insn_sltiu_rs2_addr :
		spec_insn_sltu_valid ? spec_insn_sltu_rs2_addr :
		spec_insn_sra_valid ? spec_insn_sra_rs2_addr :
		spec_insn_srai_valid ? spec_insn_srai_rs2_addr :
		spec_insn_srl_valid ? spec_insn_srl_rs2_addr :
		spec_insn_srli_valid ? spec_insn_srli_rs2_addr :
		spec_insn_sub_valid ? spec_insn_sub_rs2_addr :
		spec_insn_sw_valid ? spec_insn_sw_rs2_addr :
		spec_insn_xor_valid ? spec_insn_xor_rs2_addr :
		spec_insn_xori_valid ? spec_insn_xori_rs2_addr : 0;
  assign spec_rd_addr =
		spec_insn_add_valid ? spec_insn_add_rd_addr :
		spec_insn_addi_valid ? spec_insn_addi_rd_addr :
		spec_insn_and_valid ? spec_insn_and_rd_addr :
		spec_insn_andi_valid ? spec_insn_andi_rd_addr :
		spec_insn_auipc_valid ? spec_insn_auipc_rd_addr :
		spec_insn_beq_valid ? spec_insn_beq_rd_addr :
		spec_insn_bge_valid ? spec_insn_bge_rd_addr :
		spec_insn_bgeu_valid ? spec_insn_bgeu_rd_addr :
		spec_insn_blt_valid ? spec_insn_blt_rd_addr :
		spec_insn_bltu_valid ? spec_insn_bltu_rd_addr :
		spec_insn_bne_valid ? spec_insn_bne_rd_addr :
		spec_insn_c_add_valid ? spec_insn_c_add_rd_addr :
		spec_insn_c_addi_valid ? spec_insn_c_addi_rd_addr :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_rd_addr :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_rd_addr :
		spec_insn_c_and_valid ? spec_insn_c_and_rd_addr :
		spec_insn_c_andi_valid ? spec_insn_c_andi_rd_addr :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_rd_addr :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_rd_addr :
		spec_insn_c_j_valid ? spec_insn_c_j_rd_addr :
		spec_insn_c_jal_valid ? spec_insn_c_jal_rd_addr :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_rd_addr :
		spec_insn_c_jr_valid ? spec_insn_c_jr_rd_addr :
		spec_insn_c_li_valid ? spec_insn_c_li_rd_addr :
		spec_insn_c_lui_valid ? spec_insn_c_lui_rd_addr :
		spec_insn_c_lw_valid ? spec_insn_c_lw_rd_addr :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_rd_addr :
		spec_insn_c_mv_valid ? spec_insn_c_mv_rd_addr :
		spec_insn_c_or_valid ? spec_insn_c_or_rd_addr :
		spec_insn_c_slli_valid ? spec_insn_c_slli_rd_addr :
		spec_insn_c_srai_valid ? spec_insn_c_srai_rd_addr :
		spec_insn_c_srli_valid ? spec_insn_c_srli_rd_addr :
		spec_insn_c_sub_valid ? spec_insn_c_sub_rd_addr :
		spec_insn_c_sw_valid ? spec_insn_c_sw_rd_addr :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_rd_addr :
		spec_insn_c_xor_valid ? spec_insn_c_xor_rd_addr :
		spec_insn_div_valid ? spec_insn_div_rd_addr :
		spec_insn_divu_valid ? spec_insn_divu_rd_addr :
		spec_insn_jal_valid ? spec_insn_jal_rd_addr :
		spec_insn_jalr_valid ? spec_insn_jalr_rd_addr :
		spec_insn_lb_valid ? spec_insn_lb_rd_addr :
		spec_insn_lbu_valid ? spec_insn_lbu_rd_addr :
		spec_insn_lh_valid ? spec_insn_lh_rd_addr :
		spec_insn_lhu_valid ? spec_insn_lhu_rd_addr :
		spec_insn_lui_valid ? spec_insn_lui_rd_addr :
		spec_insn_lw_valid ? spec_insn_lw_rd_addr :
		spec_insn_mul_valid ? spec_insn_mul_rd_addr :
		spec_insn_mulh_valid ? spec_insn_mulh_rd_addr :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_rd_addr :
		spec_insn_mulhu_valid ? spec_insn_mulhu_rd_addr :
		spec_insn_or_valid ? spec_insn_or_rd_addr :
		spec_insn_ori_valid ? spec_insn_ori_rd_addr :
		spec_insn_rem_valid ? spec_insn_rem_rd_addr :
		spec_insn_remu_valid ? spec_insn_remu_rd_addr :
		spec_insn_sb_valid ? spec_insn_sb_rd_addr :
		spec_insn_sh_valid ? spec_insn_sh_rd_addr :
		spec_insn_sll_valid ? spec_insn_sll_rd_addr :
		spec_insn_slli_valid ? spec_insn_slli_rd_addr :
		spec_insn_slt_valid ? spec_insn_slt_rd_addr :
		spec_insn_slti_valid ? spec_insn_slti_rd_addr :
		spec_insn_sltiu_valid ? spec_insn_sltiu_rd_addr :
		spec_insn_sltu_valid ? spec_insn_sltu_rd_addr :
		spec_insn_sra_valid ? spec_insn_sra_rd_addr :
		spec_insn_srai_valid ? spec_insn_srai_rd_addr :
		spec_insn_srl_valid ? spec_insn_srl_rd_addr :
		spec_insn_srli_valid ? spec_insn_srli_rd_addr :
		spec_insn_sub_valid ? spec_insn_sub_rd_addr :
		spec_insn_sw_valid ? spec_insn_sw_rd_addr :
		spec_insn_xor_valid ? spec_insn_xor_rd_addr :
		spec_insn_xori_valid ? spec_insn_xori_rd_addr : 0;
  assign spec_rd_wdata =
		spec_insn_add_valid ? spec_insn_add_rd_wdata :
		spec_insn_addi_valid ? spec_insn_addi_rd_wdata :
		spec_insn_and_valid ? spec_insn_and_rd_wdata :
		spec_insn_andi_valid ? spec_insn_andi_rd_wdata :
		spec_insn_auipc_valid ? spec_insn_auipc_rd_wdata :
		spec_insn_beq_valid ? spec_insn_beq_rd_wdata :
		spec_insn_bge_valid ? spec_insn_bge_rd_wdata :
		spec_insn_bgeu_valid ? spec_insn_bgeu_rd_wdata :
		spec_insn_blt_valid ? spec_insn_blt_rd_wdata :
		spec_insn_bltu_valid ? spec_insn_bltu_rd_wdata :
		spec_insn_bne_valid ? spec_insn_bne_rd_wdata :
		spec_insn_c_add_valid ? spec_insn_c_add_rd_wdata :
		spec_insn_c_addi_valid ? spec_insn_c_addi_rd_wdata :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_rd_wdata :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_rd_wdata :
		spec_insn_c_and_valid ? spec_insn_c_and_rd_wdata :
		spec_insn_c_andi_valid ? spec_insn_c_andi_rd_wdata :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_rd_wdata :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_rd_wdata :
		spec_insn_c_j_valid ? spec_insn_c_j_rd_wdata :
		spec_insn_c_jal_valid ? spec_insn_c_jal_rd_wdata :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_rd_wdata :
		spec_insn_c_jr_valid ? spec_insn_c_jr_rd_wdata :
		spec_insn_c_li_valid ? spec_insn_c_li_rd_wdata :
		spec_insn_c_lui_valid ? spec_insn_c_lui_rd_wdata :
		spec_insn_c_lw_valid ? spec_insn_c_lw_rd_wdata :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_rd_wdata :
		spec_insn_c_mv_valid ? spec_insn_c_mv_rd_wdata :
		spec_insn_c_or_valid ? spec_insn_c_or_rd_wdata :
		spec_insn_c_slli_valid ? spec_insn_c_slli_rd_wdata :
		spec_insn_c_srai_valid ? spec_insn_c_srai_rd_wdata :
		spec_insn_c_srli_valid ? spec_insn_c_srli_rd_wdata :
		spec_insn_c_sub_valid ? spec_insn_c_sub_rd_wdata :
		spec_insn_c_sw_valid ? spec_insn_c_sw_rd_wdata :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_rd_wdata :
		spec_insn_c_xor_valid ? spec_insn_c_xor_rd_wdata :
		spec_insn_div_valid ? spec_insn_div_rd_wdata :
		spec_insn_divu_valid ? spec_insn_divu_rd_wdata :
		spec_insn_jal_valid ? spec_insn_jal_rd_wdata :
		spec_insn_jalr_valid ? spec_insn_jalr_rd_wdata :
		spec_insn_lb_valid ? spec_insn_lb_rd_wdata :
		spec_insn_lbu_valid ? spec_insn_lbu_rd_wdata :
		spec_insn_lh_valid ? spec_insn_lh_rd_wdata :
		spec_insn_lhu_valid ? spec_insn_lhu_rd_wdata :
		spec_insn_lui_valid ? spec_insn_lui_rd_wdata :
		spec_insn_lw_valid ? spec_insn_lw_rd_wdata :
		spec_insn_mul_valid ? spec_insn_mul_rd_wdata :
		spec_insn_mulh_valid ? spec_insn_mulh_rd_wdata :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_rd_wdata :
		spec_insn_mulhu_valid ? spec_insn_mulhu_rd_wdata :
		spec_insn_or_valid ? spec_insn_or_rd_wdata :
		spec_insn_ori_valid ? spec_insn_ori_rd_wdata :
		spec_insn_rem_valid ? spec_insn_rem_rd_wdata :
		spec_insn_remu_valid ? spec_insn_remu_rd_wdata :
		spec_insn_sb_valid ? spec_insn_sb_rd_wdata :
		spec_insn_sh_valid ? spec_insn_sh_rd_wdata :
		spec_insn_sll_valid ? spec_insn_sll_rd_wdata :
		spec_insn_slli_valid ? spec_insn_slli_rd_wdata :
		spec_insn_slt_valid ? spec_insn_slt_rd_wdata :
		spec_insn_slti_valid ? spec_insn_slti_rd_wdata :
		spec_insn_sltiu_valid ? spec_insn_sltiu_rd_wdata :
		spec_insn_sltu_valid ? spec_insn_sltu_rd_wdata :
		spec_insn_sra_valid ? spec_insn_sra_rd_wdata :
		spec_insn_srai_valid ? spec_insn_srai_rd_wdata :
		spec_insn_srl_valid ? spec_insn_srl_rd_wdata :
		spec_insn_srli_valid ? spec_insn_srli_rd_wdata :
		spec_insn_sub_valid ? spec_insn_sub_rd_wdata :
		spec_insn_sw_valid ? spec_insn_sw_rd_wdata :
		spec_insn_xor_valid ? spec_insn_xor_rd_wdata :
		spec_insn_xori_valid ? spec_insn_xori_rd_wdata : 0;
  assign spec_pc_wdata =
		spec_insn_add_valid ? spec_insn_add_pc_wdata :
		spec_insn_addi_valid ? spec_insn_addi_pc_wdata :
		spec_insn_and_valid ? spec_insn_and_pc_wdata :
		spec_insn_andi_valid ? spec_insn_andi_pc_wdata :
		spec_insn_auipc_valid ? spec_insn_auipc_pc_wdata :
		spec_insn_beq_valid ? spec_insn_beq_pc_wdata :
		spec_insn_bge_valid ? spec_insn_bge_pc_wdata :
		spec_insn_bgeu_valid ? spec_insn_bgeu_pc_wdata :
		spec_insn_blt_valid ? spec_insn_blt_pc_wdata :
		spec_insn_bltu_valid ? spec_insn_bltu_pc_wdata :
		spec_insn_bne_valid ? spec_insn_bne_pc_wdata :
		spec_insn_c_add_valid ? spec_insn_c_add_pc_wdata :
		spec_insn_c_addi_valid ? spec_insn_c_addi_pc_wdata :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_pc_wdata :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_pc_wdata :
		spec_insn_c_and_valid ? spec_insn_c_and_pc_wdata :
		spec_insn_c_andi_valid ? spec_insn_c_andi_pc_wdata :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_pc_wdata :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_pc_wdata :
		spec_insn_c_j_valid ? spec_insn_c_j_pc_wdata :
		spec_insn_c_jal_valid ? spec_insn_c_jal_pc_wdata :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_pc_wdata :
		spec_insn_c_jr_valid ? spec_insn_c_jr_pc_wdata :
		spec_insn_c_li_valid ? spec_insn_c_li_pc_wdata :
		spec_insn_c_lui_valid ? spec_insn_c_lui_pc_wdata :
		spec_insn_c_lw_valid ? spec_insn_c_lw_pc_wdata :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_pc_wdata :
		spec_insn_c_mv_valid ? spec_insn_c_mv_pc_wdata :
		spec_insn_c_or_valid ? spec_insn_c_or_pc_wdata :
		spec_insn_c_slli_valid ? spec_insn_c_slli_pc_wdata :
		spec_insn_c_srai_valid ? spec_insn_c_srai_pc_wdata :
		spec_insn_c_srli_valid ? spec_insn_c_srli_pc_wdata :
		spec_insn_c_sub_valid ? spec_insn_c_sub_pc_wdata :
		spec_insn_c_sw_valid ? spec_insn_c_sw_pc_wdata :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_pc_wdata :
		spec_insn_c_xor_valid ? spec_insn_c_xor_pc_wdata :
		spec_insn_div_valid ? spec_insn_div_pc_wdata :
		spec_insn_divu_valid ? spec_insn_divu_pc_wdata :
		spec_insn_jal_valid ? spec_insn_jal_pc_wdata :
		spec_insn_jalr_valid ? spec_insn_jalr_pc_wdata :
		spec_insn_lb_valid ? spec_insn_lb_pc_wdata :
		spec_insn_lbu_valid ? spec_insn_lbu_pc_wdata :
		spec_insn_lh_valid ? spec_insn_lh_pc_wdata :
		spec_insn_lhu_valid ? spec_insn_lhu_pc_wdata :
		spec_insn_lui_valid ? spec_insn_lui_pc_wdata :
		spec_insn_lw_valid ? spec_insn_lw_pc_wdata :
		spec_insn_mul_valid ? spec_insn_mul_pc_wdata :
		spec_insn_mulh_valid ? spec_insn_mulh_pc_wdata :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_pc_wdata :
		spec_insn_mulhu_valid ? spec_insn_mulhu_pc_wdata :
		spec_insn_or_valid ? spec_insn_or_pc_wdata :
		spec_insn_ori_valid ? spec_insn_ori_pc_wdata :
		spec_insn_rem_valid ? spec_insn_rem_pc_wdata :
		spec_insn_remu_valid ? spec_insn_remu_pc_wdata :
		spec_insn_sb_valid ? spec_insn_sb_pc_wdata :
		spec_insn_sh_valid ? spec_insn_sh_pc_wdata :
		spec_insn_sll_valid ? spec_insn_sll_pc_wdata :
		spec_insn_slli_valid ? spec_insn_slli_pc_wdata :
		spec_insn_slt_valid ? spec_insn_slt_pc_wdata :
		spec_insn_slti_valid ? spec_insn_slti_pc_wdata :
		spec_insn_sltiu_valid ? spec_insn_sltiu_pc_wdata :
		spec_insn_sltu_valid ? spec_insn_sltu_pc_wdata :
		spec_insn_sra_valid ? spec_insn_sra_pc_wdata :
		spec_insn_srai_valid ? spec_insn_srai_pc_wdata :
		spec_insn_srl_valid ? spec_insn_srl_pc_wdata :
		spec_insn_srli_valid ? spec_insn_srli_pc_wdata :
		spec_insn_sub_valid ? spec_insn_sub_pc_wdata :
		spec_insn_sw_valid ? spec_insn_sw_pc_wdata :
		spec_insn_xor_valid ? spec_insn_xor_pc_wdata :
		spec_insn_xori_valid ? spec_insn_xori_pc_wdata : 0;
  assign spec_mem_addr =
		spec_insn_add_valid ? spec_insn_add_mem_addr :
		spec_insn_addi_valid ? spec_insn_addi_mem_addr :
		spec_insn_and_valid ? spec_insn_and_mem_addr :
		spec_insn_andi_valid ? spec_insn_andi_mem_addr :
		spec_insn_auipc_valid ? spec_insn_auipc_mem_addr :
		spec_insn_beq_valid ? spec_insn_beq_mem_addr :
		spec_insn_bge_valid ? spec_insn_bge_mem_addr :
		spec_insn_bgeu_valid ? spec_insn_bgeu_mem_addr :
		spec_insn_blt_valid ? spec_insn_blt_mem_addr :
		spec_insn_bltu_valid ? spec_insn_bltu_mem_addr :
		spec_insn_bne_valid ? spec_insn_bne_mem_addr :
		spec_insn_c_add_valid ? spec_insn_c_add_mem_addr :
		spec_insn_c_addi_valid ? spec_insn_c_addi_mem_addr :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_mem_addr :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_mem_addr :
		spec_insn_c_and_valid ? spec_insn_c_and_mem_addr :
		spec_insn_c_andi_valid ? spec_insn_c_andi_mem_addr :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_mem_addr :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_mem_addr :
		spec_insn_c_j_valid ? spec_insn_c_j_mem_addr :
		spec_insn_c_jal_valid ? spec_insn_c_jal_mem_addr :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_mem_addr :
		spec_insn_c_jr_valid ? spec_insn_c_jr_mem_addr :
		spec_insn_c_li_valid ? spec_insn_c_li_mem_addr :
		spec_insn_c_lui_valid ? spec_insn_c_lui_mem_addr :
		spec_insn_c_lw_valid ? spec_insn_c_lw_mem_addr :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_mem_addr :
		spec_insn_c_mv_valid ? spec_insn_c_mv_mem_addr :
		spec_insn_c_or_valid ? spec_insn_c_or_mem_addr :
		spec_insn_c_slli_valid ? spec_insn_c_slli_mem_addr :
		spec_insn_c_srai_valid ? spec_insn_c_srai_mem_addr :
		spec_insn_c_srli_valid ? spec_insn_c_srli_mem_addr :
		spec_insn_c_sub_valid ? spec_insn_c_sub_mem_addr :
		spec_insn_c_sw_valid ? spec_insn_c_sw_mem_addr :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_mem_addr :
		spec_insn_c_xor_valid ? spec_insn_c_xor_mem_addr :
		spec_insn_div_valid ? spec_insn_div_mem_addr :
		spec_insn_divu_valid ? spec_insn_divu_mem_addr :
		spec_insn_jal_valid ? spec_insn_jal_mem_addr :
		spec_insn_jalr_valid ? spec_insn_jalr_mem_addr :
		spec_insn_lb_valid ? spec_insn_lb_mem_addr :
		spec_insn_lbu_valid ? spec_insn_lbu_mem_addr :
		spec_insn_lh_valid ? spec_insn_lh_mem_addr :
		spec_insn_lhu_valid ? spec_insn_lhu_mem_addr :
		spec_insn_lui_valid ? spec_insn_lui_mem_addr :
		spec_insn_lw_valid ? spec_insn_lw_mem_addr :
		spec_insn_mul_valid ? spec_insn_mul_mem_addr :
		spec_insn_mulh_valid ? spec_insn_mulh_mem_addr :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_mem_addr :
		spec_insn_mulhu_valid ? spec_insn_mulhu_mem_addr :
		spec_insn_or_valid ? spec_insn_or_mem_addr :
		spec_insn_ori_valid ? spec_insn_ori_mem_addr :
		spec_insn_rem_valid ? spec_insn_rem_mem_addr :
		spec_insn_remu_valid ? spec_insn_remu_mem_addr :
		spec_insn_sb_valid ? spec_insn_sb_mem_addr :
		spec_insn_sh_valid ? spec_insn_sh_mem_addr :
		spec_insn_sll_valid ? spec_insn_sll_mem_addr :
		spec_insn_slli_valid ? spec_insn_slli_mem_addr :
		spec_insn_slt_valid ? spec_insn_slt_mem_addr :
		spec_insn_slti_valid ? spec_insn_slti_mem_addr :
		spec_insn_sltiu_valid ? spec_insn_sltiu_mem_addr :
		spec_insn_sltu_valid ? spec_insn_sltu_mem_addr :
		spec_insn_sra_valid ? spec_insn_sra_mem_addr :
		spec_insn_srai_valid ? spec_insn_srai_mem_addr :
		spec_insn_srl_valid ? spec_insn_srl_mem_addr :
		spec_insn_srli_valid ? spec_insn_srli_mem_addr :
		spec_insn_sub_valid ? spec_insn_sub_mem_addr :
		spec_insn_sw_valid ? spec_insn_sw_mem_addr :
		spec_insn_xor_valid ? spec_insn_xor_mem_addr :
		spec_insn_xori_valid ? spec_insn_xori_mem_addr : 0;
  assign spec_mem_rmask =
		spec_insn_add_valid ? spec_insn_add_mem_rmask :
		spec_insn_addi_valid ? spec_insn_addi_mem_rmask :
		spec_insn_and_valid ? spec_insn_and_mem_rmask :
		spec_insn_andi_valid ? spec_insn_andi_mem_rmask :
		spec_insn_auipc_valid ? spec_insn_auipc_mem_rmask :
		spec_insn_beq_valid ? spec_insn_beq_mem_rmask :
		spec_insn_bge_valid ? spec_insn_bge_mem_rmask :
		spec_insn_bgeu_valid ? spec_insn_bgeu_mem_rmask :
		spec_insn_blt_valid ? spec_insn_blt_mem_rmask :
		spec_insn_bltu_valid ? spec_insn_bltu_mem_rmask :
		spec_insn_bne_valid ? spec_insn_bne_mem_rmask :
		spec_insn_c_add_valid ? spec_insn_c_add_mem_rmask :
		spec_insn_c_addi_valid ? spec_insn_c_addi_mem_rmask :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_mem_rmask :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_mem_rmask :
		spec_insn_c_and_valid ? spec_insn_c_and_mem_rmask :
		spec_insn_c_andi_valid ? spec_insn_c_andi_mem_rmask :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_mem_rmask :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_mem_rmask :
		spec_insn_c_j_valid ? spec_insn_c_j_mem_rmask :
		spec_insn_c_jal_valid ? spec_insn_c_jal_mem_rmask :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_mem_rmask :
		spec_insn_c_jr_valid ? spec_insn_c_jr_mem_rmask :
		spec_insn_c_li_valid ? spec_insn_c_li_mem_rmask :
		spec_insn_c_lui_valid ? spec_insn_c_lui_mem_rmask :
		spec_insn_c_lw_valid ? spec_insn_c_lw_mem_rmask :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_mem_rmask :
		spec_insn_c_mv_valid ? spec_insn_c_mv_mem_rmask :
		spec_insn_c_or_valid ? spec_insn_c_or_mem_rmask :
		spec_insn_c_slli_valid ? spec_insn_c_slli_mem_rmask :
		spec_insn_c_srai_valid ? spec_insn_c_srai_mem_rmask :
		spec_insn_c_srli_valid ? spec_insn_c_srli_mem_rmask :
		spec_insn_c_sub_valid ? spec_insn_c_sub_mem_rmask :
		spec_insn_c_sw_valid ? spec_insn_c_sw_mem_rmask :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_mem_rmask :
		spec_insn_c_xor_valid ? spec_insn_c_xor_mem_rmask :
		spec_insn_div_valid ? spec_insn_div_mem_rmask :
		spec_insn_divu_valid ? spec_insn_divu_mem_rmask :
		spec_insn_jal_valid ? spec_insn_jal_mem_rmask :
		spec_insn_jalr_valid ? spec_insn_jalr_mem_rmask :
		spec_insn_lb_valid ? spec_insn_lb_mem_rmask :
		spec_insn_lbu_valid ? spec_insn_lbu_mem_rmask :
		spec_insn_lh_valid ? spec_insn_lh_mem_rmask :
		spec_insn_lhu_valid ? spec_insn_lhu_mem_rmask :
		spec_insn_lui_valid ? spec_insn_lui_mem_rmask :
		spec_insn_lw_valid ? spec_insn_lw_mem_rmask :
		spec_insn_mul_valid ? spec_insn_mul_mem_rmask :
		spec_insn_mulh_valid ? spec_insn_mulh_mem_rmask :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_mem_rmask :
		spec_insn_mulhu_valid ? spec_insn_mulhu_mem_rmask :
		spec_insn_or_valid ? spec_insn_or_mem_rmask :
		spec_insn_ori_valid ? spec_insn_ori_mem_rmask :
		spec_insn_rem_valid ? spec_insn_rem_mem_rmask :
		spec_insn_remu_valid ? spec_insn_remu_mem_rmask :
		spec_insn_sb_valid ? spec_insn_sb_mem_rmask :
		spec_insn_sh_valid ? spec_insn_sh_mem_rmask :
		spec_insn_sll_valid ? spec_insn_sll_mem_rmask :
		spec_insn_slli_valid ? spec_insn_slli_mem_rmask :
		spec_insn_slt_valid ? spec_insn_slt_mem_rmask :
		spec_insn_slti_valid ? spec_insn_slti_mem_rmask :
		spec_insn_sltiu_valid ? spec_insn_sltiu_mem_rmask :
		spec_insn_sltu_valid ? spec_insn_sltu_mem_rmask :
		spec_insn_sra_valid ? spec_insn_sra_mem_rmask :
		spec_insn_srai_valid ? spec_insn_srai_mem_rmask :
		spec_insn_srl_valid ? spec_insn_srl_mem_rmask :
		spec_insn_srli_valid ? spec_insn_srli_mem_rmask :
		spec_insn_sub_valid ? spec_insn_sub_mem_rmask :
		spec_insn_sw_valid ? spec_insn_sw_mem_rmask :
		spec_insn_xor_valid ? spec_insn_xor_mem_rmask :
		spec_insn_xori_valid ? spec_insn_xori_mem_rmask : 0;
  assign spec_mem_wmask =
		spec_insn_add_valid ? spec_insn_add_mem_wmask :
		spec_insn_addi_valid ? spec_insn_addi_mem_wmask :
		spec_insn_and_valid ? spec_insn_and_mem_wmask :
		spec_insn_andi_valid ? spec_insn_andi_mem_wmask :
		spec_insn_auipc_valid ? spec_insn_auipc_mem_wmask :
		spec_insn_beq_valid ? spec_insn_beq_mem_wmask :
		spec_insn_bge_valid ? spec_insn_bge_mem_wmask :
		spec_insn_bgeu_valid ? spec_insn_bgeu_mem_wmask :
		spec_insn_blt_valid ? spec_insn_blt_mem_wmask :
		spec_insn_bltu_valid ? spec_insn_bltu_mem_wmask :
		spec_insn_bne_valid ? spec_insn_bne_mem_wmask :
		spec_insn_c_add_valid ? spec_insn_c_add_mem_wmask :
		spec_insn_c_addi_valid ? spec_insn_c_addi_mem_wmask :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_mem_wmask :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_mem_wmask :
		spec_insn_c_and_valid ? spec_insn_c_and_mem_wmask :
		spec_insn_c_andi_valid ? spec_insn_c_andi_mem_wmask :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_mem_wmask :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_mem_wmask :
		spec_insn_c_j_valid ? spec_insn_c_j_mem_wmask :
		spec_insn_c_jal_valid ? spec_insn_c_jal_mem_wmask :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_mem_wmask :
		spec_insn_c_jr_valid ? spec_insn_c_jr_mem_wmask :
		spec_insn_c_li_valid ? spec_insn_c_li_mem_wmask :
		spec_insn_c_lui_valid ? spec_insn_c_lui_mem_wmask :
		spec_insn_c_lw_valid ? spec_insn_c_lw_mem_wmask :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_mem_wmask :
		spec_insn_c_mv_valid ? spec_insn_c_mv_mem_wmask :
		spec_insn_c_or_valid ? spec_insn_c_or_mem_wmask :
		spec_insn_c_slli_valid ? spec_insn_c_slli_mem_wmask :
		spec_insn_c_srai_valid ? spec_insn_c_srai_mem_wmask :
		spec_insn_c_srli_valid ? spec_insn_c_srli_mem_wmask :
		spec_insn_c_sub_valid ? spec_insn_c_sub_mem_wmask :
		spec_insn_c_sw_valid ? spec_insn_c_sw_mem_wmask :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_mem_wmask :
		spec_insn_c_xor_valid ? spec_insn_c_xor_mem_wmask :
		spec_insn_div_valid ? spec_insn_div_mem_wmask :
		spec_insn_divu_valid ? spec_insn_divu_mem_wmask :
		spec_insn_jal_valid ? spec_insn_jal_mem_wmask :
		spec_insn_jalr_valid ? spec_insn_jalr_mem_wmask :
		spec_insn_lb_valid ? spec_insn_lb_mem_wmask :
		spec_insn_lbu_valid ? spec_insn_lbu_mem_wmask :
		spec_insn_lh_valid ? spec_insn_lh_mem_wmask :
		spec_insn_lhu_valid ? spec_insn_lhu_mem_wmask :
		spec_insn_lui_valid ? spec_insn_lui_mem_wmask :
		spec_insn_lw_valid ? spec_insn_lw_mem_wmask :
		spec_insn_mul_valid ? spec_insn_mul_mem_wmask :
		spec_insn_mulh_valid ? spec_insn_mulh_mem_wmask :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_mem_wmask :
		spec_insn_mulhu_valid ? spec_insn_mulhu_mem_wmask :
		spec_insn_or_valid ? spec_insn_or_mem_wmask :
		spec_insn_ori_valid ? spec_insn_ori_mem_wmask :
		spec_insn_rem_valid ? spec_insn_rem_mem_wmask :
		spec_insn_remu_valid ? spec_insn_remu_mem_wmask :
		spec_insn_sb_valid ? spec_insn_sb_mem_wmask :
		spec_insn_sh_valid ? spec_insn_sh_mem_wmask :
		spec_insn_sll_valid ? spec_insn_sll_mem_wmask :
		spec_insn_slli_valid ? spec_insn_slli_mem_wmask :
		spec_insn_slt_valid ? spec_insn_slt_mem_wmask :
		spec_insn_slti_valid ? spec_insn_slti_mem_wmask :
		spec_insn_sltiu_valid ? spec_insn_sltiu_mem_wmask :
		spec_insn_sltu_valid ? spec_insn_sltu_mem_wmask :
		spec_insn_sra_valid ? spec_insn_sra_mem_wmask :
		spec_insn_srai_valid ? spec_insn_srai_mem_wmask :
		spec_insn_srl_valid ? spec_insn_srl_mem_wmask :
		spec_insn_srli_valid ? spec_insn_srli_mem_wmask :
		spec_insn_sub_valid ? spec_insn_sub_mem_wmask :
		spec_insn_sw_valid ? spec_insn_sw_mem_wmask :
		spec_insn_xor_valid ? spec_insn_xor_mem_wmask :
		spec_insn_xori_valid ? spec_insn_xori_mem_wmask : 0;
  assign spec_mem_wdata =
		spec_insn_add_valid ? spec_insn_add_mem_wdata :
		spec_insn_addi_valid ? spec_insn_addi_mem_wdata :
		spec_insn_and_valid ? spec_insn_and_mem_wdata :
		spec_insn_andi_valid ? spec_insn_andi_mem_wdata :
		spec_insn_auipc_valid ? spec_insn_auipc_mem_wdata :
		spec_insn_beq_valid ? spec_insn_beq_mem_wdata :
		spec_insn_bge_valid ? spec_insn_bge_mem_wdata :
		spec_insn_bgeu_valid ? spec_insn_bgeu_mem_wdata :
		spec_insn_blt_valid ? spec_insn_blt_mem_wdata :
		spec_insn_bltu_valid ? spec_insn_bltu_mem_wdata :
		spec_insn_bne_valid ? spec_insn_bne_mem_wdata :
		spec_insn_c_add_valid ? spec_insn_c_add_mem_wdata :
		spec_insn_c_addi_valid ? spec_insn_c_addi_mem_wdata :
		spec_insn_c_addi16sp_valid ? spec_insn_c_addi16sp_mem_wdata :
		spec_insn_c_addi4spn_valid ? spec_insn_c_addi4spn_mem_wdata :
		spec_insn_c_and_valid ? spec_insn_c_and_mem_wdata :
		spec_insn_c_andi_valid ? spec_insn_c_andi_mem_wdata :
		spec_insn_c_beqz_valid ? spec_insn_c_beqz_mem_wdata :
		spec_insn_c_bnez_valid ? spec_insn_c_bnez_mem_wdata :
		spec_insn_c_j_valid ? spec_insn_c_j_mem_wdata :
		spec_insn_c_jal_valid ? spec_insn_c_jal_mem_wdata :
		spec_insn_c_jalr_valid ? spec_insn_c_jalr_mem_wdata :
		spec_insn_c_jr_valid ? spec_insn_c_jr_mem_wdata :
		spec_insn_c_li_valid ? spec_insn_c_li_mem_wdata :
		spec_insn_c_lui_valid ? spec_insn_c_lui_mem_wdata :
		spec_insn_c_lw_valid ? spec_insn_c_lw_mem_wdata :
		spec_insn_c_lwsp_valid ? spec_insn_c_lwsp_mem_wdata :
		spec_insn_c_mv_valid ? spec_insn_c_mv_mem_wdata :
		spec_insn_c_or_valid ? spec_insn_c_or_mem_wdata :
		spec_insn_c_slli_valid ? spec_insn_c_slli_mem_wdata :
		spec_insn_c_srai_valid ? spec_insn_c_srai_mem_wdata :
		spec_insn_c_srli_valid ? spec_insn_c_srli_mem_wdata :
		spec_insn_c_sub_valid ? spec_insn_c_sub_mem_wdata :
		spec_insn_c_sw_valid ? spec_insn_c_sw_mem_wdata :
		spec_insn_c_swsp_valid ? spec_insn_c_swsp_mem_wdata :
		spec_insn_c_xor_valid ? spec_insn_c_xor_mem_wdata :
		spec_insn_div_valid ? spec_insn_div_mem_wdata :
		spec_insn_divu_valid ? spec_insn_divu_mem_wdata :
		spec_insn_jal_valid ? spec_insn_jal_mem_wdata :
		spec_insn_jalr_valid ? spec_insn_jalr_mem_wdata :
		spec_insn_lb_valid ? spec_insn_lb_mem_wdata :
		spec_insn_lbu_valid ? spec_insn_lbu_mem_wdata :
		spec_insn_lh_valid ? spec_insn_lh_mem_wdata :
		spec_insn_lhu_valid ? spec_insn_lhu_mem_wdata :
		spec_insn_lui_valid ? spec_insn_lui_mem_wdata :
		spec_insn_lw_valid ? spec_insn_lw_mem_wdata :
		spec_insn_mul_valid ? spec_insn_mul_mem_wdata :
		spec_insn_mulh_valid ? spec_insn_mulh_mem_wdata :
		spec_insn_mulhsu_valid ? spec_insn_mulhsu_mem_wdata :
		spec_insn_mulhu_valid ? spec_insn_mulhu_mem_wdata :
		spec_insn_or_valid ? spec_insn_or_mem_wdata :
		spec_insn_ori_valid ? spec_insn_ori_mem_wdata :
		spec_insn_rem_valid ? spec_insn_rem_mem_wdata :
		spec_insn_remu_valid ? spec_insn_remu_mem_wdata :
		spec_insn_sb_valid ? spec_insn_sb_mem_wdata :
		spec_insn_sh_valid ? spec_insn_sh_mem_wdata :
		spec_insn_sll_valid ? spec_insn_sll_mem_wdata :
		spec_insn_slli_valid ? spec_insn_slli_mem_wdata :
		spec_insn_slt_valid ? spec_insn_slt_mem_wdata :
		spec_insn_slti_valid ? spec_insn_slti_mem_wdata :
		spec_insn_sltiu_valid ? spec_insn_sltiu_mem_wdata :
		spec_insn_sltu_valid ? spec_insn_sltu_mem_wdata :
		spec_insn_sra_valid ? spec_insn_sra_mem_wdata :
		spec_insn_srai_valid ? spec_insn_srai_mem_wdata :
		spec_insn_srl_valid ? spec_insn_srl_mem_wdata :
		spec_insn_srli_valid ? spec_insn_srli_mem_wdata :
		spec_insn_sub_valid ? spec_insn_sub_mem_wdata :
		spec_insn_sw_valid ? spec_insn_sw_mem_wdata :
		spec_insn_xor_valid ? spec_insn_xor_mem_wdata :
		spec_insn_xori_valid ? spec_insn_xori_mem_wdata : 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_add (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // ADD instruction
  wire [32-1:0] result = rvfi_rs1_rdata + rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000000 && insn_funct3 == 3'b 000 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_addi (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // ADDI instruction
  wire [32-1:0] result = rvfi_rs1_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 000 && insn_opcode == 7'b 0010011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_and (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // AND instruction
  wire [32-1:0] result = rvfi_rs1_rdata & rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000000 && insn_funct3 == 3'b 111 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_andi (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // ANDI instruction
  wire [32-1:0] result = rvfi_rs1_rdata & insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 111 && insn_opcode == 7'b 0010011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_auipc (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // U-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31:12], 12'b0});
  wire [4:0] insn_rd     = rvfi_insn[11:7];
  wire [6:0] insn_opcode = rvfi_insn[ 6:0];

  wire misa_ok = 1;

  // AUIPC instruction
  assign spec_valid = rvfi_valid && !insn_padding && insn_opcode == 7'b 0010111;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? rvfi_pc_rdata + insn_imm : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs1_addr = 0;
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_beq (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // SB-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31], rvfi_insn[7], rvfi_insn[30:25], rvfi_insn[11:8], 1'b0});
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;
  wire ialign16 = 1;

  // BEQ instruction
  wire cond = rvfi_rs1_rdata == rvfi_rs2_rdata;
  wire [32-1:0] next_pc = cond ? rvfi_pc_rdata + insn_imm : rvfi_pc_rdata + 4;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 000 && insn_opcode == 7'b 1100011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (ialign16 ? (next_pc[0] != 0) : (next_pc[1:0] != 0)) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_bge (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // SB-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31], rvfi_insn[7], rvfi_insn[30:25], rvfi_insn[11:8], 1'b0});
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;
  wire ialign16 = 1;

  // BGE instruction
  wire cond = $signed(rvfi_rs1_rdata) >= $signed(rvfi_rs2_rdata);
  wire [32-1:0] next_pc = cond ? rvfi_pc_rdata + insn_imm : rvfi_pc_rdata + 4;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 101 && insn_opcode == 7'b 1100011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (ialign16 ? (next_pc[0] != 0) : (next_pc[1:0] != 0)) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_bgeu (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // SB-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31], rvfi_insn[7], rvfi_insn[30:25], rvfi_insn[11:8], 1'b0});
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;
  wire ialign16 = 1;

  // BGEU instruction
  wire cond = rvfi_rs1_rdata >= rvfi_rs2_rdata;
  wire [32-1:0] next_pc = cond ? rvfi_pc_rdata + insn_imm : rvfi_pc_rdata + 4;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 111 && insn_opcode == 7'b 1100011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (ialign16 ? (next_pc[0] != 0) : (next_pc[1:0] != 0)) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_blt (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // SB-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31], rvfi_insn[7], rvfi_insn[30:25], rvfi_insn[11:8], 1'b0});
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;
  wire ialign16 = 1;

  // BLT instruction
  wire cond = $signed(rvfi_rs1_rdata) < $signed(rvfi_rs2_rdata);
  wire [32-1:0] next_pc = cond ? rvfi_pc_rdata + insn_imm : rvfi_pc_rdata + 4;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 100 && insn_opcode == 7'b 1100011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (ialign16 ? (next_pc[0] != 0) : (next_pc[1:0] != 0)) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_bltu (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // SB-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31], rvfi_insn[7], rvfi_insn[30:25], rvfi_insn[11:8], 1'b0});
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;
  wire ialign16 = 1;

  // BLTU instruction
  wire cond = rvfi_rs1_rdata < rvfi_rs2_rdata;
  wire [32-1:0] next_pc = cond ? rvfi_pc_rdata + insn_imm : rvfi_pc_rdata + 4;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 110 && insn_opcode == 7'b 1100011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (ialign16 ? (next_pc[0] != 0) : (next_pc[1:0] != 0)) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_bne (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // SB-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31], rvfi_insn[7], rvfi_insn[30:25], rvfi_insn[11:8], 1'b0});
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;
  wire ialign16 = 1;

  // BNE instruction
  wire cond = rvfi_rs1_rdata != rvfi_rs2_rdata;
  wire [32-1:0] next_pc = cond ? rvfi_pc_rdata + insn_imm : rvfi_pc_rdata + 4;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 001 && insn_opcode == 7'b 1100011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (ialign16 ? (next_pc[0] != 0) : (next_pc[1:0] != 0)) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_add (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [3:0] insn_funct4 = rvfi_insn[15:12];
  wire [4:0] insn_rs1_rd = rvfi_insn[11:7];
  wire [4:0] insn_rs2 = rvfi_insn[6:2];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_ADD instruction
  wire [32-1:0] result = rvfi_rs1_rdata + rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct4 == 4'b 1001 && insn_rs2 && insn_opcode == 2'b 10;
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_addi (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[12], rvfi_insn[6:2]});
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rs1_rd = rvfi_insn[11:7];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_ADDI instruction
  wire [32-1:0] result = rvfi_rs1_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 000 && insn_opcode == 2'b 01;
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_addi16sp (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format (SP variation)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[12], rvfi_insn[4:3], rvfi_insn[5], rvfi_insn[2], rvfi_insn[6], 4'b0});
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rs1_rd = rvfi_insn[11:7];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_ADDI16SP instruction
  wire [32-1:0] result = rvfi_rs1_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 011 && insn_opcode == 2'b 01 && insn_rs1_rd == 5'd 2 && insn_imm;
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_addi4spn (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CIW-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = {rvfi_insn[10:7], rvfi_insn[12:11], rvfi_insn[5], rvfi_insn[6], 2'b00};
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rd = {1'b1, rvfi_insn[4:2]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_ADDI4SPN instruction
  wire [32-1:0] result = rvfi_rs1_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 000 && insn_opcode == 2'b 00 && insn_imm;
  assign spec_rs1_addr = 2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_and (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CS-type instruction format (ALU version)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [5:0] insn_funct6 = rvfi_insn[15:10];
  wire [1:0] insn_funct2 = rvfi_insn[6:5];
  wire [4:0] insn_rs1_rd = {1'b1, rvfi_insn[9:7]};
  wire [4:0] insn_rs2 = {1'b1, rvfi_insn[4:2]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_AND instruction
  wire [32-1:0] result = rvfi_rs1_rdata & rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct6 == 6'b 100011 && insn_funct2 == 2'b 11 && insn_opcode == 2'b 01;
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_andi (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format (ANDI variation)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[12], rvfi_insn[6:2]});
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [1:0] insn_funct2 = rvfi_insn[11:10];
  wire [4:0] insn_rs1_rd = {1'b1, rvfi_insn[9:7]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_ANDI instruction
  wire [32-1:0] result = rvfi_rs1_rdata & insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 100 && insn_funct2 == 2'b 10 && insn_opcode == 2'b 01;
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_beqz (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CB-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[12], rvfi_insn[6:5], rvfi_insn[2], rvfi_insn[11:10], rvfi_insn[4:3], 1'b0});
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rs1 = {1'b1, rvfi_insn[9:7]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_BEQZ instruction
  wire cond = rvfi_rs1_rdata == 0;
  wire [32-1:0] next_pc = cond ? rvfi_pc_rdata + insn_imm : rvfi_pc_rdata + 2;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 110 && insn_opcode == 2'b 01;
  assign spec_rs1_addr = insn_rs1;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (next_pc[0] != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_bnez (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CB-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[12], rvfi_insn[6:5], rvfi_insn[2], rvfi_insn[11:10], rvfi_insn[4:3], 1'b0});
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rs1 = {1'b1, rvfi_insn[9:7]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_BNEZ instruction
  wire cond = rvfi_rs1_rdata != 0;
  wire [32-1:0] next_pc = cond ? rvfi_pc_rdata + insn_imm : rvfi_pc_rdata + 2;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 111 && insn_opcode == 2'b 01;
  assign spec_rs1_addr = insn_rs1;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (next_pc[0] != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_j (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CJ-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[12], rvfi_insn[8], rvfi_insn[10], rvfi_insn[9],
      rvfi_insn[6], rvfi_insn[7], rvfi_insn[2], rvfi_insn[11], rvfi_insn[5], rvfi_insn[4], rvfi_insn[3], 1'b0});
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_J instruction
  wire [32-1:0] next_pc = rvfi_pc_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 101 && insn_opcode == 2'b 01;
  assign spec_pc_wdata = next_pc;

  // default assignments
  assign spec_rs1_addr = 0;
  assign spec_rs2_addr = 0;
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_jal (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CJ-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[12], rvfi_insn[8], rvfi_insn[10], rvfi_insn[9],
      rvfi_insn[6], rvfi_insn[7], rvfi_insn[2], rvfi_insn[11], rvfi_insn[5], rvfi_insn[4], rvfi_insn[3], 1'b0});
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_JAL instruction
  wire [32-1:0] next_pc = rvfi_pc_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 001 && insn_opcode == 2'b 01;
  assign spec_rd_addr = 5'd 1;
  assign spec_rd_wdata = rvfi_pc_rdata + 2;
  assign spec_pc_wdata = next_pc;

  // default assignments
  assign spec_rs1_addr = 0;
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_jalr (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [3:0] insn_funct4 = rvfi_insn[15:12];
  wire [4:0] insn_rs1_rd = rvfi_insn[11:7];
  wire [4:0] insn_rs2 = rvfi_insn[6:2];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_JALR instruction
  wire [32-1:0] next_pc = rvfi_rs1_rdata & ~1;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct4 == 4'b 1001 && insn_rs1_rd && !insn_rs2 && insn_opcode == 2'b 10;
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rd_addr = 5'd 1;
  assign spec_rd_wdata = rvfi_pc_rdata + 2;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (next_pc[0] != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_jr (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [3:0] insn_funct4 = rvfi_insn[15:12];
  wire [4:0] insn_rs1_rd = rvfi_insn[11:7];
  wire [4:0] insn_rs2 = rvfi_insn[6:2];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_JR instruction
  wire [32-1:0] next_pc = rvfi_rs1_rdata & ~1;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct4 == 4'b 1000 && insn_rs1_rd && !insn_rs2 && insn_opcode == 2'b 10;
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (next_pc[0] != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_li (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[12], rvfi_insn[6:2]});
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rs1_rd = rvfi_insn[11:7];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_LI instruction
  wire [32-1:0] result = insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 010 && insn_opcode == 2'b 01;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_rs1_addr = 0;
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_lui (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format (LUI variation)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[12], rvfi_insn[6:2], 12'b0});
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rs1_rd = rvfi_insn[11:7];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_LUI instruction
  wire [32-1:0] result = insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 011 && insn_opcode == 2'b 01 && insn_rs1_rd != 5'd 2 && insn_imm;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_rs1_addr = 0;
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_lw (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CL-type instruction format (32 bit version)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = {rvfi_insn[5], rvfi_insn[12:10], rvfi_insn[6], 2'b00};
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rs1 = {1'b1, rvfi_insn[9:7]};
  wire [4:0] insn_rd = {1'b1, rvfi_insn[4:2]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_LW instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  wire [31:0] result = rvfi_mem_rdata >> (8*(addr-spec_mem_addr));
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 010 && insn_opcode == 2'b 00;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_rmask = ((1 << 4)-1) << (addr-spec_mem_addr);
  assign spec_rd_wdata = spec_rd_addr ? $signed(result) : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;
  assign spec_trap = ((addr & (4-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_lwsp (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format (LSP variation, 32 bit version)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = {rvfi_insn[3:2], rvfi_insn[12], rvfi_insn[6:4], 2'b00};
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rd = rvfi_insn[11:7];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_LWSP instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  wire [31:0] result = rvfi_mem_rdata >> (8*(addr-spec_mem_addr));
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 010 && insn_opcode == 2'b 10 && insn_rd;
  assign spec_rs1_addr = 2;
  assign spec_rd_addr = insn_rd;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_rmask = ((1 << 4)-1) << (addr-spec_mem_addr);
  assign spec_rd_wdata = spec_rd_addr ? $signed(result) : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;
  assign spec_trap = ((addr & (4-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_mv (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [3:0] insn_funct4 = rvfi_insn[15:12];
  wire [4:0] insn_rs1_rd = rvfi_insn[11:7];
  wire [4:0] insn_rs2 = rvfi_insn[6:2];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_MV instruction
  wire [32-1:0] result = rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct4 == 4'b 1000 && insn_rs2 && insn_opcode == 2'b 10;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_rs1_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_or (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CS-type instruction format (ALU version)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [5:0] insn_funct6 = rvfi_insn[15:10];
  wire [1:0] insn_funct2 = rvfi_insn[6:5];
  wire [4:0] insn_rs1_rd = {1'b1, rvfi_insn[9:7]};
  wire [4:0] insn_rs2 = {1'b1, rvfi_insn[4:2]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_OR instruction
  wire [32-1:0] result = rvfi_rs1_rdata | rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct6 == 6'b 100011 && insn_funct2 == 2'b 10 && insn_opcode == 2'b 01;
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_slli (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format (SLI variation)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [5:0] insn_shamt = {rvfi_insn[12], rvfi_insn[6:2]};
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rs1_rd = rvfi_insn[11:7];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_SLLI instruction
  wire [32-1:0] result = rvfi_rs1_rdata << insn_shamt;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 000 && insn_opcode == 2'b 10 && (!insn_shamt[5] || 32 == 64);
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_srai (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format (SRI variation)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [5:0] insn_shamt = {rvfi_insn[12], rvfi_insn[6:2]};
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [1:0] insn_funct2 = rvfi_insn[11:10];
  wire [4:0] insn_rs1_rd = {1'b1, rvfi_insn[9:7]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_SRAI instruction
  wire [32-1:0] result = $signed(rvfi_rs1_rdata) >>> insn_shamt;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 100 && insn_funct2 == 2'b 01 && insn_opcode == 2'b 01 && (!insn_shamt[5] || 32 == 64);
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_srli (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CI-type instruction format (SRI variation)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [5:0] insn_shamt = {rvfi_insn[12], rvfi_insn[6:2]};
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [1:0] insn_funct2 = rvfi_insn[11:10];
  wire [4:0] insn_rs1_rd = {1'b1, rvfi_insn[9:7]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_SRLI instruction
  wire [32-1:0] result = rvfi_rs1_rdata >> insn_shamt;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 100 && insn_funct2 == 2'b 00 && insn_opcode == 2'b 01 && (!insn_shamt[5] || 32 == 64);
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_sub (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CS-type instruction format (ALU version)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [5:0] insn_funct6 = rvfi_insn[15:10];
  wire [1:0] insn_funct2 = rvfi_insn[6:5];
  wire [4:0] insn_rs1_rd = {1'b1, rvfi_insn[9:7]};
  wire [4:0] insn_rs2 = {1'b1, rvfi_insn[4:2]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_SUB instruction
  wire [32-1:0] result = rvfi_rs1_rdata - rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct6 == 6'b 100011 && insn_funct2 == 2'b 00 && insn_opcode == 2'b 01;
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_sw (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CS-type instruction format (32 bit version)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = {rvfi_insn[5], rvfi_insn[12:10], rvfi_insn[6], 2'b00};
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rs1 = {1'b1, rvfi_insn[9:7]};
  wire [4:0] insn_rs2 = {1'b1, rvfi_insn[4:2]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_SW instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 110 && insn_opcode == 2'b 00;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_wmask = ((1 << 4)-1) << (addr-spec_mem_addr);
  assign spec_mem_wdata = rvfi_rs2_rdata << (8*(addr-spec_mem_addr));
  assign spec_pc_wdata = rvfi_pc_rdata + 2;
  assign spec_trap = ((addr & (4-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_rmask = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_swsp (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CSS-type instruction format (32 bit version)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [32-1:0] insn_imm = {rvfi_insn[8:7], rvfi_insn[12:9], 2'b00};
  wire [2:0] insn_funct3 = rvfi_insn[15:13];
  wire [4:0] insn_rs2 = rvfi_insn[6:2];
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_SWSP instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 110 && insn_opcode == 2'b 10;
  assign spec_rs1_addr = 2;
  assign spec_rs2_addr = insn_rs2;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_wmask = ((1 << 4)-1) << (addr-spec_mem_addr);
  assign spec_mem_wdata = rvfi_rs2_rdata << (8*(addr-spec_mem_addr));
  assign spec_pc_wdata = rvfi_pc_rdata + 2;
  assign spec_trap = ((addr & (4-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_rmask = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_c_xor (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // CS-type instruction format (ALU version)
  wire [32-1:0] insn_padding = rvfi_insn >> 16;
  wire [5:0] insn_funct6 = rvfi_insn[15:10];
  wire [1:0] insn_funct2 = rvfi_insn[6:5];
  wire [4:0] insn_rs1_rd = {1'b1, rvfi_insn[9:7]};
  wire [4:0] insn_rs2 = {1'b1, rvfi_insn[4:2]};
  wire [1:0] insn_opcode = rvfi_insn[1:0];

  wire misa_ok = 1;

  // C_XOR instruction
  wire [32-1:0] result = rvfi_rs1_rdata ^ rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct6 == 6'b 100011 && insn_funct2 == 2'b 01 && insn_opcode == 2'b 01;
  assign spec_rs1_addr = insn_rs1_rd;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rs1_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 2;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_div (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // DIV instruction
  reg [32-1:0] result;
  always @* begin
    if (rvfi_rs2_rdata == 32'b0) begin
      result = {32{1'b1}};
    end else begin
      if (rvfi_rs1_rdata == {1'b1, {32-1{1'b0}}} && rvfi_rs2_rdata == {32{1'b1}}) begin
        result = {1'b1, {32-1{1'b0}}};
      end else begin
        result = $signed(rvfi_rs1_rdata) / $signed(rvfi_rs2_rdata);
      end
    end
  end
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000001 && insn_funct3 == 3'b 100 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_divu (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // DIVU instruction
  wire [32-1:0] result = rvfi_rs2_rdata == 32'b0 ? {32{1'b1}} :
                                         rvfi_rs1_rdata / rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000001 && insn_funct3 == 3'b 101 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_jal (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // UJ-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31], rvfi_insn[19:12], rvfi_insn[20], rvfi_insn[30:21], 1'b0});
  wire [4:0] insn_rd     = rvfi_insn[11:7];
  wire [6:0] insn_opcode = rvfi_insn[6:0];

  wire misa_ok = 1;
  wire ialign16 = 1;

  // JAL instruction
  wire [32-1:0] next_pc = rvfi_pc_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_opcode == 7'b 1101111;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? rvfi_pc_rdata + 4 : 0;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (ialign16 ? (next_pc[0] != 0) : (next_pc[1:0] != 0)) || !misa_ok;

  // default assignments
  assign spec_rs1_addr = 0;
  assign spec_rs2_addr = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_jalr (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;
  wire ialign16 = 1;

  // JALR instruction
  wire [32-1:0] next_pc = (rvfi_rs1_rdata + insn_imm) & ~1;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 000 && insn_opcode == 7'b 1100111;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? rvfi_pc_rdata + 4 : 0;
  assign spec_pc_wdata = next_pc;
  assign spec_trap = (ialign16 ? (next_pc[0] != 0) : (next_pc[1:0] != 0)) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_lb (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // LB instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  wire [7:0] result = rvfi_mem_rdata >> (8*(addr-spec_mem_addr));
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 000 && insn_opcode == 7'b 0000011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_rmask = ((1 << 1)-1) << (addr-spec_mem_addr);
  assign spec_rd_wdata = spec_rd_addr ? $signed(result) : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;
  assign spec_trap = ((addr & (1-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_lbu (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // LBU instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  wire [7:0] result = rvfi_mem_rdata >> (8*(addr-spec_mem_addr));
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 100 && insn_opcode == 7'b 0000011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_rmask = ((1 << 1)-1) << (addr-spec_mem_addr);
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;
  assign spec_trap = ((addr & (1-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_lh (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // LH instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  wire [15:0] result = rvfi_mem_rdata >> (8*(addr-spec_mem_addr));
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 001 && insn_opcode == 7'b 0000011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_rmask = ((1 << 2)-1) << (addr-spec_mem_addr);
  assign spec_rd_wdata = spec_rd_addr ? $signed(result) : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;
  assign spec_trap = ((addr & (2-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_lhu (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // LHU instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  wire [15:0] result = rvfi_mem_rdata >> (8*(addr-spec_mem_addr));
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 101 && insn_opcode == 7'b 0000011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_rmask = ((1 << 2)-1) << (addr-spec_mem_addr);
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;
  assign spec_trap = ((addr & (2-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_lui (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // U-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31:12], 12'b0});
  wire [4:0] insn_rd     = rvfi_insn[11:7];
  wire [6:0] insn_opcode = rvfi_insn[ 6:0];

  wire misa_ok = 1;

  // LUI instruction
  assign spec_valid = rvfi_valid && !insn_padding && insn_opcode == 7'b 0110111;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? insn_imm : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs1_addr = 0;
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_lw (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // LW instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  wire [31:0] result = rvfi_mem_rdata >> (8*(addr-spec_mem_addr));
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 010 && insn_opcode == 7'b 0000011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_rmask = ((1 << 4)-1) << (addr-spec_mem_addr);
  assign spec_rd_wdata = spec_rd_addr ? $signed(result) : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;
  assign spec_trap = ((addr & (4-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_mul (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // MUL instruction
  wire [32-1:0] result = rvfi_rs1_rdata * rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000001 && insn_funct3 == 3'b 000 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_mulh (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // MULH instruction
  wire [32-1:0] result = ({{32{rvfi_rs1_rdata[32-1]}}, rvfi_rs1_rdata} *
		{{32{rvfi_rs2_rdata[32-1]}}, rvfi_rs2_rdata}) >> 32;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000001 && insn_funct3 == 3'b 001 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_mulhsu (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // MULHSU instruction
  wire [32-1:0] result = ({{32{rvfi_rs1_rdata[32-1]}}, rvfi_rs1_rdata} *
		{32'b0, rvfi_rs2_rdata}) >> 32;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000001 && insn_funct3 == 3'b 010 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_mulhu (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // MULHU instruction
  wire [32-1:0] result = ({32'b0, rvfi_rs1_rdata} * {32'b0, rvfi_rs2_rdata}) >> 32;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000001 && insn_funct3 == 3'b 011 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_or (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // OR instruction
  wire [32-1:0] result = rvfi_rs1_rdata | rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000000 && insn_funct3 == 3'b 110 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_ori (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // ORI instruction
  wire [32-1:0] result = rvfi_rs1_rdata | insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 110 && insn_opcode == 7'b 0010011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_rem (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // REM instruction
  reg [32-1:0] result;
  always @* begin
    if (rvfi_rs2_rdata == 32'b0) begin
      result = rvfi_rs1_rdata;
    end else begin
      if (rvfi_rs1_rdata == {1'b1, {32-1{1'b0}}} && rvfi_rs2_rdata == {32{1'b1}}) begin
        result = {32{1'b0}};
      end else begin
        result = $signed(rvfi_rs1_rdata) % $signed(rvfi_rs2_rdata);
      end
    end
  end
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000001 && insn_funct3 == 3'b 110 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_remu (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // REMU instruction
  wire [32-1:0] result = rvfi_rs2_rdata == 32'b0 ? rvfi_rs1_rdata :
                                         rvfi_rs1_rdata % rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000001 && insn_funct3 == 3'b 111 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_sb (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // S-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31:25], rvfi_insn[11:7]});
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SB instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 000 && insn_opcode == 7'b 0100011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_wmask = ((1 << 1)-1) << (addr-spec_mem_addr);
  assign spec_mem_wdata = rvfi_rs2_rdata << (8*(addr-spec_mem_addr));
  assign spec_pc_wdata = rvfi_pc_rdata + 4;
  assign spec_trap = ((addr & (1-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_rmask = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_sh (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // S-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31:25], rvfi_insn[11:7]});
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SH instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 001 && insn_opcode == 7'b 0100011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_wmask = ((1 << 2)-1) << (addr-spec_mem_addr);
  assign spec_mem_wdata = rvfi_rs2_rdata << (8*(addr-spec_mem_addr));
  assign spec_pc_wdata = rvfi_pc_rdata + 4;
  assign spec_trap = ((addr & (2-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_rmask = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_sll (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SLL instruction
  wire [5:0] shamt = 32 == 64 ? rvfi_rs2_rdata[5:0] : rvfi_rs2_rdata[4:0];
  wire [32-1:0] result = rvfi_rs1_rdata << shamt;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000000 && insn_funct3 == 3'b 001 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_slli (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format (shift variation)
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct6 = rvfi_insn[31:26];
  wire [5:0] insn_shamt  = rvfi_insn[25:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SLLI instruction
  wire [32-1:0] result = rvfi_rs1_rdata << insn_shamt;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct6 == 6'b 000000 && insn_funct3 == 3'b 001 && insn_opcode == 7'b 0010011 && (!insn_shamt[5] || 32 == 64);
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_slt (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SLT instruction
  wire [32-1:0] result = $signed(rvfi_rs1_rdata) < $signed(rvfi_rs2_rdata);
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000000 && insn_funct3 == 3'b 010 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_slti (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SLTI instruction
  wire [32-1:0] result = $signed(rvfi_rs1_rdata) < $signed(insn_imm);
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 010 && insn_opcode == 7'b 0010011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_sltiu (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SLTIU instruction
  wire [32-1:0] result = rvfi_rs1_rdata < insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 011 && insn_opcode == 7'b 0010011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_sltu (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SLTU instruction
  wire [32-1:0] result = rvfi_rs1_rdata < rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000000 && insn_funct3 == 3'b 011 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_sra (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SRA instruction
  wire [5:0] shamt = 32 == 64 ? rvfi_rs2_rdata[5:0] : rvfi_rs2_rdata[4:0];
  wire [32-1:0] result = $signed(rvfi_rs1_rdata) >>> shamt;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0100000 && insn_funct3 == 3'b 101 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_srai (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format (shift variation)
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct6 = rvfi_insn[31:26];
  wire [5:0] insn_shamt  = rvfi_insn[25:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SRAI instruction
  wire [32-1:0] result = $signed(rvfi_rs1_rdata) >>> insn_shamt;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct6 == 6'b 010000 && insn_funct3 == 3'b 101 && insn_opcode == 7'b 0010011 && (!insn_shamt[5] || 32 == 64);
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_srl (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SRL instruction
  wire [5:0] shamt = 32 == 64 ? rvfi_rs2_rdata[5:0] : rvfi_rs2_rdata[4:0];
  wire [32-1:0] result = rvfi_rs1_rdata >> shamt;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000000 && insn_funct3 == 3'b 101 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_srli (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format (shift variation)
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct6 = rvfi_insn[31:26];
  wire [5:0] insn_shamt  = rvfi_insn[25:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SRLI instruction
  wire [32-1:0] result = rvfi_rs1_rdata >> insn_shamt;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct6 == 6'b 000000 && insn_funct3 == 3'b 101 && insn_opcode == 7'b 0010011 && (!insn_shamt[5] || 32 == 64);
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_sub (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SUB instruction
  wire [32-1:0] result = rvfi_rs1_rdata - rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0100000 && insn_funct3 == 3'b 000 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_sw (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // S-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed({rvfi_insn[31:25], rvfi_insn[11:7]});
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // SW instruction
  wire [32-1:0] addr = rvfi_rs1_rdata + insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 010 && insn_opcode == 7'b 0100011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_mem_addr = addr & ~(32/8-1);
  assign spec_mem_wmask = ((1 << 4)-1) << (addr-spec_mem_addr);
  assign spec_mem_wdata = rvfi_rs2_rdata << (8*(addr-spec_mem_addr));
  assign spec_pc_wdata = rvfi_pc_rdata + 4;
  assign spec_trap = ((addr & (4-1)) != 0) || !misa_ok;

  // default assignments
  assign spec_rd_addr = 0;
  assign spec_rd_wdata = 0;
  assign spec_mem_rmask = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_xor (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // R-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [6:0] insn_funct7 = rvfi_insn[31:25];
  wire [4:0] insn_rs2    = rvfi_insn[24:20];
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // XOR instruction
  wire [32-1:0] result = rvfi_rs1_rdata ^ rvfi_rs2_rdata;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct7 == 7'b 0000000 && insn_funct3 == 3'b 100 && insn_opcode == 7'b 0110011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rs2_addr = insn_rs2;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule

// DO NOT EDIT -- auto-generated from riscv-formal/insns/generate.py

module riscv_formal_monitor_rv32imc_insn_xori (
  input                                 rvfi_valid,
  input  [32   - 1 : 0] rvfi_insn,
  input  [32   - 1 : 0] rvfi_pc_rdata,
  input  [32   - 1 : 0] rvfi_rs1_rdata,
  input  [32   - 1 : 0] rvfi_rs2_rdata,
  input  [32   - 1 : 0] rvfi_mem_rdata,

  output                                spec_valid,
  output                                spec_trap,
  output [                       4 : 0] spec_rs1_addr,
  output [                       4 : 0] spec_rs2_addr,
  output [                       4 : 0] spec_rd_addr,
  output [32   - 1 : 0] spec_rd_wdata,
  output [32   - 1 : 0] spec_pc_wdata,
  output [32   - 1 : 0] spec_mem_addr,
  output [32/8 - 1 : 0] spec_mem_rmask,
  output [32/8 - 1 : 0] spec_mem_wmask,
  output [32   - 1 : 0] spec_mem_wdata
);

  // I-type instruction format
  wire [32-1:0] insn_padding = rvfi_insn >> 32;
  wire [32-1:0] insn_imm = $signed(rvfi_insn[31:20]);
  wire [4:0] insn_rs1    = rvfi_insn[19:15];
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  wire [4:0] insn_rd     = rvfi_insn[11: 7];
  wire [6:0] insn_opcode = rvfi_insn[ 6: 0];

  wire misa_ok = 1;

  // XORI instruction
  wire [32-1:0] result = rvfi_rs1_rdata ^ insn_imm;
  assign spec_valid = rvfi_valid && !insn_padding && insn_funct3 == 3'b 100 && insn_opcode == 7'b 0010011;
  assign spec_rs1_addr = insn_rs1;
  assign spec_rd_addr = insn_rd;
  assign spec_rd_wdata = spec_rd_addr ? result : 0;
  assign spec_pc_wdata = rvfi_pc_rdata + 4;

  // default assignments
  assign spec_rs2_addr = 0;
  assign spec_trap = !misa_ok;
  assign spec_mem_addr = 0;
  assign spec_mem_rmask = 0;
  assign spec_mem_wmask = 0;
  assign spec_mem_wdata = 0;
endmodule
