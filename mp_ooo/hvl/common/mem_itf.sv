interface mem_itf_w_mask #(
    parameter               CHANNELS = 1,
    parameter               DWIDTH = 32,
    parameter               MWIDTH = DWIDTH / 8
)(
    input   bit             clk,
    input   bit             rst
);

    logic   [31:0]          addr    [CHANNELS];
    logic   [MWIDTH-1:0]    rmask   [CHANNELS];
    logic   [MWIDTH-1:0]    wmask   [CHANNELS];
    logic   [DWIDTH-1:0]    rdata   [CHANNELS];
    logic   [DWIDTH-1:0]    wdata   [CHANNELS];
    logic                   resp    [CHANNELS];

    bit                     error = 1'b0;

    modport dut (
        input               clk,
        input               rst,
        output              addr,
        output              rmask,
        output              wmask,
        input               rdata,
        output              wdata,
        input               resp
    );

    modport mem (
        input               clk,
        input               rst,
        input               addr,
        input               rmask,
        input               wmask,
        output              rdata,
        input               wdata,
        output              resp,
        output              error
    );

    modport mon (
        input               clk,
        input               rst,
        input               addr,
        input               rmask,
        input               wmask,
        input               rdata,
        input               wdata,
        input               resp,
        output              error
    );

endinterface

interface mem_itf_wo_mask #(
    parameter               CHANNELS = 1,
    parameter               DWIDTH = 32
)(
    input   bit             clk,
    input   bit             rst
);

    logic   [31:0]          addr    [CHANNELS];
    logic                   read    [CHANNELS];
    logic                   write   [CHANNELS];
    logic   [DWIDTH-1:0]    rdata   [CHANNELS];
    logic   [DWIDTH-1:0]    wdata   [CHANNELS];
    logic                   resp    [CHANNELS];

    bit                     error = 1'b0;

    modport dut (
        input               clk,
        input               rst,
        output              addr,
        output              read,
        output              write,
        input               rdata,
        output              wdata,
        input               resp
    );

    modport mem (
        input               clk,
        input               rst,
        input               addr,
        input               read,
        input               write,
        output              rdata,
        input               wdata,
        output              resp,
        output              error
    );

    modport mon (
        input               clk,
        input               rst,
        input               addr,
        input               read,
        input               write,
        input               rdata,
        input               wdata,
        input               resp,
        output              error
    );

endinterface

interface mem_itf_banked(
    input   bit             clk,
    input   bit             rst
);

    logic   [31:0]          addr;
    logic                   read;
    logic                   write;
    logic   [63:0]          wdata;
    logic                   ready;
    logic   [31:0]          raddr;
    logic   [63:0]          rdata;
    logic                   rvalid;

    bit                     error = 1'b0;

    modport dut (
        input               clk,
        input               rst,
        output              addr,
        output              read,
        output              write,
        output              wdata,
        input               ready,
        input               raddr,
        input               rdata,
        input               rvalid
    );

    modport mem (
        input               clk,
        input               rst,
        input               addr,
        input               read,
        input               write,
        input               wdata,
        output              ready,
        output              rdata,
        output              raddr,
        output              rvalid,
        output              error
    );

    modport mon (
        input               clk,
        input               rst,
        input               addr,
        input               read,
        input               write,
        input               wdata,
        input               ready,
        input               rdata,
        input               raddr,
        input               rvalid,
        input               error
    );

endinterface
