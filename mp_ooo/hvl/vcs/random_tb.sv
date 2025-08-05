module random_tb
import rv32i_types::*;
(
    mem_itf_banked.mem itf
);
    int num_rand_insts = 50000;
    `include "randinst.svh"

    RandInst gen[8];

    initial
        for (int i = 0; i < 8; i++) gen[i] = new();

    // Do a bunch of LUI/LIs to get useful register state.
    // Thirty-two 32-bit registers
    // Each 'bursts' contains 256-bits, can initialize 256/32 = 8 insts at a time
    //
    task init_register_state();
        logic [255:0] bursts;
        logic [31:0]  captured_addr;

        for (int i = 0; i < 4; i++) begin

            @(posedge itf.clk iff (itf.read));
            captured_addr = itf.addr;

            for (int j = 0; j < 8; j++) begin
                if (j%2 == 0) begin //LUI
                    gen[j].randomize() with {
                        instr.j_type.opcode == op_b_lui;
                        instr.j_type.rd == 5'(8*i + j);
                    };
                end
                else begin  //LI (ADDI RD, X0, IMM)
                    gen[j].randomize() with {
                        instr.i_type.opcode == op_b_imm;
                        instr.i_type.rs1    == '0;
                        instr.i_type.funct3 == arith_f3_add;
                        instr.i_type.rd     == 5'(8*i + j);
                    };
                end

                bursts[j*32 +: 32] = gen[j].instr.word;
            end

            // send four bursts of data
            @(posedge itf.clk);
            itf.raddr <= captured_addr;
            itf.rdata <= bursts[63:0];
            itf.rvalid <= 1'b1;

            @(posedge itf.clk);
            itf.rdata <= bursts[127:64];

            @(posedge itf.clk);
            itf.rdata <= bursts[191:128];

            @(posedge itf.clk);
            itf.rdata <= bursts[255:192];
            
            @(posedge itf.clk);
            itf.raddr <= 'x;
            itf.rvalid <= 1'b0;
            itf.rdata <= 'x;
        end

    endtask : init_register_state

    task run_random_instrs();
        repeat (num_rand_insts / 8) begin
            logic [255:0] bursts;
            logic [31:0]  captured_addr;

            @(posedge itf.clk iff (itf.read));

            captured_addr = itf.addr;
            for (int i = 0; i < 8; i++) begin
                gen[i].randomize();
                bursts[i*32 +: 32] = gen[i].instr.word;
            end

            // send four bursts of data
            @(posedge itf.clk);
            itf.raddr <= captured_addr;
            itf.rdata <= bursts[63:0];
            itf.rvalid <= 1'b1;

            @(posedge itf.clk);
            itf.rdata <= bursts[127:64];

            @(posedge itf.clk);
            itf.rdata <= bursts[191:128];

            @(posedge itf.clk);
            itf.rdata <= bursts[255:192];
            
            @(posedge itf.clk);
            itf.raddr <= 'x;
            itf.rvalid <= 1'b0;
            itf.rdata <= 'x;
        end
    endtask : run_random_instrs

    always @(posedge itf.clk iff !itf.rst) begin
        if ($isunknown(itf.read) || $isunknown(itf.write)) begin
            $error("Memory Error: read containes 1'bx");
            itf.error <= 1'b1;
        end
        if ((|itf.read) && (|itf.write)) begin
            $error("Memory Error: Simultaneous memory read and write");
            itf.error <= 1'b1;
        end
        if ((|itf.read) || (|itf.write)) begin
            if ($isunknown(itf.addr[0])) begin
                $error("Memory Error: Address contained 'x");
                itf.error <= 1'b1;
            end
            // 256-bit (32B) alignment requirement
            if (itf.addr[4:0]!= 5'd0) begin
                $error("Memory Error: Address is not 256-bit aligned");
                itf.error <= 1'b1;
            end
        end
    end

    // A single initial block ensures random stability.
    initial begin
        itf.ready  = '1;
        itf.raddr  = 'x;
        itf.rdata  = 'x;
        itf.rvalid = '0;

        @(posedge itf.clk iff itf.rst == 1'b0);

        // Get some useful state into the processor by loading in a bunch of state.
        init_register_state();

        // Run!
        run_random_instrs();

        // Finish up
        $display("Random testbench finished!");
        $finish;
    end

endmodule : random_tb