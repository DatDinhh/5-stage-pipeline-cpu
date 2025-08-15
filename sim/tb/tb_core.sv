`timescale 1ns/1ps
`default_nettype none

module tb_core;

    reg clk=0, rst_n=0; always #5 clk=~clk;

    // IF
    wire         ib_req_v, ib_req_r, ib_resp_v, ib_resp_r;
    wire [31:0]  ib_req_a, ib_resp_d;
    assign ib_resp_r = 1'b1;

    // D
    wire         db_req_v, db_req_r, db_resp_v, db_resp_r;
    wire [31:0]  db_req_a, db_req_wd, db_resp_d;
    wire [3:0]   db_req_ws;
    assign db_resp_r = 1'b1;

    // DUT
    cpu_core dut (
        .clk(clk), .rst_n(rst_n),
        .ibus_req_valid(ib_req_v), .ibus_req_ready(ib_req_r),
        .ibus_req_addr(ib_req_a), .ibus_resp_valid(ib_resp_v), .ibus_resp_ready(ib_resp_r), .ibus_resp_rdata(ib_resp_d),
        .dbus_req_valid(db_req_v), .dbus_req_ready(db_req_r),
        .dbus_req_addr(db_req_a), .dbus_req_wdata(db_req_wd), .dbus_req_wstrb(db_req_ws),
        .dbus_resp_valid(db_resp_v), .dbus_resp_ready(db_resp_r), .dbus_resp_rdata(db_resp_d)
    );

    // Memories
    rv_rom #(.BYTES(4096), .RANDOM_WAIT(1), .WAIT_MAX(5)) irom (
        .clk(clk), .rst_n(rst_n),
        .req_valid(ib_req_v), .req_ready(ib_req_r), .req_addr(ib_req_a),
        .resp_valid(ib_resp_v), .resp_ready(ib_resp_r), .resp_rdata(ib_resp_d)
    );

    rv_mem #(.BYTES(4096), .RANDOM_WAIT(1), .WAIT_MAX(12)) dram (
        .clk(clk), .rst_n(rst_n),
        .req_valid(db_req_v), .req_ready(db_req_r),
        .req_addr(db_req_a), .req_wdata(db_req_wd), .req_wstrb(db_req_ws),
        .resp_valid(db_resp_v), .resp_ready(db_resp_r), .resp_rdata(db_resp_d)
    );

    wire [1:0]  ex_mem_memSize    = dut.EX_MEM_memSize;
    wire        ex_mem_loadSigned = dut.EX_MEM_loadSigned;

    wire        mem_wb_valid    = dut.MEM_WB_valid;
    wire        mem_wb_regWrite = dut.MEM_WB_regWrite;
    wire [4:0]  mem_wb_regDest  = dut.MEM_WB_regDest;
    wire        mem_wb_memToReg = dut.MEM_WB_memToReg;
    wire        mem_wb_jal      = dut.MEM_WB_jal;
    wire [31:0] mem_wb_wdata    = dut.MEM_WB_jal ? dut.MEM_WB_pc_link :
                                  (dut.MEM_WB_memToReg ? dut.MEM_WB_memData : dut.MEM_WB_aluResult);

    // Scoreboard
    scoreboard u_sb (
        .clk(clk), .rst_n(rst_n),
        .dbus_req_valid(db_req_v), .dbus_req_ready(db_req_r),
        .dbus_req_addr(db_req_a), .dbus_req_wdata(db_req_wd), .dbus_req_wstrb(db_req_ws),
        .dbus_resp_valid(db_resp_v), .dbus_resp_rdata(db_resp_d),
        .ex_mem_memSize(ex_mem_memSize), .ex_mem_loadSigned(ex_mem_loadSigned),
        .mem_wb_valid(mem_wb_valid), .mem_wb_regWrite(mem_wb_regWrite),
        .mem_wb_regDest(mem_wb_regDest), .mem_wb_memToReg(mem_wb_memToReg),
        .mem_wb_jal(mem_wb_jal), .mem_wb_wdata(mem_wb_wdata)
    );

    // Waves / reset / finish
    initial begin
        $dumpfile("sim/waves/core.vcd");
        $dumpvars(0, tb_core);
        repeat(10) @(posedge clk);
        rst_n = 1'b1;
        repeat(3000) @(posedge clk);
        $finish;
    end

endmodule

`default_nettype wire
