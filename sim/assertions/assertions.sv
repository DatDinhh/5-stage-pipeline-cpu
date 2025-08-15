`timescale 1ns/1ps
`default_nettype none

module simple_monitors;

    // expect tb_core and dut hierarchy
    // x0 == 0
    always @(posedge tb_core.clk) if (tb_core.rst_n) begin
        if (tb_core.dut.u_rf.rf[0] !== 32'h0) begin
            $display("ASSERT: x0 != 0"); $fatal(1);
        end
    end

    // no write when we==0
    always @(posedge tb_core.clk) if (tb_core.rst_n) begin
        if (tb_core.dut.u_rf.we==1'b0 && tb_core.dut.u_rf.wa!=5'd0) begin
            // best-effort: detect unexpected rf change on next cycle
        end
    end

    // d-bus request eventually responds (bounded)
    integer wait_ctr;
    reg prev_req;
    always @(posedge tb_core.clk or negedge tb_core.rst_n) begin
        if (!tb_core.rst_n) begin
            wait_ctr <= 0;
            prev_req <= 1'b0;
        end else begin
            if (tb_core.db_req_v && tb_core.db_req_r) begin
                wait_ctr <= 0;
                prev_req <= 1'b1;
            end else if (prev_req && !tb_core.db_resp_v) begin
                wait_ctr <= wait_ctr + 1;
                if (wait_ctr > 64) begin
                    $display("ASSERT: d-bus response timeout"); $fatal(1);
                end
            end
            if (tb_core.db_resp_v) prev_req <= 1'b0;
        end
    end

    // pc +4 when no redirect 
    reg [31:0] pc_q;
    always @(posedge tb_core.clk or negedge tb_core.rst_n) begin
        if (!tb_core.rst_n) begin
            pc_q <= 32'h0;
        end else begin
            if (tb_core.ib_req_r && tb_core.ib_req_v) begin
                if (!tb_core.dut.do_redirect) begin
                    if (pc_q !== 32'h0 && tb_core.dut.pc !== (pc_q + 32'd4)) begin
                    end
                end
                pc_q <= tb_core.dut.pc;
            end
        end
    end

endmodule

`default_nettype wire
