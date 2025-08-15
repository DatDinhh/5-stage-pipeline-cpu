`timescale 1ns/1ps
module reg_file (
    input  wire        clk,
    input  wire        we,
    input  wire [4:0]  ra1,
    input  wire [4:0]  ra2,
    input  wire [4:0]  wa,
    input  wire [31:0] wd,
    output wire [31:0] rd1,
    output wire [31:0] rd2
);
    reg [31:0] rf[31:0];
    integer i;
    initial begin
        for (i=0;i<32;i=i+1) rf[i] = 32'b0;
    end
    always @(posedge clk) begin
        if (we && wa != 0) rf[wa] <= wd;
    end
    assign rd1 = (ra1 == 0) ? 32'b0 : rf[ra1];
    assign rd2 = (ra2 == 0) ? 32'b0 : rf[ra2];
endmodule
