`timescale 1ns/1ps
module forwarding_unit (
    input  wire       EX_MEM_regWrite,
    input  wire [4:0] EX_MEM_rd,
    input  wire       MEM_WB_regWrite,
    input  wire [4:0] MEM_WB_rd,
    input  wire [4:0] ID_EX_rs,
    input  wire [4:0] ID_EX_rt,
    output reg [1:0]  forwardA,
    output reg [1:0]  forwardB
);
    always @* begin
        forwardA = 2'b00;
        forwardB = 2'b00;
        if(EX_MEM_regWrite && (EX_MEM_rd!=0) && (EX_MEM_rd==ID_EX_rs)) forwardA = 2'b10;
        if(EX_MEM_regWrite && (EX_MEM_rd!=0) && (EX_MEM_rd==ID_EX_rt)) forwardB = 2'b10;
        if(MEM_WB_regWrite && (MEM_WB_rd!=0) && (MEM_WB_rd==ID_EX_rs)) forwardA = 2'b01;
        if(MEM_WB_regWrite && (MEM_WB_rd!=0) && (MEM_WB_rd==ID_EX_rt)) forwardB = 2'b01;
    end
endmodule
