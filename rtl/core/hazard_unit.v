`timescale 1ns/1ps
module hazard_unit (
    input  wire       ID_EX_memRead,
    input  wire [4:0] ID_EX_rt,
    input  wire [4:0] IF_ID_rs,
    input  wire [4:0] IF_ID_rt,
    input  wire       IF_ID_usesRt,
    output reg        pcWrite,
    output reg        IF_ID_Write,
    output reg        controlBubble
);
    always @* begin
        pcWrite      = 1'b1;
        IF_ID_Write  = 1'b1;
        controlBubble= 1'b0;
        if (ID_EX_memRead && (ID_EX_rt!=0) &&
            ((ID_EX_rt==IF_ID_rs) || (IF_ID_usesRt && (ID_EX_rt==IF_ID_rt)))) begin
            pcWrite      = 1'b0;
            IF_ID_Write  = 1'b0;
            controlBubble= 1'b1;
        end
    end
endmodule
