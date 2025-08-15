`timescale 1ns/1ps
module bpu #(
    parameter BHT_ENTRIES=64,
    parameter BTB_ENTRIES=32
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] if_pc,
    output wire        pred_taken,
    output wire [31:0] pred_target,
    output wire        pred_hit,
    input  wire        update_en,
    input  wire [31:0] update_pc,
    input  wire        update_taken,
    input  wire [31:0] update_target
);
    localparam BHTW = $clog2(BHT_ENTRIES);
    reg [1:0] bht[BHT_ENTRIES-1:0];
    integer i;
    initial for (i=0;i<BHT_ENTRIES;i=i+1) bht[i]=2'b01;
    wire [BHTW-1:0] bht_idx_if = if_pc[BHTW+1:2];
    wire [BHTW-1:0] bht_idx_up = update_pc[BHTW+1:2];
    assign pred_taken = bht[bht_idx_if][1];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin end
        else if(update_en) begin
            case ({update_taken, bht[bht_idx_up]})
                3'b1_00: bht[bht_idx_up]<=2'b01;
                3'b1_01: bht[bht_idx_up]<=2'b10;
                3'b1_10: bht[bht_idx_up]<=2'b11;
                3'b1_11: bht[bht_idx_up]<=2'b11;
                3'b0_00: bht[bht_idx_up]<=2'b00;
                3'b0_01: bht[bht_idx_up]<=2'b00;
                3'b0_10: bht[bht_idx_up]<=2'b01;
                3'b0_11: bht[bht_idx_up]<=2'b10;
            endcase
        end
    end

    localparam BTBW = $clog2(BTB_ENTRIES);
    reg [31:0] btb_target[BTB_ENTRIES-1:0];
    reg [19:0] btb_tag[BTB_ENTRIES-1:0];
    reg        btb_valid[BTB_ENTRIES-1:0];
    integer j;
    initial for (j=0;j<BTB_ENTRIES;j=j+1) begin
        btb_target[j]=32'b0; btb_tag[j]=20'b0; btb_valid[j]=1'b0;
    end
    wire [BTBW-1:0] btb_idx_if = if_pc[BTBW+1:2];
    wire [19:0]     tag_if     = if_pc[31:12];
    wire [BTBW-1:0] btb_idx_up = update_pc[BTBW+1:2];
    wire [19:0]     tag_up     = update_pc[31:12];

    assign pred_target = btb_target[btb_idx_if];
    assign pred_hit    = btb_valid[btb_idx_if] & (btb_tag[btb_idx_if]==tag_if);

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin end
        else if(update_en && update_taken) begin
            btb_valid[btb_idx_up]  <= 1'b1;
            btb_target[btb_idx_up] <= update_target;
            btb_tag[btb_idx_up]    <= tag_up;
        end
    end
endmodule
