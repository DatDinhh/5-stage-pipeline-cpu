`timescale 1ns/1ps
module rv_mem #(
    parameter BYTES = 4096,
    parameter RANDOM_WAIT = 1,
    parameter WAIT_MAX = 12
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        req_valid,
    output reg         req_ready,
    input  wire [31:0] req_addr,
    input  wire [31:0] req_wdata,
    input  wire [3:0]  req_wstrb,
    output reg         resp_valid,
    input  wire        resp_ready,
    output reg  [31:0] resp_rdata
);
    localparam WORDS = BYTES/4;
    reg [31:0] mem [0:WORDS-1];
    integer i;
    initial begin
        for (i=0;i<WORDS;i=i+1) mem[i]=32'h0000_0000;
    end
    reg [31:0] lat_addr, lat_wdata;
    reg [3:0]  lat_wstrb;
    reg [3:0]  wait_cnt;
    reg [7:0]  lfsr;
    reg        busy;
    wire       wait_done = (wait_cnt==0);
    wire [31:0] aligned_addr = {req_addr[31:2],2'b00};

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            req_ready<=1'b0; resp_valid<=1'b0; busy<=1'b0; wait_cnt<=0; lfsr<=8'hA5;
            lat_addr<=32'b0; lat_wdata<=32'b0; lat_wstrb<=4'b0000; resp_rdata<=32'b0;
        end else begin
            lfsr <= {lfsr[6:0], lfsr[7]^lfsr[5]^lfsr[4]^lfsr[3]};
            req_ready <= ~busy;
            if (req_valid & req_ready) begin
                lat_addr  <= aligned_addr;
                lat_wdata <= req_wdata;
                lat_wstrb <= req_wstrb;
                busy      <= 1'b1;
                wait_cnt  <= RANDOM_WAIT ? (lfsr % (WAIT_MAX+1)) : 4'd0;
            end
            if (busy && wait_done) begin
                if (|lat_wstrb) begin
                    if (lat_wstrb[0]) mem[lat_addr[31:2]][7:0]   <= lat_wdata[7:0];
                    if (lat_wstrb[1]) mem[lat_addr[31:2]][15:8]  <= lat_wdata[15:8];
                    if (lat_wstrb[2]) mem[lat_addr[31:2]][23:16] <= lat_wdata[23:16];
                    if (lat_wstrb[3]) mem[lat_addr[31:2]][31:24] <= lat_wdata[31:24];
                end
                resp_rdata <= mem[lat_addr[31:2]];
                resp_valid <= 1'b1;
                busy       <= 1'b0;
            end else begin
                resp_valid <= (resp_valid & ~resp_ready);
                if (busy && wait_cnt!=0) wait_cnt <= wait_cnt - 1;
            end
        end
    end
endmodule
