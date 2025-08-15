`timescale 1ns/1ps

module scoreboard (
    input  clk,
    input  rst_n,

    input        dbus_req_valid,
    input        dbus_req_ready,
    input [31:0] dbus_req_addr,
    input [31:0] dbus_req_wdata,
    input [3:0]  dbus_req_wstrb,

    input        dbus_resp_valid,
    input [31:0] dbus_resp_rdata,   

    input [1:0]  ex_mem_memSize,   
    input        ex_mem_loadSigned,

    input        mem_wb_valid,
    input        mem_wb_regWrite,
    input [4:0]  mem_wb_regDest,
    input        mem_wb_memToReg,
    input        mem_wb_jal,
    input [31:0] mem_wb_wdata
);
  // ----------------------------
  // Parameters / storage
  // ----------------------------
  localparam QDEPTH     = 128;
  localparam MEM_WORDS  = 65536;          
  localparam [31:0] CSR_BASE_MASK = 32'hFFFF_FF00;
  localparam [31:0] CSR_BASE      = 32'hFFFF_F000;

  // FIFO
  reg        q_is_store [0:QDEPTH-1];
  reg [31:0] q_addr     [0:QDEPTH-1];
  reg [31:0] q_wdata    [0:QDEPTH-1];
  reg [3:0]  q_wstrb    [0:QDEPTH-1];
  reg [1:0]  q_size     [0:QDEPTH-1];
  reg        q_signed   [0:QDEPTH-1];
  reg        q_is_csr   [0:QDEPTH-1];

  integer q_head, q_tail, q_count;

  reg [31:0] mirror_mem [0:MEM_WORDS-1];

  reg [31:0] t_addr, t_wdata, before_w, after_w, expected_w;
  reg [3:0]  t_wstrb;
  reg [1:0]  t_size;
  reg        t_signed, t_is_store, t_is_csr;
  integer    idx, i;

  wire req_fire  = dbus_req_valid & dbus_req_ready;
  wire resp_fire = dbus_resp_valid;

  // ----------------------------
  // Helpers
  // ----------------------------
  function [31:0] rd_word;
    input [31:0] addr;
    integer li;
    begin
      li = addr[17:2]; // 256 KiB window
      if (li >= 0 && li < MEM_WORDS) rd_word = mirror_mem[li];
      else                           rd_word = 32'h0000_0000;
    end
  endfunction

  function [31:0] wr_merge;
    input [31:0] oldw;
    input [31:0] wdata;
    input [3:0]  wstrb;
    reg   [31:0] r;
    begin
      r = oldw;
      if (wstrb[0]) r[ 7: 0] = wdata[ 7: 0];
      if (wstrb[1]) r[15: 8] = wdata[15: 8];
      if (wstrb[2]) r[23:16] = wdata[23:16];
      if (wstrb[3]) r[31:24] = wdata[31:24];
      wr_merge = r;
    end
  endfunction

  function [31:0] load_align;
    input [31:0] rdata;
    input [1:0]  size;
    input [31:0] addr;
    input        sgn;
    reg   [31:0] x;
    begin
      case (size)
        2'b10: begin // byte
          case (addr[1:0])
            2'd0: x = {24'b0, rdata[ 7: 0]};
            2'd1: x = {24'b0, rdata[15: 8]};
            2'd2: x = {24'b0, rdata[23:16]};
            default: x = {24'b0, rdata[31:24]};
          endcase
          if (sgn) x = {{24{x[7]}}, x[7:0]};
        end
        2'b01: begin // half
          x = (addr[1]==1'b0) ? {16'b0, rdata[15:0]} : {16'b0, rdata[31:16]};
          if (sgn) x = {{16{x[15]}}, x[15:0]};
        end
        default: x = rdata; // word
      endcase
      load_align = x;
    end
  endfunction

  // ----------------------------
  // Reset/init
  // ----------------------------
  initial begin
    q_head  = 0; q_tail = 0; q_count = 0;
    for (i=0; i<MEM_WORDS; i=i+1) mirror_mem[i] = 32'h0;
  end

  // ----------------------------
  // FIFO push on request
  // ----------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      q_head  <= 0;
      q_tail  <= 0;
      q_count <= 0;
    end else begin
      if (req_fire) begin
        q_addr[q_tail]     <= {dbus_req_addr[31:2],2'b00};
        q_wdata[q_tail]    <= dbus_req_wdata;
        q_wstrb[q_tail]    <= dbus_req_wstrb;
        q_is_store[q_tail] <= |dbus_req_wstrb;
        q_size[q_tail]     <= ex_mem_memSize;
        q_signed[q_tail]   <= ex_mem_loadSigned;
        q_is_csr[q_tail]   <= ((dbus_req_addr & CSR_BASE_MASK) == CSR_BASE);

        q_tail  <= (q_tail==(QDEPTH-1)) ? 0 : (q_tail+1);
        if (q_count < QDEPTH) q_count <= q_count + 1;
        else begin $display("SB: FIFO overflow"); $fatal(1); end
      end
    end
  end

  // ----------------------------
  // Pop on response, mirror, check
  // ----------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
    end else if (resp_fire) begin
      if (q_count == 0) begin
        $display("SB: unexpected response"); $fatal(1);
      end else begin
        t_addr     = q_addr[q_head];
        t_wdata    = q_wdata[q_head];
        t_wstrb    = q_wstrb[q_head];
        t_size     = q_size[q_head];
        t_signed   = q_signed[q_head];
        t_is_store = q_is_store[q_head];
        t_is_csr   = q_is_csr[q_head];

        q_head  <= (q_head==(QDEPTH-1)) ? 0 : (q_head+1);
        q_count <= q_count - 1;

        if (!t_is_csr) begin
          if (t_is_store) begin
            idx      = t_addr[17:2];
            before_w = rd_word(t_addr);
            after_w  = wr_merge(before_w, t_wdata, t_wstrb);
            if (idx >= 0 && idx < MEM_WORDS) mirror_mem[idx] <= after_w;

            if (mem_wb_valid && mem_wb_memToReg && !mem_wb_jal) begin
              $display("SB: store produced reg write"); $fatal(1);
            end
          end else begin
            expected_w = load_align(rd_word(t_addr), t_size, t_addr, t_signed);

            if (!(mem_wb_valid && mem_wb_memToReg && (mem_wb_regDest!=5'd0))) begin
              $display("SB: load not committed correctly"); $fatal(1);
            end
            if (mem_wb_wdata !== expected_w) begin
              $display("SB: load mismatch addr=%h exp=%h got=%h", t_addr, expected_w, mem_wb_wdata);
              $fatal(1);
            end
          end
        end
      end
    end
  end

endmodule
