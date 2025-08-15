`timescale 1ns/1ps

module cpu_core (
    input  wire        clk,
    input  wire        rst_n,

    output reg         ibus_req_valid,
    input  wire        ibus_req_ready,
    output reg  [31:0] ibus_req_addr,
    input  wire        ibus_resp_valid,
    output wire        ibus_resp_ready,
    input  wire [31:0] ibus_resp_rdata,

    output reg         dbus_req_valid,
    input  wire        dbus_req_ready,
    output reg  [31:0] dbus_req_addr,
    output reg  [31:0] dbus_req_wdata,
    output reg  [3:0]  dbus_req_wstrb,
    input  wire        dbus_resp_valid,
    output wire        dbus_resp_ready,
    input  wire [31:0] dbus_resp_rdata
);

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // Const
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    localparam RESET_PC   = 32'h0000_0000;
    localparam TRAP_VEC   = 32'h0000_0080;
    localparam MSIZE_W    = 2'b00;
    localparam MSIZE_H    = 2'b01;
    localparam MSIZE_B    = 2'b10;
    localparam CSR_BASE   = 32'hFFFF_F000;

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // IF state
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    reg  [31:0] pc;
    wire [31:0] pc_plus4 = pc + 32'd4;

    wire        bpu_pred_taken;
    wire        bpu_pred_hit;
    wire [31:0] bpu_pred_target;

    reg         bpu_upd_en;
    reg  [31:0] bpu_upd_pc;
    reg         bpu_upd_taken;
    reg  [31:0] bpu_upd_target;

    bpu #(.BHT_ENTRIES(64), .BTB_ENTRIES(32)) u_bpu (
        .clk(clk), .rst_n(rst_n),
        .if_pc(pc),
        .pred_taken(bpu_pred_taken),
        .pred_target(bpu_pred_target),
        .pred_hit(bpu_pred_hit),
        .update_en(bpu_upd_en),
        .update_pc(bpu_upd_pc),
        .update_taken(bpu_upd_taken),
        .update_target(bpu_upd_target)
    );

    reg         if_busy;
    reg         if_kill;
    reg  [31:0] if_req_pc;
    reg         if_req_pred_taken;
    reg  [31:0] if_req_pred_target;

    reg         if_buf_valid;
    reg  [31:0] if_buf_instr;
    reg  [31:0] if_buf_pc_plus4;
    reg         if_buf_pred_taken;
    reg  [31:0] if_buf_pred_target;

    assign ibus_resp_ready = 1'b1;

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // IF/ID regs
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    reg         IF_ID_valid;
    reg  [31:0] IF_ID_instr;
    reg  [31:0] IF_ID_pc;
    reg         IF_ID_pred_taken;
    reg  [31:0] IF_ID_pred_target;

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // ID/EX regs
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    reg         ID_EX_valid;
    reg  [31:0] ID_EX_pc;
    reg  [31:0] ID_EX_regData1;
    reg  [31:0] ID_EX_regData2;
    reg  [31:0] ID_EX_immExt;
    reg  [4:0]  ID_EX_rs;
    reg  [4:0]  ID_EX_rt;
    reg  [4:0]  ID_EX_rd;
    reg  [4:0]  ID_EX_shamt;
    reg  [5:0]  ID_EX_funct;
    reg  [5:0]  ID_EX_opcode;
    reg         ID_EX_regWrite;
    reg         ID_EX_memRead;
    reg         ID_EX_memWrite;
    reg         ID_EX_memToReg;
    reg         ID_EX_branch;
    reg         ID_EX_branch_ne;
    reg  [2:0]  ID_EX_aluOp;
    reg         ID_EX_aluSrc;
    reg         ID_EX_regDst;
    reg         ID_EX_jal;
    reg  [1:0]  ID_EX_memSize;
    reg         ID_EX_loadSigned;

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // EX/MEM regs
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    reg         EX_MEM_valid;
    reg  [31:0] EX_MEM_aluResult;
    reg  [31:0] EX_MEM_storeData;
    reg  [31:0] EX_MEM_pc_link;
    reg  [4:0]  EX_MEM_regDest;
    reg         EX_MEM_regWrite;
    reg         EX_MEM_memRead;
    reg         EX_MEM_memWrite;
    reg         EX_MEM_memToReg;
    reg         EX_MEM_jal;
    reg  [1:0]  EX_MEM_memSize;
    reg         EX_MEM_loadSigned;

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // MEM/WB regs
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    reg         MEM_WB_valid;
    reg  [31:0] MEM_WB_aluResult;
    reg  [31:0] MEM_WB_memData;
    reg  [31:0] MEM_WB_pc_link;
    reg  [4:0]  MEM_WB_regDest;
    reg         MEM_WB_regWrite;
    reg         MEM_WB_memToReg;
    reg         MEM_WB_jal;

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // Decode
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    wire [5:0] opcode = IF_ID_instr[31:26];
    wire [4:0] rs     = IF_ID_instr[25:21];
    wire [4:0] rt     = IF_ID_instr[20:16];
    wire [4:0] rd     = IF_ID_instr[15:11];
    wire [4:0] shamt  = IF_ID_instr[10:6];
    wire [5:0] funct  = IF_ID_instr[5:0];
    wire [15:0] imm   = IF_ID_instr[15:0];

    wire [31:0] immExt;
    wire        regDst, aluSrc, memToReg, regWrite, memRead, memWrite, branch, branch_ne, jump, jal, immZeroExt, loadSigned, eret;
    wire [2:0]  aluOp;
    wire [1:0]  memSize;

    control_unit u_ctrl (
        .opcode(opcode),
        .funct(funct),
        .regDst(regDst),
        .aluSrc(aluSrc),
        .memToReg(memToReg),
        .regWrite(regWrite),
        .memRead(memRead),
        .memWrite(memWrite),
        .branch(branch),
        .branch_ne(branch_ne),
        .aluOp(aluOp),
        .jump(jump),
        .jal(jal),
        .immZeroExt(immZeroExt),
        .memSize(memSize),
        .loadSigned(loadSigned),
        .eret(eret)
    );

    assign immExt = immZeroExt ? {16'b0, imm} : {{16{imm[15]}}, imm};

    wire [31:0] jump_target_ID = {IF_ID_pc[31:28], IF_ID_instr[25:0], 2'b00};
    wire        IF_ID_usesRt = (opcode==6'b000000) | (opcode==6'b101011) | (opcode==6'b000100) | (opcode==6'b000101);

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // Register File
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    wire [31:0] regData1, regData2;

    reg_file u_rf (
        .clk(clk),
        .we(MEM_WB_regWrite),
        .ra1(IF_ID_instr[25:21]),
        .ra2(IF_ID_instr[20:16]),
        .wa(MEM_WB_regDest),
        .wd(MEM_WB_jal ? MEM_WB_pc_link : (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_aluResult)),
        .rd1(regData1),
        .rd2(regData2)
    );

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // Hazards & Forwarding
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    wire [1:0] forwardA, forwardB;

    forwarding_unit u_fwd (
        .EX_MEM_regWrite(EX_MEM_regWrite),
        .EX_MEM_rd(EX_MEM_regDest),
        .MEM_WB_regWrite(MEM_WB_regWrite),
        .MEM_WB_rd(MEM_WB_regDest),
        .ID_EX_rs(ID_EX_rs),
        .ID_EX_rt(ID_EX_rt),
        .forwardA(forwardA),
        .forwardB(forwardB)
    );

    wire hz_pcWrite, hz_IF_ID_Write, hz_controlBubble;

    hazard_unit u_hz (
        .ID_EX_memRead(ID_EX_memRead),
        .ID_EX_rt(ID_EX_rt),
        .IF_ID_rs(rs),
        .IF_ID_rt(rt),
        .IF_ID_usesRt(IF_ID_usesRt),
        .pcWrite(hz_pcWrite),
        .IF_ID_Write(hz_IF_ID_Write),
        .controlBubble(hz_controlBubble)
    );

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // ALU & Branch
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    wire [31:0] fwdA = (forwardA==2'b10) ? EX_MEM_aluResult :
                       (forwardA==2'b01) ? (MEM_WB_jal?MEM_WB_pc_link:(MEM_WB_memToReg?MEM_WB_memData:MEM_WB_aluResult)) :
                                           ID_EX_regData1;

    wire [31:0] fwdB = (forwardB==2'b10) ? EX_MEM_aluResult :
                       (forwardB==2'b01) ? (MEM_WB_jal?MEM_WB_pc_link:(MEM_WB_memToReg?MEM_WB_memData:MEM_WB_aluResult)) :
                                           ID_EX_regData2;

    wire        br_eq = (fwdA == fwdB);
    wire        takeBranch_EX = ID_EX_branch & (ID_EX_branch_ne ? ~br_eq : br_eq);
    wire [31:0] aluIn2 = ID_EX_aluSrc ? ID_EX_immExt : fwdB;

    wire [31:0] aluOut;
    wire        aluZero;

    alu u_alu (
        .in1(fwdA),
        .in2(aluIn2),
        .shamt(ID_EX_shamt),
        .aluOp(ID_EX_aluOp),
        .funct(ID_EX_funct),
        .zero(aluZero),
        .result(aluOut)
    );

    wire [31:0] br_target_EX = ID_EX_pc + (ID_EX_immExt << 2);
    wire [4:0]  regDest_EX   = ID_EX_jal ? 5'd31 : (ID_EX_regDst ? ID_EX_rd : ID_EX_rt);
    wire [31:0] storeData_EX = fwdB;
    wire [31:0] instr_pc_EX  = ID_EX_pc - 32'd4;

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // Traps (misaligned LW/SW)
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    wire [1:0] addr_low_EX = aluOut[1:0];
    wire mis_w_EX = (ID_EX_memWrite && (ID_EX_memSize==MSIZE_W) && (addr_low_EX!=2'b00));
    wire mis_r_EX = (ID_EX_memRead  && (ID_EX_memSize==MSIZE_W) && (addr_low_EX!=2'b00));
    wire trap_EX  = mis_w_EX | mis_r_EX;
    wire [31:0] trap_cause_EX = mis_r_EX ? 32'd1 : (mis_w_EX ? 32'd2 : 32'd0);

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // Predictor Update & Redirects
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    wire mispredict_EX = (ID_EX_valid & ID_EX_branch) &
                         ((takeBranch_EX != IF_ID_pred_taken) | (takeBranch_EX && (IF_ID_pred_target != br_target_EX)));

    always @* begin
        bpu_upd_en     = ID_EX_valid & ID_EX_branch;
        bpu_upd_pc     = instr_pc_EX;
        bpu_upd_taken  = takeBranch_EX;
        bpu_upd_target = br_target_EX;
    end

    wire do_jump_ID   = (jump | jal) & IF_ID_valid;
    wire do_eret_ID   = eret & IF_ID_valid;

    wire [31:0] pc_pred_IF = (bpu_pred_taken & bpu_pred_hit) ? bpu_pred_target : pc_plus4;

    wire [31:0] pc_redirect_target =
          trap_EX       ? TRAP_VEC :
          do_eret_ID    ? csr_epc_o :
          mispredict_EX ? (takeBranch_EX ? br_target_EX : ID_EX_pc) :
          do_jump_ID    ? jump_target_ID :
                          pc_pred_IF;

    wire do_redirect = trap_EX | do_eret_ID | mispredict_EX | do_jump_ID;

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // IF bus + PC
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pc <= RESET_PC;
        end else if (do_redirect) begin
            pc <= pc_redirect_target;
        end else if (ibus_req_valid && ibus_req_ready && ~if_busy) begin
            pc <= pc_pred_IF;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ibus_req_valid      <= 1'b0;
            ibus_req_addr       <= 32'b0;
            if_busy             <= 1'b0;
            if_kill             <= 1'b0;
            if_req_pc           <= 32'b0;
            if_req_pred_taken   <= 1'b0;
            if_req_pred_target  <= 32'b0;
            if_buf_valid        <= 1'b0;
            if_buf_instr        <= 32'b0;
            if_buf_pc_plus4     <= 32'b0;
            if_buf_pred_taken   <= 1'b0;
            if_buf_pred_target  <= 32'b0;
        end else begin
            if (do_redirect) if_kill <= if_busy | if_buf_valid;

            if (~if_busy & ~if_buf_valid) begin
                ibus_req_valid     <= 1'b1;
                ibus_req_addr      <= pc;
                if_req_pc          <= pc;
                if_req_pred_taken  <= (bpu_pred_taken & bpu_pred_hit);
                if_req_pred_target <= bpu_pred_target;
            end

            if (ibus_req_valid & ibus_req_ready) begin
                if_busy        <= 1'b1;
                ibus_req_valid <= 1'b0;
            end

            if (ibus_resp_valid) begin
                if (if_kill) begin
                    if_kill   <= 1'b0;
                    if_busy   <= 1'b0;
                end else begin
                    if_buf_valid       <= 1'b1;
                    if_buf_instr       <= ibus_resp_rdata;
                    if_buf_pc_plus4    <= if_req_pc + 32'd4;
                    if_buf_pred_taken  <= if_req_pred_taken;
                    if_buf_pred_target <= if_req_pred_target;
                    if_busy            <= 1'b0;
                end
            end
        end
    end

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // IF/ID write
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    reg mem_busy;

    wire pcWrite_final     = hz_pcWrite     & ~mem_busy;
    wire IF_ID_Write_final = hz_IF_ID_Write & ~mem_busy;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            IF_ID_valid        <= 1'b0;
            IF_ID_instr        <= 32'b0;
            IF_ID_pc           <= 32'b0;
            IF_ID_pred_taken   <= 1'b0;
            IF_ID_pred_target  <= 32'b0;
            if_buf_valid       <= 1'b0;
        end else if (do_redirect) begin
            IF_ID_valid        <= 1'b0;
            IF_ID_instr        <= 32'b0;
            IF_ID_pc           <= 32'b0;
            IF_ID_pred_taken   <= 1'b0;
            IF_ID_pred_target  <= 32'b0;
            if_buf_valid       <= 1'b0;
        end else if (IF_ID_Write_final) begin
            if (if_buf_valid) begin
                IF_ID_valid        <= 1'b1;
                IF_ID_instr        <= if_buf_instr;
                IF_ID_pc           <= if_buf_pc_plus4;
                IF_ID_pred_taken   <= if_buf_pred_taken;
                IF_ID_pred_target  <= if_buf_pred_target;
                if_buf_valid       <= 1'b0;
            end
        end
    end

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // ID/EX write
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ID_EX_valid      <= 1'b0;
            ID_EX_pc         <= 32'b0;
            ID_EX_regData1   <= 32'b0;
            ID_EX_regData2   <= 32'b0;
            ID_EX_immExt     <= 32'b0;
            ID_EX_rs         <= 5'b0;
            ID_EX_rt         <= 5'b0;
            ID_EX_rd         <= 5'b0;
            ID_EX_shamt      <= 5'b0;
            ID_EX_funct      <= 6'b0;
            ID_EX_opcode     <= 6'b0;
            ID_EX_regWrite   <= 1'b0;
            ID_EX_memRead    <= 1'b0;
            ID_EX_memWrite   <= 1'b0;
            ID_EX_memToReg   <= 1'b0;
            ID_EX_branch     <= 1'b0;
            ID_EX_branch_ne  <= 1'b0;
            ID_EX_aluOp      <= 3'b000;
            ID_EX_aluSrc     <= 1'b0;
            ID_EX_regDst     <= 1'b0;
            ID_EX_jal        <= 1'b0;
            ID_EX_memSize    <= MSIZE_W;
            ID_EX_loadSigned <= 1'b1;
        end else if (hz_controlBubble) begin
            ID_EX_valid      <= 1'b0;
            ID_EX_regWrite   <= 1'b0;
            ID_EX_memRead    <= 1'b0;
            ID_EX_memWrite   <= 1'b0;
            ID_EX_memToReg   <= 1'b0;
            ID_EX_branch     <= 1'b0;
            ID_EX_branch_ne  <= 1'b0;
            ID_EX_aluOp      <= 3'b010;
            ID_EX_aluSrc     <= 1'b0;
            ID_EX_regDst     <= 1'b0;
            ID_EX_jal        <= 1'b0;
            ID_EX_memSize    <= MSIZE_W;
            ID_EX_loadSigned <= 1'b1;
        end else if (IF_ID_Write_final) begin
            ID_EX_valid      <= IF_ID_valid;
            ID_EX_pc         <= IF_ID_pc;
            ID_EX_regData1   <= regData1;
            ID_EX_regData2   <= regData2;
            ID_EX_immExt     <= immExt;
            ID_EX_rs         <= rs;
            ID_EX_rt         <= rt;
            ID_EX_rd         <= rd;
            ID_EX_shamt      <= shamt;
            ID_EX_funct      <= funct;
            ID_EX_opcode     <= opcode;
            ID_EX_regWrite   <= regWrite;
            ID_EX_memRead    <= memRead;
            ID_EX_memWrite   <= memWrite;
            ID_EX_memToReg   <= memToReg;
            ID_EX_branch     <= branch;
            ID_EX_branch_ne  <= branch_ne;
            ID_EX_aluOp      <= aluOp;
            ID_EX_aluSrc     <= aluSrc;
            ID_EX_regDst     <= regDst;
            ID_EX_jal        <= jal;
            ID_EX_memSize    <= memSize;
            ID_EX_loadSigned <= loadSigned;
        end
    end

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // EX/MEM write
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            EX_MEM_valid      <= 1'b0;
            EX_MEM_aluResult  <= 32'b0;
            EX_MEM_storeData  <= 32'b0;
            EX_MEM_pc_link    <= 32'b0;
            EX_MEM_regDest    <= 5'b0;
            EX_MEM_regWrite   <= 1'b0;
            EX_MEM_memRead    <= 1'b0;
            EX_MEM_memWrite   <= 1'b0;
            EX_MEM_memToReg   <= 1'b0;
            EX_MEM_jal        <= 1'b0;
            EX_MEM_memSize    <= MSIZE_W;
            EX_MEM_loadSigned <= 1'b1;
        end else begin
            EX_MEM_valid      <= ID_EX_valid & ~trap_EX;
            EX_MEM_aluResult  <= aluOut;
            EX_MEM_storeData  <= storeData_EX;
            EX_MEM_pc_link    <= ID_EX_pc;
            EX_MEM_regDest    <= regDest_EX;
            EX_MEM_regWrite   <= ID_EX_regWrite;
            EX_MEM_memRead    <= ID_EX_memRead  & ~trap_EX;
            EX_MEM_memWrite   <= ID_EX_memWrite & ~trap_EX;
            EX_MEM_memToReg   <= ID_EX_memToReg;
            EX_MEM_jal        <= ID_EX_jal;
            EX_MEM_memSize    <= ID_EX_memSize;
            EX_MEM_loadSigned <= ID_EX_loadSigned;
        end
    end

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // CSR Perf / EPC / CAUSE
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    wire [31:0] csr_rdata, csr_epc_o, csr_cause_o;
    reg         csr_upd_trap;
    reg  [31:0] csr_epc_w;
    reg  [31:0] csr_cause_w;
    reg         csr_retire_pulse;

    csr_perf u_csr (
        .clk(clk),
        .rst_n(rst_n),
        .stall_event(mem_busy | if_busy),
        .flush_event(mispredict_EX),
        .retire_event(csr_retire_pulse),
        .trap_set(csr_upd_trap),
        .epc_w(csr_epc_w),
        .cause_w(csr_cause_w),
        .addr(EX_MEM_aluResult),
        .rdata(csr_rdata),
        .epc_ro(csr_epc_o),
        .cause_ro(csr_cause_o)
    );

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            csr_upd_trap <= 1'b0;
            csr_epc_w    <= 32'b0;
            csr_cause_w  <= 32'b0;
        end else begin
            csr_upd_trap <= trap_EX;
            csr_epc_w    <= instr_pc_EX;
            csr_cause_w  <= trap_cause_EX;
        end
    end

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // D-bus issue/complete
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    assign dbus_resp_ready = 1'b1;

    reg        mem_out_is_load;
    reg        mem_out_is_store;

    wire       csr_hit   = (EX_MEM_aluResult & 32'hFFFF_FF00) == CSR_BASE;
    wire       csr_read  = EX_MEM_memRead  & csr_hit;
    wire       csr_write = EX_MEM_memWrite & csr_hit;

    reg  [3:0] wstrb;
    reg  [31:0] wdata_aligned;
    wire [1:0] addr_low_MEM = EX_MEM_aluResult[1:0];

    always @* begin
        wstrb = 4'b0000;
        wdata_aligned = 32'b0;
        case (EX_MEM_memSize)
            MSIZE_B: begin
                wstrb = 4'b0001 << addr_low_MEM;
                wdata_aligned = EX_MEM_storeData << (8*addr_low_MEM);
            end
            MSIZE_H: begin
                if (addr_low_MEM[0]==1'b0) begin
                    wstrb = addr_low_MEM[1] ? 4'b1100 : 4'b0011;
                    wdata_aligned = EX_MEM_storeData << (16*addr_low_MEM[1]);
                end
            end
            default: begin
                wstrb = 4'b1111;
                wdata_aligned = EX_MEM_storeData;
            end
        endcase
    end

    reg [31:0] load_aligned;

    always @* begin
        case (EX_MEM_memSize)
            MSIZE_B: begin
                case (addr_low_MEM)
                    2'd0: load_aligned = {24'b0, dbus_resp_rdata[7:0]};
                    2'd1: load_aligned = {24'b0, dbus_resp_rdata[15:8]};
                    2'd2: load_aligned = {24'b0, dbus_resp_rdata[23:16]};
                    default: load_aligned = {24'b0, dbus_resp_rdata[31:24]};
                endcase
                if (EX_MEM_loadSigned) load_aligned = {{24{load_aligned[7]}}, load_aligned[7:0]};
            end
            MSIZE_H: begin
                if (addr_low_MEM[1]==1'b0)
                    load_aligned = {16'b0, dbus_resp_rdata[15:0]};
                else
                    load_aligned = {16'b0, dbus_resp_rdata[31:16]};
                if (EX_MEM_loadSigned) load_aligned = {{16{load_aligned[15]}}, load_aligned[15:0]};
            end
            default: load_aligned = dbus_resp_rdata;
        endcase
    end

    wire mem_req_fire  = dbus_req_valid & dbus_req_ready;
    wire mem_resp_fire = dbus_resp_valid & dbus_resp_ready;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            mem_busy        <= 1'b0;
            dbus_req_valid  <= 1'b0;
            dbus_req_addr   <= 32'b0;
            dbus_req_wdata  <= 32'b0;
            dbus_req_wstrb  <= 4'b0000;
            mem_out_is_load <= 1'b0;
            mem_out_is_store<= 1'b0;
        end else begin
            if (!mem_busy) begin
                if ((EX_MEM_memRead | EX_MEM_memWrite) & EX_MEM_valid & ~csr_hit) begin
                    dbus_req_valid <= 1'b1;
                    dbus_req_addr  <= EX_MEM_aluResult;
                    dbus_req_wdata <= wdata_aligned;
                    dbus_req_wstrb <= EX_MEM_memWrite ? wstrb : 4'b0000;
                end else begin
                    dbus_req_valid <= 1'b0;
                    dbus_req_wstrb <= 4'b0000;
                end
            end
            if (mem_req_fire) begin
                mem_busy        <= 1'b1;
                mem_out_is_load <= (dbus_req_wstrb==4'b0000);
                mem_out_is_store<= (dbus_req_wstrb!=4'b0000);
                dbus_req_valid  <= 1'b0;
                dbus_req_wstrb  <= 4'b0000;
            end
            if (mem_resp_fire) begin
                mem_busy <= 1'b0;
            end
        end
    end

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // MEM/WB write
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            MEM_WB_valid    <= 1'b0;
            MEM_WB_aluResult<= 32'b0;
            MEM_WB_memData  <= 32'b0;
            MEM_WB_pc_link  <= 32'b0;
            MEM_WB_regDest  <= 5'b0;
            MEM_WB_regWrite <= 1'b0;
            MEM_WB_memToReg <= 1'b0;
            MEM_WB_jal      <= 1'b0;
        end else begin
            MEM_WB_aluResult <= EX_MEM_aluResult;
            MEM_WB_pc_link   <= EX_MEM_pc_link;
            MEM_WB_regDest   <= EX_MEM_regDest;
            MEM_WB_jal       <= EX_MEM_jal;

            if (csr_read) MEM_WB_memData <= csr_rdata;
            else if (mem_resp_fire & mem_out_is_load) MEM_WB_memData <= load_aligned;

            MEM_WB_regWrite <= EX_MEM_regWrite &
                               ( EX_MEM_jal |
                                 (~EX_MEM_memToReg) |
                                 csr_read |
                                 (mem_resp_fire & mem_out_is_load) );

            MEM_WB_memToReg <= EX_MEM_memToReg & ~EX_MEM_jal;

            MEM_WB_valid <= (EX_MEM_valid & ~EX_MEM_memRead & ~EX_MEM_memWrite) |
                            csr_read |
                            (mem_resp_fire & mem_out_is_load);
        end
    end

    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    // Retire pulse
    //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    always @* begin
        csr_retire_pulse = MEM_WB_valid |
                           (mem_resp_fire & (mem_out_is_load | mem_out_is_store)) |
                           (ID_EX_valid & ID_EX_branch);
    end

endmodule
