`timescale 1ns/1ps

//=================================================
//  TOP-LEVEL CPU MODULE
//=================================================
module cpu_top (
    input  wire         clk,
    input  wire         rst_n
);

    // ================================
    // INTERNAL WIRES & REGS
    // ================================
    
    // Program Counter
    reg  [31:0] pc;
    wire [31:0] pc_next;
    
    // Instruction from Instruction Memory
    wire [31:0] instr;
    
    // Pipeline Registers
    reg  [31:0] IF_ID_instr;
    reg  [31:0] IF_ID_pc;

    // ID/EX pipeline regs
    reg  [31:0] ID_EX_pc;
    reg  [31:0] ID_EX_regData1;
    reg  [31:0] ID_EX_regData2;
    reg  [31:0] ID_EX_signExtImm;
    reg  [4:0]  ID_EX_rs;
    reg  [4:0]  ID_EX_rt;
    reg  [4:0]  ID_EX_rd;
    reg  [4:0]  ID_EX_shamt;
    reg  [5:0]  ID_EX_funct;
    reg  [5:0]  ID_EX_opcode;

    // Control signals in ID/EX stage
    reg         ID_EX_regWrite;
    reg         ID_EX_memRead;
    reg         ID_EX_memWrite;
    reg         ID_EX_memToReg;
    reg         ID_EX_branch;
    reg  [2:0]  ID_EX_aluOp;
    reg         ID_EX_aluSrc;
    reg         ID_EX_regDst;
    reg         ID_EX_jump;

    // EX/MEM pipeline regs
    reg  [31:0] EX_MEM_aluResult;
    reg  [31:0] EX_MEM_regData2;
    reg  [4:0]  EX_MEM_regDest;
    reg         EX_MEM_regWrite;
    reg         EX_MEM_memRead;
    reg         EX_MEM_memWrite;
    reg         EX_MEM_memToReg;
    reg         EX_MEM_branch;
    reg         EX_MEM_jump;
    reg         EX_MEM_zero;
    reg  [31:0] EX_MEM_pcPlusImm;

    // MEM/WB pipeline regs
    reg  [31:0] MEM_WB_aluResult;
    reg  [31:0] MEM_WB_memData;
    reg  [4:0]  MEM_WB_regDest;
    reg         MEM_WB_regWrite;
    reg         MEM_WB_memToReg;

    // Hazard / Forwarding
    wire [1:0]  forwardA, forwardB;
    wire        pcWrite, IF_ID_Write, controlFlush;

    // Control signals from main control
    wire        regDst, aluSrc, memToReg, regWrite;
    wire        memRead, memWrite, branch, jump;
    wire [2:0]  aluOp;

    // ALU wires
    wire [31:0] aluIn1, aluIn2, aluOut;
    wire        aluZero;

    // Data from Register File
    wire [31:0] regData1, regData2;

    // Data from Data Memory
    wire [31:0] memReadData;

    // ================================
    // (1) INSTRUCTION FETCH (IF) STAGE
    // ================================
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) 
            pc <= 32'h00000000;
        else if(pcWrite)
            pc <= pc_next;
    end

    wire [31:0] pc_plus_4 = pc + 4;

    // Instruction memory
    instr_mem instr_mem_inst (
        .addr(pc[9:2]),
        .instr(instr)
    );

    // IF/ID pipeline
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            IF_ID_instr <= 32'b0;
            IF_ID_pc    <= 32'b0;
        end else if(IF_ID_Write) begin
            IF_ID_instr <= instr;
            IF_ID_pc    <= pc_plus_4;
        end
    end

    // ================================
    // (2) INSTRUCTION DECODE (ID) STAGE
    // ================================
    wire [5:0] opcode = IF_ID_instr[31:26];
    wire [4:0] rs     = IF_ID_instr[25:21];
    wire [4:0] rt     = IF_ID_instr[20:16];
    wire [4:0] rd     = IF_ID_instr[15:11];
    wire [4:0] shamt  = IF_ID_instr[10:6];
    wire [5:0] funct  = IF_ID_instr[5:0];
    wire [15:0] imm   = IF_ID_instr[15:0];

    wire [31:0] signExtImm = {{16{imm[15]}}, imm};

    // Register file
    reg_file reg_file_inst (
        .clk   (clk),
        .we    (MEM_WB_regWrite),
        .ra1   (rs),
        .ra2   (rt),
        .wa    (MEM_WB_regDest),
        .wd    ((MEM_WB_memToReg) ? MEM_WB_memData : MEM_WB_aluResult),
        .rd1   (regData1),
        .rd2   (regData2)
    );

    // Main control
    control_unit ctrl_unit_inst (
        .opcode     (opcode),
        .funct      (funct),
        .regDst     (regDst),
        .aluSrc     (aluSrc),
        .memToReg   (memToReg),
        .regWrite   (regWrite),
        .memRead    (memRead),
        .memWrite   (memWrite),
        .branch     (branch),
        .aluOp      (aluOp),
        .jump       (jump)
    );

    // ID/EX pipeline registers
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n || controlFlush) begin
            ID_EX_pc        <= 32'b0;
            ID_EX_regData1  <= 32'b0;
            ID_EX_regData2  <= 32'b0;
            ID_EX_signExtImm<= 32'b0;
            ID_EX_opcode    <= 6'b0;
            ID_EX_funct     <= 6'b0;
            ID_EX_rs        <= 5'b0;
            ID_EX_rt        <= 5'b0;
            ID_EX_rd        <= 5'b0;
            ID_EX_shamt     <= 5'b0;

            ID_EX_regWrite  <= 1'b0;
            ID_EX_memRead   <= 1'b0;
            ID_EX_memWrite  <= 1'b0;
            ID_EX_memToReg  <= 1'b0;
            ID_EX_branch    <= 1'b0;
            ID_EX_jump      <= 1'b0;
            ID_EX_aluOp     <= 3'b000;
            ID_EX_aluSrc    <= 1'b0;
            ID_EX_regDst    <= 1'b0;
        end else begin
            ID_EX_pc        <= IF_ID_pc;
            ID_EX_regData1  <= regData1;
            ID_EX_regData2  <= regData2;
            ID_EX_signExtImm<= signExtImm;
            ID_EX_opcode    <= opcode;
            ID_EX_funct     <= funct;
            ID_EX_rs        <= rs;
            ID_EX_rt        <= rt;
            ID_EX_rd        <= rd;
            ID_EX_shamt     <= shamt;

            // Pass control signals
            ID_EX_regWrite  <= regWrite;
            ID_EX_memRead   <= memRead;
            ID_EX_memWrite  <= memWrite;
            ID_EX_memToReg  <= memToReg;
            ID_EX_branch    <= branch;
            ID_EX_jump      <= jump;
            ID_EX_aluOp     <= aluOp;
            ID_EX_aluSrc    <= aluSrc;
            ID_EX_regDst    <= regDst;
        end
    end

    // ================================
    // (3) EXECUTE (EX) STAGE
    // ================================
    // Forwarding for ALU inputs
    assign aluIn1 = (forwardA == 2'b10) ? EX_MEM_aluResult :
                    (forwardA == 2'b01) ? ((MEM_WB_memToReg) ? MEM_WB_memData : MEM_WB_aluResult) :
                                          ID_EX_regData1;

    assign aluIn2 = (forwardB == 2'b10) ? EX_MEM_aluResult :
                    (forwardB == 2'b01) ? ((MEM_WB_memToReg) ? MEM_WB_memData : MEM_WB_aluResult) :
                                          (ID_EX_aluSrc ? ID_EX_signExtImm : ID_EX_regData2);

    // ALU
    alu alu_inst (
        .in1    (aluIn1),
        .in2    (aluIn2),
        .shamt  (ID_EX_shamt),
        .aluOp  (ID_EX_aluOp),
        .funct  (ID_EX_funct),
        .zero   (aluZero),
        .result (aluOut)
    );

    // Branch Address
    wire [31:0] pcPlusImm = ID_EX_pc + (ID_EX_signExtImm << 2);

    // Determine register destination
    wire [4:0] regDest = (ID_EX_regDst) ? ID_EX_rd : ID_EX_rt;

    // EX/MEM pipeline registers
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            EX_MEM_aluResult <= 32'b0;
            EX_MEM_regData2  <= 32'b0;
            EX_MEM_regDest   <= 5'b0;
            EX_MEM_regWrite  <= 1'b0;
            EX_MEM_memRead   <= 1'b0;
            EX_MEM_memWrite  <= 1'b0;
            EX_MEM_memToReg  <= 1'b0;
            EX_MEM_branch    <= 1'b0;
            EX_MEM_jump      <= 1'b0;
            EX_MEM_zero      <= 1'b0;
            EX_MEM_pcPlusImm <= 32'b0;
        end else begin
            EX_MEM_aluResult <= aluOut;
            EX_MEM_regData2  <= ID_EX_regData2;
            EX_MEM_regDest   <= regDest;
            EX_MEM_regWrite  <= ID_EX_regWrite;
            EX_MEM_memRead   <= ID_EX_memRead;
            EX_MEM_memWrite  <= ID_EX_memWrite;
            EX_MEM_memToReg  <= ID_EX_memToReg;
            EX_MEM_branch    <= ID_EX_branch;
            EX_MEM_jump      <= ID_EX_jump;
            EX_MEM_zero      <= aluZero;
            EX_MEM_pcPlusImm <= pcPlusImm;
        end
    end

    // ================================
    // (4) MEMORY (MEM) STAGE
    // ================================
    // Data memory
    data_mem data_mem_inst (
        .clk   (clk),
        .addr  (EX_MEM_aluResult[9:2]),
        .we    (EX_MEM_memWrite),
        .re    (EX_MEM_memRead),
        .wdata (EX_MEM_regData2),
        .rdata (memReadData)
    );

    // Branch Decision
    wire takeBranch = EX_MEM_branch & EX_MEM_zero;

    // Next PC Logic
    wire [31:0] pc_branch = EX_MEM_pcPlusImm;
    wire [31:0] pc_jump   = {EX_MEM_pcPlusImm[31:28], 28'h000000}; // simplified jump address

    wire [31:0] pc_branch_or_plus4 = (takeBranch) ? pc_branch : (pc + 4);
    assign pc_next = (EX_MEM_jump) ? pc_jump : pc_branch_or_plus4;

    // MEM/WB pipeline
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            MEM_WB_aluResult <= 32'b0;
            MEM_WB_memData   <= 32'b0;
            MEM_WB_regDest   <= 5'b0;
            MEM_WB_regWrite  <= 1'b0;
            MEM_WB_memToReg  <= 1'b0;
        end else begin
            MEM_WB_aluResult <= EX_MEM_aluResult;
            MEM_WB_memData   <= memReadData;
            MEM_WB_regDest   <= EX_MEM_regDest;
            MEM_WB_regWrite  <= EX_MEM_regWrite;
            MEM_WB_memToReg  <= EX_MEM_memToReg;
        end
    end

    // ================================
    // (5) WRITE-BACK (WB) STAGE
    // ================================
    // The actual register file write is inside reg_file, using MEM_WB signals.

    // ================================
    // HAZARD DETECTION & FORWARDING
    // ================================
    hazard_unit hazard_unit_inst (
        .ID_EX_memRead   (ID_EX_memRead),
        .ID_EX_rt        (ID_EX_rt),
        .IF_ID_rs        (rs),
        .IF_ID_rt        (rt),
        .pcWrite         (pcWrite),
        .IF_ID_Write     (IF_ID_Write),
        .controlFlush    (controlFlush)
    );

    forwarding_unit forwarding_unit_inst (
        .EX_MEM_regWrite (EX_MEM_regWrite),
        .EX_MEM_rd       (EX_MEM_regDest),
        .MEM_WB_regWrite (MEM_WB_regWrite),
        .MEM_WB_rd       (MEM_WB_regDest),
        .ID_EX_rs        (ID_EX_rs),
        .ID_EX_rt        (ID_EX_rt),
        .forwardA        (forwardA),
        .forwardB        (forwardB)
    );

endmodule // cpu_top


//=================================================
//  ALU MODULE
//=================================================
module alu (
    input  wire [31:0] in1,
    input  wire [31:0] in2,
    input  wire [4:0]  shamt,
    input  wire [2:0]  aluOp,  
    input  wire [5:0]  funct, 
    output reg         zero,
    output reg  [31:0] result
);
    always @* begin
        result = 32'b0;
        case (aluOp)
            3'b000: begin
                // R-type (use funct)
                case (funct)
                    6'h20: result = in1 + in2;   // ADD
                    6'h22: result = in1 - in2;   // SUB
                    6'h24: result = in1 & in2;   // AND
                    6'h25: result = in1 | in2;   // OR
                    6'h2a: result = (in1 < in2) ? 32'd1 : 32'd0; // SLT
                    default: result = 32'd0;
                endcase
            end

            3'b001: begin
                // BEQ => subtract
                result = in1 - in2;
            end

            3'b010: begin
                // e.g. LW/SW, ADDI => add
                result = in1 + in2;
            end

            default: result = 32'b0;
        endcase

        zero = (result == 32'b0);
    end
endmodule


//=================================================
//  REGISTER FILE
//=================================================
module reg_file (
    input  wire        clk,
    input  wire        we,
    input  wire [4:0]  ra1, ra2, wa,
    input  wire [31:0] wd,
    output wire [31:0] rd1,
    output wire [31:0] rd2
);

    reg [31:0] rf[31:0];
    integer i;

    initial begin
        for (i = 0; i < 32; i = i + 1)
            rf[i] = 32'b0;
    end

    always @(posedge clk) begin
        if (we && wa != 0)
            rf[wa] <= wd;
    end

    assign rd1 = (ra1 == 0) ? 32'b0 : rf[ra1];
    assign rd2 = (ra2 == 0) ? 32'b0 : rf[ra2];

endmodule


//=================================================
//  INSTRUCTION MEMORY (imem)
//=================================================
module instr_mem (
    input  wire [7:0] addr,
    output wire [31:0] instr
);

    reg [31:0] imem[255:0];

    initial begin
        integer j;
        for(j=0; j<256; j=j+1) imem[j] = 32'h00000000;

        // Example instructions (MIPS-like):
        // addi $1, $0, 5   => 0x20010005
        // addi $2, $0, 10  => 0x2002000A
        // add  $3, $1, $2  => 0x00221820
        // sw   $3, 0($0)   => 0xAC030000
        // addi $1, $1, 1   => 0x20210001
        imem[0] = 32'h20010005; // addi $1, $0, 5
        imem[1] = 32'h2002000A; // addi $2, $0, 10
        imem[2] = 32'h00221820; // add  $3, $1, $2
        imem[3] = 32'hAC030000; // sw   $3, 0($0)
        imem[4] = 32'h20210001; // addi $1, $1, 1
    end

    assign instr = imem[addr];
endmodule


//=================================================
//  DATA MEMORY (Stub)
//=================================================
module data_mem (
    input  wire        clk,
    input  wire [7:0]  addr,
    input  wire        we,
    input  wire        re,
    input  wire [31:0] wdata,
    output wire [31:0] rdata
);

    reg [31:0] dmem[255:0];
    integer k;

    initial begin
        for(k=0; k<256; k=k+1) dmem[k] = 32'h00000000;
    end

    always @(posedge clk) begin
        if (we)
            dmem[addr] <= wdata;
    end

    assign rdata = (re) ? dmem[addr] : 32'h00000000;

endmodule


//=================================================
//  CONTROL UNIT (Simplified MIPS-like)
//=================================================
module control_unit (
    input  wire [5:0] opcode,
    input  wire [5:0] funct,
    output reg        regDst,
    output reg        aluSrc,
    output reg        memToReg,
    output reg        regWrite,
    output reg        memRead,
    output reg        memWrite,
    output reg        branch,
    output reg [2:0]  aluOp,
    output reg        jump
);

    localparam R_TYPE = 6'b000000;
    localparam LW     = 6'b100011;
    localparam SW     = 6'b101011;
    localparam BEQ    = 6'b000100;
    localparam JUMP   = 6'b000010;
    localparam ADDI   = 6'b001000;

    always @* begin
        regDst   = 0;
        aluSrc   = 0;
        memToReg = 0;
        regWrite = 0;
        memRead  = 0;
        memWrite = 0;
        branch   = 0;
        jump     = 0;
        aluOp    = 3'b000; // default R-type usage

        case (opcode)
            R_TYPE: begin
                regDst   = 1;
                regWrite = 1;
                aluOp    = 3'b000; // use funct
            end
            LW: begin
                aluSrc   = 1;
                memToReg = 1;
                regWrite = 1;
                memRead  = 1;
                aluOp    = 3'b010; // add
            end
            SW: begin
                aluSrc   = 1;
                memWrite = 1;
                aluOp    = 3'b010; // add
            end
            BEQ: begin
                branch   = 1;
                aluOp    = 3'b001; // sub
            end
            JUMP: begin
                jump     = 1;
            end
            ADDI: begin
                aluSrc   = 1;
                regWrite = 1;
                aluOp    = 3'b010; // add
            end
            default: ;
        endcase
    end
endmodule


//=================================================
//  HAZARD UNIT
//=================================================
module hazard_unit (
    input wire ID_EX_memRead,
    input wire [4:0] ID_EX_rt,
    input wire [4:0] IF_ID_rs,
    input wire [4:0] IF_ID_rt,
    
    output reg pcWrite,
    output reg IF_ID_Write,
    output reg controlFlush
);

    always @* begin
        pcWrite      = 1'b1;
        IF_ID_Write  = 1'b1;
        controlFlush = 1'b0;

        // Simple load-use hazard
        if(ID_EX_memRead && ((ID_EX_rt == IF_ID_rs) || (ID_EX_rt == IF_ID_rt))) begin
            pcWrite      = 1'b0;
            IF_ID_Write  = 1'b0;
            controlFlush = 1'b1; // flush ID
        end
    end
endmodule


//=================================================
//  FORWARDING UNIT
//=================================================
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

        // EX hazard
        if(EX_MEM_regWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs)) begin
            forwardA = 2'b10;
        end
        if(EX_MEM_regWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rt)) begin
            forwardB = 2'b10;
        end

        // MEM hazard
        if(MEM_WB_regWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rs)) begin
            forwardA = 2'b01;
        end
        if(MEM_WB_regWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rt)) begin
            forwardB = 2'b01;
        end
    end
endmodule


//=================================================
//  SIMPLE TESTBENCH
//=================================================
module cpu_tb;

    reg clk;
    reg rst_n;

    // Instantiate the CPU
    cpu_top dut (
        .clk   (clk),
        .rst_n (rst_n)
    );

    // Generate clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 10ns period => 100MHz
    end

    // Reset and run
    initial begin
        rst_n = 0;
        #20;
        rst_n = 1;

        // Let it run for more cycles to ensure we see pipeline
        #1000;
        $finish;
    end

    // Dump waves for GTKWave
    initial begin
        $dumpfile("cpu_wave.vcd");
        $dumpvars(0, cpu_tb);
    end

    // Debug print each cycle
    always @(posedge clk) begin
        $display(
          "Time=%0t ns | pc=%h instr=%h | R1=%d R2=%d R3=%d | pcWrite=%b IFIDW=%b",
          $time, 
          dut.pc, 
          dut.IF_ID_instr,
          dut.reg_file_inst.rf[1], // Register $1
          dut.reg_file_inst.rf[2], // Register $2
          dut.reg_file_inst.rf[3], // Register $3
          dut.hazard_unit_inst.pcWrite,
          dut.hazard_unit_inst.IF_ID_Write
        );
    end

endmodule
