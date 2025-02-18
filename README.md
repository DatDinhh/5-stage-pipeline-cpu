# 5-stage-pipeline-cpu
A 5-stage pipelined CPU in Verilog with hazard detection, forwarding, branching, and a simple instruction memory.

## 1 Overview 

This repository contains a five-stage pipelined CPU implemented in Verilog. It demonstrates the fundamental concepts of pipelining, hazard detection, and forwarding for a MIPS-like instruction set. The design includes:

Instruction Fetch (IF)
Instruction Decode (ID)
Execute (EX)
Memory (MEM)
Write Back (WB)

  Hazard detection logic handles load-use hazards, and a forwarding unit resolves data hazards. Simple branch logic is included for control hazards, and a basic instruction memory supplies instructions.

## 2 Features

5-stage pipeline (IF, ID, EX, MEM, WB)
Hazard detection for load-use stalls
Forwarding/bypassing for data hazards
Simple branch and jump logic
Instruction and data memories modeled in Verilog
Example programs (loops, branching) loaded into instruction memory

## 3 File/Module Structure

Briefly describe each file or module:

cpu_top.v: Top-level module with pipeline registers
alu.v: ALU for arithmetic/logic operations
reg_file.v: 32x32 register file
instr_mem.v: Hard-coded instruction memory with example programs
data_mem.v: Simple data memory
control_unit.v: Decodes instructions into control signals
hazard_unit.v: Detects load-use hazards
forwarding_unit.v: Resolves data hazards by forwarding
cpu_tb.v: Testbench for simulation
