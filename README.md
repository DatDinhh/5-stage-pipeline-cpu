# 5-Stage Pipelined CPU (Verilog)

A classic 5-stage pipelined CPU written in Verilog/SystemVerilog with full hazard handling, basic branch prediction, performance counters, and a self-checking simulation environment.

##  Project Structure

```
rtl/core/         # CPU RTL design
  alu.v                 - Arithmetic Logic Unit
  bpu.v                 - Branch Prediction Unit
  control_unit.v        - Main control logic
  cpu_core.v            - Top-level CPU core (pipeline stages + control)
  csr_perf.v            - Performance counters (cycle, instret, stalls)
  forwarding_unit.v     - Data forwarding logic
  hazard_unit.v         - Hazard detection & stall control
  reg_file.v            - 32x32 Register File

sim/mem/          # Memory models for simulation
  rv_mem.v              - Data memory model (configurable wait states)
  rv_rom.v              - Instruction ROM model (configurable wait states)

sim/score/        # Verification helpers
  scoreboard.v          - Memory-mirroring scoreboard to check load/store correctness

sim/assertions/   # Optional SystemVerilog assertions
  assertions.sv         - Checks for hazards, flushes, and protocol correctness

sim/tb/           # Testbenches
  tb_core.sv             - Top-level CPU + memory + scoreboard testbench

sim/waves/        # Waveform outputs
  core.vcd               - Generated VCD file from simulation
```

##  Features

- **5 Pipeline Stages:** IF, ID, EX, MEM, WB
- **Hazard Handling:** Load-use stall detection, branch flushes
- **Forwarding Unit:** Resolves RAW data hazards without stalling
- **Branch Prediction:** Simple predictor with branch target tracking
- **Performance Counters:** Cycle count, instructions retired, stalls
- **Self-Checking Scoreboard:** Verifies load/store correctness at commit
- **Optional Assertions:** For hazards, flushes, bus handshake correctness

##  Prerequisites

- [Icarus Verilog](http://iverilog.icarus.com/) (iverilog / vvp)
- [GTKWave](http://gtkwave.sourceforge.net/)
- PowerShell (for provided run scripts)
- (Optional) Python 3 — if generating `.hex` ROM images from assembly

##  How to Build & Run

1. **Clone this repo:**
   ```powershell
   git clone https://github.com/DatDinhh/5_stage_pipeline_cpu.git
   cd 5_pipeline_cpu
   ```

2. **Compile and simulate:**
   ```powershell
   $proj = "C:\5_pipeline_cpu"
   $gtkw = "C:\iverilog\gtkwave\bin\gtkwave.exe"

   Remove-Item -Recurse -Force "$proj\build","$proj\sim\waves" -ErrorAction SilentlyContinue
   New-Item -ItemType Directory -Force -Path "$proj\build","$proj\sim\waves" | Out-Null

   $srcs = @(
     "$proj\rtl\core\alu.v",
     "$proj\rtl\core\bpu.v",
     "$proj\rtl\core\control_unit.v",
     "$proj\rtl\core\cpu_core.v",
     "$proj\rtl\core\csr_perf.v",
     "$proj\rtl\core\forwarding_unit.v",
     "$proj\rtl\core\hazard_unit.v",
     "$proj\rtl\core\reg_file.v",
     "$proj\sim\mem\rv_mem.v",
     "$proj\sim\mem\rv_rom.v",
     "$proj\sim\score\scoreboard.v",
     "$proj\sim\tb\tb_core.sv"
   ) | Where-Object { Test-Path $_ }

   iverilog -g2012 -s tb_core -o "$proj\build\sim.out" $srcs
   Push-Location $proj; vvp ".\build\sim.out"; Pop-Location
   ```

3. **Open waveform in GTKWave:**
   ```powershell
   $latest = Get-ChildItem -Path "$proj\sim\waves" -Filter *.vcd | Sort-Object LastWriteTime -Descending | Select-Object -First 1
   if ($latest) { & $gtkw $latest.FullName }
   ```

##  Customization

- **Program ROM:**  
  Edit `rv_rom.v` and point `$readmemh` to your program `.hex` file.
- **Memory Wait States:**  
  Change `RANDOM_WAIT` and `WAIT_MAX` parameters in `rv_mem.v` / `rv_rom.v`.
- **Assertions:**  
  Include `sim/assertions/assertions.sv` and compile with `+define+SVA` if your simulator supports SVAs.

##  License
MIT License — see `LICENSE` for details.
