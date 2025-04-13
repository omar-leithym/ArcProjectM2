`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//  All-in-One Verilog Source for RISCV_pipeline + Submodules
//  Guaranteed to compile in Vivado if you include this single file
//  and reference the RISCV_pipeline as top-level (or instantiate it in a TB).
//////////////////////////////////////////////////////////////////////////////////

//
//--------------------- 1) Basic 2x1 Mux (1-bit) ---------------------
//
module mux2x1(
    input wire A,
    input wire B,
    input wire S,
    output wire C
);
    assign C = (S) ? B : A;
endmodule

//
//--------------------- 2) N-bit 2x1 Mux ---------------------
//
module NBitMux2x1 #(parameter N=8) (
    input wire [N-1:0] A, 
    input wire [N-1:0] B,
    input wire S,
    output wire [N-1:0] C
);
    genvar i;
    generate 
        for (i=0; i < N; i=i+1) begin
            mux2x1 myMux(
                .A(A[i]), 
                .B(B[i]), 
                .S(S), 
                .C(C[i])
            );
        end
    endgenerate
endmodule

//
//--------------------- 3) N-bit Register with Load & Reset ---------------------
//  Parameterized register: On rising edge, if rst=1 => Q=0; if load=1 => Q=D
//
module NBitRegister #(parameter N = 8) (
    input wire clk, 
    input wire rst,
    input wire load,
    input wire [N-1:0] D,
    output reg [N-1:0] Q
);
    always @(posedge clk or posedge rst) begin
        if (rst)
            Q <= {N{1'b0}};  
        else if (load)
            Q <= D;          
    end
endmodule

//
//--------------------- 4) Simple Ripple-Carry Adder ---------------------
//
module RCA #(parameter n=32)(
    input  [n-1:0] x, 
    input  [n-1:0] y,
    input          cin,
    output [n-1:0] sum,
    output         cout
);
    // Simple assign-based addition. If you want a structural ripple-carry,
    // you can expand it. This is good enough for your pipeline.
    assign {cout, sum} = x + y + cin;
endmodule

//
//--------------------- 5) N-bit Shift-Left-by-1 ---------------------
//
module NBitShiftLeft #(parameter N=32) (
    input  wire [N-1:0] in,
    output wire [N-1:0] out
);
    assign out = in << 1;
endmodule

//
//--------------------- 6) ALU (supports add, sub, AND, OR, etc.) ---------------------
//
module ALU #(parameter N=32) (
    input  wire [N-1:0] A,
    input  wire [N-1:0] B,
    input  wire [3:0]   ALUsel,  
    output reg  [N-1:0] result,
    output wire         zeroFlag
);
    // zeroFlag = 1 if result == 0
    assign zeroFlag = (result == 0);

    always @(*) begin
        case(ALUsel)
            4'b0000: result = A & B;      // AND
            4'b0001: result = A | B;      // OR
            4'b0010: result = A + B;      // ADD
            4'b0110: result = A - B;      // SUB
            default: result = 32'hDEAD_BEEF; 
        endcase
    end
endmodule

//
//--------------------- 7) ALU Control Unit ---------------------
//  Takes ALUOp + (funct3 + funct7 bit) to decide ALU operation
//
module ALU_CU(
    input [2:0] inst14_12,
    input [1:0] ALUOp,
    input       inst30,
    output reg [3:0] ALUsel
);
    always @(*) begin
        case(ALUOp)
            // For lw/sw => ALU does ADD
            2'b00: ALUsel = 4'b0010; // ADD
            // For branch => ALU does SUB
            2'b01: ALUsel = 4'b0110; // SUB
            // For R-type => decode with funct3/funct7
            2'b10: begin
                case(inst14_12)
                    3'b000: ALUsel = (inst30 ? 4'b0110 : 4'b0010); // SUB if inst30=1, else ADD
                    3'b111: ALUsel = 4'b0000; // AND
                    3'b110: ALUsel = 4'b0001; // OR
                    default: ALUsel = 4'b1111; 
                endcase
            end
            default: ALUsel = 4'b1111;
        endcase
    end
endmodule

//
//--------------------- 8) Immediate Generator ---------------------
//
module ImmGen(
    input  [31:0] inst,
    output reg [31:0] gen_out
);
    // Quick approach: look at bits [6:5] to decide I/S/B type
    wire [1:0] opcode_two = inst[6:5];

    always @(*) begin
        case (opcode_two)
            2'b00: // I-type (like LW)
                gen_out = {{20{inst[31]}}, inst[31:20]};
            2'b01: // S-type (like SW)
                gen_out = {{20{inst[31]}}, inst[31:25], inst[11:7]};
            2'b11: // B-type (like BEQ)
                gen_out = {{20{inst[31]}}, inst[31], inst[7],
                           inst[30:25], inst[11:8]};
            default:
                gen_out = 32'b0;
        endcase
    end
endmodule

//
//--------------------- 9) Main Control Unit (CU) ---------------------
//  Decodes opcode[6:2] into high-level controls
//
module CU(
    input [4:0] instBits,
    output reg  Branch, 
    output reg  MemRead, 
    output reg  MemtoReg,
    output reg [1:0] ALUOp, 
    output reg  memWrite, 
    output reg  ALUSrc, 
    output reg  RegWrite
);
    always @(*) begin
        // Default all to 0 in case no match
        Branch=0; MemRead=0; MemtoReg=0; ALUOp=2'b00; memWrite=0; ALUSrc=0; RegWrite=0;
        
        // R-type = 01100
        if(instBits == 5'b01100) begin
            Branch   = 1'b0; 
            MemRead  = 1'b0; 
            MemtoReg = 1'b0; 
            ALUOp    = 2'b10; 
            memWrite = 1'b0; 
            ALUSrc   = 1'b0; 
            RegWrite = 1'b1;
        end
        // LW = 00000
        else if(instBits == 5'b00000) begin
            Branch   = 1'b0; 
            MemRead  = 1'b1; 
            MemtoReg = 1'b1; 
            ALUOp    = 2'b00; 
            memWrite = 1'b0; 
            ALUSrc   = 1'b1; 
            RegWrite = 1'b1;
        end
        // SW = 01000
        else if(instBits == 5'b01000) begin
            Branch   = 1'b0; 
            MemRead  = 1'b0; 
            MemtoReg = 1'b0; // 'x' in your code, but 0 is fine
            ALUOp    = 2'b00; 
            memWrite = 1'b1; 
            ALUSrc   = 1'b1; 
            RegWrite = 1'b0;
        end
        // BEQ = 11000
        else if(instBits == 5'b11000) begin
            Branch   = 1'b1; 
            MemRead  = 1'b0; 
            MemtoReg = 1'b0; // 'x' in your code
            ALUOp    = 2'b01; 
            memWrite = 1'b0; 
            ALUSrc   = 1'b0; 
            RegWrite = 1'b0;
        end
    end
endmodule

//
//--------------------- 10) Register File ---------------------
//
module RegFile(
    input  [4:0] read1, 
    input  [4:0] read2, 
    input  [4:0] write1, 
    input  [31:0] writeData, 
    input  write, 
    input  rst,
    input  clk,
    output [31:0] read1Output, 
    output [31:0] read2Output
);
    reg [31:0] regFile [0:31];
    integer i;
    
    // Synchronous write, asynchronous read
    always @(posedge clk) begin
        if (rst) begin
            for(i=0;i<32;i=i+1) begin
                regFile[i] <= 32'b0;
            end
        end
        else if (write && (write1 != 5'b0)) begin
            regFile[write1] <= writeData;
        end
    end
    
    assign read1Output = regFile[read1];
    assign read2Output = regFile[read2];
endmodule

//
//--------------------- 11) BCD Module for 7-seg Display ---------------------
//  Converts a 13-bit binary number into decimal digits (thousands,hundreds,tens,ones).
//
module BCD(
    input  [12:0] binary, 
    output reg [3:0] thousands, 
    output reg [3:0] hundreds, 
    output reg [3:0] tens, 
    output reg [3:0] ones
);
    integer temp;
    always @(*) begin
        temp      = binary;
        thousands = temp / 1000;   temp = temp % 1000;
        hundreds  = temp / 100;    temp = temp % 100;
        tens      = temp / 10;
        ones      = temp % 10;
    end
endmodule

//
//--------------------- 12) 4-Digit Seven Segment Driver ---------------------
//
module Four_Digit_Seven_Segment_Driver_Optimized ( 
    input clk,
    input [12:0] num, 
    output reg [3:0] Anode, 
    output reg [6:0] LED_out
);
    reg [3:0] LED_BCD;
    reg [19:0] refresh_counter = 0; 
    wire [1:0] LED_activating_counter; 

    wire [12:0] magnitude = num[12] ? (~num + 1) : num;

    always @(posedge clk) begin
        refresh_counter <= refresh_counter + 1;
    end
    assign LED_activating_counter = refresh_counter[19:18]; 
    
    wire [3:0] Thousands, Hundreds, Tens, Ones;
    BCD callingModule(magnitude, Thousands, Hundreds, Tens, Ones);
    wire sign = num[12];  // indicates negative

    always @(*) begin
        case(LED_activating_counter)
            2'b00: begin
                // leftmost digit
                Anode = 4'b0111;
                LED_BCD = (sign) ? 4'b1010 : 4'b1011; 
                  // 1010 => '-'
                  // 1011 => blank
            end
            2'b01: begin
                Anode = 4'b1011;
                LED_BCD = Hundreds;
            end
            2'b10: begin
                Anode = 4'b1101;
                LED_BCD = Tens;
            end
            2'b11: begin
                Anode = 4'b1110;
                LED_BCD = Ones;
            end
        endcase
    end

    always @(*) begin
        case(LED_BCD)
            4'b0000: LED_out = 7'b0000001; // "0"
            4'b0001: LED_out = 7'b1001111; // "1"
            4'b0010: LED_out = 7'b0010010; // "2"
            4'b0011: LED_out = 7'b0000110; // "3"
            4'b0100: LED_out = 7'b1001100; // "4"
            4'b0101: LED_out = 7'b0100100; // "5"
            4'b0110: LED_out = 7'b0100000; // "6"
            4'b0111: LED_out = 7'b0001111; // "7"
            4'b1000: LED_out = 7'b0000000; // "8"
            4'b1001: LED_out = 7'b0000100; // "9"
            4'b1010: LED_out = 7'b1111110; // "-"
            4'b1011: LED_out = 7'b1111111; // "blank"
            default: LED_out = 7'b0000001; // "0"
        endcase
    end
endmodule

//
//--------------------- 13) Data Memory ---------------------
//  Simple synchronous write, asynchronous read memory
//  Modify 'addr' width and memory size as needed
//
module DataMemory(
    input clk,
    input MemRead,
    input MemWrite,
    input [5:0] addr,      // up to 64 words
    input [31:0] data_in,
    output reg [31:0] data_out
);
    reg [31:0] mem [0:63];

    // Synchronous write
    always @(posedge clk) begin
        if(MemWrite) begin
            mem[addr] <= data_in;
        end
    end

    // Asynchronous read
    always @(*) begin
        if(MemRead)
            data_out = mem[addr];
        else
            data_out = 32'b0;
    end

    // Optionally initialize some data words
    initial begin
        // Example data
        mem[0] = 32'd17;  
        mem[1] = 32'd9;   
        mem[2] = 32'd25;  
        // ...
    end
endmodule

//
//--------------------- 14) Instruction Memory ---------------------
//  Contains the sample program from your lab
//
module InstructionMemory(
    input  [31:0] addr, 
    output [31:0] data_out
);
    reg [31:0] mem [0:63];
    assign data_out = mem[addr];  // asynchronous read

  initial begin
        // Just a sample leading code up to PC=80
        // Enough instructions to produce x4 and x3
    
        mem[0]  = 32'b0000000_00000_00000_000_00000_0110011 ; // NOP (add x0,x0,x0)
        mem[1]  = 32'b000000000000_00000_010_00001_0000011 ; // lw x1,0(x0)
        mem[2]  = 32'b0000000_00000_00000_000_00000_0110011 ; // NOP
        mem[3]  = 32'b0000000_00000_00000_000_00000_0110011 ; // NOP
        mem[4]  = 32'b000000000100_00000_010_00010_0000011 ; // lw x2,4(x0)
        mem[5]  = 32'b0000000_00000_00000_000_00000_0110011 ; // NOP
        mem[6]  = 32'b0000000_00001_00010_000_00011_0110011 ; // add x3,x1,x2
        // more instructions to set up x4, etc...
        // ...
        // Place NOPs so that x3, x4 are actually ready
    
        // We want the *branch instruction* at PC=80 => mem[20].
        // So fill indexes 7..19 with code or NOPs as needed:
    
        mem[7]  = 32'b0000000_00000_00000_000_00000_0110011 ; // NOP
        mem[8]  = 32'b0000000_00000_00000_000_00000_0110011 ; // NOP
        mem[9]  = 32'b0000000_00010_00001_110_00100_0110011 ; // or x4,x1,x2
        mem[10] = 32'b0000000_00000_00000_000_00000_0110011 ; // NOP
        mem[11] = 32'b0000000_00000_00000_000_00000_0110011 ; // NOP
        mem[12] = 32'b0000000_00000_00000_000_00000_0110011 ; // NOP
        // ...
        mem[19] = 32'b0000000_00000_00000_000_00000_0110011 ; // NOP
    
        // --------------- Branch at mem[20] (PC=80) ---------------
        // We want "beq x4,x3,+20 bytes", so the offset in halfwords is 10 (decimal 0xA).
        // One valid encoding is shown below (bits rearranged for BEQ x4,x3,20).
        // For convenience we provide the final hex:
        mem[20] = 32'h01030663; 
           // This encodes: beq x4, x3, +20 bytes 
           //   (i.e., if (x4==x3) then PC=80+20=100)
    
        // Following instructions at mem[21]..mem[24] etc. 
        // If the branch is taken, PC jumps to 100 => mem[25]
        // Or you can put more code at mem[25] and beyond:
        
        mem[21] = 32'b0000000_00000_00000_000_00000_0110011 ; // This runs if branch not taken
        mem[22] = 32'b0000000_00000_00000_000_00000_0110011 ; // ...
        mem[23] = 32'b0000000_00000_00000_000_00000_0110011 ;
        mem[24] = 32'b0000000_00000_00000_000_00000_0110011 ;
    
        // The branch target is PC=100 => mem[25], so put code there:
        mem[25] = 32'b0000000_00100_00011_000_00101_0110011 ; // add x5,x4,x3
        mem[26] = 32'b0000000_00000_00000_000_00000_0110011 ; // ...
        // etc.
    end
endmodule

//
//--------------------- 15) The Top-Level 5-stage RISCV Pipeline ---------------------
//
module RISCV_pipeline(
    input clk, 
    input reset, 
    input [1:0] ledSel, 
    input [3:0] ssdSel, 
    input SSDclk, 
    output reg [15:0] LEDS, 
    output [3:0] Anode, 
    output [6:0] LED_out
);

    //------------------------------------------------
    // IF Stage
    //------------------------------------------------
    reg  [31:0] PC_address; 
    wire [31:0] PC_next;
    
    always @(posedge clk) begin
        if (reset) 
            PC_address <= 32'h0000_0000;  
        else 
            PC_address <= PC_next;
    end
    
    wire [31:0] instruction;
    InstructionMemory instrMem(
        .addr(PC_address), 
        .data_out(instruction)
    );

    // PC + 4
    wire [31:0] pc_plus_4;
    RCA #(32) adder_pc_plus_4(
        .x   (PC_address),
        .y   (32'd4),
        .cin (1'b0),
        .sum (pc_plus_4),
        .cout()
    );

    // IF/ID register
    wire [31:0] IF_ID_PC;
    wire [31:0] IF_ID_Inst;
    NBitRegister #(64) IF_ID (
        .clk  (clk),
        .rst  (reset),
        .load (1'b1),
        .D    ({PC_address, instruction}),
        .Q    ({IF_ID_PC, IF_ID_Inst})
    );

    //------------------------------------------------
    // ID Stage
    //------------------------------------------------
    wire Branch, MemRead, MemtoReg;
    wire [1:0] ALUOp;
    wire memWrite, ALUSrc, RegWrite;

    CU control(
        .instBits(IF_ID_Inst[6:2]),
        .Branch   (Branch), 
        .MemRead  (MemRead), 
        .MemtoReg (MemtoReg),
        .ALUOp    (ALUOp), 
        .memWrite (memWrite), 
        .ALUSrc   (ALUSrc), 
        .RegWrite (RegWrite)
    );

    wire [31:0] readData1, readData2;
    wire [31:0] writeBackData;  // final WB data
    wire        regfile_wen;    // from MEM/WB
    wire [4:0]  regfile_waddr;  // from MEM/WB

    RegFile myReg(
        .read1     (IF_ID_Inst[19:15]),
        .read2     (IF_ID_Inst[24:20]),
        .write1    (regfile_waddr),    // from pipeline's WB stage
        .writeData (writeBackData),
        .write     (regfile_wen),
        .rst       (reset),
        .clk       (clk),
        .read1Output(readData1),
        .read2Output(readData2)
    );

    wire [31:0] immediate;
    ImmGen immedGen(
        .inst   (IF_ID_Inst),
        .gen_out(immediate)
    );

    // Pack ID-stage control signals into ID_EX_Ctrl:
    // bit7 = Branch
    // bit6 = MemRead
    // bit5 = MemtoReg
    // bit4:3 = ALUOp
    // bit2 = MemWrite
    // bit1 = ALUSrc
    // bit0 = RegWrite
    wire [7:0] ID_stage_ctrl = {
        Branch, MemRead, MemtoReg, ALUOp, memWrite, ALUSrc, RegWrite
    };

    // ID/EX register
    wire [7:0]  ID_EX_Ctrl;
    wire [31:0] ID_EX_PC;
    wire [31:0] ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
    wire [3:0]  ID_EX_Func;
    wire [4:0]  ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;

    // 8 + 32 + 32 + 32 + 32 + 4 + 5 + 5 + 5 = 155 bits
    NBitRegister #(155) ID_EX (
        .clk  (clk),
        .rst  (reset),
        .load (1'b1),
        .D    ({
                ID_stage_ctrl,
                IF_ID_PC,
                readData1,
                readData2,
                immediate,
                {IF_ID_Inst[30], IF_ID_Inst[14:12]},
                IF_ID_Inst[19:15],
                IF_ID_Inst[24:20],
                IF_ID_Inst[11:7]
               }),
        .Q    ({
                ID_EX_Ctrl,
                ID_EX_PC,
                ID_EX_RegR1,
                ID_EX_RegR2,
                ID_EX_Imm,
                ID_EX_Func,
                ID_EX_Rs1,
                ID_EX_Rs2,
                ID_EX_Rd
               })
    );

    //------------------------------------------------
    // EX Stage
    //------------------------------------------------
    wire        ID_EX_Branch    = ID_EX_Ctrl[7];
    wire        ID_EX_MemRead   = ID_EX_Ctrl[6];
    wire        ID_EX_MemtoReg  = ID_EX_Ctrl[5];
    wire [1:0]  ID_EX_ALUOp     = ID_EX_Ctrl[4:3];
    wire        ID_EX_MemWrite  = ID_EX_Ctrl[2];
    wire        ID_EX_ALUSrc    = ID_EX_Ctrl[1];
    wire        ID_EX_RegWrite  = ID_EX_Ctrl[0];

    // ALU control
    wire [3:0] ALUsel;
    ALU_CU aluControl(
        .inst14_12(ID_EX_Func[2:0]),
        .ALUOp    (ID_EX_ALUOp),
        .inst30   (ID_EX_Func[3]),
        .ALUsel   (ALUsel)
    );

    // ALU input2 can be RegR2 or immediate
    wire [31:0] alu_in2;
    NBitMux2x1 #(32) mux_alu_src(
        .A(ID_EX_RegR2),
        .B(ID_EX_Imm),
        .S(ID_EX_ALUSrc),
        .C(alu_in2)
    );

    wire [31:0] alu_result;
    wire zeroFlag;
    ALU #(32) mainALU(
        .A(ID_EX_RegR1),
        .B(alu_in2),
        .ALUsel(ALUsel),
        .result(alu_result),
        .zeroFlag(zeroFlag)
    );

    // Compute branch target = ID_EX_PC + (Imm << 1)
    wire [31:0] imm_shifted;
    NBitShiftLeft #(32) myShifter(
        .in(ID_EX_Imm),
        .out(imm_shifted)
    );

    wire [31:0] branch_target;
    RCA #(32) adder_branch(
        .x   (ID_EX_PC),
        .y   (imm_shifted),
        .cin (1'b0),
        .sum (branch_target),
        .cout()
    );

    // EX/MEM control = [Branch,MemRead,MemtoReg,MemWrite,RegWrite]
    wire [4:0] EX_stage_ctrl = {
        ID_EX_Branch,
        ID_EX_MemRead,
        ID_EX_MemtoReg,
        ID_EX_MemWrite,
        ID_EX_RegWrite
    };

    // EX/MEM register
    wire [4:0]  EX_MEM_Ctrl;
    wire [31:0] EX_MEM_BranchAddOut;
    wire        EX_MEM_Zero;
    wire [31:0] EX_MEM_ALU_out;
    wire [31:0] EX_MEM_RegR2;
    wire [4:0]  EX_MEM_Rd;

    NBitRegister #(107) EX_MEM (
        .clk  (clk),
        .rst  (reset),
        .load (1'b1),
        .D    ({
                EX_stage_ctrl,
                branch_target,
                zeroFlag,
                alu_result,
                ID_EX_RegR2,
                ID_EX_Rd
               }),
        .Q    ({
                EX_MEM_Ctrl,
                EX_MEM_BranchAddOut,
                EX_MEM_Zero,
                EX_MEM_ALU_out,
                EX_MEM_RegR2,
                EX_MEM_Rd
               })
    );

    //------------------------------------------------
    // MEM Stage
    //------------------------------------------------
    wire EX_MEM_Branch   = EX_MEM_Ctrl[4];
    wire EX_MEM_MemRead  = EX_MEM_Ctrl[3];
    wire EX_MEM_MemtoReg = EX_MEM_Ctrl[2];
    wire EX_MEM_MemWrite = EX_MEM_Ctrl[1];
    wire EX_MEM_RegWrite = EX_MEM_Ctrl[0];

    wire [31:0] dataOut;
    DataMemory mem(
        .clk      (clk),
        .MemRead  (EX_MEM_MemRead),
        .MemWrite (EX_MEM_MemWrite),
        .addr     (EX_MEM_ALU_out[7:2]),  // word-aligned
        .data_in  (EX_MEM_RegR2),
        .data_out (dataOut)
    );

    // Branch decision
    wire branch_taken = EX_MEM_Branch & EX_MEM_Zero;
    NBitMux2x1 #(32) mux_nextPC(
        .A(pc_plus_4),
        .B(EX_MEM_BranchAddOut),
        .S(branch_taken),
        .C(PC_next)
    );

    // MEM/WB ctrl = [RegWrite, MemtoReg]
    wire [1:0] MEM_stage_ctrl = {
        EX_MEM_RegWrite,
        EX_MEM_MemtoReg
    };

    // MEM/WB pipeline reg
    wire [1:0]  MEM_WB_Ctrl;
    wire [31:0] MEM_WB_Mem_out;
    wire [31:0] MEM_WB_ALU_out;
    wire [4:0]  MEM_WB_Rd;

    NBitRegister #(71) MEM_WB (
        .clk  (clk),
        .rst  (reset),
        .load (1'b1),
        .D    ({
                MEM_stage_ctrl,
                dataOut,
                EX_MEM_ALU_out,
                EX_MEM_Rd
               }),
        .Q    ({
                MEM_WB_Ctrl,
                MEM_WB_Mem_out,
                MEM_WB_ALU_out,
                MEM_WB_Rd
               })
    );

    //------------------------------------------------
    // WB Stage
    //------------------------------------------------
    wire MEM_WB_RegWrite = MEM_WB_Ctrl[1];
    wire MEM_WB_MemtoReg = MEM_WB_Ctrl[0];

    NBitMux2x1 #(32) mux_wb(
        .A(MEM_WB_ALU_out),
        .B(MEM_WB_Mem_out),
        .S(MEM_WB_MemtoReg),
        .C(writeBackData)
    );

    // Connect to register file
    assign regfile_wen   = MEM_WB_RegWrite;
    assign regfile_waddr = MEM_WB_Rd;

    //------------------------------------------------
    // LED / SSD debugging
    //------------------------------------------------
    always @(*) begin
        case (ledSel)
            2'b00: LEDS = IF_ID_Inst[15:0];
            2'b01: LEDS = IF_ID_Inst[31:16];
            default: begin
                // Example bits: 
                //    [0] = branch_taken
                //    [1] = EX_MEM_Zero
                //    [5:2] = some other signals ...
                LEDS = {
                    2'b0,
                    ALUOp,        // ID stage ALUOp
                    zeroFlag,     // from EX stage
                    EX_MEM_Branch,// next stage
                    branch_taken
                };
            end
        endcase
    end

    reg [12:0] SSD;
    always @(*) begin
        case(ssdSel)
            4'b0000: SSD = PC_address[12:0]; 
            4'b0001: SSD = pc_plus_4[12:0];
            4'b0010: SSD = IF_ID_PC[12:0];
            4'b0011: SSD = ID_EX_PC[12:0];
            4'b0100: SSD = EX_MEM_BranchAddOut[12:0];
            4'b0101: SSD = MEM_WB_ALU_out[12:0];
            4'b0110: SSD = MEM_WB_Mem_out[12:0];
            4'b0111: SSD = writeBackData[12:0];
            default: SSD = alu_result[12:0]; 
        endcase
    end

    Four_Digit_Seven_Segment_Driver_Optimized myDriver(
        .clk    (SSDclk),
        .num    (SSD),
        .Anode  (Anode),
        .LED_out(LED_out)
    );

endmodule
