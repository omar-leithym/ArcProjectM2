`timescale 1ns / 1ps

module mux2x1(
    input wire A,
    input wire B,
    input wire S,
    output wire C
);
    assign C = (S) ? B : A;
endmodule

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

module RCA #(parameter n=32)(
    input  [n-1:0] x, 
    input  [n-1:0] y,
    input          cin,
    output [n-1:0] sum,
    output         cout
);

    assign {cout, sum} = x + y + cin;
endmodule


module NBitShiftLeft #(parameter N=32) (
    input  wire [N-1:0] in,
    output wire [N-1:0] out
);
    assign out = in << 1;
endmodule

module ALU(
    input   wire [31:0] a, b,
    input   wire [4:0]  shamt,
    output  reg  [31:0] r,
    output  wire        cf, zf, vf, sf,
    input   wire [3:0]  alufn
);

    wire [31:0] add, op_b;
    wire [31:0] shift_left, shift_right, shift_right_arith;
    assign op_b = (~b);
    
    assign {cf, add} = alufn[0] ? (a + op_b + 1'b1) : (a + b);
    
    assign zf = (add == 0);
    assign sf = add[31];
    assign vf = (a[31] ^ (op_b[31]) ^ add[31] ^ cf);
    
    wire[31:0] sh;
    assign shift_left = a << shamt;
    assign shift_right = a >> shamt;
    assign shift_right_arith = $signed(a) >>> shamt;

    always @ * begin
        r = 0;
        case (alufn)
            // arithmetic
            4'b0000 : r = add;              // ADD, ADDI
            4'b0001 : r = add;              // SUB
            4'b0011 : r = b;                // PASS B
            // logic
            4'b0100 : r = a | b;            // OR, ORI
            4'b0101 : r = a & b;            // AND, ANDI
            4'b0111 : r = a ^ b;            // XOR, XORI
            // shift
            4'b1000 : r = sh;               // SLL
            4'b1001 : r = sh;               // SRL
            4'b1010 : r = sh;               // SRA
            // slt & sltu
            4'b1101 : r = {31'b0,(sf != vf)}; // SLT, SLTI
            4'b1111 : r = {31'b0,(~cf)};      // SLTU, SLTIU
            // additional operations for LUI and AUIPC
            4'b0010 : r = {b[19:0], 12'b0};   // LUI
            4'b0110 : r = a + b;              // AUIPC
            // default case
            default : r = 0;
        endcase
    end
endmodule

module ALU_CU(
    input [2:0] inst14_12,
    input [1:0] ALUOp,
    input       inst30,
    output reg [3:0] ALUsel
);
    always @(*) begin
        case(ALUOp)
            2'b00: ALUsel = 4'b0010; // ADD
            2'b01: ALUsel = 4'b0110; // SUB
            2'b10: begin
                case(inst14_12)
                    3'b000: ALUsel = (inst30 ? 4'b0110 : 4'b0010); // SUB 
                    3'b111: ALUsel = 4'b0000; // AND
                    3'b110: ALUsel = 4'b0001; // OR
                    default: ALUsel = 4'b1111; 
                endcase
            end
            default: ALUsel = 4'b1111;
        endcase
    end
endmodule


module ImmGen(
    input  [31:0] inst,
    output reg [31:0] gen_out
);
    wire [1:0] opcode_two = inst[6:5];

    always @(*) begin
        case (opcode_two)
            2'b00: // I-type 
                gen_out = {{20{inst[31]}}, inst[31:20]};
            2'b01: // S-type 
                gen_out = {{20{inst[31]}}, inst[31:25], inst[11:7]};
            2'b11: // B-type 
                gen_out = {{20{inst[31]}}, inst[31], inst[7],
                           inst[30:25], inst[11:8]};
            default:
                gen_out = 32'b0;
        endcase
    end
endmodule

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
    

    always @(negedge clk) begin
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
    wire sign = num[12];

    always @(*) begin
        case(LED_activating_counter)
            2'b00: begin
                Anode = 4'b0111;
                LED_BCD = (sign) ? 4'b1010 : 4'b1011; 
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


module DataMemory(
    input clk,
    input MemRead,
    input MemWrite,
    input [5:0] addr,
    input [31:0] data_in,
    output reg [31:0] data_out
);
    reg [31:0] mem [0:63];

    always @(posedge clk) begin
        if(MemWrite) begin
            mem[addr] <= data_in;
        end
    end

    always @(*) begin
        if(MemRead)
            data_out = mem[addr];
        else
            data_out = 32'b0;
    end

    initial begin
        mem[0] = 32'd17;  
        mem[1] = 32'd9;   
        mem[2] = 32'd25;  
    end
endmodule


module InstructionMemory(
    input  [31:0] addr, 
    output [31:0] data_out
);
    reg [31:0] mem [0:63];

    assign data_out = mem[addr[7:2]]; 

    initial begin
    mem[0]  = 32'b000000000000_00000_010_00001_0000011; // lw x1, 0(x0)
 
    mem[1]  = 32'b000000000100_00000_010_00010_0000011; // lw x2, 4(x0)
  
    mem[2]  = 32'b000000001000_00000_010_00011_0000011; // lw x3, 8(x0)
    mem[3]  = 32'b0000000_00010_00001_110_00100_0110011; // or x4, x1, x2
    mem[4]  = 32'b0_000001_00011_00100_000_0000_0_1100011; // beq x4, x3, 16
    mem[5]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[6]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[7]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[8]  = 32'b0000000_00010_00001_000_00011_0110011; // add x3, x1, x2
    mem[9]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[10]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0/
    mem[11]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[12]  = 32'b0000000_00010_00011_000_00101_0110011; // add x5, x3, x2
    mem[13]  = 32'b0000000_00101_00000_010_01100_0100011; // sw x5, 12(x0)
    mem[14]  = 32'b000000001100_00000_010_00110_0000011; // lw x6, 12(x0)
    mem[15]  = 32'b0000000_00001_00110_111_00111_0110011; // and x7, x6, x1
    mem[16] = 32'b0100000_00010_00001_000_01000_0110011; // sub x8, x1, x2
    mem[17] = 32'b0000000_00010_00001_000_00000_0110011; // add x0, x1, x2
    mem[18] = 32'b0000000_00001_00000_000_01001_0110011; // add x9, x0, x1

    end
endmodule

module forwardingUnit(
    input ExMemRegWrite,
    input [4:0] ExMemRegisterRd,
    input MemWbRegWrite,
    input [4:0] MemWbRegisterRd,
    input [4:0] IdExRegisterRs1,
    input [4:0] IdExRegisterRs2,
    output reg [1:0] forwardA,
    output reg [1:0] forwardB
    );

    always @(*) begin
        // Default: no forwarding
        forwardA = 2'b00;
        forwardB = 2'b00;

        // EX hazard
        if (ExMemRegWrite && (ExMemRegisterRd != 0) && (ExMemRegisterRd == IdExRegisterRs1))
            forwardA = 2'b10;
        if (ExMemRegWrite && (ExMemRegisterRd != 0) && (ExMemRegisterRd == IdExRegisterRs2))
            forwardB = 2'b10;

        // MEM hazard (only if no EX hazard)
        if (MemWbRegWrite && (MemWbRegisterRd != 0) && (MemWbRegisterRd == IdExRegisterRs1) &&
            !(ExMemRegWrite && (ExMemRegisterRd != 0) && (ExMemRegisterRd == IdExRegisterRs1)))
            forwardA = 2'b01;
        if (MemWbRegWrite && (MemWbRegisterRd != 0) && (MemWbRegisterRd == IdExRegisterRs2) &&
            !(ExMemRegWrite && (ExMemRegisterRd != 0) && (ExMemRegisterRd == IdExRegisterRs2)))
            forwardB = 2'b01;
    end

endmodule

module HazardDetection(
    input  [4:0] IF_ID_RegisterRs1,
    input  [4:0] IF_ID_RegisterRs2,
    input  [4:0] ID_EX_RegisterRd,
    input        ID_EX_MemRead,
    output reg   stall
);
    always @(*) begin
       
        stall = 1'b0;
       
        if (ID_EX_MemRead &&
            (ID_EX_RegisterRd != 5'b0) &&
            ((IF_ID_RegisterRs1 == ID_EX_RegisterRd) ||
             (IF_ID_RegisterRs2 == ID_EX_RegisterRd))) 
        stall = 1'b1;
    end
endmodule

module BranchingUnit(
    input [2:0] funct3, input cf, zf, vf, sf, output reg Branch
    );
    
    always@(*) begin
        case(funct3)
            3'b000: // beq
                Branch = zf;
            3'b001: // bne
                Branch = ~zf;
            3'b100: // blt
                Branch = (sf != vf);
            3'b101: // bge
                Branch = (sf == vf);
            3'b110: // bltu
                Branch = ~cf;
            3'b111: // bgeu
                Branch = cf;
            default: Branch = 1'b0;
        endcase
    end
endmodule

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

    // IF Stage
    reg  [31:0] PC_address; 
    wire [31:0] PC_next;
    wire Stall;
    always @(posedge clk) begin
        if (reset) 
            PC_address <= 32'h0000_0000;  
        else begin
            if(!Stall) begin
                PC_address <= PC_next;
            end
        end
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
        .load (!Stall),
        .D    ({PC_address, instruction}),
        .Q    ({IF_ID_PC, IF_ID_Inst})
    );

    // ID Stage
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
    wire [31:0] writeBackData;
    wire        regfile_wen;
    wire [4:0]  regfile_waddr;
    RegFile myReg(
        .read1     (IF_ID_Inst[19:15]),
        .read2     (IF_ID_Inst[24:20]),
        .write1    (regfile_waddr),
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
    
    
    wire [7:0] ID_stage_ctrl = Stall ? 8'b0 : {
        Branch, MemRead, MemtoReg, ALUOp, memWrite, ALUSrc, RegWrite
    };

    wire [7:0]  ID_EX_Ctrl;
    wire [31:0] ID_EX_PC;
    wire [31:0] ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
    wire [3:0]  ID_EX_Func;
    wire [4:0]  ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;

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
    wire [31:0] alu_input_A;
    wire [31:0] alu_input_B;
    wire [31:0] alu_in2;
    
    assign alu_in2 = ID_EX_ALUSrc ? ID_EX_Imm: alu_input_B;
    wire [31:0] alu_result;
    wire zeroFlag, cFlag, vFlag, sFlag;
    
    ALU #(32) mainALU(
        .a(alu_input_A),
        .b(alu_in2),
        .alufn(ALUsel),
        .r(alu_result),
        .zf(zeroFlag),
        .cf(cFlag),
        .vf(vFlag),
        .sf(sFlag),
        .shamt(ID_EX_Rs2)
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
    wire [2:0] EX_MEM_funct3;
    wire EX_MEM_cFlag, EX_MEM_vFlag, EX_MEM_sFlag;
    NBitRegister #(113) EX_MEM (
        .clk  (clk),
        .rst  (reset),
        .load (1'b1),
        .D    ({
                EX_stage_ctrl,
                branch_target,
                zeroFlag,
                alu_result,
                alu_input_B,
                ID_EX_Rd,
                ID_EX_Func[2:0],
                cFlag,
                vFlag,
                sFlag
               }),
        .Q    ({
                EX_MEM_Ctrl,
                EX_MEM_BranchAddOut,
                EX_MEM_Zero,
                EX_MEM_ALU_out,
                EX_MEM_RegR2,
                EX_MEM_Rd,
                EX_MEM_funct3,
                EX_MEM_cFlag,
                EX_MEM_vFlag,
                EX_MEM_sFlag
               })
    );

    // MEM Stage
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
        .addr     (EX_MEM_ALU_out[7:2]),
        .data_in  (EX_MEM_RegR2),
        .data_out (dataOut)
    );

    wire branchOutput;
    BranchingUnit myBranchUnit(.funct3(EX_MEM_funct3), .cf(EX_MEM_cFlag), .zf(EX_MEM_Zero), .vf(EX_MEM_vFlag), .sf(EX_MEM_sFlag), .Branch(branchOutput));
    wire branch_taken = EX_MEM_Branch && branchOutput;
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

    // WB Stage
    wire MEM_WB_RegWrite = MEM_WB_Ctrl[1];
    wire MEM_WB_MemtoReg = MEM_WB_Ctrl[0];

    NBitMux2x1 #(32) mux_wb(
        .A(MEM_WB_ALU_out),
        .B(MEM_WB_Mem_out),
        .S(MEM_WB_MemtoReg),
        .C(writeBackData)
    );

    wire [1:0] forwardA, forwardB;
    
    forwardingUnit myForwarder(
        .ExMemRegWrite(EX_MEM_RegWrite),
        .ExMemRegisterRd(EX_MEM_Rd), 
        .MemWbRegWrite(MEM_WB_RegWrite), 
        .MemWbRegisterRd(MEM_WB_Rd), 
        .IdExRegisterRs1(ID_EX_Rs1), 
        .IdExRegisterRs2(ID_EX_Rs2), 
        .forwardA(forwardA),
        .forwardB(forwardB)
    );

    
    HazardDetection myHazardDetector (
        .IF_ID_RegisterRs1(IF_ID_Inst[19:15]),
        .IF_ID_RegisterRs2(IF_ID_Inst[24:20]),
        .ID_EX_RegisterRd(ID_EX_Rd),
        .ID_EX_MemRead(ID_EX_MemRead),
        .stall(Stall)
    );
    
    assign alu_input_A = (forwardA == 2'b00) ? ID_EX_RegR1 :
                         (forwardA == 2'b10) ? EX_MEM_ALU_out :
                         (forwardA == 2'b01) ? writeBackData : 
                         ID_EX_RegR1;
    
    // For ALU input B (using ForwardB)

    assign alu_input_B = (forwardB == 2'b00) ? ID_EX_RegR2 :
                         (forwardB == 2'b10) ? EX_MEM_ALU_out :
                         (forwardB == 2'b01) ? writeBackData : 
                         ID_EX_RegR2;

    assign regfile_wen   = MEM_WB_RegWrite;
    assign regfile_waddr = MEM_WB_Rd;

    // LED / SSD debugging
    always @(*) begin
        case (ledSel)
            2'b00: LEDS = IF_ID_Inst[15:0];
            2'b01: LEDS = IF_ID_Inst[31:16];
            default: begin
                LEDS = {
                    2'b0,
                    ALUOp,
                    zeroFlag,
                    EX_MEM_Branch,
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
