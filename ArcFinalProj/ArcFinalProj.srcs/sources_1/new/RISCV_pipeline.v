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
    input   wire [4:0]  alufn          // ? was [3:0]
);

    wire [31:0] add, op_b;
    wire [31:0] shift_left, shift_right, shift_right_arith;
    reg  [63:0] prod;                  // helper for MUL / DIV hi bits
    assign op_b = (~b);
    assign {cf, add} = alufn[0] ? (a + op_b + 1'b1) : (a + b);

    assign zf = (add == 0);
    assign sf = add[31];
    assign vf = (a[31] ^ (op_b[31]) ^ add[31] ^ cf);

    wire[31:0] sh;
    assign shift_left        = a <<  shamt;
    assign shift_right       = a >>  shamt;
    assign shift_right_arith = $signed(a) >>> shamt;

    always @ * begin
        r = 0;
        case (alufn)
            // arithmetic
            5'b00000 : r = add;              // ADD, ADDI
            5'b00001 : r = add;              // SUB
            5'b00011 : r = b;                // PASS B
            // logic
            5'b00100 : r = a | b;            // OR, ORI
            5'b00101 : r = a & b;            // AND, ANDI
            5'b00111 : r = a ^ b;            // XOR, XORI
            // shift
            5'b01000 : r = sh;               // SLL
            5'b01001 : r = sh;               // SRL
            5'b01010 : r = sh;               // SRA
            // slt & sltu
            5'b01101 : r = {31'b0,(sf != vf)}; // SLT, SLTI
            5'b01111 : r = {31'b0,(~cf)};      // SLTU, SLTIU
            // additional operations for LUI and AUIPC
            5'b00010 : r = {b[19:0], 12'b0};   // LUI
            5'b00110 : r = a + b;              // AUIPC
            // multiply & divide
            5'b10000 : r = a * b;              // MUL
            5'b10001 : begin                   // MULH
                          prod = $signed(a) * $signed(b);
                          r    = prod[63:32];
                       end
            5'b10010 : begin                   // MULHSU
                          prod = $signed(a) * $unsigned(b);
                          r    = prod[63:32];
                       end
            5'b10011 : begin                   // MULHU
                          prod = $unsigned(a) * $unsigned(b);
                          r    = prod[63:32];
                       end
            5'b10100 : r = (b==0) ? 32'hFFFF_FFFF : $signed(a) / $signed(b); // DIV
            5'b10101 : r = (b==0) ? 32'hFFFF_FFFF : a / b;                   // DIVU
            5'b10110 : r = (b==0) ? a            : $signed(a) % $signed(b);  // REM
            5'b10111 : r = (b==0) ? a            : a % b;                    // REMU
            // default case
            default  : r = 0;
        endcase
    end
endmodule


`timescale 1ns / 1ps
module ALU_CU(
    input [2:0] inst14_12,
    input [1:0] ALUOp,
    input       inst30,
    input       inst25,         
    output reg [4:0] ALUsel      
);
    always @(*) begin
        case(ALUOp)
            // For load/store instructions: use ADD
            2'b00: ALUsel = 5'b00000;
            
            // For branches: use SUB
            2'b01: ALUsel = 5'b00001;
            
            // For R-type instructions, decode function fields
            2'b10: begin
                if(inst25) begin
                    // M-extension operations (MUL/DIV)
                    case(inst14_12)
                        3'b000: ALUsel = 5'b10000; // MUL
                        3'b001: ALUsel = 5'b10001; // MULH
                        3'b010: ALUsel = 5'b10010; // MULHSU
                        3'b011: ALUsel = 5'b10011; // MULHU
                        3'b100: ALUsel = 5'b10100; // DIV
                        3'b101: ALUsel = 5'b10101; // DIVU
                        3'b110: ALUsel = 5'b10110; // REM
                        3'b111: ALUsel = 5'b10111; // REMU
                        default: ALUsel = 5'b11111; 
                    endcase
                end else begin
                    // Standard ALU operations
                    case(inst14_12)
                        3'b000: ALUsel = (inst30) ? 5'b00001 : 5'b00000; // SUB if inst30==1 else ADD
                        3'b001: ALUsel = 5'b01000; // SLL
                        3'b010: ALUsel = 5'b01101; // SLT
                        3'b011: ALUsel = 5'b01111; // SLTU
                        3'b100: ALUsel = 5'b00111; // XOR
                        3'b101: ALUsel = (inst30) ? 5'b01010 : 5'b01001; // SRA if inst30 else SRL
                        3'b110: ALUsel = 5'b00100; // OR
                        3'b111: ALUsel = 5'b00101; // AND
                        default: ALUsel = 5'b11111;
                    endcase
                end
            end
            // For JAL/JALR (ALUOp = 2'b11)
            2'b11: begin
                if (inst14_12 == 3'b000) // JALR
                    ALUsel = 5'b00000;   // Use ADD for rs1+imm
                else                     // JAL
                    ALUsel = 5'b00000;   // Use ADD for PC+imm
            end

            
            // Default case
            default: ALUsel = 5'b11111;
        endcase
    end
endmodule


module ImmGen(
    input  [31:0] inst,
    output reg [31:0] gen_out
);
    // extract immediates based on full 7-bit opcode
    always @(*) begin
        case (inst[6:0])
            // I-type: ADDI, LW, JALR, etc.
            7'b0010011, // ADDI, ORI, ANDI, ...
            7'b0000011, // LB, LH, LW, ...
            7'b1100111: // JALR
                gen_out = {{20{inst[31]}}, inst[31:20]};

            // S-type: SB, SH, SW
            7'b0100011: // SW, SH, SB
                gen_out = {{20{inst[31]}}, inst[31:25], inst[11:7]};

            // B-type: BEQ, BNE, BLT, BGE, ...
            7'b1100011: // BEQ, BNE, BLT, BGE, BLTU, BGEU
                gen_out = {{20{inst[31]}}, inst[31], inst[7],
                           inst[30:25], inst[11:8]};

            // U-type: LUI, AUIPC
            7'b0110111, // LUI
            7'b0010111: // AUIPC
                gen_out = {inst[31:12], 12'b0};

            // J-type: JAL
            7'b1101111: // JAL
                gen_out = {{inst[31]}, inst[19:12], inst[20],
                           inst[30:21], 1'b0};

            default:
                gen_out = 32'b0;
        endcase
    end
endmodule


`timescale 1ns / 1ps

module CU(
    input [4:0] instBits,
    output reg Branch, 
    output reg MemRead, 
    output reg MemtoReg,
    output reg [1:0] ALUOp, 
    output reg memWrite, 
    output reg ALUSrc, 
    output reg RegWrite,
    output reg Halt
);
    always @(*) begin
        // Default values to avoid latches
        Branch = 1'b0; 
        MemRead = 1'b0; 
        MemtoReg = 1'b0; 
        ALUOp = 2'b00; 
        memWrite = 1'b0; 
        ALUSrc = 1'b0; 
        RegWrite = 1'b0;
        Halt = 1'b0;
        
        case(instBits)
            // R-type instructions
            5'b01100: begin
                Branch = 1'b0; 
                MemRead = 1'b0; 
                MemtoReg = 1'b0; 
                ALUOp = 2'b10; 
                memWrite = 1'b0; 
                ALUSrc = 1'b0; 
                RegWrite = 1'b1;
                Halt = 1'b0;
            end
            
            // Load instructions (LW)
            5'b00000: begin
                Branch = 1'b0; 
                MemRead = 1'b1; 
                MemtoReg = 1'b1; 
                ALUOp = 2'b00; 
                memWrite = 1'b0; 
                ALUSrc = 1'b1; 
                RegWrite = 1'b1;
                Halt = 1'b0;
            end
            
            // Store instructions (SW)
            5'b01000: begin
                Branch = 1'b0; 
                MemRead = 1'b0; 
                MemtoReg = 1'b0; 
                ALUOp = 2'b00; 
                memWrite = 1'b1; 
                ALUSrc = 1'b1; 
                RegWrite = 1'b0;
                Halt = 1'b0;
            end
            
            // Branch instructions (BEQ)
            5'b11000: begin
                Branch = 1'b1; 
                MemRead = 1'b0; 
                MemtoReg = 1'b0; 
                ALUOp = 2'b01; 
                memWrite = 1'b0; 
                ALUSrc = 1'b0; 
                RegWrite = 1'b0;
                Halt = 1'b0;
            end
            
            // I-type instructions (ADDI)
            5'b00100: begin
                Branch = 1'b0; 
                MemRead = 1'b0; 
                MemtoReg = 1'b0; 
                ALUOp = 2'b00; 
                memWrite = 1'b0; 
                ALUSrc = 1'b1; 
                RegWrite = 1'b1;
                Halt = 1'b0;
            end
            
            // LUI instruction
            5'b01101: begin
                Branch = 1'b0; 
                MemRead = 1'b0; 
                MemtoReg = 1'b0; 
                ALUOp = 2'b10; 
                memWrite = 1'b0; 
                ALUSrc = 1'b1; 
                RegWrite = 1'b1;
                Halt = 1'b0;
            end
            
            // AUIPC instruction
            5'b00101: begin
                Branch = 1'b0; 
                MemRead = 1'b0; 
                MemtoReg = 1'b0; 
                ALUOp = 2'b11; 
                memWrite = 1'b0; 
                ALUSrc = 1'b1; 
                RegWrite = 1'b1;
                Halt = 1'b0;
            end
            
            // FENCE instruction
            5'b00011: begin
                Branch = 1'b0; 
                MemRead = 1'b0; 
                MemtoReg = 1'b0; 
                ALUOp = 2'b00; 
                memWrite = 1'b0; 
                ALUSrc = 1'b0; 
                RegWrite = 1'b0;
                Halt = 1'b1;
            end
            
            // ECALL and EBREAK
            5'b11100: begin
                Branch = 1'b0; 
                MemRead = 1'b0; 
                MemtoReg = 1'b0; 
                ALUOp = 2'b00; 
                memWrite = 1'b0; 
                ALUSrc = 1'b0; 
                RegWrite = 1'b0;
                Halt = 1'b1;
            end
            
            // JAL instruction (opcode 1101111)
            5'b11011: begin
                Branch = 1'b1;     // Treat as branch for PC control
                MemRead = 1'b0;
                MemtoReg = 1'b0;
                ALUOp = 2'b11;     // Special ALUOp for JAL
                memWrite = 1'b0;
                ALUSrc = 1'b1;     // Use immediate
                RegWrite = 1'b1;   // Write return address to rd
                Halt = 1'b0;
            end
            
            // JALR instruction (opcode 1100111)
            5'b11001: begin
                Branch = 1'b1;     // Treat as branch for PC control
                MemRead = 1'b0;
                MemtoReg = 1'b0;
                ALUOp = 2'b11;     // Special ALUOp for JALR
                memWrite = 1'b0;
                ALUSrc = 1'b1;     // Use immediate
                RegWrite = 1'b1;   // Write return address to rd
                Halt = 1'b0;
            end
            
            // JAL instruction (opcode 1101111)
            5'b11011: begin
                Branch = 1'b1;     // Treat as branch for PC control
                MemRead = 1'b0;
                MemtoReg = 1'b0;
                ALUOp = 2'b11;     // Special ALUOp for JAL
                memWrite = 1'b0;
                ALUSrc = 1'b1;     // Use immediate
                RegWrite = 1'b1;   // Write return address to rd
                Halt = 1'b0;
            end
            
            // JALR instruction (opcode 1100111)
            5'b11001: begin
                Branch = 1'b1;     // Treat as branch for PC control
                MemRead = 1'b0;
                MemtoReg = 1'b0;
                ALUOp = 2'b11;     // Special ALUOp for JALR
                memWrite = 1'b0;
                ALUSrc = 1'b1;     // Use immediate
                RegWrite = 1'b1;   // Write return address to rd
                Halt = 1'b0;
            end
         
            default: begin
                Branch = 1'b0; 
                MemRead = 1'b0; 
                MemtoReg = 1'b0; 
                ALUOp = 2'b00; 
                memWrite = 1'b0; 
                ALUSrc = 1'b0; 
                RegWrite = 1'b0;
                Halt = 1'b0;
            end
        endcase
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


module Memory(
    input clk,
    input MemRead,
    input MemWrite,
    input [2:0] funct3,      // Instruction type (lb, lh, lw, etc.)
    input [31:0] addr,       // Full 32-bit address
    input [31:0] data_in,    // Data to write
    output reg [31:0] data_out // Data read
);
    reg [7:0] mem [0:255];   // Byte-addressable memory
    wire [1:0] byte_offset = addr[1:0];
    
    // Read logic
    always @(*) begin
        if(MemRead) begin
            case(funct3)
                3'b000: begin // lb - load byte
                    case(byte_offset)
                        2'b00: data_out = {{24{mem[addr][7]}}, mem[addr]};
                        2'b01: data_out = {{24{mem[addr][7]}}, mem[addr]};
                        2'b10: data_out = {{24{mem[addr][7]}}, mem[addr]};
                        2'b11: data_out = {{24{mem[addr][7]}}, mem[addr]};
                    endcase
                end
                3'b001: begin // lh - load halfword
                    case(byte_offset)
                        2'b00: data_out = {{16{mem[addr+1][7]}}, mem[addr+1], mem[addr]};
                        2'b10: data_out = {{16{mem[addr+1][7]}}, mem[addr+1], mem[addr]};
                        default: data_out = 32'b0; // Unaligned access
                    endcase
                end
                3'b010: begin // lw - load word
                    if(byte_offset == 2'b00)
                        data_out = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};
                    else
                        data_out = 32'b0; // Unaligned access
                end
                3'b100: begin // lbu - load byte unsigned
                    case(byte_offset)
                        2'b00: data_out = {24'b0, mem[addr]};
                        2'b01: data_out = {24'b0, mem[addr]};
                        2'b10: data_out = {24'b0, mem[addr]};
                        2'b11: data_out = {24'b0, mem[addr]};
                    endcase
                end
                3'b101: begin // lhu - load halfword unsigned
                    case(byte_offset)
                        2'b00: data_out = {16'b0, mem[addr+1], mem[addr]};
                        2'b10: data_out = {16'b0, mem[addr+1], mem[addr]};
                        default: data_out = 32'b0; // Unaligned access
                    endcase
                end
                default: begin
                    // For instruction fetch or when funct3 is not specified
                    // Always fetch a word (4 bytes) for instructions
                    if(byte_offset == 2'b00)
                        data_out = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};
                    else
                        data_out = 32'b0; // Unaligned access
                end
            endcase
        end else begin
            // Even when MemRead is not active, we need to read instructions
            // This handles the instruction fetch path
            data_out = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};
        end
    end
    
    // Write logic
    always @(*) begin
        if(MemWrite) begin
            case(funct3)
                3'b000: begin // sb - store byte
                    mem[addr] <= data_in[7:0];
                end
                3'b001: begin // sh - store halfword
                    if(byte_offset == 2'b00 || byte_offset == 2'b10) begin
                        mem[addr] <= data_in[7:0];
                        mem[addr+1] <= data_in[15:8];
                    end
                end
                3'b010: begin // sw - store word
                    if(byte_offset == 2'b00) begin
                        mem[addr] <= data_in[7:0];
                        mem[addr+1] <= data_in[15:8];
                        mem[addr+2] <= data_in[23:16];
                        mem[addr+3] <= data_in[31:24];
                    end
                end
                default: begin // Default to word store
                    if(byte_offset == 2'b00) begin
                        mem[addr] <= data_in[7:0];
                        mem[addr+1] <= data_in[15:8];
                        mem[addr+2] <= data_in[23:16];
                        mem[addr+3] <= data_in[31:24];
                    end
                end
            endcase
        end
    end

    initial begin
        // Initialize memory with test data and program
        // Data values (byte by byte in little-endian)
        mem[80] = 8'd100;   // Value 100 at address 0x50
        mem[81] = 8'd0;
        mem[82] = 8'd0;
        mem[83] = 8'd0;
    
        mem[84] = 8'd20;    // Value 20 at address 0x54
        mem[85] = 8'd0;
        mem[86] = 8'd0;
        mem[87] = 8'd0;
    
        // Program (machine code instructions)
        // lw x10, 5(x0)
        mem[0]  = 8'b00000011;
        mem[1]  = 8'b00000101;
        mem[2]  = 8'b01010000;
        mem[3]  = 8'b00000000;
    
        // beq x0,x0,8
        mem[4]  = 8'b01100011;
        mem[5]  = 8'b00000100;
        mem[6]  = 8'b00000000;
        mem[7]  = 8'b00000000;
    
        // add x7, x5, x6 - x7 = x5 + x6 WRONG
        mem[8]  = 8'b00110011;
        mem[9]  = 8'b10100110;
        mem[10] = 8'b00110100;
        mem[11] = 8'b00000000;
    
        // sub x8, x7, x6 - x8 = x7 - x6 WRONG
        mem[12] = 8'b00110011;
        mem[13] = 8'b00000111;
        mem[14] = 8'b00111000;
        mem[15] = 8'b01000000;
    
        // sw x8, 88(x0) - Store result at 0x58 WRONG
        mem[16] = 8'b00100011;
        mem[17] = 8'b00001000;
        mem[18] = 8'b10010000;
        mem[19] = 8'b00000000;
    
        // ecall (just placeholder)
        mem[20] = 8'b00001111;
        mem[21] = 8'b00000000;
        mem[22] = 8'b00000000;
        mem[23] = 8'b00000000;
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
            // In BranchingUnit, add cases for JAL/JALR
            3'b011: // JAL
                Branch = 1'b1;  // Always take the jump
            3'b100: // JALR
                Branch = 1'b1;  // Always take the jump
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
    wire Stall, Halt;
    reg Halt_latched;
    wire EX_MEM_MemRead;
    wire EX_MEM_MemWrite;
    wire branch_taken;
    always @(posedge clk) begin
        if (reset) 
            PC_address <= 32'h0000_0000;  
        else begin
            if(!Stall && !Halt_latched && !EX_MEM_MemRead && !EX_MEM_MemWrite) begin
                PC_address <= PC_next;
            end
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset)
            Halt_latched <= 1'b0;
        else if (Halt)
            Halt_latched <= 1'b1;
    end
    wire [31:0] instruction;

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
    wire [31:0] instructionSent = branch_taken ? 32'b00000000000000000000000000110011: instruction; // add x0, x0, x0
    NBitRegister #(64) IF_ID (
        .clk  (clk),
        .rst  (reset),
        .load (!Stall && !EX_MEM_MemRead && !EX_MEM_MemWrite),
        .D    ({PC_address, instructionSent}),
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
        .RegWrite (RegWrite),
        .Halt(Halt)
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
    
    
    wire [7:0] ID_stage_ctrl = Stall || Halt_latched || EX_MEM_MemRead || EX_MEM_MemWrite || branch_taken ? 8'b0 : {
        Branch, MemRead, MemtoReg, ALUOp, memWrite, ALUSrc, RegWrite
    };

    wire [7:0]  ID_EX_Ctrl;
    wire [31:0] ID_EX_PC;
    wire [31:0] ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
    wire [4:0]  ID_EX_Func;
    wire [4:0]  ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;

    NBitRegister #(156) ID_EX (
        .clk  (clk),
        .rst  (reset),
        .load (1'b1),
        .D    ({
             ID_stage_ctrl,
             IF_ID_PC,
             readData1,
             readData2,
             immediate,
             {IF_ID_Inst[30], IF_ID_Inst[14:12], IF_ID_Inst[25]}, // +inst25
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
    wire [4:0] ALUsel;
    ALU_CU aluControl(
        .inst14_12(ID_EX_Func[3:1]),
        .ALUOp    (ID_EX_ALUOp),
        .inst30   (ID_EX_Func[4]),
        .inst25 (ID_EX_Func[0]),
        .ALUsel   (ALUsel)
    );

    // ALU input2 can be RegR2 or immediate
    wire [31:0] alu_input_A;
    wire [31:0] alu_input_B;
    wire [31:0] alu_in2;
    
    assign alu_in2 = ID_EX_ALUSrc ? ID_EX_Imm: alu_input_B;
    wire [31:0] alu_result;
    wire zeroFlag, cFlag, vFlag, sFlag;
    
        ALU mainALU(
        .a     (alu_input_A),
        .b     (alu_in2),
        .shamt (ID_EX_Rs2),
        .r     (alu_result),
        .zf    (zeroFlag),
        .cf    (cFlag),
        .vf    (vFlag),
        .sf    (sFlag),
        .alufn (ALUsel)              
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
    wire [4:0] EX_stage_ctrl = branch_taken ? 5'b0 : {
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
    assign EX_MEM_MemRead  = EX_MEM_Ctrl[3];
    wire EX_MEM_MemtoReg = EX_MEM_Ctrl[2];
    assign EX_MEM_MemWrite = EX_MEM_Ctrl[1];
    wire EX_MEM_RegWrite = EX_MEM_Ctrl[0];

    wire [31:0] dataOut;
    wire [31:0] mem_out;
    Memory mem(
        .clk      (clk),
        .MemRead  (EX_MEM_MemRead),
        .MemWrite (EX_MEM_MemWrite),
        .addr     (EX_MEM_MemWrite || EX_MEM_MemRead ? EX_MEM_ALU_out : PC_address),
        .data_in  (EX_MEM_RegR2),
        .data_out (mem_out),
        .funct3(EX_MEM_funct3)
    );
    
    wire is_data_access = EX_MEM_MemWrite || EX_MEM_MemRead;
    assign dataOut = is_data_access ? mem_out : 32'b0;
    assign instruction = is_data_access || Halt_latched ? instruction : mem_out;
    
    wire branchOutput;
    BranchingUnit myBranchUnit(.funct3(EX_MEM_funct3), .cf(EX_MEM_cFlag), .zf(EX_MEM_Zero), .vf(EX_MEM_vFlag), .sf(EX_MEM_sFlag), .Branch(branchOutput));
    assign branch_taken = EX_MEM_Branch && branchOutput;
    
    // Determine next PC value
    wire is_jalr = (ID_EX_Ctrl[4:3] == 2'b11) && (ID_EX_Func[3:1] == 3'b000);
    wire [31:0] jalr_target = alu_result & 32'hFFFFFFFE; // Clear LSB for JALR
    
    NBitMux2x1 #(32) mux_nextPC(
        .A(pc_plus_4),
        .B(is_jalr ? jalr_target : EX_MEM_BranchAddOut),
        .S(branch_taken || is_jalr),
        .C(PC_next)
    );

    
//    NBitMux2x1 #(32) mux_nextPC(
//        .A(pc_plus_4),
//        .B(EX_MEM_BranchAddOut),
//        .S(branch_taken),
//        .C(PC_next)
//    );

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
    
// Create a signal for the return address (PC+4)
        wire [31:0] return_address = ID_EX_PC + 4;
        
        // Modify ALU input selection for JAL/JALR
        wire is_jump = (ID_EX_ALUOp == 2'b11);
        assign alu_input_A = is_jump ? ID_EX_PC : 
                            (forwardA == 2'b00) ? ID_EX_RegR1 :
                            (forwardA == 2'b10) ? EX_MEM_ALU_out :
                            (forwardA == 2'b01) ? writeBackData : 
                            ID_EX_RegR1;


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
