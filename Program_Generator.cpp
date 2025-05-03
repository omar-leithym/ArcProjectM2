#include <iostream>
#include <cstdlib>
#include <ctime>
#include <string>

using namespace std;

// Register names
const char* regs[32] = {
    "x0","x1","x2","x3","x4","x5","x6","x7",
    "x8","x9","x10","x11","x12","x13","x14","x15",
    "x16","x17","x18","x19","x20","x21","x22","x23",
    "x24","x25","x26","x27","x28","x29","x30","x31"
};

// Simple random functions
int rand_range(int min, int max) {
    return rand() % (max - min + 1) + min;
}
bool rand_bool() {
    return rand_range(0, 1) == 1;
}
int rand_reg(bool nonzero) {
    return nonzero ? rand_range(1, 31) : rand_range(0, 31);
}

// RV32I encoders
unsigned int encodeR(int rd, int rs1, int rs2, bool sub) {
    int f3 = 0;
    int f7 = sub ? 0x20 : 0x00;  // SUB=0b0100000, ADD=0b0000000
    return (f7 << 25) | (rs2 << 20) | (rs1 << 15)
         | (f3 << 12)  | (rd << 7)    | 0x33;
}
unsigned int encodeI(int rd, int rs1, int imm) {
    int opcode = 0x13;  // ADDI
    int f3 = 0;
    imm &= 0xFFF;
    return (imm << 20) | (rs1 << 15) | (f3 << 12)
         | (rd << 7)   | opcode;
}
unsigned int encodeLw(int rd, int imm) {
    int opcode = 0x03;
    int f3 = 2;
    imm &= 0xFFF;
    return (imm << 20) | (0 << 15) | (f3 << 12)
         | (rd << 7)   | opcode;
}
unsigned int encodeSw(int rs2, int imm) {
    int opcode = 0x23;
    int f3 = 2;
    imm &= 0xFFF;
    int hi = (imm >> 5) & 0x7F;
    int lo = imm & 0x1F;
    return (hi << 25) | (rs2 << 20) | (0 << 15)
         | (f3 << 12)  | (lo << 7)    | opcode;
}
unsigned int encodeB(int rs1, int rs2, int imm, bool beq) {
    int opcode = 0x63;
    int f3 = beq ? 0 : 1;
    imm &= 0x1FFF;
    int b12 = (imm >> 12) & 1;
    int b11 = (imm >> 11) & 1;
    int b10_5 = (imm >> 5) & 0x3F;
    int b4_1 = (imm >> 1) & 0xF;
    return (b12 << 31) | (b10_5 << 25) | (rs2 << 20)
         | (rs1 << 15) | (f3 << 12)    | (b4_1 << 8)
         | (b11 << 7)   | opcode;
}
unsigned int encodeU(int rd, int imm, bool lui) {
    int opcode = lui ? 0x37 : 0x17;
    imm &= 0xFFFFF000;
    return (imm) | (rd << 7) | opcode;
}
unsigned int encodeJ(int rd, int imm) {
    int opcode = 0x6F;
    imm &= 0x1FFFFF;
    int b20 = (imm >> 20) & 1;
    int b10_1 = (imm >> 1) & 0x3FF;
    int b11 = (imm >> 11) & 1;
    int b19_12 = (imm >> 12) & 0xFF;
    return (b20 << 31) | (b19_12 << 12) | (b11 << 20)
         | (b10_1 << 21)| (rd << 7)      | opcode;
}

int main(int argc, char* argv[]) {
    int N = 10;
    if (argc > 1) {
        N = atoi(argv[1]);
    }
    srand((unsigned)time(0));

    for (int i = 0; i < N; i++) {
        unsigned int ins;
        string comment;
        int r = rand_range(0, 99);
        if (r < 20) {
            bool sub = rand_bool();
            int rd = rand_reg(true);
            int r1 = rand_reg(false);
            int r2 = rand_reg(false);
            ins = encodeR(rd, r1, r2, sub);
            comment = string(sub ? "sub " : "add ") + regs[rd] + ", " + regs[r1] + ", " + regs[r2];
        } else if (r < 35) {
            int rd = rand_reg(true);
            int r1 = rand_reg(false);
            int imm = rand_range(-2048, 2047);
            ins = encodeI(rd, r1, imm);
            comment = string("addi ") + regs[rd] + ", " + regs[r1] + ", " + to_string(imm);
        } else if (r < 50) {
            int rd = rand_reg(true);
            int imm = rand_range(0, 60);
            ins = encodeLw(rd, imm);
            comment = string("lw ") + regs[rd] + ", " + to_string(imm) + "(x0)";
        } else if (r < 65) {
            int r2 = rand_reg(false);
            int imm = rand_range(0, 60);
            ins = encodeSw(r2, imm);
            comment = string("sw ") + regs[r2] + ", " + to_string(imm) + "(x0)";
        } else if (r < 80) {
            bool beq = rand_bool();
            int r1 = rand_reg(false);
            int r2 = rand_reg(false);
            int imm = rand_range(1, 4) * 4 * (rand_bool() ? 1 : -1);
            ins = encodeB(r1, r2, imm, beq);
            comment = string(beq ? "beq " : "bne ") + regs[r1] + ", " + regs[r2] + ", " + to_string(imm);
        } else if (r < 90) {
            bool lui = rand_bool();
            int rd = rand_reg(true);
            int imm = rand_range(0, 0xFFFFF) << 12;
            ins = encodeU(rd, imm, lui);
            comment = string(lui ? "lui " : "auipc ") + regs[rd] + ", 0x" + to_string(imm);
        } else {
            int rd = rand_reg(true);
            int imm = rand_range(1, 4) * 4 * (rand_bool() ? 1 : -1);
            ins = encodeJ(rd, imm);
            comment = string("jal ") + regs[rd] + ", " + to_string(imm);
        }

        // print binary with underscores
        cout << "mem[" << i << "]=32'b";
        for (int b = 31; b >= 0; b--) {
            cout << ((ins >> b) & 1);
            if (b == 20 || b == 15 || b == 12 || b == 7) cout << '_';
        }
        cout << "; // " << comment << "\n";
    }
    return 0;
}

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <string>

using namespace std;

// Register names
const char* regs[32] = {
    "x0","x1","x2","x3","x4","x5","x6","x7",
    "x8","x9","x10","x11","x12","x13","x14","x15",
    "x16","x17","x18","x19","x20","x21","x22","x23",
    "x24","x25","x26","x27","x28","x29","x30","x31"
};

// Simple random functions
int rand_range(int min, int max) {
    return rand() % (max - min + 1) + min;
}
bool rand_bool() {
    return rand_range(0, 1) == 1;
}
int rand_reg(bool nonzero) {
    return nonzero ? rand_range(1, 31) : rand_range(0, 31);
}

// RV32I encoders
unsigned int encodeR(int rd, int rs1, int rs2, bool sub) {
    int f3 = 0;
    int f7 = sub ? 0x20 : 0x00;  // SUB=0b0100000, ADD=0b0000000
    return (f7 << 25) | (rs2 << 20) | (rs1 << 15)
         | (f3 << 12)  | (rd << 7)    | 0x33;
}
unsigned int encodeI(int rd, int rs1, int imm) {
    int opcode = 0x13;  // ADDI
    int f3 = 0;
    imm &= 0xFFF;
    return (imm << 20) | (rs1 << 15) | (f3 << 12)
         | (rd << 7)   | opcode;
}
unsigned int encodeLw(int rd, int imm) {
    int opcode = 0x03;
    int f3 = 2;
    imm &= 0xFFF;
    return (imm << 20) | (0 << 15) | (f3 << 12)
         | (rd << 7)   | opcode;
}
unsigned int encodeSw(int rs2, int imm) {
    int opcode = 0x23;
    int f3 = 2;
    imm &= 0xFFF;
    int hi = (imm >> 5) & 0x7F;
    int lo = imm & 0x1F;
    return (hi << 25) | (rs2 << 20) | (0 << 15)
         | (f3 << 12)  | (lo << 7)    | opcode;
}
unsigned int encodeB(int rs1, int rs2, int imm, bool beq) {
    int opcode = 0x63;
    int f3 = beq ? 0 : 1;
    imm &= 0x1FFF;
    int b12 = (imm >> 12) & 1;
    int b11 = (imm >> 11) & 1;
    int b10_5 = (imm >> 5) & 0x3F;
    int b4_1 = (imm >> 1) & 0xF;
    return (b12 << 31) | (b10_5 << 25) | (rs2 << 20)
         | (rs1 << 15) | (f3 << 12)    | (b4_1 << 8)
         | (b11 << 7)   | opcode;
}
unsigned int encodeU(int rd, int imm, bool lui) {
    int opcode = lui ? 0x37 : 0x17;
    imm &= 0xFFFFF000;
    return (imm) | (rd << 7) | opcode;
}
unsigned int encodeJ(int rd, int imm) {
    int opcode = 0x6F;
    imm &= 0x1FFFFF;
    int b20 = (imm >> 20) & 1;
    int b10_1 = (imm >> 1) & 0x3FF;
    int b11 = (imm >> 11) & 1;
    int b19_12 = (imm >> 12) & 0xFF;
    return (b20 << 31) | (b19_12 << 12) | (b11 << 20)
         | (b10_1 << 21)| (rd << 7)      | opcode;
}

int main(int argc, char* argv[]) {
    int N = 10;
    if (argc > 1) {
        N = atoi(argv[1]);
    }
    srand((unsigned)time(0));

    for (int i = 0; i < N; i++) {
        unsigned int ins;
        string comment;
        int r = rand_range(0, 99);
        if (r < 20) {
            bool sub = rand_bool();
            int rd = rand_reg(true);
            int r1 = rand_reg(false);
            int r2 = rand_reg(false);
            ins = encodeR(rd, r1, r2, sub);
            comment = string(sub ? "sub " : "add ") + regs[rd] + ", " + regs[r1] + ", " + regs[r2];
        } else if (r < 35) {
            int rd = rand_reg(true);
            int r1 = rand_reg(false);
            int imm = rand_range(-2048, 2047);
            ins = encodeI(rd, r1, imm);
            comment = string("addi ") + regs[rd] + ", " + regs[r1] + ", " + to_string(imm);
        } else if (r < 50) {
            int rd = rand_reg(true);
            int imm = rand_range(0, 60);
            ins = encodeLw(rd, imm);
            comment = string("lw ") + regs[rd] + ", " + to_string(imm) + "(x0)";
        } else if (r < 65) {
            int r2 = rand_reg(false);
            int imm = rand_range(0, 60);
            ins = encodeSw(r2, imm);
            comment = string("sw ") + regs[r2] + ", " + to_string(imm) + "(x0)";
        } else if (r < 80) {
            bool beq = rand_bool();
            int r1 = rand_reg(false);
            int r2 = rand_reg(false);
            int imm = rand_range(1, 4) * 4 * (rand_bool() ? 1 : -1);
            ins = encodeB(r1, r2, imm, beq);
            comment = string(beq ? "beq " : "bne ") + regs[r1] + ", " + regs[r2] + ", " + to_string(imm);
        } else if (r < 90) {
            bool lui = rand_bool();
            int rd = rand_reg(true);
            int imm = rand_range(0, 0xFFFFF) << 12;
            ins = encodeU(rd, imm, lui);
            comment = string(lui ? "lui " : "auipc ") + regs[rd] + ", 0x" + to_string(imm);
        } else {
            int rd = rand_reg(true);
            int imm = rand_range(1, 4) * 4 * (rand_bool() ? 1 : -1);
            ins = encodeJ(rd, imm);
            comment = string("jal ") + regs[rd] + ", " + to_string(imm);
        }

        // print binary with underscores
        cout << "mem[" << i << "]=32'b";
        for (int b = 31; b >= 0; b--) {
            cout << ((ins >> b) & 1);
            if (b == 20 || b == 15 || b == 12 || b == 7) cout << '_';
        }
        cout << "; // " << comment << "\n";
    }
    return 0;
}
