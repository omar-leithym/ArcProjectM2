import random
import os
def create_instruction_generator():
    registers = [f"x{i}" for i in range(32)]
    register_codes = {f"x{i}": i for i in range(32)}

    R_instructions = {
        "sll": (0b0110011, 0b001, 0b0000000),
        "srl": (0b0110011, 0b101, 0b0000000),
        "sra": (0b0110011, 0b101, 0b0100000),
        "sub": (0b0110011, 0b000, 0b0100000),
        "xor": (0b0110011, 0b100, 0b0000000),
        "or": (0b0110011, 0b110, 0b0000000),
        "and": (0b0110011, 0b111, 0b0000000),
        "slt": (0b0110011, 0b010, 0b0000000),
        "sltu": (0b0110011, 0b011, 0b0000000),
        "add": (0b0110011, 0b000, 0b0000000),
    }

    I_instructions = {
        "srli": (0b0010011, 0b101),
        "srai": (0b0010011, 0b101),
        "jalr": (0b1100111, 0b000),
        "lbu": (0b0000011, 0b100),
        "lhu": (0b0000011, 0b101),
        "addi": (0b0010011, 0b000),
        "slti": (0b0010011, 0b010),
        "sltiu": (0b0010011, 0b011),
        "slli": (0b0010011, 0b001),
        "lb": (0b0000011, 0b000),
        "lh": (0b0000011, 0b001),
        "lw": (0b0000011, 0b010),
        "xori": (0b0010011, 0b100),
        "ori": (0b0010011, 0b110),
        "andi": (0b0010011, 0b111),
    }

    S_instructions = {
        "sb": (0b0100011, 0b000),
        "sh": (0b0100011, 0b001),
        "sw": (0b0100011, 0b010)
    }

    B_instructions = {
        "beq": (0b1100011, 0b000),
        "bne": (0b1100011, 0b001),
        "blt": (0b1100011, 0b100),
        "bge": (0b1100011, 0b101),
        "bltu": (0b1100011, 0b110),
        "bgeu": (0b1100011, 0b111)
    }

    J_instructions = {
        "jal": 0b1101111
    }

    U_instructions = {
        "lui": 0b0110111,
        "auipc": 0b0010111
    }

    HALTING_INSTRUCTIONS = {
        "ecall": (0b1110011, 0b000),
    }

    def generate_r_type_instruction(opcode, func3, func7, rd, rs1, rs2):
        return (func7 << 25) | (register_codes[rs2] << 20) | (register_codes[rs1] << 15) | \
               (register_codes[rd] << 7) | (func3 << 12) | (opcode << 0)

    def generate_i_type_instruction(opcode, func3, imm, rd, rs1):
        return (imm & 0xFFF) | (register_codes[rs1] << 15) | (register_codes[rd] << 7) | \
               (func3 << 12) | (opcode << 0)

    def generate_b_type_instruction(opcode, func3, imm, rs1, rs2):
        imm12 = (imm >> 12) & 0x1
        imm10_5 = (imm >> 5) & 0x3F
        imm4_1 = (imm >> 1) & 0xF
        imm11 = (imm >> 11) & 0x1
        return (imm12 << 31) | (imm11 << 7) | (imm10_5 << 25) | (register_codes[rs2] << 20) | \
               (register_codes[rs1] << 15) | (func3 << 12) | (opcode << 0)

    def generate_j_type_instruction(opcode, imm, rd):
        imm20 = (imm >> 20) & 0x1
        imm10_1 = (imm >> 1) & 0x3FF
        imm11 = (imm >> 11) & 0x1
        imm19_12 = (imm >> 12) & 0xFF
        return (imm20 << 31) | (imm19_12 << 12) | (imm11 << 20) | (imm10_1 << 1) | \
               (register_codes[rd] << 7) | (opcode << 0)
               
    def generate_u_type_instruction(opcode, imm, rd):
        return (imm & 0xFFFFF) << 12 | (register_codes[rd] << 7) | opcode
    
    def generate_s_type_instruction(opcode, func3, imm, rs1, rs2):
        imm11_5 = (imm >> 5) & 0x7F
        imm4_0 = imm & 0x1F
        return (imm11_5 << 25) | (register_codes[rs2] << 20) | (register_codes[rs1] << 15) | \
            (func3 << 12) | (imm4_0 << 7) | (opcode << 0)

    def write_hex_file(instructions, filename="instructions.hex"):
        print(f"File created at: {os.path.abspath(filename)}")

        with open(filename, "w") as f:
            for instr in instructions:
                f.write(f"{instr:08x}\n")

    def generate_custom_instructions(instruction_types, count):
        instructions = []
        for i in range(count):
            if i == count - 1:
                instr = random.choice(list(HALTING_INSTRUCTIONS.keys()))
                opcode, func3 = HALTING_INSTRUCTIONS[instr]
                rd = "x0" 
                rs1 = "x0"
                imm = 0
                instructions.append(generate_i_type_instruction(opcode, func3, imm, rd, rs1))
            else:
                instr_type = random.choice(instruction_types)
                
                if instr_type == "R":
                    instr = random.choice(list(R_instructions.keys()))
                    opcode, func3, func7 = R_instructions[instr]
                    rd = random.choice(registers)
                    rs1 = random.choice(registers)
                    rs2 = random.choice(registers)
                    instructions.append(generate_r_type_instruction(opcode, func3, func7, rd, rs1, rs2))
                    
                elif instr_type == "I":
                    instr = random.choice(list(I_instructions.keys()))
                    opcode, func3 = I_instructions[instr]
                    imm = random.randint(-128, 127)
                    rd = random.choice(registers)
                    rs1 = random.choice(registers)
                    instructions.append(generate_i_type_instruction(opcode, func3, imm, rd, rs1))
                    
                elif instr_type == "S":
                    instr = random.choice(list(S_instructions.keys()))
                    opcode, func3 = S_instructions[instr]
                    imm = random.randint(-128, 127)
                    rs1 = random.choice(registers)
                    rs2 = random.choice(registers)
                    instructions.append(generate_s_type_instruction(opcode, func3, imm, rs1, rs2))
                    
                elif instr_type == "B":
                    instr = random.choice(list(B_instructions.keys()))
                    opcode, func3 = B_instructions[instr]
                    imm = random.randint(-32, 32) * 2
                    rs1 = random.choice(registers)
                    rs2 = random.choice(registers)
                    instructions.append(generate_b_type_instruction(opcode, func3, imm, rs1, rs2))
                    
                elif instr_type == "J":
                    instr = random.choice(list(J_instructions.keys()))
                    opcode = J_instructions[instr]
                    imm = random.randint(-32, 32) * 2
                    rd = random.choice(registers)
                    instructions.append(generate_j_type_instruction(opcode, imm, rd))
                    
                elif instr_type == "U":
                    instr = random.choice(list(U_instructions.keys()))
                    opcode = U_instructions[instr]
                    imm = random.randint(0, 0xFFFFF)
                    rd = random.choice(registers)
                    instructions.append(generate_u_type_instruction(opcode, imm, rd))
                    
        return instructions


    def display_menu():
        print("\n===== RISC-V Instruction Generator =====")
        print("1: Generate R-type instructions")
        print("2: Generate I-type instructions")
        print("3: Generate S-type instructions")
        print("4: Generate J-type instructions")
        print("5: Generate U-type instructions")
        print("6: Generate B-type instructions")
        print("7: Generate custom mix of instructions")
        print("8: Exit")
        return input("\nEnter your choice (1-8): ")


    def run_generator():
        while True:
            choice = display_menu()
            
            if choice == "1":
                count = int(input("How many R-type instructions? "))
                instructions = generate_custom_instructions(["R"], count)
                filename = input("Output filename (default: r_instructions.hex): ") or "r_instructions.hex"
                write_hex_file(instructions, filename)
                print(f"Generated {count} R-type instructions to {filename}")
                
            elif choice == "2":
                count = int(input("How many I-type instructions? "))
                instructions = generate_custom_instructions(["I"], count)
                filename = input("Output filename (default: i_instructions.hex): ") or "i_instructions.hex"
                write_hex_file(instructions, filename)
                print(f"Generated {count} I-type instructions to {filename}")
                
            elif choice == "3":
                count = int(input("How many S-type instructions? "))
                instructions = generate_custom_instructions(["S"], count)
                filename = input("Output filename (default: s_instructions.hex): ") or "s_instructions.hex"
                write_hex_file(instructions, filename)
                print(f"Generated {count} S-type instructions to {filename}")

                
            elif choice == "4":
                count = int(input("How many J-type instructions? "))
                instructions = generate_custom_instructions(["J"], count)
                filename = input("Output filename (default: j_instructions.hex): ") or "j_instructions.hex"
                write_hex_file(instructions, filename)
                print(f"Generated {count} J-type instructions to {filename}")
                
            elif choice == "5":
                count = int(input("How many U-type instructions? "))
                instructions = generate_custom_instructions(["U"], count)
                filename = input("Output filename (default: u_instructions.hex): ") or "u_instructions.hex"
                write_hex_file(instructions, filename)
                print(f"Generated {count} U-type instructions to {filename}")
                
            elif choice == "6":
                count = int(input("How many B-type instructions? "))
                instructions = generate_custom_instructions(["B"], count)
                filename = input("Output filename (default: b_instructions.hex): ") or "b_instructions.hex"
                write_hex_file(instructions, filename)
                print(f"Generated {count} B-type instructions to {filename}")
              
                
            elif choice == "7":
                types = []
                if input("Include R-type? (y/n): ").lower() == 'y':
                    types.append("R")
                if input("Include I-type? (y/n): ").lower() == 'y':
                    types.append("I")
                if input("Include S-type? (y/n): ").lower() == 'y':
                    types.append("S")
                if input("Include B-type? (y/n): ").lower() == 'y':
                    types.append("B")
                if input("Include J-type? (y/n): ").lower() == 'y':
                    types.append("J")
                if input("Include U-type? (y/n): ").lower() == 'y':
                    types.append("U")

                if not types:
                    print("No instruction types selected!")
                    continue      

                count = int(input("How many instructions to generate? "))
                instructions = generate_custom_instructions(types, count)
                filename = input("Output filename (default: mixed_instructions.hex): ") or "mixed_instructions.hex"
                write_hex_file(instructions, filename)
                print(f"Generated {count} mixed instructions to {filename}")

            elif choice == "8":
                print("Exiting program.")
                break
                
            else:
                print("Invalid choice. Please try again.")

    return run_generator

instruction_generator = create_instruction_generator()
instruction_generator()
