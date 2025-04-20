`timescale 1ns / 1ps

module forwardingUnit_tb;

    // Inputs
    reg ExMemRegWrite;
    reg [4:0] ExMemRegisterRd;
    reg MemWbRegWrite;
    reg [4:0] MemWbRegisterRd;
    reg [4:0] IdExRegisterRs1;
    reg [4:0] IdExRegisterRs2;

    // Outputs
    wire [1:0] forwardA;
    wire [1:0] forwardB;

    // Instantiate the Unit Under Test (UUT)
    forwardingUnit uut (
        .ExMemRegWrite(ExMemRegWrite),
        .ExMemRegisterRd(ExMemRegisterRd),
        .MemWbRegWrite(MemWbRegWrite),
        .MemWbRegisterRd(MemWbRegisterRd),
        .IdExRegisterRs1(IdExRegisterRs1),
        .IdExRegisterRs2(IdExRegisterRs2),
        .forwardA(forwardA),
        .forwardB(forwardB)
    );

    initial begin
        // Test 1: No hazard
        ExMemRegWrite = 0; ExMemRegisterRd = 5;
        MemWbRegWrite = 0; MemWbRegisterRd = 6;
        IdExRegisterRs1 = 3; IdExRegisterRs2 = 4;
        #10;
        $display("Test 1 (No hazard): forwardA=%b, forwardB=%b", forwardA, forwardB);

        // Test 2: EX hazard on Rs1
        ExMemRegWrite = 1; ExMemRegisterRd = 3;
        MemWbRegWrite = 0; MemWbRegisterRd = 0;
        IdExRegisterRs1 = 3; IdExRegisterRs2 = 4;
        #10;
        $display("Test 2 (EX hazard Rs1): forwardA=%b, forwardB=%b", forwardA, forwardB);

        // Test 3: EX hazard on Rs2
        ExMemRegWrite = 1; ExMemRegisterRd = 4;
        MemWbRegWrite = 0; MemWbRegisterRd = 0;
        IdExRegisterRs1 = 2; IdExRegisterRs2 = 4;
        #10;
        $display("Test 3 (EX hazard Rs2): forwardA=%b, forwardB=%b", forwardA, forwardB);

        // Test 4: MEM hazard on Rs1 (no EX hazard)
        ExMemRegWrite = 0; ExMemRegisterRd = 0;
        MemWbRegWrite = 1; MemWbRegisterRd = 3;
        IdExRegisterRs1 = 3; IdExRegisterRs2 = 2;
        #10;
        $display("Test 4 (MEM hazard Rs1): forwardA=%b, forwardB=%b", forwardA, forwardB);

        // Test 5: MEM hazard on Rs2 (no EX hazard)
        ExMemRegWrite = 0; ExMemRegisterRd = 0;
        MemWbRegWrite = 1; MemWbRegisterRd = 4;
        IdExRegisterRs1 = 1; IdExRegisterRs2 = 4;
        #10;
        $display("Test 5 (MEM hazard Rs2): forwardA=%b, forwardB=%b", forwardA, forwardB);

        // Test 6: Both EX and MEM hazard on Rs1 (EX should take priority)
        ExMemRegWrite = 1; ExMemRegisterRd = 3;
        MemWbRegWrite = 1; MemWbRegisterRd = 3;
        IdExRegisterRs1 = 3; IdExRegisterRs2 = 2;
        #10;
        $display("Test 6 (EX+MEM hazard Rs1): forwardA=%b, forwardB=%b", forwardA, forwardB);

        // Test 7: Both EX and MEM hazard on Rs2 (EX should take priority)
        ExMemRegWrite = 1; ExMemRegisterRd = 4;
        MemWbRegWrite = 1; MemWbRegisterRd = 4;
        IdExRegisterRs1 = 1; IdExRegisterRs2 = 4;
        #10;
        $display("Test 7 (EX+MEM hazard Rs2): forwardA=%b, forwardB=%b", forwardA, forwardB);

        // Test 8: No forwarding when destination register is zero
        ExMemRegWrite = 1; ExMemRegisterRd = 0;
        MemWbRegWrite = 1; MemWbRegisterRd = 0;
        IdExRegisterRs1 = 0; IdExRegisterRs2 = 0;
        #10;
        $display("Test 8 (RegRd=0): forwardA=%b, forwardB=%b", forwardA, forwardB);

        $finish;
    end

endmodule
