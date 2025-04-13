`timescale 1ns / 1ps
module Simulation();
    // Inputs
    reg clk;
    reg reset;
    reg SSDclk;
    
    // Instantiate the Unit Under Test (UUT)
    RISCV_pipeline uut (
        .clk(clk), 
        .reset(reset), 
        .SSDclk(SSDclk)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns period (100MHz)
    end
    
    // SSD clock generation (slower)
    initial begin
        SSDclk = 0;
        forever #20 SSDclk = ~SSDclk; // 100ns period (10MHz)
    end
    
    // Test sequence
    initial begin
        // Initialize inputs
        reset = 1;
        #15
        reset =0;
        
        // Run for enough cycles to execute several instructions
        #600;
        
        // End simulation
        $finish;
    end
    
    // Monitor important signals
    initial begin
        $monitor("Time=%t, PC=%h, Instr=%h, ALUResult=%h, RegWrite=%b", 
                 $time, 
                 uut.PC_address, 
                 uut.instruction, 
                 uut.result, 
                 uut.RegWrite);
    end
    
endmodule
