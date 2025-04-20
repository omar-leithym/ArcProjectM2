`timescale 1ns / 1ps

module Simulation();
    // Inputs
    reg clk;
    reg reset;
    reg SSDclk;
    reg [1:0] ledSel; // Extra input
    reg [3:0] ssdSel; // Extra input
    
    // Outputs
    wire [15:0] LEDS; // Extra output
    wire [3:0] Anode; // Extra output
    wire [6:0] LED_out; // Extra output
    
    // Instantiate the Unit Under Test (UUT)
    RISCV_pipeline uut (
        .clk(clk), 
        .reset(reset), 
        .ledSel(ledSel), // Connect extra input
        .ssdSel(ssdSel), // Connect extra input
        .SSDclk(SSDclk),
        .LEDS(LEDS),     // Connect extra output
        .Anode(Anode),   // Connect extra output
        .LED_out(LED_out) // Connect extra output
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
        ledSel = 2'b00;   // Initialize ledSel
        ssdSel = 4'b0000; // Initialize ssdSel
        
        #15 reset = 0;
        
        // Modify ledSel and ssdSel during simulation to test functionality
        #50 ledSel = 2'b01;
        #50 ssdSel = 4'b0011;
        
        // Run for enough cycles to execute several instructions
        #600;
        
        // End simulation
        $finish;
    end
    
endmodule
