// ============================================================================
// Module Name  : glitch_filter
// Project      : Automotive ECU Power & Startup Manager IP
// Description  : Synthesizable N-bit digital glitch filter and debouncer.
//                Suppresses relay chatter and transient voltage spikes on
//                critical control inputs (ignition switch, battery fault lines).
// Parameters   : FILTER_DEPTH - Number of consecutive stable clock cycles 
//                                required before transitioning output state.
// Standard     : IEEE 1364-2001 Verilog HDL
// ============================================================================

`timescale 1ns / 1ps

module glitch_filter #(
    parameter integer FILTER_DEPTH = 16
)(
    input  wire clk,         // System clock (e.g., 100 MHz)
    input  wire rst_n,       // Active-low asynchronous reset
    input  wire signal_in,   // Raw, noisy input signal
    output reg  signal_out   // Filtered, stable output signal
);

    // Calculate counter width dynamically based on FILTER_DEPTH
    localparam integer CNT_WIDTH = (FILTER_DEPTH > 1) ? $clog2(FILTER_DEPTH + 1) : 1;

    reg [CNT_WIDTH-1:0] filter_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            filter_counter <= {CNT_WIDTH{1'b0}};
            signal_out     <= 1'b0;
        end else begin
            if (signal_in != signal_out) begin
                if (filter_counter == (FILTER_DEPTH - 1)) begin
                    signal_out     <= signal_in;
                    filter_counter <= {CNT_WIDTH{1'b0}};
                end else begin
                    filter_counter <= filter_counter + 1'b1;
                end
            end else begin
                // Input matches output; reset counter to filter subsequent glitches
                filter_counter <= {CNT_WIDTH{1'b0}};
            end
        end
    end

endmodule
