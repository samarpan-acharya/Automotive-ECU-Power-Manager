// ============================================================================
// Module Name  : timer_unit
// Project      : Automotive ECU Power & Startup Manager IP
// Description  : Hardware timer pipeline generating single-cycle completion
//                pulses for 10ms power rail precharge delays and 50ms graceful
//                sleep transitions during low-voltage cranking & power sequences.
// Parameters   : PRECHARGE_CYCLES - Clock cycles for 10ms precharge delay
//                SLEEP_CYCLES     - Clock cycles for 50ms sleep delay
// Standard     : IEEE 1364-2001 Verilog HDL
// ============================================================================

`timescale 1ns / 1ps

module timer_unit #(
    parameter integer PRECHARGE_CYCLES = 1000000, // Default 10ms @ 100MHz
    parameter integer SLEEP_CYCLES     = 5000000  // Default 50ms @ 100MHz
)(
    input  wire clk,              // System clock (100 MHz)
    input  wire rst_n,            // Active-low asynchronous reset
    input  wire timer_clear,      // Synchronous timer counter reset/clear
    input  wire start_precharge,  // Active-high enable for 10ms precharge delay counter
    input  wire start_sleep,      // Active-high enable for 50ms sleep delay counter
    output reg  precharge_done,   // Single-cycle completion pulse for precharge
    output reg  sleep_done        // Single-cycle completion pulse for sleep
);

    // Calculate width needed for largest cycle parameter
    localparam integer MAX_CYCLES = (PRECHARGE_CYCLES > SLEEP_CYCLES) ? PRECHARGE_CYCLES : SLEEP_CYCLES;
    localparam integer CNT_WIDTH  = $clog2(MAX_CYCLES + 1);

    reg [CNT_WIDTH-1:0] timer_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer_counter  <= {CNT_WIDTH{1'b0}};
            precharge_done <= 1'b0;
            sleep_done     <= 1'b0;
        end else if (timer_clear) begin
            timer_counter  <= {CNT_WIDTH{1'b0}};
            precharge_done <= 1'b0;
            sleep_done     <= 1'b0;
        end else begin
            // Default pulse values
            precharge_done <= 1'b0;
            sleep_done     <= 1'b0;

            if (start_precharge) begin
                if (timer_counter >= (PRECHARGE_CYCLES - 1)) begin
                    precharge_done <= 1'b1;
                    timer_counter  <= {CNT_WIDTH{1'b0}};
                end else begin
                    timer_counter <= timer_counter + 1'b1;
                end
            end else if (start_sleep) begin
                if (timer_counter >= (SLEEP_CYCLES - 1)) begin
                    sleep_done    <= 1'b1;
                    timer_counter <= {CNT_WIDTH{1'b0}};
                end else begin
                    timer_counter <= timer_counter + 1'b1;
                end
            end else begin
                timer_counter <= {CNT_WIDTH{1'b0}};
            end
        end
    end

endmodule
