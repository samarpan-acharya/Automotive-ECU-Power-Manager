// ============================================================================
// Module Name  : ecu_pwr_top
// Project      : Automotive ECU Power & Startup Manager IP
// Description  : Top-level structural wrapper integrating N-bit digital glitch 
//                filters, hardware timer pipeline, and 7-state power manager 
//                FSM for automotive electronic control units (ECU).
// Standard     : IEEE 1364-2001 Verilog HDL
// ============================================================================

`timescale 1ns / 1ps

module ecu_pwr_top #(
    parameter integer FILTER_DEPTH       = 16,
    parameter integer PRECHARGE_CYCLES   = 1000000,
    parameter integer SLEEP_CYCLES       = 5000000,
    parameter integer MAX_WATCHDOG_COUNT = 250,
    parameter integer MAX_RETRIES        = 3
)(
    input  wire       clk,              // 100 MHz System Clock
    input  wire       rst_n,            // Active-low asynchronous reset
    input  wire       raw_ign_sw,       // Raw, noisy Ignition Switch input
    input  wire       raw_vbatt_ok,     // Raw, noisy Battery Voltage OK status
    input  wire       sys_boot_ack,     // CPU/System boot acknowledge flag
    input  wire       heartbeat,        // Watchdog periodic heartbeat pulse
    input  wire       soft_fault_clear, // Manual diagnostic fault clear trigger

    output wire       rail_analog_en,   // Multi-Rail Power Gate 1: Analog
    output wire       rail_digital_en,  // Multi-Rail Power Gate 2: Digital Core
    output wire       rail_sensor_en,   // Multi-Rail Power Gate 3: Sensors & I/O
    output wire [2:0] fsm_state,        // 3-bit FSM state telemetry
    output wire [7:0] fault_code,       // 8-bit diagnostic fault status
    output wire [1:0] retry_count_out,  // 3-strike retry counter monitor
    output wire       system_ready,     // System fully operational in RUN_NORMAL
    output wire       lockout_active    // Critical failure hard lockout indicator
);

    // Internal Wires & Interconnects
    wire filtered_ign_sw;
    wire filtered_vbatt_ok;
    wire start_precharge;
    wire start_sleep;
    wire timer_clear;
    wire precharge_done;
    wire sleep_done;

    // =========================================================================
    // 1. Ignition Switch Glitch Filter Instance
    // =========================================================================
    glitch_filter #(
        .FILTER_DEPTH(FILTER_DEPTH)
    ) u_glitch_filter_ign (
        .clk        (clk),
        .rst_n      (rst_n),
        .signal_in  (raw_ign_sw),
        .signal_out (filtered_ign_sw)
    );

    // =========================================================================
    // 2. Battery Voltage OK Glitch Filter Instance
    // =========================================================================
    glitch_filter #(
        .FILTER_DEPTH(FILTER_DEPTH)
    ) u_glitch_filter_vbatt (
        .clk        (clk),
        .rst_n      (rst_n),
        .signal_in  (raw_vbatt_ok),
        .signal_out (filtered_vbatt_ok)
    );

    // =========================================================================
    // 3. Hardware Timer Pipeline Instance
    // =========================================================================
    timer_unit #(
        .PRECHARGE_CYCLES(PRECHARGE_CYCLES),
        .SLEEP_CYCLES    (SLEEP_CYCLES)
    ) u_timer_unit (
        .clk             (clk),
        .rst_n           (rst_n),
        .timer_clear     (timer_clear),
        .start_precharge (start_precharge),
        .start_sleep     (start_sleep),
        .precharge_done  (precharge_done),
        .sleep_done      (sleep_done)
    );

    // =========================================================================
    // 4. 7-State ECU Power Manager FSM Controller Instance
    // =========================================================================
    ecu_pwr_fsm #(
        .MAX_WATCHDOG_COUNT(MAX_WATCHDOG_COUNT),
        .MAX_RETRIES        (MAX_RETRIES)
    ) u_ecu_pwr_fsm (
        .clk              (clk),
        .rst_n            (rst_n),
        .ign_sw           (filtered_ign_sw),
        .vbatt_ok         (filtered_vbatt_ok),
        .precharge_done   (precharge_done),
        .sleep_done       (sleep_done),
        .sys_boot_ack     (sys_boot_ack),
        .heartbeat        (heartbeat),
        .soft_fault_clear (soft_fault_clear),
        .rail_analog_en   (rail_analog_en),
        .rail_digital_en  (rail_digital_en),
        .rail_sensor_en   (rail_sensor_en),
        .start_precharge  (start_precharge),
        .start_sleep      (start_sleep),
        .timer_clear      (timer_clear),
        .fsm_state        (fsm_state),
        .fault_code       (fault_code),
        .retry_count_out  (retry_count_out),
        .system_ready     (system_ready),
        .lockout_active   (lockout_active)
    );

endmodule
