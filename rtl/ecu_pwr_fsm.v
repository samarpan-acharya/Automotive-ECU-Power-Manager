// ============================================================================
// Module Name  : ecu_pwr_fsm
// Project      : Automotive ECU Power & Startup Manager IP
// Description  : Synthesizable 7-state finite state machine (FSM) enforcing 
//                sequential multi-rail power gating, brown-out protection,
//                watchdog heartbeat monitoring, and 3-strike automated recovery
//                lockout for automotive electronic control units.
// Parameters   : MAX_WATCHDOG_COUNT - Threshold for heartbeat timeout (default 250)
//                MAX_RETRIES        - Consecutive retry limit before lockout (default 3)
// Standard     : IEEE 1364-2001 Verilog HDL
// ============================================================================

`timescale 1ns / 1ps

module ecu_pwr_fsm #(
    parameter integer MAX_WATCHDOG_COUNT = 250,
    parameter integer MAX_RETRIES        = 3
)(
    input  wire       clk,              // System clock (100 MHz)
    input  wire       rst_n,            // Active-low asynchronous reset
    input  wire       ign_sw,           // Filtered ignition switch status
    input  wire       vbatt_ok,         // Filtered battery voltage status (1 = Normal, 0 = Brown-out)
    input  wire       precharge_done,   // 10ms timer completion pulse
    input  wire       sleep_done,       // 50ms timer completion pulse
    input  wire       sys_boot_ack,     // CPU/System boot success acknowledge
    input  wire       heartbeat,        // CPU periodic watchdog reset pulse
    input  wire       soft_fault_clear, // External diagnostic override to clear lockout

    output reg        rail_analog_en,   // Power Rail 1: Analog / Precharge circuit
    output reg        rail_digital_en,  // Power Rail 2: Digital Core / MCU rail
    output reg        rail_sensor_en,   // Power Rail 3: Peripheral & Sensor rail
    output reg        start_precharge,  // Active-high timer precharge trigger
    output reg        start_sleep,      // Active-high timer sleep trigger
    output reg        timer_clear,      // Synchronous timer reset
    output reg  [2:0] fsm_state,        // Telemetry state output
    output reg  [7:0] fault_code,       // Diagnostic fault code register
    output reg  [1:0] retry_count_out,  // 3-strike retry counter output
    output reg        system_ready,     // High in RUN_NORMAL state
    output reg        lockout_active    // High in HARD_LOCKOUT state
);

    // =========================================================================
    // State Encoding (3-bit binary encoding)
    // =========================================================================
    localparam [2:0] OFF_RESET      = 3'b000,
                     ACC_PRECHARGE  = 3'b001,
                     BOOT_SEQ       = 3'b010,
                     RUN_NORMAL     = 3'b011,
                     SLEEP_PREP     = 3'b100,
                     FAULT_RECOVERY = 3'b101,
                     HARD_LOCKOUT   = 3'b110;

    // Fault Codes
    localparam [7:0] FAULT_NONE             = 8'h00,
                     FAULT_BROWN_OUT        = 8'h01,
                     FAULT_WATCHDOG_TIMEOUT = 8'h02,
                     FAULT_BOOT_FAILED      = 8'h03,
                     FAULT_HARD_LOCKOUT     = 8'h04;

    // Internal Registers
    reg [2:0] current_state, next_state;
    reg [7:0] watchdog_cnt;
    reg [1:0] retry_cnt;
    reg [7:0] normal_run_timer; // Sustained clean run counter to clear retries

    // Watchdog Flag
    wire watchdog_timeout = (watchdog_cnt >= MAX_WATCHDOG_COUNT);

    // =========================================================================
    // 1. Sequential State Register & Counters
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state    <= OFF_RESET;
            watchdog_cnt     <= 8'd0;
            retry_cnt        <= 2'd0;
            normal_run_timer <= 8'd0;
            fault_code       <= FAULT_NONE;
        end else begin
            current_state <= next_state;

            // Watchdog Counter Logic
            if (current_state == RUN_NORMAL || current_state == BOOT_SEQ) begin
                if (heartbeat) begin
                    watchdog_cnt <= 8'd0;
                end else if (!watchdog_timeout) begin
                    watchdog_cnt <= watchdog_cnt + 1'b1;
                end
            end else begin
                watchdog_cnt <= 8'd0;
            end

            // Retry Counter & Fault Code Management
            if (soft_fault_clear) begin
                retry_cnt  <= 2'd0;
                fault_code <= FAULT_NONE;
            end else if (current_state == FAULT_RECOVERY && next_state != FAULT_RECOVERY) begin
                if (retry_cnt < MAX_RETRIES) begin
                    retry_cnt <= retry_cnt + 1'b1;
                end
            end else if (current_state == RUN_NORMAL) begin
                // Clear retry history after sustained stable run (e.g. 200 cycles)
                if (normal_run_timer >= 8'd200) begin
                    retry_cnt        <= 2'd0;
                    normal_run_timer <= 8'd0;
                end else begin
                    normal_run_timer <= normal_run_timer + 1'b1;
                end
            end else begin
                normal_run_timer <= 8'd0;
            end

            // Fault Code Update on Failure Triggers
            if (!vbatt_ok && current_state != OFF_RESET && current_state != SLEEP_PREP) begin
                fault_code <= FAULT_BROWN_OUT;
            end else if (watchdog_timeout) begin
                fault_code <= FAULT_WATCHDOG_TIMEOUT;
            end else if (current_state == HARD_LOCKOUT) begin
                fault_code <= FAULT_HARD_LOCKOUT;
            end
        end
    end

    // Assign Telemetry Outputs
    always @(*) begin
        fsm_state       = current_state;
        retry_count_out = retry_cnt;
    end

    // =========================================================================
    // 2. Combinational Next-State Logic
    // =========================================================================
    always @(*) begin
        // Default assignment to prevent latch inference
        next_state = current_state;

        case (current_state)
            OFF_RESET: begin
                if (ign_sw && vbatt_ok) begin
                    next_state = ACC_PRECHARGE;
                end
            end

            ACC_PRECHARGE: begin
                if (!vbatt_ok) begin
                    next_state = FAULT_RECOVERY;
                end else if (!ign_sw) begin
                    next_state = SLEEP_PREP;
                end else if (precharge_done) begin
                    next_state = BOOT_SEQ;
                end
            end

            BOOT_SEQ: begin
                if (!vbatt_ok) begin
                    next_state = FAULT_RECOVERY;
                end else if (!ign_sw) begin
                    next_state = SLEEP_PREP;
                end else if (watchdog_timeout) begin
                    next_state = FAULT_RECOVERY;
                end else if (sys_boot_ack) begin
                    next_state = RUN_NORMAL;
                end
            end

            RUN_NORMAL: begin
                if (!vbatt_ok) begin
                    next_state = FAULT_RECOVERY;
                end else if (!ign_sw) begin
                    next_state = SLEEP_PREP;
                end else if (watchdog_timeout) begin
                    next_state = FAULT_RECOVERY;
                end
            end

            SLEEP_PREP: begin
                if (ign_sw && vbatt_ok) begin
                    next_state = ACC_PRECHARGE; // Wakeup interrupt during shutdown
                end else if (sleep_done) begin
                    next_state = OFF_RESET;
                end
            end

            FAULT_RECOVERY: begin
                if (retry_cnt >= MAX_RETRIES) begin
                    next_state = HARD_LOCKOUT;
                end else if (vbatt_ok && ign_sw) begin
                    next_state = ACC_PRECHARGE;
                end
            end

            HARD_LOCKOUT: begin
                if (soft_fault_clear) begin
                    next_state = OFF_RESET;
                end else begin
                    next_state = HARD_LOCKOUT; // Permanent safe-state lockout
                end
            end

            default: next_state = OFF_RESET;
        endcase
    end

    // =========================================================================
    // 3. Combinational Output Logic (Sequential Power Rail Control)
    // =========================================================================
    always @(*) begin
        // Default outputs
        rail_analog_en  = 1'b0;
        rail_digital_en = 1'b0;
        rail_sensor_en  = 1'b0;
        start_precharge = 1'b0;
        start_sleep     = 1'b0;
        timer_clear     = 1'b0;
        system_ready    = 1'b0;
        lockout_active  = 1'b0;

        case (current_state)
            OFF_RESET: begin
                timer_clear = 1'b1;
            end

            ACC_PRECHARGE: begin
                rail_analog_en  = 1'b1;
                start_precharge = 1'b1;
            end

            BOOT_SEQ: begin
                rail_analog_en  = 1'b1;
                rail_digital_en = 1'b1;
            end

            RUN_NORMAL: begin
                rail_analog_en  = 1'b1;
                rail_digital_en = 1'b1;
                rail_sensor_en  = 1'b1;
                system_ready    = 1'b1;
            end

            SLEEP_PREP: begin
                rail_analog_en  = 1'b1;
                rail_digital_en = 1'b1; // Core stays on briefly for context save
                start_sleep     = 1'b1;
            end

            FAULT_RECOVERY: begin
                timer_clear = 1'b1;
            end

            HARD_LOCKOUT: begin
                lockout_active = 1'b1;
                timer_clear    = 1'b1;
            end

            default: ;
        endcase
    end

endmodule
