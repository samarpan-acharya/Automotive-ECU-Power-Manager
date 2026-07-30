// ============================================================================
// Testbench    : tb_ecu_pwr_top
// Project      : Automotive ECU Power & Startup Manager IP
// Description  : Self-checking testbench with automated assertion verification,
//                covering cold boot, glitch rejection, brown-out filtering,
//                watchdog timeouts, 3-strike lockout, and graceful shutdown.
// Standard     : IEEE 1364-2001 Verilog HDL
// ============================================================================

`timescale 1ns / 1ps

module tb_ecu_pwr_top;

    // Simulation Clock & Timing Parameters
    localparam CLK_PERIOD         = 10; // 10ns = 100 MHz
    localparam SIM_FILTER_DEPTH   = 4;  // Scaled filter depth for simulation
    localparam SIM_PRECHARGE_CYC  = 10; // Scaled 10ms precharge cycles
    localparam SIM_SLEEP_CYC      = 20; // Scaled 50ms sleep cycles
    localparam SIM_MAX_WATCHDOG   = 15; // Scaled watchdog timeout threshold
    localparam SIM_MAX_RETRIES    = 3;  // 3-strike limit

    // State Encoding Constants for Verification
    localparam [2:0] ST_OFF_RESET      = 3'b000,
                     ST_ACC_PRECHARGE  = 3'b001,
                     ST_BOOT_SEQ       = 3'b010,
                     ST_RUN_NORMAL     = 3'b011,
                     ST_SLEEP_PREP     = 3'b100,
                     ST_FAULT_RECOVERY = 3'b101,
                     ST_HARD_LOCKOUT   = 3'b110;

    // Testbench Signals
    reg        clk;
    reg        rst_n;
    reg        raw_ign_sw;
    reg        raw_vbatt_ok;
    reg        sys_boot_ack;
    reg        heartbeat;
    reg        soft_fault_clear;

    wire       rail_analog_en;
    wire       rail_digital_en;
    wire       rail_sensor_en;
    wire [2:0] fsm_state;
    wire [7:0] fault_code;
    wire [1:0] retry_count_out;
    wire       system_ready;
    wire       lockout_active;

    // Pass/Fail Counters & Test Tracking
    integer error_count = 0;
    integer test_case_num = 0;

    // =========================================================================
    // Device Under Test (DUT) Instantiation
    // =========================================================================
    ecu_pwr_top #(
        .FILTER_DEPTH      (SIM_FILTER_DEPTH),
        .PRECHARGE_CYCLES  (SIM_PRECHARGE_CYC),
        .SLEEP_CYCLES      (SIM_SLEEP_CYC),
        .MAX_WATCHDOG_COUNT(SIM_MAX_WATCHDOG),
        .MAX_RETRIES       (SIM_MAX_RETRIES)
    ) dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .raw_ign_sw      (raw_ign_sw),
        .raw_vbatt_ok    (raw_vbatt_ok),
        .sys_boot_ack    (sys_boot_ack),
        .heartbeat       (heartbeat),
        .soft_fault_clear(soft_fault_clear),
        .rail_analog_en  (rail_analog_en),
        .rail_digital_en (rail_digital_en),
        .rail_sensor_en  (rail_sensor_en),
        .fsm_state       (fsm_state),
        .fault_code      (fault_code),
        .retry_count_out (retry_count_out),
        .system_ready    (system_ready),
        .lockout_active  (lockout_active)
    );

    // =========================================================================
    // Clock Generation (100 MHz)
    // =========================================================================
    always #(CLK_PERIOD / 2) clk = ~clk;

    // Helper Task: State Name Converter for Display
    function [127:0] get_state_name(input [2:0] st);
        case (st)
            ST_OFF_RESET:      get_state_name = "OFF_RESET";
            ST_ACC_PRECHARGE:  get_state_name = "ACC_PRECHARGE";
            ST_BOOT_SEQ:       get_state_name = "BOOT_SEQ";
            ST_RUN_NORMAL:     get_state_name = "RUN_NORMAL";
            ST_SLEEP_PREP:     get_state_name = "SLEEP_PREP";
            ST_FAULT_RECOVERY: get_state_name = "FAULT_RECOVERY";
            ST_HARD_LOCKOUT:   get_state_name = "HARD_LOCKOUT";
            default:           get_state_name = "UNKNOWN";
        endcase
    endfunction

    // Assertion Check Helper Task
    task check_state(input [2:0] expected_state, input [127:0] test_desc);
        begin
            if (fsm_state !== expected_state) begin
                $display("[FAIL] Time=%0t ns | Test #%0d (%0s) | Expected State: %0s, Got: %0s", 
                         $time, test_case_num, test_desc, get_state_name(expected_state), get_state_name(fsm_state));
                error_count = error_count + 1;
            end else begin
                $display("[PASS] Time=%0t ns | Test #%0d (%0s) | Reached State: %0s", 
                         $time, test_case_num, test_desc, get_state_name(fsm_state));
            end
        end
    endtask

    // Check Rail Output Helper Task
    task check_rails(input a_en, input d_en, input s_en, input [127:0] desc);
        begin
            if (rail_analog_en !== a_en || rail_digital_en !== d_en || rail_sensor_en !== s_en) begin
                $display("[FAIL] Time=%0t ns | Rails (%0s) | Expected [A:%b, D:%b, S:%b], Got [A:%b, D:%b, S:%b]",
                         $time, desc, a_en, d_en, s_en, rail_analog_en, rail_digital_en, rail_sensor_en);
                error_count = error_count + 1;
            end else begin
                $display("[PASS] Time=%0t ns | Power Rails OK (%0s) | [A:%b, D:%b, S:%b]",
                         $time, desc, rail_analog_en, rail_digital_en, rail_sensor_en);
            end
        end
    endtask

    // =========================================================================
    // Main Test Stimulus Sequence
    // =========================================================================
    initial begin
        // Setup VCD Waveform Output
        $dumpfile("sim/waveform.vcd");
        $dumpvars(0, tb_ecu_pwr_top);

        // Initialize Signals
        clk              = 0;
        rst_n            = 0;
        raw_ign_sw       = 0;
        raw_vbatt_ok     = 0;
        sys_boot_ack     = 0;
        heartbeat        = 0;
        soft_fault_clear = 0;

        $display("=======================================================================");
        $display("   STARTING AUTOMOTIVE ECU POWER MANAGER SELF-CHECKING VERIFICATION    ");
        $display("=======================================================================");

        // Hold Reset for 50 ns
        #(CLK_PERIOD * 5);
        rst_n = 1;
        #(CLK_PERIOD * 2);
        check_state(ST_OFF_RESET, "Reset Initialization");
        check_rails(0, 0, 0, "Off State Rails");

        // ---------------------------------------------------------------------
        // TEST 1: Glitch Filtering on Ignition and Battery Lines
        // ---------------------------------------------------------------------
        test_case_num = 1;
        $display("\n--- TEST 1: Ignition & Battery Glitch Rejection ---");
        // Apply short 2-cycle noise pulse on ignition (Filter depth = 4)
        raw_ign_sw = 1;
        raw_vbatt_ok = 1;
        #(CLK_PERIOD * 2);
        raw_ign_sw = 0; // Glitch removed before reaching filter threshold
        #(CLK_PERIOD * 5);
        check_state(ST_OFF_RESET, "Ignition Noise Glitch Ignored");

        // ---------------------------------------------------------------------
        // TEST 2: Cold Boot & Sequential Power-Up Sequence
        // ---------------------------------------------------------------------
        test_case_num = 2;
        $display("\n--- TEST 2: Cold Boot Power-Up & Rail Sequencing ---");
        // Apply persistent ignition and battery voltage OK (> 4 clock cycles)
        raw_ign_sw   = 1;
        raw_vbatt_ok = 1;
        #(CLK_PERIOD * (SIM_FILTER_DEPTH + 2)); // Wait for glitch filter
        check_state(ST_ACC_PRECHARGE, "Transition to ACC_PRECHARGE");
        check_rails(1, 0, 0, "Precharge Analog Rail Enabled");

        // Wait for Precharge Hardware Timer (10 cycles)
        #(CLK_PERIOD * (SIM_PRECHARGE_CYC + 2));
        check_state(ST_BOOT_SEQ, "Transition to BOOT_SEQ");
        check_rails(1, 1, 0, "Boot Digital Rail Enabled");

        // Supply CPU Boot Acknowledge
        #(CLK_PERIOD * 3);
        sys_boot_ack = 1;
        #(CLK_PERIOD * 2);
        check_state(ST_RUN_NORMAL, "Transition to RUN_NORMAL");
        check_rails(1, 1, 1, "Run Normal All Rails Enabled");

        // ---------------------------------------------------------------------
        // TEST 3: Heartbeat & Watchdog Timeout Fault Recovery
        // ---------------------------------------------------------------------
        test_case_num = 3;
        $display("\n--- TEST 3: Heartbeat Verification & Watchdog Timeout ---");
        // Send valid heartbeats to maintain RUN_NORMAL
        repeat (5) begin
            heartbeat = 1;
            #(CLK_PERIOD);
            heartbeat = 0;
            #(CLK_PERIOD * 3);
        end
        check_state(ST_RUN_NORMAL, "Maintained RUN_NORMAL via Heartbeat");

        // Stop sending heartbeats to trigger watchdog timeout (15 cycles)
        #(CLK_PERIOD * (SIM_MAX_WATCHDOG + 2));
        check_state(ST_FAULT_RECOVERY, "Watchdog Timeout Triggered FAULT_RECOVERY");
        check_rails(0, 0, 0, "Power Rails Gated Off During Recovery");

        // Auto recovery retry 1 -> ACC_PRECHARGE
        #(CLK_PERIOD * 3);
        check_state(ST_ACC_PRECHARGE, "Automated Recovery Retry #1");

        // Complete boot sequence back to RUN_NORMAL
        #(CLK_PERIOD * (SIM_PRECHARGE_CYC + 2));
        #(CLK_PERIOD * 2);
        sys_boot_ack = 1;
        #(CLK_PERIOD * 2);
        check_state(ST_RUN_NORMAL, "Recovered to RUN_NORMAL");

        // ---------------------------------------------------------------------
        // TEST 4: Engine Cranking Low-Voltage Transient (Brown-Out)
        // ---------------------------------------------------------------------
        test_case_num = 4;
        $display("\n--- TEST 4: Low-Voltage Engine Cranking Brown-Out Transient ---");
        // Simulate sudden voltage dip (raw_vbatt_ok = 0 for 10 cycles)
        raw_vbatt_ok = 0;
        #(CLK_PERIOD * (SIM_FILTER_DEPTH + 2));
        check_state(ST_FAULT_RECOVERY, "Brown-Out Triggered FAULT_RECOVERY");

        // Restore battery voltage
        raw_vbatt_ok = 1;
        #(CLK_PERIOD * (SIM_FILTER_DEPTH + 2));
        check_state(ST_ACC_PRECHARGE, "Automated Recovery Retry #2");

        // ---------------------------------------------------------------------
        // TEST 5: 3-Strike Retry Counter & Permanent HARD_LOCKOUT
        // ---------------------------------------------------------------------
        test_case_num = 5;
        $display("\n--- TEST 5: 3-Strike Retry Counter & HARD_LOCKOUT ---");
        // Cause 3rd failure via boot watchdog failure without completing boot ack
        #(CLK_PERIOD * (SIM_PRECHARGE_CYC + 2)); // Advance to BOOT_SEQ
        sys_boot_ack = 0;
        #(CLK_PERIOD * (SIM_MAX_WATCHDOG + 2)); // Trigger 3rd fault

        #(CLK_PERIOD * 5);
        check_state(ST_HARD_LOCKOUT, "3-Strike Enforced HARD_LOCKOUT");
        if (lockout_active !== 1'b1) begin
            $display("[FAIL] Lockout Active Signal expected HIGH!");
            error_count = error_count + 1;
        end else begin
            $display("[PASS] Lockout Active Signal is HIGH (Safe State)");
        end

        // Ensure state remains locked out even if inputs toggle
        raw_vbatt_ok = 1;
        raw_ign_sw   = 1;
        #(CLK_PERIOD * 10);
        check_state(ST_HARD_LOCKOUT, "Locked Out State Maintained");

        // ---------------------------------------------------------------------
        // TEST 6: Diagnostic Soft Fault Clear
        // ---------------------------------------------------------------------
        test_case_num = 6;
        $display("\n--- TEST 6: Diagnostic Soft Fault Clear ---");
        soft_fault_clear = 1;
        #(CLK_PERIOD * 2);
        soft_fault_clear = 0;
        #(CLK_PERIOD * 2);
        check_state(ST_OFF_RESET, "Soft Fault Clear Restored System to OFF_RESET");

        // ---------------------------------------------------------------------
        // TEST 7: Graceful Power Shutdown Sequence
        // ---------------------------------------------------------------------
        test_case_num = 7;
        $display("\n--- TEST 7: Graceful Power Shutdown Sequence ---");
        // Bring system back to RUN_NORMAL first
        raw_ign_sw   = 1;
        raw_vbatt_ok = 1;
        #(CLK_PERIOD * (SIM_FILTER_DEPTH + 2));
        #(CLK_PERIOD * (SIM_PRECHARGE_CYC + 2));
        sys_boot_ack = 1;
        #(CLK_PERIOD * 2);
        check_state(ST_RUN_NORMAL, "System Operating in RUN_NORMAL");

        // Turn ignition switch OFF
        raw_ign_sw = 0;
        #(CLK_PERIOD * (SIM_FILTER_DEPTH + 2));
        check_state(ST_SLEEP_PREP, "Transition to SLEEP_PREP on Ignition Off");

        // Wait for 50ms Sleep Timer (20 cycles in simulation)
        #(CLK_PERIOD * (SIM_SLEEP_CYC + 2));
        check_state(ST_OFF_RESET, "Graceful Shutdown Complete to OFF_RESET");
        check_rails(0, 0, 0, "All Power Rails Safely Disabled");

        // ---------------------------------------------------------------------
        // Simulation Summary & Output Report
        // ---------------------------------------------------------------------
        $display("\n=======================================================================");
        if (error_count == 0) begin
            $display("   VERIFICATION SUCCESSFUL: ALL TEST CASES PASSED (0 ERRORS)");
            $display("   SYSTEM READY FOR SYNTHESIS & STA VERIFICATION");
        end else begin
            $display("   VERIFICATION FAILED WITH %0d ERROR(S)", error_count);
        end
        $display("=======================================================================\n");

        $finish;
    end

endmodule
