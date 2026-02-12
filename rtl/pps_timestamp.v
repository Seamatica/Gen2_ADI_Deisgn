`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// Create Date: 06/06/2025 04:05:41 PM
// Module Name: pps_timestamp
// Description: 
//   - Free-running cycle counter (clk_counter)
//   - PPS event counter (pps_count), resets clk_counter on each PPS
//   - Captures both counters when preamble_detected is asserted
//////////////////////////////////////////////////////////////////////////////////

module pps_timestamp #(
    parameter UTC_SECONDS_WIDTH        = 6,
    parameter COUNT_LAST_SECOND_WIDTH  = 26,
    parameter DRIFT_COUNT_WIDTH        = 13,
    parameter NOMINAL_CYCLES_PER_SEC   = 61_440_000
)(
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         pps,
    input  wire                         event_detected,
    input  wire                         confirm,

    // Event-captured outputs:
    output reg  [UTC_SECONDS_WIDTH-1:0]       event_utc_seconds,
    output reg  [COUNT_LAST_SECOND_WIDTH-1:0] event_clk_counter,
    output reg  signed [DRIFT_COUNT_WIDTH-1:0] event_drift,
    output reg  ready
);
    reg  [COUNT_LAST_SECOND_WIDTH-1:0] clk_counter;
    reg  [UTC_SECONDS_WIDTH-1:0]       pps_count;
    reg  signed [DRIFT_COUNT_WIDTH-1:0] drift;
    reg  [COUNT_LAST_SECOND_WIDTH-1:0] latched_clk_counter;
    reg  [UTC_SECONDS_WIDTH-1:0]       latched_pps_count;
    reg  signed [DRIFT_COUNT_WIDTH-1:0] latched_drift;
    reg  signed [DRIFT_COUNT_WIDTH-1:0] drift_est;

    reg event_detected_d =0;
    reg confirm_d =0;
    reg        fabricated_pps;
    reg        started;

// Parameters
    localparam integer MARGIN = 5;  // tolerance in cycles
    //--------------------------------------------------
    // 1.  PPS synchronizer   (asynchronous ? clk domain)
    //--------------------------------------------------
//--------------------------------------------------
// 1. PPS Synchronization with metastability protection
//--------------------------------------------------
    reg pps_meta, pps_sync, pps_sync_d;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pps_meta <= 0;
            pps_sync <= 0;
            pps_sync_d <= 0;
        end else begin
            pps_meta <= pps;               // Stage 1: Initial synchronization
            pps_sync <= pps_meta;          // Stage 2: Stabilized sync
            pps_sync_d <= pps_sync;        // For edge detection
        end
    end
    
    wire pps_rise = pps_sync & ~pps_sync_d;  // 1-clock strobe
    wire pps_event = pps_rise | fabricated_pps;

    // Start flag logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            started <= 1'b0;
        end else if (pps_rise) begin
            started <= 1'b1;
        end
    end
    
    // Free-running counter
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            clk_counter <= 0;
            fabricated_pps <= 1'b0;
        end else if (started) begin
            if (pps_rise) begin
                clk_counter <= 0;
                fabricated_pps <= 1'b0;
            end else if (clk_counter >= NOMINAL_CYCLES_PER_SEC + drift + MARGIN) begin
                // No PPS arrived in time, fabricate one
                clk_counter <= 5;
                fabricated_pps <= 1'b1;
            end else begin
                clk_counter <= clk_counter + 1'b1;
                fabricated_pps <= 1'b0;
            end
        end
    end
   
    always @(posedge clk or posedge rst) begin
    if (rst) begin
        pps_count   <= 0;
    end else if (started) begin
        if (pps_event) begin
            pps_count <= (pps_count == 6'd59) ? 0 : pps_count + 1'b1;
        end
    end
end

    // Drift computation
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            drift <= 0;
            drift_est <= 0;
        end else if (started && pps_event) begin
            if (fabricated_pps) begin
                // Use last known drift as an estimate
                drift <= drift_est;
            end else begin
                // Normal drift computation
                drift <= $signed(clk_counter) - $signed(NOMINAL_CYCLES_PER_SEC);
                drift_est <= drift;  // update estimate
            end
        end
    end
    
    //--------------------------------------------------
    // 6. Event detection with proper synchronization
    //--------------------------------------------------
    reg event_d;
    always @(posedge clk or posedge rst) begin
        if (rst)
            event_d <= 1'b0;
        else
            event_d <= event_detected;
    end
    
    wire event_rise = event_detected & ~event_d;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            latched_clk_counter <= 0;
            latched_pps_count <= 0;
            latched_drift <= 0;
        end else if (event_rise) begin
            latched_clk_counter <= clk_counter;  // Time since last sync PPS
            latched_pps_count <= pps_count;      // Current UTC second
            latched_drift <= drift;              // Most recent drift
        end
    end
    
    //--------------------------------------------------
    // 7. Confirm stage with edge detection
    //--------------------------------------------------
  
    always @(posedge clk) begin
        confirm_d <= confirm;
    end
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            event_clk_counter <= 0;
            event_utc_seconds <= 0;
            event_drift <= 0;
            ready <= 1'b0;
        end else if (confirm && !confirm_d) begin
            // Calculate actual time since last PPS (add sync delay)
            event_clk_counter <= latched_clk_counter;
            event_utc_seconds <= latched_pps_count;
            event_drift <= latched_drift;
            ready <= 1'b1;
        end
        else ready <= 1'b0;
    end

endmodule

