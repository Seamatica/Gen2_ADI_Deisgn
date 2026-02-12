`timescale 1ns / 1ps

module pps_clk_counter#(
    parameter COUNT_WIDTH = 32
)(
    input  wire                    clk,
    input  wire                    rst,
    input  wire                    pps,         // async PPS input
    output wire [COUNT_WIDTH-1:0]  time_stamp  // clk cycles between PPS pulses
);

    // ------------------------------------------------------------------------
    // 1) Synchronize PPS into clk domain (2-flop synchronizer)
    // ------------------------------------------------------------------------
    reg pps_meta  = 1'b0;
    reg pps_sync  = 1'b0;
    reg pps_sync_d = 1'b0;   // delayed version for edge detect

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pps_meta   <= 1'b0;
            pps_sync   <= 1'b0;
            pps_sync_d <= 1'b0;
        end else begin
            pps_meta   <= pps;        // first stage
            pps_sync   <= pps_meta;   // second stage (this is your pps_sync)
            pps_sync_d <= pps_sync;   // 1-cycle delayed pps_sync
        end
    end

    // Rising edge of synchronized PPS: pps_sync goes 0 -> 1
    wire pps_sync_rise = (pps_sync == 1'b1) && (pps_sync_d == 1'b0);

    // ------------------------------------------------------------------------
    // 2) Count clk cycles between pps_sync rising edges
    // ------------------------------------------------------------------------
    reg [COUNT_WIDTH-1:0] clk_counter   = {COUNT_WIDTH{1'b0}};
    reg [COUNT_WIDTH-1:0] timestamp_reg = {COUNT_WIDTH{1'b0}};

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            clk_counter   <= {COUNT_WIDTH{1'b0}};
            timestamp_reg <= {COUNT_WIDTH{1'b0}};
        end else begin
            if (pps_sync_rise) begin
                // Capture cycles between previous and current PPS edge
                timestamp_reg <= clk_counter;
                // Reset for next interval
                clk_counter   <= {COUNT_WIDTH{1'b0}};
            end else begin
                // Count clk cycles between PPS pulses
                clk_counter <= clk_counter + 1'b1;
            end
        end
    end

    assign time_stamp = timestamp_reg;

endmodule
