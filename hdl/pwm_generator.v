// ============================================================================
// MODULE 4: PWM GENERATOR
// ============================================================================

module pwm_generator #(
    parameter CLOCK_FREQ = 50000000,
    parameter NUM_CHANNELS = 4,
    parameter COUNTER_WIDTH = 16,
    parameter DEAD_TIME_CYCLES = 10
)(
    input wire clk,
    input wire rst_n,
    input wire [31:0] pwm_frequency,
    input wire update_config,
    input wire [COUNTER_WIDTH-1:0] duty_cycle_0,
    input wire [COUNTER_WIDTH-1:0] duty_cycle_1,
    input wire [COUNTER_WIDTH-1:0] duty_cycle_2,
    input wire [COUNTER_WIDTH-1:0] duty_cycle_3,
    input wire enable_0,
    input wire enable_1,
    input wire enable_2,
    input wire enable_3,
    input wire invert_0,
    input wire invert_1,
    input wire invert_2,
    input wire invert_3,
    input wire [7:0] phase_shift_0,
    input wire [7:0] phase_shift_1,
    input wire [7:0] phase_shift_2,
    input wire [7:0] phase_shift_3,
    input wire hbridge_mode_01,
    input wire hbridge_mode_23,
    input wire emergency_stop,
    output reg pwm_out_0,
    output reg pwm_out_1,
    output reg pwm_out_2,
    output reg pwm_out_3,
    output reg pwm_out_0_n,
    output reg pwm_out_1_n,
    output reg pwm_out_2_n,
    output reg pwm_out_3_n,
    output reg pwm_active,
    output reg [15:0] actual_period
);

    reg [COUNTER_WIDTH-1:0] pwm_counter;
    reg [COUNTER_WIDTH-1:0] pwm_period;
    reg [COUNTER_WIDTH-1:0] pwm_period_next;
    reg [COUNTER_WIDTH-1:0] duty_0_current, duty_0_next;
    reg [COUNTER_WIDTH-1:0] duty_1_current, duty_1_next;
    reg [COUNTER_WIDTH-1:0] duty_2_current, duty_2_next;
    reg [COUNTER_WIDTH-1:0] duty_3_current, duty_3_next;
    reg [COUNTER_WIDTH-1:0] counter_0;
    reg [COUNTER_WIDTH-1:0] counter_1;
    reg [COUNTER_WIDTH-1:0] counter_2;
    reg [COUNTER_WIDTH-1:0] counter_3;
    reg [COUNTER_WIDTH+8-1:0] temp_counter_0;
    reg [COUNTER_WIDTH+8-1:0] temp_counter_1;
    reg [COUNTER_WIDTH+8-1:0] temp_counter_2;
    reg [COUNTER_WIDTH+8-1:0] temp_counter_3;
    reg [7:0] deadtime_counter_01_high;
    reg [7:0] deadtime_counter_01_low;
    reg [7:0] deadtime_counter_23_high;
    reg [7:0] deadtime_counter_23_low;
    wire pwm_raw_0, pwm_raw_1, pwm_raw_2, pwm_raw_3;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_period_next <= {COUNTER_WIDTH{1'b1}};
        end else if (update_config) begin
            if (pwm_frequency > 0 && pwm_frequency <= CLOCK_FREQ) begin
                pwm_period_next <= CLOCK_FREQ / pwm_frequency;
            end else begin
                pwm_period_next <= {COUNTER_WIDTH{1'b1}};
            end
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_period <= {COUNTER_WIDTH{1'b1}};
            actual_period <= {16{1'b1}};
        end else if (pwm_counter == pwm_period - 1) begin
            pwm_period <= pwm_period_next;
            actual_period <= pwm_period_next[15:0];
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= {COUNTER_WIDTH{1'b0}};
            pwm_active <= 1'b0;
        end else if (emergency_stop) begin
            pwm_counter <= {COUNTER_WIDTH{1'b0}};
            pwm_active <= 1'b0;
        end else begin
            if (pwm_counter >= pwm_period - 1) begin
                pwm_counter <= {COUNTER_WIDTH{1'b0}};
            end else begin
                pwm_counter <= pwm_counter + 1'b1;
            end
            pwm_active <= 1'b1;
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter_0 <= {COUNTER_WIDTH{1'b0}};
            counter_1 <= {COUNTER_WIDTH{1'b0}};
            counter_2 <= {COUNTER_WIDTH{1'b0}};
            counter_3 <= {COUNTER_WIDTH{1'b0}};
        end else begin
            temp_counter_0 = pwm_counter + ((pwm_period * phase_shift_0) >> 8);
            temp_counter_1 = pwm_counter + ((pwm_period * phase_shift_1) >> 8);
            temp_counter_2 = pwm_counter + ((pwm_period * phase_shift_2) >> 8);
            temp_counter_3 = pwm_counter + ((pwm_period * phase_shift_3) >> 8);
            
            if (temp_counter_0 >= pwm_period) begin
                counter_0 <= temp_counter_0 - pwm_period;
            end else begin
                counter_0 <= temp_counter_0[COUNTER_WIDTH-1:0];
            end
            if (temp_counter_1 >= pwm_period) begin
                counter_1 <= temp_counter_1 - pwm_period;
            end else begin
                counter_1 <= temp_counter_1[COUNTER_WIDTH-1:0];
            end
            if (temp_counter_2 >= pwm_period) begin
                counter_2 <= temp_counter_2 - pwm_period;
            end else begin
                counter_2 <= temp_counter_2[COUNTER_WIDTH-1:0];
            end
            if (temp_counter_3 >= pwm_period) begin
                counter_3 <= temp_counter_3 - pwm_period;
            end else begin
                counter_3 <= temp_counter_3[COUNTER_WIDTH-1:0];
            end
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            duty_0_next <= {COUNTER_WIDTH{1'b0}};
            duty_1_next <= {COUNTER_WIDTH{1'b0}};
            duty_2_next <= {COUNTER_WIDTH{1'b0}};
            duty_3_next <= {COUNTER_WIDTH{1'b0}};
        end else begin
            duty_0_next <= duty_cycle_0;
            duty_1_next <= duty_cycle_1;
            duty_2_next <= duty_cycle_2;
            duty_3_next <= duty_cycle_3;
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            duty_0_current <= {COUNTER_WIDTH{1'b0}};
            duty_1_current <= {COUNTER_WIDTH{1'b0}};
            duty_2_current <= {COUNTER_WIDTH{1'b0}};
            duty_3_current <= {COUNTER_WIDTH{1'b0}};
        end else if (pwm_counter == pwm_period - 1) begin
            duty_0_current <= duty_0_next;
            duty_1_current <= duty_1_next;
            duty_2_current <= duty_2_next;
            duty_3_current <= duty_3_next;
        end
    end
    
    assign pwm_raw_0 = (counter_0 < duty_0_current) && enable_0 && !emergency_stop;
    assign pwm_raw_1 = (counter_1 < duty_1_current) && enable_1 && !emergency_stop;
    assign pwm_raw_2 = (counter_2 < duty_2_current) && enable_2 && !emergency_stop;
    assign pwm_raw_3 = (counter_3 < duty_3_current) && enable_3 && !emergency_stop;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            deadtime_counter_01_high <= 8'd0;
            deadtime_counter_01_low <= 8'd0;
        end else if (hbridge_mode_01) begin
            if (pwm_raw_0 && !pwm_raw_1) begin
                if (deadtime_counter_01_high < DEAD_TIME_CYCLES) begin
                    deadtime_counter_01_high <= deadtime_counter_01_high + 1'b1;
                end
            end else begin
                deadtime_counter_01_high <= 8'd0;
            end
            if (!pwm_raw_0 && pwm_raw_1) begin
                if (deadtime_counter_01_low < DEAD_TIME_CYCLES) begin
                    deadtime_counter_01_low <= deadtime_counter_01_low + 1'b1;
                end
            end else begin
                deadtime_counter_01_low <= 8'd0;
            end
        end else begin
            deadtime_counter_01_high <= 8'd0;
            deadtime_counter_01_low <= 8'd0;
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            deadtime_counter_23_high <= 8'd0;
            deadtime_counter_23_low <= 8'd0;
        end else if (hbridge_mode_23) begin
            if (pwm_raw_2 && !pwm_raw_3) begin
                if (deadtime_counter_23_high < DEAD_TIME_CYCLES) begin
                    deadtime_counter_23_high <= deadtime_counter_23_high + 1'b1;
                end
            end else begin
                deadtime_counter_23_high <= 8'd0;
            end
            if (!pwm_raw_2 && pwm_raw_3) begin
                if (deadtime_counter_23_low < DEAD_TIME_CYCLES) begin
                    deadtime_counter_23_low <= deadtime_counter_23_low + 1'b1;
                end
            end else begin
                deadtime_counter_23_low <= 8'd0;
            end
        end else begin
            deadtime_counter_23_high <= 8'd0;
            deadtime_counter_23_low <= 8'd0;
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_out_0 <= 1'b0;
            pwm_out_1 <= 1'b0;
            pwm_out_2 <= 1'b0;
            pwm_out_3 <= 1'b0;
            pwm_out_0_n <= 1'b0;
            pwm_out_1_n <= 1'b0;
            pwm_out_2_n <= 1'b0;
            pwm_out_3_n <= 1'b0;
        end else if (emergency_stop) begin
            pwm_out_0 <= 1'b0;
            pwm_out_1 <= 1'b0;
            pwm_out_2 <= 1'b0;
            pwm_out_3 <= 1'b0;
            pwm_out_0_n <= 1'b0;
            pwm_out_1_n <= 1'b0;
            pwm_out_2_n <= 1'b0;
            pwm_out_3_n <= 1'b0;
        end else begin
            if (hbridge_mode_01) begin
                pwm_out_0 <= pwm_raw_0 && (deadtime_counter_01_high >= DEAD_TIME_CYCLES);
                pwm_out_1 <= pwm_raw_1 && (deadtime_counter_01_low >= DEAD_TIME_CYCLES);
                pwm_out_0_n <= !pwm_raw_0 && (deadtime_counter_01_low >= DEAD_TIME_CYCLES);
                pwm_out_1_n <= !pwm_raw_1 && (deadtime_counter_01_high >= DEAD_TIME_CYCLES);
            end else begin
                pwm_out_0 <= invert_0 ? !pwm_raw_0 : pwm_raw_0;
                pwm_out_1 <= invert_1 ? !pwm_raw_1 : pwm_raw_1;
                pwm_out_0_n <= invert_0 ? pwm_raw_0 : !pwm_raw_0;
                pwm_out_1_n <= invert_1 ? pwm_raw_1 : !pwm_raw_1;
            end
            if (hbridge_mode_23) begin
                pwm_out_2 <= pwm_raw_2 && (deadtime_counter_23_high >= DEAD_TIME_CYCLES);
                pwm_out_3 <= pwm_raw_3 && (deadtime_counter_23_low >= DEAD_TIME_CYCLES);
                pwm_out_2_n <= !pwm_raw_2 && (deadtime_counter_23_low >= DEAD_TIME_CYCLES);
                pwm_out_3_n <= !pwm_raw_3 && (deadtime_counter_23_high >= DEAD_TIME_CYCLES);
            end else begin
                pwm_out_2 <= invert_2 ? !pwm_raw_2 : pwm_raw_2;
                pwm_out_3 <= invert_3 ? !pwm_raw_3 : pwm_raw_3;
                pwm_out_2_n <= invert_2 ? pwm_raw_2 : !pwm_raw_2;
                pwm_out_3_n <= invert_3 ? pwm_raw_3 : !pwm_raw_3;
            end
        end
    end
endmodule
