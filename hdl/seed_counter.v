// ============================================================================
// MODULE 2: SEED COUNTER
// ============================================================================

module seed_counter #(
    parameter CLOCK_FREQ = 50000000,
    parameter MIN_PULSE_WIDTH_US = 10,
    parameter MAX_PULSE_WIDTH_US = 500,
    parameter SEEDS_PER_HOLE_BEANS = 5,
    parameter SEEDS_PER_HOLE_MAIZE = 2,
    parameter DEBOUNCE_CYCLES = 100
)(
    input wire clk,
    input wire rst_n,
    input wire ir_sensor_1,
    input wire ir_sensor_2,
    input wire sensor_enable,
    input wire start_counting,
    input wire stop_counting,
    input wire clear_count,
    input wire crop_type,
    input wire use_dual_sensors,
    input wire [7:0] target_seed_count,
    input wire use_custom_target,
    output reg [7:0] seed_count,
    output reg [31:0] total_seed_count,
    output reg [15:0] hole_count,
    output reg target_reached,
    output reg counting_active,
    output reg [15:0] seed_rate,
    output reg sensor_error,
    output reg [7:0] error_count,
    output reg sensor_1_status,
    output reg sensor_2_status,
    output reg mismatch_detected,
    output reg [7:0] mismatch_count
);

    localparam MIN_PULSE_CYCLES = (CLOCK_FREQ / 1000000) * MIN_PULSE_WIDTH_US;
    localparam MAX_PULSE_CYCLES = (CLOCK_FREQ / 1000000) * MAX_PULSE_WIDTH_US;
    localparam RATE_SAMPLE_CYCLES = CLOCK_FREQ;
    localparam SENSOR_TIMEOUT_CYCLES = CLOCK_FREQ * 10;
    
    reg [DEBOUNCE_CYCLES-1:0] sensor_1_shift;
    reg [DEBOUNCE_CYCLES-1:0] sensor_2_shift;
    wire sensor_1_stable;
    wire sensor_2_stable;
    reg sensor_1_prev;
    reg sensor_2_prev;
    wire sensor_1_falling;
    wire sensor_2_falling;
    wire sensor_1_rising;
    wire sensor_2_rising;
    reg [31:0] pulse_width_1;
    reg [31:0] pulse_width_2;
    reg pulse_1_active;
    reg pulse_2_active;
    wire pulse_1_valid;
    wire pulse_2_valid;
    reg seed_detected_1;
    reg seed_detected_2;
    reg seed_confirmed;
    reg [7:0] target_seeds;
    reg [31:0] rate_timer;
    reg [7:0] seeds_in_period;
    reg [31:0] last_seed_time;
    reg [31:0] sensor_timeout_1;
    reg [31:0] sensor_timeout_2;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_1_shift <= {DEBOUNCE_CYCLES{1'b1}};
            sensor_2_shift <= {DEBOUNCE_CYCLES{1'b1}};
        end else if (sensor_enable) begin
            sensor_1_shift <= {sensor_1_shift[DEBOUNCE_CYCLES-2:0], ir_sensor_1};
            sensor_2_shift <= {sensor_2_shift[DEBOUNCE_CYCLES-2:0], ir_sensor_2};
        end
    end
    
    assign sensor_1_stable = (sensor_1_shift == {DEBOUNCE_CYCLES{1'b0}}) ? 1'b0 :
                            (sensor_1_shift == {DEBOUNCE_CYCLES{1'b1}}) ? 1'b1 :
                            sensor_1_prev;
    assign sensor_2_stable = (sensor_2_shift == {DEBOUNCE_CYCLES{1'b0}}) ? 1'b0 :
                            (sensor_2_shift == {DEBOUNCE_CYCLES{1'b1}}) ? 1'b1 :
                            sensor_2_prev;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_1_prev <= 1'b1;
            sensor_2_prev <= 1'b1;
        end else begin
            sensor_1_prev <= sensor_1_stable;
            sensor_2_prev <= sensor_2_stable;
        end
    end
    
    assign sensor_1_falling = sensor_1_prev & ~sensor_1_stable;
    assign sensor_1_rising = ~sensor_1_prev & sensor_1_stable;
    assign sensor_2_falling = sensor_2_prev & ~sensor_2_stable;
    assign sensor_2_rising = ~sensor_2_prev & sensor_2_stable;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_width_1 <= 32'd0;
            pulse_1_active <= 1'b0;
        end else if (sensor_enable && counting_active) begin
            if (sensor_1_falling) begin
                pulse_1_active <= 1'b1;
                pulse_width_1 <= 32'd0;
            end else if (pulse_1_active) begin
                if (sensor_1_rising) begin
                    pulse_1_active <= 1'b0;
                end else if (pulse_width_1 < MAX_PULSE_CYCLES + 1000) begin
                    pulse_width_1 <= pulse_width_1 + 1'b1;
                end else begin
                    pulse_1_active <= 1'b0;
                    pulse_width_1 <= 32'd0;
                end
            end
        end else begin
            pulse_1_active <= 1'b0;
            pulse_width_1 <= 32'd0;
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_width_2 <= 32'd0;
            pulse_2_active <= 1'b0;
        end else if (sensor_enable && counting_active) begin
            if (sensor_2_falling) begin
                pulse_2_active <= 1'b1;
                pulse_width_2 <= 32'd0;
            end else if (pulse_2_active) begin
                if (sensor_2_rising) begin
                    pulse_2_active <= 1'b0;
                end else if (pulse_width_2 < MAX_PULSE_CYCLES + 1000) begin
                    pulse_width_2 <= pulse_width_2 + 1'b1;
                end else begin
                    pulse_2_active <= 1'b0;
                    pulse_width_2 <= 32'd0;
                end
            end
        end else begin
            pulse_2_active <= 1'b0;
            pulse_width_2 <= 32'd0;
        end
    end
    
    assign pulse_1_valid = (pulse_width_1 >= MIN_PULSE_CYCLES) && (pulse_width_1 <= MAX_PULSE_CYCLES);
    assign pulse_2_valid = (pulse_width_2 >= MIN_PULSE_CYCLES) && (pulse_width_2 <= MAX_PULSE_CYCLES);
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seed_detected_1 <= 1'b0;
            seed_detected_2 <= 1'b0;
            seed_confirmed <= 1'b0;
        end else begin
            if (sensor_1_rising && !pulse_1_active && pulse_1_valid) begin
                seed_detected_1 <= 1'b1;
            end else begin
                seed_detected_1 <= 1'b0;
            end
            if (sensor_2_rising && !pulse_2_active && pulse_2_valid) begin
                seed_detected_2 <= 1'b1;
            end else begin
                seed_detected_2 <= 1'b0;
            end
            if (use_dual_sensors) begin
                seed_confirmed <= seed_detected_1 && seed_detected_2;
            end else begin
                seed_confirmed <= seed_detected_1 || seed_detected_2;
            end
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_seeds <= SEEDS_PER_HOLE_BEANS;
        end else begin
            if (use_custom_target) begin
                target_seeds <= target_seed_count;
            end else if (crop_type == 1'b0) begin
                target_seeds <= SEEDS_PER_HOLE_BEANS;
            end else begin
                target_seeds <= SEEDS_PER_HOLE_MAIZE;
            end
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seed_count <= 8'd0;
            total_seed_count <= 32'd0;
            hole_count <= 16'd0;
            target_reached <= 1'b0;
            counting_active <= 1'b0;
        end else if (clear_count) begin
            seed_count <= 8'd0;
            total_seed_count <= 32'd0;
            hole_count <= 16'd0;
            target_reached <= 1'b0;
            counting_active <= 1'b0;
        end else if (start_counting) begin
            seed_count <= 8'd0;
            target_reached <= 1'b0;
            counting_active <= 1'b1;
        end else if (stop_counting) begin
            if (counting_active) begin
                hole_count <= hole_count + 1'b1;
            end
            counting_active <= 1'b0;
        end else if (counting_active && sensor_enable) begin
            if (seed_confirmed && !target_reached) begin
                seed_count <= seed_count + 1'b1;
                total_seed_count <= total_seed_count + 1'b1;
                if ((seed_count + 1'b1) >= target_seeds) begin
                    target_reached <= 1'b1;
                end
            end
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rate_timer <= 32'd0;
            seeds_in_period <= 8'd0;
            seed_rate <= 16'd0;
            last_seed_time <= 32'd0;
        end else if (clear_count) begin
            rate_timer <= 32'd0;
            seeds_in_period <= 8'd0;
            seed_rate <= 16'd0;
        end else if (sensor_enable) begin
            if (rate_timer >= RATE_SAMPLE_CYCLES) begin
                seed_rate <= {8'd0, seeds_in_period} * 16'd60;
                rate_timer <= 32'd0;
                seeds_in_period <= 8'd0;
            end else begin
                rate_timer <= rate_timer + 1'b1;
            end
            if (seed_confirmed) begin
                seeds_in_period <= seeds_in_period + 1'b1;
                last_seed_time <= rate_timer;
            end
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_error <= 1'b0;
            error_count <= 8'd0;
            sensor_1_status <= 1'b1;
            sensor_2_status <= 1'b1;
            mismatch_detected <= 1'b0;
            mismatch_count <= 8'd0;
            sensor_timeout_1 <= 32'd0;
            sensor_timeout_2 <= 32'd0;
        end else if (clear_count) begin
            sensor_error <= 1'b0;
            error_count <= 8'd0;
            mismatch_detected <= 1'b0;
            mismatch_count <= 8'd0;
        end else if (sensor_enable) begin
            if (use_dual_sensors && (seed_detected_1 != seed_detected_2)) begin
                if (seed_detected_1 || seed_detected_2) begin
                    mismatch_detected <= 1'b1;
                    if (mismatch_count < 8'hFF) begin
                        mismatch_count <= mismatch_count + 1'b1;
                    end
                end
            end else begin
                mismatch_detected <= 1'b0;
            end
            
            if (sensor_1_falling || sensor_1_rising) begin
                sensor_timeout_1 <= 32'd0;
                sensor_1_status <= 1'b1;
            end else if (sensor_timeout_1 < SENSOR_TIMEOUT_CYCLES) begin
                sensor_timeout_1 <= sensor_timeout_1 + 1'b1;
            end else begin
                sensor_1_status <= 1'b0;
            end
            
            if (sensor_2_falling || sensor_2_rising) begin
                sensor_timeout_2 <= 32'd0;
                sensor_2_status <= 1'b1;
            end else if (sensor_timeout_2 < SENSOR_TIMEOUT_CYCLES) begin
                sensor_timeout_2 <= sensor_timeout_2 + 1'b1;
            end else begin
                sensor_2_status <= 1'b0;
            end
            
            sensor_error <= !sensor_1_status || (use_dual_sensors && !sensor_2_status);
            
            if (pulse_1_active && (pulse_width_1 > MAX_PULSE_CYCLES)) begin
                if (error_count < 8'hFF) begin
                    error_count <= error_count + 1'b1;
                end
            end
        end
    end
endmodule

