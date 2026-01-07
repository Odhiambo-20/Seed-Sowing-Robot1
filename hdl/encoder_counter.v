// ============================================================================
// MODULE 3: ENCODER COUNTER
// ============================================================================

module encoder_counter #(
    parameter COUNTER_WIDTH = 32,
    parameter VELOCITY_WIDTH = 16,
    parameter FILTER_STAGES = 3,
    parameter VELOCITY_SAMPLE_PERIOD = 1000
)(
    input wire clk,
    input wire rst_n,
    input wire enc_a,
    input wire enc_b,
    input wire enc_z,
    input wire enable,
    input wire clear_count,
    input wire set_count,
    input wire [COUNTER_WIDTH-1:0] count_value,
    input wire clear_velocity,
    output reg [COUNTER_WIDTH-1:0] position,
    output reg [VELOCITY_WIDTH-1:0] velocity,
    output reg direction,
    output reg index_detected,
    output reg overflow,
    output reg underflow,
    output reg [1:0] error_flags
);

    reg [FILTER_STAGES-1:0] enc_a_filter;
    reg [FILTER_STAGES-1:0] enc_b_filter;
    reg [FILTER_STAGES-1:0] enc_z_filter;
    wire enc_a_filtered;
    wire enc_b_filtered;
    wire enc_z_filtered;
    reg enc_a_prev;
    reg enc_b_prev;
    reg enc_z_prev;
    reg [1:0] encoder_state;
    reg [1:0] encoder_state_prev;
    reg [31:0] velocity_timer;
    reg [COUNTER_WIDTH-1:0] position_prev;
    reg [COUNTER_WIDTH-1:0] position_delta;
    reg [1:0] invalid_transition_count;
    
    localparam [1:0] STATE_00 = 2'b00;
    localparam [1:0] STATE_01 = 2'b01;
    localparam [1:0] STATE_11 = 2'b11;
    localparam [1:0] STATE_10 = 2'b10;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_a_filter <= {FILTER_STAGES{1'b0}};
            enc_b_filter <= {FILTER_STAGES{1'b0}};
            enc_z_filter <= {FILTER_STAGES{1'b0}};
        end else begin
            enc_a_filter <= {enc_a_filter[FILTER_STAGES-2:0], enc_a};
            enc_b_filter <= {enc_b_filter[FILTER_STAGES-2:0], enc_b};
            enc_z_filter <= {enc_z_filter[FILTER_STAGES-2:0], enc_z};
        end
    end
    
    generate
        if (FILTER_STAGES == 3) begin : gen_majority_voting
            assign enc_a_filtered = (enc_a_filter[2] & enc_a_filter[1]) | 
                                   (enc_a_filter[2] & enc_a_filter[0]) | 
                                   (enc_a_filter[1] & enc_a_filter[0]);
            assign enc_b_filtered = (enc_b_filter[2] & enc_b_filter[1]) | 
                                   (enc_b_filter[2] & enc_b_filter[0]) | 
                                   (enc_b_filter[1] & enc_b_filter[0]);
            assign enc_z_filtered = (enc_z_filter[2] & enc_z_filter[1]) | 
                                   (enc_z_filter[2] & enc_z_filter[0]) | 
                                   (enc_z_filter[1] & enc_z_filter[0]);
        end else begin : gen_simple_filter
            assign enc_a_filtered = enc_a_filter[FILTER_STAGES-1];
            assign enc_b_filtered = enc_b_filter[FILTER_STAGES-1];
            assign enc_z_filtered = enc_z_filter[FILTER_STAGES-1];
        end
    endgenerate
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_a_prev <= 1'b0;
            enc_b_prev <= 1'b0;
            enc_z_prev <= 1'b0;
        end else begin
            enc_a_prev <= enc_a_filtered;
            enc_b_prev <= enc_b_filtered;
            enc_z_prev <= enc_z_filtered;
        end
    end
    
    wire enc_z_rising = enc_z_filtered & ~enc_z_prev;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            encoder_state <= STATE_00;
            encoder_state_prev <= STATE_00;
        end else begin
            encoder_state <= {enc_a_filtered, enc_b_filtered};
            encoder_state_prev <= encoder_state;
        end
    end
    
    reg count_up;
    reg count_down;
    
    always @(*) begin
        count_up = 1'b0;
        count_down = 1'b0;
        case (encoder_state_prev)
            STATE_00: begin
                if (encoder_state == STATE_01) count_up = 1'b1;
                else if (encoder_state == STATE_10) count_down = 1'b1;
            end
            STATE_01: begin
                if (encoder_state == STATE_11) count_up = 1'b1;
                else if (encoder_state == STATE_00) count_down = 1'b1;
            end
            STATE_11: begin
                if (encoder_state == STATE_10) count_up = 1'b1;
                else if (encoder_state == STATE_01) count_down = 1'b1;
            end
            STATE_10: begin
                if (encoder_state == STATE_00) count_up = 1'b1;
                else if (encoder_state == STATE_11) count_down = 1'b1;
            end
        endcase
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            position <= {COUNTER_WIDTH{1'b0}};
            direction <= 1'b1;
            overflow <= 1'b0;
            underflow <= 1'b0;
        end else if (clear_count) begin
            position <= {COUNTER_WIDTH{1'b0}};
            overflow <= 1'b0;
            underflow <= 1'b0;
        end else if (set_count) begin
            position <= count_value;
            overflow <= 1'b0;
            underflow <= 1'b0;
        end else if (enable) begin
            if (count_up) begin
                if (position == {COUNTER_WIDTH{1'b1}}) begin
                    overflow <= 1'b1;
                    position <= {COUNTER_WIDTH{1'b0}};
                end else begin
                    position <= position + 1'b1;
                    overflow <= 1'b0;
                end
                direction <= 1'b1;
            end else if (count_down) begin
                if (position == {COUNTER_WIDTH{1'b0}}) begin
                    underflow <= 1'b1;
                    position <= {COUNTER_WIDTH{1'b1}};
                end else begin
                    position <= position - 1'b1;
                    underflow <= 1'b0;
                end
                direction <= 1'b0;
            end
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            index_detected <= 1'b0;
        end else if (clear_count) begin
            index_detected <= 1'b0;
        end else if (enc_z_rising) begin
            index_detected <= 1'b1;
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            velocity_timer <= 32'd0;
            position_prev <= {COUNTER_WIDTH{1'b0}};
            position_delta <= {COUNTER_WIDTH{1'b0}};
            velocity <= {VELOCITY_WIDTH{1'b0}};
        end else if (clear_velocity) begin
            velocity_timer <= 32'd0;
            position_prev <= position;
            velocity <= {VELOCITY_WIDTH{1'b0}};
        end else if (enable) begin
            if (velocity_timer >= VELOCITY_SAMPLE_PERIOD) begin
                if (position >= position_prev) begin
                    position_delta <= position - position_prev;
                end else begin
                    position_delta <= ({COUNTER_WIDTH{1'b1}} - position_prev) + position + 1'b1;
                end
                if (position_delta > {VELOCITY_WIDTH{1'b1}}) begin
                    velocity <= {VELOCITY_WIDTH{1'b1}};
                end else begin
                    velocity <= position_delta[VELOCITY_WIDTH-1:0];
                end
                velocity_timer <= 32'd0;
                position_prev <= position;
            end else begin
                velocity_timer <= velocity_timer + 1'b1;
            end
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            invalid_transition_count <= 2'b00;
            error_flags <= 2'b00;
        end else if (clear_count) begin
            invalid_transition_count <= 2'b00;
            error_flags <= 2'b00;
        end else if (enable) begin
            if (encoder_state != encoder_state_prev) begin
                case ({encoder_state_prev, encoder_state})
                    {STATE_00, STATE_01}, {STATE_01, STATE_00},
                    {STATE_01, STATE_11}, {STATE_11, STATE_01},
                    {STATE_11, STATE_10}, {STATE_10, STATE_11},
                    {STATE_10, STATE_00}, {STATE_00, STATE_10}: begin
                        invalid_transition_count <= 2'b00;
                    end
                    default: begin
                        if (invalid_transition_count < 2'b11) begin
                            invalid_transition_count <= invalid_transition_count + 1'b1;
                        end
                    end
                endcase
            end
            error_flags[0] <= (invalid_transition_count >= 2'b10);
            error_flags[1] <= overflow | underflow;
        end
    end
endmodule
