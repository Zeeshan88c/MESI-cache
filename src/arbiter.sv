typedef struct packed {
    logic [3:0] Com_Bus_Req_proc;   // Processor requests
    logic [3:0] Com_Bus_Req_snoop;    // Snoop requests
} Arbiter_Input_t;

// Struct for Outputs
typedef struct packed {
    logic [3:0] Com_Bus_Gnt_proc;    // Processor grant signals
    logic [3:0] Com_Bus_Gnt_snoop;   // Snoop grant signals
    logic wr_en;
    logic [3:0] fifo_data_in;
    logic [3:0] fifo_data_snoop;
    logic [3:0] proc_gnt;
    logic [3:0] snoop_gnt;
    logic snoop_active;
    logic no_snoop;
} Arbiter_Output_t;

module arbiter (
    input logic rst,
    input logic clk,
    input Arbiter_Input_t arbiter_input,   // Input struct
    output Arbiter_Output_t arbiter_output  // Output struct
);

    reg [1:0] state;
    reg [3:0] prev_fifo_data_in; // Store previous fifo_data_in value
    reg fifo_hold;              // Flag to track if FIFO data should be held
    reg next_fifo_hold;         // Next value for fifo_hold computation
    integer i; 

    // State encoding
    parameter IDLE      = 2'b00;
    parameter PROC_REQ  = 2'b01;
    parameter SNOOP_REQ = 2'b10;

    // Sequential block: state transitions and registered signals
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state                     <= IDLE;
            arbiter_output.wr_en      <= 1'b0;
            arbiter_output.fifo_data_snoop <= 4'b0000;
            arbiter_output.fifo_data_in    <= 4'b0000;
            arbiter_output.snoop_active<= 1'b0;
            prev_fifo_data_in         <= 4'b0000;
            fifo_hold                 <= 1'b0;
        end else begin
            // Clear snoop_active when no snoop request is present
            if (~|arbiter_input.Com_Bus_Req_snoop)
                arbiter_output.snoop_active <= 1'b0;
            case (state)
                IDLE: begin
                    arbiter_output.proc_gnt  <= 4'b0000;
                    arbiter_output.snoop_gnt <= 4'b0000;
                    if (|arbiter_input.Com_Bus_Req_snoop) begin  
                        arbiter_output.snoop_active <= 1'b1;
                        state <= SNOOP_REQ;
                    end else if (|arbiter_input.Com_Bus_Req_proc) begin
                        state <= PROC_REQ;
                    end
                end
                PROC_REQ: begin
                    if (~|arbiter_input.Com_Bus_Req_proc) begin 
                        state <= IDLE;
                    end else begin
                        arbiter_output.proc_gnt <= arbiter_output.Com_Bus_Gnt_proc;
                        prev_fifo_data_in      <= arbiter_output.fifo_data_in;
                    end
                end
                SNOOP_REQ: begin
                    if (~|arbiter_input.Com_Bus_Req_snoop) begin
                        state <= IDLE;
                    end else begin
                        arbiter_output.snoop_gnt <= arbiter_output.Com_Bus_Gnt_snoop;
                    end
                end
            endcase
            // Update fifo_hold with the combinationally computed next_fifo_hold
            fifo_hold <= next_fifo_hold;
        end
    end

    // Combinational block for grant signal computation using explicit priority encoding
    always @(*) begin
        arbiter_output.Com_Bus_Gnt_proc = 4'b0000;
        arbiter_output.Com_Bus_Gnt_snoop = 4'b0000;
        if (state == PROC_REQ) begin
            if (arbiter_input.Com_Bus_Req_proc[0])
                arbiter_output.Com_Bus_Gnt_proc = 4'b0001;
            else if (arbiter_input.Com_Bus_Req_proc[1])
                arbiter_output.Com_Bus_Gnt_proc = 4'b0010;
            else if (arbiter_input.Com_Bus_Req_proc[2])
                arbiter_output.Com_Bus_Gnt_proc = 4'b0100;
            else if (arbiter_input.Com_Bus_Req_proc[3])
                arbiter_output.Com_Bus_Gnt_proc = 4'b1000;
        end else if (state == SNOOP_REQ) begin
            if (arbiter_input.Com_Bus_Req_snoop[0])
                arbiter_output.Com_Bus_Gnt_snoop = 4'b0001;
            else if (arbiter_input.Com_Bus_Req_snoop[1])
                arbiter_output.Com_Bus_Gnt_snoop = 4'b0010;
            else if (arbiter_input.Com_Bus_Req_snoop[2])
                arbiter_output.Com_Bus_Gnt_snoop = 4'b0100;
            else if (arbiter_input.Com_Bus_Req_snoop[3])
                arbiter_output.Com_Bus_Gnt_snoop = 4'b1000;
        end
    end

    // Combinational block for FIFO data and write enable generation with FIFO hold mechanism
    always @(*) begin
        // Default assignments
        arbiter_output.wr_en       = 1'b0;
        arbiter_output.fifo_data_in = 4'b0000;
        arbiter_output.fifo_data_snoop = 4'b0000;
        next_fifo_hold            = 1'b0;
        if (state == PROC_REQ) begin
            if (arbiter_input.Com_Bus_Req_proc[0]) begin
                arbiter_output.fifo_data_in = {2'b01, 2'b00};
                if (({2'b01, 2'b00} != prev_fifo_data_in) || fifo_hold)
                    arbiter_output.wr_en = 1'b1;
                else
                    next_fifo_hold = 1'b1;
            end else if (arbiter_input.Com_Bus_Req_proc[1]) begin
                arbiter_output.fifo_data_in = {2'b01, 2'b01};
                if (({2'b01, 2'b01} != prev_fifo_data_in) || fifo_hold)
                    arbiter_output.wr_en = 1'b1;
                else
                    next_fifo_hold = 1'b1;
            end else if (arbiter_input.Com_Bus_Req_proc[2]) begin
                arbiter_output.fifo_data_in = {2'b01, 2'b10};
                if (({2'b01, 2'b10} != prev_fifo_data_in) || fifo_hold)
                    arbiter_output.wr_en = 1'b1;
                else
                    next_fifo_hold = 1'b1;
            end else if (arbiter_input.Com_Bus_Req_proc[3]) begin
                arbiter_output.fifo_data_in = {2'b01, 2'b11};
                if (({2'b01, 2'b11} != prev_fifo_data_in) || fifo_hold)
                    arbiter_output.wr_en = 1'b1;
                else
                    next_fifo_hold = 1'b1;
            end
        end else if (state == SNOOP_REQ) begin
            // Use a case statement to map the one-hot grant to FIFO data
            case (arbiter_output.Com_Bus_Gnt_snoop)
                4'b0001: arbiter_output.fifo_data_snoop = {2'b10, 2'b00};
                4'b0010: arbiter_output.fifo_data_snoop = {2'b10, 2'b01};
                4'b0100: arbiter_output.fifo_data_snoop = {2'b10, 2'b10};
                4'b1000: arbiter_output.fifo_data_snoop = {2'b10, 2'b11};
                default: arbiter_output.fifo_data_snoop = 4'b0000;
            endcase
        end
    end

    // Combinational block for no_snoop signal generation
    always_comb begin
        if (arbiter_output.snoop_active)
            arbiter_output.no_snoop = 1'b0;
        else if (~|arbiter_input.Com_Bus_Req_snoop || (^arbiter_input.Com_Bus_Req_snoop === 1'bx))
            arbiter_output.no_snoop = 1'b1;
        else
            arbiter_output.no_snoop = 1'b0;
    end

endmodule
