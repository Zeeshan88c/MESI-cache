`define BUF_WIDTH 8
`define BUF_SIZE (1 << `BUF_WIDTH)

// Struct for Inputs
typedef struct{
    logic [3:0] data_in;             // Data to be written
    logic wr_en;                     // Write enable
    logic rd_en;                     // Read enable for processor requests
    logic rd_en_snoop;               // Snoop read enable
} FIFO_Input;

// Struct for Outputs
typedef struct{
    logic [3:0] buf_out;             // Output buffer
    logic buf_empty;                 // Empty flag
    logic buf_full;                  // Full flag
} FIFO_Output;

// Struct for Internal Signals
typedef struct {
    logic [3:0] buffer [`BUF_SIZE-1:0];  // FIFO buffer storage
    logic [`BUF_WIDTH-1:0] rd_ptr;       // Read pointer
    logic [`BUF_WIDTH-1:0] wr_ptr;       // Write pointer
    logic [`BUF_WIDTH:0] fifo_counter;   // Counter to track buffer size
    logic prev_wr_en;                    // Previous write enable state
} FIFO_Internal_t;

module fifo(
    input logic rst,
    input FIFO_Input fifo_input,      // Input struct
    output FIFO_Output fifo_output    // Output struct
);

   
    FIFO_Internal_t fifo_internal;


    assign fifo_output.buf_full = (fifo_internal.fifo_counter == `BUF_SIZE); 
    assign fifo_output.buf_empty = (fifo_internal.fifo_counter == 0);        

always_ff @(posedge rst or posedge fifo_input.wr_en or posedge fifo_input.rd_en) begin
    if (rst) begin
        fifo_internal.wr_ptr <= 0;
        fifo_internal.rd_ptr <= 0;
        fifo_output.buf_out <= 4'b0000;
        fifo_internal.fifo_counter <= 0;
        fifo_internal.prev_wr_en <= 0;
    end else begin
       
        fifo_internal.prev_wr_en <= fifo_input.wr_en;

      if (fifo_input.wr_en && fifo_input.rd_en) begin
          if (!fifo_output.buf_full && !fifo_output.buf_empty) begin
               
                fifo_internal.buffer[fifo_internal.wr_ptr] <= fifo_input.data_in;
                fifo_internal.wr_ptr <= (fifo_internal.wr_ptr + 1) % `BUF_SIZE;

             
                fifo_output.buf_out <= fifo_internal.buffer[fifo_internal.rd_ptr];
                fifo_internal.rd_ptr <= (fifo_internal.rd_ptr + 1) % `BUF_SIZE; 
                
                fifo_internal.fifo_counter <= fifo_internal.fifo_counter;
          end
        end else if (!fifo_input.wr_en && fifo_internal.prev_wr_en && fifo_input.rd_en) begin
           
            if (!fifo_output.buf_empty) begin
                fifo_output.buf_out <= fifo_internal.buffer[fifo_internal.rd_ptr];
                fifo_internal.rd_ptr <= (fifo_internal.rd_ptr + 1) % `BUF_SIZE;
                fifo_internal.fifo_counter <= fifo_internal.fifo_counter - 1;
            end
        end else begin
          
          if (fifo_input.wr_en &&  !fifo_output.buf_full) begin
                fifo_internal.buffer[fifo_internal.wr_ptr] <= fifo_input.data_in;
                fifo_internal.wr_ptr <= (fifo_internal.wr_ptr + 1) % `BUF_SIZE;
                fifo_internal.fifo_counter <= fifo_internal.fifo_counter + 1;
            end

       
            if (fifo_input.rd_en && !fifo_output.buf_empty) begin
                fifo_output.buf_out <= fifo_internal.buffer[fifo_internal.rd_ptr];
                fifo_internal.rd_ptr <= (fifo_internal.rd_ptr + 1) % `BUF_SIZE;
                fifo_internal.fifo_counter <= fifo_internal.fifo_counter - 1;
            end
        end
      
    end
end

endmodule

