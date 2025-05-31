// Struct for Inputs
typedef struct packed {
    logic write_signal;
    logic [31:0] addr;             // Address from CCU to read/write
    logic [127:0] write_data;      // Data to be written to L2 from CCU
    logic l2_read_req;
    logic l2_write_req;
    logic [127:0] mem_read_data;   // Read data from memory
    logic mem_ready;               // Signal that memory operation is complete
    logic written;
} L2_Input_t;

// Struct for Outputs
typedef struct packed {
    logic [127:0] read_data;       // Data read from L2 to be sent to CCU
    logic ready;                   // Ready signal to CCU
    logic l2_miss;
    logic l2_hit;
    logic mem_read_req;
    logic mem_write_req;
    logic [31:0] mem_addr;         // Address to access in main memory
    logic [127:0] mem_write_data;  // Write data to memory
} L2_Output_t;

// Struct for Internal Signals
typedef struct packed {
    logic [18:0] tag;
    logic [8:0] index;
    logic [3:0] offset;
    logic [18:0] cache_tag;
    logic cache_valid;
    logic cache_dirty;
    logic [127:0] cache_data;
    logic next_ready;
    logic next_l2_hit;
    logic next_l2_miss;
    logic next_mem_read_req;
    logic next_mem_write_req;
    logic [31:0] next_mem_addr;
    logic [127:0] next_read_data;
    logic [127:0] next_mem_write_data;
} L2_Internal_t;

module L2_cache (
    input  logic clk,
    input logic rst,
    input L2_Input_t l2_input,      // Using the input struct
    output L2_Output_t l2_output   // Using the output struct
);
    logic [148:0] cache [0:511]; // 511 sets, direct-mapped (1 line per set), 150 bits per line
    // Internal Signals
    L2_Internal_t l2_internal;
    //States Encoding
    typedef enum logic [2:0] {
        IDLE = 3'b000,
        PROCESS_REQ = 3'b001,
        READ_FROM_L2 = 3'b010,
        WRITE_TO_L2 = 3'b011,
        MEM_FETCH = 3'b100,        // Fetching data from main memory (L2 miss)
        MEM_WRITEBACK = 3'b101     // Writing dirty data back to memory
    } state_t;

    state_t current_state, next_state;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            current_state <= IDLE;
            l2_output.read_data <= 128'b0;
            l2_output.ready <= 1'b0;
            l2_output.l2_miss <= 1'b0;
            l2_output.l2_hit <= 1'b0;
            l2_output.mem_read_req <= 1'b0;
            l2_output.mem_write_req <= 1'b0;
            l2_output.mem_addr <= 32'b0;
            l2_output.mem_write_data <= 128'b0;
        end else begin
            current_state <= next_state;
            l2_output.read_data <= l2_internal.next_read_data;
            l2_output.ready <= l2_internal.next_ready;
            l2_output.l2_miss <= l2_internal.next_l2_miss;
            l2_output.l2_hit <= l2_internal.next_l2_hit;
            l2_output.mem_read_req <= l2_internal.next_mem_read_req;
            l2_output.mem_write_req <= l2_internal.next_mem_write_req;
            l2_output.mem_addr <= l2_internal.next_mem_addr;
            l2_output.mem_write_data <= l2_internal.next_mem_write_data;
        end
    end

    always_comb begin
        l2_internal.tag     =  l2_input.addr[31:13]; // Changed to 19 bits
        l2_internal.index   =  l2_input.addr[12:4];  // Changed to 9 bits
        l2_internal.offset  =  l2_input.addr[3:0];       
    end
  

    always_comb begin
        //l2_internal.next_ready = 1'b0;
        l2_internal.next_l2_miss = 1'b0;
        l2_internal.next_l2_hit = 1'b0;
        l2_internal.next_mem_read_req = 1'b0;
        l2_internal.next_mem_write_req = 1'b0;
        l2_internal.next_mem_addr = l2_output.mem_addr;
        l2_internal.next_mem_write_data = l2_output.mem_write_data;
         
        if(l2_input.written)begin
         l2_internal.next_ready = 1'b0;
        end
      
        case (current_state)
            IDLE: begin
                if (l2_input.l2_read_req || l2_input.l2_write_req) begin
                    next_state = PROCESS_REQ;
                    l2_internal.next_ready = 1'b0;
                    l2_internal.cache_data = cache[l2_internal.index][127:0];
                    l2_internal.cache_dirty = cache[l2_internal.index][128];
                    l2_internal.cache_valid = cache[l2_internal.index][129];
                    l2_internal.cache_tag = cache[l2_internal.index][148:130]; // Changed to 19 bits
                end else begin
                    next_state = IDLE;
                end
            end

            PROCESS_REQ: begin
                
                if (l2_internal.cache_valid && (l2_internal.cache_tag == l2_internal.tag)) begin
                    // Cache hit
                    l2_internal.next_l2_hit = 1'b1;
                    l2_internal.next_l2_miss = 1'b0;

                    if (l2_input.l2_read_req) begin
                        next_state = READ_FROM_L2;
                    end else if (l2_input.l2_write_req) begin
                        next_state = WRITE_TO_L2;
                    end
                end else begin
                    // Cache miss
                    l2_internal.next_l2_miss = 1'b1;
                    l2_internal.next_l2_hit = 1'b0;
                    if (l2_internal.cache_valid && l2_internal.cache_dirty) begin
                        // Dirty line, writeback to main memory
                        next_state = MEM_WRITEBACK;
                        l2_internal.next_mem_addr = {l2_internal.cache_tag, l2_internal.index, 4'b0};
                        l2_internal.next_mem_write_data = l2_internal.cache_data;
                        l2_internal.next_mem_write_req = 1'b1;
                    end else begin
                        // Empty line, fetch from main memory
                        next_state = MEM_FETCH;
                        l2_internal.next_mem_addr = l2_input.addr;
                        l2_internal.next_mem_read_req = 1'b1;
                    end
                end
            end

            READ_FROM_L2: begin
                if(l2_input.l2_read_req)begin
                  l2_internal.next_read_data = l2_internal.cache_data;
                  l2_internal.next_ready = 1'b1;
                end
                next_state = IDLE;
            end

            WRITE_TO_L2: begin
                if(l2_input.l2_write_req)begin
                  l2_internal.next_ready = 1'b1;
                end
                l2_internal.cache_dirty = 1'b1;  // Dirty
                l2_internal.cache_valid = 1'b1;  // Valid
                l2_internal.cache_tag = l2_internal.tag;
                if(l2_input.write_signal)begin
                  next_state = IDLE;
                  l2_internal.cache_data = l2_input.write_data;
                end else begin
                  next_state = WRITE_TO_L2;
                end
            end

            MEM_FETCH: begin
                if (l2_input.mem_ready) begin
                    l2_internal.cache_data = l2_input.mem_read_data;
                    l2_internal.cache_dirty = 1'b0;  // Not dirty
                    l2_internal.cache_valid = 1'b1;  // Valid
                    l2_internal.cache_tag = l2_internal.tag;
                    next_state = READ_FROM_L2;
                    l2_internal.next_mem_read_req = 1'b0;
                end else begin
                    l2_internal.next_mem_read_req = 1'b1;
                end
            end

            MEM_WRITEBACK: begin
                if (l2_input.mem_ready) begin
                    l2_internal.next_mem_addr = {l2_internal.cache_tag, l2_internal.index, l2_internal.offset};
                    l2_internal.next_mem_write_data = l2_internal.cache_data;
                    l2_internal.cache_dirty = 1'b0; // Line is clean now
                    next_state = WRITE_TO_L2;
                    l2_internal.next_mem_write_req = 1'b0;
                end else begin
                    l2_internal.next_mem_write_req = 1'b1;
                end
            end
        endcase
       cache[l2_internal.index][127:0] = l2_internal.cache_data;
       cache[l2_internal.index][148:130] = l2_internal.cache_tag;
       cache[l2_internal.index][129] = l2_internal.cache_valid ;
       cache[l2_internal.index][128] = l2_internal.cache_dirty;
    end
endmodule