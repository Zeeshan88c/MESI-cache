// Struct for inputs to Cache_Controller
typedef struct{
    logic read;
    logic write;
    logic [31:0] pr_addr;   // processor address
    logic [31:0] pr_data;   // processor data
    logic c_flush;          // flush signal
    logic bs_req;           // Bus request signal
    logic [1:0] cache_upd_state_core; // Cache state (M,E,S,I)
    logic CCU_ready;        // CCU ready signal
    logic Core_send;        // Core send signal
    logic [31:0] snoop_address; // Snoop address      
    logic bs_req_data;      // Bus request data signal
    logic proc_gnt;         // Processor grant signal
    logic snoop_gnt;        // Snoop grant signal
    logic bs_signal;        // Bus signal
    logic write_done;
} Cache_Controller_Inputs_3;

// Struct for outputs from Cache_Controller
typedef struct{
    logic hit;
    logic miss;
    logic [31:0] data_out_pr;
    logic bs_resp;
    logic [127:0] cache_data;
    logic core_valid;
    logic [1:0] cache_state_core;
    logic [31:0] snoop_data;
    logic Cache_Ready;
    logic Request;
    logic stall;
    logic [31:0] addr;
    logic [31:0] data;
    logic rd;
    logic wr;
} Cache_Controller_Outputs_3;
 
 module Cache_Controller_3 (
    // Inputs
    input logic clk,
    input logic rst,
    input Cache_Controller_Inputs_3 inputs3,  // The struct for all input signals
    input logic [31:0] data_out_CCU4,
    input logic L1_4_CCU,
    
    // outputs3
    output Cache_Controller_Outputs_3 outputs3 // The struct for all output signals
);

// Struct for internal signals
typedef struct{
    logic [19:0] p_tag;            // Processor tag
    logic [7:0] p_index;           // Processor index
    logic [3:0] p_offset;          // Processor offset

    logic [19:0] snoop_tag;        // Snoop tag
    logic [7:0] snoop_index;       // Snoop index
    logic [3:0] snoop_offset;      // Snoop offset

    logic [148:0] cache [0 : 255]; // Cache array (256 entries, each 149 bits wide)
    logic [19:0] cache_tag;        // Cache tag
    logic cache_valid;             // Cache valid flag
    logic [127:0] cache_data_out;  // Cache data output
    logic send;                    // Send signal
    logic request_issued;          // Request issued flag
    int i;                         // Loop counter
    int j;
    logic [31:0] prev_pr_addr;
    logic [31:0] prev_pr_data;
    logic snooping;                // Snoop signal
    logic update;                  // Update flag
    logic cache_hit;               // Cache hit signal
    logic cache_miss;              // Cache miss signal
    logic write_req;
    logic write_hit_case;          // Write hit case signal
    logic write_done;              // Write done signal
    logic no_write;
    logic temp;
    logic [3:0] write_counter;
    logic stop;
} Cache_Internal_Signals;

// States 
typedef enum logic [2:0] {IDLE, PROCESS_REQ, SEND_TO_CCU, FLUSH, RECEIVE_FROM_CCU, STALL} state_t;
state_t current_state, next_state;

typedef enum logic [1:0] {M = 2'b00, E = 2'b01, S = 2'b10, I = 2'b11} mesi_t;
mesi_t cache_mesi_state_temp [0:255];
mesi_t cache_mesi_state [0:255];
   
reg write_done_latched;

Cache_Internal_Signals int_signals3; // Instance of internal signals struct

// Sequential block for state and cache data updates
always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        current_state <= IDLE;
      for(int_signals3.j = 0; int_signals3.j < 256; int_signals3.j++) begin
        cache_mesi_state_temp[int_signals3.j] <= I; // invalid
        end
    end else begin
        current_state <= next_state;
    end
end

always @(*) begin
    if (inputs3.Core_send) begin
        int_signals3.p_tag = inputs3.pr_addr[31:12];  
        int_signals3.p_index = inputs3.pr_addr[11:4];  
        int_signals3.p_offset = inputs3.pr_addr[3:0];  
    end

    if (int_signals3.send) begin
        outputs3.cache_data = int_signals3.cache_data_out;
    end
   
   if (inputs3.proc_gnt && int_signals3.cache_miss) begin
        outputs3.Request = 1'b0;        
        int_signals3.request_issued = 1'b1; 
    end else if (inputs3.proc_gnt && int_signals3.cache_hit) begin
        int_signals3.write_hit_case = 1'b1;
        outputs3.Request = 1'b0;
    end else if (inputs3.proc_gnt)begin
        outputs3.Request = 1'b0;
    end

    if (inputs3.snoop_gnt) begin
        outputs3.bs_resp = 1'b0;
        outputs3.Request = 1'b0;
    end

    if (rst) begin
        for (int_signals3.i = 0; int_signals3.i < 256; int_signals3.i++) begin
            cache_mesi_state[int_signals3.i] = cache_mesi_state_temp[int_signals3.i];
        end
      int_signals3.write_counter = 0;
      int_signals3.temp = 1'b0;
    end
    if(inputs3.write && int_signals3.cache_hit)begin
        int_signals3.write_req = 1'b1;
    end
 
  if (inputs3.write_done && !write_done_latched) begin
        int_signals3.write_counter = int_signals3.write_counter - 1;
        write_done_latched = 1'b1;  // Latch the flag to prevent further decrements
    end
  if (!inputs3.write_done) begin
        write_done_latched = 1'b0;  // Reset the flag when `write_done` goes low
    end
  
  if(int_signals3.temp && !L1_4_CCU )begin
     int_signals3.write_req = 1'b0;
    int_signals3.temp = 1'b0;
    //int_signals3.write_counter = int_signals3.write_counter-1;
  end
  
  if(L1_4_CCU && int_signals3.write_counter != 4'b0000)begin
       outputs3.addr = int_signals3.prev_pr_addr;
       outputs3.data = int_signals3.prev_pr_data;
       outputs3.wr = 1'b1;
       outputs3.hit = 1'b1;
       outputs3.rd = 1'b0;
       outputs3.miss = 1'b0;
       outputs3.bs_resp = 1'b0; 
       //int_signals3.write_req = 1'b0;  
       int_signals3.temp = 1'b1;
       outputs3.cache_state_core = cache_mesi_state[int_signals3.p_index];
    end else if (L1_4_CCU) begin
        outputs3.addr = inputs3.pr_addr;
        outputs3.data = inputs3.pr_data;
        outputs3.wr = inputs3.write;
        outputs3.rd = inputs3.read;
        outputs3.hit = 1'b0;
        outputs3.miss = int_signals3.cache_miss;
        outputs3.bs_resp = 1'b0;
        outputs3.cache_state_core = cache_mesi_state[int_signals3.p_index];
    end 


    case (current_state)
        IDLE: begin
            int_signals3.update = 1'b0;
            int_signals3.send = 1'b0;
           // outputs3.Request = 1'b0;
            int_signals3.request_issued = 1'b0; 
            int_signals3.write_done = 1'b0;
            int_signals3.write_hit_case = 1'b0;
            int_signals3.stop = 1'b0;
            outputs3.stall = 1'b0;
            int_signals3.no_write = 1'b0;
            if (inputs3.bs_req || inputs3.bs_req_data) begin
                next_state = SEND_TO_CCU;
                int_signals3.snoop_tag = inputs3.snoop_address[31:12];  // Tag is 20 bits
                int_signals3.snoop_index = inputs3.snoop_address[11:4];  // Index is 8 bits
                int_signals3.snoop_offset = inputs3.snoop_address[3:0]; 
                int_signals3.cache_data_out = int_signals3.cache[int_signals3.snoop_index][127:0];  
                int_signals3.cache_tag      = int_signals3.cache[int_signals3.snoop_index][148:129];
                int_signals3.cache_valid    = int_signals3.cache[int_signals3.snoop_index][128];
                outputs3.stall = 1'b1;
            end else if (inputs3.Core_send && (inputs3.read || inputs3.write)) begin
                next_state = PROCESS_REQ;
                //int_signals3.cache_valid = 1'b0;
                int_signals3.cache_hit   = 1'b0;
                int_signals3.cache_miss  = 1'b0; 
                int_signals3.cache_data_out = int_signals3.cache[int_signals3.p_index][127:0];
                int_signals3.cache_tag      = int_signals3.cache[int_signals3.p_index][148:129];
                int_signals3.cache_valid    = int_signals3.cache[int_signals3.p_index][128];
                 outputs3.stall = 1'b1;
                outputs3.Cache_Ready = 1'b0;
            end else if (inputs3.Core_send && inputs3.c_flush) begin
                next_state = FLUSH;
                outputs3.stall = 1'b1;
                outputs3.Cache_Ready = 1'b0;
            end else begin
                next_state = IDLE;
            end
        end
    PROCESS_REQ: begin
          
		  int_signals3.cache_hit  = 1'b0;
          int_signals3.cache_miss = 1'b0;
    
			// Case 1: Tag matches and data is valid (Cache hit)
		  if (int_signals3.cache_tag == int_signals3.p_tag && int_signals3.cache_valid) begin
			int_signals3.cache_hit = 1'b1;
			if (inputs3.read) begin
			  case(int_signals3.p_offset)
				4'b0000: outputs3.data_out_pr = int_signals3.cache_data_out[31:0];
				4'b0100: outputs3.data_out_pr = int_signals3.cache_data_out[63:32];
				4'b1000: outputs3.data_out_pr = int_signals3.cache_data_out[95:64];
				4'b1100: outputs3.data_out_pr = int_signals3.cache_data_out[127:96];
			  endcase
              if(inputs3.bs_req || inputs3.bs_req_data) begin
				   next_state = SEND_TO_CCU;
			  end else begin
				next_state = STALL;
			  end
              
			end else if (inputs3.write) begin
			  case(int_signals3.p_offset)
				4'b0000: int_signals3.cache_data_out[31:0] = inputs3.pr_data;
				4'b0100: int_signals3.cache_data_out[63:32] = inputs3.pr_data;
				4'b1000: int_signals3.cache_data_out[95:64] = inputs3.pr_data;
				4'b1100: int_signals3.cache_data_out[127:96] = inputs3.pr_data;
			  endcase
			  int_signals3.send = 1'b1;
			  int_signals3.update = 1'b1;
			  int_signals3.cache_tag   = int_signals3.p_tag;
			  cache_mesi_state[int_signals3.p_index] = M;  // Update MESI state
              int_signals3.prev_pr_addr = inputs3.pr_addr;
              int_signals3.prev_pr_data = inputs3.pr_data;
              if(!int_signals3.stop)begin
                 int_signals3.write_counter = int_signals3.write_counter+1;
                 int_signals3.stop = 1'b1;
              end
              if(~int_signals3.write_hit_case)begin
				    outputs3.Request = 1'b1;
			  end
			  if(L1_4_CCU || int_signals3.write_done)begin
                
			      next_state  = STALL;      // Move to CCU state
              end else if(inputs3.bs_req || inputs3.bs_req_data) begin
			      next_state = IDLE;
			 
              end else begin
			      next_state = PROCESS_REQ;
			  end
			end

		  end else if (int_signals3.cache_tag == int_signals3.p_tag && !int_signals3.cache_valid) begin
			  if (inputs3.read) begin
				int_signals3.cache_miss = 1'b1;
				int_signals3.cache_tag  = int_signals3.p_tag;   // Keep the tag
				next_state = RECEIVE_FROM_CCU;
			  end
			  else if (inputs3.write) begin
				case(int_signals3.p_offset)
				  4'b0000: int_signals3.cache_data_out[31:0] = inputs3.pr_data;
				  4'b0100: int_signals3.cache_data_out[63:32] = inputs3.pr_data;
				  4'b1000: int_signals3.cache_data_out[95:64] = inputs3.pr_data;
				  4'b1100: int_signals3.cache_data_out[127:96] = inputs3.pr_data;
				endcase
				int_signals3.cache_valid = 1'b1;
				int_signals3.send = 1'b1;
				int_signals3.update = 1'b1;
				int_signals3.cache_tag   = int_signals3.p_tag;
				cache_mesi_state[int_signals3.p_index] = M;  // Update MESI state
				int_signals3.cache_hit = 1'b1;               // Consider this as a hit after writing
                int_signals3.prev_pr_addr = inputs3.pr_addr;
              int_signals3.prev_pr_data = inputs3.pr_data;
               if(!int_signals3.stop)begin
                 int_signals3.write_counter = int_signals3.write_counter+1;
                 int_signals3.stop = 1'b1;
              end
				if(~int_signals3.write_hit_case)begin
				    outputs3.Request = 1'b1;
				end
				
                
			  if(L1_4_CCU || int_signals3.write_done)begin
			      next_state  = STALL;      // Move to CCU state
              end else if(inputs3.bs_req || inputs3.bs_req_data) begin
                next_state = SEND_TO_CCU;
			  end else begin
			      next_state = PROCESS_REQ;
			  end
			  end
		  end else begin
				int_signals3.cache_miss = 1'b1;
				int_signals3.cache_valid = 1'b0;
				next_state = RECEIVE_FROM_CCU;  // Move to receive state
		  end
		end

	    SEND_TO_CCU: begin
           int_signals3.send = 1'b0;
          int_signals3.snoop_tag = inputs3.snoop_address[31:12];  // Tag is 20 bits
                int_signals3.snoop_index = inputs3.snoop_address[11:4];  // Index is 8 bits
                int_signals3.snoop_offset = inputs3.snoop_address[3:0]; 
                int_signals3.cache_data_out = int_signals3.cache[int_signals3.snoop_index][127:0];  
                int_signals3.cache_tag      = int_signals3.cache[int_signals3.snoop_index][148:129];
                int_signals3.cache_valid    = int_signals3.cache[int_signals3.snoop_index][128]; 
          
		   if (inputs3.bs_req) begin
			  if (int_signals3.cache_tag == int_signals3.snoop_tag && int_signals3.cache_valid) begin
				case(int_signals3.p_offset) 
					4'b0000: outputs3.snoop_data = int_signals3.cache_data_out[31:0];
					4'b0100: outputs3.snoop_data = int_signals3.cache_data_out[63:32];
					4'b1000: outputs3.snoop_data = int_signals3.cache_data_out[95:64];
					4'b1100: outputs3.snoop_data = int_signals3.cache_data_out[127:96];
				endcase
				outputs3.bs_resp = 1'b1;
				cache_mesi_state[int_signals3.p_index] = mesi_t'(inputs3.cache_upd_state_core);
				if(int_signals3.snooping) begin
				   next_state = RECEIVE_FROM_CCU;
				end else begin
				   next_state = STALL;
                   int_signals3.no_write = 1'b1;
				end
				outputs3.core_valid = 1'b1;
			  end else begin
				outputs3.bs_resp = 1'b0;
				
				if(int_signals3.snooping) begin
				   next_state = RECEIVE_FROM_CCU;
				end else begin
				   next_state = STALL;
                   int_signals3.no_write = 1'b1;
				end
			  end
			end else if (inputs3.bs_req_data) begin
			  if (int_signals3.cache_tag == int_signals3.snoop_tag && int_signals3.cache_valid) begin
				outputs3.bs_resp = 1'b1;
				if(int_signals3.snooping) begin
				   next_state = RECEIVE_FROM_CCU;
				end else begin
				   next_state = STALL;
                  int_signals3.no_write = 1'b1;
				end
				cache_mesi_state[int_signals3.p_index] = mesi_t'(inputs3.cache_upd_state_core);
				outputs3.core_valid = 1'b1;
			  end else begin
                if(int_signals3.snooping) begin
				   next_state = RECEIVE_FROM_CCU;
				end else begin
				   next_state = STALL;
                end
                 outputs3.bs_resp = 1'b0;
              end
			end else begin
              if(!inputs3.bs_signal && int_signals3.snooping)begin
                next_state = RECEIVE_FROM_CCU;
              end  else if(!inputs3.bs_req_data || !inputs3.bs_req) begin
                if(int_signals3.snooping) begin
				   next_state = RECEIVE_FROM_CCU;
				 end else begin
				   next_state = STALL;
                 end
              end  else begin
				next_state = SEND_TO_CCU;
              end
			end
		end


		RECEIVE_FROM_CCU: begin 
		  int_signals3.cache_tag   = int_signals3.p_tag;
          int_signals3.no_write = 1'b0;
		  if (!int_signals3.request_issued) begin
              outputs3.Request = 1'b1;  // Assert Request once
          end
         
          if(inputs3.bs_signal)begin
             next_state = SEND_TO_CCU;
             int_signals3.snooping = 1'b1;
          end else if (inputs3.CCU_ready) begin
		    if(inputs3.read)begin
			 outputs3.data_out_pr = data_out_CCU4;
		    end
			cache_mesi_state[int_signals3.p_index] = mesi_t'(inputs3.cache_upd_state_core);
			if(inputs3.write)begin
			  next_state = PROCESS_REQ;
			  int_signals3.write_done= 1'b1;
			end else begin
			  next_state = STALL;
			end
            case(int_signals3.p_offset)
              4'b0000:int_signals3.cache_data_out[31:0] = data_out_CCU4;
              4'b0100:int_signals3.cache_data_out[63:32] = data_out_CCU4;
              4'b1000:int_signals3.cache_data_out[95:64] = data_out_CCU4;
              4'b1100:int_signals3.cache_data_out[127:96] = data_out_CCU4;
            endcase
			int_signals3.cache_miss = 1'b0;
			int_signals3.cache_valid = 1'b1;
		  end
		  else begin
		    next_state = RECEIVE_FROM_CCU;
		  end
		end
	  	
	  	
	  	FLUSH: begin
            if(inputs3.c_flush) begin
              int_signals3.cache_valid = 1'b0;
              int_signals3.p_index = int_signals3.p_index + 1;    
              if (int_signals3.p_index == 8'd255) begin 
                int_signals3.p_index <= 0; 
                next_state = STALL; 
              end else begin
                next_state = FLUSH; 
              end
            end
		end

		STALL: begin
			if(outputs3.stall == 1'b1)begin
				outputs3.stall = 1'b0;
				outputs3.Cache_Ready = 1'b1;
              
              /*if(inputs3.bs_req || inputs3.bs_req_data)begin
				next_state = SEND_TO_CCU;
              end else begin
                next_state = IDLE;
              end*/
                next_state = IDLE;
				//outputs3.Request = 1'b0;
				int_signals3.request_issued = 1'b0;
				int_signals3.write_hit_case = 1'b0;
				int_signals3.snooping = 1'b0;
			    cache_mesi_state[int_signals3.p_index] = mesi_t'(inputs3.cache_upd_state_core);
              if(!int_signals3.no_write)begin
			    int_signals3.cache[int_signals3.p_index][127:0] = int_signals3.cache_data_out;
                int_signals3.cache[int_signals3.p_index][148:129] = int_signals3.cache_tag;
                int_signals3.cache[int_signals3.p_index][128] = int_signals3.cache_valid;
              end
			end
		end

    endcase
end
endmodule
