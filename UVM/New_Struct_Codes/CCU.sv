// Struct for CCU inputs
typedef struct{
    logic [3:0] read;
    logic [3:0] write;
    logic [3:0] cache_hit;
    logic [3:0] cache_miss;
    logic [31:0] pr_addr_ccu [3:0];
    logic [31:0] pr_data_ccu [3:0];
    logic ready;
    logic [127:0] l2_data;
    logic [31:0] snoop_data; // Array for snoop data (ccu_input.snoop_data_1, 2, 3)
    logic [31:0] snoop_data_1;
    logic [31:0] snoop_data_2;
    logic [31:0] snoop_data_3;
    logic core1_valid; // Core valid signals (ccu_input.core1_valid, ccu_input.core2_valid, etc.)
    logic core2_valid; 
    logic core3_valid; 
    logic core4_valid; 
    logic [127:0] cache_data1; // Cache data for core 1, 2, 3, 4
    logic [127:0] cache_data2;
    logic [127:0] cache_data3;
    logic [127:0] cache_data4;
    logic [1:0] cache_state_core [3:0]; // Cache states for core 1, 2, 3, 4
    logic [3:0] buf_out; // 4-bit data output
    logic buf_empty;
    logic buf_full;
    logic snoop_active;
    logic no_snoop;
    logic [3:0] fifo_data_snoop;
} CCU_Input;

// Struct for CCU outputs
typedef struct{
    logic bs_req;
    logic [3:0] bs_req_data;
    logic l2_write_req;
    logic read_req;
    logic [127:0] write_data;
    logic [31:0] data_out_CCU [3:0]; // Core 1,2,3,4
    logic [1:0] cache_upd_state_core [3:0]; // Cache update state for core 1,2,3,4
    logic [7:0] CCU_index;
    logic [3:0] CCU_ready;
    logic [3:0] bs_signal;
    logic L1_1_CCU;
    logic L1_2_CCU;
    logic L1_3_CCU;
    logic L1_4_CCU;
    logic [31:0] snoop_address;
    logic rd_en;
    //logic rd_en_snoop;
    logic [31:0] addr_to_send;
    logic write_signal;
    logic written;
    logic [3:0] write_done;
} CCU_Output;

module CCU (
    // INPUTS
    input logic clk,
    input logic rst,
    input CCU_Input ccu_input, // Using the input struct
    // OUTPUTS
    output CCU_Output ccu_output // Using the output struct
);
typedef struct{
    logic [19:0] p_tag;          // Tag portion of the address
    logic [7:0] p_index;         // Index portion of the address
    logic [3:0] p_offset;        // Offset portion of the address
    logic [1:0] cache_state;     // Current state of the cache line
    logic upd_cache;             // Signal to update the cache
    logic [1:0]req_core;        // Requesting core                       /
    logic start;                 // Signal to start a cache operation
    logic bs_resp1;              // Bus response from Core 0
    logic bs_resp2;              // Bus response from Core 1
    logic bs_resp3;              // Bus response from Core 2
    logic bs_resp4;              // Bus response from Core 3
    logic burst_mode;
    logic write_case;            // Signal for write operation case
} Internal_Signals;

Internal_Signals internal_signals_ccu;
int i;

 typedef enum logic [1:0] {M = 2'b00, E = 2'b01, S = 2'b10, I = 2'b11} mesi_t;
    mesi_t core_mesi_state [4];
     mesi_t core_mesi_state_duplicate [4];
  

    typedef enum logic [2:0] {IDLE = 3'b000,FIFO_REQ = 3'b001, PROCESS_REQ = 3'b010, SNOOPING = 3'b011, Read_FROM_L2 = 3'b100, WRITE_TO_L2 = 3'b101} state_t;
    state_t current_state, next_state;

     always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            current_state <= IDLE;
            for (int i = 0; i < 4; i++) begin
                core_mesi_state_duplicate [i] <= I; // Set duplicate to invalid at reset
            end
        end else begin
            current_state <= next_state;
    end
    end

  always @(*) begin

       //Default values for control signals
      internal_signals_ccu.start = 1'b0;
      ccu_output.CCU_ready = 1'b0;

      if(rst)begin
        for (int i = 0; i < 4; i++) begin
            core_mesi_state[i] = core_mesi_state_duplicate[i];
        end
        ccu_output.rd_en = 1'b0;
        ccu_output.bs_signal = 4'b0000;
        ccu_output.write_done = 4'b0000;
      end
        
      if (internal_signals_ccu.start)begin
          case(internal_signals_ccu.req_core)//2'b00
            2'b00:internal_signals_ccu.cache_state = ccu_input.cache_state_core[0];
            2'b01:internal_signals_ccu.cache_state = ccu_input.cache_state_core[1];
            2'b10:internal_signals_ccu.cache_state = ccu_input.cache_state_core[2];
            2'b11:internal_signals_ccu.cache_state = ccu_input.cache_state_core[3];
          endcase

          if(internal_signals_ccu.cache_state == 2'b00)begin
             core_mesi_state[internal_signals_ccu.req_core] = M;
          end
          else if(internal_signals_ccu.cache_state == 2'b01)begin
             core_mesi_state[internal_signals_ccu.req_core] = E;
          end
          else if(internal_signals_ccu.cache_state == 2'b10)begin
             core_mesi_state[internal_signals_ccu.req_core] = S;
          end
          else if(internal_signals_ccu.cache_state == 2'b11)begin
             core_mesi_state[internal_signals_ccu.req_core] = I;
          end

        end
      
      if(ccu_input.ready)begin
        ccu_output.l2_write_req = 1'b0;
      end
   
      if (current_state == PROCESS_REQ) begin
           for (int i = 0; i < 4; i++) begin
             if (i != internal_signals_ccu.req_core) begin
                ccu_output.bs_signal[i] = 1'b1;//1011
              end
           end
        end
       
       
      if(internal_signals_ccu.upd_cache) begin
        ccu_output.cache_upd_state_core[0] = core_mesi_state[0];  
        ccu_output.cache_upd_state_core[1] = core_mesi_state[1];
        ccu_output.cache_upd_state_core[2] = core_mesi_state[2];
        ccu_output.cache_upd_state_core[3] = core_mesi_state[3];
        internal_signals_ccu.upd_cache = 1'b0;
      end
     
    case(current_state)
        IDLE: begin
           ccu_output.rd_en = 1'b0;
           ccu_output.L1_1_CCU= 1'b0;
           ccu_output.L1_2_CCU = 1'b0;
           ccu_output.L1_3_CCU = 1'b0;
           ccu_output.L1_4_CCU = 1'b0;
           ccu_output.bs_req = 1'b0;
           ccu_output.write_done = 4'b0000;
           internal_signals_ccu.bs_resp1 = 1'b0;
           internal_signals_ccu.bs_resp2 = 1'b0;
           internal_signals_ccu.bs_resp3 = 1'b0;
           internal_signals_ccu.bs_resp4 = 1'b0;
           internal_signals_ccu.burst_mode = 1'b0;
           ccu_output.written = 1'b0;
           //ccu_output.rd_en_snoop = 1'b0;
           internal_signals_ccu.write_case = 1'b0;
           ccu_output.write_signal = 1'b0;
            
           for (int i = 0; i < 4; i++) begin
                if (i != internal_signals_ccu.req_core) begin
                    core_mesi_state[i] = I;  // Set all other cores to invalid state
                end
             end
           
           if(!ccu_input.buf_empty) begin
               next_state = FIFO_REQ;
           end else begin
               next_state = IDLE;
           end
           
        end
            
        FIFO_REQ: begin
             
           internal_signals_ccu.start = 1'b1;
           ccu_output.rd_en = 1'b1;
           ccu_output.L1_1_CCU= 1'b0;
           ccu_output.L1_2_CCU = 1'b0;
           ccu_output.L1_3_CCU = 1'b0;
           ccu_output.L1_4_CCU = 1'b0;
            
          
            if(ccu_output.rd_en)begin
              case(ccu_input.buf_out)
               4'b0100: internal_signals_ccu.req_core = 2'b00;
               4'b0101: internal_signals_ccu.req_core = 2'b01;
               4'b0110: internal_signals_ccu.req_core = 2'b10;
               4'b0111: internal_signals_ccu.req_core = 2'b11;
              endcase
              if(internal_signals_ccu.req_core == 2'b00)begin
                ccu_output.L1_1_CCU = 1'b1;  //signal 
                internal_signals_ccu.p_tag    = ccu_input.pr_addr_ccu[0][31:12];  
                internal_signals_ccu.p_index  = ccu_input.pr_addr_ccu[0][11:4];   
                internal_signals_ccu.p_offset = ccu_input.pr_addr_ccu[0][3:0]; 
                ccu_output.CCU_index = internal_signals_ccu.p_index; 
              end
              if(internal_signals_ccu.req_core == 2'b01)begin
                ccu_output.L1_2_CCU  = 1'b1;
                internal_signals_ccu.p_tag    = ccu_input.pr_addr_ccu[1][31:12];  
                internal_signals_ccu.p_index  = ccu_input.pr_addr_ccu[1][11:4];   
                internal_signals_ccu.p_offset = ccu_input.pr_addr_ccu[1][3:0]; 
                ccu_output.CCU_index = internal_signals_ccu.p_index; 
              end
              if(internal_signals_ccu.req_core == 2'b10)begin
                ccu_output.L1_3_CCU  = 1'b1;
                internal_signals_ccu.p_tag    = ccu_input.pr_addr_ccu[2][31:12];  
                internal_signals_ccu.p_index  = ccu_input.pr_addr_ccu[2][11:4];   
                internal_signals_ccu.p_offset = ccu_input.pr_addr_ccu[2][3:0]; 
                ccu_output.CCU_index = internal_signals_ccu.p_index;
              end
              if(internal_signals_ccu.req_core == 2'b11)begin
                ccu_output.L1_4_CCU  = 1'b1; //
                internal_signals_ccu.p_tag    = ccu_input.pr_addr_ccu[3][31:12];  
                internal_signals_ccu.p_index  = ccu_input.pr_addr_ccu[3][11:4];   
                internal_signals_ccu.p_offset = ccu_input.pr_addr_ccu[3][3:0]; 
                ccu_output.CCU_index = internal_signals_ccu.p_index; 
              end
            ccu_output.CCU_ready = 1'b0;
            next_state =  PROCESS_REQ;
            core_mesi_state[internal_signals_ccu.req_core] = mesi_t'(ccu_input.cache_state_core[internal_signals_ccu.req_core]);
            end
          
            else begin
             next_state = FIFO_REQ;
            end
        end
        
        PROCESS_REQ: begin
        
        ccu_output.rd_en = 1'b0;
        ccu_output.L1_1_CCU= 1'b0;
        ccu_output.L1_2_CCU = 1'b0;
        ccu_output.L1_3_CCU = 1'b0;
        ccu_output.L1_4_CCU = 1'b0;
         
        if(ccu_input.cache_hit[internal_signals_ccu.req_core] || ccu_input.cache_miss[internal_signals_ccu.req_core])begin 
               case (core_mesi_state[internal_signals_ccu.req_core])
                M: begin 
                    if (ccu_input.write[internal_signals_ccu.req_core] && ccu_input.cache_hit[internal_signals_ccu.req_core]) begin
                      ccu_output.l2_write_req = 1'b1;
                      
                      ccu_output.bs_req_data = 4'b1111;  
                      ccu_output.bs_req_data[internal_signals_ccu.req_core] = 1'b0;  
                      ccu_output.snoop_address = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];
                      internal_signals_ccu.write_case = 1'b1;
                      next_state = SNOOPING; // Change to next_state for state transition
                    end else if ((ccu_input.read[internal_signals_ccu.req_core] || ccu_input.write[internal_signals_ccu.req_core] ) && ccu_input.cache_miss[internal_signals_ccu.req_core]) begin
                      ccu_output.bs_req = 1'b1;
                      ccu_output.snoop_address = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];
                      next_state = SNOOPING; // Change to next_state for state transition
                    end
                end
                E: begin
                    if (ccu_input.write[internal_signals_ccu.req_core] && ccu_input.cache_hit[internal_signals_ccu.req_core]) begin
                      ccu_output.bs_req_data = 4'b1111;  
                      ccu_output.bs_req_data[internal_signals_ccu.req_core] = 1'b0;  
                      ccu_output.snoop_address = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];;
                      internal_signals_ccu.write_case = 1'b1;
                      next_state = SNOOPING; // Change to next_state for state transition
                    end else if ((ccu_input.read[internal_signals_ccu.req_core] || ccu_input.write[internal_signals_ccu.req_core]) && ccu_input.cache_miss[internal_signals_ccu.req_core]) begin
                      ccu_output.bs_req = 1'b1;
                      ccu_output.read_req = 1'b1; 
                      ccu_output.snoop_address = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];
                      next_state = SNOOPING; // Change to next_state for state transition
                    end
                end
                S: begin
                    if (ccu_input.write[internal_signals_ccu.req_core] && ccu_input.cache_hit[internal_signals_ccu.req_core]) begin
                      ccu_output.bs_req_data = 4'b1111;  
                      ccu_output.bs_req_data[internal_signals_ccu.req_core] = 1'b0; 
                      ccu_output.snoop_address = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];
                      internal_signals_ccu.write_case = 1'b1;
                      next_state = SNOOPING; // Change to next_state for state transition
                    end else if ((ccu_input.read[internal_signals_ccu.req_core] || ccu_input.write[internal_signals_ccu.req_core]) && ccu_input.cache_miss[internal_signals_ccu.req_core]) begin
                      ccu_output.bs_req = 1'b1;
                      ccu_output.read_req = 1'b1;   
                      ccu_output.snoop_address = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];                
                      next_state = SNOOPING; // Change to next_state for state transition
                    end
                end
                I: begin
                    if (ccu_input.read[internal_signals_ccu.req_core] && ccu_input.cache_hit[internal_signals_ccu.req_core]) begin
                      ccu_output.bs_req = 1'b1;
                      ccu_output.snoop_address = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];
                      next_state = SNOOPING; // Change to next_state for state transition
                    end else if (ccu_input.write[internal_signals_ccu.req_core] && ccu_input.cache_hit[internal_signals_ccu.req_core]) begin
                      ccu_output.bs_req_data = 4'b1111;  
                      ccu_output.bs_req_data[internal_signals_ccu.req_core] = 1'b0; 
                      internal_signals_ccu.write_case = 1'b1;
                      ccu_output.snoop_address = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];
                      next_state = SNOOPING; // Change to next_state for state transition
                    end else if ((ccu_input.read[internal_signals_ccu.req_core] || ccu_input.write[internal_signals_ccu.req_core]) && ccu_input.cache_miss[internal_signals_ccu.req_core]) begin
                      ccu_output.bs_req = 1'b1;
                      ccu_output.snoop_address = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];
                      next_state = SNOOPING; // Change to next_state for state transition
                    end
                  end
             endcase
          end else begin
            next_state = PROCESS_REQ;
          end
        end
        
        SNOOPING: begin
            // Logic for SNOOPING state remains unchanged ccu_input.buf_out [1][][][]
           //while (ccu_input.fifo_data_snoop != 4'b0000) // Wait until fifo_data_snoop becomes 0000
#2;
if (ccu_input.snoop_active || ccu_input.no_snoop && ccu_input.fifo_data_snoop != 4'b0000) begin
  ccu_output.bs_req = 1'b0;
  ccu_output.bs_req_data = 1'b0;
  if (ccu_input.fifo_data_snoop == 4'b1000) begin
    internal_signals_ccu.bs_resp1 = 1'b1;
  end
  else if (ccu_input.fifo_data_snoop == 4'b1001) begin
    internal_signals_ccu.bs_resp2 = 1'b1;
  end
  else if (ccu_input.fifo_data_snoop == 4'b1010) begin
    internal_signals_ccu.bs_resp3 = 1'b1;
  end
  else if (ccu_input.fifo_data_snoop == 4'b1011) begin
    internal_signals_ccu.bs_resp4 = 1'b1;
  end
end
          
#1;
if(ccu_input.snoop_active || ccu_input.no_snoop && ccu_input.fifo_data_snoop == 4'b0000) begin
   internal_signals_ccu.burst_mode = 1'b1;
end

//ccu_output.bs_req_data = 1'b0;
          
            if((ccu_input.snoop_active|| ccu_input.no_snoop ||  internal_signals_ccu.write_case) && ccu_input.fifo_data_snoop == 4'b0000 && internal_signals_ccu.burst_mode)begin
            if ((ccu_input.read[internal_signals_ccu.req_core] || ccu_input.write[internal_signals_ccu.req_core]) && ccu_input.cache_miss[internal_signals_ccu.req_core] ) begin
                if (internal_signals_ccu.bs_resp1 && ccu_input.core1_valid || internal_signals_ccu.bs_resp2 && ccu_input.core2_valid|| internal_signals_ccu.bs_resp3 && ccu_input.core3_valid|| internal_signals_ccu.bs_resp4 && ccu_input.core4_valid) begin
                    if (internal_signals_ccu.bs_resp1 && ccu_input.core1_valid) begin
                        ccu_output.data_out_CCU[internal_signals_ccu.req_core] = ccu_input.snoop_data;
                        core_mesi_state[0] = S;
                    end
                    if (internal_signals_ccu.bs_resp2 && ccu_input.core2_valid) begin
                        ccu_output.data_out_CCU[internal_signals_ccu.req_core] = ccu_input.snoop_data_1;
                        core_mesi_state[1] = S;
                    end
                    if (internal_signals_ccu.bs_resp3 && ccu_input.core3_valid) begin
                        ccu_output.data_out_CCU[internal_signals_ccu.req_core] = ccu_input.snoop_data_2;
                        core_mesi_state[2] = S;
                    end
                    if (internal_signals_ccu.bs_resp4 && ccu_input.core4_valid) begin
                        ccu_output.data_out_CCU[internal_signals_ccu.req_core] = ccu_input.snoop_data_3;
                        core_mesi_state[3] = S;
                    end
                    next_state = IDLE;
                    ccu_output.bs_signal = 4'b0000;
                    internal_signals_ccu.upd_cache = 1'b1;
                    internal_signals_ccu.start = 1'b0;
                    core_mesi_state[internal_signals_ccu.req_core] = S;
                    for (int i = 0; i < 4; i++) begin
                      if (i == internal_signals_ccu.req_core) begin
                     ccu_output.CCU_ready[i] = 1'b1;  // Set all other cores to invalid state
                    end
                  end
                end else begin
                    ccu_output.read_req = 1'b1;
                    next_state = Read_FROM_L2; // Change to next_state for state transition
                    internal_signals_ccu.start = 1'b0;
                end
            end  else if (ccu_input.write[internal_signals_ccu.req_core] && ccu_input.cache_hit[internal_signals_ccu.req_core]) begin
                if (internal_signals_ccu.bs_resp1 || internal_signals_ccu.bs_resp2 || internal_signals_ccu.bs_resp3 || internal_signals_ccu.bs_resp4) begin
                    if (internal_signals_ccu.bs_resp1) begin
                        core_mesi_state[0] = I;
                    end
                    if (internal_signals_ccu.bs_resp2) begin
                        core_mesi_state[1] = I;
                    end
                    if (internal_signals_ccu.bs_resp3) begin
                        core_mesi_state[2] = I;
                    end
                    if (internal_signals_ccu.bs_resp4) begin
                        core_mesi_state[3] = I;
                    end
                end
                next_state = WRITE_TO_L2; // Change to next_state for state transition
                ccu_output.l2_write_req = 1'b1;  
                internal_signals_ccu.start = 1'b0;
                internal_signals_ccu.upd_cache = 1'b1;
                //ccu_output.bs_req_data = 1'b0;
             end else begin
               
                next_state = IDLE; // Change to next_state for state transition
                internal_signals_ccu.upd_cache = 1'b1; 
                internal_signals_ccu.start = 1'b0; 
             end
           end else begin
        // Stay in SNOOPING if no conditions are met
        next_state = SNOOPING;
    end
        end 

        Read_FROM_L2: begin
            //ccu_output.rd_en_snoop = 1'b0;
            ccu_output.bs_req = 1'b0;
            ccu_output.bs_signal = 4'b0000;
            ccu_output.addr_to_send = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];
            if (ccu_input.ready) begin
                i = internal_signals_ccu.req_core;
                case (internal_signals_ccu.p_offset)
                    4'b0000: ccu_output.data_out_CCU[i] = ccu_input.l2_data[31:0];
                    4'b0100: ccu_output.data_out_CCU[i] = ccu_input.l2_data[63:32];
                    4'b1000: ccu_output.data_out_CCU[i] = ccu_input.l2_data[95:64];
                    4'b1100: ccu_output.data_out_CCU[i] = ccu_input.l2_data[127:96];
                endcase
                core_mesi_state[internal_signals_ccu.req_core] = E; 
                next_state = IDLE; // Change to next_state for state transition
                internal_signals_ccu.upd_cache = 1'b1;
                ccu_output.read_req = 1'b0;
                internal_signals_ccu.start = 1'b0;
                for (int i = 0; i < 4; i++) begin
                 if (i == internal_signals_ccu.req_core) begin
                     ccu_output.CCU_ready[i] = 1'b1;  // Set all other cores to invalid state
                end
              end
        end else begin
           next_state = Read_FROM_L2;
        end
       end

        WRITE_TO_L2: begin
            ccu_output.bs_signal = 4'b0000;
            //ccu_output.rd_en_snoop = 1'b0;
            internal_signals_ccu.write_case = 1'b0;
            ccu_output.bs_req_data = 1'b0;
            ccu_output.addr_to_send = ccu_input.pr_addr_ccu[internal_signals_ccu.req_core];
            if (ccu_input.ready) begin
                #1;
                ccu_output.written = 1'b1;
                case (internal_signals_ccu.req_core)
                    2'b00: ccu_output.write_data = ccu_input.cache_data1;  
                    2'b01: ccu_output.write_data = ccu_input.cache_data2;  
                    2'b10: ccu_output.write_data = ccu_input.cache_data3;  
                    2'b11: ccu_output.write_data = ccu_input.cache_data4;  
                endcase
                next_state = IDLE; // Change to next_state for state transition      
                internal_signals_ccu.upd_cache = 1'b1;
                ccu_output.write_signal = 1'b1;
                ccu_output.write_done[internal_signals_ccu.req_core] = 1'b1;
                internal_signals_ccu.start = 1'b0;
                core_mesi_state[internal_signals_ccu.req_core] = E;
            end
            else begin
              next_state = WRITE_TO_L2;
            end
          end
    endcase
   end
endmodule

   
