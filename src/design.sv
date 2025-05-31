`include "Cache_Controller0.sv"
`include "Cache_Controller1.sv"
`include "Cache_Controller2.sv"
`include "Cache_Controller3.sv"
`include "CCU.sv"
`include "L2_cache.sv"
`include "fifo.sv"
`include "arbiter.sv"

module top_module(
    // INPUTS
    input logic clk,
    input logic rst,
    input logic [3:0] read ,write,
    input logic [31:0] pr_addr[3:0],
    input logic [31:0] pr_data[3:0],
    input logic [3:0] Core_send,
    input logic [3:0] c_flush,
    input logic [127:0] mem_read_data,
    input logic mem_ready,
    input logic write_signal,

    // OUTPUTS
  output logic [31:0] data_out_pr [3:0],
  output logic stall,
  output logic stall_1,
  output logic stall_2,
  output logic stall_3
);

Cache_Controller_Inputs cc_inputs;
Cache_Controller_Outputs cc_outputs;
Cache_Controller_Inputs_1 cc_inputs1;
Cache_Controller_Outputs_1 cc_outputs1;
Cache_Controller_Inputs_2 cc_inputs2;
Cache_Controller_Outputs_2 cc_outputs2;
Cache_Controller_Inputs_3 cc_inputs3;
Cache_Controller_Outputs_3 cc_outputs3;
CCU_Input ccu_input;
CCU_Output ccu_output;
logic [31:0]data_out_CCU1;
logic [31:0]data_out_CCU2;
logic [31:0]data_out_CCU3;
logic [31:0]data_out_CCU4;
logic L1_1_CCU;
logic L1_2_CCU;
logic L1_3_CCU;
logic L1_4_CCU;
L2_Input_t l2_input;
L2_Output_t l2_output;
Arbiter_Input_t arbiter_input;
Arbiter_Output_t arbiter_output;
FIFO_Input fifo_input;
FIFO_Output fifo_output;


CCU ccu (
    .clk(clk),
    .rst(rst), 
    .ccu_input(ccu_input),
    .ccu_output(ccu_output)
 );

   Cache_Controller_0 Cache_Controller_0_module(
    .clk(clk),
    .rst(rst),
    .inputs(cc_inputs),
    .outputs(cc_outputs),
    .L1_1_CCU(L1_1_CCU),
    .data_out_CCU1(data_out_CCU1)
  );

   Cache_Controller_1 Cache_Controller_1_module(
     .clk(clk),
    .rst(rst),
    .inputs1(cc_inputs1),
    .outputs1(cc_outputs1),
    .L1_2_CCU(L1_2_CCU),
    .data_out_CCU2(data_out_CCU2)
  );

  Cache_Controller_2 Cache_Controller_2_module(
      .clk(clk),
    .rst(rst),
     .inputs2(cc_inputs2),
    .outputs2(cc_outputs2),
    .L1_3_CCU(L1_3_CCU),
    .data_out_CCU3(data_out_CCU3)
  );

  Cache_Controller_3 Cache_Controller_3_module(
     .clk(clk),
    .rst(rst),
    .inputs3(cc_inputs3),
    .outputs3(cc_outputs3),
    .L1_4_CCU(L1_4_CCU),
    .data_out_CCU4(data_out_CCU4)
  );

  
arbiter uut (
    .clk(clk),
    .rst(rst),
    .arbiter_input(arbiter_input),
    .arbiter_output(arbiter_output)
);

 // FIFO instance for storing granted requests
fifo f1 (
    .rst(rst),
    .fifo_input(fifo_input),
    .fifo_output(fifo_output)
);

 L2_cache L2(
     .clk(clk),
    .rst(rst),
     .l2_input(l2_input),
     .l2_output(l2_output)
    );

always_comb begin

    arbiter_input.Com_Bus_Req_proc[0] = cc_outputs.Request; //core 0
    arbiter_input.Com_Bus_Req_proc[1] = cc_outputs1.Request; //core 1
    arbiter_input.Com_Bus_Req_proc[2] = cc_outputs2.Request; //core 2
    arbiter_input.Com_Bus_Req_proc[3] = cc_outputs3.Request; //core 3
   
    arbiter_input.Com_Bus_Req_snoop[0] = cc_outputs.bs_resp; //core 0
    arbiter_input.Com_Bus_Req_snoop[1] = cc_outputs1.bs_resp;//core 1
    arbiter_input.Com_Bus_Req_snoop[2] = cc_outputs2.bs_resp;//core 2
    arbiter_input.Com_Bus_Req_snoop[3] = cc_outputs3.bs_resp;//core 3
  
    ccu_input.cache_state_core[0] = cc_outputs.cache_state_core; 
    ccu_input.cache_state_core[1] = cc_outputs1.cache_state_core;
    ccu_input.cache_state_core[2] = cc_outputs2.cache_state_core;
    ccu_input.cache_state_core[3] = cc_outputs3.cache_state_core;

    ccu_input.read[0] = cc_outputs.rd;
    ccu_input.read[1] = cc_outputs1.rd;
    ccu_input.read[2] = cc_outputs2.rd;
    ccu_input.read[3] = cc_outputs3.rd;
    
    ccu_input.write[0] = cc_outputs.wr;
    ccu_input.write[1] = cc_outputs1.wr;
    ccu_input.write[2] = cc_outputs2.wr;
    ccu_input.write[3] = cc_outputs3.wr;
    
    cc_inputs.read = read[0];
    cc_inputs1.read = read[1];
    cc_inputs2.read = read[2];
    cc_inputs3.read = read[3];

    cc_inputs.write = write[0];
    cc_inputs1.write = write[1];
    cc_inputs2.write = write[2];
    cc_inputs3.write = write[3];
    
    cc_inputs.pr_addr = pr_addr[0];
    cc_inputs1.pr_addr = pr_addr[1];
    cc_inputs2.pr_addr = pr_addr[2];
    cc_inputs3.pr_addr = pr_addr[3];
    
     cc_inputs.pr_data = pr_data[0];
    cc_inputs1.pr_data = pr_data[1];
    cc_inputs2.pr_data = pr_data[2];
    cc_inputs3.pr_data = pr_data[3];
    
    cc_inputs.Core_send = Core_send[0];
    cc_inputs1.Core_send = Core_send[1];
    cc_inputs2.Core_send = Core_send[2];
    cc_inputs3.Core_send = Core_send[3];
    
    cc_inputs.c_flush = c_flush[0];
    cc_inputs1.c_flush = c_flush[1];
    cc_inputs2.c_flush = c_flush[2];
    cc_inputs3.c_flush = c_flush[3];
    
    data_out_pr[0] = cc_outputs.data_out_pr;
    data_out_pr[1] = cc_outputs1.data_out_pr;
    data_out_pr[2] = cc_outputs2.data_out_pr;
    data_out_pr[3] = cc_outputs3.data_out_pr;
    
    stall = cc_outputs.stall;
    stall_1 = cc_outputs1.stall;
    stall_2 = cc_outputs2.stall;
    stall_3 = cc_outputs3.stall;
    
    ccu_input.cache_hit[0] = cc_outputs.hit;
    ccu_input.cache_hit[1] = cc_outputs1.hit;
    ccu_input.cache_hit[2] = cc_outputs2.hit;
    ccu_input.cache_hit[3] = cc_outputs3.hit;
    
    data_out_CCU1 = ccu_output.data_out_CCU[0];
    data_out_CCU2 = ccu_output.data_out_CCU[1];
    data_out_CCU3 = ccu_output.data_out_CCU[2];
    data_out_CCU4 = ccu_output.data_out_CCU[3];
    
    ccu_input.cache_miss[0] = cc_outputs.miss;
    ccu_input.cache_miss[1] = cc_outputs1.miss;
    ccu_input.cache_miss[2] = cc_outputs2.miss;
    ccu_input.cache_miss[3] = cc_outputs3.miss;
   
    ccu_input.pr_addr_ccu[0] = cc_outputs.addr;
    ccu_input.pr_addr_ccu[1] = cc_outputs1.addr;
    ccu_input.pr_addr_ccu[2] = cc_outputs2.addr;
    ccu_input.pr_addr_ccu[3] = cc_outputs3.addr;

    ccu_input.pr_data_ccu[0] = cc_outputs.data;
    ccu_input.pr_data_ccu[1] = cc_outputs1.data;
    ccu_input.pr_data_ccu[2] = cc_outputs2.data;
    ccu_input.pr_data_ccu[3] = cc_outputs3.data;
    
    cc_inputs.proc_gnt = arbiter_output.proc_gnt[0];
    cc_inputs1.proc_gnt = arbiter_output.proc_gnt[1];
    cc_inputs2.proc_gnt = arbiter_output.proc_gnt[2];
    cc_inputs3.proc_gnt = arbiter_output.proc_gnt[3];
    
    cc_inputs.snoop_gnt = arbiter_output.snoop_gnt[0];
    cc_inputs1.snoop_gnt = arbiter_output.snoop_gnt[1];
    cc_inputs2.snoop_gnt = arbiter_output.snoop_gnt[2];
    cc_inputs3.snoop_gnt = arbiter_output.snoop_gnt[3];
    
    cc_inputs.bs_signal = ccu_output.bs_signal[0];
    cc_inputs1.bs_signal = ccu_output.bs_signal[1];
    cc_inputs2.bs_signal = ccu_output.bs_signal[2];
    cc_inputs3.bs_signal = ccu_output.bs_signal[3];
    
    cc_inputs.CCU_ready = ccu_output.CCU_ready[0];
    cc_inputs1.CCU_ready = ccu_output.CCU_ready[1];
    cc_inputs2.CCU_ready = ccu_output.CCU_ready[2];    
    cc_inputs3.CCU_ready = ccu_output.CCU_ready[3];
    
    fifo_input.data_in = arbiter_output.fifo_data_in ;
    fifo_input.wr_en = arbiter_output.wr_en;
    fifo_input.rd_en = ccu_output.rd_en;
    //fifo_input.rd_en_snoop = ccu_output.rd_en_snoop;
    ccu_input.fifo_data_snoop = arbiter_output.fifo_data_snoop;
    
    ccu_input.buf_empty = fifo_output.buf_empty;
    ccu_input.buf_full = fifo_output.buf_full;
    ccu_input.buf_out = fifo_output.buf_out;
    ccu_input.no_snoop = arbiter_output.no_snoop;
    ccu_input.snoop_active = arbiter_output.snoop_active;
    
    ccu_input.ready = l2_output.ready;
    ccu_input.l2_data = l2_output.read_data;
    
    cc_inputs.bs_req = ccu_output.bs_req;
    
    cc_inputs1.bs_req = ccu_output.bs_req;
   
    cc_inputs2.bs_req = ccu_output.bs_req;
  
    cc_inputs3.bs_req = ccu_output.bs_req;
 
    
    l2_input.l2_write_req =  ccu_output.l2_write_req; 
    l2_input.l2_read_req = ccu_output.read_req;
    l2_input.write_data = ccu_output.write_data;
    
    ccu_input.core1_valid = cc_outputs.core_valid;
    ccu_input.core2_valid = cc_outputs1.core_valid;
    ccu_input.core3_valid = cc_outputs2.core_valid;
    ccu_input.core4_valid = cc_outputs3.core_valid;
    
    ccu_input.cache_data1 = cc_outputs.cache_data;
    ccu_input.cache_data2 = cc_outputs1.cache_data;
    ccu_input.cache_data3 = cc_outputs2.cache_data;
    ccu_input.cache_data4 = cc_outputs3.cache_data;
    
    ccu_input.snoop_data = cc_outputs.snoop_data;
    ccu_input.snoop_data_1 = cc_outputs1.snoop_data;
    ccu_input.snoop_data_2 = cc_outputs2.snoop_data;
    ccu_input.snoop_data_3 = cc_outputs3.snoop_data;
    
    cc_inputs.cache_upd_state_core = ccu_output.cache_upd_state_core[0];
    cc_inputs1.cache_upd_state_core = ccu_output.cache_upd_state_core[1];
    cc_inputs2.cache_upd_state_core = ccu_output.cache_upd_state_core[2];
    cc_inputs3.cache_upd_state_core = ccu_output.cache_upd_state_core[3];
    
    cc_inputs.CCU_ready = ccu_output.CCU_ready[0];
    cc_inputs1.CCU_ready = ccu_output.CCU_ready[1];
    cc_inputs2.CCU_ready = ccu_output.CCU_ready[2];
    cc_inputs3.CCU_ready = ccu_output.CCU_ready[3];
    
    cc_inputs.bs_signal = ccu_output.bs_signal[0];
    cc_inputs1.bs_signal = ccu_output.bs_signal[1];
    cc_inputs2.bs_signal = ccu_output.bs_signal[2];
    cc_inputs3.bs_signal = ccu_output.bs_signal[3];
    
    L1_1_CCU = ccu_output.L1_1_CCU;
    L1_2_CCU = ccu_output.L1_2_CCU;
    L1_3_CCU = ccu_output.L1_3_CCU;
    L1_4_CCU = ccu_output.L1_4_CCU;
    
    cc_inputs.snoop_address = ccu_output.snoop_address;
    cc_inputs1.snoop_address = ccu_output.snoop_address;
    cc_inputs2.snoop_address = ccu_output.snoop_address;
    cc_inputs3.snoop_address = ccu_output.snoop_address;
    
    l2_input.write_signal = ccu_output.write_signal;
    l2_input.addr = ccu_output.addr_to_send;
    l2_input.mem_ready = mem_ready;
    l2_input.mem_read_data =  mem_read_data;
    l2_input.write_signal = write_signal;
  
    cc_inputs.bs_req_data = ccu_output.bs_req_data[0];
    cc_inputs1.bs_req_data = ccu_output.bs_req_data[1];
    cc_inputs2.bs_req_data = ccu_output.bs_req_data[2];
    cc_inputs3.bs_req_data = ccu_output.bs_req_data[3];
  
    cc_inputs.write_done = ccu_output.write_done[0];
    cc_inputs1.write_done = ccu_output.write_done[1];
    cc_inputs2.write_done = ccu_output.write_done[2];
    cc_inputs3.write_done = ccu_output.write_done[3];
  
    l2_input.written = ccu_output.written;
    
end

endmodule
