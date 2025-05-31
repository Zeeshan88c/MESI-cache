class core_scoreboard extends uvm_test;
  `uvm_component_utils(core_scoreboard)
  
  uvm_analysis_imp #(core_sequence_item, core_scoreboard) scoreboard_port;
  uvm_analysis_imp #(core_sequence_item, core_scoreboard) scoreboard_port1;
  uvm_analysis_imp #(core_sequence_item, core_scoreboard) scoreboard_port2;
  uvm_analysis_imp #(core_sequence_item, core_scoreboard) scoreboard_port3;
  
  core_sequence_item transactions[$];
  
  //--------------------------------------------------------
  // Constructor
  //--------------------------------------------------------
  function new(string name = "core_scoreboard", uvm_component parent);
    super.new(name, parent);
    `uvm_info("SCB_CLASS", "Inside Constructor!", UVM_HIGH)
  endfunction: new
  
  //--------------------------------------------------------
  // Build Phase
  //--------------------------------------------------------
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    `uvm_info("SCB_CLASS", "Build Phase!", UVM_HIGH)
   
    scoreboard_port = new("scoreboard_port", this);
    scoreboard_port1 = new("scoreboard_port1", this);
    scoreboard_port2 = new("scoreboard_port2", this);
    scoreboard_port3 = new("scoreboard_port3", this);
  endfunction: build_phase

  //--------------------------------------------------------
  // Connect Phase
  //--------------------------------------------------------
  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    `uvm_info("SCB_CLASS", "Connect Phase!", UVM_HIGH)
  endfunction: connect_phase
  
  //--------------------------------------------------------
  // Write Method
  //--------------------------------------------------------
  function void write(core_sequence_item item);
    transactions.push_back(item);
  endfunction: write 
  
  //--------------------------------------------------------
  // Run Phase
  //--------------------------------------------------------
  task run_phase(uvm_phase phase);
    super.run_phase(phase);
    `uvm_info("SCB_CLASS", "Run Phase!", UVM_HIGH)
   
    forever begin
      core_sequence_item curr_trans;
      wait(transactions.size() != 0);
      curr_trans = transactions.pop_front();
      //display(curr_trans);
      compare(curr_trans);
    end
  endtask: run_phase
  
  //--------------------------------------------------------
  // Display Task (Ensures One-Time Display)
  //--------------------------------------------------------
  task display(core_sequence_item curr_trans);
    //static bit displayed = 0; // Ensures one-time execution
    logic [3:0] read;
    logic [3:0] read_CC;
    logic [3:0] addr_CC;
    logic [3:0] bs_resp;
    logic [3:0] Req;
    logic [3:0] L1;
    logic snoop_act;
    logic [31:0] snoop_data [3:0];
    logic bs_request;
    logic [3:0] miss;
 
    read = curr_trans.read;
    
    read_CC[0] = top.dut.cc_inputs.read;
    read_CC[1] = top.dut.cc_inputs1.read;
    read_CC[2] = top.dut.cc_inputs2.read;
    read_CC[3] = top.dut.cc_inputs3.read;
    
    addr_CC[0] = top.dut.cc_inputs.pr_addr;
    addr_CC[1] = top.dut.cc_inputs1.pr_addr;
    addr_CC[2] = top.dut.cc_inputs2.pr_addr;
    addr_CC[3] = top.dut.cc_inputs3.pr_addr;
    
    bs_resp[0] = top.dut.cc_outputs.bs_resp;
    bs_resp[1] = top.dut.cc_outputs1.bs_resp;
    bs_resp[2] = top.dut.cc_outputs2.bs_resp;
    bs_resp[3] = top.dut.cc_outputs3.bs_resp;
    
    Req[0] = top.dut.cc_outputs.Request;
    Req[1] = top.dut.cc_outputs1.Request;
    Req[2] = top.dut.cc_outputs2.Request;
    Req[3] = top.dut.cc_outputs3.Request;
    
    snoop_act = top.dut.ccu_input.snoop_active;
    snoop_data[0] = top.dut.ccu_input.snoop_data;
    snoop_data[1] = top.dut.ccu_input.snoop_data_1;
    snoop_data[2] = top.dut.ccu_input.snoop_data_2;
    snoop_data[3] = top.dut.ccu_input.snoop_data_3;
    bs_request = top.dut.ccu_output.bs_req;
    
    miss[0] = top.dut.Cache_Controller_0_module.int_signals.cache_miss;
    miss[1] = top.dut.Cache_Controller_1_module.int_signals1.cache_miss;
    miss[2] = top.dut.Cache_Controller_2_module.int_signals2.cache_miss;
    miss[3] = top.dut.Cache_Controller_3_module.int_signals3.cache_miss;
    //$display("Arbiter Request: %b", top.dut.arbiter_input.Com_Bus_Req_proc);
    
    L1[0] = top.dut.ccu_output.L1_1_CCU;
    L1[1] = top.dut.ccu_output.L1_2_CCU;
    L1[2] = top.dut.ccu_output.L1_3_CCU;
    L1[3] = top.dut.ccu_output.L1_4_CCU;
    
for (int i = 0; i < 4; i++) begin
    if (read_CC[i] && miss[i]) begin
        if (Req[i] == 1'b1) begin
            `uvm_info("COMPARE", $sformatf("Request sent by Core %d! ACT=%h, EXP=%h", i, Req[i], 1'b1), UVM_LOW);
        end  
        if (top.dut.arbiter_output.proc_gnt[i] == 1'b1) begin 
            `uvm_info("COMPARE", $sformatf("Request Gnt has been given to Core %d", i), UVM_LOW);
        end
        if (L1[i]) begin
            `uvm_info("COMPARE", $sformatf("CCU currently working on Core %d", i), UVM_LOW);
        end

        #1;
        if (bs_request) begin
            `uvm_info("COMPARE", "Bus request sent to all cores", UVM_LOW);
            
            if (bs_resp[i]) begin
                `uvm_info("SEND", $sformatf("Core %d sends data for snooping: %h", i, snoop_data[i]), UVM_LOW);
            end else begin
                `uvm_info("NOT SEND", $sformatf("Core %d, doesn't have data", i), UVM_LOW);
            end
        end
        
        // Display state updates for each core
        for (int j = 0; j < 4; j++) begin
            if (top.dut.ccu_output.cache_upd_state_core[j] == 2'b01) begin
                `uvm_info("RECEIVE", $sformatf("Core %d state is Exclusive, Got data from L2 Cache: %h", j+1, top.dut.l2_output.read_data), UVM_LOW);
            end else if (top.dut.ccu_output.cache_upd_state_core[j] == 2'b10) begin
                `uvm_info("RECEIVE", $sformatf("Core %d state is Shared, Got data from Cache: %h", j+1, top.dut.ccu_output.data_out_CCU[j]), UVM_LOW);
            end 
        end
    end
end
  endtask: display

  //--------------------------------------------------------
  // Compare Task: Data Validation
  //--------------------------------------------------------
task compare(core_sequence_item curr_trans);
    logic [31:0] data_out_pr[3:0];
    logic [127:0] mem;
    logic [3:0] read;
    logic [3:0] write;
    logic [3:0] miss_cc;
    logic [3:0] hit_cc;
    logic [31:0] pr_addr [3:0];
    logic [2:0] state[3:0];
    logic L2_req;
    logic [127:0] L2_data;
    logic miss_l2;
  logic [127:0] s_to_l2;
   logic [31:0] addr_tmp;
    mem = 128'h123456789ABCDEF0123456789ABCDEF0;
    
  
    read = curr_trans.read;
    write= curr_trans.write;
     

    // Capture Data Output from CCU
    data_out_pr[0] = top.dut.cc_outputs.data_out_pr;
    data_out_pr[1] = top.dut.cc_outputs1.data_out_pr;
    data_out_pr[2] = top.dut.cc_outputs2.data_out_pr;
    data_out_pr[3] = top.dut.cc_outputs3.data_out_pr;

    pr_addr[0] = top.dut.cc_inputs.pr_addr;
    pr_addr[1] = top.dut.cc_inputs1.pr_addr;
    pr_addr[2] = top.dut.cc_inputs2.pr_addr;
    pr_addr[3] = top.dut.cc_inputs3.pr_addr;
  
    state[0] = top.dut.Cache_Controller_0_module.current_state;
    state[1] = top.dut.Cache_Controller_1_module.current_state;
    state[2] = top.dut.Cache_Controller_2_module.current_state;
    state[3] = top.dut.Cache_Controller_3_module.current_state;
   
    L2_req = top.dut.l2_input.l2_read_req;
    L2_data = top.dut.l2_output.read_data;
    miss_l2 = top.dut.l2_output.l2_miss;
    s_to_l2 = top.dut.ccu_output.write_data;
  
    miss_cc[0] = top.dut.Cache_Controller_0_module.int_signals.cache_miss;
  miss_cc[1] = top.dut.Cache_Controller_1_module.int_signals1.cache_miss;
  miss_cc[2] = top.dut.Cache_Controller_2_module.int_signals2.cache_miss;
  miss_cc[3] = top.dut.Cache_Controller_3_module.int_signals3.cache_miss;
  
  hit_cc[0] = top.dut.Cache_Controller_0_module.int_signals.cache_hit;
  hit_cc[1] = top.dut.Cache_Controller_1_module.int_signals1.cache_hit;
  hit_cc[2] = top.dut.Cache_Controller_2_module.int_signals2.cache_hit;
  hit_cc[3] = top.dut.Cache_Controller_3_module.int_signals3.cache_hit;

    for (int i = 0; i < 4; i++) begin 
        // Extract LSB from pr_addr[i]
      int lsb = pr_addr[i][3:0];
      if(read[i])begin
        if (state[i] == 3'b101) begin
            // Check for LSB address and compare respective memory segment
          if (lsb == 4'b0000) begin
                if (mem[31:0] === data_out_pr[i]) begin
                    `uvm_info("COMPARE", $sformatf("Core %d Match: %h == %h at address %h", i, mem[31:0], data_out_pr[i], pr_addr[i]), UVM_LOW);
                end else begin
                    `uvm_error("COMPARE", $sformatf("Core %d Mismatch! Expected: %h, Got: %h at address %h", i, mem[31:0], data_out_pr[i], pr_addr[i]));
                end
            end

          if (lsb == 4'b0100) begin
                if (mem[63:32] === data_out_pr[i]) begin
                    `uvm_info("COMPARE", $sformatf("Core %d Match: %h == %h at address %h", i, mem[63:32], data_out_pr[i], pr_addr[i]), UVM_LOW);
                end else begin
                    `uvm_error("COMPARE", $sformatf("Core %d Mismatch! Expected: %h, Got: %h at address %h", i, mem[63:32], data_out_pr[i], pr_addr[i]));
                end
            end

          if (lsb == 4'b1000) begin
                if (mem[95:64] === data_out_pr[i]) begin
                    `uvm_info("COMPARE", $sformatf("Core %d Match: %h == %h at address %h", i, mem[95:64], data_out_pr[i], pr_addr[i]), UVM_LOW);
                end else begin
                    `uvm_error("COMPARE", $sformatf("Core %d Mismatch! Expected: %h, Got: %h at address %h", i, mem[95:64], data_out_pr[i], pr_addr[i]));
                end
            end

          if (lsb == 4'b1100) begin
                if (mem[127:96] === data_out_pr[i]) begin
                    `uvm_info("COMPARE", $sformatf("Core %d Match: %h == %h at address %h", i, mem[127:96], data_out_pr[i], pr_addr[i]), UVM_LOW);
                end else begin
                    `uvm_error("COMPARE", $sformatf("Core %d Mismatch! Expected: %h, Got: %h at address %h", i, mem[127:96], data_out_pr[i], pr_addr[i]));
                end
            end
          
    
    end
    end
    end
  
        if(L2_req) begin
  //if(miss_l2)begin
    if(L2_data == mem)begin
        `uvm_info("COMPARE", $sformatf("L2 got data from Main Memory"), UVM_LOW);
      end 
    //end
 end
  
  for (int j = 0; j < 4; j++) begin 
        // Extract LSB from pr_addr[j]
       
     int lsb_1 = pr_addr[j][3:0];
     
      if(write[j] )begin
        if(hit_cc[j])begin
        //if (state[j] == 3'b101) begin
            // Check for LSB address and compare respective memory segment
          if (lsb_1 == 4'b0000) begin
            if (s_to_l2[31:0] == 32'hCAFEBABE) begin
                    `uvm_info("COMPARE", $sformatf("Core %d Match: data %h wtitten to L2 having address: %h", j, s_to_l2[31:0], pr_addr[j]), UVM_LOW);
                end /* else begin
                    `uvm_error("COMPARE", $sformatf("Core %d Mismatch! Expected: %h,  at address %h", j, s_to_l2[31:0], pr_addr[j]));
                end*/
            end

          if (lsb_1 == 4'b0100) begin
                if (s_to_l2[63:32] == 32'hCAFEBABE) begin
                    `uvm_info("COMPARE", $sformatf("Core %d Match: data %h wtitten to L2 having address: %h", j, s_to_l2[63:32],  pr_addr[j]), UVM_LOW);
                end /*else begin
                    `uvm_error("COMPARE", $sformatf("Core %d Mismatch! Expected: %h, at address %h", j, s_to_l2[63:32], pr_addr[j]));
                end*/
            end

          if (lsb_1 == 4'b1000) begin
                if (s_to_l2[95:64] == 32'hCAFEBABE) begin
                    `uvm_info("COMPARE", $sformatf("Core %d Match: data %h wtitten to L2 having address: %h", j, s_to_l2[95:64], pr_addr[j]), UVM_LOW);
                end /* else begin
                    `uvm_error("COMPARE", $sformatf("Core %d Mismatch! Expected: %h, at address %h", j, s_to_l2[95:64], pr_addr[j]));
                end*/
            end

          if (lsb_1 == 4'b1100) begin
                if (s_to_l2[127:96] == 32'hCAFEBABE) begin
                    `uvm_info("COMPARE", $sformatf("Core %d Match: data %h wtitten to L2 having address: %h", j, s_to_l2[127:96],pr_addr[j]), UVM_LOW);
                end /*else begin
                    `uvm_error("COMPARE", $sformatf("Core %d Mismatch! Expected: %h, at address %h", j, s_to_l2[127:96],  pr_addr[j]));
                end*/
            end
          
    
   // end
    end
    end
  end
  
endtask: compare



endclass: core_scoreboard
