class core_sequence_item extends uvm_sequence_item;
  `uvm_object_utils(core_sequence_item)

  //--------------------------------------------------------
  //Instantiation
  //--------------------------------------------------------
  rand logic rst;
  rand logic [3:0] read, write;
  rand logic [31:0] pr_addr[3:0];
  rand logic [31:0] pr_data[3:0];
  rand logic [3:0] Core_send;
  rand logic [3:0] c_flush;
  rand logic [127:0] mem_read_data;
  rand logic mem_ready;
  rand logic write_signal;

  // OUTPUTS
  logic [31:0] data_out_pr[3:0];
  logic stall;
  logic stall_1;
  logic stall_2;
  logic stall_3;

  //--------------------------------------------------------
  //Default Constraints
  //--------------------------------------------------------
  // Ensure read and write are mutually exclusive
/*constraint read_write_exclusive {
    foreach (read[i]) {
       !(read[i] && write[i]); 
    }
}

constraint read_c{
    read inside {[4'b0000 : 4'b1111]};
}

constraint write_c{
    write inside {[4'b0000 : 4'b1111]};
}
*/  
constraint pr_addr_c {
    foreach (pr_addr[i]) {
        pr_addr[i] % 16 == 4'h0 || pr_addr[i] % 16 == 4'h4 || 
        pr_addr[i] % 16 == 4'h8 || pr_addr[i] % 16 == 4'hC; 
    }
}

constraint pr_addr_rand_c {
    foreach (pr_addr[i]) {
        pr_addr[i][31] dist {1'b0 := 50, 1'b1 := 50}; 
    }
}

      
constraint read_c{
    read == 4'b1111;
}

constraint write_c{
    write == 4'b0000;
}
 
 /*constraint pr_addr_c{
    pr_addr[0] == 32'h11111000;
    pr_addr[1] == 32'h11111000;
    pr_addr[2] == 32'h11111000;
    pr_addr[3] == 32'h11111000;
 }*/
      
  constraint pr_data_c {
    foreach (pr_data[i]) {
      pr_data[i] == 32'hCAFEBABE; 
    }
  }

  constraint Core_send_c {
    Core_send == 4'b1111;
  }

  constraint c_flush_c {
    c_flush == 0; 
  }

 
  constraint mem_ready_c {
    mem_ready == 1'b1;
  }
      
  constraint write_signal_c {
    write_signal == 1'b1;
  }

  constraint mem_read_data_c {
    mem_read_data == 128'h123456789ABCDEF0123456789ABCDEF0;
  }
  
  //--------------------------------------------------------
  //Constructor
  //--------------------------------------------------------
  function new(string name = "core_sequence_item");
    super.new(name);
  endfunction: new

endclass: core_sequence_item
