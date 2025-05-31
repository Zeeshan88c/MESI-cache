
module top_module_tb;

  // Testbench signals
  logic clk;
  logic rst;
  logic [3:0] read, write;
  logic [31:0] pr_addr[3:0];
  logic [31:0] pr_data[3:0];
  logic [3:0] Core_send;
  logic [3:0] c_flush;
  logic [127:0] mem_read_data;
  logic mem_ready;
  logic [31:0] data_out_pr[3:0];
  logic stall;
  logic stall_1;
  logic stall_2;
  logic stall_3;

  // Instantiate the DUT
  top_module uut (
    .clk(clk),
    .rst(rst),
    .read(read),
    .write(write),
    .pr_addr(pr_addr),
    .pr_data(pr_data),
    .Core_send(Core_send),
    .c_flush(c_flush),
    .mem_read_data(mem_read_data),
    .mem_ready(mem_ready),
    .data_out_pr(data_out_pr),
    .stall(stall),
    .stall_1(stall_1),
    .stall_2(stall_2),
    .stall_3(stall_3)
  );

  // Clock generation
  initial begin
    clk = 0;
    forever #5 clk = ~clk; // 10ns clock period
  end

  // Reset and test sequence
  initial begin
    // Initialize inputs
    rst = 1;
    read = 4'b0000;
    write = 4'b0000;
    pr_addr = '{32'h0, 32'h0, 32'h0, 32'h0};
    pr_data = '{32'h0, 32'h0, 32'h0, 32'h0};
    Core_send = 4'b0000;
    c_flush = 4'b0000;
    mem_read_data = 128'h0;
    mem_ready = 0;

    // Apply reset
    #20 rst = 0;

    // Test read operations
    read = 4'b1111; // Enable read for cores 0 and 2
    pr_addr[0] = 32'h11111000;
    pr_addr[1] = 32'h11111000;
    pr_addr[2] = 32'h10000000;
    pr_addr[3] = 32'h10000000;
    Core_send[0] = 1'b1;
    Core_send[1] = 1'b1;
    Core_send[2] = 1'b1;
    Core_send[3] = 1'b1;
    mem_read_data = 128'h123456789ABCDEF0123456789ABCDEF0;
    mem_ready = 1;
    #120;

    // Test write operations
    read = 4'b0000;
    /*write = 4'b0101; // Enable write for cores 1 and 3
    pr_addr[1] = 32'h3000;
    pr_addr[3] = 32'h4000;
    pr_data[1] = 32'hDEADBEEF;
    pr_data[3] = 32'hCAFEBABE;
    #10;

    // Test cache flush
    write = 4'b0000;
    c_flush = 4'b1111;
    #10;

    // Test memory ready signal
    c_flush = 4'b0000;
    mem_read_data = 128'h123456789ABCDEF0123456789ABCDEF0;
    mem_ready = 1;
    #10;

    mem_ready = 0;*/

    // Additional test scenarios can be added here

    // End simulation
    #1000 $stop;
  end

  // Monitor outputs
  initial begin
    $monitor($time, " clk=%b, rst=%b, read=%b, write=%b, pr_addr[0]=%h, pr_data[0]=%h, data_out_pr[0]=%h, stall=%b",
             clk, rst, read, write, pr_addr[0], pr_data[0], data_out_pr[0], stall);
  end

endmodule
