// 1. DESIGN - uart_tx
module uart_tx #(
  parameter CLKS_PER_BIT = 4   // sim-friendly; for real HW: CLK_FREQ/BAUD_RATE
)(
  input  logic       clk,
  input  logic       rst,
  input  logic       tx_start,
  input  logic [7:0] data_in,
  output logic       tx_serial,
  output logic       tx_busy,
  output logic       tx_done
);
  typedef enum logic [1:0] {TX_IDLE, TX_START, TX_DATA, TX_STOP} state_e;
  state_e state;
  logic [2:0]                          bit_idx;
  logic [$clog2(CLKS_PER_BIT+1)-1:0]   clk_cnt;
  logic [7:0]                          data_reg;
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      state     <= TX_IDLE;
      tx_serial <= 1'b1;
      tx_busy   <= 1'b0;
      tx_done   <= 1'b0;
      clk_cnt   <= 0;
      bit_idx   <= 0;
      data_reg  <= 0;
    end else begin
      tx_done <= 1'b0;  // default: one-cycle pulse
      case (state)
        TX_IDLE: begin
          tx_serial <= 1'b1;
          clk_cnt   <= 0;
          bit_idx   <= 0;
          if (tx_start) begin
            data_reg <= data_in;
            tx_busy  <= 1'b1;
            state    <= TX_START;
          end else begin
            tx_busy <= 1'b0;
          end
        end
        TX_START: begin
          tx_serial <= 1'b0;              // start bit = 0
          if (clk_cnt < CLKS_PER_BIT-1) clk_cnt <= clk_cnt + 1;
          else begin
            clk_cnt <= 0;
            state   <= TX_DATA;
          end
        end
        TX_DATA: begin
          tx_serial <= data_reg[bit_idx]; // LSB first
          if (clk_cnt < CLKS_PER_BIT-1) clk_cnt <= clk_cnt + 1;
          else begin
            clk_cnt <= 0;
            if (bit_idx < 7) bit_idx <= bit_idx + 1;
            else begin
              bit_idx <= 0;
              state   <= TX_STOP;
            end
          end
        end
        TX_STOP: begin
          tx_serial <= 1'b1;              // stop bit = 1
          if (clk_cnt < CLKS_PER_BIT-1) clk_cnt <= clk_cnt + 1;
          else begin
            clk_cnt <= 0;
            tx_busy <= 1'b0;
            tx_done <= 1'b1;
            state   <= TX_IDLE;
          end
        end
      endcase
    end
  end
endmodule
// 1b. DESIGN - uart_rx
module uart_rx #(
  parameter CLKS_PER_BIT = 4
)(
  input  logic       clk,
  input  logic       rst,
  input  logic       rx_serial,
  output logic [7:0] rx_data,
  output logic       rx_done,
  output logic       rx_busy
);
  typedef enum logic [1:0] {RX_IDLE, RX_START, RX_DATA, RX_STOP} state_e;
  state_e state;
  logic [2:0]                          bit_idx;
  logic [$clog2(CLKS_PER_BIT+1)-1:0]   clk_cnt;
  logic [7:0]                          data_reg;
  logic                                rx_sync1, rx_sync2; // 2-FF synchronizer
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      rx_sync1 <= 1'b1;
      rx_sync2 <= 1'b1;
    end else begin
      rx_sync1 <= rx_serial;
      rx_sync2 <= rx_sync1;
    end
  end
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      state    <= RX_IDLE;
      rx_done  <= 1'b0;
      rx_busy  <= 1'b0;
      clk_cnt  <= 0;
      bit_idx  <= 0;
      rx_data  <= 0;
      data_reg <= 0;
    end else begin
      rx_done <= 1'b0;  // default: one-cycle pulse
      case (state)
        RX_IDLE: begin
          rx_busy <= 1'b0;
          clk_cnt <= 0;
          bit_idx <= 0;
          if (!rx_sync2) begin          // falling edge -> possible start bit
            state   <= RX_START;
            rx_busy <= 1'b1;
          end
        end
        RX_START: begin
          // sample at mid-bit to confirm it's a real start bit, not glitch
          if (clk_cnt == (CLKS_PER_BIT/2)) begin
            if (!rx_sync2) begin
              clk_cnt <= 0;
              state   <= RX_DATA;
            end else begin
              state <= RX_IDLE;         // false start, bail out
            end
          end else begin
            clk_cnt <= clk_cnt + 1;
          end
        end
        RX_DATA: begin
          if (clk_cnt < CLKS_PER_BIT-1) begin
            clk_cnt <= clk_cnt + 1;
          end else begin
            clk_cnt          <= 0;
            data_reg[bit_idx] <= rx_sync2;   // sample mid-bit-aligned
            if (bit_idx < 7) bit_idx <= bit_idx + 1;
            else begin
              bit_idx <= 0;
              state   <= RX_STOP;
            end
          end
        end
        RX_STOP: begin
          if (clk_cnt < CLKS_PER_BIT-1) begin
            clk_cnt <= clk_cnt + 1;
          end else begin
            clk_cnt <= 0;
            rx_data <= data_reg;
            rx_done <= 1'b1;
            rx_busy <= 1'b0;
            state   <= RX_IDLE;
          end
        end
      endcase
    end
  end
endmodule
// 2. INTERFACE - uart_if
interface uart_if (input logic clk);
  logic       rst;
  // TX side (driver drives these)
  logic       tx_start;
  logic [7:0] data_in;
  logic       tx_busy;
  logic       tx_done;
  logic       tx_serial;
  // RX side (monitor observes these)
  logic [7:0] rx_data;
  logic       rx_done;
  logic       rx_busy;
  clocking driver_cb @(posedge clk);
    default input #1 output #1;
    output tx_start, data_in;
    input  rst, tx_busy, tx_done;
  endclocking
  clocking monitor_cb @(posedge clk);
    default input #1;
    input rst, tx_start, data_in, tx_busy, tx_done,
          tx_serial, rx_data, rx_done, rx_busy;
  endclocking
  // No modports: driver and monitor use plain 'virtual uart_if'
  // so direct signal access and clocking block access both work.
endinterface
// 3. TRANSACTION - uart_transaction
class uart_transaction;
  typedef enum logic [1:0] {
    SEND    = 2'b00,   // generator -> driver -> DUT tx
    RECEIVE = 2'b01     // monitor-created, from DUT rx
  } op_e;
  op_e           operation;
  rand bit [7:0] data;
  function uart_transaction copy();
    uart_transaction t = new();
    t.operation = this.operation;
    t.data      = this.data;
    return t;
  endfunction
  function void display(string tag = "TXN");
    $display("[%s] op=%-7s  data=0x%02h  @%0t",
             tag, operation.name(), data, $time);
  endfunction
endclass
// 4. GENERATOR - uart_generator
class uart_generator;
  uart_transaction            trans;
  mailbox #(uart_transaction) gen2drv;
  int unsigned                num_transactions;
  event                       gen_done;
  function new(mailbox #(uart_transaction) mbx,
               int unsigned n,
               ref event done);
    gen2drv          = mbx;
    num_transactions = n;
    gen_done         = done;
  endfunction
  task run();
    repeat (num_transactions) begin
      trans = new();
      if (!trans.randomize())
        $fatal(1, "[GENERATOR] Randomization failed");
      trans.operation = uart_transaction::SEND;
      trans.display("GEN");
      gen2drv.put(trans.copy());
    end
    $display("[GENERATOR] Done - %0d transactions queued", num_transactions);
    -> gen_done;
  endtask
endclass
// 5. DRIVER - uart_driver
class uart_driver;
  virtual uart_if              vif;
  mailbox #(uart_transaction)  gen2drv;
  function new(virtual uart_if vif,
               mailbox #(uart_transaction) mbx);
    this.vif     = vif;
    this.gen2drv = mbx;
  endfunction
  // FIX: drive rst directly (not through clocking block) so it
  //      asserts before the first clock edge
  task reset();
    vif.rst      = 1;
    vif.tx_start = 0;
    vif.data_in  = 0;
    repeat (4) @(posedge vif.clk);
    @(negedge vif.clk);
    vif.rst = 0;
    $display("[DRIVER] Reset deasserted @%0t", $time);
    @(posedge vif.clk);
  endtask
  task run();
    uart_transaction trans;
    forever begin
      gen2drv.get(trans);
      // wait until TX engine is free
      while (vif.driver_cb.tx_busy) @(vif.driver_cb);
      @(vif.driver_cb);
      vif.driver_cb.data_in  <= trans.data;
      vif.driver_cb.tx_start <= 1;
      trans.display("DRV-TX");
      @(vif.driver_cb);
      vif.driver_cb.tx_start <= 0;   // one-cycle start pulse
      // wait for this byte to fully finish transmitting before
      // grabbing the next one, so bytes never overlap
      @(vif.driver_cb);
      while (!vif.driver_cb.tx_done) @(vif.driver_cb);
    end
  endtask
endclass
// 6. MONITOR - uart_monitor
class uart_monitor;
  virtual uart_if              vif;
  mailbox #(uart_transaction)  mon2scb;
  function new(virtual uart_if vif,
               mailbox #(uart_transaction) mbx);
    this.vif     = vif;
    this.mon2scb = mbx;
  endfunction
  task run();
    uart_transaction trans;
    forever begin
      @(vif.monitor_cb);
      // ---- Observe a byte entering the TX engine ----
      if (vif.monitor_cb.tx_start && !vif.monitor_cb.rst) begin
        trans           = new();
        trans.operation = uart_transaction::SEND;
        trans.data      = vif.monitor_cb.data_in;
        trans.display("MON-TX");
        mon2scb.put(trans.copy());
      end
      // ---- Observe a byte completing on the RX engine ----
      if (vif.monitor_cb.rx_done && !vif.monitor_cb.rst) begin
        trans           = new();
        trans.operation = uart_transaction::RECEIVE;
        trans.data      = vif.monitor_cb.rx_data;
        trans.display("MON-RX");
        mon2scb.put(trans.copy());
      end
    end
  endtask
endclass
// 7. SCOREBOARD - uart_scoreboard
class uart_scoreboard;
  mailbox #(uart_transaction) mon2scb;
  bit [7:0]    ref_queue[$];
  int unsigned checks_passed;
  int unsigned checks_failed;
  function new(mailbox #(uart_transaction) mbx);
    mon2scb       = mbx;
    checks_passed = 0;
    checks_failed = 0;
  endfunction
  task run();
    uart_transaction trans;
    forever begin
      mon2scb.get(trans);
      case (trans.operation)
        uart_transaction::SEND: begin
          ref_queue.push_back(trans.data);
          $display("[SCB] EXPECT 0x%02h | pending=%0d",
                   trans.data, ref_queue.size());
        end
        uart_transaction::RECEIVE: begin
          if (ref_queue.size() == 0) begin
            $error("[SCB] FAIL - RX byte with no pending SEND @%0t", $time);
            checks_failed++;
          end else begin
            bit [7:0] expected = ref_queue.pop_front();
            if (trans.data === expected) begin
              $display("[SCB] PASS - got=0x%02h  exp=0x%02h", trans.data, expected);
              checks_passed++;
            end else begin
              $error("[SCB] FAIL - got=0x%02h  exp=0x%02h @%0t",
                     trans.data, expected, $time);
              checks_failed++;
            end
          end
        end
        default: ;
      endcase
    end
  endtask
  function void report();
    $display("==============================================");
    $display("  SCOREBOARD REPORT");
    $display("  PASSED : %0d", checks_passed);
    $display("  FAILED : %0d", checks_failed);
    if (checks_failed == 0)
      $display("  RESULT : ** ALL TESTS PASSED **");
    else
      $display("  RESULT : ** %0d TEST(S) FAILED **", checks_failed);
    $display("==============================================");
  endfunction
endclass
// 8. ENVIRONMENT - uart_env
class uart_env;
  uart_generator  gen;
  uart_driver     drv;
  uart_monitor    mon;
  uart_scoreboard scb;
  mailbox #(uart_transaction) gen2drv;
  mailbox #(uart_transaction) mon2scb;
  virtual uart_if vif;
  event            gen_done;
  // one full byte = (1 start + 8 data + 1 stop) * CLKS_PER_BIT cycles
  localparam int CLKS_PER_BIT   = 4;
  localparam int CYCLES_PER_BYTE = 10 * CLKS_PER_BIT;
  function new(virtual uart_if vif, int unsigned num_transactions = 20);
    this.vif = vif;
    gen2drv  = new();
    mon2scb  = new();
    gen = new(gen2drv, num_transactions, gen_done);
    drv = new(vif, gen2drv);
    mon = new(vif, mon2scb);
    scb = new(mon2scb);
  endfunction
  task run();
    // FIX: reset BEFORE forking stimulus threads
    drv.reset();
    fork
      gen.run();
      drv.run();   // killed after drain below
      mon.run();   // killed after drain below
      scb.run();   // killed after drain below
    join_none
    // wait for generator to finish, then drain remaining
    // transactions through driver -> DUT -> monitor -> scoreboard
    @(gen_done);
    $display("[ENV] Generator done, draining pipeline...");
    wait (gen2drv.num() == 0);
    // give enough cycles for the last byte(s) to fully serialize
    // through TX and RX plus a safety margin
    repeat (3 * CYCLES_PER_BYTE) @(posedge vif.clk);
    wait (mon2scb.num() == 0);
    repeat (5) @(posedge vif.clk);
    disable fork;  // cleanly kill forever loops
    scb.report();
  endtask
endclass
// 9. TEST - uart_test
class uart_test;
  uart_env        env;
  virtual uart_if vif;
  function new(virtual uart_if vif, int unsigned num_transactions = 20);
    this.vif = vif;
    env = new(vif, num_transactions);
  endfunction
  task run();
    $display("============================================");
    $display("  [TEST] UART TX/RX Verification Start");
    $display("============================================");
    env.run();
    $display("[TEST] Complete @%0t", $time);
  endtask
endclass
// 10. TESTBENCH TOP - tb_uart_top
module tb_uart_top;
  localparam int CLKS_PER_BIT = 4;
  // Clock
  logic clk;
  initial clk = 0;
  always #5 clk = ~clk;   // 100 MHz
  // Interface
  uart_if intf (.clk(clk));
  // DUT - TX
  uart_tx #(
    .CLKS_PER_BIT (CLKS_PER_BIT)
  ) dut_tx (
    .clk       (clk),
    .rst       (intf.rst),
    .tx_start  (intf.tx_start),
    .data_in   (intf.data_in),
    .tx_serial (intf.tx_serial),
    .tx_busy   (intf.tx_busy),
    .tx_done   (intf.tx_done)
  );
  // DUT - RX  (loopback: tx_serial feeds rx_serial directly)
  uart_rx #(
    .CLKS_PER_BIT (CLKS_PER_BIT)
  ) dut_rx (
    .clk       (clk),
    .rst       (intf.rst),
    .rx_serial (intf.tx_serial),
    .rx_data   (intf.rx_data),
    .rx_done   (intf.rx_done),
    .rx_busy   (intf.rx_busy)
  );
  // typedef fixes "token is ';'" error on parameterized class handle
  typedef uart_test uart_test_t;
  uart_test_t test_h;
  initial begin
    test_h = new(intf, 20);   // 20 randomized bytes
    test_h.run();
    $finish;
  end
  initial begin
    $dumpfile("uart_tx_rx.vcd");
    $dumpvars(0, tb_uart_top);
  end
  // Watchdog
  initial begin
    #100000;
    $display("[WATCHDOG] Timeout - check for deadlock");
    $finish;
  end
endmodule
