/*
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

module hardware (
    input clk_16mhz,

    // onboard USB interface
    output pin_pu,
    output pin_usbp,
    output pin_usbn,

    // hardware UART
    output UART_TX,
    input UART_RX,

    // onboard SPI flash interface
    output flash_csb,
    output flash_clk,
    inout  flash_io0,
    inout  flash_io1,
    inout  flash_io2,
    inout  flash_io3,

    //////////////////////////////////
    // User inputs and outputs
    //
    input UP_DOWN, // HIGH = Count Up (send to GPIO)
    input START_STOP, // HIGH = start counting (send to GPIO)

    output [3:0] DBG,  // 4 debug leds on breadboard
    output user_led,   // onboard LED
    output [3:0] COMM, // 4 common cathodes
    output COLON, // colon on display board
    output [6:0] SEG   // seven segments 6=g, 5=f, ... 0=a
    //               (no comma on the last one)
    //////////////////////////////////
);
    assign pin_pu = 1'b1;
    assign pin_usbp = 1'b0;
    assign pin_usbn = 1'b0;

    wire clk = clk_16mhz;


    ///////////////////////////////////
    // Power-on Reset
    ///////////////////////////////////
    reg [5:0] reset_cnt = 0;
    wire resetn = &reset_cnt;

    always @(posedge clk) begin
        reset_cnt <= reset_cnt + !resetn;
    end


    ///////////////////////////////////
    // SPI Flash Interface
    ///////////////////////////////////
    wire flash_io0_oe, flash_io0_do, flash_io0_di;
    wire flash_io1_oe, flash_io1_do, flash_io1_di;
    wire flash_io2_oe, flash_io2_do, flash_io2_di;
    wire flash_io3_oe, flash_io3_do, flash_io3_di;

    SB_IO #(
        .PIN_TYPE(6'b 1010_01),
        .PULLUP(1'b 0)
    ) flash_io_buf [3:0] (
        .PACKAGE_PIN({flash_io3, flash_io2, flash_io1, flash_io0}),
        .OUTPUT_ENABLE({flash_io3_oe, flash_io2_oe, flash_io1_oe, flash_io0_oe}),
        .D_OUT_0({flash_io3_do, flash_io2_do, flash_io1_do, flash_io0_do}),
        .D_IN_0({flash_io3_di, flash_io2_di, flash_io1_di, flash_io0_di})
    );



    ///////////////////////////////////
    // Peripheral Bus
    ///////////////////////////////////
    wire        iomem_valid;
    reg         iomem_ready;
    wire [3:0]  iomem_wstrb;
    wire [31:0] iomem_addr;
    wire [31:0] iomem_wdata;
    reg  [31:0] iomem_rdata;

    ///////////////////////////
    // GPIO mapping
    //
    reg [31:0] gpio; // register declaration for GPIO.
    //
    // Starter:
    assign DBG = gpio[31:28];
    assign user_led = gpio[4];
/// sec is the 16 bit value we read from the C program
    reg [15:0] sec = gpio[15:0];
    //

//////////////////////////////////////////////
///
/// Display Interface
///
///

// Initialize common anode signals
// ACTIVE LOW
reg [3:0] comm;
assign COMM = comm; // tie to output

// Declare segment signals
// ACTIVE HIGH
reg [6:0] seg;
assign SEG = seg; // tie to output

//assign colon to second_toggle
assign COLON = second_toggle;


/// second timer signals
localparam [23:0] timer_init = 24'hF423FF;
reg [23:0] second_timer_state;
reg second_toggle;

wire start_stop = START_STOP;
wire [31:0]  read_data;
assign read_data = ((second_toggle & 32'b001) | ((UP_DOWN << 1) & 32'b010) | ((start_stop << 2) & 32'b0100));
/// preload second timer state machine
initial begin
  second_timer_state = timer_init;
  second_toggle = 1;
end /// end timer state initial begin

/// second_timer
/// second timer state machine
/// generates a tick every second
  always @(posedge clk) begin
      if (second_timer_state == 0) begin
        second_timer_state = timer_init;
        second_toggle <= ~second_toggle;
      end /// end if
      else begin
        second_timer_state <= second_timer_state - 1;
        end /// end else
  end /// second timer state machine

/// Display refresh signals
/// 2 bit signal to track displayed digit
reg [1:0] dd = 2'b11; // dd range 0-3
/// current hex digit
reg [3:0] hex_to_display;

/// refresh timer signals
localparam [23:0] refresh_init = 24'h000F00;
reg [23:0] refresh_timer_state;
reg refresh_tick;

// preload refresh state machine
initial begin
  refresh_timer_state = refresh_init;
  refresh_tick = 1;
end // end refresh timer initial begin


/// refresh timer state machine
/// generates a tick roughly every ms
  always @(posedge clk) begin
      if (refresh_timer_state == 0) begin
        refresh_timer_state = refresh_init;
        refresh_tick = 1;
      end /// end if
      else begin
        refresh_timer_state <= refresh_timer_state - 1;
        refresh_tick = 0;
      end /// end else
  end /// refresh timer state machine

  /// display digit state machine
  /// follows team pattern
  always @(posedge refresh_tick) begin
      case(dd)
          2'b00   : dd = 2'b11;
          2'b01   : dd = 2'b00;
          2'b10   : dd = 2'b01;
          2'b11   : dd = 2'b10;
          default : dd = 2'b00;
      endcase
  end /// display_digit state machine

///////////////////////////////////////////////////
/// Combinational circuits start here
///////////////////////////////////////////////////
  always @ ( * ) begin

/// drives the 'one hot' common cathodes
case(dd)
    2'b00   : comm = 4'b1110;
    2'b01   : comm = 4'b1101;
    2'b10   : comm = 4'b1011;
    2'b11   : comm = 4'b0111;
    default : comm = 4'b1110;
endcase

/// selects the hex digit for the currently displayed digit
      case(dd)
          2'b00   : hex_to_display = sec[15:12]; // 4'b0001;
          2'b01   : hex_to_display = sec[11:8]; // 4'b0010;
          2'b10   : hex_to_display = sec[7:4]; // 4'b0100;
          2'b11   : hex_to_display = sec[3:0]; // 4'b1000;
          default : hex_to_display = 4'b0000;
      endcase



      case(hex_to_display) // upside down
        4'b0000   : seg = 7'b0111111;
        4'b0001   : seg = 7'b0110000;
        4'b0010   : seg = 7'b1011011;
        4'b0011   : seg = 7'b1111001;
        4'b0100   : seg = 7'b1110100;
        4'b0101   : seg = 7'b1101101;
        4'b0110   : seg = 7'b1101111;
        4'b0111   : seg = 7'b0111000;
        4'b1000   : seg = 7'b1111111;
        4'b1001   : seg = 7'b1111100;
        4'b1010   : seg = 7'b1111110;
        4'b1011   : seg = 7'b1100111;
        4'b1100   : seg = 7'b0001111;
        4'b1101   : seg = 7'b1110011;
        4'b1110   : seg = 7'b1001111;
        4'b1111   : seg = 7'b1001110;
        default   : seg = 7'b0000000;
      endcase

  end /// - ends the combinational circuits


/// Processor hardware starts here
    always @(posedge clk) begin
        if (!resetn) begin
            gpio <= 0;
        end else begin
            iomem_ready <= 0;

            ///////////////////////////
            // GPIO Peripheral
            ///////////////////////////
            if (iomem_valid && !iomem_ready && iomem_addr[31:24] == 8'h03) begin
                iomem_ready <= 1;
                iomem_rdata <= read_data;
                if (iomem_wstrb[0]) gpio[ 7: 0] <= iomem_wdata[ 7: 0];
                if (iomem_wstrb[1]) gpio[15: 8] <= iomem_wdata[15: 8];
                if (iomem_wstrb[2]) gpio[23:16] <= iomem_wdata[23:16];
                if (iomem_wstrb[3]) gpio[31:24] <= iomem_wdata[31:24];
            end


            ///////////////////////////
            // Template Peripheral
            ///////////////////////////
            if (iomem_valid && !iomem_ready && iomem_addr[31:24] == 8'h04) begin
                iomem_ready <= 1;
                iomem_rdata <= 32'h0;
            end
        end
    end

    picosoc #(
        .PROGADDR_RESET(32'h0005_0000), // beginning of user space in SPI flash
        .PROGADDR_IRQ(32'h0005_0010),
        .MEM_WORDS(2048)                // use 2KBytes of block RAM by default
    ) soc (
        .clk          (clk         ),
        .resetn       (resetn      ),

        .ser_tx       (UART_TX       ),
        .ser_rx       (UART_RX      ),

        .flash_csb    (flash_csb   ),
        .flash_clk    (flash_clk   ),

        .flash_io0_oe (flash_io0_oe),
        .flash_io1_oe (flash_io1_oe),
        .flash_io2_oe (flash_io2_oe),
        .flash_io3_oe (flash_io3_oe),

        .flash_io0_do (flash_io0_do),
        .flash_io1_do (flash_io1_do),
        .flash_io2_do (flash_io2_do),
        .flash_io3_do (flash_io3_do),

        .flash_io0_di (flash_io0_di),
        .flash_io1_di (flash_io1_di),
        .flash_io2_di (flash_io2_di),
        .flash_io3_di (flash_io3_di),

        .irq_5        (1'b0        ),
        .irq_6        (1'b0        ),
        .irq_7        (1'b0        ),

        .iomem_valid  (iomem_valid ),
        .iomem_ready  (iomem_ready ),
        .iomem_wstrb  (iomem_wstrb ),
        .iomem_addr   (iomem_addr  ),
        .iomem_wdata  (iomem_wdata ),
        .iomem_rdata  (iomem_rdata )
    );
endmodule
