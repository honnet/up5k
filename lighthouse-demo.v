/** \file
 * Print the lengths of timer pulses from the lighthouse sensors.
 */
`include "util.v"
`include "uart.v"
`include "lighthouse.v"

module top(
	input pin_1,  input pin_2,  input pin_3,  input pin_4,
	input pin_5,  input pin_6,  input pin_7,  input pin_8,

	input  pin_14, // serial_rxd,
	output pin_15, // serial_txd,

	output pin_led
);

	wire serial_rxd = pin_14;
	wire serial_txd = pin_15;

    // drive USB pull-up resistor to '0' to disable USB
    assign pin_pu = 0;

	// map the sensor
	parameter NUM_SENSORS = 16;
	wire [15:0] lighthouse_pin = {
        pin_1,  pin_2,  pin_3,  pin_4,
        pin_5,  pin_6,  pin_7,  pin_8
	};

	wire clk_48;
	wire reset = 0;
	SB_HFOSC u_hfosc (
		.CLKHFPU(1'b1),
		.CLKHFEN(1'b1),
		.CLKHF(clk_48)
	);

/*
	// pulse the green LED to know that we're alive
	reg [25:0] counter;
	always @(posedge clk_48)
		counter <= counter + 1;
	wire pwm_g;
	pwm pwm_g_driver(clk_48, 1, pwm_g);
	assign pin_led = !(counter[25:23] == 0 && pwm_g);
*/

	assign pin_led = serial_rxd && serial_txd;  // TODO: test

	// generate a 3 MHz/12 MHz serial clock from the 48 MHz clock
	// this is the 3 Mb/s maximum supported by the FTDI chip
	wire clk_1, clk_4;
	divide_by_n #(.N(16)) div1(clk_48, reset, clk_1);
	divide_by_n #(.N( 4)) div4(clk_48, reset, clk_4);

	wire [7:0] uart_rxd;
	wire uart_rxd_strobe;

	uart_rx rxd(
		.mclk(clk_48),
		.reset(reset),
		.baud_x4(clk_4),
		.serial(serial_rxd),
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
	);

	reg [7:0] uart_txd;
	reg uart_txd_strobe = 0;

	uart_tx_fifo #(.NUM(256)) txd(
		.clk(clk_48),
		.reset(reset),
		.baud_x1(clk_1),
		.serial(serial_txd),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe)
	);

	// output buffer
	parameter FIFO_WIDTH = 28;
	reg [FIFO_WIDTH-1:0] fifo_write;
	reg fifo_write_strobe;
	wire fifo_available;
	wire [FIFO_WIDTH-1:0] fifo_read;
	reg fifo_read_strobe;

	fifo #(.WIDTH(FIFO_WIDTH),.NUM(32)) timer_fifo(
		.clk(clk_48),
		.reset(reset),
		.data_available(fifo_available),
		.write_data(fifo_write),
		.write_strobe(fifo_write_strobe),
		.read_data(fifo_read),
		.read_strobe(fifo_read_strobe)
	);

	wire angle_strobe;
	wire lighthouse;
	wire axis;
	wire [3:0] sensor;
	wire [19:0] angle;

	wire data_strobe;
	wire data_out;
	reg data;

	lighthouse_sensor #(.SENSORS(NUM_SENSORS)) lh(
		.clk(clk_48),
		.reset(reset),
		.raw_pins(lighthouse_pin),

		.angle_strobe(angle_strobe),
		.sensor(sensor),
		.lighthouse(lighthouse),
		.axis(axis),
		.angle(angle),

		.data_strobe(data_strobe),
		.data(data_out)
	);

	always @(posedge clk_48)
	begin
		fifo_write_strobe <= 0;

		if (data_strobe)
			data <= data_out;

		if (angle_strobe) begin
			fifo_write_strobe <= 1;
			fifo_write <= {
				4'hA + sensor,
				2'b0,
				lighthouse,
				axis,
				3'b0,
				data,
				angle
			};
		end
	end


	reg [FIFO_WIDTH-1:0] out;
	reg [5:0] out_bytes;

	always @(posedge clk_48)
	begin
		uart_txd_strobe <= 0;
		fifo_read_strobe <= 0;

		// convert timer deltas to hex digits
		if (out_bytes != 0)
		begin
			uart_txd_strobe <= 1;
			out_bytes <= out_bytes - 1;
			if (out_bytes == 1)
				uart_txd <= "\r";
			else
			if (out_bytes == 2)
				uart_txd <= "\n";
			else
			if (out_bytes == 3+4)
				uart_txd <= " ";
			else begin
				uart_txd <= hexdigit(out[FIFO_WIDTH-1:FIFO_WIDTH-4]);
				out <= { out[FIFO_WIDTH-5:0], 4'b0 };
			end

		end else
		if (fifo_available && !fifo_write_strobe)
		begin
			out <= fifo_read;
			fifo_read_strobe <= 1;
			out_bytes <= 2 + 1 + FIFO_WIDTH/4;
		end
	end

endmodule
