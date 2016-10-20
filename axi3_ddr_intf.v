`timescale 1ns / 1ps
//	Author : Shawn.Ye shawnye90@163.com
//	This module will do a self check on specified partition of DDR3_BASE_ADDR ~ DDR3_END_ADDR
//	If any error detected , ddr_init_fail will be asserted after the operation is done.
//	After self check , any data in FIFO will be moved to DDR3 using AXI3 interface directly.
//	The burst length will be generated automatically. If it meets the end of the partition , 
//	it will jump to the start and overwrite. So make sure to read the data before any 
//	overwrite occurs.

//FIFO TO AXI DDR INTERFACE WITH ADAPTIVE BURST LENGTH

module ddr_axi_interface#(
	parameter DDR3_BASE_ADDR  		= 32'h3000_0000,
	parameter DDR3_END_ADDR   		= 32'h4000_0000
)
(
	input wire						clk,
	input wire						rstn,
//FIFO INTERFACE
	input 	wire	[31:0]			fifo_data,
	output 	wire					fifo_rd,
	input 			[10:0]			fifo_rdcount,
//MASTER AXI INTERFACE
	input 	wire					m_axi_arready,	
	output 	wire	[31:0]			m_axi_awaddr,	
	output 	reg		[5:0]			m_axi_awid,
	input 	wire					m_axi_awready,
	output 	reg						m_axi_awvalid,
	input 			[5:0]			m_axi_bid,
	input 			[1:0]			m_axi_bresp,
	input 	wire					m_axi_bvalid,
	input wire[31:0]				m_axi_rdata,
	input wire[5:0]					m_axi_rid,
	input wire						m_axi_rlast,
	input wire[1:0]					m_axi_rresp,
	input wire						m_axi_rvalid,
	output wire	[31:0]				m_axi_wdata,
	output reg	[5:0]				m_axi_wid ,
	output reg						m_axi_wlast ,
	input wire						m_axi_wready,
	output reg						m_axi_wvalid,
	
	output wire	[3:0]				m_axi_wstrb,//
	output wire	[1:0]				m_axi_arburst,//
	output wire	[3:0]				m_axi_arcache,//
	output wire	[3:0]				m_axi_arlen,//
	output wire	[1:0]				m_axi_arlock,//
	output wire	[2:0]				m_axi_arprot,//
	output wire	[3:0]				m_axi_arqos,//
	output wire	[2:0]				m_axi_arsize,//
	output wire	[1:0]				m_axi_awburst,//
	output wire [3:0]				m_axi_awcache,//
	output reg	[3:0]				m_axi_awlen,
	output wire	[1:0]				m_axi_awlock,
	output wire	[2:0]				m_axi_awprot,
	output wire	[3:0]				m_axi_awqos,
	output wire	[2:0]				m_axi_awsize,
	output wire						m_axi_bready,
	output wire						m_axi_rready,
(*dont_touch = "TRUE"*)	output reg	[31:0]				m_axi_araddr = DDR3_BASE_ADDR - 64,
	output reg	[5:0]				m_axi_arid,
	output reg						m_axi_arvalid,
//DDR INIT STATUS INDICATOR 
	output reg						ddr_init_busy = 0,
	output reg						ddr_init_finish = 0,
	output reg						ddr_init_fail = 0,
//PS CONTROLLER INTERFACE
	input 							reset_addr,
	output reg						axi_busy
    );


//LOCAL PARAMETERS
localparam BURST_LENGTH 			= 4'd15;//burst len = 16
localparam STATE_TOTAL_NUMBER 		= 11;


localparam STATE_INIT_IDLE     		= 11'b000_0000_0000;
localparam STATE_INIT_WR_AID   		= 11'b000_0000_0001;
localparam STATE_INIT_WR_DATA  		= 11'b000_0000_0010;
localparam STATE_INIT_BRESP    		= 11'b000_0000_0100;
localparam STATE_INIT_END      		= 11'b000_0000_1000;
localparam STATE_IDLE          		= 11'b000_0001_0000;
localparam STATE_PRE_WR        		= 11'b000_0010_0000;
localparam STATE_WR_AID        		= 11'b000_0100_0000;
localparam STATE_WR_DATA       		= 11'b000_1000_0000;
localparam STATE_WR_BRESP      		= 11'b001_0000_0000;
localparam STATE_WR_END        		= 11'b010_0000_0000;
localparam STATE_RST_PLADDR    		= 11'b100_0000_0000;

localparam RDSTATE_IDLE				= 2'b00;
localparam ASSERT_ARVALID			= 2'b01;
localparam ARVALID_DELAY			= 2'b11;

    
reg [511:0] 						internal_cache;
reg [STATE_TOTAL_NUMBER-1:0] 		current_state,next_state;
reg [1:0]							current_state_rd,next_state_rd;
(*keep = "TRUE"*)reg [3:0] 			data_cnt = 0;
reg [3:0] 							data_cnt_rd;
reg [25:0] 							int_addr;
(*dont_touch = "TRUE"*)reg [25:0]	int_addr_rd;
wire	[31:0]	 					int_rdaddr;
//reg bvalid_cnt;
reg 								L2_cache_ready;
reg [511:0] 						L1_cache , L2_cache;
reg [4:0]							max_burst_len;
reg									fifo_rd_reg = 0;
reg [31:0]							int_m_axi_wdata;

reg	[1:0]							rd_cs,rd_ns;
wire								bvalid_empty;



assign m_axi_wdata					= fifo_rd_reg ? fifo_data : int_m_axi_wdata;
assign m_axi_wstrb  				= 4'b1111;
assign m_axi_arburst 				= 2'b01;//burst type : incr
assign m_axi_arcache 				= 4'b0011;
assign m_axi_arlock					= 2'b00;
assign m_axi_arprot					= 3'b000;
assign m_axi_arqos 					= 4'b0;
assign m_axi_arsize 				= 3'b010;
assign m_axi_awburst				= 2'b01;
assign m_axi_awcache				= 4'b0011;
assign m_axi_awlock					= 2'b0;
assign m_axi_awprot					= 3'b0;
assign m_axi_awqos					= 4'b0;
assign m_axi_awsize 				= 3'b010;
assign m_axi_bready					= 1'b1;
assign m_axi_rready					= 1'b1;
//assign m_axi_awlen 					= 4'b0;//WRITE BURST LENGTH
assign m_axi_arlen 					= 4'hf;//READ BURST LENGTH


assign m_axi_awaddr 				= {DDR3_BASE_ADDR[31:28],int_addr,2'b00};//4 bytes.
//assign m_axi_araddr 				= {4'b0011,int_addr_rd,2'b00};
assign int_rdaddr					= {DDR3_BASE_ADDR[31:28],int_addr_rd,2'b00};
//assign max_burst_len				= (fifo_rdcount >= 16) ? 16 : {0 , fifo_rdcount[3:0]};
assign fifo_rd 						= fifo_rd_reg ? m_axi_wready : 1'b0;



always @ (posedge clk)	begin
	if (fifo_rdcount >= 16)
		max_burst_len <= 16;
	else
		max_burst_len <= {1'b0,fifo_rdcount[3:0]};
end

///////////////////////////////////////////////WRITE CHANNEL STATE MACHINE
always @ (*)	begin
	if (!rstn)
		next_state = STATE_INIT_IDLE;
	else		begin
		case (current_state)
		STATE_INIT_IDLE:
			next_state = STATE_INIT_WR_AID ;
		STATE_INIT_WR_AID:
			next_state = m_axi_awready ? STATE_INIT_WR_DATA : STATE_INIT_WR_AID;
		STATE_INIT_WR_DATA:
			next_state = (data_cnt == BURST_LENGTH) ? STATE_INIT_END : STATE_INIT_WR_DATA;
		STATE_INIT_BRESP:
			next_state = (m_axi_bvalid) ? STATE_INIT_END : STATE_INIT_BRESP;
		STATE_INIT_END:
//			next_state = (int_addr == 26'h3fffff0)? STATE_IDLE :STATE_INIT_WR_AID;
			next_state = (int_addr == 26'h3ff_fff0)? STATE_IDLE :STATE_INIT_WR_AID;
		STATE_IDLE:
//				next_state = reset_addr ? STATE_RST_PLADDR : (1'b0 ? STATE_PRE_WR : STATE_IDLE) 
			next_state = (fifo_rdcount != 0)? STATE_PRE_WR:(reset_addr ? STATE_RST_PLADDR :STATE_IDLE);
		STATE_PRE_WR://calculate burst length.
			next_state = STATE_WR_AID;
		STATE_WR_AID:
			next_state = m_axi_awready ? STATE_WR_DATA : STATE_WR_AID; 
		STATE_WR_DATA:
			next_state = (data_cnt == m_axi_awlen) ? STATE_WR_BRESP : STATE_WR_DATA; 
		STATE_WR_BRESP:
			next_state = (m_axi_bvalid) ? (STATE_WR_END) : STATE_WR_BRESP;
		STATE_WR_END:
			next_state = (fifo_rdcount != 0) ? STATE_PRE_WR : STATE_IDLE;
		STATE_RST_PLADDR:
			next_state = (!reset_addr) ? STATE_IDLE : STATE_RST_PLADDR;
		default:
			next_state = STATE_INIT_IDLE;	
		endcase
	end
end

always @ (posedge clk)	begin
	case (current_state)
		STATE_INIT_IDLE:	begin
			ddr_init_busy <= 1;
			int_addr <= 0;
			m_axi_awlen <= 4'hf;
		end
		STATE_INIT_WR_AID:
			m_axi_awvalid <= 1'b1;
		STATE_INIT_WR_DATA:	begin
			m_axi_awvalid <= 1'b0;
			int_m_axi_wdata <= m_axi_awaddr + data_cnt;
			m_axi_wvalid <= 1'b1;
			if (m_axi_wready)	begin
				if (data_cnt == BURST_LENGTH)
					m_axi_wlast <= 1'b1;
				else
					m_axi_wlast <= 1'b0;
				data_cnt <= data_cnt + 1;
			end				
			else
				data_cnt <= data_cnt;
		end
		STATE_INIT_BRESP:	begin
			;
		end
		STATE_INIT_END:	begin
			m_axi_wlast <= 1'b0;
			m_axi_wvalid <= 1'b0;
			int_addr <= int_addr + 16;
			data_cnt <= 0;
			m_axi_awid <= m_axi_awid + 1;
			m_axi_wid <= m_axi_wid + 1;
		end
	//NORMAL OPERATION
		STATE_IDLE:	begin
			axi_busy <= 1'b0;
			ddr_init_busy <= 0;
		end
		STATE_PRE_WR:	begin //int_addr : width 26
			if (max_burst_len + {1'b0,int_addr} >= 27'h400_0000)
//				m_axi_awlen <= 26'h3ffffff - int_addr;
				m_axi_awlen <= 26'h3ff_ffff - int_addr;
			else
				m_axi_awlen <= max_burst_len - 1;	
			axi_busy <= 1'b1;
			m_axi_awid <= m_axi_awid + 1;
			m_axi_wid <= m_axi_wid + 1;
		end
		STATE_WR_AID:	begin
			m_axi_awvalid <= 1'b1;
		end
		STATE_WR_DATA:	begin
			fifo_rd_reg <= 1'b1;
			m_axi_awvalid <= 1'b0;
//			m_axi_wdata <= fifo_data;
			m_axi_wvalid <= 1'b1;
			if (m_axi_wready)	begin
				int_addr <= int_addr + 1;
				if (data_cnt == m_axi_awlen)
					m_axi_wlast <= 1'b1;
				else
					m_axi_wlast <= 1'b0;
				data_cnt <= data_cnt + 1;
			end				
			else
				data_cnt <= data_cnt;
		end		
		STATE_WR_BRESP:	begin
			fifo_rd_reg <= 1'b0;
			m_axi_wlast <= 1'b0;
			m_axi_wvalid <= 1'b0;
		end
		STATE_WR_END:	begin
			data_cnt <= 0;
		end		
		STATE_RST_PLADDR:
//			int_addr <= 0;
//			int_addr <= 26'h3fffff8;
			int_addr <= 26'h3ff_fff8;
		default:
			;
	endcase
end

///////////////////////////////////////////////READ CHANNEL STATE MACHINE
/*
It seems that axi slave port can return 2 or more cycles of bvalid continuously , and each cycle 
stands for different AXI_WID.So we abandon state machine.For a state machine cannot handle 2 or more cycles
of bvalid signal continuously.
This process has a RISK  -- it assumes ARREADY is always asserted.However,this happens pretty rarely.
In fact never happend so far.
*/



bvalid_fifo u0(
	.clk(clk),
	.din(1'b0),
	.wr_en(m_axi_bvalid),
	.rd_en(m_axi_arvalid),
	.empty(bvalid_empty)
);

always @ (posedge clk or negedge rstn)	begin
	if (!rstn)
		rd_cs <= RDSTATE_IDLE;
	else
		rd_cs <= rd_ns;
end

always @ (*)	begin
	if (!rstn)
		rd_ns = RDSTATE_IDLE;
	else
		case (rd_cs)
		RDSTATE_IDLE:
			rd_ns = (bvalid_empty) ? RDSTATE_IDLE : ASSERT_ARVALID;
		ASSERT_ARVALID:
			rd_ns = m_axi_arready ? ARVALID_DELAY : ASSERT_ARVALID;
		ARVALID_DELAY:
			rd_ns = RDSTATE_IDLE;	
		endcase
end

always @ (posedge clk)	begin
	case (rd_cs)
	RDSTATE_IDLE:
		m_axi_arvalid <= 1'b0;
	ASSERT_ARVALID:
		if (m_axi_arready)	begin
			m_axi_arvalid <= 1'b1;
			m_axi_arid <= m_axi_arid + 1;
			m_axi_araddr <= m_axi_araddr +64;
		end
		else	begin
			m_axi_araddr <= m_axi_araddr;
			m_axi_arid <= m_axi_arid;
			m_axi_arvalid <= 1'b0;
		end
	ARVALID_DELAY:
		m_axi_arvalid <= 1'b0;
	default:
		;
	endcase
end



always @ (posedge clk)	begin
	if (m_axi_rvalid)	begin
		data_cnt_rd <= data_cnt_rd + 1;
		if (m_axi_rdata == int_rdaddr + data_cnt_rd)
			;
		else
			ddr_init_fail <= 1;
	end
	else
		data_cnt_rd <= data_cnt_rd;
end
		

always @ (posedge clk)	begin
	if (m_axi_rlast)
		if (26'h3ff_fff0 == int_addr_rd)
			ddr_init_finish <= 1'b1;
		else	begin
			int_addr_rd <= int_addr_rd + 16;
			ddr_init_finish <= 0;
		end
	else	begin
		int_addr_rd <= int_addr_rd;
//		ddr_init_finish <= 0;
	end
end


		

always @ (posedge clk or negedge rstn) 	begin
	if(!rstn)
		current_state <= STATE_INIT_IDLE;
//		current_state <= STATE_IDLE;
	else
		current_state <= next_state;
end
endmodule
