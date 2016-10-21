//	Author : Shawn.ye shawnye90@163.com
`timescale 1 ps / 1 ps

module cyusb3014_if(               
//Global reset                                                   
	input                     reset_n_sysclk                                       ,
	input                     sysclk                                               ,
	input                     reset_n_usbclk                                       ,
	input                     usb_clk_100m                                         ,
//USB data port interface                                                          
	inout     [31:0]          fifo_dq                                              ,
	output                    fifo_slcs_n                                          ,                          
	output                    fifo_slwr_n                                          ,                          
	output                    fifo_sloe_n                                          ,                          
	output                    fifo_slrd_n                                          ,                          
	input                     fifo_thread_dma0_ready                               ,   
	input                     fifo_thread_dma0_watermark                           ,                       
	input                     fifo_thread_dma1_ready                               , 
	input                     fifo_thread_dma1_watermark                           ,                         
	output                    fifo_pktend_n                                        ,                          
	output                    fifo_a0                                              ,                                           
	output                    fifo_a1                                              ,                                          
	output                    fifo_pclk                                            ,                         
	inout                     fifo_gpio                                            ,
	input                     fifo_a_fu3                                           ,                         
	output                    fifo_epswitch                                        ,                                              
	output                    fifo_int_ctl                                         , 
//interface to transmit and receive module                                         
	output  [7:0]             fifo_rd_data                                         , 
	output                    fifo_rd_valid                                        ,  
	output  [15:0]            fifo_rd_cnt                                          ,
	output                    fifo_rd_empty                                        ,
	input                     fifo_rd                                              , 
	input   [8:0]             fifo_wr_data                                         ,                                                                                      
	input                     fifo_wr                                              ,  
	output  [13:0]            fifo_wr_cnt                                          ,
	output                    fifo_wr_full                                         , 
//COMMAND OUTPUT	
	(* KEEP="TRUE" *)output	wire							datalog,
	(* KEEP="TRUE" *)output	wire							scope,
	(* KEEP="TRUE" *)output	wire							trigger,
	(* KEEP="TRUE" *)output	wire							cancel,
	(* KEEP="TRUE" *)output	wire							ctrl_signal_crc_err,
	(* KEEP="TRUE" *)output	wire	[63:0]					ctrl_signal_cmddata,
	(* KEEP="TRUE" *)output	wire	[7:0]					ctrl_signal_frameid,		
																					   
	output                    receive_led_enable                                   ,
	output                    transmit_led_enable                                  
);          
                                        

//signal definition                                                                
wire    [7:0]                 ingress_fifo_dout                                    ;
reg                           ingress_fifo_rd_en_1dly                              ; 
wire                          ingress_fifo_rd_en                                   ;
wire    [15:0]                 ingress_fifo_rd_cnt                                  ;
wire    [13:0]                 ingress_fifo_wr_cnt                                  ;
reg     [7:0]                 fifo_rd_data                                         ;
reg                           fifo_rd_valid                                        ;
reg     [35:0]                egress_fifo_din                                      ;
reg                           egress_fifo_wr_en                                    ;
wire    [13:0]                 egress_fifo_rd_cnt                                   ;
wire                          egress_fifo_rd_empty                                 ;
wire    [13:0]                 egress_fifo_wr_cnt                                   ;
reg     [1:0]                 egress_fifo_wr_round_robin                           ;
reg                           fifo_wr_full                                         ;
reg     [31:0]                ingress_fifo_din                                     ;
reg                           ingress_fifo_wr_en                                   ;
reg                           egress_fifo_rd_en                                    ;
wire    [35:0]                egress_fifo_dout                                     ;
reg     [6:0]                 next_state                                           ;
reg     [6:0]                 current_state                                        ;
reg     [1:0]                 read_wait_cnt                                        ;
(* KEEP="TRUE" *)  
reg     [1:0]                 fifo_address                                         ;
(* KEEP="TRUE" *) 
reg                           fifo_slrd_n_reg                                      ;
reg                           fifo_slrd_n_reg_1dly                                 ;
reg                           fifo_slrd_n_reg_2dly                                 ;
(* KEEP="TRUE" *)  
reg     [31:0]                fifo_dq_out_reg                                      ;   
reg                           fifo_slcs_n                                          ;
(* KEEP="TRUE" *)  
reg                           fifo_sloe_n                                          ;
(* KEEP="TRUE" *)  
reg                           fifo_slwr_n                                          ;  
(* KEEP="TRUE" *)    
reg                           fifo_pktend_n                                        ;
reg                           fifo_thread_dma1_watermark_1dly                      ;
reg     [31:0]                cyusb3014_write_wait_counter                         ;
reg     [31:0]                receive_led_enable_cnt                               ;
reg     [31:0]                transmit_led_enable_cnt                              ;
reg                           receive_led_enable                                   ;
reg                           transmit_led_enable                                  ;
(* KEEP="TRUE" *)wire	cmd_valid;
(* KEEP="TRUE" *)wire	crc_error;
(* KEEP="TRUE" *)wire	[7:0]	cmd;
(* KEEP="TRUE" *)wire	[7:0]	subcmd;
(* KEEP="TRUE" *)wire	[7:0]	frameid;
(* KEEP="TRUE" *)wire	[63:0]	cmd_data;

wire                          ingress_fifo_rd_empty                                ;
reg		[31:0]					cnt;
                                                                                   
parameter                     DLY = 1                                              ;
parameter                     CYUSB3014_WRITE_WAIT                 = 32'h00000050  ; //USB??????800ns
localparam                    st_SYNC3014_PREREAD                  = 7'b0000001    ;
localparam                    st_SYNC3014_READ_CHECK               = 7'b0000010    ;
localparam                    st_SYNC3014_READ                     = 7'b0000100    ;
localparam                    st_SYNC3014_READ_WAIT                = 7'b0001000    ;
localparam                    st_SYNC3014_PREWRITE                 = 7'b0010000    ;
localparam                    st_SYNC3014_WRITE_CHECK              = 7'b0100000    ;
localparam                    st_SYNC3014_WRITE                    = 7'b1000000    ;

asynfifo_w32_r8_64K u0_asynfifo_ingress_inst (
	.wr_clk                            (usb_clk_100m                             ), // input wr_clk                              
	.rd_clk                            (usb_clk_100m                                   ), // input rd_clk
//	.clk							   (usb_clk_100m),	
	.din                               (ingress_fifo_din                         ), // input [31 : 0] din                        
	.wr_en                             (ingress_fifo_wr_en                       ), // input wr_en                               
	.rd_en                             (ingress_fifo_rd_en                       ), // input rd_en                               
	.dout                              (ingress_fifo_dout                        ), // output [7 : 0] dout                       
	.full                              (                                         ), // output full                               
	.empty                             (ingress_fifo_rd_empty                    ), // output empty                              
	.rd_data_count                     (ingress_fifo_rd_cnt                      ), // output [9 : 0] rd_data_count              
	.wr_data_count                     (ingress_fifo_wr_cnt                      )  // output [7 : 0] wr_data_count              
);

fifo_4Byte2Byte	u1(
	.clk								(usb_clk_100m),
	.fifo_empty							(ingress_fifo_rd_empty),
	.rd_en								(ingress_fifo_rd_en)
);



usb_command_decoder	u2(
	.clk								(usb_clk_100m),
	//FIFO INTERFACE
	.ingress_fifo_dout					(ingress_fifo_dout),
	.ingress_fifo_rd_en					(ingress_fifo_rd_en),
	//COMMAND
	.cmd_valid							(cmd_valid),
	.crc_error							(crc_error),
	.cmd_reg							(cmd),
	.subcmd_reg							(subcmd),
	.frameid_reg						(frameid),
	.data_reg							(cmd_data)	

);


usb_cmd_processor	u3(
	.clk											(usb_clk_100m),
//COMMAND INPUT
	.cmd_valid										(cmd_valid),
	.crc_error										(crc_error),
	.cmd											(cmd),
	.subcmd											(subcmd),
	.frameid										(frameid),
	.cmd_data										(cmd_data),
//CONTROL SIGNALS
	.datalog										(datalog),
	.scope											(scope),
	.trigger										(trigger),
	.cancel											(cancel),
	.ctrl_signal_crc_err							(ctrl_signal_crc_err),
	.ctrl_signal_frameid							(ctrl_signal_frameid),
	.ctrl_signal_cmddata							(ctrl_signal_cmddata)

);                                                                                                             
   
                                                                                                                                                                                       
asynfifo_w36_r36_16K_t u0_asynfifo_egress_inst (
  .wr_clk                            (usb_clk_100m                                   ), // input wr_clk
  .rd_clk                            (usb_clk_100m                             ), // input rd_clk
//	.clk								(usb_clk_100m),
  .din                               (egress_fifo_din                          ), // input [35 : 0] din
  .wr_en                             (egress_fifo_wr_en                        ), // input wr_en
  .rd_en                             (egress_fifo_rd_en                        ), // input rd_en
  .dout                              (egress_fifo_dout                         ), // output [35 : 0] dout
  .full                              (                                         ), // output full
  .empty                             (egress_fifo_rd_empty                     ), // output empty
  .rd_data_count                     (egress_fifo_rd_cnt                       ), // output [9 : 0] rd_data_count
  .wr_data_count                     (egress_fifo_wr_cnt                       )  // output [9 : 0] wr_data_count
);



always @ (posedge sysclk or negedge reset_n_sysclk)         
begin
    if(reset_n_sysclk == 1'b0)
    begin
        receive_led_enable_cnt <= #DLY 32'b0;
    end    
    else if(ingress_fifo_rd_en == 1'b1)                 
    begin
    	  receive_led_enable_cnt <= #DLY 32'h05F5E100;
    end	 
    else if (receive_led_enable_cnt != 32'h0)
    begin
    	  receive_led_enable_cnt <= #DLY receive_led_enable_cnt - 1'b1;
    end
    else;
end

always @ (posedge sysclk or negedge reset_n_sysclk)         
begin
    if(reset_n_sysclk == 1'b0)
    begin
        receive_led_enable <= #DLY 1'b0;
    end     
    else if (receive_led_enable_cnt != 32'h0)
    begin
    	  receive_led_enable <= #DLY 1'b1;
    end
    else
    begin
    	  receive_led_enable <= #DLY 1'b0;
    end 	  
end

always @ (posedge sysclk or negedge reset_n_sysclk)         
begin
    if(reset_n_sysclk == 1'b0)
    begin
        transmit_led_enable_cnt <= #DLY 32'b0;
    end    
    else if(egress_fifo_wr_en == 1'b1)                 
    begin
    	  transmit_led_enable_cnt <= #DLY 32'h05F5E100;
    end	 
    else if (transmit_led_enable_cnt != 32'h0)
    begin
    	  transmit_led_enable_cnt <= #DLY transmit_led_enable_cnt - 1'b1;
    end
    else;
end

always @ (posedge sysclk or negedge reset_n_sysclk)         
begin
    if(reset_n_sysclk == 1'b0)
    begin
        transmit_led_enable <= #DLY 1'b0;
    end     
    else if (transmit_led_enable_cnt != 32'h0)
    begin
    	  transmit_led_enable <= #DLY 1'b1;
    end
    else
    begin
    	  transmit_led_enable <= #DLY 1'b0;
    end 	  
end



assign fifo_rd_cnt = ingress_fifo_rd_cnt;
assign fifo_rd_empty = ingress_fifo_rd_empty;  
assign fifo_wr_cnt  = egress_fifo_wr_cnt ;


always @ (posedge sysclk or negedge reset_n_sysclk) 
begin
    if(reset_n_sysclk == 1'b0)
    begin
        ingress_fifo_rd_en_1dly <= #DLY 1'b0;
    end    
    else 
    begin
        ingress_fifo_rd_en_1dly <= #DLY ingress_fifo_rd_en;
    end  
end

always @ (posedge sysclk or negedge reset_n_sysclk) 
begin
    if(reset_n_sysclk == 1'b0)
    begin
        fifo_rd_valid <= #DLY 1'b0;
    end    
    else 
    begin
        fifo_rd_valid <= #DLY ingress_fifo_rd_en_1dly;
    end  
end

       

always @ (posedge sysclk or negedge reset_n_sysclk) 
begin
    if(reset_n_sysclk == 1'b0)
    begin
        egress_fifo_wr_round_robin <= #DLY 2'b0;
    end    
    else if(fifo_wr == 1'b1) 
    begin
        if(fifo_wr_data[8] == 1'b1)
        begin
        	  egress_fifo_wr_round_robin <= #DLY 2'b0;
        end
        else
        begin
        	  egress_fifo_wr_round_robin <= #DLY egress_fifo_wr_round_robin + 1'b1;
        end
    end
    else;     	  	   
end  

always @ (posedge sysclk or negedge reset_n_sysclk) 
begin
    if(reset_n_sysclk == 1'b0)
    begin
        fifo_wr_full <= #DLY 1'b0;
    end    
    else if (egress_fifo_wr_cnt[13:6] == 8'b11111111)
    begin
        fifo_wr_full <= #DLY 1'b1;
    end
    else if (egress_fifo_wr_cnt[13:6] == 8'b11111100)
    begin
        fifo_wr_full <= #DLY 1'b0;
    end    
    else;
end

//usb_clock domain


always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        current_state <= #DLY st_SYNC3014_PREREAD;
    end    
    else 
    begin
        current_state <= #DLY next_state;
    end
end

always @ (*)   
begin
    case(current_state)
        st_SYNC3014_PREREAD : 
            next_state = st_SYNC3014_READ_CHECK;
        st_SYNC3014_READ_CHECK :
            if(fifo_thread_dma0_ready == 1'b0 && ingress_fifo_wr_cnt[13:7] <= 7'b1111110)  //???3014???????????FIFO???????????????3014????????FIFO
            begin 
            	  next_state = st_SYNC3014_READ;
            end
            else
            begin
            	  next_state = st_SYNC3014_PREWRITE;
            end
        st_SYNC3014_READ:   	  	     
           //if(fifo_thread_dma0_watermark == 1'b1 || ingress_fifo_wr_cnt[7] == 1'b1)  //???3014?????????FIFO???????????????????3014????????FIFO
           if(fifo_thread_dma0_ready == 1'b1 || ingress_fifo_wr_cnt[13:7] == 7'b1111111)  //???3014?????????FIFO???????????????????3014????????FIFO
           begin
           	  next_state = st_SYNC3014_READ_WAIT;
           end
           else
           begin
           	  next_state = st_SYNC3014_READ; 	
           end
        st_SYNC3014_READ_WAIT:
           if(read_wait_cnt == 2'b01)               //?????????????2 clock?????
           begin
           	 next_state = st_SYNC3014_PREWRITE;
           end
           else
           begin
           	 next_state = st_SYNC3014_READ_WAIT;
           end
        st_SYNC3014_PREWRITE:
           next_state = st_SYNC3014_WRITE_CHECK;  
        st_SYNC3014_WRITE_CHECK:
           if(fifo_thread_dma1_ready == 1'b0 &&  egress_fifo_rd_empty == 1'b0 && cyusb3014_write_wait_counter == 32'b0)
           begin
           	   next_state = st_SYNC3014_WRITE;
           end
           else
           begin
           	   next_state = st_SYNC3014_PREREAD;	
           end
        st_SYNC3014_WRITE:
           if(fifo_thread_dma1_watermark == 1'b0)
           begin
           	   if(egress_fifo_dout[32]== 1'b1)         //??????????????
           	   begin
           	   	   next_state = st_SYNC3014_PREREAD;
           	   end	   
           	   else if(egress_fifo_rd_empty == 1'b1)    //???????????
           	   begin
           	   	   next_state = st_SYNC3014_PREREAD;
           	   end
           	   else                                    //???????????
           	   begin
           	   	   next_state = st_SYNC3014_WRITE;
           	   end
          end
          else                                         //??FIFO?????
          begin
          	  next_state = st_SYNC3014_PREREAD;
          end 	   	   	   	   
       default:
          next_state = st_SYNC3014_PREREAD;   	       
   endcase
end     


always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        cyusb3014_write_wait_counter <= #DLY 32'b0;
    end         	   
    else 
    	if(current_state == st_SYNC3014_WRITE)	begin
			cyusb3014_write_wait_counter <= #DLY CYUSB3014_WRITE_WAIT;
   		end
    	else	begin
			if(cyusb3014_write_wait_counter != 32'b0)	begin
				cyusb3014_write_wait_counter <= #DLY cyusb3014_write_wait_counter - 1'b1;
			end
			else
				;    
    end
end

always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        read_wait_cnt <= #DLY 2'b0;
    end         	   
    else if(current_state == st_SYNC3014_READ_WAIT)
    begin
    	  read_wait_cnt <= #DLY read_wait_cnt + 1'b1;
    end
    else
    begin
    	  read_wait_cnt <= #DLY 2'b0;
    end
end

assign fifo_a0 = fifo_address[0];
assign fifo_a1 = fifo_address[1];

always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_address <= #DLY 2'b00;
    end         	   
    else if(current_state == st_SYNC3014_PREREAD)       //read ???
    begin
    	  fifo_address <= #DLY 2'b00;
    end
    else if(current_state == st_SYNC3014_PREWRITE)      //write ???
    begin
    	  fifo_address <= #DLY 2'b01;
    end
    else;
end

always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_slcs_n <= #DLY 1'b1;
    end         	   
    else 
    begin
    	  fifo_slcs_n <= #DLY 1'b0;
    end
end

always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_sloe_n <= #DLY 1'b1;
    end         	   
    else if(current_state == st_SYNC3014_PREREAD)       //read ???
    begin
    	  fifo_sloe_n <= #DLY 1'b0;
    end
    else if(current_state == st_SYNC3014_PREWRITE)       //write ???
    begin
    	  fifo_sloe_n <= #DLY 1'b1;
    end
    else; 	  
end    	  	          	   

assign fifo_slrd_n = fifo_slrd_n_reg;

always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_slrd_n_reg <= #DLY 1'b1;
    end         	   
    else if(current_state == st_SYNC3014_READ)       //read ???
    begin
    	  fifo_slrd_n_reg <= #DLY 1'b0;
    end
    else 
    begin
    	  fifo_slrd_n_reg <= #DLY 1'b1;
    end	  
end     


always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_slrd_n_reg_1dly <= #DLY 1'b1;
    end         	   
    else
    begin
    	  fifo_slrd_n_reg_1dly <= #DLY fifo_slrd_n_reg;
    end	  
end     

always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_slrd_n_reg_2dly <= #DLY 1'b1;
    end         	   
    else
    begin
    	  fifo_slrd_n_reg_2dly <= #DLY fifo_slrd_n_reg_1dly;
    end	  
end  

always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_thread_dma1_watermark_1dly <= #DLY 1'b0;
    end         	   
    else
    begin
    	  fifo_thread_dma1_watermark_1dly <= #DLY fifo_thread_dma1_watermark;
    end	  
end      

/*always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        ingress_fifo_wr_en <= #DLY 1'b0;
    end         	   
    else
    begin
    	  ingress_fifo_wr_en <= #DLY !fifo_slrd_n_reg_2dly;
    end	  
end   
*/
always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        ingress_fifo_wr_en <= #DLY 1'b0;
    end         	   
    else if(fifo_slrd_n_reg_2dly == 1'b0 && fifo_thread_dma0_ready ==1'b0) 
    begin
    	  ingress_fifo_wr_en <=  #DLY 1'b1;
    end
    else 
    begin
      	  ingress_fifo_wr_en <= #DLY 1'b0;
    end	  
end     

always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        ingress_fifo_din <= #DLY 32'b0;
    end         	   
    else
    begin
    	  ingress_fifo_din <= #DLY fifo_dq;
    end	  
end

   	    	         	 
/*always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_slwr_n <= #DLY 1'b1;
    end         	   
    else if(current_state == st_SYNC3014_WRITE && egress_fifo_rd_empty == 1'b0 && fifo_thread_dma1_watermark == 1'b0)       //read ???
    begin
    	  fifo_slwr_n <= #DLY 1'b0;
    end
    else if(current_state == st_SYNC3014_WRITE && fifo_thread_dma1_watermark == 1'b1)
    begin
          fifo_slwr_n <= #DLY 1'b0;
    end   
    else 
    begin
    	  fifo_slwr_n <= #DLY 1'b1;
    end	  
end  
*/
always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_slwr_n <= #DLY 1'b1;
    end         	   
    else if(current_state == st_SYNC3014_WRITE && egress_fifo_rd_empty == 1'b0)       //read ???
    begin
    	  fifo_slwr_n <= #DLY 1'b0;
    end   
    else 
    begin
    	  fifo_slwr_n <= #DLY 1'b1;
    end	  
end 

/*always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_pktend_n <= #DLY 1'b1;
    end         	   
    else if(current_state == st_SYNC3014_WRITE)       //read ???
    begin
    	  if(fifo_thread_dma1_watermark == 1'b0 && egress_fifo_dout[32]== 1'b1 && egress_fifo_rd_empty == 1'b0)
    	  begin
    	      fifo_pktend_n <= #DLY 1'b0;
    	  end
    	  else if(fifo_thread_dma1_watermark == 1'b1)
          begin
              fifo_pktend_n <= #DLY 1'b0;
          end
    	  else
    	  begin
    	  	  fifo_pktend_n <= #DLY 1'b1;
    	  end	      
    end
    else 
    begin
    	  fifo_pktend_n <= #DLY 1'b1;
    end	  
end 
*/
always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_pktend_n <= #DLY 1'b1;
    end         	   
    else if(current_state == st_SYNC3014_WRITE && egress_fifo_rd_empty == 1'b0)       //read ???
    begin
    	  if(fifo_thread_dma1_watermark == 1'b0 && egress_fifo_dout[32]== 1'b1)
    	  begin
    	      fifo_pktend_n <= #DLY 1'b0;
    	  end
    	  else if(fifo_thread_dma1_watermark == 1'b1)
          begin
              fifo_pktend_n <= #DLY 1'b0;
          end
    	  else
    	  begin
    	  	  fifo_pktend_n <= #DLY 1'b1;
    	  end	      
    end
    else 
    begin
    	  fifo_pktend_n <= #DLY 1'b1;
    end	  
end 

assign fifo_dq = (fifo_slwr_n == 1'b0)? fifo_dq_out_reg : 32'bz;

always @ (posedge usb_clk_100m or negedge reset_n_usbclk) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        fifo_dq_out_reg <= #DLY 32'b0;
    end         	   
    else if(current_state == st_SYNC3014_WRITE)       //read ???
    begin
        fifo_dq_out_reg <= #DLY egress_fifo_dout[31:0];
    end
    else 
    begin
    	  fifo_dq_out_reg <= #DLY 32'b0;
    end	  
end

assign fifo_pclk = usb_clk_100m;
assign fifo_gpio = 1'bz;
assign fifo_epswitch = 1'b1;
assign fifo_int_ctl = 1'b1;


always @ (*) 
begin
    if(reset_n_usbclk == 1'b0)
    begin
        egress_fifo_rd_en = 1'b0;
    end         	   
    else if(current_state == st_SYNC3014_WRITE)       //read ???
    begin
    	  //if(fifo_thread_dma1_watermark == 1'b0 && egress_fifo_rd_empty == 1'b0)
    	  if(egress_fifo_rd_empty == 1'b0)
    	  begin
    	      egress_fifo_rd_en = 1'b1;
    	  end
    	  else
    	  begin
    	  	  egress_fifo_rd_en = 1'b0;
    	  end	      
    end
    else 
    begin
    	  egress_fifo_rd_en = 1'b0;
    end	  
end          
   	     	           
endmodule
    
