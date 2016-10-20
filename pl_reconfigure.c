//Modified by Shawn.Ye. shawnye90@163.com

/******************************************************************************
*
* Copyright (C) 2010 - 2015 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*****************************************************************************/
/**
* @file  xdcfg_polled_example.c
*
* This file contains a polled mode design example for the Device Configuration
* Interface. This example downloads a given bitstream to the FPGA fabric.
*
* BIT_STREAM_LOCATION specifies the memory location of the bitstream.
* BIT_STREAM_SIZE_WORDS specifies the size of the bitstream in words.
* User has to define these correctly for this example to work.
*
* @note		None.
*
* MODIFICATION HISTORY:
*
*<pre>
* Ver   Who  Date     Changes
* ----- ---- -------- ---------------------------------------------
* 1.00a hvm  11/19/10 First release
* 1.00a nm   11/26/11 Holding FPGA in reset before download and
*                     releasing it after bitstream download. This code
*                     is not checking bitstream download errors.
* 2.00a nm   05/31/12 Updated the notes in the example for CR 660139 to add
*		      information that the 2 LSBs of the Source/Destination
*		      address when equal to 2�b01 indicate the last DMA command
*		      of an overall transfer.
* 		      Updated the example for CR 660835 so that input length for
*		      source/destination to the XDcfg_Transfer APIs is words
*		      (32 bit) and not bytes.
* 2.01a nm   11/21/12 Fixed CR# 688146. Modified the bitstream address.
* 2.02a nm   01/31/13 Fixed CR# 679335.
* 		      Removed disabling and enabling AXI interface.
*		      Clearing the interrupts before the transfer.
*		      Added support for partial reconfiguration.
* 3.00a kpc  02/20/14 Renamed the DcfgInstance variable name to DcfgInstPtr
* 3.1   kpc  04/22/14 Fixed CR#780203. Enable the pcap clock if it is not set.
*</pre>
******************************************************************************/

/***************************** Include Files *********************************/

#include "xparameters.h"
#include "xdevcfg.h"
#include "xil_cache.h"
#include "pl_reconfigure.h"

/************************** Constant Definitions *****************************/
/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are only defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define DCFG_DEVICE_ID		XPAR_XDCFG_0_DEVICE_ID

/*
 * The BIT_STREAM_LOCATION is a dummy address and BIT_STREAM_SIZE_WORDS is a
 * dummy size. This has to replaced with the actual location/size of the bitstream.
 *
 * The 2 LSBs of the Source/Destination address when equal to 2�b01 indicate
 * the last DMA command of an overall transfer.
 * The 2 LSBs of the BIT_STREAM_LOCATION in this example is set to 2b01
 * indicating that this is the last DMA transfer (and the only one).
 */
#define BIT_STREAM_LOCATION	0x20000001	/* Bitstream location */
#define BIT_STREAM_SIZE_WORDS	0xd6463		/* Size in Words (32 bit)*/

/*
 * SLCR registers
 */
#define SLCR_LOCK	0xF8000004 /**< SLCR Write Protection Lock */
#define SLCR_UNLOCK	0xF8000008 /**< SLCR Write Protection Unlock */
#define SLCR_LVL_SHFTR_EN 0xF8000900 /**< SLCR Level Shifters Enable */
#define SLCR_PCAP_CLK_CTRL XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x168 /**< SLCR
					* PCAP clock control register address
					*/

#define SLCR_PCAP_CLK_CTRL_EN_MASK 0x1
#define SLCR_LOCK_VAL	0x767B
#define SLCR_UNLOCK_VAL	0xDF0D

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/

int XDcfgPolledExample(XDcfg * DcfgInstance, u16 DeviceId);

/************************** Variable Definitions *****************************/

XDcfg DcfgInstance;		/* Device Configuration Interface Instance */



/*****************************************************************************/
/**
This function downloads the non-secure bitstream to FPGA using Device Config
Interface. It works even when PL is already programmed.

@param	DcfgInstPtr is a pointer to the instance of XDcfg driver.
@param	bitfile_addr is a pointer to the start address of the bitstream
		location.

@note	Remember to modify the BIT_STREAM_SIZE_BYTES marco defined in 
pl_reconfigure.h.The bitfile used should not be *.bit, it must be a bin file
generated by using write_cfgmem command in Vivado. Tested on Zynq 7z015.
****************************************************************************/
int Reconfig_PL(XDcfg *DcfgInstPtr,u32 bitfile_addr)
{
	int Status;
	u32 IntrStsReg = 0;
	u32 StatusReg;
	u32 PartialCfg = 0;

	XDcfg_Config *ConfigPtr;

	/*
	 * Initialize the Device Configuration Interface driver.
	 */
	ConfigPtr = XDcfg_LookupConfig(DCFG_DEVICE_ID);

	/*
	 * This is where the virtual address would be used, this example
	 * uses physical address.
	 */
	Status = XDcfg_CfgInitialize(DcfgInstPtr, ConfigPtr,
					ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	Status = XDcfg_SelfTest(DcfgInstPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Check first time configuration or partial reconfiguration
	 */
	IntrStsReg = XDcfg_IntrGetStatus(DcfgInstPtr);
	if (IntrStsReg & XDCFG_IXR_DMA_DONE_MASK) {
		PartialCfg = 1;
	}

	/*
	 * Enable the pcap clock.
	 */
	StatusReg = Xil_In32(SLCR_PCAP_CLK_CTRL);
	if (!(StatusReg & SLCR_PCAP_CLK_CTRL_EN_MASK)) {
		Xil_Out32(SLCR_UNLOCK, SLCR_UNLOCK_VAL);
		Xil_Out32(SLCR_PCAP_CLK_CTRL,
				(StatusReg | SLCR_PCAP_CLK_CTRL_EN_MASK));
		Xil_Out32(SLCR_UNLOCK, SLCR_LOCK_VAL);
	}

	/*
	 * Disable the level-shifters from PS to PL.
	 */
	if (!PartialCfg) {
		Xil_Out32(SLCR_UNLOCK, SLCR_UNLOCK_VAL);
		Xil_Out32(SLCR_LVL_SHFTR_EN, 0xA);
		Xil_Out32(SLCR_LOCK, SLCR_LOCK_VAL);
	}

	/*
	 * Select PCAP interface for partial reconfiguration
	 */
	if (PartialCfg) {
		XDcfg_EnablePCAP(DcfgInstPtr);
		XDcfg_SetControlRegister(DcfgInstPtr, XDCFG_CTRL_PCAP_PR_MASK);
	}

	/*
	 * Clear the interrupt status bits
	 */
	XDcfg_IntrClear(DcfgInstPtr, (XDCFG_IXR_PCFG_DONE_MASK |
					XDCFG_IXR_D_P_DONE_MASK |
					XDCFG_IXR_DMA_DONE_MASK));

	/* Check if DMA command queue is full */
	StatusReg = XDcfg_ReadReg(DcfgInstPtr->Config.BaseAddr,
				XDCFG_STATUS_OFFSET);
	if ((StatusReg & XDCFG_STATUS_DMA_CMD_Q_F_MASK) ==
			XDCFG_STATUS_DMA_CMD_Q_F_MASK) {
		return XST_FAILURE;
	}

	/********
	 *
	 * Reset PL to init State.
	 * ********/
//	Clear_PL(DcfgInstPtr);

	/*
	 * Download bitstream in non secure mode
	 */
	XDcfg_Transfer(DcfgInstPtr, (u8 *)bitfile_addr,
			BIT_STREAM_SIZE_WORDS,
			(u8 *)XDCFG_DMA_INVALID_ADDRESS,
			0, XDCFG_NON_SECURE_PCAP_WRITE);

	/* Poll IXR_DMA_DONE */
	IntrStsReg = XDcfg_IntrGetStatus(DcfgInstPtr);
	while ((IntrStsReg & XDCFG_IXR_DMA_DONE_MASK) !=
			XDCFG_IXR_DMA_DONE_MASK) {
		IntrStsReg = XDcfg_IntrGetStatus(DcfgInstPtr);
	}

	if (PartialCfg) {
		/* Poll IXR_D_P_DONE */
		while ((IntrStsReg & XDCFG_IXR_D_P_DONE_MASK) !=
				XDCFG_IXR_D_P_DONE_MASK) {
			IntrStsReg = XDcfg_IntrGetStatus(DcfgInstPtr);
		}
	}
	else {
		/* Poll IXR_PCFG_DONE */
		while ((IntrStsReg & XDCFG_IXR_PCFG_DONE_MASK) !=
				XDCFG_IXR_PCFG_DONE_MASK) {
			IntrStsReg = XDcfg_IntrGetStatus(DcfgInstPtr);
		}
		/*
		 * Enable the level-shifters from PS to PL.
		 */
		Xil_Out32(SLCR_UNLOCK, SLCR_UNLOCK_VAL);
		Xil_Out32(SLCR_LVL_SHFTR_EN, 0xF);
		Xil_Out32(SLCR_LOCK, SLCR_LOCK_VAL);
	}

	return XST_SUCCESS;
}


//This function clears the bitstream in PL.
int Clear_PL(XDcfg *InstancePtr)
{
	u32 CtrlReg;
	CtrlReg = XDcfg_ReadReg(InstancePtr->Config.BaseAddr,
				XDCFG_CTRL_OFFSET);

	XDcfg_WriteReg(InstancePtr->Config.BaseAddr,
				XDCFG_CTRL_OFFSET, (CtrlReg |
				  XDCFG_CTRL_PCFG_PROG_B_MASK));//set to 1
	XDcfg_WriteReg(InstancePtr->Config.BaseAddr,
			XDCFG_CTRL_OFFSET, (CtrlReg &
			  ~XDCFG_CTRL_PCFG_PROG_B_MASK));//set to 0
	while((XDcfg_ReadReg(InstancePtr->Config.BaseAddr, XDCFG_STATUS_OFFSET) & XDCFG_STATUS_PCFG_INIT_MASK) != 0)
		;
	XDcfg_WriteReg(InstancePtr->Config.BaseAddr,
				XDCFG_CTRL_OFFSET, (CtrlReg |
				  XDCFG_CTRL_PCFG_PROG_B_MASK));//set to 1
	while((XDcfg_ReadReg(InstancePtr->Config.BaseAddr, XDCFG_STATUS_OFFSET) & XDCFG_STATUS_PCFG_INIT_MASK) == 0)
		;
	return XST_SUCCESS;
}

int Check_bitfile(u8* startaddr)
{
	u32 i;
	u8 checksum;
	checksum = *startaddr;
	for (i=1;i<BIT_STREAM_SIZE_BYTES-2;i++)	{
		checksum ^= startaddr[i];
	}
	//check that if the first 4 bytes of the bitstream are all 0xff
	if ((startaddr[BIT_STREAM_SIZE_BYTES-1] == checksum)
			& (startaddr[0] == 0xff) & (startaddr[1] == 0xff)
			& (startaddr[2] == 0xff) & (startaddr[3] == 0xff))
		return XST_SUCCESS;
	else
		return XST_FAILURE;
}







