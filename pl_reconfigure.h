/*
 * pl_reconfigure.h
 *
 *  Created on: 2016Äê1ÔÂ5ÈÕ
 *      Author: shuang.ye
 */
#include "xdevcfg.h"

#ifndef PL_RECONFIGURE_H_
#define PL_RECONFIGURE_H_

#define PL_BITFILE_ADDR_0 0x1e000000
#define PL_BITFILE_ADDR_1 0x1e400000
#define PL_BITFILE_ADDR_2 0x1e800000
#define PL_BITFILE_ADDR_3 0x1eC00000
#define PL_BITFILE_ADDR_4 0x1F000000
#define PL_BITFILE_ADDR_5 0x1F400000
#define PL_BITFILE_ADDR_6 0x1F800000
#define PL_BITFILE_ADDR_7 0x1FC00000

#define BIT_STREAM_SIZE_WORDS 0xd6463
#define BIT_STREAM_SIZE_BYTES 0x35918d

int Reconfig_PL(XDcfg* DcfgInstance,u32 bitfile_addr);
int Clear_PL(XDcfg *InstancePtr);
int Check_bitfile(u8* startaddr);


#endif /* PL_RECONFIGURE_H_ */
