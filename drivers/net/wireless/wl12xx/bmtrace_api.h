/***************************************************************************
**+----------------------------------------------------------------------+**
**|                                ****                                  |**
**|                                ****                                  |**
**|                                ******o***                            |**
**|                          ********_///_****                           |**
**|                           ***** /_//_/ ****                          |**
**|                            ** ** (__/ ****                           |**
**|                                *********                             |**
**|                                 ****                                 |**
**|                                  ***                                 |**
**|                                                                      |**
**|     Copyright (c) 1998 - 2009 Texas Instruments Incorporated         |**
**|                        ALL RIGHTS RESERVED                           |**
**|                                                                      |**
**| Permission is hereby granted to licensees of Texas Instruments       |**
**| Incorporated (TI) products to use this computer program for the sole |**
**| purpose of implementing a licensee product based on TI products.     |**
**| No other rights to reproduce, use, or disseminate this computer      |**
**| program, whether in part or in whole, are granted.                   |**
**|                                                                      |**
**| TI makes no representation or warranties with respect to the         |**
**| performance of this computer program, and specifically disclaims     |**
**| any responsibility for any damages, special or consequential,        |**
**| connected with the use of this program.                              |**
**|                                                                      |**
**+----------------------------------------------------------------------+**
***************************************************************************/

/** \file   bmtrace_api.h 
 *  \brief  bmtrace performance tracing module API definition                                  
 *
 *  \see    bmtrace.c
 */

#ifndef __BM_TRACE_API_H
#define __BM_TRACE_API_H

void            bm_enable(void);
void            bm_disable(void);
unsigned long   bm_act_trace_in(void);
void            bm_act_trace_out(int loc, unsigned long in_ts);
int             bm_act_register_event(char* module, char* context, char* group, unsigned char level, char* name, char* suffix, int is_param, int val);
void            bm_init(void);
int             print_out_buffer(char *buf);

#ifdef TIWLAN_BMTRACE
#define TIWLAN_CLT_LEVEL 4
#else
#define TIWLAN_CLT_LEVEL 0
#endif


#define BM_VAR_INFO_PRINT_BIT 0x8000000
#define BM_PARAM_ENTRY_BIT    0x1000000


#define CL_TRACE_START() \
	unsigned long in_ts = bm_act_trace_in();

#define CL_TRACE_RESTART_Lx() \
	in_ts = bm_act_trace_in();

#define CL_TRACE_END(MODULE, CONTEXT, GROUP, LEVEL, SUFFIX) \
	{ \
		static int loc = 0; \
		if (loc==0) \
			loc = bm_act_register_event(MODULE, CONTEXT, GROUP, LEVEL, (char*)__FUNCTION__, SUFFIX, 0, 0); \
		bm_act_trace_out(loc, in_ts); \
	}

#define CL_TRACE_VAL_DUMP(VAR_NAME, VAR_VALUE_IN_DECIMAL) \
	{ \
		bm_act_trace_out(BM_VAR_INFO_PRINT_BIT | bm_act_register_event("val_dump", VAR_NAME, "0" , '0' , "", "", 0, VAR_VALUE_IN_DECIMAL) \
						, bm_act_trace_in()); \
	}


	#define CL_TRACE_START_L0() CL_TRACE_START()
	#define CL_TRACE_END_L0() CL_TRACE_END("KERNEL", "SYS_CALL", "PLATFORM_TEST", 0, "")

#if TIWLAN_CLT_LEVEL == 1
    #define CL_TRACE_INIT()     bm_init()
    #define CL_TRACE_ENABLE()   bm_enable()
    #define CL_TRACE_DISABLE()  bm_disable()
    #define CL_TRACE_PRINT(buf) print_out_buffer(buf)
	#define CL_TRACE_START_L1() CL_TRACE_START()
	#define CL_TRACE_START_L2() 
	#define CL_TRACE_START_L3() 
	#define CL_TRACE_START_L4() 
	#define CL_TRACE_START_L5() 
    #define CL_TRACE_RESTART()  CL_TRACE_RESTART_Lx()
	#define CL_TRACE_END_L1(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 1, SUFFIX)
	#define CL_TRACE_END_L2(MODULE, CONTEXT, GROUP, SUFFIX) 
	#define CL_TRACE_END_L3(MODULE, CONTEXT, GROUP, SUFFIX) 
	#define CL_TRACE_END_L4(MODULE, CONTEXT, GROUP, SUFFIX) 
	#define CL_TRACE_END_L5(MODULE, CONTEXT, GROUP, SUFFIX) 
#elif TIWLAN_CLT_LEVEL == 2
    #define CL_TRACE_INIT()     bm_init()
    #define CL_TRACE_ENABLE()   bm_enable()
    #define CL_TRACE_DISABLE()  bm_disable()
    #define CL_TRACE_PRINT(buf) print_out_buffer(buf)
	#define CL_TRACE_START_L1() CL_TRACE_START()
	#define CL_TRACE_START_L2() CL_TRACE_START()
	#define CL_TRACE_START_L3() 
	#define CL_TRACE_START_L4() 
	#define CL_TRACE_START_L5() 
    #define CL_TRACE_RESTART()  CL_TRACE_RESTART_Lx()
	#define CL_TRACE_END_L1(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 1, SUFFIX)
	#define CL_TRACE_END_L2(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 2, SUFFIX)
	#define CL_TRACE_END_L3(MODULE, CONTEXT, GROUP, SUFFIX) 
	#define CL_TRACE_END_L4(MODULE, CONTEXT, GROUP, SUFFIX) 
	#define CL_TRACE_END_L5(MODULE, CONTEXT, GROUP, SUFFIX) 
#elif TIWLAN_CLT_LEVEL == 3
    #define CL_TRACE_INIT()     bm_init()
    #define CL_TRACE_ENABLE()   bm_enable()
    #define CL_TRACE_DISABLE()  bm_disable()
    #define CL_TRACE_PRINT(buf) print_out_buffer(buf)
	#define CL_TRACE_START_L1() CL_TRACE_START()
	#define CL_TRACE_START_L2() CL_TRACE_START()
	#define CL_TRACE_START_L3() CL_TRACE_START()
	#define CL_TRACE_START_L4() 
	#define CL_TRACE_START_L5() 
    #define CL_TRACE_RESTART()  CL_TRACE_RESTART_Lx()
	#define CL_TRACE_END_L1(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 1, SUFFIX)
	#define CL_TRACE_END_L2(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 2, SUFFIX)
	#define CL_TRACE_END_L3(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 3, SUFFIX)
	#define CL_TRACE_END_L4(MODULE, CONTEXT, GROUP, SUFFIX) 
	#define CL_TRACE_END_L5(MODULE, CONTEXT, GROUP, SUFFIX) 
#elif TIWLAN_CLT_LEVEL == 4
    #define CL_TRACE_INIT()     bm_init()
    #define CL_TRACE_ENABLE()   bm_enable()
    #define CL_TRACE_DISABLE()  bm_disable()
    #define CL_TRACE_PRINT(buf) print_out_buffer(buf)
	#define CL_TRACE_START_L1() CL_TRACE_START()
	#define CL_TRACE_START_L2() CL_TRACE_START()
	#define CL_TRACE_START_L3() CL_TRACE_START()
	#define CL_TRACE_START_L4() CL_TRACE_START()
	#define CL_TRACE_START_L5() 
    #define CL_TRACE_RESTART()  CL_TRACE_RESTART_Lx()
	#define CL_TRACE_END_L1(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 1, SUFFIX)
	#define CL_TRACE_END_L2(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 2, SUFFIX)
	#define CL_TRACE_END_L3(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 3, SUFFIX)
	#define CL_TRACE_END_L4(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 4, SUFFIX)
	#define CL_TRACE_END_L5(MODULE, CONTEXT, GROUP, SUFFIX) 
#elif TIWLAN_CLT_LEVEL == 5
    #define CL_TRACE_INIT()     bm_init()
    #define CL_TRACE_ENABLE()   bm_enable()
    #define CL_TRACE_DISABLE()  bm_disable()
    #define CL_TRACE_PRINT(buf) print_out_buffer(buf)
	#define CL_TRACE_START_L1() CL_TRACE_START()
	#define CL_TRACE_START_L1() CL_TRACE_START()
	#define CL_TRACE_START_L2() CL_TRACE_START()
	#define CL_TRACE_START_L3() CL_TRACE_START()
	#define CL_TRACE_START_L4() CL_TRACE_START()
	#define CL_TRACE_START_L5() CL_TRACE_START()
    #define CL_TRACE_RESTART()  CL_TRACE_RESTART_Lx()
	#define CL_TRACE_END_L1(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 1, SUFFIX)
	#define CL_TRACE_END_L2(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 2, SUFFIX)
	#define CL_TRACE_END_L3(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 3, SUFFIX)
	#define CL_TRACE_END_L4(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 4, SUFFIX)
	#define CL_TRACE_END_L5(MODULE, CONTEXT, GROUP, SUFFIX) CL_TRACE_END(MODULE, CONTEXT, GROUP, 5, SUFFIX)
#else
    #define CL_TRACE_INIT()   
    #define CL_TRACE_ENABLE() 
    #define CL_TRACE_DISABLE()
    #define CL_TRACE_RESTART() 
    #define CL_TRACE_PRINT(buf)  
	#define CL_TRACE_START_L1() 
	#define CL_TRACE_START_L1() 
	#define CL_TRACE_START_L2() 
	#define CL_TRACE_START_L3() 
	#define CL_TRACE_START_L4() 
	#define CL_TRACE_START_L5() 
	#define CL_TRACE_END_L1(MODULE, CONTEXT, GROUP, SUFFIX) 
	#define CL_TRACE_END_L2(MODULE, CONTEXT, GROUP, SUFFIX) 
	#define CL_TRACE_END_L3(MODULE, CONTEXT, GROUP, SUFFIX) 
	#define CL_TRACE_END_L4(MODULE, CONTEXT, GROUP, SUFFIX) 
	#define CL_TRACE_END_L5(MODULE, CONTEXT, GROUP, SUFFIX) 
#endif


#endif /* __BM_TRACE_API_H */
