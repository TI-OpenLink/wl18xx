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
**|     Copyright (c) 1998-2009 Texas Instruments Incorporated           |**
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
 
/** \file   bmtrace.c 
 *  \brief  The Linux bmtrace performance tracing implementation 
 *
 *  \see    bmtrace_api.h 
 */

#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/timex.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <plat/dmtimer.h>

#include "bmtrace_api.h"


#define BM_INIT_CALL(func_name)                  __initcall(func_name)
#define BM_ALLOC(seg_id, size, alignment)        vmalloc(size)
#define BM_ALLOC_IN_ISR(seg_id, size, alignment) kmalloc(size, GFP_ATOMIC)
#define BM_FREE(seg_id, mem_addr, size)         (vfree(mem_addr))
#define BM_FREE_IN_ISR(seg_id, mem_addr, size)  (kfree(mem_addr))
/*#define BM_GET_TIMESTAMP(tmp)                   \
    {                                           \
        struct timeval tv;                      \
        do_gettimeofday(&tv);                   \
        tmp = tv.tv_sec*1000000 + tv.tv_usec;   \
    }
*/

#define BM_GET_TIMESTAMP(tmp)                   \
    {                                           \
		tmp = omap_dm_timer_read_counter(timer);\
    }

#define sprintfX sprintf

/* CPU depend parameters */
#define CACHE_SIZE 16384
#define CACHE_LINE_SIZE 32
#define NUMBER_OF_TLBS 128

#define BM_NUM_ENTRIES 100000
/* #define BM_NUM_ENTRIES 1024 */


#ifndef False
#define False 0
#endif
#ifndef True
#define True 1
#endif
#ifndef NULL
#define NULL 0
#endif

typedef struct {
      int event_id;/* trace entry identification */
      unsigned long ts; /* Timestamp */
      unsigned long param; /* Timestamp */
} bm_entry_t;

typedef struct {
      int stop_running;
      int print_on;
	  int init_done;
	  int enable;
      unsigned int num_of_entries;
      unsigned long prev_param; /* Timestamp */
} bm_control_t;

typedef struct {
      char* event_name;
} bm_event_record_in_lut_t;

bm_control_t bm_control = {True, False, False, False, 0, 0};

bm_entry_t   *bm_entry = NULL;
bm_entry_t   *bm_entry_pointer = NULL;
bm_entry_t   *bm_entry_end = NULL;
char *bm_print_buf = NULL;

typedef struct {
    int counter;
    bm_event_record_in_lut_t *lut;
} bm_trace_lut_t;

static bm_trace_lut_t bm_trace_lut = {0, NULL};


/* Proc entries */
static struct proc_dir_entry *trace_ctl = NULL;

static struct omap_dm_timer *timer;
static bool first = false;

/*--------------------------------------------------------------------------------------*/

unsigned long bm_act_trace_in()
{
    unsigned long ts;

	if (!first) 
	{
        printk("bm_act_trace_in: get one of the free gptimers.\n");
        /* get one of the free gptimers */
        timer = omap_dm_timer_request();
        printk("bm_act_trace_in: change clock source to sysclk, which is 38.4MHz on OMAP4.\n");
		/* change clock source to sysclk, which is 38.4MHz on OMAP4 */
		omap_dm_timer_set_source(timer, OMAP_TIMER_SRC_SYS_CLK);
		omap_dm_timer_set_load_start(timer, 0, 0);
		first = true;
	}

    
    BM_GET_TIMESTAMP(ts);
    if (ts <= bm_control.prev_param)
    {
        ts = bm_control.prev_param + 1; /* work around to avoid previous out_ts <= ts */
    }

    return ts;  
}

/*--------------------------------------------------------------------------------------*/

void bm_act_trace_out(int event_id, unsigned long in_ts)
{
    unsigned long out_ts;

    if (unlikely(bm_control.stop_running == True))
    {
        return;
    }
    
    BM_GET_TIMESTAMP(out_ts);
    if (out_ts <= in_ts)
    {
        out_ts = in_ts + 1; /* avoid events length < 1 */
    }
    bm_entry_pointer->event_id = event_id;
    bm_entry_pointer->ts = in_ts;
    bm_entry_pointer->param = bm_control.prev_param = out_ts;

    ++bm_entry_pointer;
    if (unlikely(bm_entry_pointer == bm_entry_end))
    {
        bm_control.enable = False;
        bm_control.stop_running = True;
        bm_control.prev_param = 0;
        printk("Bmtrace buffer is full. Stopping traces.\n");
    }
}

/*--------------------------------------------------------------------------------------*/

void bm_disable(void)
{
    bm_control.stop_running = True;
	bm_control.enable = False;
}

/*--------------------------------------------------------------------------------------*/

void bm_enable(void) {
	bm_control.enable = True;
    bm_control.stop_running = (bm_control.print_on==True)?True:False;
}

/*--------------------------------------------------------------------------------------*/

static void bm_print_off(void)
{
	bm_control.print_on = False;
    bm_control.stop_running = (bm_control.enable==True)?False:True;
}

/*--------------------------------------------------------------------------------------*/

static void bm_print_on(void) {
    bm_control.stop_running = True;
	bm_control.print_on = True;
}

/*--------------------------------------------------------------------------------------*/

void print_tic_time(void)
{

    printk("\n=============> tic time is * 1000 * ns <=============\n\n");

    /* Old:
    printk("\n=============> tic time is * %ld * ns <=============\n\n", 
          (1000000000L + (CLOCK_TICK_RATE >> 1 )) / CLOCK_TICK_RATE);
    */

} /* print_tic_time */


/************************* Start print log ****************************/

/*--------------------------------------------------------------------------------------*/

int print_out_buffer(char *buf)
{
    int event_id;
    int len = 0;
    unsigned int i;
    unsigned int g_position;

    print_tic_time();
    g_position = (bm_entry_pointer - bm_entry);
	len += sprintf(buf+len, "\n------------------ TRACE BUFFER, pos = %d  --------------------\n\n", g_position);

    for (i = 0; i < g_position; i++)
	{
    	if (bm_entry[i].event_id & BM_VAR_INFO_PRINT_BIT)
    	{
    		event_id = bm_entry[i].event_id & (~BM_VAR_INFO_PRINT_BIT);
    		len += sprintf(buf+len, "%s\n",   bm_trace_lut.lut[event_id].event_name);
    	}
    	else
    	{
			event_id = bm_entry[i].event_id & (~BM_PARAM_ENTRY_BIT);
			if (bm_entry[i].event_id & BM_PARAM_ENTRY_BIT)
			{
				len += sprintf(buf+len, "TI-SoT: CLT_REC: UINT64: %s: %lu: %lu\n",
							   bm_trace_lut.lut[event_id].event_name,
							   bm_entry[i].ts,
							   bm_entry[i].param);
			}
			else
			{
				len += sprintf(buf+len, "TI-SoT: CLT_REC: %s: <->: %lu: %lu\n",
							   bm_trace_lut.lut[event_id].event_name,
							   bm_entry[i].ts,
							   bm_entry[i].param);
			}
    	}
	}

	len += sprintf(buf+len, "\n------------------------ END OF TRACE BUFFER -------------------------\n\n");

    bm_entry_pointer = &bm_entry[0];
    bm_control.prev_param = 0;

	return(len);

}

/*--------------------------------------------------------------------------------------*/

static int print_proc(char* page, char **start, off_t off, int count,int *eof, void *data)
{
	static int len;
	int new_count;

	if (off == 0)
	{
        bm_print_on();
		len = print_out_buffer(bm_print_buf);
	}

	if (off >= len)
	{
		*eof = 1;
		return 0;
	}

	new_count =  count + off > len ? len - off : count;

	memcpy(page, bm_print_buf + off, new_count);

	off += new_count;

	*eof = 0;

	*start = page;

	if( new_count < count )
	{
        bm_entry_pointer = bm_entry;
        bm_print_off();
	}

	return new_count;
}

/*--------------------------------------------------------------------------------------*/

/************************* End print log ****************************/

/************************* Start measurements ****************************/
#define NUM_OF_TRACE_ERROR 100
#define NUM_OF_LOOP_TEST 1000000
#define NUM_OF_FUNCTION_CALL_TEST 100

static void bm_test_two_seq_samples(void)
{
    CL_TRACE_START_L0();
    CL_TRACE_END_L0();
}

/*--------------------------------------------------------------------------------------*/
/*
static void bm_test_trace_error(void)
{
    int i;

    for (i=0; i<NUM_OF_TRACE_ERROR; ++i)
    {
        CL_TRACE_START_L0();
        CL_TRACE_END_L0();
    }
}
*/
/*--------------------------------------------------------------------------------------*/

static void bm_test_simple_loop(unsigned char with_printing, int loop_count, int nop)
{
    unsigned long num = 0x5a5; /* this will mask the optimizer to cut out the loop */
    int i;

    CL_TRACE_START_L0();
    for (i=0; i<NUM_OF_LOOP_TEST; ++i)
    {
	  ++num;
	  if (nop)
      {
		printk("%d\n",(int)num);
	  }
    }
    CL_TRACE_END_L0();
    if (with_printing!=False)
    {
        printk("bm_test_simple_loop result: %lu\n", num); /* disable the optimizer to optimize this test */
    }
}

/*--------------------------------------------------------------------------------------*/

typedef struct bm_test_st{
    unsigned long sum;
    struct bm_test_st* next;
} bm_test_t;

/*--------------------------------------------------------------------------------------*/

static void bm_test_pointer(unsigned char with_printing)
{
    int  counter = NUM_OF_LOOP_TEST;

    bm_test_t* node;
    bm_test_t node1, node2;

    node1.sum = 1;
    node2.sum = 1;

    node1.next = &node2;
    node2.next = &node1;

    node = &node1;
    {
        CL_TRACE_START_L0();
        while (--counter)
        {
            node->sum += node->next->sum;
            node = node->next;
        }
        CL_TRACE_END_L0();
    }
    if (with_printing!=False)
    {
        printk("bm_test_pointer result: %lu\n", node->sum); /* disable the optimizer to optimize this test */
    }
}

/*--------------------------------------------------------------------------------------*/

static int bm_test_fibonacci(int counter, unsigned long a, unsigned long b)
{
    unsigned long sum;

    if (--counter==0)
        return a+b;

    sum = a+b;
    return sum+bm_test_fibonacci(counter, b, sum);
}

/*--------------------------------------------------------------------------------------*/

static void bm_test_function_call(unsigned char with_printing)
{
    unsigned long result;

    CL_TRACE_START_L0();
    result = bm_test_fibonacci(NUM_OF_FUNCTION_CALL_TEST, 1, 1);
    CL_TRACE_END_L0();
    if (with_printing!=False)
    {
        printk("bm_test_function_call result: %lu\n", result); /* disable the optimizer to optimize this test */
    }
}

/*--------------------------------------------------------------------------------------*/

#define CACHE_MISS_MEM_REQUIRED (2*CACHE_SIZE)
static void bm_mis_calibration(unsigned char with_printing)
{
    int counter = NUM_OF_LOOP_TEST;
    int line_counter = 0;
    CL_TRACE_START_L0();
    while (--counter)
    {
        line_counter += (CACHE_LINE_SIZE/sizeof(int)); /* move to the next line */
        line_counter = line_counter%(CACHE_MISS_MEM_REQUIRED/sizeof(int)); /* make sure you stay in the memory chunk */
    }
    CL_TRACE_END_L0();
    if (with_printing!=False)
    {
        printk("bm_mis_calibration result: %d\n", line_counter); /* disable the optimizer to optimize this test */
    }
}

/*--------------------------------------------------------------------------------------*/

static void bm_test_cache_miss(unsigned char with_printing)
{
    int* p_buffer;
    int counter = NUM_OF_LOOP_TEST;
    int line_counter = 0;
    int num = 0;
    int i;

    /* we used eight pages to generate cache miss (8 is factor of 2 good for optimizations) */
    /* the */
    /* one extra page for aligment */
    p_buffer = BM_ALLOC(0, CACHE_MISS_MEM_REQUIRED, 32);
    for (i=0; i<CACHE_MISS_MEM_REQUIRED/sizeof(int); ++i) /* init and make sure all TLBs' are set */
        p_buffer[i] = 1;

    {
        CL_TRACE_START_L0();
        while (--counter)
        {
            num += p_buffer[line_counter];
            line_counter += (CACHE_LINE_SIZE/sizeof(int)); /* move to the next line */
            line_counter = line_counter%(CACHE_MISS_MEM_REQUIRED/sizeof(int)); /* make sure you stay in the memory chunk */
        }
        CL_TRACE_END_L0();
    }
    if (with_printing!=False)
    {
        printk("bm_test_cache_miss result: %d\n", num); /* disable the optimizer to optimize this test */
    }
    BM_FREE(0, p_buffer, CACHE_MISS_MEM_REQUIRED);
}

/*--------------------------------------------------------------------------------------*/

#define BM_MIN(a, b) ((a) < (b) ? (a) : (b))
#define TLB_CACHE_SIZE NUMBER_OF_TLBS*PAGE_SIZE
#define MIN_CAUSE_CACHE (CACHE_SIZE/CACHE_LINE_SIZE)*PAGE_SIZE
#define TLB_FAIL_MEM_REQUIRED 2*BM_MIN(TLB_CACHE_SIZE, MIN_CAUSE_CACHE)

/*--------------------------------------------------------------------------------------*/

static void bm_test_tlb_miss(unsigned char with_printing)
{
    int* p_buffer;
    int counter = NUM_OF_LOOP_TEST;
    int line_counter = 0;
    int num = 0;
    unsigned int i;

    /* we used eight pages to generate cache miss (8 is factor of 2 good for optimizations) */
    /* the */
    /* one extra page for aligment */
    p_buffer = BM_ALLOC(0, TLB_FAIL_MEM_REQUIRED, 32);
    for (i=0; i<TLB_FAIL_MEM_REQUIRED/sizeof(int); ++i)
        p_buffer[i] = 1;
    {
        CL_TRACE_START_L0();
        while (--counter)
        {
            num += p_buffer[line_counter];
            line_counter += (PAGE_SIZE/sizeof(int)); /* move to the next line */
            line_counter = line_counter%(TLB_FAIL_MEM_REQUIRED/sizeof(int)); /* make sure you stay in the memory chunk */
        }
        CL_TRACE_END_L0();
    }

    if (with_printing!=False)
    {
	  printk("bm_test_tlb_miss result: %d\n", num); /* disable the optimizer to optimize this test */
	}
    BM_FREE(0, p_buffer, TLB_FAIL_MEM_REQUIRED);
}

/*--------------------------------------------------------------------------------------*/


/************************* End measurements ****************************/

/************************* Start trace ctl ****************************/

/*--------------------------------------------------------------------------------------*/

static void bm_measure_all(unsigned char with_printing)
{
    if (with_printing!=False)
        printk("Running: bm_test_two_seq_samples\n");
    bm_test_two_seq_samples();
    bm_test_two_seq_samples();
    bm_test_two_seq_samples();
/*
    if (with_printing!=False)
    {
        printk("Running: bm_test_trace_error\n");
        printk("         number of loops is %d\n", NUM_OF_TRACE_ERROR);
    }
    bm_test_trace_error();
*/
    if (with_printing!=False)
    {
        printk("Running: bm_test_simple_loop\n");
        printk("         number of loops is %d\n", NUM_OF_LOOP_TEST);
    }
    bm_test_simple_loop(with_printing, NUM_OF_LOOP_TEST, 0);

    if (with_printing!=False)
    {
        printk("Running: bm_test_pointer\n");
        printk("         number of redirections is %d\n", NUM_OF_LOOP_TEST);
    }
    bm_test_pointer(with_printing);

    if (with_printing!=False)
    {
        printk("Running: bm_test_function_call\n");
        printk("         via fibonacci, recursive depth is %d\n", NUM_OF_FUNCTION_CALL_TEST);
    }
    bm_test_function_call(with_printing);

    if (with_printing!=False)
    {
        printk("Running: bm_mis_calibration\n");
        printk("         number of reads %d\n", NUM_OF_LOOP_TEST);
    }
    bm_mis_calibration(with_printing);

    if (with_printing!=False)
    {
        printk("Running: bm_test_cache_miss\n");
        printk("         assume: d-cache size is %d and d-cache line is %d\n", CACHE_SIZE, CACHE_LINE_SIZE);
        printk("         number of reads %d\n", NUM_OF_LOOP_TEST);
    }
    bm_test_cache_miss(with_printing);

    if (with_printing!=False)
    {
        printk("Running: bm_test_tlb_miss\n");
        printk("         assume: %d TLBs in the CPU. Page size is %ld\n", NUMBER_OF_TLBS, PAGE_SIZE);
        printk("         number of reads %d\n", NUM_OF_LOOP_TEST);
    }
    bm_test_tlb_miss(with_printing);
}

/*--------------------------------------------------------------------------------------*/

static void bm_act_print_lut(void)
{
    int i;

    for (i=0; i<bm_trace_lut.counter; ++bm_trace_lut.counter)
        printk("lut[%d].event_name=%s", i, bm_trace_lut.lut[i].event_name);
}


/************************* End trace ctl ****************************/

/*--------------------------------------------------------------------------------------*/

static void bm_act_clear_lut(void)
{
    int i;

    for (i=0; i<bm_trace_lut.counter; ++bm_trace_lut.counter)
        BM_FREE(0, bm_trace_lut.lut[i].event_name, sizeof(bm_event_record_in_lut_t.event_name));

    BM_FREE(0, bm_trace_lut.lut, sizeof(bm_event_record_in_lut_t)*bm_trace_lut.counter);
    bm_trace_lut.lut = NULL;
}

/*--------------------------------------------------------------------------------------*/

static void bm_act_shutdown(void)
{
    BM_FREE(0, bm_entry, sizeof(bm_entry_t)*bm_control.num_of_entries);
    bm_entry = NULL;
    BM_FREE(0, bm_print_buf, bm_control.num_of_entries*40+200);
    bm_print_buf = NULL;
    bm_act_clear_lut();

    bm_control.num_of_entries = 0;
	bm_control.init_done = False;
}

/*--------------------------------------------------------------------------------------*/

int bm_act_register_event(char* module, char* context, char* group, unsigned char level, char* name, char* suffix, int is_param, int val)
{
    bm_event_record_in_lut_t *tmp_lut;

   //printk("\n ++++++++++ register function: level=%d+++++++++++++ \n", level);
	
    tmp_lut = BM_ALLOC_IN_ISR(0, sizeof(bm_event_record_in_lut_t)*(bm_trace_lut.counter+1), 0);
    if(NULL == tmp_lut)
    {
        printk("bm_act_register_event() : Memory allocation failed \n");
        return -1;
    }
    //printk("++++++++++ register function: %p\n", tmp_lut);
    /* Know copy the pointers */
    memcpy(tmp_lut, bm_trace_lut.lut, sizeof(bm_event_record_in_lut_t)*bm_trace_lut.counter);
    BM_FREE_IN_ISR(0, bm_trace_lut.lut, sizeof(bm_event_record_in_lut_t)*bm_trace_lut.counter);
    bm_trace_lut.lut = tmp_lut;

    bm_trace_lut.lut[bm_trace_lut.counter].event_name = BM_ALLOC_IN_ISR(0, strlen(module)+2+strlen(context)+2+strlen(group)+2+3+2+strlen(name)+2+strlen(suffix)+2, 0);


    /* level 0 is for the specific variable info print */
    if (unlikely(level == '0'))
    {
    	sprintf(bm_trace_lut.lut[bm_trace_lut.counter].event_name,"%s: %lu: %s: %d", module,  bm_act_trace_in(), context, val);
    }
    else
    {
    	sprintf(bm_trace_lut.lut[bm_trace_lut.counter].event_name,"%s: %s: %s: %d: %s%s", module, context, group, level, name,suffix);
    }
	//printk("++++++++++ register function: %s\n", bm_trace_lut.lut[bm_trace_lut.counter].event_name);
    ++bm_trace_lut.counter;

    if (unlikely(is_param))
        return (bm_trace_lut.counter-1)|BM_PARAM_ENTRY_BIT;
    else
        return bm_trace_lut.counter-1;
}

/*--------------------------------------------------------------------------------------*/

static void trace_ctl_usage(void)
{
    printk("Use \"echo 0 > trace_ctl\" to disable the trace\n");
    printk("Use \"echo 1 > trace_ctl\" to enable the trace\n");
    printk("Use \"echo l|L > trace_ctl\" to print Look Up Table\n");
    printk("Use \"echo r|R > trace_ctl\" to reset the trace log\n");
    printk("Use \"echo p|P > trace_ctl\" to reset logs, perform platform tests, alt until reset\n");
    printk("Use \"echo a|A > trace_ctl\" to run full measurement of memory access\n");
}

/*--------------------------------------------------------------------------------------*/

static int trace_ctl_read(char* page, char **start, off_t off, int count,int *eof, void *data)
{
    int len = 0;

    len += sprintf(page+len, "Use \"echo 0 > trace_ctl\" to disable the trace\n");
    len += sprintf(page+len, "Use \"echo 1 > trace_ctl\" to enable the trace\n");
    len += sprintf(page+len, "Use \"echo l|L > trace_ctl\" to print Look Up Table\n");
    len += sprintf(page+len, "Use \"echo r|R > trace_ctl\" to reset the trace log\n");
    len += sprintf(page+len, "Use \"echo p|P > trace_ctl\" to reset logs, perform platform tests, alt until reset\n");
	len += sprintf(page+len, "Use \"echo s|S > trace_ctl\" to get 1000 clock cycles in nSec\n");
	len += sprintf(page+len, "Use \"echo a|A > trace_ctl\" to run full measurement of memory access\n");

	if(bm_control.enable == True)
       len+= sprintf(page+len, "\nTrace enable\n");
	else
       len+= sprintf(page+len, "\nTrace disable\n");

	return(len);
}

/*--------------------------------------------------------------------------------------*/
#define ALL_TEST_COUNT 20
static void bm_test_all(void)
{
  int j;

  print_tic_time();
  printk("\nTrace enable\n");
  bm_enable();
  bm_measure_all(0); /* make sure that functions in cache, no printing */
  printk("\nReseting logs\n");
  bm_entry_pointer = bm_entry;
  printk("\nRunning measurements %d times, please wait... \n", ALL_TEST_COUNT);
  for (j = 0; j < ALL_TEST_COUNT; j++)
  {
      bm_measure_all(0); 
  }
  printk("\nTrace disable\n");
  bm_disable();
  printk("\nDone, please print the trace log.\n");
} 

/*--------------------------------------------------------------------------------------*/

//#define PRCM_BASE_PA               0x49006000

#define CM_CLKEN_PLL         PRCM_REG32(0x500)
#define CM_AUTOIDLE_PLL      PRCM_REG32(0x530)
#define CM_CLKSEL1_PLL       PRCM_REG32(0x540)
#define CM_CLKSEL2_PLL       PRCM_REG32(0x544)
#define CM_CLKSEL1_CORE      PRCM_REG32(0x240)

#define DISPLAY_REGISTER_VALUE(x, y)  printk("%-20s (0x%04x) = 0x%08X\n",#y, (u32)&((typeof(x))0)->y, __REG32((u32)x + (u32)&((typeof(x))0)->y))

/*--------------------------------------------------------------------------------------*/

typedef unsigned int UINT32, *PUINT32;

static int trace_ctl_write (struct file *fp, const char * buf, unsigned long count, void * data)
{
    char local_buf[31];
    int  ret_val = 0;
    unsigned long flags;

    if(count > 30)
    {
        printk("Error : Buffer Overflow\n");
        trace_ctl_usage();
        return -EFAULT;
    }

    copy_from_user(local_buf,buf,count);
    local_buf[count-1]='\0'; /* Ignoring last \n char */
        ret_val = count;


    local_irq_save(flags);
    if(strcmp("0",local_buf)==0)
    {
		printk("\nTrace disable\n");
        bm_disable();
	}
    else if(strcmp("1",local_buf)==0)
    {
        printk("\nTrace enable\n");
        bm_enable();
    }
    else if(strcmp("l",local_buf)==0 || strcmp("L",local_buf)==0)
    {
        printk("\nPrinting Look Up Table\n");
        bm_act_print_lut();
    }
    else if(strcmp("r",local_buf)==0 || strcmp("R",local_buf)==0)
    {
        printk("\nReset trace logs\n");
        bm_entry_pointer = bm_entry;
    }
	else if(strcmp("s",local_buf)==0 || strcmp("S",local_buf)==0)
    {
		// printk("\n1000 clock cycles = %ld\n", arch_cycle_to_nsec((unsigned long)1000));
        bm_disable();
	}
   	else if(strcmp("a",local_buf)==0 || strcmp("A",local_buf)==0)
    {
        bm_test_all();
	}
    else 
    {
        trace_ctl_usage();
    }
    local_save_flags(flags);

    return ret_val;
}

/*--------------------------------------------------------------------------------------*/

static int bm_act_init(int num_of_entries)
{
	if(bm_control.init_done == True)
	{
			printk("!!!!!!! Trace is already up !!!!!!!!!\n");
			return -1;
	}

    bm_entry = BM_ALLOC(0, sizeof(bm_entry_t)*num_of_entries, 0);
    bm_print_buf = BM_ALLOC(0, num_of_entries*128+200, 0);
    if (bm_entry==NULL || bm_print_buf==NULL)
    {
        printk("!!!!!!! bm_trace:: OUT of memory !!!!!!!!!\n");
        bm_act_shutdown();
        return -1;
    }

    bm_entry_pointer = bm_entry;
    bm_entry_end = bm_entry + num_of_entries;
    bm_control.num_of_entries = num_of_entries;
	bm_control.print_on = False;
    bm_control.enable = False;
    bm_control.stop_running = True;
	bm_control.init_done = True;

	create_proc_read_entry("trace",0,NULL,print_proc,NULL);
    trace_ctl = create_proc_entry("trace_ctl", 0644, NULL);
    if(trace_ctl)
    {
        trace_ctl->read_proc  = trace_ctl_read;
        trace_ctl->write_proc = trace_ctl_write;
    }

	return 0;
}

/*--------------------------------------------------------------------------------------*/

int bm_act_reinit(int num_of_entries)
{
    bm_act_shutdown();
    bm_act_init(num_of_entries);
    return 0;
}

/*--------------------------------------------------------------------------------------*/

static int  __init  bm_module_init(void)
{
    return bm_act_init(BM_NUM_ENTRIES);
}

/*--------------------------------------------------------------------------------------*/

static void __exit bm_module_exit(void)
{
  /*    bm_act_shutdown(); */
}

/*--------------------------------------------------------------------------------------*/

EXPORT_SYMBOL(bm_act_register_event);
EXPORT_SYMBOL(bm_act_trace_in);
EXPORT_SYMBOL(bm_act_trace_out);
EXPORT_SYMBOL(bm_act_reinit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("WLAN BenchMark test");

module_init(bm_module_init);
module_exit(bm_module_exit);

