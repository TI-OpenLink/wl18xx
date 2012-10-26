#ifndef _LINUX_MMU_CONTEXT_H
#define _LINUX_MMU_CONTEXT_H

#include <linux/sched.h>
#include <asm/mmu_context.h>

struct mm_struct;

void use_mm(struct mm_struct *mm);
void unuse_mm(struct mm_struct *mm);

#ifndef finish_arch_post_lock_switch
# define finish_arch_post_lock_switch()	do { } while (0)
#endif

#endif
