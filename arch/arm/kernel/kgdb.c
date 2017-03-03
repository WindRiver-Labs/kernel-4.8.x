/*
 * arch/arm/kernel/kgdb.c
 *
 * ARM KGDB support
 *
 * Copyright (c) 2002-2004 MontaVista Software, Inc
 * Copyright (c) 2008 Wind River Systems, Inc.
 *
 * Authors:  George Davis <davis_g@mvista.com>
 *           Deepak Saxena <dsaxena@plexity.net>
 */
#include <linux/irq.h>
#include <linux/kdebug.h>
#include <linux/kgdb.h>
#include <linux/uaccess.h>

#include <asm/patch.h>
#include <asm/traps.h>

/* <WRS_ADDED> */
#include <asm/cacheflush.h>

#define BIT32(n) ((uint32_t)1U << (n))
#define BITSET(x, n) (((uint32_t)(x) & (1U<<(n))) >> (n))
#define BITS(x, m, n) (((uint32_t)((x) & (BIT32(n) - BIT32(m) +\
					BIT32(n)))) >> (m))

/* #define DEBUG_IT */
#ifdef DEBUG_IT
#define PRINTK(args...) printk(KERN_ALERT args)
#else
#define PRINTK(args...)
#endif /* DEBUG_IT */

/*
 * ccTable is used to determine whether an instruction will be executed,
 * according to the flags in the PSR and the condition field of the
 * instruction. The table has an entry for each possible value of the
 * condition field of the instruction. Each bit indicates whether a particular
 * combination of flags will cause the instruction to be executed. Since
 * ther are four flags, this makes 16 possible TRUE/FALSE values.
 */

static uint32_t ccTable[] = {
	0xF0F0, 0x0F0F, 0xCCCC, 0x3333, 0xFF00, 0x00FF, 0xAAAA, 0x5555,
	0x0C0C, 0xF3F3, 0xAA55, 0x55AA, 0x0A05, 0xF5FA, 0xFFFF, 0x0000
};

static uint32_t *stepped_address;
static uint32_t stepped_opcode;
static uint32_t stepped_cpsr_it_mask;
static uint32_t *armGetNpc(uint32_t instr,	/* the current instruction */
			struct pt_regs *pRegs);	/* pointer to task registers */

/* </WRS_ADDED> */

struct dbg_reg_def_t dbg_reg_def[DBG_MAX_REG_NUM] =
{
	{ "r0", 4, offsetof(struct pt_regs, ARM_r0)},
	{ "r1", 4, offsetof(struct pt_regs, ARM_r1)},
	{ "r2", 4, offsetof(struct pt_regs, ARM_r2)},
	{ "r3", 4, offsetof(struct pt_regs, ARM_r3)},
	{ "r4", 4, offsetof(struct pt_regs, ARM_r4)},
	{ "r5", 4, offsetof(struct pt_regs, ARM_r5)},
	{ "r6", 4, offsetof(struct pt_regs, ARM_r6)},
	{ "r7", 4, offsetof(struct pt_regs, ARM_r7)},
	{ "r8", 4, offsetof(struct pt_regs, ARM_r8)},
	{ "r9", 4, offsetof(struct pt_regs, ARM_r9)},
	{ "r10", 4, offsetof(struct pt_regs, ARM_r10)},
	{ "fp", 4, offsetof(struct pt_regs, ARM_fp)},
	{ "ip", 4, offsetof(struct pt_regs, ARM_ip)},
	{ "sp", 4, offsetof(struct pt_regs, ARM_sp)},
	{ "lr", 4, offsetof(struct pt_regs, ARM_lr)},
	{ "pc", 4, offsetof(struct pt_regs, ARM_pc)},
	{ "f0", 12, -1 },
	{ "f1", 12, -1 },
	{ "f2", 12, -1 },
	{ "f3", 12, -1 },
	{ "f4", 12, -1 },
	{ "f5", 12, -1 },
	{ "f6", 12, -1 },
	{ "f7", 12, -1 },
	{ "fps", 4, -1 },
	{ "cpsr", 4, offsetof(struct pt_regs, ARM_cpsr)},
};

char *dbg_get_reg(int regno, void *mem, struct pt_regs *regs)
{
	if (regno >= DBG_MAX_REG_NUM || regno < 0)
		return NULL;

	if (dbg_reg_def[regno].offset != -1)
		memcpy(mem, (void *)regs + dbg_reg_def[regno].offset,
		       dbg_reg_def[regno].size);
	else
		memset(mem, 0, dbg_reg_def[regno].size);
	return dbg_reg_def[regno].name;
}

int dbg_set_reg(int regno, void *mem, struct pt_regs *regs)
{
	if (regno >= DBG_MAX_REG_NUM || regno < 0)
		return -EINVAL;

	if (dbg_reg_def[regno].offset != -1)
		memcpy((void *)regs + dbg_reg_def[regno].offset, mem,
		       dbg_reg_def[regno].size);
	return 0;
}

void
sleeping_thread_to_gdb_regs(unsigned long *gdb_regs, struct task_struct *task)
{
	struct thread_info *ti;
	int regno;

	/* Just making sure... */
	if (task == NULL)
		return;

	/* Initialize to zero */
	for (regno = 0; regno < GDB_MAX_REGS; regno++)
		gdb_regs[regno] = 0;

	/* Otherwise, we have only some registers from switch_to() */
	ti			= task_thread_info(task);
	gdb_regs[_R4]		= ti->cpu_context.r4;
	gdb_regs[_R5]		= ti->cpu_context.r5;
	gdb_regs[_R6]		= ti->cpu_context.r6;
	gdb_regs[_R7]		= ti->cpu_context.r7;
	gdb_regs[_R8]		= ti->cpu_context.r8;
	gdb_regs[_R9]		= ti->cpu_context.r9;
	gdb_regs[_R10]		= ti->cpu_context.sl;
	gdb_regs[_FP]		= ti->cpu_context.fp;
	gdb_regs[_SPT]		= ti->cpu_context.sp;
	gdb_regs[_PC]		= ti->cpu_context.pc;
}

void kgdb_arch_set_pc(struct pt_regs *regs, unsigned long pc)
{
	regs->ARM_pc = pc;
}

static int compiled_break;
static struct undef_hook kgdb_brkpt_hook;

int kgdb_arch_handle_exception(int exception_vector, int signo,
			       int err_code, char *remcom_in_buffer,
			       char *remcom_out_buffer,
			       struct pt_regs *linux_regs)
{
	unsigned long addr;
	char *ptr;
	int error;

	switch (remcom_in_buffer[0]) {
	case 'D':
	case 'k':
	case 'c':
		/*
		 * Try to read optional parameter, pc unchanged if no parm.
		 * If this was a compiled breakpoint, we need to move
		 * to the next instruction or we will just breakpoint
		 * over and over again.
		 */
		ptr = &remcom_in_buffer[1];
		if (kgdb_hex2long(&ptr, &addr))
			linux_regs->ARM_pc = addr;
		else if (compiled_break == 1)
			linux_regs->ARM_pc += 4;

		compiled_break = 0;

		return 0;
	case 's':
	{
		uint32_t *next_addr;
		uint32_t currentInst;

		/*
		 * Do a software step. We assume that the host
		 * debuuger has already REMOVED the breakpoint, so if
		 * we read the memory, we have the REAL instruction.
		 */
		PRINTK("KGDB: s command\n");

		/* Try to read optional parameter, PC unchanged if
		 * none
		 */
		ptr = &remcom_in_buffer[1];
		if (kgdb_hex2long(&ptr, &addr))
			linux_regs->ARM_pc = addr;

		atomic_set(&kgdb_cpu_doing_single_step, -1);

		/* Read the current instruction at the PC */
		error = probe_kernel_read(&currentInst,
					  (char *)linux_regs->ARM_pc,
					  BREAK_INSTR_SIZE);
		if (error)
			return -EINVAL;
		PRINTK("KGDB current pc %lx %x\n",
		       linux_regs->ARM_pc, currentInst);

		/* Compute the next address */
		next_addr = armGetNpc(currentInst, linux_regs);
		stepped_address = next_addr;

		PRINTK("KGDB next pc %x\n", next_addr);

		/* Saves original instruction */
		error = probe_kernel_read(&stepped_opcode,
					  (char *) next_addr,
					  BREAK_INSTR_SIZE);
		if (error != 0) {
			PRINTK("Unable to access opcode at next pc 0x%x\n",
			       (int)next_addr);
			return error;
			break;
		}

		/* Sets the temporary breakpoint */
		error = probe_kernel_write((char *)next_addr,
					   (char *)&kgdb_brkpt_hook.instr_val,
					   BREAK_INSTR_SIZE);
		if (error != 0) {
			PRINTK("Unable to write tmp BP at next pc 0x%x\n",
			       (int)next_addr);
			return error;
			break;
		}

		/*
		 * Store the value of F & I bit
		 * in order to restore them later.
		 *
		 * Mask the interrupts
		 */
		stepped_cpsr_it_mask =
			linux_regs->ARM_cpsr & (PSR_F_BIT | PSR_I_BIT);
		linux_regs->ARM_cpsr |= PSR_F_BIT | PSR_I_BIT;

		/* Flush and return */
		flush_icache_range((long)next_addr,
				   (long)next_addr + 4);
		if (kgdb_contthread)
			atomic_set(&kgdb_cpu_doing_single_step,
				   smp_processor_id());
		return 0;
		break;
	}
	}

	return -1;
}

static int kgdb_brk_fn(struct pt_regs *regs, unsigned int instr)
{
	int error;

	/* If we have been single-stepping, put back the old instruction.
	 * We use stepped_address in case we have stopped more than one
	 * instruction away. */
	if (stepped_opcode != 0) {
		PRINTK("Step done at %x putting back %x\n",
		       stepped_address, stepped_opcode);
		/* restores original instruction */
		error = probe_kernel_write((char *)stepped_address,
					   (char *) &stepped_opcode,
					   BREAK_INSTR_SIZE);
		if (error != 0)	{
			PRINTK("Unable to restore original instruction\n");
			return 1;
		}

		flush_icache_range((long)stepped_address,
				   (long)stepped_address + 4);
		/* Restore original interrupts in cpsr regs */
		/* Clean the I & F bits */
		regs->ARM_cpsr &= ~(PSR_F_BIT | PSR_I_BIT);
		/* Add the original values */
		regs->ARM_cpsr |= stepped_cpsr_it_mask;
	}
	stepped_opcode = 0;
	kgdb_handle_exception(1, SIGTRAP, 0, regs);

	return 0;
}

static int kgdb_compiled_brk_fn(struct pt_regs *regs, unsigned int instr)
{
	compiled_break = 1;
	kgdb_handle_exception(1, SIGTRAP, 0, regs);

	return 0;
}

static struct undef_hook kgdb_brkpt_hook = {
	.instr_mask		= 0xffffffff,
	.instr_val		= KGDB_BREAKINST,
	.cpsr_mask		= MODE_MASK,
	.cpsr_val		= SVC_MODE,
	.fn			= kgdb_brk_fn
};

static struct undef_hook kgdb_compiled_brkpt_hook = {
	.instr_mask		= 0xffffffff,
	.instr_val		= KGDB_COMPILED_BREAK,
	.cpsr_mask		= MODE_MASK,
	.cpsr_val		= SVC_MODE,
	.fn			= kgdb_compiled_brk_fn
};

static void kgdb_call_nmi_hook(void *ignored)
{
       kgdb_nmicallback(raw_smp_processor_id(), get_irq_regs());
}

void kgdb_roundup_cpus(unsigned long flags)
{
       local_irq_enable();
       smp_call_function(kgdb_call_nmi_hook, NULL, 0);
       local_irq_disable();
}

static int __kgdb_notify(struct die_args *args, unsigned long cmd)
{
	struct pt_regs *regs = args->regs;

	if (kgdb_handle_exception(1, args->signr, cmd, regs))
		return NOTIFY_DONE;
	return NOTIFY_STOP;
}
static int
kgdb_notify(struct notifier_block *self, unsigned long cmd, void *ptr)
{
	unsigned long flags;
	int ret;

	local_irq_save(flags);
	ret = __kgdb_notify(ptr, cmd);
	local_irq_restore(flags);

	return ret;
}

static struct notifier_block kgdb_notifier = {
	.notifier_call	= kgdb_notify,
	.priority	= -INT_MAX,
};


/**
 *	kgdb_arch_init - Perform any architecture specific initalization.
 *
 *	This function will handle the initalization of any architecture
 *	specific callbacks.
 */
int kgdb_arch_init(void)
{
	int ret = register_die_notifier(&kgdb_notifier);

	if (ret != 0)
		return ret;

	register_undef_hook(&kgdb_brkpt_hook);
	register_undef_hook(&kgdb_compiled_brkpt_hook);

	return 0;
}

/**
 *	kgdb_arch_exit - Perform any architecture specific uninitalization.
 *
 *	This function will handle the uninitalization of any architecture
 *	specific callbacks, for dynamic registration and unregistration.
 */
void kgdb_arch_exit(void)
{
	unregister_undef_hook(&kgdb_brkpt_hook);
	unregister_undef_hook(&kgdb_compiled_brkpt_hook);
	unregister_die_notifier(&kgdb_notifier);
}

int kgdb_arch_set_breakpoint(struct kgdb_bkpt *bpt)
{
	int err;

	/* patch_text() only supports int-sized breakpoints */
	BUILD_BUG_ON(sizeof(int) != BREAK_INSTR_SIZE);

	err = probe_kernel_read(bpt->saved_instr, (char *)bpt->bpt_addr,
				BREAK_INSTR_SIZE);
	if (err)
		return err;

	/* Machine is already stopped, so we can use __patch_text() directly */
	__patch_text((void *)bpt->bpt_addr,
		     *(unsigned int *)arch_kgdb_ops.gdb_bpt_instr);

	return err;
}

int kgdb_arch_remove_breakpoint(struct kgdb_bkpt *bpt)
{
	/* Machine is already stopped, so we can use __patch_text() directly */
	__patch_text((void *)bpt->bpt_addr, *(unsigned int *)bpt->saved_instr);

	return 0;
}

/*
 * Register our undef instruction hooks with ARM undef core.
 * We regsiter a hook specifically looking for the KGB break inst
 * and we handle the normal undef case within the do_undefinstr
 * handler.
 */
struct kgdb_arch arch_kgdb_ops = {
#ifndef __ARMEB__
	.gdb_bpt_instr		= {0xfe, 0xde, 0xff, 0xe7}
#else /* ! __ARMEB__ */
	.gdb_bpt_instr		= {0xe7, 0xff, 0xde, 0xfe}
#endif
};

/* Copyright (c) 1996-2001 Wind River Systems, Inc. */
/* From dbgArmLib.c */
/* <WRS_ADDED> */
/*
 * The following is borrowed from vxWorks
 */

static uint32_t armShiftedRegVal(struct pt_regs *pRegs,
				 uint32_t instr,
				 int cFlag)
{
	uint32_t res, shift, rm, rs, shiftType;

	rm = BITS(instr, 0, 3);
	shiftType = BITS(instr, 5, 6);

	if (BITSET(instr, 4)) {
		rs = BITS(instr, 8, 11);
		shift =
		    (rs == 15 ? (uint32_t) pRegs->ARM_pc + 8 :
		     pRegs->uregs[rs]) & 0xFF;
	} else {
		shift = BITS(instr, 7, 11);
	}

	res = rm == 15 ? (uint32_t) pRegs->ARM_pc + (BITSET(instr, 4) ? 12 : 8)
	    : pRegs->uregs[rm];

	switch (shiftType) {
	case 0:		/* LSL */
		res = shift >= 32 ? 0 : res << shift;
		break;

	case 1:		/* LSR */
		res = shift >= 32 ? 0 : res >> shift;
		break;

	case 2:		/* ASR */
		if (shift >= 32)
			shift = 31;
		res = (res & 0x80000000L) ? ~((~res) >> shift) : res >> shift;
		break;

	case 3:		/* ROR */
		shift &= 31;
		if (shift == 0)
			res = (res >> 1) | (cFlag ? 0x80000000L : 0);
		else
			res = (res >> shift) | (res << (32 - shift));
		break;
	}
	return res;

}				/* armShiftedRegVal() */

static uint32_t *armGetNpc(uint32_t instr,	/* the current instruction */
			struct pt_regs *pRegs)	/* pointer to task registers */
{
	uint32_t pc;		/* current program counter */
	uint32_t nPc;		/* next program counter */

	/*
	 * Early versions of this file looked at the PSR to determine
	 * whether the CPU was in ARM state or Thumb state and decode
	 * the next instruction accordingly. This has been removed
	 * since there is to be no support for ARM/Thumb interworking.
	 */

	pc = (uint32_t) pRegs->ARM_pc;	/* current PC as a uint32_t */
	nPc = pc + 4;		/* default */

	PRINTK("nPc %x CPSR %x\n", nPc, pRegs->ARM_cpsr);

	/*
	 * Now examine the instruction
	 * First, check the current condition codes against the condition
	 * field of the instruction since, if this instruction is not going
	 * to be executed, we can return immediately
	 *
	 * The following code is a translation of the code supplied by ARM
	 * for instruction decoding (EAN-26). Note that this version, unlike
	 * the original assembly language version cannot generate unaligned
	 * accesses which might be faulted by some systems.
	 *
	 * Briefly, there are 16 entries in ccTable, one for each possible
	 * value of the condition part of an instruction. Each entry has one
	 * bit for each possible value of the flags in the PSR. The table
	 * entry is extracted using the condition part of the instruction and
	 * the bits are indexed using the value obtained by extracting the
	 * flags from the PSR. If the bit so obtained is 1, the instruction
	 * will be executed.
	 */

	PRINTK("Index %x\n", ((instr >> 28) & 0xF));
	PRINTK("Value %x\n", (ccTable[(instr >> 28) & 0xF]));
	PRINTK("CPSRd %x\n", (pRegs->ARM_cpsr >> 28) & 0xF);
	PRINTK(KERN_ALERT "Res %x\n",	\
	       ((ccTable[(instr >> 28) & 0xF] >> \
		 ((pRegs->ARM_cpsr >> 28) & 0xF))));

	if (((ccTable[(instr >> 28) & 0xF] >>
	      ((pRegs->ARM_cpsr >> 28) & 0xF)) & 1) == 0)
		return (uint32_t *) nPc; /* instruction will not be executed */

	/*
	 * This instruction WILL be executed so look at its type
	 * We're looking for anything that affects the PC e.g.
	 *    B
	 *    BL
	 *    any data processing op where PC is the destination
	 *    any LDR with the PC as the destination
	 *    any LDM with the PC in the list of registers to be loaded
	 *
	 * Following code is derived from the ARM symbolic debugger.
	 */

	switch (BITS(instr, 24, 27)) {
	case 1:		/* check for halfword or signed byte load to PC */
		if (BITSET(instr, 4) && BITSET(instr, 7) && BITSET(instr, 20) &&
		    BITS(instr, 5, 6) != 0 && BITS(instr, 12, 15) == 15)
			break;	/* bad instruction */

		/* FALL THROUGH */

	case 0:		/* data processing */
	case 2:
	case 3:
		{
			uint32_t rn, op1, op2, cFlag;

			if (BITS(instr, 12, 15) != 15)
				/* Rd */
				/* operation does not affect PC */
				break;

			if (BITS(instr, 22, 25) == 0 && BITS(instr, 4, 7) == 9)
				/* multiply with PC as destination not
				 * allowed */
				break;

			if (BITS(instr, 4, 23) == 0x2FFF1) {
				/* BX */
				rn = BITS(instr, 0, 3);
				nPc = (rn == 15 ? pc + 8 :
				       pRegs->uregs[rn]) & ~1;
				break;
			}

			if (BITS(instr, 4, 23) == 0x2FFF3) {
				/* BLX */
				rn = BITS(instr, 0, 3);
				nPc = (rn == 15 ? pc + 8 : pRegs->uregs[rn]);
				break;
			}

			cFlag = BITSET(pRegs->ARM_cpsr, 29);
			rn = BITS(instr, 16, 19);
			op1 = rn == 15 ? pc + 8 : pRegs->uregs[rn];

			if (BITSET(instr, 25)) {
				uint32_t immVal, rotate;

				immVal = BITS(instr, 0, 7);
				rotate = 2 * BITS(instr, 8, 11);
				op2 =
				    (immVal >> rotate) | (immVal <<
							  (32 - rotate));
			} else
				op2 = armShiftedRegVal(pRegs, instr, cFlag);

			switch (BITS(instr, 21, 24)) {
			case 0x0:	/* AND */
				nPc = op1 & op2;
				break;
			case 0x1:	/* EOR */
				nPc = op1 ^ op2;
				break;
			case 0x2:	/* SUB */
				nPc = op1 - op2;
				break;
			case 0x3:	/* RSB */
				nPc = op2 - op1;
				break;
			case 0x4:	/* ADD */
				nPc = op1 + op2;
				break;
			case 0x5:	/* ADC */
				nPc = op1 + op2 + cFlag;
				break;
			case 0x6:	/* SBC */
				nPc = op1 - op2 + cFlag;
				break;
			case 0x7:	/* RSC */
				nPc = op2 - op1 + cFlag;
				break;
			case 0x8:	/* TST */
			case 0x9:	/* TEQ */
			case 0xa:	/* CMP */
			case 0xb:	/* CMN */
				break;
			case 0xc:	/* ORR */
				nPc = op1 | op2;
				break;
			case 0xd:	/* MOV */
				nPc = op2;
				break;
			case 0xe:	/* BIC */
				nPc = op1 & ~op2;
				break;
			case 0xf:	/* MVN */
				nPc = ~op2;
				break;
			}
		}
		break;

	case 4:		/* data transfer */
	case 5:
	case 6:
	case 7:
		if (BITSET(instr, 20) && BITS(instr, 12, 15) == 15 &&
		    !BITSET(instr, 22))
			/* load, PC and not a byte load */
		{
			uint32_t rn, cFlag, base;
			int32_t offset;

			rn = BITS(instr, 16, 19);
			base = rn == 15 ? pc + 8 : pRegs->uregs[rn];
			cFlag = BITSET(pRegs->ARM_cpsr, 29);
			offset = BITSET(instr, 25)
			    ? armShiftedRegVal(pRegs, instr, cFlag)
			    : BITS(instr, 0, 11);

			if (!BITSET(instr, 23))	/* down */
				offset = -offset;

			if (BITSET(instr, 24))	/* pre-indexed */
				base += offset;

			nPc = *(uint32_t *) base;

			/*
			 * don't check for nPc == pc like the ARM
			 * debugger does but let the higher level (or
			 * user) notice.
			 */
		}
		break;

	case 8:
	case 9:		/* block transfer */
		if (BITSET(instr, 20) && BITSET(instr, 15)) { /* loading PC */
			uint32_t rn;
			int32_t offset = 0;

			rn = BITS(instr, 16, 19);
			if (BITSET(instr, 23)) {	/* up */
				uint32_t regBit, regList;

				for (regList = BITS(instr, 0, 14); regList != 0;
				     regList &= ~regBit) {
					regBit = regList & (-regList);
					offset += 4;
				}
				if (BITSET(instr, 24))	/* preincrement */
					offset += 4;
			} else if (BITSET(instr, 24))
				/* predecrement */
				offset = -4;

			nPc = *(uint32_t *) (pRegs->uregs[rn] + offset);

			/*
			 * don't check for nPc == pc like the ARM
			 * debugger does but let the higher level (or
			 * user) notice.
			 */
		}
		break;

	case 0xA:		/* branch */
	case 0xB:		/* branch & link */
		/*
		 * extract offset, sign extend it and add it to current PC,
		 * adjusting for the pipeline
		 */
		nPc = pc + 8 + ((int32_t) (instr << 8) >> 6);
		break;

	case 0xC:
	case 0xD:
	case 0xE:		/* coproc ops */
	case 0xF:		/* SWI */
		break;
	}

	return (uint32_t *) nPc;

}				/* armGetNpc() */
/* </WRS_ADDED> */
