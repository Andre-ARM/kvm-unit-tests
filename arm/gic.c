/*
 * GIC tests
 *
 * GICv2
 *   + test sending/receiving IPIs
 *   + MMIO access tests
 * GICv3
 *   + test sending/receiving IPIs
 *   + MMIO access tests
 *
 * Copyright (C) 2016, Red Hat Inc, Andrew Jones <drjones@redhat.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.
 */
#include <libcflat.h>
#include <asm/setup.h>
#include <asm/processor.h>
#include <asm/delay.h>
#include <asm/gic.h>
#include <asm/smp.h>
#include <asm/barrier.h>
#include <asm/io.h>

#define IPI_SENDER	1
#define IPI_IRQ		1
#define SPI_IRQ		(GIC_FIRST_SPI + 30)

struct gic {
	struct {
		void (*send_self)(void);
		void (*send_broadcast)(void);
	} ipi;
};

static struct gic *gic;
static int acked[2][NR_CPUS], spurious[2][NR_CPUS];
static int bad_sender[2][NR_CPUS], bad_irq[2][NR_CPUS];
static cpumask_t ready;

static void nr_cpu_check(int nr)
{
	if (nr_cpus < nr)
		report_abort("At least %d cpus required", nr);
}

static void wait_on_ready(void)
{
	cpumask_set_cpu(smp_processor_id(), &ready);
	while (!cpumask_full(&ready))
		cpu_relax();
}

static void stats_reset(void)
{
	int i;

	for (i = 0; i < nr_cpus; ++i) {
		acked[0][i] = 0;
		acked[1][i] = 0;
		bad_sender[0][i] = -1;
		bad_sender[1][i] = -1;
		bad_irq[0][i] = -1;
		bad_irq[1][i] = -1;
	}
	smp_wmb();
}

static int check_acked(const char *testname, cpumask_t *mask, int group)
{
	int missing = 0, extra = 0, unexpected = 0;
	int nr_pass, cpu, i;
	bool bad = false;
	bool noirqs = cpumask_empty(mask);

	/* Wait up to 5s for all interrupts to be delivered */
	for (i = 0; i < (noirqs ? 15 : 50); ++i) {
		mdelay(100);
		nr_pass = 0;
		for_each_present_cpu(cpu) {
			smp_rmb();
			nr_pass += cpumask_test_cpu(cpu, mask) ?
				acked[group][cpu] == 1 : acked[group][cpu] == 0;

			if (bad_sender[group][cpu] != -1) {
				printf("cpu%d received IPI from wrong sender %d\n",
					cpu, bad_sender[group][cpu]);
				bad = true;
			}

			if (bad_irq[group][cpu] != -1) {
				printf("cpu%d received wrong irq %d\n",
					cpu, bad_irq[group][cpu]);
				bad = true;
			}
		}
		if (!noirqs && nr_pass == nr_cpus) {
			if (testname) {
				report("%s", !bad, testname);
				if (i)
					report_info("took more than %d ms",
						    i * 100);
			}
			return i * 100;
		}
	}

	if (noirqs && nr_pass == nr_cpus) {
		if (testname)
			report("%s", !bad, testname);
		return i * 100;
	}

	for_each_present_cpu(cpu) {
		if (cpumask_test_cpu(cpu, mask)) {
			if (!acked[group][cpu])
				++missing;
			else if (acked[group][cpu] > 1)
				++extra;
		} else {
			if (acked[group][cpu])
				++unexpected;
		}
	}

	if (testname)
		report("%s", false, testname);
	report_info("Timed-out (5s). ACKS: missing=%d extra=%d unexpected=%d",
		    missing, extra, unexpected);
	return -1;
}

static void check_spurious(void)
{
	int cpu;

	smp_rmb();
	for_each_present_cpu(cpu) {
		if (spurious[0][cpu])
			report_info("WARN: cpu%d got %d spurious FIQs",
				    cpu, spurious[0][cpu]);
		if (spurious[1][cpu])
			report_info("WARN: cpu%d got %d spurious IRQs",
				    cpu, spurious[1][cpu]);
	}
}

static void check_ipi_sender(u32 irqstat)
{
	if (gic_version() == 2) {
		int src = (irqstat >> 10) & 7;

		if (src != IPI_SENDER)
			bad_sender[1][smp_processor_id()] = src;
	}
}

static void check_irqnr(u32 irqnr, int expected, int group)
{
	if (irqnr != expected)
		bad_irq[group][smp_processor_id()] = irqnr;
}

static void irq_handler(struct pt_regs *regs __unused)
{
	u32 irqstat = gic_read_iar(1);
	u32 irqnr = gic_iar_irqnr(irqstat);

	if (irqnr == GICC_INT_SPURIOUS) {
		++spurious[1][smp_processor_id()];
		smp_wmb();
		return;
	}

	gic_write_eoir(irqstat, 1);

	smp_rmb(); /* pairs with wmb in stats_reset */
	++acked[1][smp_processor_id()];
	if (irqnr < GIC_NR_PRIVATE_IRQS) {
		check_ipi_sender(irqstat);
		check_irqnr(irqnr, IPI_IRQ, 1);
	} else {
		check_irqnr(irqnr, SPI_IRQ, 1);
	}
	smp_wmb(); /* pairs with rmb in check_acked */
}

static inline void fiq_handler(struct pt_regs *regs __unused)
{
	u32 irqstat = gic_read_iar(0);
	u32 irqnr = gic_iar_irqnr(irqstat);

	if (irqnr == GICC_INT_SPURIOUS) {
		++spurious[0][smp_processor_id()];
		smp_wmb();
		return;
	}

	gic_write_eoir(irqstat, 0);

	smp_rmb(); /* pairs with wmb in stats_reset */
	++acked[0][smp_processor_id()];
	if (irqnr < GIC_NR_PRIVATE_IRQS) {
		check_ipi_sender(irqstat);
		check_irqnr(irqnr, IPI_IRQ, 0);
	} else {
		check_irqnr(irqnr, SPI_IRQ, 0);
	}
	smp_wmb(); /* pairs with rmb in check_acked */
}

static void gicv2_ipi_send_self(void)
{
	writel(2 << 24 | IPI_IRQ, gicv2_dist_base() + GICD_SGIR);
}

static void gicv2_ipi_send_broadcast(void)
{
	writel(1 << 24 | IPI_IRQ, gicv2_dist_base() + GICD_SGIR);
}

static void gicv3_ipi_send_self(void)
{
	gic_ipi_send_single(IPI_IRQ, smp_processor_id());
}

static void gicv3_ipi_send_broadcast(void)
{
	gicv3_write_sgi1r(1ULL << 40 | IPI_IRQ << 24);
	isb();
}

static void ipi_test_self(void)
{
	cpumask_t mask;

	report_prefix_push("self");
	stats_reset();
	cpumask_clear(&mask);
	cpumask_set_cpu(smp_processor_id(), &mask);
	gic->ipi.send_self();
	check_acked("IPI: self", &mask, 1);
	report_prefix_pop();
}

static void ipi_test_smp(void)
{
	cpumask_t mask;
	int i;

	report_prefix_push("target-list");
	stats_reset();
	cpumask_copy(&mask, &cpu_present_mask);
	for (i = smp_processor_id() & 1; i < nr_cpus; i += 2)
		cpumask_clear_cpu(i, &mask);
	gic_ipi_send_mask(IPI_IRQ, &mask);
	check_acked("IPI: directed", &mask, 1);
	report_prefix_pop();

	report_prefix_push("broadcast");
	stats_reset();
	cpumask_copy(&mask, &cpu_present_mask);
	cpumask_clear_cpu(smp_processor_id(), &mask);
	gic->ipi.send_broadcast();
	check_acked("IPI: broadcast", &mask, 1);
	report_prefix_pop();
}

static void irqs_enable(void)
{
	gic_enable_defaults();
#ifdef __arm__
	install_exception_handler(EXCPTN_IRQ, irq_handler);
#else
	install_irq_handler(EL1H_IRQ, irq_handler);
#endif
	local_irq_enable();
}

static void ipi_send(void)
{
	irqs_enable();
	wait_on_ready();
	ipi_test_self();
	ipi_test_smp();
	check_spurious();
	exit(report_summary());
}

static void irq_recv(void)
{
	irqs_enable();
	cpumask_set_cpu(smp_processor_id(), &ready);
	while (1)
		wfi();
}

static void ipi_test(void *data __unused)
{
	if (smp_processor_id() == IPI_SENDER)
		ipi_send();
	else
		irq_recv();
}

static struct gic gicv2 = {
	.ipi = {
		.send_self = gicv2_ipi_send_self,
		.send_broadcast = gicv2_ipi_send_broadcast,
	},
};

static struct gic gicv3 = {
	.ipi = {
		.send_self = gicv3_ipi_send_self,
		.send_broadcast = gicv3_ipi_send_broadcast,
	},
};

static void ipi_clear_active_handler(struct pt_regs *regs __unused)
{
	u32 irqstat = gic_read_iar(1);
	u32 irqnr = gic_iar_irqnr(irqstat);

	if (irqnr != GICC_INT_SPURIOUS) {
		void *base;
		u32 val = 1 << IPI_IRQ;

		if (gic_version() == 2)
			base = gicv2_dist_base();
		else
			base = gicv3_sgi_base();

		writel(val, base + GICD_ICACTIVER);

		smp_rmb(); /* pairs with wmb in stats_reset */
		++acked[1][smp_processor_id()];
		check_irqnr(irqnr, IPI_IRQ, 1);
		smp_wmb(); /* pairs with rmb in check_acked */
	} else {
		++spurious[1][smp_processor_id()];
		smp_wmb();
	}
}

static void run_active_clear_test(void)
{
	report_prefix_push("active");
	gic_enable_defaults();
#ifdef __arm__
	install_exception_handler(EXCPTN_IRQ, ipi_clear_active_handler);
#else
	install_irq_handler(EL1H_IRQ, ipi_clear_active_handler);
#endif
	local_irq_enable();

	ipi_test_self();
	report_prefix_pop();
}

static bool test_ro_pattern_32(void *address, u32 pattern, u32 orig)
{
	u32 reg;

	writel(pattern, address);
	reg = readl(address);

	if (reg != orig)
		writel(orig, address);

	return reg == orig;
}

static bool test_readonly_32(void *address, bool razwi)
{
	u32 orig, pattern;

	orig = readl(address);
	if (razwi && orig)
		return false;

	pattern = 0xffffffff;
	if (orig != pattern) {
		if (!test_ro_pattern_32(address, pattern, orig))
			return false;
	}

	pattern = 0xa5a55a5a;
	if (orig != pattern) {
		if (!test_ro_pattern_32(address, pattern, orig))
			return false;
	}

	pattern = 0;
	if (orig != pattern) {
		if (!test_ro_pattern_32(address, pattern, orig))
			return false;
	}

	return true;
}

static void test_typer_v2(uint32_t reg)
{
	int nr_gic_cpus = ((reg >> 5) & 0x7) + 1;

	report_info("nr_cpus=%d", nr_cpus);
	report("all CPUs have interrupts", nr_cpus == nr_gic_cpus);
}

#define BYTE(reg32, byte) (((reg32) >> ((byte) * 8)) & 0xff)
#define REPLACE_BYTE(reg32, byte, new) (((reg32) & ~(0xff << ((byte) * 8))) |\
					((new) << ((byte) * 8)))

/*
 * Some registers are byte accessible, do a byte-wide read and write of known
 * content to check for this.
 * Apply a @mask to cater for special register properties.
 * @pattern contains the value already in the register.
 */
static void test_byte_access(void *base_addr, u32 pattern, u32 mask)
{
	u32 reg = readb(base_addr + 1);
	bool res;

	res = (reg == (BYTE(pattern, 1) & (mask >> 8)));
	report("byte reads successful", res);
	if (!res)
		report_info("byte 1 of 0x%08x => 0x%02x", pattern & mask, reg);

	pattern = REPLACE_BYTE(pattern, 2, 0x1f);
	writeb(BYTE(pattern, 2), base_addr + 2);
	reg = readl(base_addr);
	res = (reg == (pattern & mask));
	report("byte writes successful", res);
	if (!res)
		report_info("writing 0x%02x into bytes 2 => 0x%08x",
			    BYTE(pattern, 2), reg);
}

static void test_priorities(int nr_irqs, void *priptr)
{
	u32 orig_prio, reg, pri_bits;
	u32 pri_mask, pattern;
	void *first_spi = priptr + GIC_FIRST_SPI;

	orig_prio = readl(first_spi);
	report_prefix_push("IPRIORITYR");

	/*
	 * Determine implemented number of priority bits by writing all 1's
	 * and checking the number of cleared bits in the value read back.
	 */
	writel(0xffffffff, first_spi);
	pri_mask = readl(first_spi);

	reg = ~pri_mask;
	report("consistent priority masking",
	       (((reg >> 16) == (reg & 0xffff)) &&
	        ((reg & 0xff) == ((reg >> 8) & 0xff))));
	report_info("priority mask is 0x%08x", pri_mask);

	reg = reg & 0xff;
	for (pri_bits = 8; reg & 1; reg >>= 1, pri_bits--)
		;
	report("implements at least 4 priority bits", pri_bits >= 4);
	report_info("%d priority bits implemented", pri_bits);

	pattern = 0;
	writel(pattern, first_spi);
	report("clearing priorities", readl(first_spi) == pattern);

	/* setting all priorities to their max valus was tested above */

	report("accesses beyond limit RAZ/WI",
	       test_readonly_32(priptr + nr_irqs, true));

	writel(pattern, priptr + nr_irqs - 4);
	report("accessing last SPIs",
	       readl(priptr + nr_irqs - 4) == (pattern & pri_mask));

	pattern = 0xff7fbf3f;
	writel(pattern, first_spi);
	report("priorities are preserved",
	       readl(first_spi) == (pattern & pri_mask));

	/* The PRIORITY registers are byte accessible. */
	test_byte_access(first_spi, pattern, pri_mask);

	report_prefix_pop();
	writel(orig_prio, first_spi);
}

/* GICD_ITARGETSR is only used by GICv2. */
static void test_targets(int nr_irqs)
{
	void *targetsptr = gicv2_dist_base() + GICD_ITARGETSR;
	u32 orig_targets;
	u32 cpu_mask;
	u32 pattern, reg;

	orig_targets = readl(targetsptr + GIC_FIRST_SPI);
	report_prefix_push("ITARGETSR");

	cpu_mask = (1 << nr_cpus) - 1;
	cpu_mask |= cpu_mask << 8;
	cpu_mask |= cpu_mask << 16;

	/* Check that bits for non implemented CPUs are RAZ/WI. */
	if (nr_cpus < 8) {
		writel(0xffffffff, targetsptr + GIC_FIRST_SPI);
		report("bits for non-existent CPUs masked",
		       !(readl(targetsptr + GIC_FIRST_SPI) & ~cpu_mask));
		report_info("%d non-existent CPUs", 8 - nr_cpus);
	} else {
		report_skip("CPU masking (all CPUs implemented)");
	}

	report("accesses beyond limit RAZ/WI",
	       test_readonly_32(targetsptr + nr_irqs, true));

	pattern = 0x0103020f;
	writel(pattern, targetsptr + GIC_FIRST_SPI);
	reg = readl(targetsptr + GIC_FIRST_SPI);
	report("register content preserved", reg == (pattern & cpu_mask));
	if (reg != (pattern & cpu_mask))
		report_info("writing %08x reads back as %08x",
			    pattern & cpu_mask, reg);

	/* The TARGETS registers are byte accessible. */
	test_byte_access(targetsptr + GIC_FIRST_SPI, pattern, cpu_mask);

	writel(orig_targets, targetsptr + GIC_FIRST_SPI);

	report_prefix_pop();
}

static void gic_test_mmio(void)
{
	u32 reg;
	int nr_irqs;
	void *gic_dist_base, *idreg;

	switch(gic_version()) {
	case 0x2:
		gic_dist_base = gicv2_dist_base();
		idreg = gic_dist_base + GICD_ICPIDR2;
		break;
	case 0x3:
		/*
		 * We only test generic registers or those affecting
		 * SPIs, so don't need to consider the SGI base in
		 * the redistributor here.
		 */
		gic_dist_base = gicv3_dist_base();
		idreg = gic_dist_base + GICD_PIDR2;
		break;
	default:
		report_abort("GIC version %d not supported", gic_version());
	}

	reg = readl(gic_dist_base + GICD_TYPER);
	nr_irqs = GICD_TYPER_IRQS(reg);
	report_info("number of implemented SPIs: %d", nr_irqs - GIC_FIRST_SPI);
	report_info("GIC %s security extension",
		reg & (1U << 10) ? "has" : "does not have");

	if (gic_version() == 0x2)
		test_typer_v2(reg);

	report_info("IIDR: 0x%08x", readl(gic_dist_base + GICD_IIDR));

	report("GICD_TYPER is read-only",
	       test_readonly_32(gic_dist_base + GICD_TYPER, false));
	report("GICD_IIDR is read-only",
	       test_readonly_32(gic_dist_base + GICD_IIDR, false));

	reg = readl(idreg);
	report("ICPIDR2 is read-only", test_readonly_32(idreg, false));
	report_info("value of ICPIDR2: 0x%08x", reg);

	test_priorities(nr_irqs, gic_dist_base + GICD_IPRIORITYR);

	if (gic_version() == 2)
		test_targets(nr_irqs);
}

static void gic_spi_trigger(int irq)
{
	gic_set_irq_bit(irq, GICD_ISPENDR);
}

static void spi_configure_irq(int irq, int cpu)
{
	gic_set_irq_target(irq, cpu);
	gic_set_irq_priority(irq, 0xa0);
	gic_enable_irq(irq);
}

#define IRQ_STAT_NONE		0
#define IRQ_STAT_IRQ		1
#define IRQ_STAT_TYPE_MASK	0x3
#define IRQ_STAT_NO_CLEAR	4

/*
 * Wait for an SPI to fire (or not) on a certain CPU.
 * Clears the pending bit if requested afterwards.
 */
static bool trigger_and_check_spi(const char *test_name,
				  unsigned int irq_stat,
				  int cpu)
{
	cpumask_t cpumask;
	bool ret = true;

	stats_reset();
	gic_spi_trigger(SPI_IRQ);
	cpumask_clear(&cpumask);
	switch (irq_stat & IRQ_STAT_TYPE_MASK) {
	case IRQ_STAT_NONE:
		break;
	case IRQ_STAT_IRQ:
		cpumask_set_cpu(cpu, &cpumask);
		break;
	}

	ret = (check_acked(test_name, &cpumask, 1) >= 0);

	/* Clean up pending bit in case this IRQ wasn't taken. */
	if (!(irq_stat & IRQ_STAT_NO_CLEAR))
		gic_set_irq_bit(SPI_IRQ, GICD_ICPENDR);

	return ret;
}

static void spi_test_single(void)
{
	cpumask_t cpumask;
	int cpu = smp_processor_id();

	spi_configure_irq(SPI_IRQ, cpu);

	trigger_and_check_spi("SPI triggered by CPU write", IRQ_STAT_IRQ, cpu);

	gic_disable_irq(SPI_IRQ);
	trigger_and_check_spi("disabled SPI does not fire",
			      IRQ_STAT_NONE | IRQ_STAT_NO_CLEAR, cpu);

	stats_reset();
	cpumask_clear(&cpumask);
	cpumask_set_cpu(cpu, &cpumask);
	gic_enable_irq(SPI_IRQ);
	check_acked("now enabled SPI fires", &cpumask, 1);
}

static void spi_test_smp(void)
{
	int cpu;
	int cores = 1;

	wait_on_ready();
	for_each_present_cpu(cpu) {
		if (cpu == smp_processor_id())
			continue;
		spi_configure_irq(SPI_IRQ, cpu);
		if (trigger_and_check_spi(NULL, IRQ_STAT_IRQ, cpu))
			cores++;
		else
			report_info("SPI delivery failed on core %d", cpu);
	}
	report("SPI delievered on all cores", cores == nr_cpus);
}

#define GICD_CTLR_ENABLE_BOTH (GICD_CTLR_ENABLE_G0 | GICD_CTLR_ENABLE_G1)

/*
 * Check the security state configuration of the GIC.
 * Test whether we can switch to a single security state, to test both
 * group 0 and group 1 interrupts.
 * Architecturally a GIC can be configured in different ways, so we don't
 * insist on the current way KVM emulates the GIC.
 */
static bool gicv3_check_security(void *gicd_base)
{
	u32 ctlr = readl(gicd_base + GICD_CTLR);

	if (ctlr & GICD_CTLR_DS) {
		writel(ctlr & ~GICD_CTLR_DS, gicd_base + GICD_CTLR);
		ctlr = readl(gicd_base + GICD_CTLR);
		if (!(ctlr & GICD_CTLR_DS))
			report_info("GIC allowing two security states");
		else
			report_info("GIC is one security state only");
	} else {
		report_info("GIC resets to two security states");
	}

	writel(ctlr | GICD_CTLR_DS, gicd_base + GICD_CTLR);
	ctlr = readl(gicd_base + GICD_CTLR);
	report("switching to single security state", ctlr & GICD_CTLR_DS);

	/* Group0 delivery only works in single security state. */
	return ctlr & GICD_CTLR_DS;
}

/*
 * The GIC architecture describes two interrupt groups, group 0 and group 1.
 * On bare-metal systems, running in non-secure world on a GIC with the
 * security extensions, there is only one group available: group 1.
 * However in the kernel KVM emulates a GIC with only one security state,
 * so both groups are available to guests.
 * Check whether this works as expected (as Linux will not use this feature).
 * We can only verify this state on a GICv3, so we check it there and silently
 * assume it's valid for GICv2.
 */
static void test_irq_group(void *gicd_base)
{
	bool is_gicv3 = (gic_version() == 3);
	u32 reg;

	report_prefix_push("GROUP");
	gic_enable_defaults();

	if (is_gicv3) {
		/* GICv3 features a bit to read and set the security state. */
		if (!gicv3_check_security(gicd_base))
			return;
	}

	/* Check whether the group enable bits stick. */
	reg = readl(gicd_base + GICD_CTLR);
	writel(reg & ~GICD_CTLR_ENABLE_BOTH, gicd_base + GICD_CTLR);
	reg = readl(gicd_base + GICD_CTLR);
	report("both groups disabled sticks",
	       (reg & GICD_CTLR_ENABLE_BOTH) == 0);

	reg &= ~GICD_CTLR_ENABLE_BOTH;
	writel(reg | GICD_CTLR_ENABLE_G1, gicd_base + GICD_CTLR);
	reg = readl(gicd_base + GICD_CTLR);
	report("group 1 enabled sticks",
	       (reg & GICD_CTLR_ENABLE_BOTH) == GICD_CTLR_ENABLE_G1);

	reg &= ~GICD_CTLR_ENABLE_BOTH;
	writel(reg | GICD_CTLR_ENABLE_G0, gicd_base + GICD_CTLR);
	reg = readl(gicd_base + GICD_CTLR);
	report("group 0 enabled sticks",
	       (reg & GICD_CTLR_ENABLE_BOTH) == GICD_CTLR_ENABLE_G0);

	/*
	 * On a security aware GIC in non-secure world the IGROUPR registers
	 * are RAZ/WI. KVM emulates a single-security-state GIC, so both
	 * groups are available and the IGROUPR registers are writable.
	 */
	reg = gic_get_irq_group(SPI_IRQ);
	gic_set_irq_group(SPI_IRQ, !reg);
	report("IGROUPR is writable", gic_get_irq_group(SPI_IRQ) != reg);
	gic_set_irq_group(SPI_IRQ, reg);
}

static void spi_send(void)
{
	irqs_enable();

	spi_test_single();

	if (nr_cpus > 1)
		spi_test_smp();

	if (gic_version() == 3)
		test_irq_group(gicv3_dist_base());

	if (gic_version() == 2)
		test_irq_group(gicv2_dist_base());

	check_spurious();
	exit(report_summary());
}

static void spi_test(void *data __unused)
{
	if (smp_processor_id() == 0)
		spi_send();
	else
		irq_recv();
}

int main(int argc, char **argv)
{
	if (!gic_init()) {
		printf("No supported gic present, skipping tests...\n");
		return report_summary();
	}

	report_prefix_pushf("gicv%d", gic_version());

	switch (gic_version()) {
	case 2:
		gic = &gicv2;
		break;
	case 3:
		gic = &gicv3;
		break;
	}

	if (argc < 2)
		report_abort("no test specified");

	if (strcmp(argv[1], "ipi") == 0) {
		report_prefix_push(argv[1]);
		nr_cpu_check(2);
		on_cpus(ipi_test, NULL);
	} else if (strcmp(argv[1], "active") == 0) {
		run_active_clear_test();
	} else if (strcmp(argv[1], "mmio") == 0) {
		report_prefix_push(argv[1]);
		gic_test_mmio();
		report_prefix_pop();
	} else if (strcmp(argv[1], "irq") == 0) {
		report_prefix_push(argv[1]);
		on_cpus(spi_test, NULL);
		report_prefix_pop();
	} else {
		report_abort("Unknown subtest '%s'", argv[1]);
	}

	return report_summary();
}
