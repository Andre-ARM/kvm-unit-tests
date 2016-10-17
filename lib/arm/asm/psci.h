#ifndef _ASMARM_PSCI_H_
#define _ASMARM_PSCI_H_
#include <libcflat.h>
#include <linux/psci.h>

typedef int (*psci_invoke_fn)(unsigned long function_id, unsigned long arg0,
			      unsigned long arg1, unsigned long arg2);
extern psci_invoke_fn psci_invoke;
extern int psci_invoke_hvc(unsigned long function_id, unsigned long arg0,
			   unsigned long arg1, unsigned long arg2);
extern int psci_invoke_smc(unsigned long function_id, unsigned long arg0,
			   unsigned long arg1, unsigned long arg2);
extern int psci_cpu_on(unsigned long cpuid, unsigned long entry_point);
extern void psci_sys_reset(void);
extern int cpu_psci_cpu_boot(unsigned int cpu);
extern void cpu_psci_cpu_die(void);

#endif /* _ASMARM_PSCI_H_ */
