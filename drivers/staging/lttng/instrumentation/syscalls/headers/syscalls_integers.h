#ifdef CONFIG_X86_64
#include "x86-64-syscalls-3.10.0-rc7_integers.h"
#endif

#ifdef CONFIG_X86_32
#include "x86-32-syscalls-3.1.0-rc6_integers.h"
#endif

#ifdef CONFIG_ARM
#include "arm-32-syscalls-3.4.25_integers.h"
#endif

#ifdef CONFIG_PPC
#include "powerpc-32-syscalls-3.0.34_integers.h"
#endif

#ifdef CONFIG_CPU_MIPS32
#include "mips-32-syscalls-3.18.0_integers.h"
#endif

#ifdef CONFIG_ARM64
#include "arm-64-syscalls-4.4.0_integers.h"
#endif
