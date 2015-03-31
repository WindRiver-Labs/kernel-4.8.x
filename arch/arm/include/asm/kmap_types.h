#ifndef __ARM_KMAP_TYPES_H
#define __ARM_KMAP_TYPES_H

/*
 * This is the "bare minimum".  AIO seems to require this.
 */
#if (NR_CPUS > 15)
/* Prevent overlap between fixmap mapping and CPU vector page for 16th core */
#define KM_TYPE_NR 15
#else
#define KM_TYPE_NR 16
#endif

#endif
