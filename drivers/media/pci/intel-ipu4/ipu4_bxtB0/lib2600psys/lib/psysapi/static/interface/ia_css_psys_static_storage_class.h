/*
* Support for Intel Camera Imaging ISP subsystem.
* Copyright (c) 2010 - 2016, Intel Corporation.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*/

#ifndef __IA_CSS_PSYS_STATIC_STORAGE_CLASS_H
#define __IA_CSS_PSYS_STATIC_STORAGE_CLASS_H

#include "storage_class.h"

#ifndef _IA_CSS_PSYS_STATIC_INLINE_
#define IA_CSS_PSYS_STATIC_STORAGE_CLASS_H STORAGE_CLASS_EXTERN
#define IA_CSS_PSYS_STATIC_STORAGE_CLASS_C
#else
#define IA_CSS_PSYS_STATIC_STORAGE_CLASS_H STORAGE_CLASS_INLINE
#define IA_CSS_PSYS_STATIC_STORAGE_CLASS_C STORAGE_CLASS_INLINE
#endif

#endif /* __IA_CSS_PSYS_STATIC_STORAGE_CLASS_H */
