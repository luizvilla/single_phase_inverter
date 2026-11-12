/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file inverterFastCode.h
 * @brief INV_FAST_CODE: places a per-period function in fast RAM when the application provides one.
 *
 * On an OwnTech Core whose board declares the CCM SRAM as Zephyr's zephyr,itcm (owntech_fast_code.h), the
 * function runs from RAM at 0 wait states instead of from flash. Elsewhere (older Core, host tests) the macro
 * is empty and the function stays where the compiler puts it.
 */
#ifndef INVERTER_FAST_CODE_H_
#define INVERTER_FAST_CODE_H_

#if defined(__ZEPHYR__) && defined(__has_include)
#if __has_include("owntech_fast_code.h")
#include "owntech_fast_code.h"
#define INV_FAST_CODE OWNTECH_FAST_CODE
#endif
#endif

#ifndef INV_FAST_CODE
#define INV_FAST_CODE
#endif

#endif /* INVERTER_FAST_CODE_H_ */
