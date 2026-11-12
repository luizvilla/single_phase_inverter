/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file inverterTiming.h
 * @brief Optional cycle-counter probes inside singlePhaseInverterFollowing::calculateDuty().
 *
 * Used by the application's on-target timing tests (Core docs/adr/0005). INV_TIMING_PROBE(i) stores the
 * Cortex-M DWT cycle counter in inv_timing_cycles[i]; the probe at the end of calculateDuty() also increments
 * inv_timing_seq, so a caller can tell a complete call from one that returned early. Without TIMING_HARNESS
 * the probes expand to nothing.
 */
#pragma once

#include <stdint.h>

/** @brief Probe points: entry (after the input checks), after the PLL, after the SOGIs, after the Park
 * transforms, after the sync window, after mode and feedforward, after the current PIs, at return. */
#define INV_TIMING_PROBE_COUNT 8U

#ifdef TIMING_HARNESS
extern volatile uint32_t inv_timing_cycles[INV_TIMING_PROBE_COUNT];
extern volatile uint32_t inv_timing_seq;
#define INV_TIMING_PROBE(i) (inv_timing_cycles[(i)] = *reinterpret_cast<volatile const uint32_t *>(0xE0001004UL))
#define INV_TIMING_DONE() (inv_timing_seq = inv_timing_seq + 1U)
#else
#define INV_TIMING_PROBE(i) ((void)0)
#define INV_TIMING_DONE() ((void)0)
#endif
