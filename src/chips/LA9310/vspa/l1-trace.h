// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#ifndef LIME_L1_TRACE_H
#define LIME_L1_TRACE_H

#include <stdint.h>

typedef struct l1_trace_data_s {
    uint64_t cnt;
    uint32_t msg;
    uint32_t param;
} l1_trace_data_t;

enum {
    T_XFER_BUFFER = 0,
    T_QEC_TX_BUFFER,
    T_QEC_RX_BUFFER,
    T_DEC_BUFFER,
    T_INT_BUFFER,
    T_UNDERRUN,
    T_OVERRUN,
};

typedef struct l1_trace_state_s {
    uint32_t la9310_mem_address;
    uint32_t buffer_size;
    uint32_t bytes_produced;
    uint32_t event_count;
    uint32_t event_drops;
} l1_trace_hif_t; // L1 trace host interface

extern l1_trace_hif_t trace_hif;

extern void l1_trace_init(void);
extern void l1_trace_clear(void);
void l1_trace_upload(void);

#endif // LIME_L1_TRACE_H
