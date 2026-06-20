#ifndef LIME_VSPA_FEATURES_H
#define LIME_VSPA_FEATURES_H

#include <stdint.h>

typedef enum {
    F_VSPA_NONE = 0,
    F_VSPA_L1_TRACE,

} e_vspa_feature;

typedef struct {
    e_vspa_feature feature;
    uint32_t address;
} feature_t;

#endif