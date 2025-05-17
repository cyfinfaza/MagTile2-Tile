/*
 * registry.c
 *
 *  Created on: May 16, 2025
 *      Author: cyfin
 */

#include "registry.h"
#include <string.h>

extern const Registry_RegConfig registry_table[];
extern const uint32_t registry_count;

uint8_t Registry_Read(uint8_t id, void *out_buf) {
    if (id >= registry_count) return 0;
    const Registry_RegConfig *cfg = &registry_table[id];
    if (cfg->addr == NULL || cfg->size == 0) return 0;
    memcpy(out_buf, cfg->addr, cfg->size);
    return 1;
}

uint8_t Registry_Write(uint8_t id, const void *in_buf) {
    if (id >= registry_count) return 0;
    const Registry_RegConfig *cfg = &registry_table[id];
    if (cfg->access != Registry_READWRITE || cfg->addr == NULL || cfg->size == 0)
        return 0;
    memcpy((void *)cfg->addr, in_buf, cfg->size);
    return 1;
}

uint32_t Registry_GetTotal(void) {
    return registry_count;
}
