/*
 * registry.h
 *
 *  Created on: May 16, 2025
 *      Author: cyfin
 */

#ifndef INC_REGISTRY_H_
#define INC_REGISTRY_H_

#include <stdint.h>

typedef enum {
    Registry_READONLY,
    Registry_READWRITE
} Registry_RegAccess;

typedef struct {
	uint8_t enabled : 1;
    Registry_RegAccess access : 3;
    uint8_t size : 4;
    const void *addr;
} Registry_RegConfig;

extern const Registry_RegConfig registry_table[];
extern const uint32_t registry_count;

// Handy macro to define entries
#define Registry_DEFINE(id, addr, access) \
    [id] = { 1, access, sizeof(*(addr)), (const void *)(addr) }

// Registry API
uint8_t Registry_Read(uint8_t id, void *out_buf);
uint8_t Registry_Write(uint8_t id, const void *in_buf);
uint32_t Registry_GetTotal(void);


#endif /* INC_REGISTRY_H_ */
