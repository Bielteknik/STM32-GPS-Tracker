/*
 * util.h
 *
 *  Created on: 22.01.2012
 *      Author: dimaz
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>

#define SPRNT(N, S, L) stodebug(N, S, sizeof(S), L)

extern uint16_t atos(const char *string);

extern uint16_t atos_len(uint8_t *buf, uint8_t len);

extern void stoa(uint16_t n, char *buf, uint8_t *len);

extern void stoh(uint16_t n, char *buf, uint8_t *len);

extern void stodebug(uint16_t n, char *buf, uint8_t len, uint8_t new_line);

#endif /* UTIL_H_ */
