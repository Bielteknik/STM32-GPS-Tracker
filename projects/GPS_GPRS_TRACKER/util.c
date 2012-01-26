/*
 * util.c
 *
 *  Created on: 22.01.2012
 *      Author: dimaz
 */


#include "ch.h"
#include "hal.h"

#include "util.h"


uint16_t atos(const char *string) {
    uint16_t value = 0;
    uint8_t digit;
    char c;

    while ((c = *string++) != '\0') {

        if (c >= '0' && c <= '9')
        	digit = (uint8_t) (c - '0');
        else
            break;

        value = (value * 10) + digit;
    }

    return value;
}

uint16_t atos_len(uint8_t *buf, uint8_t len) {
	uint16_t value = 0;
	uint8_t digit;
	uint8_t i;
	char c;

	for (i = 0; i < len; i++) {
		c = buf[i];

		if (c >= '0' && c <= '9')
			digit = (uint8_t) (c - '0');
		else
			return 0;

		value = (value * 10) + digit;
    }

    return value;
}

void stoa(uint16_t n, char *buf, uint8_t *len) {
  char *p, *left = buf, *right, tmp;
  uint8_t i;

  *len = 0;

  if (!n) {
    buf[0] = '0';
    buf[1] = '\0';
    *len = 2;
  } else {
    p = buf;
    while (n) {
    	*p++ = (n % 10) + '0', n /= 10;
    	(*len)++;
    }

    // Reverse the string
    right = p - 1;

    while (left < right) {
    	tmp      = *left;
    	*left++  = *right;
    	*right-- = tmp;
    }

    *p++ = '\0';
    (*len)++;
  }
}

void stoh(uint16_t n, char *buf, uint8_t *len) {
	char *p, *left = buf, *right, tmp;
	uint8_t i, digit;

	*len = 0;

	if (!n) {
		buf[0] = '0';
		buf[1] = '\0';
		*len = 2;
	} else {
		p = buf;
		while (n) {
			digit = n % 16;

			if (digit < 10) {
				*p++ = digit + '0';
			} else {
				*p++ = digit - 10 + 'A';
			}

			n /= 16;
			(*len)++;
		}

		// Reverse the string
		right = p - 1;

		while (left < right) {
			tmp      = *left;
			*left++  = *right;
			*right-- = tmp;
		}

		*p++ = '\0';
		(*len)++;
	}
}

void stodebug(uint16_t n, char *buf, uint8_t len, uint8_t new_line) {
	char num_buf[10];
	uint8_t num_len;

	stoa(n, num_buf, &num_len);

	sdWrite(&SD1, buf, len);
	sdWrite(&SD1, num_buf, num_len - 1);

	if (new_line)
		sdWrite(&SD1, "\r\n", sizeof("\r\n") - 1);
}
