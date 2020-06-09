#ifndef __ETH_H__
#define __ETH_H__

#include "functions.h"

struct eth {
  unsigned char dst[6];
  unsigned char src[6];
  uint16_t type;
  eth(char *_dst, char *_src, uint16_t _type) {
    memcpy(dst, _dst, 6);
    memcpy(src, _src, 6);
    type = _type;
  }
} __attribute__((packed));

#endif /* __ETH_H__ */
