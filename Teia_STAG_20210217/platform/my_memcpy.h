#ifndef MY_MEMCPY_H_INCLUDED
#define MY_MEMCPY_H_INCLUDED

#include <stddef.h>

static inline void *memcpy(void* dest, void* src,const size_t len)
{
	for(size_t i=0; i<len; i++)	{
		((uint8_t*)dest)[i]=((uint8_t*)src)[i];
	}
	return dest;
}

#endif /* MY_MEMCPY_H_INCLUDED */
