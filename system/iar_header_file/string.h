/*                       - STRING.H -

   The ANSI 'string' function declarations.

   $Revision: 1.1.2.1 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#ifndef _STRING_INCLUDED
#define _STRING_INCLUDED

#ifndef _SYSTEM_BUILD
  #pragma system_include
#endif

#include "sysmac.h"

#ifndef NULL
#define NULL    ((void*)0)     /* changed from char* 93.01.21 ICLM */
#endif

#if __IAR_SYSTEMS_ICC__ < 2
#if __TID__ & 0x8000
#pragma function=intrinsic(0)
#endif
#endif

#ifndef MEMORY_ATTRIBUTE
#define MEMORY_ATTRIBUTE
#endif

__INTRINSIC MEMORY_ATTRIBUTE void *memcpy(void *, const void *, size_t);

__INTRINSIC MEMORY_ATTRIBUTE void *memmove(void *, const void *, size_t);

__INTRINSIC MEMORY_ATTRIBUTE void *memchr(const void *, int, size_t);

__INTRINSIC MEMORY_ATTRIBUTE void *memset(void *, int, size_t);

__INTRINSIC MEMORY_ATTRIBUTE int memcmp(const void *, const void *, size_t);

__INTRINSIC MEMORY_ATTRIBUTE char *strchr(const char *, int);

__INTRINSIC MEMORY_ATTRIBUTE int strcmp(const char *, const char *);

__INTRINSIC MEMORY_ATTRIBUTE int strncmp(const char *, const char *, size_t);

__INTRINSIC MEMORY_ATTRIBUTE int strcoll(const char *, const char *);

__INTRINSIC MEMORY_ATTRIBUTE size_t strlen(const char *);

__INTRINSIC MEMORY_ATTRIBUTE size_t strcspn(const char *, const char *);

__INTRINSIC MEMORY_ATTRIBUTE size_t strspn(const char *, const char *);

__INTRINSIC MEMORY_ATTRIBUTE char *strpbrk(const char *, const char *);

__INTRINSIC MEMORY_ATTRIBUTE char *strrchr(const char *, int);

__INTRINSIC MEMORY_ATTRIBUTE char *strstr(const char *, const char *);

__INTRINSIC MEMORY_ATTRIBUTE char *strcat(char *, const char *);

__INTRINSIC MEMORY_ATTRIBUTE char *strncat(char *, const char *, size_t);

__INTRINSIC MEMORY_ATTRIBUTE char *strcpy(char *, const char *);

__INTRINSIC MEMORY_ATTRIBUTE char *strncpy(char *, const char *, size_t);

__INTRINSIC MEMORY_ATTRIBUTE char *strerror(int);

__INTRINSIC MEMORY_ATTRIBUTE char *strtok(char *, const char *);

__INTRINSIC MEMORY_ATTRIBUTE size_t strxfrm(char *, const char *, size_t);

#if __IAR_SYSTEMS_ICC__ < 2
#if __TID__ & 0x8000
#pragma function=default
#endif
#endif

#endif
