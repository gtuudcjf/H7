/* TinyCC 0.9.27's bundled MinGW fabsf uses an unsupported x87 constraint.
 * Use the standard double-precision fabs for host tests only. Every float
 * is exactly representable as double, so the result is unchanged. */
#ifndef H7_HOST_MATH_COMPAT_H
#define H7_HOST_MATH_COMPAT_H
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#if defined(__TINYC__) && defined(_WIN32)
#define fabsf(value) ((float)fabs((double)(value)))
/* The bundled legacy MSVCRT import library lacks these C99 float exports. */
#define sinf(value) ((float)sin((double)(value)))
#define cosf(value) ((float)cos((double)(value)))
#define sqrtf(value) ((float)sqrt((double)(value)))
#define floorf(value) ((float)floor((double)(value)))
#define fmodf(a, b) ((float)fmod((double)(a), (double)(b)))
/* MinGW's default assertion opens a modal dialog when launched without a
 * console. Keep assertions fatal and observable for unattended test runs. */
#undef assert
#define assert(condition) ((condition) ? (void)0 : \
    (fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__, \
             #condition), exit(1)))
#endif
#endif
