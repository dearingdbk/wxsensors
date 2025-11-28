#ifndef Q1_31_H
#define Q1_31_H

#include <stdint.h>

typedef int32_t q1_31;

#define Q1_SHIFT 31
#define Q1_SCALE (1LL << Q1_SHIFT)

// Convert double → q1.31
static inline q1_31 q1_from_double(double x) {
return (q1_31)(x * (double)Q1_SCALE);
}

// Convert q1.31 → double
static inline double q1_to_double(q1_31 x) {
return (double)x / (double)Q1_SCALE;
}

// Add
static inline q1_31 q1_add(q1_31 a, q1_31 b) {
return a + b;
}

// Subtract
static inline q1_31 q1_sub(q1_31 a, q1_31 b) {
return a - b;
}

// Multiply (64-bit intermediate!)
static inline q1_31 q1_mul(q1_31 a, q1_31 b) {
int64_t temp = (int64_t)a * (int64_t)b;
return (q1_31)(temp >> Q1_SHIFT);
}

// Divide
static inline q1_31 q1_div(q1_31 a, q1_31 b) {
int64_t temp = (int64_t)a << Q1_SHIFT;
return (q1_31)(temp / b);
}

#endif // Q1_31_H

