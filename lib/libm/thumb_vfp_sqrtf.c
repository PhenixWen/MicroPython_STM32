// an implementation of sqrtf for Thumb using hardware VFP instructions

#include <math.h>

float sqrtf(float x) {
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)

    asm volatile (
            "vsqrt.f32  %[r], %[x]\n"
            : [r] "=t" (x)
            : [x] "t"  (x));
#endif
    return x;

}
