#include "arm_math.h"

#define log10f_fast(x)  (log2f_approx(x)*0.3010299956639812f)
#define pow10f(x) expf(2.302585092994046f*x)

float32_t Leq(float32_t *InputData, uint32_t inSize);
float32_t log2f_approx(float X);

void arm_biquad_cascade_df2T_f32Mod(
const arm_biquad_cascade_df2T_instance_f32 * S,
float32_t * pSrc,
float32_t * pDst,
uint32_t blockSize, const float32_t * gainVec);

void arm_biquad_cascade_df2T_f32Mod2(
const arm_biquad_cascade_df2T_instance_f32 * S,
float32_t * pSrc,
float32_t * pDst,
uint32_t blockSize);
