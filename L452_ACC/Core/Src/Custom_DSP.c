#include "Custom_DSP.h"
#include "main.h"



float32_t Leq(float32_t *InputData, uint32_t inSize)
{
	float32_t retVal=0, cte = (float)M_LN10/10;

	/*float32_t InputDataTmp[fast_125ms];
	arm_scale_f32(InputData, 0.1, InputDataTmp, inSize);
	for(int i=0;i<inSize;i++)
    	retVal=retVal + pow10f(InputDataTmp[i]);*/

    for(int i=0;i<inSize;i++)
    	retVal+= expf(InputData[i] * cte);//pow10f(InputData[i]/10);

    return 10*log10f_fast(retVal/inSize);
}

float32_t log2f_approx(float X) {
  float Y, F;
  int E;
  F = frexpf(fabsf(X), &E);
  Y = 1.23149591368684f;
  Y *= F;
  Y += -4.11852516267426f;
  Y *= F;
  Y += 6.02197014179219f;
  Y *= F;
  Y += -3.13396450166353f;
  Y += E;
  return(Y);
}

LOW_OPTIMIZATION_ENTER
void arm_biquad_cascade_df2T_f32Mod(
const arm_biquad_cascade_df2T_instance_f32 * S,
float32_t * pSrc,
float32_t * pDst,
uint32_t blockSize, const float * gainVec)
{

   float32_t *pIn = pSrc;                         /*  source pointer            */
   float32_t *pOut = pDst;                        /*  destination pointer       */
   float32_t *pState = S->pState;                 /*  State pointer             */
   float32_t *pCoeffs = (float32_t*)S->pCoeffs;               /*  coefficient pointer       */
   float32_t acc1;                                /*  accumulator               */
   float32_t b0, b1, b2, a1, a2;                  /*  Filter coefficients       */
   float32_t Xn1;                                 /*  temporary input           */
   float32_t d1, d2;                              /*  state variables           */
   uint32_t sample, stage = S->numStages;         /*  loop counters             */
    unsigned short gainFlg = 0;

   float32_t Xn2, Xn3, Xn4;                       /*  Input State variables     */
   float32_t acc2, acc3, acc4;                        /*  accumulator               */


   float32_t p0, p1, p2, p3, p4, A1;

   /* Run the below code for Cortex-M4 and Cortex-M3 */
   do
   {
      /* Reading the coefficients */
      b0 = *pCoeffs++;
      b1 = *pCoeffs++;
      b2 = *pCoeffs++;
      a1 = *pCoeffs++;
      a2 = *pCoeffs++;


      /*Reading the state values */
      d1 = pState[0];
      d2 = pState[1];

      /* Apply loop unrolling and compute 4 output values simultaneously. */
      sample = blockSize >> 2u;

      /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
      while(sample > 0u) {

         /* y[n] = b0 * x[n] + d1 */
         /* d1 = b1 * x[n] + a1 * y[n] + d2 */
         /* d2 = b2 * x[n] + a2 * y[n] */

         /* Read the four inputs */
         Xn1 = pIn[0];
         Xn2 = pIn[1];
         Xn3 = pIn[2];
         Xn4 = pIn[3];
         pIn += 4;

         p0 = b0 * Xn1;
         p1 = b1 * Xn1;
         acc1 = p0 + d1;
         p0 = b0 * Xn2;
         p3 = a1 * acc1;
         p2 = b2 * Xn1;
         A1 = p1 + p3;
         p4 = a2 * acc1;
         d1 = A1 + d2;
         d2 = p2 + p4;

         p1 = b1 * Xn2;
         acc2 = p0 + d1;
         p0 = b0 * Xn3;
         p3 = a1 * acc2;
         p2 = b2 * Xn2;
         A1 = p1 + p3;
         p4 = a2 * acc2;
         d1 = A1 + d2;
         d2 = p2 + p4;

         p1 = b1 * Xn3;
         acc3 = p0 + d1;
         p0 = b0 * Xn4;
         p3 = a1 * acc3;
         p2 = b2 * Xn3;
         A1 = p1 + p3;
         p4 = a2 * acc3;
         d1 = A1 + d2;
         d2 = p2 + p4;

         acc4 = p0 + d1;
         p1 = b1 * Xn4;
         p3 = a1 * acc4;
         p2 = b2 * Xn4;
         A1 = p1 + p3;
         p4 = a2 * acc4;
         d1 = A1 + d2;
         d2 = p2 + p4;

         pOut[0] = acc1*gainVec[gainFlg];
         pOut[1] = acc2*gainVec[gainFlg];
         pOut[2] = acc3*gainVec[gainFlg];
         pOut[3] = acc4*gainVec[gainFlg];
         pOut += 4;

         sample--;
      }

      sample = blockSize & 0x3u;
      while(sample > 0u) {
         Xn1 = *pIn++;

         p0 = b0 * Xn1;
         p1 = b1 * Xn1;
         acc1 = p0 + d1;
         p3 = a1 * acc1;
         p2 = b2 * Xn1;
         A1 = p1 + p3;
         p4 = a2 * acc1;
         d1 = A1 + d2;
         d2 = p2 + p4;

         *pOut++ = acc1*gainVec[gainFlg];

         sample--;
      }

      /* Store the updated state variables back into the state array */
      *pState++ = d1;
      *pState++ = d2;

      /* The current stage input is given as the output to the next stage */
      pIn = pDst;

      /*Reset the output working pointer */
      pOut = pDst;

      /* decrement the loop counter */
      stage--;
      gainFlg++;
   } while(stage > 0u);
}
LOW_OPTIMIZATION_EXIT

LOW_OPTIMIZATION_ENTER
void arm_biquad_cascade_df2T_f32Mod2(
const arm_biquad_cascade_df2T_instance_f32 * S,
float32_t * pSrc,
float32_t * pDst,
uint32_t blockSize)
{

   float32_t *pIn = pSrc;                         /*  source pointer            */
   float32_t *pOut = pDst;                        /*  destination pointer       */
   float32_t *pState = S->pState;                 /*  State pointer             */
   float32_t *pCoeffs = (float32_t*)S->pCoeffs;               /*  coefficient pointer       */
   float32_t acc1;                                /*  accumulator               */
   float32_t b0, b1, b2, a1, a2;                  /*  Filter coefficients       */
   float32_t Xn1;                                 /*  temporary input           */
   float32_t d1, d2;                              /*  state variables           */
   uint32_t sample, stage = S->numStages;         /*  loop counters             */

#if defined(ARM_MATH_CM7)

   float32_t Xn2, Xn3, Xn4, Xn5, Xn6, Xn7, Xn8;   /*  Input State variables     */
   float32_t Xn9, Xn10, Xn11, Xn12, Xn13, Xn14, Xn15, Xn16;
   float32_t acc2, acc3, acc4, acc5, acc6, acc7;  /*  Simulates the accumulator */
   float32_t acc8, acc9, acc10, acc11, acc12, acc13, acc14, acc15, acc16;

   do
   {
      /* Reading the coefficients */
      b0 = pCoeffs[0];
      b1 = pCoeffs[1];
      b2 = pCoeffs[2];
      a1 = pCoeffs[3];
      /* Apply loop unrolling and compute 16 output values simultaneously. */
      sample = blockSize >> 4u;
      a2 = pCoeffs[4];

      /*Reading the state values */
      d1 = pState[0];
      d2 = pState[1];

      pCoeffs += 5u;


      /* First part of the processing with loop unrolling.  Compute 16 outputs at a time.
       ** a second loop below computes the remaining 1 to 15 samples. */
      while(sample > 0u) {

         /* y[n] = b0 * x[n] + d1 */
         /* d1 = b1 * x[n] + a1 * y[n] + d2 */
         /* d2 = b2 * x[n] + a2 * y[n] */

         /* Read the first 2 inputs. 2 cycles */
         Xn1  = pIn[0 ];
         Xn2  = pIn[1 ];

         /* Sample 1. 5 cycles */
         Xn3  = pIn[2 ];
         acc1 = b0 * Xn1 + d1;

         Xn4  = pIn[3 ];
         d1 = b1 * Xn1 + d2;

         Xn5  = pIn[4 ];
         d2 = b2 * Xn1;

         Xn6  = pIn[5 ];
         d1 += a1 * acc1;

         Xn7  = pIn[6 ];
         d2 += a2 * acc1;

         /* Sample 2. 5 cycles */
         Xn8  = pIn[7 ];
         acc2 = b0 * Xn2 + d1;

         Xn9  = pIn[8 ];
         d1 = b1 * Xn2 + d2;

         Xn10 = pIn[9 ];
         d2 = b2 * Xn2;

         Xn11 = pIn[10];
         d1 += a1 * acc2;

         Xn12 = pIn[11];
         d2 += a2 * acc2;

         /* Sample 3. 5 cycles */
         Xn13 = pIn[12];
         acc3 = b0 * Xn3 + d1;

         Xn14 = pIn[13];
         d1 = b1 * Xn3 + d2;

         Xn15 = pIn[14];
         d2 = b2 * Xn3;

         Xn16 = pIn[15];
         d1 += a1 * acc3;

         pIn += 16;
         d2 += a2 * acc3;

         /* Sample 4. 5 cycles */
         acc4 = b0 * Xn4 + d1;
         d1 = b1 * Xn4 + d2;
         d2 = b2 * Xn4;
         d1 += a1 * acc4;
         d2 += a2 * acc4;

         /* Sample 5. 5 cycles */
         acc5 = b0 * Xn5 + d1;
         d1 = b1 * Xn5 + d2;
         d2 = b2 * Xn5;
         d1 += a1 * acc5;
         d2 += a2 * acc5;

         /* Sample 6. 5 cycles */
         acc6 = b0 * Xn6 + d1;
         d1 = b1 * Xn6 + d2;
         d2 = b2 * Xn6;
         d1 += a1 * acc6;
         d2 += a2 * acc6;

         /* Sample 7. 5 cycles */
         acc7 = b0 * Xn7 + d1;
         d1 = b1 * Xn7 + d2;
         d2 = b2 * Xn7;
         d1 += a1 * acc7;
         d2 += a2 * acc7;

         /* Sample 8. 5 cycles */
         acc8 = b0 * Xn8 + d1;
         d1 = b1 * Xn8 + d2;
         d2 = b2 * Xn8;
         d1 += a1 * acc8;
         d2 += a2 * acc8;

         /* Sample 9. 5 cycles */
         acc9 = b0 * Xn9 + d1;
         d1 = b1 * Xn9 + d2;
         d2 = b2 * Xn9;
         d1 += a1 * acc9;
         d2 += a2 * acc9;

         /* Sample 10. 5 cycles */
         acc10 = b0 * Xn10 + d1;
         d1 = b1 * Xn10 + d2;
         d2 = b2 * Xn10;
         d1 += a1 * acc10;
         d2 += a2 * acc10;

         /* Sample 11. 5 cycles */
         acc11 = b0 * Xn11 + d1;
         d1 = b1 * Xn11 + d2;
         d2 = b2 * Xn11;
         d1 += a1 * acc11;
         d2 += a2 * acc11;

         /* Sample 12. 5 cycles */
         acc12 = b0 * Xn12 + d1;
         d1 = b1 * Xn12 + d2;
         d2 = b2 * Xn12;
         d1 += a1 * acc12;
         d2 += a2 * acc12;

         /* Sample 13. 5 cycles */
         acc13 = b0 * Xn13 + d1;
         d1 = b1 * Xn13 + d2;
         d2 = b2 * Xn13;

         pOut[0 ] = acc1 ;
         d1 += a1 * acc13;

         pOut[1 ] = acc2 ;
         d2 += a2 * acc13;

         /* Sample 14. 5 cycles */
         pOut[2 ] = acc3 ;
         acc14 = b0 * Xn14 + d1;

         pOut[3 ] = acc4 ;
         d1 = b1 * Xn14 + d2;

         pOut[4 ] = acc5 ;
         d2 = b2 * Xn14;

         pOut[5 ] = acc6 ;
         d1 += a1 * acc14;

         pOut[6 ] = acc7 ;
         d2 += a2 * acc14;

         /* Sample 15. 5 cycles */
         pOut[7 ] = acc8 ;
         pOut[8 ] = acc9 ;
         acc15 = b0 * Xn15 + d1;

         pOut[9 ] = acc10;
         d1 = b1 * Xn15 + d2;

         pOut[10] = acc11;
         d2 = b2 * Xn15;

         pOut[11] = acc12;
         d1 += a1 * acc15;

         pOut[12] = acc13;
         d2 += a2 * acc15;

         /* Sample 16. 5 cycles */
         pOut[13] = acc14;
         acc16 = b0 * Xn16 + d1;

         pOut[14] = acc15;
         d1 = b1 * Xn16 + d2;

         pOut[15] = acc16;
         d2 = b2 * Xn16;

         sample--;
         d1 += a1 * acc16;

         pOut += 16;
         d2 += a2 * acc16;
      }

      sample = blockSize & 0xFu;
      while(sample > 0u) {
         Xn1 = *pIn;
         acc1 = b0 * Xn1 + d1;

         pIn++;
         d1 = b1 * Xn1 + d2;

         *pOut = acc1;
         d2 = b2 * Xn1;

         pOut++;
         d1 += a1 * acc1;

         sample--;
         d2 += a2 * acc1;
      }

      /* Store the updated state variables back into the state array */
      pState[0] = d1;
      /* The current stage input is given as the output to the next stage */
      pIn = pDst;

      pState[1] = d2;
      /* decrement the loop counter */
      stage--;

      pState += 2u;

      /*Reset the output working pointer */
      pOut = pDst;

   } while(stage > 0u);

#elif defined(ARM_MATH_CM0_FAMILY)

   /* Run the below code for Cortex-M0 */

   do
   {
      /* Reading the coefficients */
      b0 = *pCoeffs++;
      b1 = *pCoeffs++;
      b2 = *pCoeffs++;
      a1 = *pCoeffs++;
      a2 = *pCoeffs++;

      /*Reading the state values */
      d1 = pState[0];
      d2 = pState[1];


      sample = blockSize;

      while(sample > 0u)
      {
         /* Read the input */
         Xn1 = *pIn++;

         /* y[n] = b0 * x[n] + d1 */
         acc1 = (b0 * Xn1) + d1;

         /* Store the result in the accumulator in the destination buffer. */
         *pOut++ = acc1;

         /* Every time after the output is computed state should be updated. */
         /* d1 = b1 * x[n] + a1 * y[n] + d2 */
         d1 = ((b1 * Xn1) + (a1 * acc1)) + d2;

         /* d2 = b2 * x[n] + a2 * y[n] */
         d2 = (b2 * Xn1) + (a2 * acc1);

         /* decrement the loop counter */
         sample--;
      }

      /* Store the updated state variables back into the state array */
      *pState++ = d1;
      *pState++ = d2;

      /* The current stage input is given as the output to the next stage */
      pIn = pDst;

      /*Reset the output working pointer */
      pOut = pDst;

      /* decrement the loop counter */
      stage--;

   } while(stage > 0u);

#else

   float32_t Xn2, Xn3, Xn4;                  	  /*  Input State variables     */
   float32_t acc2, acc3, acc4;              		  /*  accumulator               */


   float32_t p0, p1, p2, p3, p4, A1;

   /* Run the below code for Cortex-M4 and Cortex-M3 */
   do
   {
      /* Reading the coefficients */
      b0 = *pCoeffs++;
      b1 = *pCoeffs++;
      b2 = *pCoeffs++;
      a1 = *pCoeffs++;
      a2 = *pCoeffs++;


      /*Reading the state values */
      d1 = pState[0];
      d2 = pState[1];

      /* Apply loop unrolling and compute 4 output values simultaneously. */
      sample = blockSize >> 2u;

      /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
      while(sample > 0u) {

         /* y[n] = b0 * x[n] + d1 */
         /* d1 = b1 * x[n] + a1 * y[n] + d2 */
         /* d2 = b2 * x[n] + a2 * y[n] */

         /* Read the four inputs */
         Xn1 = pIn[0];
         Xn2 = pIn[1];
         Xn3 = pIn[2];
         Xn4 = pIn[3];
         pIn += 4;

         p0 = b0 * Xn1;
         p1 = b1 * Xn1;
         acc1 = p0 + d1;
         p0 = b0 * Xn2;
         p3 = a1 * acc1;
         p2 = b2 * Xn1;
         A1 = p1 + p3;
         p4 = a2 * acc1;
         d1 = A1 + d2;
         d2 = p2 + p4;

         p1 = b1 * Xn2;
         acc2 = p0 + d1;
         p0 = b0 * Xn3;
         p3 = a1 * acc2;
         p2 = b2 * Xn2;
         A1 = p1 + p3;
         p4 = a2 * acc2;
         d1 = A1 + d2;
         d2 = p2 + p4;

         p1 = b1 * Xn3;
         acc3 = p0 + d1;
         p0 = b0 * Xn4;
         p3 = a1 * acc3;
         p2 = b2 * Xn3;
         A1 = p1 + p3;
         p4 = a2 * acc3;
         d1 = A1 + d2;
         d2 = p2 + p4;

         acc4 = p0 + d1;
         p1 = b1 * Xn4;
         p3 = a1 * acc4;
         p2 = b2 * Xn4;
         A1 = p1 + p3;
         p4 = a2 * acc4;
         d1 = A1 + d2;
         d2 = p2 + p4;

         pOut[0] = acc1;
         pOut[1] = acc2;
         pOut[2] = acc3;
         pOut[3] = acc4;
		 pOut += 4;

         sample--;
      }

      sample = blockSize & 0x3u;
      while(sample > 0u) {
         Xn1 = *pIn++;

         p0 = b0 * Xn1;
         p1 = b1 * Xn1;
         acc1 = p0 + d1;
         p3 = a1 * acc1;
         p2 = b2 * Xn1;
         A1 = p1 + p3;
         p4 = a2 * acc1;
         d1 = A1 + d2;
         d2 = p2 + p4;

         *pOut++ = acc1;

         sample--;
      }

      /* Store the updated state variables back into the state array */
      *pState++ = d1;
      *pState++ = d2;

      /* The current stage input is given as the output to the next stage */
      pIn = pDst;

      /*Reset the output working pointer */
      pOut = pDst;

      /* decrement the loop counter */
      stage--;

   } while(stage > 0u);

#endif

}
LOW_OPTIMIZATION_EXIT
