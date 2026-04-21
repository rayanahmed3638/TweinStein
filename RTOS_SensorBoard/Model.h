#ifndef MODEL_H
#define MODEL_H

#include "stdint.h"

#define CAP_IR 305 // mm
#define CAP_TFLUNA 1000 // mm
#define CAP_THROTTLE 9000 // pwm
#define CAP_STEERING 30 // degrees
#define CAP_ANGLE 64 // degrees
#define CAP_ANGLE_POW 6 // 2^6 = 64
#define CAP_DELTA_STEERING 10
#define CAP_DELTA_THROTTLE 2000

typedef int32_t fixed_t;
typedef enum { ir_right,
               ir_left,
               tf_left,
               tf_middle,
               tf_right,
               throttle_left_prev,
               throttle_right_prev,
               steering_prev,
               angle_left,
               angle_right,
               NUM_INPUTS } input_t;

typedef enum { throttle_left,
               throttle_right,
               steering,
               NUM_OUTPUTS} output_t;

extern fixed_t Model_Inputs[NUM_INPUTS];
extern fixed_t Model_Outputs[NUM_OUTPUTS];
extern const fixed_t Model_Weights[NUM_OUTPUTS][NUM_INPUTS];
extern const fixed_t Model_Bias[NUM_OUTPUTS];

void Model_Inference(void);

fixed_t Model_Normalize(int32_t input, int32_t cap);

fixed_t Model_NormalizeSigned(int32_t input, int32_t cap);

void Model_ApplyResidual(uint16_t* throttle_l, uint16_t* throttle_r, int32_t* steering_angle);

#endif