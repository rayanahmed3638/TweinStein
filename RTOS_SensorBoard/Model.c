// Model.c
// Contains functionality for inferencing from reinforcement learning model

#include "Model.h"

fixed_t Model_Inputs[NUM_INPUTS];
fixed_t Model_Outputs[NUM_OUTPUTS];
const fixed_t Model_Weights[NUM_OUTPUTS][NUM_INPUTS] = {{0}}; // training will set weights
const fixed_t Model_Bias[NUM_OUTPUTS] = {32768, 32768, 32768}; // training will set bias, untrained model will have no effect

static inline fixed_t fixed_mul(fixed_t x, fixed_t y){
  return ((int64_t)x*y) >> 16;
}

// Simple matrix multiply + addition with weights and bias
void Model_Inference(void){
  for (int r = 0; r < NUM_OUTPUTS; r++){
    Model_Outputs[r] = 0;
    for (int c = 0; c < NUM_INPUTS; c++){
      Model_Outputs[r] += fixed_mul(Model_Weights[r][c],Model_Inputs[c]);
    }
    Model_Outputs[r] += Model_Bias[r];
  }
}

fixed_t Model_Normalize(int32_t input, int32_t cap){
    fixed_t out = (input << 16)/cap;
    if (out > 65536) return 65536;
    return out;
}

fixed_t Model_NormalizeSigned(int32_t input, int32_t cap){
    if (input >  cap) input =  cap;
    if (input < -cap) input = -cap;
    return ((input + cap) << 16) / (2 * cap);
}


// Clamp outputs, helper function
static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi){
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}


// Use model output to tweak PD controller output - improved racing
void Model_ApplyResidual(uint16_t* throttle_l, uint16_t* throttle_r, int32_t* steering_angle){
  // De-normalize model outputs to apply to controller output
  int32_t delta_steer = (((int32_t)Model_Outputs[steering] - 32768) * CAP_DELTA_STEERING) >> 15;
  int32_t delta_thr_l = (((int32_t)Model_Outputs[throttle_left] - 32768) * CAP_DELTA_THROTTLE) >> 15;
  int32_t delta_thr_r = (((int32_t)Model_Outputs[throttle_right] - 32768) * CAP_DELTA_THROTTLE) >> 15;

  // Apply residuals
  int32_t final_steer = *steering_angle + delta_steer;
  int32_t final_thr_l = (int32_t)(*throttle_l) + delta_thr_l;
  int32_t final_thr_r = (int32_t)(*throttle_r) + delta_thr_r;

  // Store in variables used in CAN transmission
  *steering_angle = clamp_i32(final_steer, -CAP_STEERING, CAP_STEERING);
  *throttle_l = (uint16_t)clamp_i32(final_thr_l, 0, CAP_THROTTLE);
  *throttle_r = (uint16_t)clamp_i32(final_thr_r, 0, CAP_THROTTLE);
}