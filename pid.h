#ifndef __PID_H__
#define __PID_H__

struct pid {
  float kp, ki, kd;
  float errors[3];
  float set, get, limit;
  float pout, iout, dout, out;
};

#ifdef __cplusplus
extern "C" {
#endif

void pid_init(struct pid *c, float kp, float t, float ti, float td, float limit);
void pid_calc(struct pid *c, float get, float set);
void pid_deinit(struct pid *c);

#ifdef __cplusplus
}
#endif

#endif //__PID_H__
