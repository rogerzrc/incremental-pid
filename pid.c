#include "pid.h"

inline float fabs_limit(const float out, const float limit, const float epsilon) {
  if ((out - limit > epsilon) || (out - limit < -epsilon))
    return limit;
  else
    return out;
}

void pid_init(struct pid *c, float kp, float t, float ti, float td, float limit) {
  c->kp = kp;
  if (ti != 0.0f) 
    c->ki = kp * (t / ti);
  else
    c->ki = 0;
  c->kd = kp * (td / ti);
  c->set = 0.0f;
  c->get = 0.0f;
  c->limit = 0.0f;
  c->errors[0] = 0.0f;
  c->errors[1] = 0.0f;
  c->errors[2] = 0.0f;
  c->pout = 0.0f;
  c->iout = 0.0f;
  c->dout = 0.0f;
  c->out = 0.0f;  
}

void pid_calc(struct pid *c, float get, float set) {
  c->get = get;
  c->set = set;

  c->errors[0] = c->set - c->get;

  c->pout = c->kp * (c->errors[0] - c->errors[1]);
  c->iout = c->ki * c->errors[0];
  c->dout = c->kd * (c->errors[0] - 2 * c->errors[1] + c->errors[2]);
  c->out += (c->pout + c->iout + c->dout);
  c->out = fabs_limit(c->out, c->limit, 0.001f);

  c->errors[2] = c->errors[1];
  c->errors[1] = c->errors[0];
}

void pid_deinit(struct pid *c) {
  // TODO
}