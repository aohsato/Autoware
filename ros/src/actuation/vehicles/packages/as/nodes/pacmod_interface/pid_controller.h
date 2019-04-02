/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
public:
  PIDController()
  : prev_e_(0.0)
  , i_(0.0)
  , kp_(0.0)
  , ki_(0.0)
  , kd_(0.0)
  , min_(0.0)
  , max_(0.0)
  {
  }

  void setGain(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setMinMax(double min, double max)
  {
    min_ = min;
    max_ = max;
  }

  void reset()
  {
    i_ = 0.0;
  }

  double update(double e, double dt)
  {
    static double p, i, d;

    p = e;
    i = i_ + e * dt;
    d = (e - prev_e_) / dt;

    double x = kp_ * p + ki_ * i_ + kd_ * d;

    if (min_ < x && x < max_)
    {
      i_ = i;
    }

    x = std::max(x, min_);
    x = std::min(x, max_);

    prev_e_ = e;

    return x;
  }

private:
  double prev_e_, i_;
  double kp_, ki_, kd_;
  double min_, max_;
};

#endif  // PID_CONTROLLER_H
