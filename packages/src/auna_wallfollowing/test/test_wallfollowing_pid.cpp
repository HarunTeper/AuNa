// Copyright 2025 Harun Teper
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Validation for the dt-scaled PID terms of the wall-following controller
// (see issue #87). The controller node itself owns ROS interfaces, so these
// tests reproduce the exact PID update law in isolation and assert the
// property that motivated the change: for fixed gains, the integral and
// derivative contributions after a fixed amount of *simulated time* must not
// depend on the rate at which the controller is stepped.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{

// Mirrors WallFollow::pid_control's accumulation of the rate-dependent terms.
struct PidState
{
  double prev_error = 0.0;
  double integral = 0.0;
  double max_integral = 1e9;  // effectively disabled unless set by the test

  // Returns the derivative term for this step and updates the state.
  double step(double error, double dt)
  {
    double derivative = 0.0;

    if (dt > 0.0) {
      derivative = (error - prev_error) / dt;
      integral += error * dt;

      if (integral > max_integral) {
        integral = max_integral;
      }
      if (integral < -max_integral) {
        integral = -max_integral;
      }
    }

    prev_error = error;
    return derivative;
  }
};

// The pre-fix behaviour, kept only so the regression test can show that the
// two rates genuinely disagreed before the change.
struct LegacyPidState
{
  double prev_error = 0.0;
  double integral = 0.0;

  double step(double error, double /*dt*/)
  {
    double derivative = error - prev_error;
    integral += error;
    prev_error = error;
    return derivative;
  }
};

// Integrates a constant error over `duration` seconds at a given rate.
template<typename State>
double integral_after(State * state, double error, double duration, double rate)
{
  const double dt = 1.0 / rate;
  const int steps = static_cast<int>(std::lround(duration * rate));

  for (int i = 0; i < steps; ++i) {
    state->step(error, dt);
  }

  return state->integral;
}

}  // namespace

// The integral term must approximate error * elapsed_time regardless of the
// callback rate: 10 Hz and 40 Hz must agree after the same simulated duration.
TEST(WallFollowPid, IntegralIsRateInvariant)
{
  const double error = 0.2;
  const double duration = 2.0;

  PidState slow;
  PidState fast;

  const double slow_integral = integral_after(&slow, error, duration, 10.0);
  const double fast_integral = integral_after(&fast, error, duration, 40.0);

  EXPECT_NEAR(slow_integral, error * duration, 1e-9);
  EXPECT_NEAR(fast_integral, error * duration, 1e-9);
  EXPECT_NEAR(slow_integral, fast_integral, 1e-9);
}

// The derivative term must approximate the true time derivative of the error,
// which is rate-independent for a fixed error ramp.
TEST(WallFollowPid, DerivativeIsRateInvariant)
{
  // Error ramps at a constant 0.5 units per second.
  const double slope = 0.5;

  auto derivative_at_rate = [slope](double rate) {
      const double dt = 1.0 / rate;
      PidState state;
      double derivative = 0.0;

      // Step through 1 second of ramp; the steady-state derivative is `slope`.
      const int steps = static_cast<int>(std::lround(rate));
      for (int i = 0; i < steps; ++i) {
        derivative = state.step(slope * (i + 1) * dt, dt);
      }

      return derivative;
    };

  EXPECT_NEAR(derivative_at_rate(10.0), slope, 1e-9);
  EXPECT_NEAR(derivative_at_rate(40.0), slope, 1e-9);
}

// Regression guard: the pre-fix update law was rate-dependent. This documents
// the magnitude of the discrepancy the fix removes.
TEST(WallFollowPid, LegacyBehaviourWasRateDependent)
{
  const double error = 0.2;
  const double duration = 2.0;

  LegacyPidState slow;
  LegacyPidState fast;

  const double slow_integral = integral_after(&slow, error, duration, 10.0);
  const double fast_integral = integral_after(&fast, error, duration, 40.0);

  // Four times the callback rate accumulated four times the integral.
  EXPECT_NEAR(fast_integral, 4.0 * slow_integral, 1e-9);
}

// A non-positive dt (first callback, or an implausible measured period) must
// leave the integral untouched and produce no derivative contribution.
TEST(WallFollowPid, NonPositiveDtHoldsRateDependentTerms)
{
  PidState state;

  state.step(0.3, 0.1);
  const double integral_before = state.integral;

  const double derivative = state.step(0.9, -1.0);

  EXPECT_DOUBLE_EQ(derivative, 0.0);
  EXPECT_DOUBLE_EQ(state.integral, integral_before);
}

// The anti-windup clamp must bound the integral in both directions.
TEST(WallFollowPid, IntegralIsClampedByAntiWindup)
{
  PidState positive;
  positive.max_integral = 0.5;
  integral_after(&positive, 1.0, 10.0, 10.0);
  EXPECT_DOUBLE_EQ(positive.integral, 0.5);

  PidState negative;
  negative.max_integral = 0.5;
  integral_after(&negative, -1.0, 10.0, 10.0);
  EXPECT_DOUBLE_EQ(negative.integral, -0.5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
