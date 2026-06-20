import numpy as np


class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.base_kp = kp
        self.base_ki = ki
        self.base_kd = kd

        # Output limits (e.g. pedal range 0-10).
        self.out_min = out_min
        self.out_max = out_max

        self.integral = 0.0     # Running sum of past error, used by the I term.
        self.prev_error = 0.0   # Last step's error, used to compute the derivative.

    def _adaptive_tuning(self, error):
        # Gain scheduling: instead of one fixed (kp, ki, kd) for every
        # situation, shift the gains depending on how far off we are.
        abs_error = abs(error)

        if abs_error > 1.39:
            # Far from setpoint: push harder to close the gap quickly,
            # and ease off D so the strong P response isn't dampened too much.
            kp = self.base_kp * 1.5
            ki = self.base_ki * 0.8
            kd = self.base_kd * 0.5
        elif abs_error < 0.28:
            # Close to setpoint: back off P to avoid overshoot, lean on I
            # to kill any small remaining steady-state error, and raise D
            # to smooth out the final approach.
            kp = self.base_kp * 0.7
            ki = self.base_ki * 1.0
            kd = self.base_kd * 2.2
        else:
            # Mid-range error: just use the original tuning as-is.
            kp = self.base_kp
            ki = self.base_ki
            kd = self.base_kd

        return kp, ki, kd

    def update(self, error, dt, use_anti_windup, use_adaptive, use_p, use_i, use_d):
        if use_adaptive:
            kp, ki, kd = self._adaptive_tuning(error)
        else:
            kp, ki, kd = self.base_kp, self.base_ki, self.base_kd

        # --- Proportional term ---
        P = kp * error if use_p else 0.0

        # --- Derivative term ---
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        D = kd * derivative if use_d else 0.0

        # --- Integral term ---
        if use_i:
            potential_integral = self.integral + error * dt
            I_potential = ki * potential_integral
        else:
            potential_integral = self.integral
            I_potential = 0.0

        raw_output = P + I_potential + D

        # Anti-windup check:
        is_saturated = raw_output > self.out_max or raw_output < self.out_min
        same_sign = np.sign(raw_output) == np.sign(error)

        if use_i and use_anti_windup and (is_saturated and same_sign):
            # Output is saturated and the integral would only push it
            # further into saturation, so freeze the integral instead of
            # letting it accumulate.
            self.integral = self.integral
        else:
            # Safe to accumulate normally.
            self.integral = potential_integral

        I = ki * self.integral if use_i else 0.0

        output = P + I + D
        output = np.clip(output, self.out_min, self.out_max)

        self.prev_error = error
        return output

    def reset(self):
        # Clears accumulated state
        self.integral = 0.0
        self.prev_error = 0.0