import math


class PIDController:

    def __init__(
        self,
        Kp,
        Ki,
        Kd,
        setpoint=0.0,
        output_limits=(None, None),
        integral_limits=(None, None),
        derivative_on_measurement=True,
        derivative_cutoff_hz=5.0,
    ):
        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Kd = float(Kd)

        self.setpoint = float(setpoint)

        self.derivative_on_measurement = (
            derivative_on_measurement
        )

        self.derivative_cutoff_hz = (
            derivative_cutoff_hz
        )

        # Internal state
        self._prev_error = 0.0
        self._prev_measurement = 0.0
        self._integral = 0.0

        self._filtered_derivative = 0.0
        self._initialized = False

        # Output and integral clamping
        self._output_min, self._output_max = (
            output_limits
        )

        self._integral_min, self._integral_max = (
            integral_limits
        )

        # Values available for logging
        self.error = 0.0

        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        self.raw_derivative = 0.0
        self.filtered_derivative = 0.0

        self.output = 0.0

    @staticmethod
    def _clamp(
        value,
        minimum,
        maximum,
    ):
        if minimum is not None:
            value = max(minimum, value)

        if maximum is not None:
            value = min(maximum, value)

        return value

    @staticmethod
    def _calculate_filter_alpha(
        cutoff_hz,
        dt,
    ):
        """
        First-order low-pass filter coefficient.

        filtered = filtered_previous
                   + alpha * (raw - filtered_previous)
        """

        if cutoff_hz is None or cutoff_hz <= 0.0:
            return 1.0

        tau = 1.0 / (
            2.0 * math.pi * cutoff_hz
        )

        return dt / (tau + dt)

    def reset(
        self,
        measurement=None,
    ):
        self._prev_error = 0.0
        self._prev_measurement = 0.0
        self._integral = 0.0

        self._filtered_derivative = 0.0
        self._initialized = False

        if measurement is not None:
            measurement = float(measurement)

            self._prev_measurement = measurement
            self._prev_error = (
                self.setpoint - measurement
            )

            self._initialized = True

        self.error = 0.0

        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        self.raw_derivative = 0.0
        self.filtered_derivative = 0.0

        self.output = 0.0

    def update(
        self,
        measurement,
        dt,
    ):
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(
                "dt must be finite, positive, and non-zero"
            )

        measurement = float(measurement)

        if not math.isfinite(measurement):
            raise ValueError(
                "measurement must be finite"
            )

        if not math.isfinite(self.setpoint):
            raise ValueError(
                "setpoint must be finite"
            )

        # -------------------------------------------------
        # Error
        # -------------------------------------------------

        error = (
            self.setpoint - measurement
        )

        self.error = error

        # -------------------------------------------------
        # Proportional term
        # -------------------------------------------------

        self.P = (
            self.Kp * error
        )

        # -------------------------------------------------
        # Integral term
        # -------------------------------------------------

        self._integral += (
            error * dt
        )

        self._integral = self._clamp(
            self._integral,
            self._integral_min,
            self._integral_max,
        )

        self.I = (
            self.Ki * self._integral
        )

        # -------------------------------------------------
        # Initialize derivative state
        # -------------------------------------------------

        if not self._initialized:

            self._prev_error = error
            self._prev_measurement = measurement

            self._filtered_derivative = 0.0

            self._initialized = True

        # -------------------------------------------------
        # Raw derivative
        # -------------------------------------------------

        if self.derivative_on_measurement:

            raw_derivative = -(
                measurement
                - self._prev_measurement
            ) / dt

        else:

            raw_derivative = (
                error
                - self._prev_error
            ) / dt

        self.raw_derivative = (
            raw_derivative
        )

        # -------------------------------------------------
        # Low-pass filter derivative
        # -------------------------------------------------

        alpha = self._calculate_filter_alpha(
            self.derivative_cutoff_hz,
            dt,
        )

        self._filtered_derivative += (
            alpha
            * (
                raw_derivative
                - self._filtered_derivative
            )
        )

        self.filtered_derivative = (
            self._filtered_derivative
        )

        self.D = (
            self.Kd
            * self.filtered_derivative
        )

        # Save states
        self._prev_error = error
        self._prev_measurement = measurement

        # -------------------------------------------------
        # Final output
        # -------------------------------------------------

        output = (
            self.P
            + self.I
            + self.D
        )

        self.output = self._clamp(
            output,
            self._output_min,
            self._output_max,
        )

        return self.output