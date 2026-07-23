#
## filters.py
#
#  Reusable filter classes for target smoothing and signal filtering.
#


class EMAFilter:
    """Exponential Moving Average (EMA) filter."""

    def __init__(self, alpha: float = 0.05, initial_value: float = 0.0):
        self.alpha = float(alpha)
        self.value = float(initial_value)

    def update(self, measurement: float) -> float:
        self.value += self.alpha * (float(measurement) - self.value)
        return self.value

    def reset(self, initial_value: float = 0.0):
        self.value = float(initial_value)


class KalmanFilter1D:
    """1D Discrete Kalman Filter for scalar tracking (angle positioning)."""

    def __init__(
        self,
        process_noise: float = 0.05,
        measurement_noise: float = 2.0,
        initial_value: float = 0.0,
        initial_uncertainty: float = 1.0,
    ):
        self.x = float(initial_value)  # Estimated state
        self.P = float(initial_uncertainty)  # Estimate error covariance
        self.Q = float(process_noise)  # Process noise covariance
        self.R = float(measurement_noise)  # Measurement noise covariance

    def predict(self):
        """Prediction stage assuming zero velocity / static state model."""
        self.P += self.Q

    def update(self, measurement: float) -> float:
        """Correction stage given a new raw target measurement."""
        self.predict()

        # Compute Kalman Gain
        K = self.P / (self.P + self.R)

        # Update State & Covariance
        self.x += K * (float(measurement) - self.x)
        self.P *= 1.0 - K

        return self.x

    def reset(
        self, initial_value: float = 0.0, initial_uncertainty: float = 1.0
    ):
        self.x = float(initial_value)
        self.P = float(initial_uncertainty)


class PassThroughFilter:
    """Bypasses filtering, returning the raw measurement directly."""

    def __init__(self, initial_value: float = 0.0):
        self.value = float(initial_value)

    def update(self, measurement: float) -> float:
        self.value = float(measurement)
        return self.value

    def reset(self, initial_value: float = 0.0):
        self.value = float(initial_value)