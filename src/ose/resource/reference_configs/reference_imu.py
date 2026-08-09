"""Reference configurations for the IMU resource. See package docstring."""

from __future__ import annotations

from ose.resource.imu import ImuParameters

TACTICAL_GRADE = ImuParameters(
    accel_noise_density=1.0e-3,      # [m/s^2 / sqrt(Hz)]
    accel_bias_sigma=1.0e-3,         # steady-state bias sigma [m/s^2]
    accel_bias_tau_s=3600.0,
    gyro_noise_density=3.0e-5,       # [rad/s / sqrt(Hz)]
    gyro_bias_sigma=5.0e-6,          # [rad/s]
    gyro_bias_tau_s=3600.0,
)
