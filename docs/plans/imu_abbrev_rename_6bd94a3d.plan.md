---
name: IMU abbrev rename
overview: Rename ImuParameters noise fields to the inspect-aligned short forms `acc_nd` / `gyr_nd` / `acc_rw` / `gyr_rw`, and update all accessors, error field paths, loaders, and tests.
todos:
  - id: rename-core
    content: Rename ImuParameters API + field_path strings in hpp/cpp
    status: pending
  - id: update-callers
    content: Update inspect, euroc/tum_vi imuSourceField, and all tests
    status: pending
  - id: verify-tests
    content: Rebuild and run affected sensor/dataset gtests
    status: pending
isProject: false
---

# Rename ImuParameters to acc/gyr + nd/rw

## Naming contract

Align [phad/sensor/imu_parameters.hpp](phad/sensor/imu_parameters.hpp) with existing inspect labels (`imu_acc_nd`, …):

| Role | Old | New |
|------|-----|-----|
| accel noise density | `accelerometer_noise_density_mps2_per_sqrt_hz` / `accelerometerNoiseDensityMps2PerSqrtHz()` | `acc_nd` / `accNd()` |
| gyro noise density | `gyroscope_noise_density_radps_per_sqrt_hz` / `gyroscopeNoiseDensityRadpsPerSqrtHz()` | `gyr_nd` / `gyrNd()` |
| accel bias RW | `accelerometer_bias_random_walk_mps3_per_sqrt_hz` / `accelerometerBiasRandomWalkMps3PerSqrtHz()` | `acc_rw` / `accRw()` |
| gyro bias RW | `gyroscope_bias_random_walk_radps2_per_sqrt_hz` / `gyroscopeBiasRandomWalkRadps2PerSqrtHz()` | `gyr_rw` / `gyrRw()` |

- Members: `m_acc_nd`, `m_gyr_nd`, `m_acc_rw`, `m_gyr_rw`
- Error `field_path`: `imu.acc_nd`, `imu.gyr_nd`, `imu.acc_rw`, `imu.gyr_rw`
- `rateHz` / `m_rate_hz` unchanged
- Units live in brief header comments only (m/s²/√Hz, rad/s/√Hz, m/s³/√Hz, rad/s²/√Hz)
- EuRoC/TUM-VI **YAML keys stay full names** (`accelerometer_noise_density`, …); only the internal API and calibration error paths change
- [ImuMeasurement](phad/sensor/imu_measurement.hpp) `accel_mps2` / `gyro_radps` unchanged (measurement vs noise-param vocabulary)

Target API sketch:

```cpp
[[nodiscard]] static CalibrationResult<ImuParameters> create(
    double rate_hz, double acc_nd, double gyr_nd,
    double acc_rw, double gyr_rw);

[[nodiscard]] double accNd() const noexcept;
[[nodiscard]] double gyrNd() const noexcept;
[[nodiscard]] double accRw() const noexcept;
[[nodiscard]] double gyrRw() const noexcept;
```

## Files to update

1. **Core**: [imu_parameters.hpp](phad/sensor/imu_parameters.hpp), [imu_parameters.cpp](phad/sensor/imu_parameters.cpp) — rename params, members, accessors, and `field_path` strings in validation.
2. **Call sites**: [apps/phad_euroc_inspect.cpp](apps/phad_euroc_inspect.cpp) — switch method calls; keep printed keys `imu_acc_nd` / `imu_gyr_nd` / `imu_acc_rw` / `imu_gyr_rw`.
3. **Error remapping** in [euroc_dataset.cpp](phad/io/dataset/euroc/euroc_dataset.cpp) and [tum_vi_dataset.cpp](phad/io/dataset/tum_vi/tum_vi_dataset.cpp) `imuSourceField`: match new paths (`acc_nd`, `gyr_nd`, `acc_rw`, `gyr_rw`) and still return the original YAML field names.
4. **Tests**: [sensor_parameters_test.cpp](tests/sensor/sensor_parameters_test.cpp), [euroc_dataset_test.cpp](tests/io/dataset/euroc/euroc_dataset_test.cpp), [tum_vi_dataset_test.cpp](tests/io/dataset/tum_vi/tum_vi_dataset_test.cpp) — update accessor calls and expected `field_path` strings (`imu.acc_nd`, …).

## Out of scope

- No docs/conventions wording change unless a field_path example is hardcoded (none found for these long names).
- No rename of dataset YAML keys or measurement fields.
- Phantom/unsaved [calibration.hpp](phad/sensor/calibration.hpp) is not on disk; ignore unless it reappears as a real file.

## Verify

Rebuild and run the affected gtests (`sensor_parameters_test`, euroc/tum_vi dataset tests).
