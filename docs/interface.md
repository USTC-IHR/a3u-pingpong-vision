# 感知到控制接口 v0.1

- 状态：草案，未与控制侧联调。

```text
BallState:
  position_xyz     # 米，球台坐标系
  velocity_xyz     # 米/秒，球台坐标系
  covariance       # 6x6
  phase            # held | tossed | free_flight | after_bounce | lost
  valid            # bool
  capture_timestamp
  state_timestamp
  measurement_age
OpponentState:
  position_xy
  velocity_xy
  reference_point  # feet | hip | torso
  confidence
  valid
```

## 约定

- 时间戳统一用单调时钟，单位纳秒。
- 超出允许陈旧阈值时，`valid=false`，下游必须停止击球而不是沿用旧值。
- 未观测到的自旋不得填零并标为实测。
