# A3U 面部内置相机正向运动学（FK）与空间坐标解算规范 v0.1

- 状态：技术契约 / 测试规范
- 适用范围：A3U 机器人面部内置双目/深度相机空间位姿解算、遮挡条件下球台与机器人相对定位评测
- 关联资产：`/home/mine/a3u-pose-viewer/robot_model/urdf/agibot_a3.urdf`

---

## 1. 标定与数据流底线原则

1. **内参独立性**：严禁用动捕数据标定相机内参。相机内参（$f_x, f_y, c_x, c_y$ 与畸变系数 $k_1, k_2, p_1, p_2$）必须使用高精度棋盘格或 ChArUco 标定板在近景多角度离线标定，参数固化至 `configs/camera_a3u_face.yaml`。
2. **动捕定位**：动捕系统仅作为 6-DoF 刚体位姿真值（Ground Truth）提供者，用于：
   - 机器人骨盆位姿 $T_{\text{world}}^{\text{pelvis}}$ 与球台位姿 $T_{\text{world}}^{\text{table}}$ 实时真值采集；
   - 评测视觉解算算法在不同遮挡率下的绝对误差；
   - 手眼标定（标定动捕 Marker 刚体到骨盆原点或相机的静态刚体外参）。

---

## 2. 运动学链（Kinematic Chain）拓扑与关节参数

面部内置相机挂载在智元 A3U 头部标准双目相机节点。从动力学根节点 `pelvis_link`（骨盆）到相机光心 `std_head_stereo_camera_Link` 的上行串联运动学链如下：

```text
pelvis_link (骨盆动力学根)
   │
   └─ [waist_yaw_joint] (revolute, 偏航)
        └─ waist_yaw_Link
             │
             └─ [waist_roll_joint] (revolute, 侧倾)
                  └─ waist_roll_Link
                       │
                       └─ [waist_pitch_joint] (revolute, 俯仰)
                            └─ torso_Link (躯干主结构)
                                 │
                                 └─ [head_yaw_joint] (revolute, 转头偏航)
                                      └─ head_yaw_Link
                                           │
                                           └─ [head_pitch_joint] (revolute, 抬头/低头俯仰)
                                                └─ head_pitch_Link (头部末端)
                                                     │
                                                     └─ [std_head_stereo_camera_joint] (fixed, 固定外参)
                                                          └─ std_head_stereo_camera_Link (面部相机)
```

### 关节标称参数表（来源于 `agibot_a3.urdf`）

| 序号 | 关节名称 (`joint`) | 类型 | 父 Link $\to$ 子 Link | 静态偏移 `origin` ($xyz$, $rpy$) | 旋转轴 `axis` | 关节限位范围 (rad) |
| :---: | :--- | :---: | :--- | :--- | :---: | :---: |
| 1 | `waist_yaw_joint` | `revolute` | `pelvis_link` $\to$ `waist_yaw_Link` | $xyz = [0, 0, 0]$<br>$rpy = [0, 0, 0]$ | $[0, 0, 1]^T$ | $[-2.6180, +2.6180]$ |
| 2 | `waist_roll_joint` | `revolute` | `waist_yaw_Link` $\to$ `waist_roll_Link` | $xyz = [0, 0, 0]$<br>$rpy = [0, 0, 0]$ | $[1, 0, 0]^T$ | $[-0.3491, +0.3491]$ |
| 3 | `waist_pitch_joint` | `revolute` | `waist_roll_Link` $\to$ `torso_Link` | $xyz = [-0.02, 0, 0.005]$<br>$rpy = [0, 0, 0]$ | $[0, 1, 0]^T$ | $[-0.4887, +0.4189]$ |
| 4 | `head_yaw_joint` | `revolute` | `torso_Link` $\to$ `head_yaw_Link` | $xyz = [0.0303, 0, 0.28896]$<br>$rpy = [0, 0, 0]$ | $[0, 0, 1]^T$ | $[-1.0472, +1.0472]$ |
| 5 | `head_pitch_joint` | `revolute` | `head_yaw_Link` $\to$ `head_pitch_Link` | $xyz = [0, 0, 0.21904]$<br>$rpy = [0, 0, 0]$ | $[0, 1, 0]^T$ | $[-0.4363, +0.2618]$ |
| 6 | `std_head_stereo_camera_joint` | `fixed` | `head_pitch_Link` $\to$ `std_head_stereo_camera_Link` | $xyz = [0.08388, 0, -0.02087]$<br>$rpy = [0, 0.17423, 0]$ | 无 (固定外参) | 标称俯仰下倾角 $\approx 9.98^\circ$ |

- **零位几何特征**：
  当腰部与头部 5 个可动关节角全部为 0 时，相机相对骨盆原点的空间坐标为：
  $$p_{\text{pelvis}}^{\text{camera}} = [0.09418,\; 0,\; 0.49213]^T\,\mathrm{m}$$
  相机安装自带向前下倾约 $9.98^\circ$（$0.17423\,\mathrm{rad}$）的俯仰下看视场。

---

## 3. 正向运动学（FK）坐标变换推导

### 3.1 局部关节齐次变换算子
对于具有静态平移 $t_i$、静态旋转 $R_{\text{static}, i}$ 及转动角 $\theta_i$（绕单位轴 $a_i$）的关节 $i$：
$$T_i(\theta_i) = \begin{bmatrix} R_{\text{static}, i} & t_i \\ 0 & 1 \end{bmatrix} \begin{bmatrix} \operatorname{Rodrigues}(a_i, \theta_i) & 0 \\ 0 & 1 \end{bmatrix}$$

其中 Rodrigues 旋转公式为：
$$\operatorname{Rodrigues}(a_i, \theta_i) = I + \sin\theta_i [a_i]_\times + (1 - \cos\theta_i) [a_i]_\times^2$$

### 3.2 骨盆到相机的相对齐次变换 $T_{\text{pelvis}}^{\text{camera}}(\theta)$
$$T_{\text{pelvis}}^{\text{camera}}(\theta) = T_{\text{wy}}(\theta_{\text{wy}}) \cdot T_{\text{wr}}(\theta_{\text{wr}}) \cdot T_{\text{wp}}(\theta_{\text{wp}}) \cdot T_{\text{hy}}(\theta_{\text{hy}}) \cdot T_{\text{hp}}(\theta_{\text{hp}}) \cdot T_{\text{cam\_fixed}}$$

### 3.3 全局世界坐标系下的相机位姿
已知骨盆在世界坐标系（或动捕世界系）下的齐次变换矩阵 $T_{\text{world}}^{\text{pelvis}}$，相机在世界坐标系下的绝对位姿为：
$$T_{\text{world}}^{\text{camera}} = T_{\text{world}}^{\text{pelvis}} \cdot T_{\text{pelvis}}^{\text{camera}}(\theta)$$

相机的世界三维空间坐标 $p_{\text{world}}^{\text{camera}}$ 与旋转矩阵 $R_{\text{world}}^{\text{camera}}$ 分别由矩阵对应分量提取：
$$p_{\text{world}}^{\text{camera}} = \left(T_{\text{world}}^{\text{camera}}\right)_{[:3, 3]}, \quad R_{\text{world}}^{\text{camera}} = \left(T_{\text{world}}^{\text{camera}}\right)_{[:3, :3]}$$

---

## 4. 实机工程落地误差源与补偿清单

在编写测试代码及实机评测时，必须明确以下四项工程补偿：

1. **动捕刚体（Marker Rig）到 `pelvis_link` 原点的刚体外参**：
   - 动捕系统输出的是 Marker 刚体几何中心 $T_{\text{world}}^{\text{marker}}$。
   - 实机工程记录（`A3_PINGPONG_DEPLOY_20261025`）：刚体中心相对 `pelvis_link` 在 $Y$-up 下垂直方向低了约 $12.65\,\mathrm{cm}$，外参平移为 $[0.0018, 0.1265, 0.0002]\,\mathrm{m}$。
   - 测试代码计算时必须显式代入：
     $$T_{\text{world}}^{\text{pelvis}} = T_{\text{world}}^{\text{marker}} \cdot T_{\text{marker}}^{\text{pelvis}}$$
2. **URDF 相机标称 CAD 外参残差**：
   - URDF 中的固定外参为设计标称值。实机机械装配通常存在 $\pm 2\sim 5\,\mathrm{mm}$ 的平移误差与 $\pm 0.5^\circ\sim 1.5^\circ$ 的旋转误差。
   - 测试脚本应预留可配置的外参残差修正接口：$\Delta T_{\text{cam\_calib}}$。
3. **结构弹性形变与间隙**：
   - 运动学链总悬臂长约 $0.5\,\mathrm{m}$。机器人剧烈移动与击球挥拍时，谐波减速器存在微小弹性迟滞，若要求极高精度可融合头部 IMU 做姿态平滑互补。
4. **多源数据时间戳硬同步**：
   - 动捕频率（$120\sim 240\,\mathrm{Hz}$）、HAL 关节编码器（$200\sim 500\,\mathrm{Hz}$）与相机曝光（$30\sim 60\,\mathrm{Hz}$）必须基于硬件单调时钟进行线性插值对齐，防止动态运动下的时间延迟导致空间重投影撕裂。

---

## 5. 快速验证代码实现（用于测试用例）

可在测试代码中直接集成以下纯 Python / NumPy 闭式解算器：

```python
#!/usr/bin/env python3
"""
A3U Face Camera Forward Kinematics (FK) Resolver
Reference implementation for tests and live estimation.
"""

import math
import numpy as np


def rpy_to_matrix(rpy):
    """ROS extrinsic fixed-axis Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr               ]
    ], dtype=np.float64)


def make_transform(xyz, rpy):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def axis_angle_transform(axis, theta):
    """Rodrigues formula for revolute joint."""
    v = np.asarray(axis, dtype=np.float64)
    v /= np.linalg.norm(v)
    x, y, z = v
    k = np.array([
        [0,  -z,  y],
        [z,   0, -x],
        [-y,  x,  0]
    ], dtype=np.float64)
    R = np.eye(3) + math.sin(theta) * k + (1.0 - math.cos(theta)) * (k @ k)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    return T


def compute_camera_world_pose(T_world_pelvis, joint_positions):
    """
    计算 A3U 面部相机的世界坐标

    参数:
      T_world_pelvis: 4x4 np.ndarray, 骨盆在世界系下的齐次变换矩阵
      joint_positions: dict, 必须包含以下 5 个关节的角度(rad):
        - 'waist_yaw_joint'
        - 'waist_roll_joint'
        - 'waist_pitch_joint'
        - 'head_yaw_joint'
        - 'head_pitch_joint'

    返回:
      T_world_camera: 4x4 np.ndarray, 相机光心在世界系下的齐次变换矩阵
    """
    # 1. waist_yaw: origin [0, 0, 0], axis [0, 0, 1]
    T_p_wy = make_transform([0, 0, 0], [0, 0, 0]) @ axis_angle_transform(
        [0, 0, 1], joint_positions.get('waist_yaw_joint', 0.0)
    )

    # 2. waist_roll: origin [0, 0, 0], axis [1, 0, 0]
    T_wy_wr = make_transform([0, 0, 0], [0, 0, 0]) @ axis_angle_transform(
        [1, 0, 0], joint_positions.get('waist_roll_joint', 0.0)
    )

    # 3. waist_pitch: origin [-0.02, 0, 0.005], axis [0, 1, 0]
    T_wr_t = make_transform([-0.02, 0, 0.005], [0, 0, 0]) @ axis_angle_transform(
        [0, 1, 0], joint_positions.get('waist_pitch_joint', 0.0)
    )

    # 4. head_yaw: origin [0.0303, 0, 0.28896], axis [0, 0, 1]
    T_t_hy = make_transform([0.0303, 0, 0.28896], [0, 0, 0]) @ axis_angle_transform(
        [0, 0, 1], joint_positions.get('head_yaw_joint', 0.0)
    )

    # 5. head_pitch: origin [0, 0, 0.21904], axis [0, 1, 0]
    T_hy_hp = make_transform([0, 0, 0.21904], [0, 0, 0]) @ axis_angle_transform(
        [0, 1, 0], joint_positions.get('head_pitch_joint', 0.0)
    )

    # 6. std_head_stereo_camera_joint (fixed):
    #    xyz=[0.0838768, 0, -0.02086925], rpy=[0, 0.1742296, 0]
    T_hp_cam = make_transform([0.0838768, 0, -0.02086925], [0, 0.1742296, 0])

    # 复合: pelvis -> camera
    T_pelvis_camera = T_p_wy @ T_wy_wr @ T_wr_t @ T_t_hy @ T_hy_hp @ T_hp_cam

    # 全局: world -> camera
    return T_world_pelvis @ T_pelvis_camera
```
