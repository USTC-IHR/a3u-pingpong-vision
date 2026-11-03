# A3U 面部单相机球台定位：运行、标定与验收

## 1. 程序做什么

程序从两颗面部相机中选一路图像，不使用双目深度。通过已知大小的球台矩形，或固定在台面上的已知尺寸 ArUco，估计相机到球台的六自由度位姿，再用规范中的五关节 FK 换算骨盆在球台坐标系的位置。

提供两种测试方式：

- 四角点选：冻结一张图，按顺序标注完整台面的四角，解算一次。适合先验证是否能大致测距，不依赖球台检测模型。
- ArUco：在台面贴一张量过位置和大小的标记，逐帧自动定位。标记可见时不要求四个球台角都可见；标记被挡住就不输出新的位置，不沿用上一帧冒充测量。

浏览器左侧显示相机与投影轮廓，右侧显示机器人、球台及光学坐标轴。所有 Three.js 文件与机器人网格来自本地，不需要 CDN。机器人只获得五关节链的数据，其余关节按零位画成橙色参考姿态，不是全身测量。程序不发布 ROS 消息，不连接运动控制接口，也不切换机器人模式。

### 先确认模型，不要跳过

本次按 `a3u-face-camera-fk-spec.md` 实现。该规范及参考查看器使用的是 `std_head_stereo_camera_Link`。仓库已有资产中，与其对应的是 `assets/robot_model/a3_t3d0/urdf/model.urdf`，所以这是程序的默认 URDF，并不代表已经确认实机硬件版本。

| 参数 | 本次规范 / 默认模型 | Ultra T2.5 资产模型 |
| --- | --- | --- |
| 相机节点 | `std_head_stereo_camera_Link` | `ultra_head_stereo_camera_Link` |
| 头俯仰关节 origin z | 0.219040 m | 0.230631 m |
| 相机安装平移 | [0.083877, 0, -0.020869] m | [0.091983, 0, -0.030221] m |
| 相机安装 pitch | 0.174230 rad | 0.241209 rad |

如果现场确认 Ultra T2.5 才是实机模型，在以下运行命令末尾加：

```bash
--urdf assets/robot_model/a3_ultra_t2d5/urdf/model.urdf
```

程序从实际选中的 URDF 读取 FK，不把规范常数硬塞进另一种模型。其他硬件版本也可通过 `--urdf` 指定，但必须有这五个关节、以 `pelvis_link` 为根，且包含唯一的头部 stereo camera 节点。网格路径需为 URDF 相对路径。

## 2. 安装与离线演示

建议 Python 3.10 或更高版本。普通 PC 可以运行界面和离线演示；读取机器人本地 V4L2 设备时需在设备所在机器运行。ROS 模式需要使用能够发现机器人话题、且装有 `rclpy` 与 `sensor_msgs` 的 ROS 2 环境。

```bash
cd /home/mine/a3u-pingpong-vision
python3 -m venv --system-site-packages .venv-face-table
source .venv-face-table/bin/activate
python -m pip install -r requirements-face-table.txt
python -m face_table run --source demo
```

浏览器打开 `http://127.0.0.1:8092`。界面应显示 “合成演示，非实机”，左侧有合成球台与标记，右侧有机器人和球台。默认合成骨盆位置为 `[-2.2, 0.12, 0.45] m`，到台面矩形的水平最短距离为 `0.83 m`。由于标记图像光栅化，自动恢复值会有小偏差。

如果机器人网格尚未初始化，可执行 `git submodule update --init assets/robot_model`，或用 `--urdf /home/mine/a3u-pose-viewer/robot_model/urdf/agibot_a3.urdf` 读取现有查看器资产。演示参数只存在于运行内存和日志，绝不会写成实机内参。

全部 CLI 参数：

```bash
python -m face_table --help
python -m face_table run --help
python -m face_table calibrate --help
```

也可使用 `scripts/face_table_test.sh`，参数与上述命令相同；`FACE_TABLE_PYTHON` 可指定 Python 解释器路径。

## 3. 找到并选择一颗面部相机

本仓库尚无现场验证过的图像话题、设备号和镜头绑定。现已有 ADU 出厂参数快照；前向左目鱼眼请使用 [专用测试指南](front-left-fisheye-test.md)，不要把鱼眼参数填进下文针孔模板。以下示例设备和话题必须替换为现场值；`camera_id: face_left` 只是标签，不会自动找到左相机。

### 3.1 优先使用现有 ROS 图像流

在机器人相机驱动已经运行、网络和 ROS domain 已配置好的终端中执行。不要为了采集而启动整套运动控制栈。

```bash
# 例：发行版不是 humble 时换成现场使用的 ROS 2 发行版。
source /opt/ros/humble/setup.bash
cd /home/mine/a3u-pingpong-vision
source .venv-face-table/bin/activate
python -c 'import rclpy; from sensor_msgs.msg import Image, JointState'
python -m face_table discover --ros
```

发现命令只查询 ROS 图，不订阅相机、也不发布任何消息。输出列出 `Image`、`CompressedImage`、`CameraInfo` 和 `JointState` 类型的话题。选定一个面部镜头的原始图像话题，例如将现场结果填入：

```bash
export FACE_IMAGE_TOPIC='/replace_with_actual_face_left_image_topic'
ros2 topic info "$FACE_IMAGE_TOPIC" -v
```

如果没有输出图像话题，需要先由现有相机驱动/厂商接口提供图像。本程序不包含未经验证的智元私有相机 SDK 驱动，也不会自动重启原厂服务。若相机只在专用 SDK 中可见，需把那一路接成 ROS `Image` 或 V4L2 后再用本程序。

仅预览并采集标定图，假设该流实际为 `1280×720`：

```bash
python -m face_table run --source ros \
  --image-topic "$FACE_IMAGE_TOPIC" \
  --preview --size 1280 720 \
  --output logs/face-left-calibration
```

若话题类型为 `sensor_msgs/msg/CompressedImage`，加 `--compressed`。原始图像支持 `mono8/rgb8/bgr8/rgba8/bgra8`，会处理行填充；不把 `16UC1` 深度图当作面部彩色图。

分辨率必须与实际图像一致。原始流可用 `ros2 topic echo "$FACE_IMAGE_TOPIC" --once --field width` 和 `--field height` 检查；压缩流可查看对应 CameraInfo 或驱动配置。CameraInfo 仅用于核对来源和尺寸，本程序不会偷偷用它替代规范要求的离线标定文件。

### 3.2 相机直接暴露为 V4L2 时

```bash
python -m face_table discover
# 如已安装 v4l2-ctl，还可查看设备格式：
v4l2-ctl --device /dev/video2 --list-formats-ext

python -m face_table run --source v4l2 --device /dev/video2 \
  --preview --size 1280 720 \
  --output logs/face-left-calibration
```

`/dev/video2` 是示例，不是已验证的面部设备。优先使用 `/dev/v4l/by-id/...` 稳定路径，核对它确实对应所选镜头。若一个 video 设备输出左右拼接图，不能直接当成一颗相机：需驱动提供单镜头图像，或先明确裁剪再为裁剪后的图像单独标定。本程序不自动裁剪、镜像、旋转或缩放图像。

## 4. 离线标定所选镜头

1. 打开预览页面，选定这一颗镜头和最终测试分辨率。
2. 使用量过格宽的高精度棋盘格。默认参数是 `9×6` 个内角点，对应 `10×7` 个方格。
3. 相机保持正常工作；移动标定板，采集 20–30 张不同距离、倾角、画面位置的清晰图像，覆盖中心和四周。每次点击“保存当前原图”。不要用同一姿态连续存几十张凑数量。
4. 停止预览服务，运行离线标定。下面的 `0.025` 表示实际单格宽 25 mm，必须按实物修改。

```bash
python -m face_table calibrate \
  --images 'logs/face-left-calibration/frame-*.jpg' \
  --columns 9 --rows 6 --square-m 0.025 \
  --camera-id face_left \
  --output configs/camera_a3u_face.yaml
```

程序至少需要 12 张可检测棋盘格的图，默认拒绝整体 RMS 超过 1.5 px 的结果。输出包含内参、畸变、分辨率、各视图误差与视图清单。低重投影误差并不证明无系统偏差，仍需留出几张未参与标定的图核对。切换镜头、分辨率、裁剪或驱动矫正模式后应重新标定。

以上命令默认标定针孔 `plumb_bob` 模型。鱼眼需显式加 `--distortion-model fisheye`，配置也应标为 `fisheye` 并使用四个 OpenCV theta 多项式系数；不接受直接拼接厂商全部 k/p 字段。相机内参不使用动捕数据。

## 5. 选定镜头的外参和关节输入

### 5.1 CAD 安装节点不等于镜头光心

`configs/camera_a3u_face.yaml` 中：

- `T_mount_optical` 把 OpenCV 光学坐标转换到 CAD 相机安装节点坐标。默认安装轴为前/左/上，光学轴为右/下/前；这是待验证约定。
- `lens_translation_mount_m` 是所选镜头光心在安装节点坐标系中的额外平移，单位米。默认 `null` 表示未知；近似模式会使用 CAD 原点并显示警告。不能把双目总基线直接当成任一镜头偏移。
- `delta_mount_xyz_m`、`delta_mount_rpy_rad` 用于标定 CAD 安装残差。乘法位置是 `T_pelvis_mount @ delta @ T_mount_optical`，其中镜头平移加在 `T_mount_optical` 的平移分量中。
- `optical_extrinsic_verified` 默认 `false`。只有实际验证轴向、镜头外参后才设 `true`。确认过的零偏移可填 `[0, 0, 0]`；不要为消除界面提示而把未知值伪装成实测零。

未验证光心外参不影响“相机到台心”的 PnP 测距，但会影响骨盆距离。程序仍显示其近似骨盆位置，单独标为 `nominal_CAD` 或 `assumed_fixed`，不会标成经过验证的机器人位姿。

### 5.2 第一次测试：固定机器人腰部和头部

先让五关节保持静止。若实际都在已核对的零位，可显式传 `--assume-zero-joints`；它表示假设，不会变成编码器测量。

若关节不是零位，把现场读取的五个角度（rad）写成 JSON，例如 `data/calib/face-static-joints.json`。以下数值只是格式示例：

```json
{
  "waist_yaw_joint": 0.0,
  "waist_roll_joint": 0.0,
  "waist_pitch_joint": 0.0,
  "head_yaw_joint": 0.0,
  "head_pitch_joint": 0.0
}
```

通过 `--joints data/calib/face-static-joints.json` 指定。静态文件仍被标为固定假设；机械姿态变化后必须重新读取。程序不会默认把缺失关节补零。未提供关节时，只有相机位姿和相机测距，机器人模型隐藏。

### 5.3 可选：ROS 关节插值

使用同时钟域的相机和编码器消息时，可传 `--joint-topic` 与 `--ros-clock-confirmed`。程序按名字映射五关节，支持 `ultra_head_pitch_joint → head_pitch_joint`；重复别名、NaN、缺失、越限和无有效时间括号都不能产生有效 FK。角度不钳位。

```bash
export FACE_JOINT_TOPIC='/replace_with_actual_joint_state_topic'
python -m face_table run --source ros \
  --image-topic "$FACE_IMAGE_TOPIC" \
  --joint-topic "$FACE_JOINT_TOPIC" --ros-clock-confirmed \
  --config configs/camera_a3u_face.yaml
```

它等待约 120 ms 收集编码器后括号，再按图像 Header 时间线性插值；括号间隔不得大于 100 ms，不外推、不保持上一关节样本。零值、重复或倒退的图像时间戳会报错。`--ros-clock-confirmed` 是你对两路 Header 属于同一时间基准的确认，并不代表程序已经实现或验证了硬件曝光同步。

V4L2/OpenCV 接入无法在这里提供可靠的曝光硬件时间戳，仅记录接收单调时钟，不能用于动态 FK。首次验收应保持静止；快速运动、结构弹性和回程间隙不在本次静态验证范围内。

## 6. 开始测距和可视化

### 6.1 四角点选，不用贴标记

完成内参标定后，确保配置里 `marker.enabled: false`。按实际接入方式运行其中一条：

```bash
# ROS 单目。零位选项只在现场核对零位并保持不动时使用。
python -m face_table run --source ros \
  --image-topic "$FACE_IMAGE_TOPIC" \
  --config configs/camera_a3u_face.yaml --assume-zero-joints

# 或 V4L2 单目：
python -m face_table run --source v4l2 --device /dev/video2 \
  --config configs/camera_a3u_face.yaml --assume-zero-joints
```

在 `http://127.0.0.1:8092`：

1. 确认画面来自所选面部镜头，完整台面四角可见。
2. 点击“冻结当前帧”，按“近左、近右、远右、远左”点选。近端指机器人所在的那端，左右按机器人面对球台的视角。点台面外边缘的角，不点网柱、桌腿或标记角。
3. 点击“解算四角”。左侧出现重投影轮廓，右侧出现距离和机器人相对位置。原图和该帧结果会保存到日志目录。
4. 点错就“清除点选”；要测下一次，点击“恢复实时”，重新冻结并点选。冻结结果始终只代表那一张图，即使后台相机还在采集。

页面冻结会在服务端保留该帧，避免标注过程中因缓存过期换成另一张图。程序不会用四个旧像素点在后续帧上持续解算，也没有实现无标记自动球台分割。

### 6.2 ArUco 连续定位

先生成默认标记：

```bash
python -m face_table marker --output data/calib/table-marker.png
```

默认字典 `DICT_4X4_50`，ID=7。按实物打印，量取黑色正方形外边长，不包括白色留边。默认配置 `size_m: 0.16` 表示黑色区域 16 cm；若不是 16 cm，修改实际数值。

把标记平贴在台面上，量取标记中心相对台面中心的 `[x, y, z]`，填入 `marker.center_table_m`；台面上应为 `z=0`。默认的 `[-1.0, 0, 0]` 只是示例，不能当作已测位置。打印图的上边朝向球台 `+x` 时 `yaw_rad: 0`；以此姿态为起点绕台面 `+z` 旋转的角度就是 yaw。OpenCV 角点顺序 TL/TR/BR/BL 对应标记局部 `[+s,+s]、[+s,-s]、[-s,-s]、[-s,+s]`，其中 `s` 是黑方块半边长。

修改配置：

```yaml
marker:
  enabled: true
  dictionary: DICT_4X4_50
  id: 7
  size_m: 0.16
  center_table_m: [-1.0, 0, 0]
  yaw_rad: 0
```

按 6.1 的同一命令重启，浏览器不冻结时会逐帧更新。标记需有足够像素面积、清晰黑白边界和白色留边。单个小标记的角点误差会放大成明显的距离误差；若结果不稳定，可先增加标记尺寸、改善曝光和视角，再重新量尺寸。此处不宣称其适合高速击球闭环。

### 6.3 保存图片与远程浏览

离线图片也能使用同一界面点选：

```bash
python -m face_table run --source image --image data/raw/face-left.jpg \
  --config configs/camera_a3u_face.yaml --joints data/calib/face-static-joints.json
```

图片必须与配置的镜头、分辨率和畸变模式一致，界面标为离线，没有伪造的曝光时间。只有该图片拍摄时的关节姿态才适用。

程序若在机器人计算单元运行，可从 PC 建立端口转发后仍用本地浏览器访问：

```bash
ssh -N -L 8092:127.0.0.1:8092 user@robot_host
```

把用户名和主机替换为现场值。默认绑定 `127.0.0.1`；也可显式用 `--host 0.0.0.0` 对局域网开放，再从 PC 打开 `http://机器人地址:8092`。默认端口被占用时用 `--port 8093`。

## 7. 距离、坐标和有效性怎么读

变换记号 `T_A_B` 表示把 B 中的坐标映射到 A。球台坐标沿用 `docs/frames.md`：台面中心为原点，x 向对手，y 向机器人左侧，z 向上，地面标称为 `z=-0.76 m`。

$$T_{\text{table,pelvis}} = T_{\text{optical,table}}^{-1} T_{\text{pelvis,optical}}^{-1}$$

| 输出字段 | 含义 |
| --- | --- |
| `camera_to_table_center_3d_m` | 所选镜头光心到台面中心的三维直线距离 |
| `pelvis_table_xyz_m` | 骨盆原点在球台坐标系的位置 |
| `pelvis_to_table_rectangle_xy_m` | 骨盆竖直投影到台面矩形的水平最短距离；投影落在矩形内为 0 |
| `pelvis_to_table_center_xy_m` | 骨盆到台心的水平距离 |
| `pelvis_to_table_center_3d_m` | 骨盆到台心的三维直线距离 |
| `near_end_signed_gap_x_m` | `-台长/2 - 骨盆x`，正值表示在近端线外；偏到侧面时不等于最近距离 |
| `valid` / `camera_pose_valid` | 当前这次视觉平面解算是否非歧义；不等于经过标定的机器人定位有效 |
| `robot_pose_valid` | 还要求镜头外参已验证、光心偏移已填写、关节非固定假设、且不是合成数据 |
| `robot_pose_grade` | `unavailable / assumed_fixed / nominal_CAD / calibrated / synthetic` |

“机器人距离”指骨盆原点，不是脚尖、身体外轮廓或防碰撞余量。标称模型、镜头外参或关节有假设时，页面保留近似数值但显示警告。

PnP 使用平面 IPPE 初值和 LM 优化，检查点序凸性、面积、像素范围、正深度、相机处在台面上方、最大距离及重投影误差。若两个不同位姿候选的 RMS 相差不到 0.2 px，标为歧义，不作为有效结果。只有四个角点时仍可能存在错误对应带来的低误差解，因此不能仅凭 RMS 验收。

实时接收超过 `--stale-s`（默认 0.7 s）无新图像，界面显示 stale 并使返回结果失效。标记丢失的当前帧不保留旧位姿。冻结图的显示是历史观察，不会被改称当前实测。JSONL 中的有效性是记录生成时的判断，下游仍须检查时间戳和方法，不能把旧行当实时定位。

## 8. 动捕与误差验收

这个程序的机器人—球台相对位置不需要动捕世界位姿，因此没有把规范中的 Y-up Marker→pelvis 平移硬加到视觉结果里。内参也绝不由动捕拟合。

若后续用动捕评测，必须先在同一时刻取得机器人 Marker 与球台真值，再按已验证的完整 Marker→pelvis 刚体变换转换。`[0.0018, 0.1265, 0.0002] m` 不能直接当作这里 z-up 下的平移，也不能漏掉局部轴旋转。参考查看器 `config/scene_20261028.json` 给出了当时的完整平移与四元数；那是有日期的映射，不是本程序对当前设备的标定证明。本版本不接入动捕，也没有实现动捕对齐后的自动误差统计。

建议先做静态粗验收：

1. 实测桌长、桌宽、黑色标记尺寸及其摆放坐标。
2. 在 3 个以上实际站位分别测量近端距离，重复采集，观察均值偏差、波动及重投影轮廓。若用卷尺对比，测量基准必须对应骨盆投影；不能用脚尖到台边直接对比骨盆距离。
3. 固定骨盆，改变并记录头部/腰部角度，检查 FK 补偿后机器人—球台位置是否保持基本一致。若换头姿就系统偏移，优先查 URDF 版本、光学轴、左右镜头偏移和关节时间对齐。
4. 测试标记遮挡、缺关节、图像流中断，确认结果变为无效或缺失，没有保持旧值冒充新值。

粗测可先自行设定例如 5–10 cm 的允许误差，最终阈值由实际用途决定。合成验证不能替代这一实机验收。

## 9. 日志、测试与已知限制

每次运行默认写入 `logs/face-table-日期-时间/`：

- `run-config.yaml`：本次实际使用的配置快照。
- `run.json`：镜头、话题、设备、URDF 路径与 SHA256、固定关节等溯源信息。
- `observations.jsonl`：逐次解算结果、时间基准、像素点、重投影误差、变换、距离和警告；自动标记丢失的帧记录 `valid=false` 与原因。
- `frame-xxxxxxxx.jpg`：手动保存/点选的原始图像。加 `--record-images` 可保存每个接收帧，会增加存储量。未标定预览与未初始化手工模式不生成有效位姿行。

```bash
python -m unittest discover -s tests -v

# 可选浏览器验证：先在另一终端启动下述演示服务。
python -m face_table run --source demo --port 18092 --output logs/face-table-validation
# 环境中已有 playwright-core 与 Chrome 时：
node tests/browser_face_table.cjs
```

浏览器测试使用 `PLAYWRIGHT_MODULE` 指向可用的 `playwright-core` 模块，`CHROME_BIN` 指向 Chrome；脚本默认沿用本机查看器测试使用的安装路径。可通过 `FACE_TABLE_URL` 指定测试服务地址。产物在 `logs/face-table-validation/`。

本次已在本机完成纯几何、合成 ArUco、HTTP 和浏览器交互验证，实际测试证据见 `face-table-test-validation.md`。没有连接实机相机：ROS 2 接入尚未现场验证，镜头内参和外参也尚未测量。程序已提供可运行入口，但不能把离线通过写成“A3U 实机已测通”。
