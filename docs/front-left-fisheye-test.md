# 头部前向左目鱼眼：机器人—球台定位测试

日期：2026-11-03。软件已增加鱼眼支持，出厂参数已从本地 ADU 快照导入；尚未用实机画面验收。本文只讨论球台静态定位，不代表高速小球三维观测已经可用。

## 已解决与待验证

| 项目 | 当前状态 |
| --- | --- |
| 投影模型 | 增加 OpenCV 四系数 `fisheye` 分支；不把厂商全部 k/p 字段拼接成畸变数组 |
| 平面位姿 | 原图角点先反投影至归一化射线，再用 IPPE、LM 求解；误差回到原始鱼眼像素计算 |
| 画面轮廓 | 先在三维中采样桌边，再投影成曲线，不用四条直线近似鱼眼中的桌边 |
| 内参来源 | HAL `front_left_camera` 与出厂 `head_stereo_left_fisheye_camera` 的内参、尺寸及四系数交叉核对；参数附输入文件 SHA256 |
| 光心外参 | 增加 `T_pelvis_parent @ T_parent_optical` 路径；不再重复乘 CAD 相机安装变换 |
| 模型错配 | 新配置绑定 Ultra T2.5 URDF 内容哈希；选错模型会拒绝启动 |
| 离线标定 | 棋盘格标定增加 `--distortion-model fisheye` |
| 实机图像来源 | 待确认具体话题/设备、实际镜头、1920×1536 原始畸变图像，无裁剪或驱动矫正 |
| 外参语义 | 待验证四元数/平移方向、`ultra_head_pitch_Link` 与 URDF `head_pitch_Link` 是否同一坐标约定 |
| 最终精度 | 待用真实球台、实测标记或独立距离真值验收，不能由合成测试推出 |

无需先采购深度相机。已知尺寸的球台或标记可提供单目定位的公制尺度。深度相机同样需要正确的机器人安装外参。

## 1. 导入出厂候选参数

在仓库根目录执行：

```bash
python3 -m face_table import-factory
```

默认读取 `robot_param/sensor_param.yaml`、`robot_param/hal/rgb_camera_params.yaml`，生成 `configs/camera_a3u_front_left_fisheye.yaml`。不要覆盖原有的针孔模板；演示和旧的针孔流程继续可用。

导入过程检查 HAL 名称，不按序列号绑定。已有快照中序列号重复，必须通过驱动通道和实际画面确认镜头。

`calibrated: true` 在这里表示已有出厂内参可供解算，不表示现场验收完成。新配置保留：

```yaml
intrinsics_field_verified: false
optical_extrinsic_verified: false
factory_reference_mapping_verified: false
```

`T_parent_optical` 暂按出厂 q/t 是“光学系到参考系”的变换解释，并将参考系候选映射到 Ultra T2.5 的 `head_pitch_Link`。这是假设，不是仅凭节点名称就能证明的事实。程序会显示警告，并保持 `robot_pose_valid: false`。不能只为消除提示而修改这些标志。

直接参考系外参不能与 `lens_translation_mount_m` 或 CAD 残差同时使用；已有矩阵包含光心平移。模型哈希只防止误选文件，不证明这个模型与实机完全一致。

## 2. 首先验证相机—球台位姿

从实际左目保存一张 1920×1536 的原始畸变图，放在 `data/raw/front-left.jpg`。以下命令假设该文件已经存在：

```bash
python3 -m face_table run --source image \
  --image data/raw/front-left.jpg \
  --config configs/camera_a3u_front_left_fisheye.yaml \
  --urdf assets/robot_model/a3_ultra_t2d5/urdf/model.urdf
```

在 `http://127.0.0.1:8092` 冻结图像，按近左、近右、远右、远左点选实际台面四角。没有关节数据时只输出相机位姿，骨盆位置不可用，这是正常状态。

程序拒绝尺寸不符的图像，不自动缩放、裁剪、翻转或更换内参。四角不全时可使用量过尺寸、位置及朝向的台面 ArUco；配置方法见原运行指南。小标记或画面边缘的强畸变可能影响检测和位姿稳定性，第一轮优先人工点选四角。

原始鱼眼像素反投影后必须能重新投回原像素，往返误差超过 0.05 px 会拒绝求解。本流程只接受可反投影到前半球的对应点，不宣称覆盖任意超过 180° 的鱼眼图像。

## 3. 固定姿态验证机器人—球台坐标链

记录图像曝光时五个关节的实际角度，使用原运行指南中的 JSON 格式：

```bash
python3 -m face_table run --source image \
  --image data/raw/front-left.jpg \
  --config configs/camera_a3u_front_left_fisheye.yaml \
  --urdf assets/robot_model/a3_ultra_t2d5/urdf/model.urdf \
  --joints data/calib/face-static-joints.json
```

文件必须来自该帧实际姿态，不要直接抄零值示例。静态 JSON 仍标为 `assumed_fixed`，只供静态粗测，不自动晋升为动态有效机器人测量。

验证顺序：

1. 用已知尺寸与位置的标记或台面四角，检查相机测距和重投影；额外保留未参与拟合的图像或点作核验。
2. 确认出厂光学轴方向与所选镜头一致，核对实机模型和参考系映射。若不一致，先修正模型或重新标定外参，不通过修改有效性标志掩盖问题。
3. 在多个已测站位比较骨盆投影到台边的距离与朝向。不能拿脚尖距离冒充骨盆距离真值。
4. 固定骨盆并记录不同头腰角度，检查 FK 补偿后结果是否一致。移动关节应由现场原有操作流程完成，本程序不发送运动指令。
5. 检查遮挡、缺关节、时间戳错误、图像中断及平面解歧义时结果是否失效。

建议将水平位置误差 5 cm、朝向误差 1°作为首轮待确认目标，不是软件对硬件的精度承诺。四角低重投影误差不等于外参、尺度或真实距离正确。

## 4. 连续输入与重新标定

已有 ROS 入口可直接使用新配置和上述 `--urdf`。用 `python3 -m face_table discover --ros` 查找实际单镜头图像与关节话题，不在命令中猜测话题名。图像/关节同钟和有效时间括号的原有门禁不变；V4L2 接收时间不能冒充曝光硬件时间。

若出厂参数在独立图像上不满足精度要求，用实际左目采集至少 12 张、建议 20–30 张不同姿态棋盘格图像，再运行：

```bash
python3 -m face_table calibrate \
  --images 'logs/front-left-calibration/frame-*.jpg' \
  --distortion-model fisheye \
  --columns 9 --rows 6 --square-m 0.025 \
  --camera-id head_stereo_left_fisheye_camera \
  --template configs/camera_a3u_front_left_fisheye.yaml \
  --output configs/camera_a3u_front_left_fisheye_offline.yaml
```

格宽必须按实物修改。此过程只重估内参，不会验证或重估机器人安装外参。更换镜头标签会清除原镜头直接参考系外参，避免将它沿用到另一颗镜头。离线标定后仍需留出图像及独立距离真值验收。

## 本轮验证与范围

本机执行 `python3 -m unittest discover -s tests -v`，22 项测试全部通过。新增覆盖包括独立 theta 多项式投影、非零关节下位姿恢复、20 次固定种子的 0.25 px 标准差角点噪声、参数不一致拒绝、错误 URDF 拒绝、重复外参叠加拒绝和未经验证不得输出有效机器人定位。

棋盘格测试分别渲染 15 张针孔图与 15 张非线性鱼眼图，真实检测角点并运行各自标定器；RMS 分别约 0.147 px、0.148 px。这些都是合成测试，不是实机精度。

测试输出保存于 `logs/fisheye-adaptation-tests.txt`。本轮没有连接 ADU、采集现场画面、修改机器人配置或服务，也没有发送运动命令。

另启动本地合成演示服务（端口 18094），运行已有浏览器回归脚本并查看桌面截图：40/40 网格载入，四角点选、冻结解算、保存、恢复实时和移动布局通过，页面脚本错误为 0。浏览器回归使用针孔合成演示，鱼眼数学与非线性标定由上述新增测试覆盖；本轮未做真实鱼眼画面的浏览器验收。浏览器产物在 `logs/fisheye-ui-check/logs/face-table-validation/`。测试服务已停止，未提交或推送代码。
