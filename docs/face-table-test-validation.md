# 面部单相机球台定位测试记录

日期：2026-11-01。执行位置：`/home/mine/a3u-pingpong-vision`，本机 CPU 与无头 Chrome。未连接机器人。

## 环境

| 项目 | 实际版本 |
| --- | --- |
| Python | 3.10.12 |
| NumPy | 1.21.5 |
| OpenCV（含 ArUco） | 4.5.4 |
| PyYAML | 5.4.1 |
| 本地可视化网格 | 40 / 40 载入 |

本次直接使用本机已有 Python 包与查看器测试所用的 Playwright/Chrome，没有安装或修改系统依赖。`requirements-face-table.txt` 允许更新版本；本次没有逐个测试所有允许版本。

## 实际执行与结果

1. 读取本仓库 FK 规范和坐标契约，核对 `/home/mine/a3u-pose-viewer` 的 URDF、球台几何与本地 Three.js 资产。确认默认 `a3_t3d0` 的相机 FK 与规范一致；另记 Ultra T2.5 的固定外参差异，未混用。
2. 在本仓库实现只读输入、标定、几何解算、HTTP 接口、前端、配置与测试。已有暂存的 `.gitmodules` 和 `assets/robot_model` 未改动，原始 FK 规范也未改写。
3. 执行 `python3 -m unittest discover -s tests -v`，最终 15 / 15 通过。覆盖非零 FK、畸变、镜头偏移、距离定义、错序/无效角点、缺失/越限关节、别名重复、时间括号、离线图片时间基准、未标定与合成内参拒绝、ArUco 整链路、HTTP 点选/保存/固定帧、资产访问边界、过期失效和遮挡不保持旧值。
4. 在单元测试中独立渲染 15 个不同视角的棋盘格，执行真实角点检测及 `calibrateCamera`。15 张全部识别，标定 RMS 约 0.147 px；这不是实机镜头标定。
5. 启动 `python3 -m face_table run --source demo --port 18092 --output logs/face-table-final-validation`，通过本地 HTTP 获取 ArUco 自动解算结果。记录如下表。
6. 执行 `node tests/browser_face_table.cjs`，通过桌面与 390 px 移动视口测试。实际点击四角、解算、保存、切换三个视角并恢复实时，40 个网格全部载入，浏览器脚本错误为 0。随后查看截图核对相机图、标注与机器人—球台场景。
7. 执行设备发现，返回 `video_devices: []`；本机 `rclpy` 不可用，未启动 ROS 接入。尝试用未标定模板启动图片解算，程序按预期拒绝，退出码为 1，而不是填充演示内参。
8. 执行 Python 编译检查、浏览器测试脚本语法检查与 `git diff --check`，通过。编写并校对运行指南。测试服务完成验证后停止；未创建 commit、推送或合并。

## 合成 ArUco 自动定位数值

| 指标 | 数值 |
| --- | --- |
| 预设骨盆在球台系位置 | [-2.200000, 0.120000, 0.450000] m |
| 解算骨盆位置 | [-2.204213, 0.118961, 0.452901] m |
| 平移误差范数 | 0.005219 m |
| 预设近端间隙 | 0.830000 m |
| 解算近端间隙 | 0.834213 m |
| 标记重投影 RMS | 0.104306 px |
| 机器人位置等级 | synthetic |
| `robot_pose_valid` | false，演示数据不标为实机有效位姿 |

上述结果使用光栅化的合成标记图像，不是直接将三维真值作为解算输出。独立的非零关节/畸变数值测试还检查了连续几何往返误差。两者均只能证明软件在设定条件下的行为，不能推导实机毫米级精度。

## 本地证据

- [最终单元测试输出](../logs/face-table-final-validation/unit-tests.txt)
- [合成自动定位数值](../logs/face-table-final-validation/synthetic-result.json)
- [浏览器测试结果](../logs/face-table-validation/browser-result.json)
- [桌面截图](../logs/face-table-validation/browser-desktop.png)
- [移动视口截图](../logs/face-table-validation/browser-mobile.png)

日志目录按仓库约定不入 Git；上述证据链接用于当前本机工作区。复现步骤见 [运行指南](face-table-test-runbook.md)。

## 尚未验证

实机左/右镜头与话题/设备的对应、厂商驱动输出、实机内参、单镜头光心相对 CAD 的偏移、光学轴约定、实机硬件型号、ROS 曝光与编码器的时间基准、机器人不同姿态下的真实定位误差，均需现场确认。没有采集动捕真值，没有进行动态精度或遮挡率统计，也没有驱动机器人运动。
