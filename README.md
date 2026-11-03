# a3u-pingpong-vision

乒乓球机器人视觉项目工作区。本目录只承载视觉方案、复现记录、标定与观测数据，不存放机器人控制代码。

## 当前阶段

已有面部单相机球台定位测试程序，已完成离线合成与浏览器验证；实机相机接入、内参和选定镜头外参仍需现场确认。

头部前向左目现已支持鱼眼投影、平面定位和离线鱼眼标定，已提供 ADU 出厂候选配置。软件测试不代表实机已验收，入口与剩余验证见 [前向左目鱼眼测试](docs/front-left-fisheye-test.md)。

## 面部相机测距测试

运行与标定步骤见 [面部相机球台定位运行指南](docs/face-table-test-runbook.md)。程序只读图像和关节状态，不发布机器人命令。

```bash
cd /home/mine/a3u-pingpong-vision
python3 -m face_table run --source demo
```

浏览器打开 `http://127.0.0.1:8092`。此模式为合成演示，不调用实机；实机使用 `--source ros` 或 `--source v4l2`，接入参数见运行指南。

## 机器人访问

A3 Ultra 三台主机在同一有线网段 `10.42.10.0/24`，账户均为 `agi`；认证凭据仅保存在本地忽略文件 `ROBOT_ACCESS.local.md`，不提交到远程。

| 主机 | 地址 | 说明 |
| --- | --- | --- |
| HDU | `agi@10.42.10.10` | 跳板机，ADU/MDU 对外不直接暴露时经此转发 |
| ADU | `agi@10.42.10.11` | 机器人端主机，传感器参数所在 |
| MDU | `agi@10.42.10.12` | 运动控制端主机 |

```bash
ssh agi@10.42.10.11
ssh -J agi@10.42.10.10 agi@10.42.10.12
```

传感器参数路径（均在 ADU）：

- 内参：`/agibot/data/param/hal/`（各相机 `*_camera_info.yaml`、`rgb_camera_params.yaml`）
- 外参：`/agibot/data/param/calibration/sensor_param.yaml`（每相机 `K`/`D` + `ref_frame` 与四元数 `q`、平移 `t`）

拉取的快照存放在 `robot_param/`（`hal/` 与 `sensor_param.yaml`），只读参考，采集时间 2026-10-08，`calib_type: factory`。

## 目录

| 路径 | 内容 |
| --- | --- |
| `docs/` | 接口契约、labels 定义与决策记录 |
| `docs/decisions/` | 每个架构决策一条记录 |
| `research/` | 论文与厂商资料笔记、复现记录 |
| `observation/` | 空间标定与观测接口说明 |
| `configs/` | 相机、标定和评测配置 |
| `robot_param/` | 从 ADU 拉取的出厂内外参快照 |
| `data/raw/` | 原始录制（不入库） |
| `data/calib/` | 标定结果 |
| `data/labels/` | 真值标签（不入库） |
| `logs/` | 运行日志（不入库） |
| `scripts/` | 采集、回放与评测脚本 |

## 约定

- 所有传感器数据必须带采集时间戳与坐标系标识。
- 坐标定义优先沿用 `docs/frames.md`，如需改动先写决策记录。
- 原始数据不提交到版本库，只提交处理脚本和元数据。
- 路由与参考：递进流程见 `docs/workflow.md`；采购清单见 `docs/hardware-bom.md`；接口契约见 `docs/interface.md`。

## 边界

本仓库不包含机器人运动控制、策略权重或部署包。感知输出经 `docs/interface.md` 定义的契约交付给控制侧。
