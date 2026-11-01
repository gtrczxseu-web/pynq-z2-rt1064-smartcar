# PYNQ-Z2 + RT1064 Smart Car

麦轮小车视觉交互、分步/连续路线与近区障碍绕行。最终应用快照：2026-11-01；MCU 协议版本：V8。

本仓库提供 PYNQ/RT1064 最终应用源码、Overlay 运行文件、验证材料及技术文档。详见 [材料清单](report/submission_status.md)。

## 功能

- STEP / CONTINUOUS，屏幕参数菜单；距离 2.0–4.0 m，每次 +0.2 m 循环；菜单速度 20%–200%，每次 +20%；固定转角 90°/180°/360°/720°。单独平移、旋转仅用于 STEP。
- 镜头朝向定义为逻辑前方，原车尾为新车头。实际轨迹小地图、摄像头预计路线、转向标记与运行结束报告。
- 连续模式：红色暂停、绿色恢复、黄色暂停 5 s；关闭数字识别。单步模式只在停下等待时检测，绿色或 BTN0 进入下一步，数字 0–9 对应下一步 60%–150% 基准速度。
- 固定摄像头/平地条件下，约 50 cm 近区视觉障碍检测与倒影抑制。确认障碍后停稳，自动右移 0.5 m → 前进 1 m → 左移 0.5 m，恢复原步骤目标。两侧障碍未检查；绕行前进阶段再次遇障碍则等待清空后 BTN0/绿色恢复，不嵌套绕行。本步骤剩余不足 1 m 时拒绝固定绕行。
- HSV 色卡与预训练 MNIST ONNX 数字识别运行在 Zynq ARM PS；本仓库不宣称 PL 神经网络加速，不包含重新训练结果。

## 目录与指南对照

采用指南允许的替代组织方式，保留现有运行目录以免破坏依赖：

| 目录 | 指南对应内容 |
| --- | --- |
| `pynq/` | PS Python/C 源码、模型、配置、Overlay 运行文件、服务与部署入口 |
| `firmware/` | RT1064 源码、Keil 工程、SDK/第三方库及最终 AXF |
| `pynq/verification/` | 最终应用测试及固定回归输入 |
| `sim/` | 离线测试入口、实际 C 控制代码的理想模型联调 |
| `build/` | MCU/native 库构建脚本与软件包校验入口 |
| `board/validation/` | 既有板端测试、标定、状态记录与编译日志 |
| `skill/` | 可复用校验与调试经验，含适用范围、操作及失效边界 |
| `report/` | 技术文档、协作记录摘录及材料清单 |

`pl_camera_test.py`、`usb_cdc_only_test.py` 是历史命名的**运行依赖**，不可因文件名含 test 删除。`rear_camera.json` / `rear_projection.py` 同样保留历史命名；逻辑前方以当前坐标配置为准。

## 离线复现

Python 3.10+，安装 NumPy、OpenCV（需要 `cv2.dnn`）。本次验证使用项目已有环境，精确版本见 `sim/results/export_environment.json`。先不要连接电机驱动。

```sh
python build/check_package.py
python pynq/install_service.py
python sim/run_tests.py
```

以上命令只检查文件或执行离线测试，不操作板卡、电机或烧录器。输出写入忽略的 `sim/generated/`。C 理想模型联调需 GCC：

```sh
python sim/run_native.py --gcc gcc
```

Keil 编译（Windows，uVision 5、Arm Compiler 6.19、NXP MIMXRT1064 DFP 19.0.0）：

```powershell
powershell -ExecutionPolicy Bypass -File build/build_firmware.ps1 -Uv4Path 'C:\Keil_v5\UV4\UV4.exe'
```

只编译、不自动烧录。工程入口 `firmware/bisai300/mdk/rt1064.uvprojx`，目标 `nor_sdram_zf_dtcm`；发布 AXF 的原始 SHA256 为 `56ad2df2484af36f81cb3d40f31f408bdfcac327df038c08ec43e645f5bf02dd`。

## PYNQ 部署与运行

1. PYNQ-Z2 使用已配置的 PYNQ Linux/Python 环境，接 HDMI、USB 摄像头、BTN0–BTN3；按实际硬件接好 UART 和共地。RT1064 使用 V8 固件。接线及引脚以实际 Overlay/板卡配置为准，不从其他版本工程推断。
2. 上传 `pynq/` **目录内的全部文件**到 `/home/xilinx/jupyter_notebooks/hud_motion_v7/`。若替换已运行服务，先关闭电机驱动，保存原目录、可变配置及服务文件，并确认没有进行中的任务。
3. 在板端执行 `python3 /home/xilinx/jupyter_notebooks/hud_motion_v7/install_service.py` 检查清单。需要安装自启动时，再以 root 执行同入口加 `--install`。这一步会重启显示服务；不会自动烧录 MCU。
4. 车体静止完成初始化，确认串口、摄像头正常且 READY；在屏幕菜单选择参数后，按 BTN0 启动。BTN1 结束当前轮次。电机方向由用户先低速验证。
5. 自启动已配置后可脱离网线运行；网线用于上传与维护。结束运行后在 **PYNQ** 执行正常 Linux 关机，再断板卡电源。

原服务支持 `install_service.py --rollback` 恢复其保存的旧服务文件；这不等于完整恢复应用源码或 MCU。完整回退应使用安装前保存的目录、配置与固件，不依赖本仓库重建私人备份。

ARM native 库的重新编译请在 PYNQ 执行 `python3 build/build_native.py`（需将 build/ 也复制到板上同级目录）；不要把 x86 主机编译的 `.so` 覆盖到 PYNQ。

## 已有验证与限制

原发布离线应用 292 项、板端应用 292 项通过；实际 C 控制代码理想模型联调 8 项、旧 V7 协议联调 11 项、部署/回滚模拟 5 项通过。各自证据类型不同，不能合并当成实车成功次数。上传副本的复测另见 `sim/results/`。

50 cm 样本 8/8 检出，1 m 样本 8/8 CLEAR，空地 5/5 CLEAR；这些是固定位置采样，不是跨场景准确率。标定约束为固定相机和近似平地，不能当作单目通用测距。最近一次静止板端记录约 4.476 摄像头帧/秒，未声称达到 30 FPS。详见 [验证说明](board/readme.md)。

用户在修复后反馈“验证无误”是定性验收；已有 JSON 中 `real_vehicle_retest_pending` 为当时快照，保留原始记录，未回填虚构次数。项目不提供所有材质/光照下的障碍识别保证。

## 许可与来源

自有发布源码按根目录 `license` 中 GPL-3.0-or-later 提供；第三方源码与模型保留各自原许可。详见 [组件清单](third_party_notices.md)。不将 NXP/CMSIS/模型或 Overlay 冒称为自有原创。Vivado/Keil 等专有工具不随仓库分发。
