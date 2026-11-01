# PYNQ-Z2 最小 Vivado 工程

工程提供传感器计算所需的 RTL、Block Design、约束、重建脚本和综合报告，接口依据当前 HWH 和 Python 寄存器定义。

## 本次范围

- XC7Z020-CLG400-1，Vivado 2018.3，TUL PYNQ-Z2 board definition 1.0。
- Zynq PS7、DDR/FIXED_IO、100 MHz FCLK0、复位及 AXI 互连。
- AXI UART Lite：115200、8N1；4 个按键 GPIO、UART RX 电平 GPIO。
- SP21 传感器计算核心：编码器累计计数差分、车体 X/Y 位移、整数平方根路径长度、速度、陀螺零偏扣除和相对航向积分。
- 输入同步、独立 AXI AW/W 握手、WSTRB、读写响应反压、清零和处理忙状态。

UART 接收字节及 CRC/RAW33 解包由 PS 软件组织，再将 7 个累计计数和零偏写入 AXI 寄存器，与 `sensor_driver.py` 的调用边界一致。摄像头识别和显示应用见仓库 `pynq/` 目录。

## 打开与重建

运行重建脚本后生成的工程入口：

`generated/smartcar_minimal/smartcar_minimal.xpr`

请在纯英文路径下操作，将 Vivado 加入 PATH 后，在本目录执行：

```powershell
vivado -mode batch -nojournal -source build.tcl -tclargs synth
python sim\run.py
```

`build.tcl create` 创建工程；`synth` 生成综合报告；`implement` 完成布局布线和对应报告。生成物在 `generated/`，报告在 `reports/`。

## 地址和接口

| 模块 | 地址 |
| --- | --- |
| `axi_uartlite_0` | `0x42c00000` |
| `btns_gpio` | `0x41210000` |
| `rx_level_0` | `0x41260000` |
| `sensor_core_0` | `0x43c80000` |

| 核心偏移 | 内容 |
| --- | --- |
| `0x00` | 工程标识 `SPM1` (`0x53504d31`) |
| `0x34` | 传感器子接口标识 `SP21` (`0x53503231`) |
| `0x40..0x58` | FL、FR、RL、RR、gyro_sum、t20、t5，均为 32 位累计量 |
| `0x5c` | 有符号 16 位 gyro bias，放在低 16 位 |
| `0x60` | bit0 提交，bit1 清零，清零优先；忙时命令不接收 |
| `0x64` | bit0 busy |
| `0x68 / 0x6c` | 已发布样本数 / 拒绝并重新建立计数基准的次数 |
| `0x70` | 速度，微米/秒 |
| `0x74 / 0x78` | 路径累计微米数，低/高 32 位 |
| `0x7c` | 有符号 Q16 百分之一度，相对航向 |
| `0x80 / 0x84` | 有符号车体 X/Y 位移，微米 |

首次输入只建立基准。有效间隔为 t20 差值 1–50、t5 差值 1–200，二者满足 `abs(4*dt20-dt5)<=5`；其他间隔重新建立计数基准并增加拒绝计数，不累加路径。累计计数按 32 位回绕差分，输入四轮增量为有符号值。

计算依据当前固件比例：东西向 2535 脉冲/0.2 m，南北向 2430 脉冲/0.2 m。先做有符号整数截断，再以整数平方根计算路径长度。航向使用定点系数 2291，并归一化到 `[-180,180)`。

## 引脚和验证说明

板卡引脚来自本机已安装的 TUL PYNQ-Z2 `part0_pins.xml`：RX 为 PMOD B1/W14，TX 为 PMOD B2/Y14；BTN0–3 为 D19、D20、L20、L19。异步输入经过两级同步；XDC 对外部异步串口/按键路径做例外处理，内部同步路径继续分析。

仿真含 46 组独立 Python 整数参考向量、383 项寄存器检查，以及 8 组 64 位除法/平方根边界例。包括现有驱动的直行/静止转向自测、正负运动、零偏、航向回绕、计数重启、32 位回绕、重复输入、清零、忙时重发、处理期间修改暂存寄存器和 AXI 响应反压。另单独检查命令接受后 busy 的立即可见性。

综合与仿真结果见 `reports/build_result.json`。

## 本次结果（2026-11-01）

| 检查 | 结果 |
| --- | --- |
| Block Design 验证与工程重新打开 | 通过，7 个直接工程文件引用均存在 |
| 完整综合 | 完成 |
| 时钟目标 | 100 MHz |
| 综合时序估计 WNS / WHS | +0.461 ns / +0.045 ns |
| LUT / 寄存器 / DSP | 3060 / 3136 / 14 |
| 仿真 | 46 组参考输入、383 项寄存器检查、8 组算术边界例通过 |

综合 DRC 保留 21 项 DSP 输出流水寄存器使用建议，无 Error/Critical Warning 项。表中时序数据为综合后的估计结果。
