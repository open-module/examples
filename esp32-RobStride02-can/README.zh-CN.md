[English](README.md)

# ESP32-S3 + RobStride RS02 CAN 例程

这个独立 ESP-IDF 例程通过 ESP32-S3 和 3.3 V CAN 收发器验证 RobStride RS02 私有 CAN 协议。默认构建不包含主动位移；另有一个需要显式启用、用于台架的 60° 往返测试配置。

实现依据 2026-07-13 版 RS02 使用说明书：Classical CAN、1 Mbit/s、29 位扩展帧、8 字节载荷。ESP32 控制器使用每 bit 20 个时间量子和 80% 采样点；说明书只规定了 1 Mbit/s，80% 采样点是本例程的控制器侧选择。

资料基线：

- [`RS02 使用说明书 260713.pdf`](https://github.com/RobStride/Product_Information/blob/0f4ad74fdb67023e75bbcbeebecd6f1a003ce000/%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/RS02/RS02%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6260713.pdf)，SHA-256 `e78aaf6d8b67dfbbc39981513d491313c9f12b543346d9e6d2a15ba20937b1dc`。
- Product Information 修订 `0f4ad74fdb67023e75bbcbeebecd6f1a003ce000`。
- 官方 [`RobStride/SampleProgram`](https://github.com/RobStride/SampleProgram/tree/5f598686b05fcc527ee0fc0ea954f0afd652b234)，修订 `5f598686b05fcc527ee0fc0ea954f0afd652b234`。

## 安全边界

RS02 是额定 48 V、资料标注工作电压范围为 24–60 V 的高扭矩执行器。上电前必须：

- 电机卸载并可靠固定，输出端周围保持空旷；不得安装在载人或安全关键机构上测试。
- 使用带限流的台式电源，并准备随时可操作的实体电机电源断开开关。
- 接线、烧录、重新连接 USB 或测量终端电阻时保持电机电源关闭。
- 电机电压只能连接 RS02 `VBAT+`，不得接入 ESP32 或收发器电源。
- 非隔离 CAN 收发器需要与电机电源负极共信号地；需要隔离地时应改用合适的隔离收发器。
- 按说明书的引脚编号核对接线，不得仅凭连接器朝向判断。

软件会拒绝错误 ID、非法帧、非有限数值、非零反馈故障、type 21 故障/预警、致命 TWAI 告警、反馈超时、24–60 V 范围外的母线电压、暂定 `[-20, 80) °C` 范围外的温度、绝对值达到 `0.5 rad/s` 的速度、达到 `1.0 N·m` 的转矩或达到 `0.10 rad` 的位置漂移。这些是暂定的软件观测边界，不是认证限值、硬件限流或电源隔离。CAN 或 MCU 掉电仍可能让停止命令无法到达电机。

## 接线

ESP32-S3 到 CAN 收发器：

若使用与 CyberGear 例程相同的开发板，其原理图、固定终端和使用限制见本仓库的 [SN65HVD230 板卡资料](../hardware/SN65HVD230-CAN-Board/)。

| ESP32-S3 | CAN 收发器 |
|---|---|
| GPIO4 | TX / D |
| GPIO5 | RX / R |
| 3V3 | 3.3 V 电源 |
| GND | 信号地 |

收发器和电源到 RS02 的 XT30PB（2+2）连接器：

| RS02 引脚 | 信号 | 连接 |
|---|---|---|
| 1 | VBAT+ | 带限流的电机电源正极 |
| 2 | GND | 电机电源负极；使用非隔离收发器时也接信号地 |
| 3 | CAN_L | 收发器 CANL |
| 4 | CAN_H | 收发器 CANH |

CANH/CANL 使用短双绞线，总线两端正确配置 120 Ω 终端。完整总线断电时，两个简单端点终端通常测得约 60 Ω。读数或接线有疑问时不要上电。

## 协议覆盖

例程实现并测试：

- 设备 ID 请求（通信类型 0）；
- 零力矩运控帧（类型 1）：位置、速度、Kp、Kd 为大端，转矩位于标识符 bits 23..8；
- 反馈解析（类型 2）：模式、故障位、位置、速度、转矩和温度；
- 使能与停止（类型 3、4）；
- float 参数读取（类型 17）：负载端计圈机械角 `0x7019` 和母线电压 `0x701C`；
- 类型 18 参数写入并读回：`run_mode`（`0x7005`）和临时运动转矩限制（`0x700B`）；
- 故障反馈（类型 21）。

说明书对 type 21 标识符中的电机/主机字节顺序存在正文与示例不一致。解析器只在两个字节分别严格匹配已配置电机 ID 和主机 ID 时兼容两种顺序，不会接受任意帧。

运控范围为：位置 `-12.5..12.5 rad`、速度 `-44..44 rad/s`、转矩 `-17..17 N·m`、Kp `0..500`、Kd `0..5`。位置量化范围跟随官方 SampleProgram；说明书将标称范围写为 `-4π..4π`，其中嵌入示例使用 `±12.57`。默认和 link-soak 配置将目标速度、前馈转矩、Kp 和 Kd 全部设为零。

## 主机测试

在仓库根目录运行：

```bash
cmake -S esp32-RobStride02-can/test \
  -B esp32-RobStride02-can/build-host
cmake --build esp32-RobStride02-can/build-host
ctest --test-dir esp32-RobStride02-can/build-host --output-on-failure
```

测试覆盖精确帧 ID 和字节序、范围钳制、身份校验、float 参数、type 21 的两种文档布局、非有限值拒绝、安全边界和 TWAI 时序。测试通过不代表 RS02 实体行为已经验证。

## 固件构建与行为

加载 ESP-IDF 5.5.x 并构建 ESP32-S3 固件：

```bash
. /path/to/esp-idf/export.sh
cd esp32-RobStride02-can
idf.py set-target esp32s3
idf.py build
```

默认电机 ID 为 `0x7F`，主机 ID 为 `0xFD`，TWAI 引脚为 GPIO4/GPIO5；可在 `idf.py menuconfig` 的 `RobStride RS02 CAN example` 中修改。

固件每次启动只执行一次故障关闭式流程：

1. 发送停止并要求收到匹配的 Reset 模式反馈。
2. 请求并核对设备 ID。
3. 读取并检查母线电压，再读取 `mechPos`。
4. 使能并要求收到匹配的 Motor 模式反馈。
5. 以标称 50 Hz 持续 500 ms：位置字段使用启用后测得的初始位置，目标速度、前馈转矩、Kp、Kd 全部为零，并检查每次反馈。
6. 再次停止，要求 Reset 模式反馈，然后关闭 TWAI。

Kp、Kd 为零意味着位置字段不会主动保持位置。但测试期间电机仍处于使能状态，硬件故障或外力仍可能造成运动。

需要更长的纯链路诊断时，使用 `sdkconfig.link-soak.defaults` 构建；它只把相同零力矩阶段延长到 60 秒，不改变控制项：

```bash
idf.py -B build-link-soak \
  -D SDKCONFIG=build-link-soak/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.link-soak.defaults' \
  set-target esp32s3
idf.py -B build-link-soak \
  -D SDKCONFIG=build-link-soak/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.link-soak.defaults' \
  build
```

## 显式启用的 60° 运动测试

仅在 RS02 已卸载并可靠固定、任一可能的指令方向都至少有 60° 无干涉运动空间、使用带限流电源且实体断电开关就绪时使用该配置。使用独立构建目录：

```bash
idf.py -B build-motion \
  -D SDKCONFIG=build-motion/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.motion-test.defaults' \
  set-target esp32s3
idf.py -B build-motion \
  -D SDKCONFIG=build-motion/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.motion-test.defaults' \
  build
```

固件会在停止状态先写入 `run_mode=0`，并读取 `0x7005` 确认，与官方 SampleProgram 一致。运动构建还会在 `0x700B` 写入并确认临时 `1.0 N·m` 电机转矩限制；说明书标明 type 18 写入会在断电后丢失。完成普通的 500 ms 零力矩检查后，运动构建取得最新反馈位置作为原点，在 3 秒内按平滑轨迹移动 60°，稳定 700 ms，再用 3 秒返回，稳定 700 ms，最后发送停止并要求 Reset 模式反馈。每次启动只执行一次。参数为 `Kp=12.0`、`Kd=1.0`、前馈转矩为零，轨迹的理论峰值目标速度约为 `0.524 rad/s`。

运动阶段在以下任一条件发生时中止：速度绝对值达到 `0.8 rad/s`、反馈转矩达到 `1.5 N·m`、估算的发送控制器出力达到 `1.0 N·m`、跟随误差达到 `0.075 rad`、终点误差仍达到 `0.035 rad`、反馈超时、出现故障或致命 TWAI 告警。这些是有延迟的软件观测和估算，不是硬件限制；出现任何异常时立即断开电机电源。

## 受控烧录与验收

烧录时断开电机电源。先启动串口监视器，再有意识地接通带限流的电机电源并复位 ESP32；全程保持电机卸载、实体断电开关就绪。

通信检查成功要求：设备 ID 匹配、母线电压在可信的 24–60 V 范围、`mechPos` 为有限值、两次停止后均为 Reset 模式、主动控制期间处于 Motor 模式、没有故障/预警或致命 TWAI 告警，最后关闭 TWAI。运动测试还要求肉眼确认两个方向均有位移且终点检查通过。出现超时、方向或幅度异常、异常噪声/振动/电流、非法遥测或停止未确认时，立即断开电机电源。

当前代码具有主机测试和固件构建验证，之前的零力矩流程已在一台 RS02 上完成。第一次受控运动尝试使用 `Kp=8.0`、`Kd=0.3` 和 3 秒单程，电机没有在指令作用下转动；随后手动转轴产生 `0.268 rad/s` 反馈，越过当时的 `0.25 rad/s` 阈值并触发了无电机故障、停止已确认的中止。因此这次不算运动验证。后续一次卸载受控测试使用 `Kp=12.0`、`Kd=1.0` 和 5 秒单程，已肉眼确认完成 5° 移动和返回。日志中去程和回程终点误差分别为 `-0.0186 rad` 和 `+0.0065 rad`，观测到的最大速度绝对值、反馈转矩绝对值、估算出力绝对值和温度分别为 `0.193 rad/s`、`0.271 N·m`、`0.457 N·m` 和 `32 °C`，无故障且最终停止已确认。随后两次 30°、8 秒单程尝试都在反馈速度达到 `0.314 rad/s` 和 `0.312 rad/s` 时被同一 `0.25 rad/s` 阈值主动提前停止，中止前去程位移约为 6.7° 和 3.1°，无故障且停止已确认。当前 60°、3 秒单程配置使用 `0.8 rad/s` 速度阈值，随后完成了一次卸载受控往返测试，并已肉眼确认运动。日志中去程和回程终点误差分别为 `-0.0092 rad` 和 `+0.0069 rad`，最大记录速度绝对值、反馈转矩绝对值、估算出力绝对值和温度分别为 `0.790 rad/s`、`0.363 N·m`、`0.556 N·m` 和 `33 °C`，无故障且最终停止已确认。这仅是一次台架实测，不构成机械、热、电气、固件或载人系统安全验收。
