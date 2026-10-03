# HERIC

**High-Performance Embodied Real-Time Integrated Control Platform**  
**具身智能高性能实时一体化控制平台**

HERIC 是面向具身智能机器人构建的高性能、强实时、感知—决策—规划—控制—执行一体化软件平台。

HERIC 的目标不是将 ROS 2、Isaac ROS、实时控制和工业现场总线简单组合，而是建立一套具有明确实时边界和功能分层的机器人控制体系：上层负责感知、智能决策与任务规划，下层负责确定性的运动控制和机器人执行，通过高性能 IPC 将智能域与实时域解耦，从而兼顾具身智能算法能力与工业机器人所要求的高频、低抖动、高可靠实时控制。

---

## 1. 项目定位

具身智能机器人需要形成完整的闭环：

```text
Perception
    ↓
Decision / Reasoning
    ↓
Motion Planning
    ↓
Real-Time Control
    ↓
Execution
    ↓
Environment
    └────────────→ Perception
```

HERIC 将“具身智能”作为上位概念，其中涵盖：

- 感知 Perception
- 决策与推理 Decision / Reasoning
- 任务与运动规划 Planning
- 实时运动控制 Real-Time Motion Control
- 机器人执行 Execution

HERIC 着重解决的并不是某一个单独算法，而是如何将这些能力组织为一个**高性能、实时、一体化的机器人闭环系统**。

平台的三个核心技术属性为：

**High Performance · Real Time · Integration**

即：

```text
高性能
+
强实时
+
感知决策控制一体化
```

---

# 2. 设计目标

HERIC 面向工业机器人、具身智能机器人以及高动态智能装备，重点实现以下目标。

### 高性能智能计算

充分利用 CPU、GPU、DLA 等异构计算资源，支持：

```text
Computer Vision
Visual SLAM
Point Cloud Processing
Deep Learning
TensorRT
Foundation Models
Reinforcement Learning
```

并允许 Isaac ROS 等 GPU 加速框架直接工作在智能计算层。

### 强实时机器人控制

建立独立于 ROS 2 Executor 和 DDS 调度的实时控制域，使底层运动控制能够达到：

```text
1 kHz
2 kHz
甚至更高频率
```

典型目标：

```text
Control Period = 500 μs
Control Frequency = 2 kHz
```

实时系统重点考核：

```text
Worst-case latency
Maximum jitter
P99.9 / P99.99 latency
Deadline miss
DC synchronization error
```

而不仅仅关注平均运行时间。

### 感知—决策—控制一体化

建立从：

```text
Camera / LiDAR / Sensor
          ↓
      Perception
          ↓
    AI / Decision
          ↓
 Motion Planning
          ↓
Real-Time Motion
          ↓
        Robot
```

的统一软件架构。

### 实时域与智能域隔离

ROS 2、Isaac ROS、MoveIt、AI 推理甚至整个非实时进程异常时，不允许直接破坏底层实时控制周期。

HERIC 的基本原则是：

> ROS 2 负责智能与系统集成，实时控制核心负责确定性。

---

# 3. 总体架构

HERIC 采用“智能域 + 实时域 + 执行域”的分层架构。

```text
                         HERIC
        High-Performance Embodied Real-Time
             Integrated Control Platform


┌───────────────────────────────────────────────┐
│              Embodied Intelligence            │
│                                               │
│  Isaac ROS / Vision / VSLAM / AI / Learning  │
│                     │                         │
│                     ▼                         │
│          Decision / Task Planning             │
│                     │                         │
│                     ▼                         │
│            MoveIt / Motion Planning           │
│                     │                         │
│                     ▼                         │
│                   ROS 2                       │
│                     │                         │
│              ros2_control                     │
│                （optional）                   │
└─────────────────────┬─────────────────────────┘
                      │
                      ▼
               HERIC Gateway
                      │
              Command / State
                      │
              iceoryx2 / SHM
                      │
════════════════ Real-Time Boundary ════════════════
                      │
                      ▼
                 HERIC RT Core
                      │
         ┌────────────┼────────────┐
         │            │            │
      Ruckig       Control       Safety
    Trajectory     Algorithm     Runtime
         │            │            │
         └────────────┼────────────┘
                      │
                      ▼
               ECAT EnableKit
                      │
                      ▼
                     IgH
                      │
                      ▼
             Industrial Fieldbus
                      │
                      ▼
                  Servo Drive
                      │
                      ▼
                    Robot
                      │
                      ▼
                 Environment
                      │
                      └────────→ Perception
```

---

# 4. 软件分层

## 4.1 Embodied Intelligence Layer

这一层承担具身智能算法和上层机器人功能，包括：

```text
Isaac ROS
ROS 2
MoveIt
Vision
VSLAM
AI
Reinforcement Learning
Task Planning
Trajectory Planning
HMI
Digital Twin
```

这一层属于非硬实时域。

其运行频率根据任务不同通常为：

```text
Vision                  30 ~ 60 Hz
AI / RL                  50 ~ 200 Hz
Motion Planning          20 ~ 100 Hz
ROS State               100 ~ 500 Hz
```

这一层不直接承担 EtherCAT 500 μs 的实时周期责任。

---

# 5. HERIC Gateway

Gateway 是智能域与实时域之间的重要边界。

主要职责：

```text
ROS Message
      ↕
HERIC Internal Data Model
      ↕
Real-Time IPC
```

例如 ROS 侧接收：

```text
trajectory_msgs/JointTrajectory
sensor_msgs/JointState
FollowJointTrajectory
AI Policy Command
Target Pose
```

然后转换为固定大小的实时数据结构。

例如：

```cpp
struct RobotCommand
{
    uint64_t sequence;
    uint64_t timestamp_ns;

    double target_position[7];
    double target_velocity[7];
    double target_acceleration[7];

    uint32_t control_mode;
};
```

反方向：

```cpp
struct RobotState
{
    uint64_t sequence;
    uint64_t timestamp_ns;

    double position[7];
    double velocity[7];
    double torque[7];

    uint16_t status_word[7];

    uint32_t robot_state;
    uint32_t ethercat_state;

    int32_t dc_error_ns;
};
```

Gateway 本身无需以 2 kHz 运行。

推荐：

```text
100 ~ 500 Hz
```

---

# 6. IPC Architecture

智能域和实时域之间原则上通过共享内存 IPC 通信。

第一阶段推荐：

```text
SPSC Shared Memory Ring Buffer
```

其优势是：

```text
结构简单
运行确定
容易分析 Worst-case latency
方便验证 2 kHz 实时性
```

随着系统复杂度增加，可进一步引入：

```text
iceoryx2
```

典型场景：

```text
                     HERIC RT Core
                           │
                           ▼
                    iceoryx2 Publisher
                           │
          ┌────────────────┼───────────────┐
          ▼                ▼               ▼
      ROS Gateway       AI Policy       Logger
          │                │               │
          ▼                ▼               ▼
        ROS 2        Intelligent Ctrl     Data
```

iceoryx2 主要解决：

```text
Zero-copy IPC
Multi-process communication
Publish / Subscribe
Multiple consumers
Low-latency shared memory communication
```

但 iceoryx2 不属于 HERIC 的强制依赖。

对于：

```text
ROS Gateway ↔ RT Controller
```

这种简单双进程架构，一个 SPSC 共享内存已经足够。

---

# 7. HERIC RT Core

HERIC RT Core 是整个平台的实时核心。

它必须独立于：

```text
ROS Executor
DDS
Python
MQTT
GUI
Database
Logging
```

运行。

典型实时周期：

```text
2 kHz
500 μs
```

核心循环：

```cpp
while (running)
{
    wait_next_period();

    ethercat_receive();

    read_robot_state();

    update_latest_command();

    trajectory_update();

    robot_control();

    safety_check();

    write_robot_command();

    ethercat_send();

    realtime_telemetry();
}
```

RT Core 内部主要包含：

```text
Scheduler
EtherCAT IO
Trajectory Generator
Robot Controller
Safety Runtime
Watchdog
Diagnostics
Realtime Telemetry
```

---

# 8. Ruckig Motion Layer

Ruckig 用于在线实时轨迹生成。

Ruckig 并不等同于 Robot Controller。

其职责为：

```text
Low-frequency Command
          ↓
      Ruckig OTG
          ↓
Continuous Position
Continuous Velocity
Continuous Acceleration
Jerk-limited Motion
          ↓
      2 kHz Target
```

对于 2 kHz 控制周期：

```cpp
Ruckig<7> otg(0.0005);
```

每个周期得到：

```text
q_des
dq_des
ddq_des
```

之后再交给真正的机器人控制器。

---

# 9. Robot Control

Robot Control 位于 Ruckig 之后。

基本结构：

```text
Ruckig
   ↓
q_des
dq_des
ddq_des
   ↓
Robot Controller
   ↓
position / velocity / torque command
```

根据机器人能力可以逐步支持：

```text
Position Control
Velocity Control
Torque Control
Computed Torque Control
Gravity Compensation
Friction Compensation
Impedance Control
Admittance Control
Force-Position Control
Model-Based Control
Learning-Based Control
```

进一步可支持：

```text
Physics-Informed Learning
Adaptive Control
Safe Reinforcement Learning
Variable Impedance
Sensorless Force Estimation
```

---

# 10. Safety Runtime

Safety 必须是实时控制核心的独立组成部分，而不能依赖 ROS 2。

主要包括：

```text
Joint position limit
Joint velocity limit
Joint acceleration limit
Joint torque limit
Following error
Trajectory timeout
Command timeout
Communication watchdog
EtherCAT Working Counter
Distributed Clock error
Servo fault
Emergency state
```

典型 command watchdog：

```text
command age < T1
        ↓
NORMAL

T1 < command age < T2
        ↓
HOLD / CONTROLLED DECELERATION

command age > T2
        ↓
SAFE STOP
```

因此即使：

```text
ROS crashes
Isaac ROS crashes
Gateway crashes
AI crashes
```

RT Core 仍然能够独立处理机器人安全状态。

---

# 11. EtherCAT Layer

HERIC 当前底层工业实时通信采用：

```text
ECAT EnableKit
       ↓
      IgH
       ↓
ec_master
       ↓
ec_igb
       ↓
EtherCAT
```

但 EtherCAT 只是当前实现方式，并不是 HERIC 顶层架构定义的一部分。

未来 HERIC 可以扩展：

```text
EtherCAT
TSN
CAN-FD
EtherNet/IP
PROFINET
Shared-memory device interface
Custom real-time buses
```

因此 Fieldbus 位于 Hardware Abstraction Layer 之下。

---

# 12. ECAT EnableKit

ECAT EnableKit 负责提供 IgH 之上的工业 EtherCAT 配置与抽象。

主要功能包括：

```text
ESI
ENI
Slave Configuration
PDO Mapping
Distributed Clock
Process Image
EtherCAT Device Abstraction
```

目标是逐渐避免大量：

```text
hard-coded PDO offset
hard-coded slave configuration
```

并实现：

```text
ESI
 +
Topology
 +
PDO selection
 +
DC configuration
      ↓
ENI
      ↓
Runtime Configuration
```

---

# 13. IgH EtherCAT Master

IgH 为当前实时现场总线核心。

典型通信路径：

```text
HERIC RT Core
      ↓
ECAT EnableKit
      ↓
libethercat / ecrt API
      ↓
ec_master
      ↓
ec_igb
      ↓
NIC
      ↓
EtherCAT
```

通过 Native EtherCAT Driver 避免传统 Linux Socket 网络路径，降低 EtherCAT 通信抖动。

---

# 14. Distributed Clock

HERIC 的 EtherCAT 执行层需要结合 Distributed Clock。

当前同步结构为：

```text
CLOCK_MONOTONIC
       ↓
Application Time
       ↓
EtherCAT Master
       ↓
DC Reference Clock
       ↓
Other EtherCAT Slaves
```

目标周期：

```text
SYNC0 = 500 μs
```

即：

```text
2 kHz
```

需要持续监控：

```text
DC offset
DC drift
SYNC0 jitter
Master cycle jitter
Slave synchronization
```

---

# 15. ROS 2 Integration

ROS 2 是 HERIC 的上层机器人软件生态，不承担硬实时 Servo Loop。

推荐架构：

```text
ROS 2
  ↓
HERIC Gateway
  ↓
IPC
══════════════
RT Core
  ↓
Robot
```

而不是：

```text
ROS Timer
  ↓
controller_manager
  ↓
500 μs
  ↓
EtherCAT
```

HERIC 的基本理念是：

> 不让 ROS 2 支撑 2 kHz，而让实时内核支撑 2 kHz，然后让 ROS 2 使用该实时内核。

---

# 16. ros2_control Integration

ros2_control 可以作为 HERIC 的 ROS 标准兼容层。

推荐：

```text
MoveIt
  ↓
ros2_control
  ↓
HERIC HardwareInterface
  ↓
HERIC Gateway
  ↓
IPC
════════════════
HERIC RT Core
```

HardwareInterface：

```text
read()
```

读取最新 RobotState。

```text
write()
```

更新最新 RobotCommand。

但它不直接执行：

```text
ecrt_master_receive()
ecrt_master_send()
```

真正 2 kHz EtherCAT 循环由 HERIC RT Core 独立管理。

---

# 17. Isaac ROS Integration

Isaac ROS 位于 HERIC 的智能感知层。

典型路径：

```text
Camera
   ↓
Isaac ROS
   ↓
CUDA / GPU
   ↓
Vision / VSLAM / Detection
   ↓
Object / Robot / Environment State
   ↓
Planning / AI
   ↓
HERIC Gateway
```

Isaac ROS 内部 GPU 数据链保持 NVIDIA 原生的数据传输体系。

iceoryx2 主要用于：

```text
RT Core ↔ Non-RT processes
```

而不是取代 Isaac ROS GPU pipeline。

---

# 18. AI / RL Integration

AI 不应在初期承担每 500 μs 必须完成一次的硬实时责任。

推荐：

```text
AI / RL
50 ~ 200 Hz
      ↓
High-Level Control Command
      ↓
HERIC RT Core
2 kHz
      ↓
Real-Time Interpolation
      ↓
Safety
      ↓
Servo
```

AI 可以输出：

```text
Trajectory correction
Force target
Motion primitive
Impedance parameters
Admittance parameters
ΔM
ΔB
ΔK
```

RT Core 则保证：

```text
deterministic execution
constraint enforcement
safe interpolation
hardware synchronization
```

---

# 19. Frequency Hierarchy

HERIC 不要求所有组件运行在同一个频率。

推荐多频率体系：

```text
Vision
30 ~ 60 Hz

Task / AI Decision
20 ~ 200 Hz

Motion Planning
20 ~ 100 Hz

ROS / Gateway
100 ~ 500 Hz

Joint State to ROS
250 ~ 500 Hz

Robot RT Control
1000 ~ 2000 Hz

Ruckig
1000 ~ 2000 Hz

EtherCAT
1000 ~ 2000 Hz

Servo SYNC0
500 μs @ 2 kHz
```

因此：

```text
High-level intelligence
        ↓
low-frequency intention
        ↓
real-time interpolation
        ↓
high-frequency execution
```

---

# 20. Real-Time CPU Architecture

建议将 CPU 资源按照实时性划分。

概念布局：

```text
CPU 0
Linux housekeeping

CPU 1
System / IRQ

CPU 2
EtherCAT + RT Loop

CPU 3
Robot Control / Motion

CPU 4
Gateway

CPU 5
ROS 2 Executor

CPU 6+
AI / Isaac ROS supporting threads

GPU
Vision / TensorRT / AI

DLA
Optional inference
```

实际分配根据具体 SoC topology 和 benchmark 决定。

---

# 21. Real-Time Optimization

HERIC 实时运行环境应逐步实现：

```text
PREEMPT_RT
SCHED_FIFO
CPU affinity
CPU isolation
IRQ affinity
Memory locking
Prefaulting
Avoid runtime allocation
Absolute-time sleep
Fixed-size data structure
Lock-free IPC
No logging in critical RT path
No blocking IO
```

实时循环原则：

```text
No malloc
No printf
No disk IO
No DDS
No Python
No network service
No uncontrolled mutex
```

---

# 22. Performance Evaluation

HERIC 不以“平均运行时间”作为主要实时指标。

重点关注：

```text
Minimum
Average
P99
P99.9
P99.99
Maximum
Deadline Miss
```

测试过程采用逐层加入组件的方法。

```text
IgH only
   ↓
+ ECAT EnableKit
   ↓
+ Ruckig
   ↓
+ Control
   ↓
+ IPC
   ↓
+ iceoryx2
   ↓
+ ROS2
   ↓
+ MoveIt
   ↓
+ Isaac ROS
   ↓
+ TensorRT / GPU load
```

每增加一层都重新测试：

```text
RT loop jitter
DC synchronization
deadline miss
CPU usage
memory behavior
```

这样能够准确定位实时性能退化来源。

---

# 23. Failure Isolation

HERIC 必须满足：

```text
ROS2 crash
      ×
RT Controller crash
```

即二者不存在必然关系。

最终目标：

```text
kill ROS 2
kill Isaac ROS
kill Gateway
kill Logger
kill AI
```

后：

```text
2 kHz RT Core
```

仍持续运行，并根据 command watchdog 自动进入：

```text
Hold
Controlled Deceleration
Safe Stop
```

这是 HERIC 区别于普通 ROS 机器人控制系统的重要特征之一。

---

# 24. Recommended Repository Structure

```text
heric/
│
├── docs/
│
├── configs/
│
├── ethercat/
│   ├── esi/
│   ├── eni/
│   ├── enablekit/
│   └── drivers/
│
├── core/
│   ├── scheduler/
│   ├── realtime/
│   ├── motion/
│   ├── control/
│   ├── safety/
│   ├── watchdog/
│   └── diagnostics/
│
├── ipc/
│   ├── common/
│   ├── spsc/
│   └── iceoryx2/
│
├── gateway/
│   └── ros2/
│
├── ros2_control/
│
├── isaac_ros/
│
├── ai/
│
├── benchmarks/
│   ├── cyclic/
│   ├── ethercat/
│   ├── dc/
│   ├── ipc/
│   └── gpu_interference/
│
├── tests/
│
├── scripts/
│
└── third_party/
```

---

# 25. Development Roadmap

HERIC 建议按照逐层验证的方式开发。

```text
Phase 1
IgH 2 kHz Baseline
      ↓
Phase 2
PREEMPT_RT Optimization
      ↓
Phase 3
ECAT EnableKit
      ↓
Phase 4
ESI / ENI
      ↓
Phase 5
Distributed Clock
      ↓
Phase 6
Ruckig
      ↓
Phase 7
HERIC RT Core
      ↓
Phase 8
Shared-memory IPC
      ↓
Phase 9
HERIC ROS2 Gateway
      ↓
Phase 10
ros2_control / MoveIt
      ↓
Phase 11
Isaac ROS
      ↓
Phase 12
iceoryx2 if necessary
      ↓
Phase 13
AI / RL
      ↓
Phase 14
Stress Test
      ↓
Phase 15
Safety Validation
```

最重要的工程原则是：

> 每加入一个新的软件层，都重新执行一次实时性能 benchmark。

不能等到所有组件全部集成以后再分析 jitter。

---

# 26. Core Design Principles

HERIC 遵循以下核心原则。

### Intelligence and Real-Time Separation

```text
Intelligence Domain
        ↓
Real-Time Boundary
        ↓
Deterministic Control Domain
```

智能域可以复杂。

实时域必须简单、可预测、可验证。

### High-Level Intent, Low-Level Determinism

上层提供：

```text
What to do
Where to go
How to interact
```

实时层负责：

```text
What happens in the next 500 μs
```

### ROS Is an Interface, Not the Servo Kernel

ROS 2 是机器人生态和系统集成框架。

HERIC RT Core 才是底层实时运动控制核心。

### AI Must Not Break Determinism

AI 推理可以改变：

```text
trajectory
control parameters
interaction strategy
```

但不能破坏：

```text
hard deadline
safety constraints
servo synchronization
```

### Fieldbus Is Replaceable

EtherCAT 是当前高性能执行接口，而不是 HERIC 的平台定义。

HERIC 的核心价值位于：

```text
Embodied Intelligence
          +
Real-Time Runtime
          +
Integrated Control Architecture
```

而不是某一种现场总线。

---

# 27. Vision

HERIC 最终希望构建一套从智能感知到高实时物理执行的统一机器人控制基础设施：

```text
                    HERIC

              Embodied Intelligence
                      │
         ┌────────────┼────────────┐
         │            │            │
     Perception    Decision     Planning
         │            │            │
         └────────────┼────────────┘
                      ▼
                 Motion Intent
                      │
══════════════ Real-Time Boundary ══════════════
                      │
                 HERIC RT Core
                      │
         ┌────────────┼────────────┐
         │            │            │
     Trajectory    Control       Safety
         │            │            │
         └────────────┼────────────┘
                      ▼
              Hardware Abstraction
                      │
                      ▼
                  Execution
                      │
                      ▼
                  Environment
                      │
                      └──────────→ Perception
```

HERIC 的最终目标不是构建一个单纯的 EtherCAT 控制器，也不是构建一个单纯的 ROS 2 软件包，而是形成：

> **面向具身智能机器人的高性能实时一体化控制基础平台。**

通过统一感知、决策、规划、运动控制和物理执行，在保持现代 AI/机器人软件生态开放性的同时，为机器人提供可验证、可扩展、确定性的高频实时控制能力。

---

**HERIC**  
*High-Performance Embodied Real-Time Integrated Control Platform*

**具身智能高性能实时一体化控制平台**
