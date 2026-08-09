# 第13章: 达妙 DM 电机实战：DM_CAN.py 与夹爪集成 (DM Motor in Practice: DM_CAN.py & Gripper Integration)
> 原课程: MIT 6.172 — Electric Machines, Fall 2013 (James L. Kirtley Jr.) · 实战补充：达妙 DM 电机驱动与夹爪代码

## 一、本章概要 (Overview)

第 12 章把达妙 DM-J4310-2EC 数据手册拆解为工程知识，建立了 CAN 协议、MIT 控制帧、量化映射与寄存器列表的完整图像。本章处理同一硬件上的软件侧：TARS-DEMO 触觉遥操作系统中实际运行的达妙电机驱动代码与夹爪集成。第 12 章解释"手册规定了什么"，本章回答"驱动代码如何把手册的规定实现为可运行的 Python 程序"。

代码栈分为四层。核心是驱动层 DM_CAN.py，它是协议栈的完整实现：封装了电机对象、控制指令打包、CAN 反馈解析、串口帧收发与参数读写。其上是三个应用层文件：gripper_control.py 与 gripper_pos_control.py 提供夹爪单点定位接口，gripper_teleop.py 与 gripper_teleop_pub.py 是 ROS 遥操作节点，replay.py 从录制数据回放夹爪轨迹。分层架构如下：

```text
┌──────────────────────────────────────────────────────────────────────┐
│ 应用层 (Application)                                                  │
│   gripper_control.py       夹爪定位：Motor + Torque_Pos + move_to_pos │
│   gripper_pos_control.py   夹爪位置控制（含注释掉的备选控制模式）      │
│   gripper_teleop.py        ROS 外的直接遥操作 + 力反馈串口            │
│   gripper_teleop_pub.py    ROS 节点：发布 /gripper_state (JointState) │
│   replay.py                replay 已录制的 data['gripper']['position']│
├──────────────────────────────────────────────────────────────────────┤
│ 驱动层 (Driver / Protocol)                                            │
│   DM_CAN.py   Motor 类        电机状态与 ID 属性                      │
│               MotorControl 类 控制、收发、参数读写、位打包            │
│               模块函数         float_to_uint / uint_to_float / 字节转换│
├──────────────────────────────────────────────────────────────────────┤
│ 传输层 (Transport)                                                    │
│   pyserial → USB-CAN 适配器（921600 bps，30 字节帧）                   │
│   适配器 ↔ 电机（CAN 2.0 标准帧，1 Mbps）                             │
├──────────────────────────────────────────────────────────────────────┤
│ 硬件 (Hardware)                                                       │
│   DM-J4310-2EC：14 对极 PMSM + 10:1 减速箱 + 双磁编码器（14 位）      │
│   驱动板：FOC 电流环 + 速度环 + 位置环（内部固件，本代码不可见）      │
└──────────────────────────────────────────────────────────────────────┘
```

关键点：上位机代码不直接接触三相绕组。它只通过 CAN 向驱动板发送高层指令（期望位置、速度、刚度、转矩），驱动板固件内部完成磁场定向控制（field-oriented control, FOC）并产生 PWM。这一控制层级划分正是第 6 章 dq0 模型与第 9 章 FOC 多环结构的实例化。本章把第 12 章手册中的每一项协议规定对应到具体代码行，并回连到理论章节的转矩表达式与控制环设计。

## 二、物理机制与数学推导 (Physics & Math Derivation)

### 2.1 量化通道：浮点量的定长映射

第 12 章给出了 float_to_uint 与 uint_to_float 的线性映射。DM_CAN.py 中的实现与其一致：设位数 $$\text{bits}$$、映射区间 $$[x_{\min}, x_{\max}]$$，正变换为

$$x_{\text{int}} = \left\lfloor (x - x_{\min})\cdot\frac{2^{\text{bits}} - 1}{x_{\max} - x_{\min}} \right\rceil$$

逆变换为

$$x = x_{\min} + x_{\text{int}}\cdot\frac{x_{\max} - x_{\min}}{2^{\text{bits}} - 1}$$

量化步长为

$$\Delta = \frac{x_{\max} - x_{\min}}{2^{\text{bits}} - 1}$$

往返误差被限定在半个步长以内。这是一个有损的定点通道：把浮点量映射到定长整数后，控制分辨率由位数决定。DM4310 各量的步长见 4.1 节算例。

### 2.2 MIT 模式位打包：64 位装 5 个量

MIT 模式把 5 个浮点量打包进 8 字节：位置 16 位，速度、Kp、Kd、转矩各 12 位，合计 $$16 + 12\times 4 = 64$$ 位。这一结构不是任意的，它与 CAN 标准帧 8 字节数据场直接对应：CAN 2.0 数据场固定 8 字节，任何超过 8 字节的负载都要求拆分多帧。MIT 模式用固定位宽近似浮点控制量，使每一帧携带完整的一次控制给定。

字段拼接的算术如下。设各量量化后为 $$q_{\text{uint}}$$（16 位）、$$dq_{\text{uint}}$$（12 位）、$$kp_{\text{uint}}$$（12 位）、$$kd_{\text{uint}}$$（12 位）、$$\tau_{\text{uint}}$$（12 位），则 8 个字节为

$$D[0] = q_{\text{uint}}[15:8], \qquad D[1] = q_{\text{uint}}[7:0]$$

$$D[2] = dq_{\text{uint}}[11:4]$$

$$D[3] = \bigl(dq_{\text{uint}}[3:0] \ll 4\bigr) \;\text{或}\; kp_{\text{uint}}[11:8]$$

$$D[4] = kp_{\text{uint}}[7:0]$$

$$D[5] = kd_{\text{uint}}[11:4]$$

$$D[6] = \bigl(kd_{\text{uint}}[3:0] \ll 4\bigr) \;\text{或}\; \tau_{\text{uint}}[11:8]$$

$$D[7] = \tau_{\text{uint}}[7:0]$$

其中"或"为按位或。这里的规律：两个 12 位量交界处总是低 4 位移到高半字节、高 4 位留在低半字节，从而把 12 位字段切分为 8 位 + 4 位两块，与相邻字段拼成一个整字节。这一布局与第 12 章 3.4 节的 MIT 控制帧格式逐字节一致。

### 2.3 PD 控制律与二阶动力学

MIT 模式在驱动板内部实现的转矩给定是

$$\tau_{\text{cmd}} = K_p(q_d - q) + K_d(\dot{q}_d - \dot{q}) + \tau_{\text{ff}}$$

其中 $$q_d, \dot{q}_d$$ 为期望位置与期望速度，$$q, \dot{q}$$ 为反馈位置与反馈速度，$$\tau_{\text{ff}}$$ 为前馈转矩。这是关节级比例-微分（PD）控制器加前馈。设转子侧等效惯量为 $$J$$，电机轴动力学为 $$J\ddot{q} = \tau_{\text{cmd}} - \tau_{\text{load}}$$，代入上式并取常量设定点（$$\dot{q}_d = 0$$、$$\tau_{\text{ff}} = 0$$、无负载），误差 $$e = q_d - q$$ 满足

$$J\ddot{e} + K_d\dot{e} + K_p e = 0$$

这是一个二阶系统，自然频率与阻尼比为

$$\omega_n = \sqrt{\frac{K_p}{J}}, \qquad \zeta = \frac{K_d}{2\sqrt{K_p J}}$$

选择 Kp、Kd 的依据是目标刚度与阻尼：Kp 决定定位刚度（误差产生的恢复转矩），Kd 决定振荡抑制。Kp 项产生的恢复转矩与位置误差成正比，其数学结构等同同步电机的同步转矩系数——第 3 章的转矩-角特性 $$T = T_{\max}\sin\delta$$ 在小角度处退化为 $$T \approx T_{\max}\,\delta$$，即转矩正比于角度偏差。二者都是"角度偏差产生恢复转矩"的机制，只是同步电机中该偏差是转子磁极相对定子磁场的方向角，此处是关节位置误差。

转矩给定随后转换为电流。第 6 章给出正弦驱动下表面磁体永磁电机的转矩表达式

$$T_e = \frac{3}{2}p\,\psi_f\,i_q$$

驱动板把转矩给定除以电机侧转矩常数 $$K_{T,m} = \frac{3}{2}p\,\psi_f$$ 得到 q 轴电流指令 $$i_q^{\ast} = \tau_{\text{cmd}} / K_{T,m}$$，再交给电流环。于是控制层级为：主机算 PD 得到转矩 → 驱动板把转矩映射为 i_q 指令 → 电流环调节相电压 → SVPWM 驱动逆变器。这正是第 9 章多环串级控制的结构，位置环在主机侧（PD），电流环在驱动板内部。

### 2.4 力位混控：限位电流的位置控制

夹爪实际使用的力位混控模式（0x300+ID）把位置控制与转矩限制结合：上位机发送期望位置 $$p_{\text{des}}$$、限速值 $$v_{\text{des}}$$ 与电流限定标幺值 $$i_{\text{des}}$$。驱动板内部运行位置环 → 速度环 → 电流环的串级结构，电流指令被限幅到 $$i_{\text{des}}$$。输出转矩上限为

$$T_{\text{out,max}} = g\,\eta_g\,K_{T,m}\cdot i_{\text{des,pu}}\,I_{\max}$$

其中 $$g = 10$$ 为减速比，$$\eta_g$$ 为齿轮效率，$$i_{\text{des,pu}} = i_{\text{des}}/10^4$$ 为电流标幺值，$$I_{\max}$$ 为最大相电流。夹爪逼近目标位置时若遇到物体，位置误差增大、位置环输出增大，但电流被限幅钳制，输出转矩被限制在 $$T_{\text{out,max}}$$，夹持力因此可控。这就是"力位混控"的含义：位置误差驱动运动，电流限幅决定夹持力上限。它对应第 6、9 章的转矩控制概念：在 FOC 下转矩正比于 q 轴电流，限制电流即限制转矩。

### 2.5 量化精度与 14 位编码器

编码器与 CAN 通道两级量化共同决定位置反馈精度。14 位编码器单圈分辨率为 $$2\pi/2^{14} \approx 3.835\times 10^{-4}$$ rad，输出轴经 10:1 减速后为 $$2\pi/(2^{14}\cdot 10) \approx 3.835\times 10^{-5}$$ rad。CAN 位置通道 16 位、映射 $$\pm 12.5$$ rad，步长为

$$\Delta_q = \frac{25}{2^{16}-1} \approx 3.815\times 10^{-4}\ \text{rad}$$

两个数量几乎相等：16 位位置量化步长恰好对应电机侧编码器的一个计数。因此 CAN 位置通道没有浪费编码器分辨率，也没有超过它；而相对输出轴 0.0022° 的分辨率，位置通道的 0.38 mrad（约 0.022°）粗了一个数量级——减速比提供的高分辨率无法穿透量化通道传回主机。这是有损定点通道的固有代价，第 12 章 4.3 节已有同样结论。

## 三、模型与方法精解 (Models & Methods)

### 3.1 Motor 类：状态变量与双 ID 模型

Motor 类封装一台电机的全部软件状态。构造参数为 `Motor(MotorType, SlaveID, MasterID)`，其中 MotorType 是 DM_Motor_Type 枚举索引，用于查 Limit_Param 表。状态变量如下：

```python
self.Pd = float(0)        # 期望位置（驱动侧，代码中未实际使用）
self.Vd = float(0)        # 期望速度（驱动侧，代码中未实际使用）
self.state_q  = float(0)  # 反馈位置（rad）
self.state_dq = float(0)  # 反馈速度（rad/s）
self.state_tau = float(0) # 反馈转矩（N·m）
self.SlaveID = SlaveID    # 电机接收 ID（ESC_ID）
self.MasterID = MasterID  # 反馈 ID（MST_ID）
self.MotorType = MotorType
self.isEnable = False
self.NowControlMode = Control_Type.MIT
self.temp_param_dict = {} # 参数缓存：RID → 值
```

ID 模型是理解整个驱动器的关键。电机有两个 CAN ID：SlaveID 是电机接收 ID（ESC_ID，控制帧发往的地址），MasterID 是反馈 ID（MST_ID，电机回传帧使用的 ID）。在 gripper 文件中二者取 `Motor(DM_Motor_Type.DM4310, 0x01, 0x02)`，即电机接收 ID 0x01、反馈 ID 0x02。addMotor 把两个 ID 都注册到 motors_map：

```python
def addMotor(self, Motor):
    self.motors_map[Motor.SlaveID] = Motor
    if Motor.MasterID != 0:
        self.motors_map[Motor.MasterID] = Motor
    return True
```

由此，反馈帧无论是按 SlaveID 还是按 MasterID 到达，都能在 motors_map 中索引到同一台电机对象，反馈数据因此能写回正确的 state_q/state_dq/state_tau。`recv_data(q, dq, tau)` 写入三个状态量，`getPosition()`、`getVelocity()`、`getTorque()` 读取它们。

### 3.2 MotorControl 初始化：串口与 30 字节帧模板

MotorControl 是驱动器的控制器。构造函数接收已创建的 pyserial 对象（波特率 921600），若串口已打开则先关闭再打开，并初始化 motors_map 与 data_save（残留数据缓冲）：

```python
self.serial_ = serial_device
self.motors_map = dict()
self.data_save = bytes()
if self.serial_.is_open:
    serial_device.close()
self.serial_.open()
```

921600 bps 对应第 12 章手册的"调参接口 UART@921600bps"。USB-CAN 适配器一端通过 UART 与主机通信，另一端通过 CAN（1 Mbps）与电机通信，主机侧串口帧格式与 CAN 帧不同。

发送帧模板 send_data_frame 是 30 字节的定长帧：

```python
send_data_frame = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00,
     0, 0, 0, 0, 0x00, 0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)
```

布局如下：

```text
字节    0     1     2       3..12         13      14      15..17   18     19..20   21..28      29
内容   0x55  0xAA  0x1E  控制/标志字节    CANID_L CANID_H  ...      0x08    ...     CAN 数据场  0x00
       帧头        帧长    （含 DLC=8）   (低8位) (高8位)                     （8 字节）
```

0x55 0xAA 是帧起始标志，0x1E（30）是帧长，字节 13-14 为 CAN 帧 ID（低高字节），字节 18 为 DLC（data length code，固定 8），字节 21-28 为 CAN 数据场。__send_data 只修改这三个位置，其余字节保持不变，直接写入串口：

```python
self.send_data_frame[13] = motor_id & 0xff
self.send_data_frame[14] = (motor_id >> 8) & 0xff  # id high 8 bits
self.send_data_frame[21:29] = data
self.serial_.write(bytes(self.send_data_frame.T))
```

接收侧是 16 字节的短帧（见 3.7 节），帧头 0xAA、帧尾 0x55、命令字节与 CAN ID、8 字节数据场。发送与接收帧长不同，说明适配器对发送与接收使用不同的封装，或发送帧包含更多控制字段。

### 3.3 Limit_Param 表

Limit_Param 按电机类型给出三组映射上限 $$[\text{PMAX}, \text{VMAX}, \text{TMAX}]$$。DM4310（索引 0）取 $$[12.5, 30, 10]$$：位置映射范围 $$\pm 12.5$$ rad、速度 $$\pm 30$$ rad/s、转矩 $$\pm 10$$ N·m。这与第 12 章手册的默认映射范围（位置 ±12.5 rad、速度 ±30 rad/s、转矩 ±10 N·m）及寄存器 0x15/0x16/0x17 一一对应。表内还列出 48V 版本与 DM4340、DM6006、DM8006、DM8009、DM10010、DMH3510、DMH6215、DMG6220 等其他型号的参数。change_limit_param 可在运行期改写这张表，用于更换电机型号或自定义映射范围——手册强调发送端与驱动端的映射范围必须一致，因此改寄存器值时必须同步改代码里的表。

### 3.4 controlMIT：逐字节位打包

controlMIT 是 MIT 模式的发送函数。五个浮点量先量化：

```python
kp_uint = float_to_uint(kp, 0, 500, 12)
kd_uint = float_to_uint(kd, 0, 5, 12)
Q_MAX, DQ_MAX, TAU_MAX = self.Limit_Param[MotorType]
q_uint  = float_to_uint(q, -Q_MAX,  Q_MAX, 16)
dq_uint = float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
```

Kp、Kd 映射区间为单侧 $$[0, 500]$$、$$[0, 5]$$；位置、速度、转矩为对称区间。随后按 2.2 节的位布局逐字节填充：

```python
data_buf[0] = (q_uint >> 8) & 0xff                     # q 高 8 位
data_buf[1] = q_uint & 0xff                            # q 低 8 位
data_buf[2] = dq_uint >> 4                             # dq 高 8 位（12 位的 [11:4]）
data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf)  # dq[3:0] | kp[11:8]
data_buf[4] = kp_uint & 0xff                           # kp 低 8 位
data_buf[5] = kd_uint >> 4                             # kd 高 8 位（12 位的 [11:4]）
data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf) # kd[3:0] | tau[11:8]
data_buf[7] = tau_uint & 0xff                          # tau 低 8 位
```

`dq_uint >> 4` 取 12 位速度值的高 8 位；`dq_uint & 0xf` 取低 4 位并左移 4 位放进 D[3] 高半字节；`kp_uint >> 8` 取 Kp 的高 4 位放进 D[3] 低半字节。D[6] 同理把 Kd 低 4 位与转矩高 4 位拼成一字节。这一布局与第 12 章 3.4 节完全一致，可对照验证。发送后立即调用 recv() 拉取反馈（见 3.7 节），形成"发一帧、收一帧"的同步节奏。

### 3.5 control_Pos_Vel / control_Vel / control_pos_force

三种模式在 CAN ID 上加偏移，并在数据场使用浮点或定点编码。

control_Pos_Vel（位置速度模式，0x100+ID）把期望位置与期望速度各编码为 32 位浮点（小端），占满 8 字节：

```python
motorid = 0x100 + Motor.SlaveID
P_desired_uint8s = float_to_uint8s(P_desired)
V_desired_uint8s = float_to_uint8s(V_desired)
data_buf[0:4] = P_desired_uint8s
data_buf[4:8] = V_desired_uint8s
self.__send_data(motorid, data_buf)
```

float_to_uint8s 用 `struct.pack('f', value)` 把浮点打包为 4 字节（x86 上为小端），再拆成 4 个 uint8。

control_Vel（速度模式，0x200+ID）只发送期望速度的 4 字节浮点，数据场其余 4 字节为零。

control_pos_force（力位混控模式，0x300+ID）是夹爪实际使用的函数：

```python
motorid = 0x300 + Motor.SlaveID
data_buf[0:4] = float_to_uint8s(Pos_des)   # 位置 float32
Vel_uint  = np.uint16(Vel_des)             # 限速 uint16，放大 100 倍
ides_uint = np.uint16(i_des)               # 电流标幺 uint16，放大 10000 倍
data_buf[4] = Vel_uint & 0xff              # v_des 低字节
data_buf[5] = Vel_uint >> 8                # v_des 高字节
data_buf[6] = ides_uint & 0xff             # i_des 低字节
data_buf[7] = ides_uint >> 8               # i_des 高字节
```

数据场为 8 字节：位置 float32 + v_des uint16 + i_des uint16。与第 12 章 3.5 节完全一致：v_des 放大 100 倍（0–10000 对应 0–100 rad/s），i_des 放大 10000 倍（0–10000 对应电流标幺 0–1.0）。第 4 章 4.4 节的算例将结合实际取值解读。

### 3.6 使能 / 失能 / 保存零点

命令帧通过 __control_cmd 发送：数据场为 7 个 0xFF 加上命令码，发送到 SlaveID：

```python
def __control_cmd(self, Motor, cmd):
    data_buf = np.array([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd], np.uint8)
    self.__send_data(Motor.SlaveID, data_buf)
```

命令码对应第 12 章 3.6 节：0xFC 使能、0xFD 失能、0xFE 保存位置零点。enable 在发送后 sleep 0.1 秒并 recv，注释提醒"上电后几秒再使能"。enable_old 兼容旧固件：命令数据场相同，但使能帧的 CAN ID 需要按控制模式加偏移 `((ControlMode-1) << 2) + SlaveID`。save_motor_param 在存储前先 disable，并发送 0xAA 存储命令到广播 ID 0x7FF，符合手册"存储只在失能模式下生效"的规定。

### 3.7 recv 与反馈解码

recv 是反馈解析入口。它把上次未解析完的残留数据与本次读到的串口数据拼接，提取完整帧后逐帧处理：

```python
def recv(self):
    data_recv = b''.join([self.data_save, self.serial_.read_all()])
    packets = self.__extract_packets(data_recv)
    for packet in packets:
        data = packet[7:15]                       # 8 字节 CAN 数据场
        CANID = (packet[6]<<24)|(packet[5]<<16)|(packet[4]<<8)|packet[3]
        CMD = packet[1]
        self.__process_packet(data, CANID, CMD)
```

__extract_packets 扫描 16 字节接收帧：帧首 0xAA、帧尾 0x55、帧长 16。命中则取 data[i:i+16] 为一帧，指针前进 16；未命中则前进 1 字节重新寻找同步。循环结束后把剩余未成帧的字节存入 data_save，留给下一次 recv。这就是残留数据处理：串口按字节流到达，一帧可能在两次 read_all 之间被截断，data_save 保证截断帧不会丢失。

接收帧布局：

```text
字节   0      1      2      3      4      5      6      7..14        15
内容   0xAA  CMD    ...    CANID  CANID  CANID  CANID  8 字节数据场  0x55
       帧头   (0x11)              bit7-0 bit15-8 bit23-16 bit31-24   帧尾
```

CANID 由字节 3-6 以字节 3 为最低字节拼成 32 位（CAN 标准帧实际只用低 11 位）。CMD=0x11 表示数据帧。__process_packet 对 0x11 帧按反馈 ID 路由并解码状态量：

```python
q_uint   = np.uint16((np.uint16(data[1]) << 8) | data[2])
dq_uint  = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
```

位置 16 位取自 data[1]-data[2]，速度 12 位取自 data[3] 与 data[4] 高 4 位，转矩 12 位取自 data[4] 低 4 位与 data[5]，然后按电机类型的 Q_MAX/DQ_MAX/TAU_MAX 用 uint_to_float 反变换为浮点，写回电机对象：

```python
recv_q   = uint_to_float(q_uint,   -Q_MAX,  Q_MAX, 16)
recv_dq  = uint_to_float(dq_uint,  -DQ_MAX, DQ_MAX, 12)
recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
self.motors_map[CANID].recv_data(recv_q, recv_dq, recv_tau)
```

路由分支：CANID 非零时按 CANID 查 motors_map；CANID 为零时（防止有人把 MasterID 设为 0）从 data[0] 的低 4 位读出 MasterID 再查表。两种分支都用 addMotor 注册的双 ID 映射，把反馈路由到对应电机。

值得注意：本代码的位置解码从 data[1] 起读，而第 12 章手册的反馈帧表把 MST_ID 放在 D[0]、状态字节 ID\\|ERR<<4 放在 D[1]、位置在 D[2]-D[3]。代码与手册相差一个字节，原因未查明（可能是手册与固件版本差异）。这不影响解码自洽性：发送与接收使用同一套字节约定，往返一致。

这就是 600 Hz 以上位置、速度、转矩反馈的来源。每次控制指令发送后立即 recv，read_all 为非阻塞读取，把缓冲区内全部完整帧一次性处理。夹爪遥操作循环以约 1 kHz 运行，每个周期发一帧并收一帧，反馈延迟即一个控制周期。由于解析是无状态的，即使某一周期缓冲了多帧，也会在同一个 recv 中全部处理完。

### 3.8 参数读写协议（RID）

参数读写通过广播帧 ID 0x7FF 发送，数据场携带目标电机 ID 与命令字。读参数：

```python
data_buf = np.array([can_id_l, can_id_h, 0x33, RID, 0,0,0,0], np.uint8)
self.__send_data(0x7FF, data_buf)
```

写参数把 0x33 换成 0x55，并把数据按类型编码到 D[4]-D[7]：is_in_ranges(RID) 判定为整型（uint32）时用 data_to_uint8s，否则按 float32 用 float_to_uint8s。is_in_ranges 判断 RID 是否属于整型集合：7–10（MST_ID、ESC_ID、TIMEOUT、CTRL_MODE）、13–16（hw_ver、sw_ver、SN、NPP）、35–36（can_br、sub_ver），与第 12 章寄存器表的类型列一致。

存储命令 0xAA 与状态命令 0xCC 结构相同，仅命令字不同。响应帧在 __process_set_param_packet 中解析：CMD=0x11 且 data[2] 为 0x33 或 0x55 时，从数据场还原从机 ID 与 RID，把 4 字节数据按类型解码后存入 temp_param_dict：

```python
if is_in_ranges(RID):
    num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
else:
    num = uint8s_to_float(data[4], data[5], data[6], data[7])
self.motors_map[masterid].temp_param_dict[RID] = num
```

temp_param_dict 是电机对象的参数缓存。read_motor_param 与 change_motor_param 用重试循环确认写入生效：发请求后以 50 ms 间隔反复调用 recv_set_param_data，最多 20 次，直到缓存中出现目标 RID 且数值与期望之差小于 0.1。switchControlMode 用同样模式写 RID=10（CTRL_MODE）并回读验证，控制模式切换由此获得确认，而不是"发了就算"。save_motor_param 先 disable 再发 0xAA 存储，写入 flash。

### 3.9 枚举：DM_Motor_Type、DM_variable、Control_Type

三个 IntEnum 把整数 ID 映射为可读符号。DM_Motor_Type 覆盖 12 种型号：DM4310、DM4310_48V、DM4340、DM4340_48V、DM6006、DM8006、DM8009、DM10010L、DM10010、DMH3510、DMH6215、DMG6220，索引即 Limit_Param 的行号。DM_variable 列出 RID 0–36 与部分扩展地址：0x07 MST_ID、0x08 ESC_ID、0x0A CTRL_MODE、0x10 NPP、0x11 Rs、0x12 LS、0x13 Flux、0x14 Gr、0x15 PMAX、0x16 VMAX、0x17 TMAX、0x18 I_BW、0x19 KP_ASR、0x1A KI_ASR、0x1B KP_APR、0x1C KI_APR、0x1E GREF、0x23 can_br、0x50 p_m、0x51 xout 等，与第 12 章寄存器表逐项对应。Control_Type 定义控制模式编码：MIT=1、POS_VEL=2、VEL=3、Torque_Pos=4，与第 12 章 0x0A 寄存器 1–4 的编码一致。switchControlMode 传入 Control_Type 枚举，写出的就是该整数。

## 四、推导与算例 (Derivations & Examples)

### 4.1 量化步长算例

DM4310 各量的量化步长（2.1 节公式）：

| 量 | 范围 | 位数 | 步长 |
|---|---|---|---|
| 位置 q | $$\pm 12.5$$ rad | 16 | $$25/65535 \approx 3.815\times 10^{-4}$$ rad |
| 速度 dq | $$\pm 30$$ rad/s | 12 | $$60/4095 \approx 1.465\times 10^{-2}$$ rad/s |
| 转矩 tau | $$\pm 10$$ N·m | 12 | $$20/4095 \approx 4.884\times 10^{-3}$$ N·m |
| Kp | $$[0,500]$$ | 12 | $$500/4095 \approx 1.221\times 10^{-1}$$ |
| Kd | $$[0,5]$$ | 12 | $$5/4095 \approx 1.221\times 10^{-3}$$ |

以速度 dq = 12.0 rad/s 验证往返：

$$dq_{\text{uint}} = \left\lfloor (12.0 + 30)\cdot\frac{4095}{60} \right\rceil = \lfloor 42\times 68.25 \rceil = \lfloor 2866.5 \rceil = 2867$$

$$dq = -30 + 2867\times\frac{60}{4095} = -30 + 42.0000 = 12.0000\ \text{rad/s}$$

往返误差为零（该值恰好落在量化网格上）。位置通道步长与 14 位编码器分辨率同数量级，见 2.5 节。

### 4.2 MIT 打包完整算例

取 q = 1.0 rad、dq = 2.0 rad/s、kp = 20.0、kd = 0.3、tau = 0.5 N·m，逐项量化：

$$q_{\text{uint}} = \left\lfloor (1.0+12.5)\cdot\frac{65535}{25} \right\rceil = \lfloor 13.5\times 2621.4 \rceil = 35389 = \text{0x8A3D}$$

$$dq_{\text{uint}} = \left\lfloor (2.0+30)\cdot\frac{4095}{60} \right\rceil = \lfloor 32\times 68.25 \rceil = 2184 = \text{0x888}$$

$$kp_{\text{uint}} = \left\lfloor 20\cdot\frac{4095}{500} \right\rceil = \lfloor 163.8 \rceil = 164 = \text{0xA4}$$

$$kd_{\text{uint}} = \left\lfloor 0.3\cdot\frac{4095}{5} \right\rceil = \lfloor 245.7 \rceil = 246 = \text{0xF6}$$

$$\tau_{\text{uint}} = \left\lfloor (0.5+10)\cdot\frac{4095}{20} \right\rceil = \lfloor 2149.9 \rceil = 2150 = \text{0x866}$$

按 3.4 节布局填充 8 字节（十六进制）：

```text
字节    0     1     2     3     4     5     6     7
值      0x8A  0x3D  0x88  0x80  0xA4  0x0F  0x68  0x66
字段    q[15:8] q[7:0] dq[11:4] dq[3:0]|kp[11:8] kp[7:0] kd[11:4] kd[3:0]|tau[11:8] tau[7:0]
```

验证 D[3]：dq[3:0] = 0x8 左移 4 位得 0x80，kp[11:8] = 0x0，合 0x80。D[6]：kd[3:0] = 0x6 左移 4 位得 0x60，tau[11:8] = 0x8，合 0x68。接收端反解：dq 从 0x88 与 0x80>>4 拼回 0x888 = 2184，反变换得 2.0 rad/s（精确）；tau 从 (0x68 & 0xF)<<8 \\| 0x66 拼回 0x866 = 2150，反变换得

$$\tau = -10 + 2150\times\frac{20}{4095} = -10 + 10.5006 = 0.5006\ \text{N·m}$$

往返误差约 0.0006 N·m，小于半个转矩步长。这一算例演示了发送端打包与接收端解包的完整逆过程。

### 4.3 应用层：gripper_control.py 与 gripper_pos_control.py

两个文件都初始化同一套硬件配置：

```python
self.motor = Motor(DM_Motor_Type.DM4310, 0x01, 0x02)   # 接收 ID 0x01，反馈 ID 0x02
motor_serial = serial.Serial(MOTOR_SERIAL_PORT, 921600, timeout=0.5)
self.motorctrl = MotorControl(motor_serial)
self.motorctrl.addMotor(self.motor)
```

MOTOR_SERIAL_PORT 为 /dev/ttyACM0（USB-CAN 适配器在 Linux 下枚举为 ACM 设备）。初始化流程为 switchControlMode(Torque_Pos) 与 enable。gripper_control.py 先切模式再使能；gripper_pos_control.py 先使能再切模式，并注释掉备选模式（POS_VEL）。两种顺序手册均允许，模式切换写寄存器、使能命令独立，先后不产生冲突。

move_to_pos 调用 control_pos_force：

```python
self.motorctrl.control_pos_force(self.motor, pos, 9900.0, 500.0)
```

三个参数的物理含义按 2.4 节：pos 为期望位置（rad，输出轴）；v_des = 9900 放大 100 倍，即限速 99 rad/s，对夹爪而言远大于实际可达速度，等效于不限制速度；i_des = 500 放大 10000 倍，即电流标幺 0.05，把夹持力限制在最大电流的 5%。备选注释 `controlMIT(self.motor, pos, 0.3, 0, 0, 0)` 显示同一位置指令也可用 MIT 模式发出，但参数顺序是 (kp, kd, q, dq, tau)，该行把位置误填到 kp 位置，仅作演示，未启用。

### 4.4 遥操作：人手指位置到夹爪宽度命令

gripper_teleop.py 与 gripper_teleop_pub.py 用外接串口读取操作者的手指位置传感器，映射为夹爪位置命令。传感器数值经 normalize_value 线性映射：

$$q = \frac{v - v_{\min}}{v_{\max} - v_{\min}}\cdot(q_{\max} - q_{\min}) + q_{\min}$$

gripper_teleop.py 中 $$[v_{\min}, v_{\max}] = [2640, 2690]$$，$$[q_{\min}, q_{\max}] = [-4.8, 0.0]$$ rad。例如传感器读数 2660：

$$q = \frac{2660-2640}{2690-2640}\times(0.0-(-4.8)) + 0.0 = 0.4\times(-4.8) = -1.92\ \text{rad}$$

即夹爪张开 1.92 rad。控制循环以约 1 kHz 运行（time.sleep(0.001)），每个周期读外接串口一行、发送 control_pos_force、读取电机反馈并打印。转矩反馈被归一化到 2–9 并写回外接串口，作为给操作者的力反馈。

gripper_teleop_pub.py 在相同循环中加入 ROS：初始化节点 gripper_teleop_node，发布 /gripper_state（sensor_msgs/JointState），把位置、速度、转矩填入 joint1 的 position、velocity、effort 字段，供上层触觉系统订阅。循环内无显式 sleep，实际发布速率受串口读写与同步 recv 限制，可达数百 Hz 至 1 kHz 量级。反馈路径：驱动板在每次收到控制帧后回传状态帧，recv 在同一周期内完成解码，因此主机侧看到的 feedback 即上一控制周期的电机状态。

### 4.5 回放：replay.py 的夹爪驱动段

replay.py 的 gripper_position 函数（第 160 行起）从录制数据回放夹爪轨迹。流程：

```python
data = load_pkl_data(save_path)          # 读 state.pkl 与 gripper.pkl
gripper_positions = data['gripper']['position']   # 录制的位置序列（rad）
```

随后以与 4.3 节相同的初始化建立电机连接（串口为 /dev/ttyACM1），switchControlMode(Torque_Pos) 成功后 enable，再逐点回放：

```python
for i in range(min_length):
    if stop_flag:
        break
    if gripper_positions[i] is not None:
        MotorControl1.control_pos_force(Motor1, gripper_positions[i], 9900.0, 500.0)
    time.sleep(0.001)
```

第 181 行 `Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x02)` 与 4.3 节相同的双 ID 配置。回放速度由 1 ms sleep 近似录制采样周期；位置跟踪由驱动板内部位置环完成，主机只重放位置序列。stop_flag 由 KeyboardInterrupt 置位，保证退出时能先失能。gripper_position 被设计为与机械臂线程并行运行的线程（xarm 线程回放机械臂轨迹，gripper 线程回放夹爪轨迹），main 中 gripper 线程默认注释掉，仅回放机械臂。

### 4.6 夹持力估计

i_des = 500 对应电流标幺 0.05。若最大相电流 $$I_{\max} = 10$$ A（遥操作注释给出），电机侧转矩常数按第 12 章约 $$K_{T,m} = 0.12$$ N·m/A，电机侧转矩上限

$$T_{m} = K_{T,m}\times 0.05\times 10 = 0.06\ \text{N·m}$$

输出轴经 10:1 减速（$$\eta_g = 1$$）为约 0.6 N·m。夹持力因此被限制在一个安全的量级，接触物体时不会过驱。这一估计依赖 Imax 与 K_{T,m} 两个假设，实际以驱动板上电打印的最大电流为准。

## 五、核心 Takeaway (Takeaways)

1. 驱动代码是协议的手工实现。DM_CAN.py 逐字节实现了第 12 章手册的 MIT 帧、反馈帧、命令帧与参数寄存器协议：量化映射、位打包、双 ID 路由、残留数据缓冲，每一项都可回溯到手册的具体条款。

2. 双 ID 模型是单电机的完整寻址方式。SlaveID（ESC_ID）是控制帧目标，MasterID（MST_ID）是反馈帧来源，addMotor 把两者注册到同一电机对象，使控制与反馈按同一对象聚合。夹爪取 ESC_ID=0x01、MST_ID=0x02。

3. MIT 模式是关节级 PD 加前馈，控制层级明确：主机算 PD 得到转矩 $$\tau = K_p(q_d-q)+K_d(\dot{q}_d-\dot{q})+\tau_{\text{ff}}$$，驱动板把转矩映射为 q 轴电流（第 6 章 $$T_e = \frac{3}{2}p\psi_f i_q$$），内部电流环完成 FOC。第 9 章的多环串级结构在此实例化。

4. 力位混控用电流限幅实现夹持力控制。位置误差驱动运动，i_des 限幅把输出转矩钳制在 $$T_{\text{out,max}} = g\,\eta_g\,K_{T,m}\,i_{\text{des,pu}}\,I_{\max}$$，这是夹爪在接触物体时不过驱的机制。

5. 量化通道有明确精度代价。16 位位置步长与 14 位电机侧编码器分辨率一致，不浪费也不超过编码器；输出轴经减速的更高分辨率无法穿透量化通道。反馈帧解码与手册表格相差一字节，原因未查明，发送与接收自洽因此功能不受影响。

## 六、练习与思考 (Exercises)

1. 写出 float_to_uint 的完整表达式，并计算 DM4310 位置通道中 q = -3.0 rad 对应的整数与反变换值。若把映射范围改为 $$\pm 6.25$$ rad，同一位置的量化步长与往返误差如何变化？

2. 手工完成一次 MIT 打包：q = 2.5 rad、dq = -5.0 rad/s、kp = 100、kd = 1.0、tau = 2.0 N·m，给出 8 个字节的十六进制值，并验证 D[3] 与 D[6] 的半字节拼接。

3. 解释 addMotor 的双 ID 注册如何使反馈路由正确。若 MasterID 设为 0，recv 的哪个分支接管，为什么？

4. control_pos_force 中 v_des 与 i_des 各放大多少倍？gripper 传入 9900.0 与 500.0，对应限速与电流标幺各是多少？若需把夹持力上限提高一倍，i_des 应设为多少？

5. 力位混控的转矩上限公式 $$T_{\text{out,max}} = g\,\eta_g\,K_{T,m}\,i_{\text{des,pu}}\,I_{\max}$$ 中每一项的来源分别是什么？若减速比改为 5，其他不变，夹持力上限如何变化？

6. 反馈解析中位置、速度、转矩分别从哪些字节解码？若某反馈帧 data[1]=0x8A、data[2]=0x3D，位置对应多少 rad（$$\pm 12.5$$ rad 映射）？与 4.2 算例的关系是什么？

7. 比较 gripper_control.py 与 gripper_pos_control.py 的初始化顺序差异，分析先使能后切模式与先切模式后使能在驱动板内部的差异（可参考 3.8 节参数写确认机制）。

8. replay.py 用 1 ms sleep 近似录制采样周期回放位置。若录制采样率与回放速率不一致，轨迹时序会如何偏差？若要严格按时间戳回放，应如何修改循环？

---

<!-- chapter-nav -->
<div style="display:flex; justify-content:space-between; align-items:center; padding:1em 0;">
  <div><a href="ch12_电机数据手册实战_DM-J4310-2EC.md">← 第12章 电机数据手册实战：DM-J4310-2EC</a></div>
  <div><a href="index.md">↑ 目录</a></div>
  <div></div>
</div>
