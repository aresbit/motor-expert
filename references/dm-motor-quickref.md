# DM 电机速查表 (DM Motor Quick Reference)

> 依据：DM-J4310-2EC 官方手册、`DM_CAN.py`、本技能内置书 `references/electric-book/ch12_电机数据手册实战_DM-J4310-2EC.md` 与 `references/electric-book/ch13_达妙DM电机实战_DM_CAN.py与夹爪集成.md`。

## 常用机型与限位参数

`DM_CAN.py` 中 `Limit_Param` 的格式为 `[PMAX, VMAX, TMAX]`（位置 rad、速度 rad/s、力矩 Nm，映射范围）。

| 机型 | 索引 | PMAX | VMAX | TMAX | 备注 |
|------|------|------|------|------|------|
| DM4310 | 0 | 12.5 | 30 | 10 | 夹爪常用（无减速） |
| DM4310_48V | 1 | 12.5 | 50 | 10 | |
| DM4340 | 2 | 12.5 | 8 | 28 | |
| DM-J4310-2EC | — | 12.5 | 30 | 10 | 减速电机，齿比 10:1，额定 3 Nm/峰值 7 Nm |

DM-J4310-2EC 关键参数：额定电压 24/48V、额定相电流 2.5A、峰值 7.5A、额定转矩 3 Nm/峰值 7 Nm、额定转速 120 rpm、空载最大 200/400 rpm、减速比 10:1、极对数 14、相电感 340 µH、相电阻 0.65 Ω、槽数 24、编码器 14 位磁编单圈×2、CAN@1Mbps、UART@921600。

## 通信链路

```
主机 Python (DM_CAN.py) ──UART 921600──▶ USB-CAN 适配器 ──CAN 1Mbps──▶ DM 电机驱动器
```

- 串口帧：30 字节模板（`send_data_frame`），内含 8 字节 CAN 数据 + CAN ID（字节 13-14）。
- 接收帧：16 字节，`0xAA` 头 + `0x55` 尾，`frame_length = 16`。
- 反馈频率可达 600Hz+（每发一帧控制命令即回读一帧反馈）。

## CAN 控制模式（帧 ID 偏移）

| 模式 | 帧 ID | 数据内容 |
|------|-------|---------|
| MIT | 设定 CAN ID | p_des(16) + v_des(12) + Kp(12) + Kd(12) + t_ff(12)，共 8 字节 |
| 位置速度 | 0x100 + ID | p_des(float32) + v_des(float32) |
| 速度 | 0x200 + ID | v_des(float32) |
| 力位混控 | 0x300 + ID | p_des(float32) + v_des(uint16) + i_des(uint16) |

`Control_Type` 枚举：`MIT=1, POS_VEL=2, VEL=3, Torque_Pos=4`。

### MIT 模式 8 字节位布局

| 字节 | 内容 |
|------|------|
| D[0] | p_des[15:8] |
| D[1] | p_des[7:0] |
| D[2] | v_des[11:4] |
| D[3] | v_des[3:0] \| Kp[11:8] |
| D[4] | Kp[7:0] |
| D[5] | Kd[11:4] |
| D[6] | Kd[3:0] \| t_ff[11:8] |
| D[7] | t_ff[7:0] |

对应 `controlMIT`：`float_to_uint(kp, 0, 500, 12)`、`float_to_uint(kd, 0, 5, 12)`、`float_to_uint(q, -Q_MAX, Q_MAX, 16)`、`float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)`、`float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)`。

### 反馈帧（8 字节）

| 字节 | 内容 |
|------|------|
| D[0] | MST_ID \| ID \| ERR<<4 |
| D[1] | POS[15:8] |
| D[2] | POS[7:0] |
| D[3] | VEL[11:4] |
| D[4] | VEL[3:0] \| T[11:8] |
| D[5] | T[7:0] |
| D[6] | T_MOS（MOS 平均温度 ℃） |
| D[7] | T_Rotor（线圈平均温度 ℃） |

## 错误码（ERR，反馈帧 D[0] 高 4 位）

| ERR | 含义 | 常见根因 |
|-----|------|---------|
| 0 | 失能（上电默认） | 未 enable / 通讯丢失后自动失能 |
| 1 | 使能 | 正常 |
| 3 | 输出轴校准异常 | 输出侧编码器未校准/损坏 |
| 4 | 传感器输出异常 | 编码器信号异常 |
| 5 | 电机编码器校准异常 | 磁编码器偏位/未校准 |
| 8 | 超压 | 母线电压过高 |
| 9 | 欠压 | 电源不足/压降 |
| A | 过电流 | 堵转、相电流超过 OC_Value |
| B | MOS 过温 | 驱动过热（保护阈值 120°C） |
| C | 电机线圈过温 | 绕组过热（建议≤100°C） |
| D | 通讯丢失 | TIMEOUT 内未收到 CAN 指令 |
| E | 过载 | 输出超 TMAX 持续 |

## 关键寄存器（RID，即 `DM_variable` 枚举）

| RID | 变量 | 含义 | 读写 |
|-----|------|------|------|
| 1 | KT_Value | 扭矩系数 | RW |
| 2 | OT_Value | 过温保护值 [80,200) | RW |
| 3 | OC_Value | 过流保护值 | RW |
| 6 | MAX_SPD | 最大速度 | RW |
| 7 | MST_ID | 反馈 ID | RW |
| 8 | ESC_ID | 接收 ID | RW |
| 9 | TIMEOUT | 通讯超时警报 | RW |
| 10 | CTRL_MODE | 控制模式 [0,4] | RW |
| 11 | Damp | 粘滞系数 | RO |
| 12 | Inertia | 转动惯量 | RO |
| 16 | NPP | 极对数 | RO |
| 17 | Rs | 相电阻 | RO |
| 18 | Ls | 相电感 | RO |
| 19 | Flux | 磁链 | RO |
| 20 | Gr | 减速比 | RO |
| 21-23 | PMAX/VMAX/TMAX | 映射范围 | RW |
| 24 | I_BW | 电流环带宽 | RW |
| 25-26 | KP_ASR/KI_ASR | 速度环 Kp/Ki | RW |
| 27-28 | KP_APR/KI_APR | 位置环 Kp/Ki | RW |
| 35 | can_br | CAN 波特率代码 [0,9] | RW |

参数读写命令：读 `0x33`、写 `0x55`、存 Flash `0xAA`、状态 `0xCC`，广播 ID `0x7FF`。

## 命令字节

| 命令 | 数据 | 说明 |
|------|------|------|
| enable | 8×0xFF + 0xFC | 使能 |
| disable | 8×0xFF + 0xFD | 失能 |
| set_zero | 8×0xFF + 0xFE | 保存当前位置为 0 位 |

## 常见故障 → 修复对照（速查）

| 症状 | 直接原因 | 修复方向 |
|------|---------|---------|
| 无反馈/无响应 | 串口端口错、波特率错、CAN ID 错、线缆/终端电阻、适配器异常 | 核对端口（`/dev/ttyACM*`）、921600、`addMotor` 注册的 ID 与电机 ESC_ID 一致、检查 `recv()` 是否解析出帧 |
| 通讯丢失 ERR=D | TIMEOUT 过短、CAN 干扰、总线负载高 | 提高 RID=9 TIMEOUT、检查屏蔽接地、降低发送频率 |
| enable 失败 ERR=0 | 上电后过早使能、旧固件、CTRL_MODE 未就绪 | 上电等 1-2s 再 enable、用 `enable_old`、确认 `switchControlMode` 成功 |
| 堵转 ERR=A/E | 力矩不足、TMAX 太低、机械卡死、Kp 太小饱和 | 提高 `Limit_Param` TMAX、检查机械、增大 Kp、确认插接力需求 ≤ 折算输出力矩 |
| 位置不准 | Kp 太小、未设零点、编码器偏移、量化误差 | 调大 Kp（`controlMIT` 或 KP_APR）、`set_zero_position`、检查 14 位编码器与齿比折算 |
| 振荡/抖动 | Kp/Kd 过大、Deta 阻尼低、电流环/速度环带宽不匹配 | 降低 Kp/Kd、提高 RID=11 Damp、检查 I_BW |
| 过温 ERR=B/C | 持续大扭矩、散热差、OT_Value 太低 | 降低占空比/负载、提高 OT_Value（≤限制）、检查冷却 |
| 模式不匹配 | CTRL_MODE≠代码假设 | `switchControlMode` 切到正确模式并验证 RID=10 回读 |
| 高速无力 | 反电动势限制、VMAX 太低 | 48V 供电提高空载转速、调整 VMAX、必要时弱磁（`ch06`） |

## 控制模式说明

- **MIT 模式** = 关节级 PD + 前馈：$$\tau = K_p(q_d - q) + K_d(\dot{q}_d - \dot{q}) + \tau_{ff}$$。适用于需自定义阻抗/柔顺的插接（力控）。
- **位置速度模式 (POS_VEL)**：驱动器内部位置环，`v_des` 是速度上限。适用于简单定位。
- **力位混控 (Torque_Pos, 0x300)**：p_des + v_des(×100) + i_des(×10000 标幺电流)。夹爪常用——位置到达后保持力矩，适合插接到位后的保力。
- 驱动器内部自行运行电流环 + 速度环 + 位置环（FOC），主机只发高层命令。见书中 ch13。
