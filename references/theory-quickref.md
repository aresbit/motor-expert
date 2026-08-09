# 电机学理论速查 (Theory Quick Reference)

> 依据：`/home/ares/yyscode/cii-code/feiman/electric/docs/` 中文书（MIT 6.685 电机学）。
> 完整推导见书中对应章节；本节只列诊断常用的核心关系。

## 1. 电磁力与力矩（ch00, ch03, ch06）

- 洛伦兹力 / 载流导体受力：$$\vec{F} = \int I\, d\vec{l} \times \vec{B}$$
- 能量法（虚功）：力 $$f = -\frac{\partial W}{\partial x}$$（恒磁链）或 $$f = +\frac{\partial W'}{\partial x}$$（恒电流），其中 $$W'$$ 为共能；线性系统 $$f = \tfrac{1}{2}i^2\frac{dL}{dx}$$
- 转矩（三相电机）：$$T_e = \frac{3}{2} p\, M\, I_a I_f \sin\delta_i$$，$$p$$ 为极对数，$$\delta_i$$ 为功率角
- 气隙切向应力（尺寸标度）：$$T = 2\pi r^2 \ell \langle\tau\rangle$$

## 2. 同步电机与永磁电机（ch03, ch06）

- 稳态相量方程：$$\tilde{V} = E_{af} - jX_d \tilde{I}$$
- 有功功率-功角：$$P = \frac{3VE_{af}}{X_d}\sin\delta$$，转矩 $$T = P/\omega_m$$
- 失步（pull-out）：$$|\delta| < 90^\circ$$ 稳定；$$\delta = 90^\circ$$ 为最大转矩
- PMSM dq 转矩：$$T_e = \frac{3}{2}p\big(\lambda_{pm} i_q + (L_d - L_q) i_d i_q\big)$$；表面贴装 $$L_d = L_q$$ 时 $$T_e = \frac{3}{2}p\lambda_{pm} i_q$$（$$i_q$$ 正比转矩）
- 反电动势：$$E \propto \lambda_{pm} \omega_e$$，电频率 $$\omega_e = p\,\omega_m$$
- 弱磁：电压受限时注入负 $$i_d$$ 扩展调速范围（$$V \approx \omega\sqrt{(L_d i_d + \lambda_{pm})^2 + (L_q i_q)^2}$$）

## 3. 磁场定向控制 (FOC)（ch09）

- 转子磁链定向：$$T_e = \frac{3}{2}p\frac{L_m}{L_r}\lambda_{rd} i_{qs}$$，滑差频率 $$\omega_{sl} = \frac{L_m i_{qs}}{\tau_r \lambda_{rd}}$$
- 控制器把 $$i_q$$（转矩）与 $$i_d$$（磁链）解耦；DM 驱动器内部即运行 FOC，主机只发高层命令

## 4. PD 控制与关节阻抗（ch13 核心）

- MIT 模式 = 关节级 PD + 前馈：$$\tau = K_p(q_d - q) + K_d(\dot{q}_d - \dot{q}) + \tau_{ff}$$
- 稳态位置误差（线性区，无积分）：$$e_{ss} \approx T_{load}/K_p$$ —— Kp 不足 → 位置不准
- 饱和：需求力矩 $$> T_{MAX}$$ 时进入饱和，误差按机械特性累积
- 阻尼不足（Kd 过小 / Damp 寄存器低）→ 振荡；Kp 过大 → 失稳/抖动

## 5. 减速齿比折算（ch12）

- 输出转矩：$$T_{out} = g \cdot T_m \cdot \eta_g$$（$$g$$ 齿比，$$\eta_g$$ 齿轮效率）
- 输出转速：$$\omega_{out} = \omega_m / g$
- 折算惯量：$$J_{eq} = J_m + J_l / g^2$$（齿比降低负载惯量影响）
- 电机侧额定转矩 = 输出转矩 / g：DM-J4310-2EC 输出 3 Nm、齿比 10 → 电机侧 ≈ 0.3 Nm

## 6. 电气时间常数与带宽（ch12）

- 电气时间常数：$$\tau_e = L_s / R_s$$；DM-J4310-2EC：$$340\mu H / 0.65\Omega \approx 0.52\ \text{ms}$$（电流环带宽上限 ~300 Hz）
- 电流环带宽必须远低于载波频率；I_BW 寄存器限制

## 7. 编码器与量化（ch12）

- 编码器分辨率：$$\Delta\theta = 2\pi / 2^{bits}$$；14 位 → 电机侧 0.022°，齿比 10 后输出侧 0.0022°
- MIT 模式位置量化 16 位：$$\Delta q = 2\pi/2^{16}$$（映射到 ±12.5 rad 时量化步 ~0.38 mrad）
- 量化/编码器误差 → 位置抖动、重复定位误差

## 8. 损耗与发热（ch02, ch10）

- 铜耗：$$P_{cu} = 3 I^2 R_s$$（三相）
- 铁耗 = 涡流 + 磁滞；涡流损耗 $$P_e \propto \omega^2 B_0^2 t^2 \sigma$$（叠片厚度平方）
- 热限制决定持续额定；过温 → 绕组电阻增大（$$R \propto (1 + \alpha\Delta T)$$）→ 铜耗更大 → 正反馈
- 保护：OT_Value（过温）、OC_Value（过流）、TIMEOUT（通讯）

## 9. 诊断常用判据

| 指标 | 正常范围（DM-J4310-2EC 参考） | 越界含义 |
|------|------------------------------|---------|
| 相电流 | 额定 2.5A / 峰值 7.5A | 接近/超峰值 → 堵转或过载 |
| 输出转矩 | 额定 3 Nm / 峰值 7 Nm | 持续超额定 → 过温 |
| 相电阻 | 0.65 Ω @25°C | 异常升高 → 绕组过热/损坏 |
| 温度 | T_MOS ≤ 120°C, T_Rotor ≤ 100°C | 超限 → 保护失能 |
| 位置误差 | 取决于插接精度要求 | 稳态误差 → Kp/齿隙/零点 |
