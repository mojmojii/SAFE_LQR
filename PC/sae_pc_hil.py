"""
sae_pc_hil.py — SAE-EAGA-LQR 上位机（PC 端）
功能：串口监听 STM32 触发帧 → 暖启动 EAGA 在线重优化 → 安全检查 → 下行新增益
运行：VS Code 打开本文件直接运行，或终端  python sae_pc_hil.py
依赖：pip install pyserial numpy scipy

两种模式（文件顶部 MODE 切换）：
  MODE = "selftest" 自检模式（先跑这个！）：不接硬件，本地完整仿真 S3 场景
                    （前馈失配→固定K消不掉→触发→暖启动重优化→切换→消偏），
                    验证算法链路，输出 sae_selftest_log.csv + sae_selftest.png
  MODE = "real"     实物模式：连真实串口，等单片机触发帧，重优化后回传新增益
"""

import sys, time, struct, csv
import numpy as np
from scipy.linalg import solve_continuous_are

# ============================================================================
#  配置区（按实际情况改）
# ============================================================================
MODE       = "real"          # "selftest" / "real" / "identify"（辨识模式：采集脉冲响应+拟合模型参数）
COM_PORT   = "COM8"          # real 模式：设备管理器里看 USB转TTL 的端口号（你机器一直是 COM8；若重插后变 COM5 改这里）
BAUD       = 115200
TARGET     = 0.0             # 球位目标 [m]（稳态误差估计用）

# 被控对象：曲柄连杆-摆杆-滚球机构（2026-08-07 按用户 MATLAB 模型对齐）
# 模型：ẍ = -(5/7)·g·K0·φ；φ̈ = -(m_ball·g·K0/J_eq)·x - (b_eq/J_eq)·φ̇ + τ/J_eq
# 已验证：K0≈0.19 时 lqr(Q11=1000,R=50) 得 K1=-4.48（符号结构与固件 [-,-,+,+] 一致）
G_GRAV     = 9.8
RHO_STEEL  = 7850;  R_BALL   = 0.005
RHO_PPR    = 1800;  D_PIPE   = 0.02;  T_WALL = 0.0034;  L_BEAM = 0.25
GR_RED     = 7;     GREF     = 0.85;  J_ROTOR = 5e-6;    B_ROTOR = 5e-4
D_CRANK    = 0.010; D_ROD    = 0.010; R_CRANK = 0.04;    L_ROD_C = 0.05
K0_TR      = 0.19   # 平衡点传动比 dθ/dφ（由 tau_ff=0.0237 反推；辨识后可精化）
TAU_MAX    = 0.6             # 电机限幅 [N·m]（与固件 TAU_MAX 一致！实物 2026-08-07 改为 0.6）

# 出厂（现役）Q/R 染色体 —— MATLAB 里算出厂增益的那组 Q/R
# 编码：[log10(q1), log10(q2), log10(q3), log10(q4), log10(r)]
CHROM_INCUMBENT = np.array([3.0, -1.0, -1.52, -0.5, np.log10(12.44)])
# 说明：R=12.44 时 K1=-8.973 与固件 K1=-8.972 精确一致；K2~K4 符号/量级一致
# （固件 K2~K4 是 LQR 后人工微调过的，无法由单一 Q/R 完全复现，不影响安全）

# S3 场景：前馈失配 +50%
MISMATCH_S3  = 0.5
ERR_THR      = 0.002         # 触发阈值 2mm（固件 ERR_TRIG 同步改成 0.002f）

# 在线重优化参数（与论文一致：暖启动 + 小预算）
EA_POP       = 20            # 2 子群 × 10
EA_GEN       = 12
WARM_SPREAD  = 0.15          # 15% 邻域扰动

LOG_CSV      = "sae_hil_log.csv"

# ============================================================================
#  被控对象模型与 LQR
# ============================================================================
def _mech_params():
    m_ball = RHO_STEEL*(4/3)*np.pi*R_BALL**3
    m_beam = RHO_PPR*np.pi*((D_PIPE/2)**2-(D_PIPE/2-T_WALL)**2)*L_BEAM
    m_crank = RHO_STEEL*np.pi*(D_CRANK/2)**2*R_CRANK
    m_rod = RHO_STEEL*np.pi*(D_ROD/2)**2*L_ROD_C
    J_eq = (J_ROTOR*GR_RED**2*GREF + (1/3)*m_crank*R_CRANK**2
            + (1/3)*m_rod*L_ROD_C**2 + (1/3)*m_beam*L_BEAM**2)
    b_eq = B_ROTOR*GR_RED**2
    return m_ball, m_beam, m_crank, J_eq, b_eq

M_BALL, M_BEAM, M_CRANK, J_EQ, B_EQ = _mech_params()

def plant_AB(J=J_EQ, b=B_EQ, K0=K0_TR, m_b=M_BALL):
    """曲柄连杆机构线性化模型（与用户 MATLAB A5 前 4 维一致，符号约定=固件）"""
    A = np.array([[0, 1, 0, 0],
                  [0, 0, -5*G_GRAV*K0/7, 0],
                  [0, 0, 0, 1],
                  [-m_b*G_GRAV*K0/J, 0, 0, -b/J]], float)
    B = np.array([[0], [0], [0], [1/J]], float)
    return A, B

def lqr_from_chrom(chrom):
    q = 10.0**chrom[:4]
    r = 10.0**chrom[4]
    A, B = plant_AB()
    P = solve_continuous_are(A, B, np.diag(q), np.array([[r]]))
    return np.linalg.solve(np.array([[r]]), B.T @ P)[0]   # (4,)

def closed_loop_stable(K):
    A, B = plant_AB()
    return max(np.linalg.eigvals(A - B @ K.reshape(1, 4)).real) < 0

def safety_precheck(K, tau_ff):
    """论文 offline pre-check 的 PC 严格版（简化实现）"""
    A, B = plant_AB()
    try:
        lam = np.linalg.eigvals(A - B @ K.reshape(1, 4))
    except Exception:
        return False, "eig_fail"
    if max(lam.real) >= -0.1:
        return False, "margin"
    for l in lam:                                   # 阻尼比 ζ>0.3（复极点）
        if abs(l.imag) > 1e-6 and -l.real/abs(l) < 0.3:
            return False, "damping"
    if abs(K[0])*0.03 + abs(tau_ff) > 0.8*TAU_MAX:  # 饱和预测 < 80% 额定
        # 偏差假设取 0.03（2026-08-07 实测修正）：CENTER 定点偏差包络 3cm；
        # 该约束同时把 |K0| 限在 ~15 以内（出厂 9 附近），防 EAGA 漂到 -22 类激进增益
        # （实测 K0→-22 后实物逼近极限环，且偏离"温和校正"的实验设定）
        return False, "saturation"
    return True, "ok"

# 出厂增益：由出厂染色体经 LQR 算出（selftest 里保证模型自洽；
# real 模式下单片机用固件自己的 K1~K4，本值仅作显示对照）
K_FACTORY = lqr_from_chrom(CHROM_INCUMBENT)
TAU_FF_INC = 0.15             # 单片机当前实际施加的前馈 [N·m]，必须与固件 TAU_FF 一致！
                              # 【S3 失配注入 v3：固件=0.15（v1 0.0356 被摩擦吸收；v2 0.08 净剩仅 0.006→偏 0.7mm
                              #   仍低于触发线；v3 净剩 0.076→稳定偏 ~8mm，每次冷却必触发），此值同步改 0.15。
                              #   恢复正常固件（0.0237）时此值必须改回 0.0237，否则估算全错、预检全拒】
KI_MCU     = 8.0              # 固件外置积分增益 Ki（与固件 #define Ki 一致）

# ※ 实物模式安全开关：PC 模型与固件符号约定对齐验证前，必须保持 False
#   False = 观察模式：只监听/重优化/打印，绝不下发（2026-08-07 振荡事故后加）
ALLOW_DOWNLINK = False# 【S3 实验开闸】符号约定已验证、观察模式已通过；False=只观察不下发

# PC→固件符号映射（曲柄连杆 θ 与编码器 φ 反向：固件 K1 负、K3 正）
# 未经 MATLAB 模型核对前不要用；核对后把下行帧里的 K 换成 map_gain_to_mcu(K)
def map_gain_to_mcu(K):
    return K.copy()   # 2026-08-07 模型已按曲柄连杆机构对齐，PC 增益即固件约定，恒等

# ============================================================================
#  闭环仿真（适应度评估 & 自检时间线共用）
# ============================================================================
def simulate(K, tau_ff_ctrl, plant_tau_ff_true, T=5.0, dt=0.001, x0=None):
    A, B = plant_AB()
    n = int(T/dt)
    X = np.zeros((n, 4)); U = np.zeros(n)
    X[0] = x0 if x0 is not None else np.array([0.01, 0, 0, 0])
    sat = 0
    for i in range(1, n):
        u = float(-(K @ X[i-1]) + tau_ff_ctrl)
        us = np.clip(u, -TAU_MAX, TAU_MAX)
        if abs(us - u) > 1e-12: sat += 1
        dX = A @ X[i-1] + B.flatten()*(us - plant_tau_ff_true)
        X[i] = X[i-1] + dX*dt
        U[i] = us
        if abs(X[i,0]) > 0.5:          # 发散保护
            X[i:] = X[i]; break
    t = np.arange(n)*dt
    trapz = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
    return t, X, U, sat/n, float(trapz(np.abs(X[:,0]-TARGET), t))

def fitness(chrom):
    try:
        K = lqr_from_chrom(chrom)
    except Exception:
        return 1e6, None
    if not closed_loop_stable(K):
        return 1e6, None
    ok, _ = safety_precheck(K, TAU_FF_INC)
    if not ok:
        return 1e5, None
    _, _, _, sat, iae = simulate(K, TAU_FF_INC, TAU_FF_INC*(1+MISMATCH_S3), T=3.0, dt=0.002)
    return iae + 50*sat, K

# ============================================================================
#  暖启动 EAGA（小预算在线版）
# ============================================================================
print("[PC] 版本标记 v8：20Hz 连续数据流记录（论文主图 x(t)）+ 清积压帧 + 信任域——看到这行才是新脚本")
incumbent_chrom = CHROM_INCUMBENT.copy()

def eaga_online():
    global incumbent_chrom
    t0 = time.time()
    pop = incumbent_chrom + np.random.uniform(-WARM_SPREAD, WARM_SPREAD, (EA_POP, 5))
    pop[0] = incumbent_chrom                     # 现役染色体直接入种群（暖启动核心）
    elite = incumbent_chrom.copy(); elite_J = np.inf; elite_K = None
    for _ in range(EA_GEN):
        Js = np.zeros(EA_POP); Ks = [None]*EA_POP
        for i in range(EA_POP):
            Js[i], Ks[i] = fitness(pop[i])
        rank = np.argsort(Js)
        if Js[rank[0]] < elite_J:
            elite_J = Js[rank[0]]; elite = pop[rank[0]].copy(); elite_K = Ks[rank[0]]
        new = [pop[rank[0]].copy(), pop[rank[1]].copy()]
        while len(new) < EA_POP:
            a, b = np.random.randint(0, EA_POP, 2)
            p1 = pop[a] if Js[a] < Js[b] else pop[b]
            a, b = np.random.randint(0, EA_POP, 2)
            p2 = pop[a] if Js[a] < Js[b] else pop[b]
            child = np.where(np.random.rand(5) < 0.5, p1, p2)
            child += np.random.randn(5) * (0.05 + 0.25*np.random.rand()) * WARM_SPREAD
            new.append(child)
        pop = np.array(new)
    ms = (time.time()-t0)*1000
    return elite, elite_K, elite_J, ms

def estimate_tau_ff(K_cur, e_ss, tau_ff_c, xi=0.0):
    """certainty-equivalence 前馈更新（v2，扣除积分在扛的部分）：
    稳态平衡给出 plant_ff = tau_ff_c + Ki·xi − (K[0]+m·g·K0)·e_ss，
    但单片机增益切换【不清积分】（xi 保留继续出力），故下发的前馈必须扣除
    积分当前贡献：ff_new = plant_ff − Ki·xi = tau_ff_c − (K[0]+m·g·K0)·e_ss。
    ※ 2026-08-07 深夜实测：含 Ki·xi 的旧式在"积分冻结保持"段双重计算，
      估计器把积分贡献重复计入前馈 → 单调过冲（137s 球已到中心仍继续下修）"""
    k_react = M_BALL*G_GRAV*K0_TR
    return float(np.clip(tau_ff_c - (K_cur[0] + k_react)*e_ss, -0.3, 0.3))

# ============================================================================
#  帧协议（与固件严格一致）
# ============================================================================
def frame_gain_downlink(K, tau_ff):
    """PC→MCU：[AA 55 A5][k1..k4,tau_ff 5×f32][sum] 共24字节"""
    payload = struct.pack("<5f", K[0], K[1], K[2], K[3], tau_ff)
    body = bytes([0xAA, 0x55, 0xA5]) + payload
    return body + bytes([sum(body) & 0xFF])

def parse_stream(buf):
    """找帧 [AA 55 T][7×f32][sum] 共32字节；T=0x5A 触发帧，T=0x5C 连续数据流帧。
    返回 (st, buf, kind)：kind="trigger"/"stream"/None"""
    while len(buf) >= 32:
        if buf[0] != 0xAA:
            del buf[0]; continue
        if len(buf) >= 2 and buf[1] != 0x55:
            del buf[0]; continue
        if len(buf) < 32: break
        if buf[2] in (0x5A, 0x5C) and (sum(buf[:31]) & 0xFF) == buf[31]:
            kind = "trigger" if buf[2] == 0x5A else "stream"
            return struct.unpack("<7f", bytes(buf[3:31])), buf[32:], kind
        del buf[0]
    return None, buf, None

# ============================================================================
#  real 模式：实物链路
# ============================================================================
def run_real():
    import serial
    while True:                              # 掉线自动重连外层循环
        try:
            run_real_session()
        except Exception as ex:
            print(f"[PC] 串口异常断开：{ex}，3s 后尝试重连...")
            time.sleep(3)

def run_real_session():
    import serial
    try:
        ser = serial.Serial(COM_PORT, BAUD, timeout=0.05)
    except Exception as ex:
        print(f"[PC] 串口打开失败：{ex}")
        print("     检查：① COM 口号对不对 ② 串口助手/其他程序是否占用了该口 ③ USB 线插没插")
        time.sleep(5)
        return
    print(f"[PC] 串口 {COM_PORT} @ {BAUD} 已打开 ✓")
    print("[PC] 等待单片机数据...（收到第一帧即确认链路连通；触发帧到达后自动重优化）\n")
    log = open(LOG_CSV, "w", newline="")
    w = csv.writer(log); w.writerow(["time_s", "event", "detail"])
    t_start = time.time()
    buf = bytearray()
    K_cur = K_FACTORY.copy()          # 显示用；实际增益在单片机里
    tau_ff_applied = TAU_FF_INC       # 单片机当前实际施加的前馈（每次下发后跟踪更新）
    trig_count = 0                    # 触发计数：每 3 次才重优化，其余复用 K（PC 降频时保住 march 节奏）
    link_ok = False
    rx_bytes_total = 0
    t_last_hint = time.time()
    while True:
        data = ser.read(64)
        if data:
            buf += data
            rx_bytes_total += len(data)
        # 链路连通反馈：收到第一个合法触发帧时打印一次
        st, buf, kind = parse_stream(buf)
        # 连续数据流帧：只记 CSV（论文主图 x(t) 用），不走触发逻辑
        if st is not None and kind == "stream":
            w.writerow([f"{time.time()-t_start:.3f}", "stream",
                        f"x={st[0]:+.5f};dx={st[1]:+.5f};phi_err={st[2]:+.5f};dphi={st[3]:+.5f};"
                        f"xi={st[4]:+.5f};target={st[5]:+.5f};tau={st[6]:+.5f}"])
            if (int(time.time()*2) % 20) == 0: log.flush()   # 每 ~10s 落盘一次
            continue
        if st is not None and not link_ok:
            link_ok = True
            print(f"[{time.time()-t_start:7.2f}s] ★ 链路确认：已收到单片机帧，校验通过 "
                  f"(x={st[0]:+.4f}m, phi_err={st[2]:+.4f}rad) —— 串口双向联通 ✓\n")
        # 静默提示：10s 没收到任何字节就提醒一次
        if not link_ok and rx_bytes_total == 0 and time.time()-t_last_hint > 10:
            t_last_hint = time.time()
            print("[PC] 10s 未收到任何字节：查 TX/RX 交叉、共地、波特率、固件是否在跑")
        if st is None: continue
        t_now = time.time() - t_start
        print(f"[{t_now:7.2f}s] 收到触发：x={st[0]:+.4f}m target={st[5]:+.4f}m "
              f"xi={st[4]:+.4f} tau={st[6]:+.4f}N·m phi_err={st[2]:+.4f}rad")
        # 摩擦/前馈归因：积分扛得多 → 偏置主因是摩擦而非前馈失配。
        # ※ 此时冻结 ff 修正（2026-08-07 深夜实测：摩擦停靠点的残余偏差会骗估计器
        #   每轮 -0.02 单调过冲到 -0.15 触预检；归因判据命中时只下发 K、ff 保持现值）
        friction_held = abs(KI_MCU*st[4]) > abs(st[0]-st[5]) * 8
        if friction_held:
            print("          提示：偏置主因为摩擦死区（积分扛着），本次冻结 ff 修正，仅更新增益")
        w.writerow([f"{t_now:.3f}", "trigger",
                    f"x={st[0]:+.5f};target={st[5]:+.5f};xi={st[4]:+.5f};tau={st[6]:+.5f}"])
        # 1) 暖启动 EAGA 重优化（每 3 次触发一次；K 已收敛后重算纯属浪费，
        #    PC 降频时单次可达 15s+，会拖垮 ff 修正节奏）
        trig_count += 1
        if trig_count % 3 == 1:   # 第 1、4、7… 次触发重优化
            chrom, K_new, J, ms = eaga_online()
            print(f"          重优化完成：J={J:.4f}，耗时 {ms:.0f}ms，K={np.round(K_new,3)}")
            # 优化期间（PC 降频可达 15s+）串口积压的触发帧全是过期状态，
            # 不丢会被当成新帧连续处理 → ff 基于陈旧偏差过冲（2026-08-07 深夜实测）
            ser.reset_input_buffer(); buf = bytearray()
        else:
            K_new = K_cur
            print(f"          复用现役增益 K={np.round(K_new,3)}（下次触发重优化），仅做 ff 修正")
        # 2) certainty-equivalence 前馈更新（用上报的 target 和 xi）
        e_ss = st[0] - st[5]
        K_mcu_est = map_gain_to_mcu(K_new)
        if friction_held:
            tau_ff_est = tau_ff_applied          # 摩擦停靠段：估计不可信，冻结修正
        else:
            tau_ff_est = estimate_tau_ff(K_mcu_est, e_ss, tau_ff_applied, xi=st[4])
        # 信任域：每次修正限幅 ±0.02 N·m（2026-08-07 实测：摩擦带 ±0.05 内前馈不可辨识，
        # 全幅修正(步长~0.1>噪声带)必然在估计器-摩擦间形成极限环，实测 tau_ff ±0.09 往复振荡；
        # 小步逼近，错也错在摩擦带内，推不动球，随机游走有界）
        tau_ff_new = tau_ff_applied + float(np.clip(tau_ff_est - tau_ff_applied, -0.02, 0.02))
        # 3) 安全检查
        ok, why = safety_precheck(K_new, tau_ff_new)
        if not ok:
            print(f"          新增益未通过安全检查（{why}），不下发")
            w.writerow([f"{t_now:.3f}", "rejected", why]); log.flush()
            continue
        # 4) 下行 + 存档新种子（受 ALLOW_DOWNLINK 总开关控制）
        if not ALLOW_DOWNLINK:
            print(f"          [观察模式] 检查通过但未下发（ALLOW_DOWNLINK=False）。"
                  f"映射后增益 K_mcu={np.round(map_gain_to_mcu(K_new),3)}")
            w.writerow([f"{t_now:.3f}", "monitor_only",
                        f"K_mcu={np.round(map_gain_to_mcu(K_new),4).tolist()};tau_ff={tau_ff_new:.5f}"])
            log.flush()
            continue
        K_mcu = map_gain_to_mcu(K_new)
        ser.write(frame_gain_downlink(K_mcu, tau_ff_new))
        globals()["incumbent_chrom"] = chrom.copy()
        K_cur = K_new
        tau_ff_applied = tau_ff_new   # 跟踪单片机实际施加值（估计器基准，勿再用 TAU_FF_INC 常量）
        print(f"          已下发新增益（K_mcu={np.round(K_mcu,3)}，tau_ff={tau_ff_new:+.4f}），种子已更新")
        w.writerow([f"{t_now:.3f}", "switched",
                    f"K={np.round(K_new,4).tolist()};tau_ff={tau_ff_new:.5f}"])
        log.flush()

# ============================================================================
#  selftest 模式：本地跑完整 S3 链条，出 CHIL 主图数据
# ============================================================================
def run_selftest():
    print("[selftest] 本地仿真 S3 场景：t=12s 注入前馈失配 +50%")
    print(f"[selftest] 出厂增益 K={np.round(K_FACTORY,3)}，TAU_FF={TAU_FF_INC}\n")
    dt = 0.001; T = 28.0; n = int(T/dt)
    A, B = plant_AB()
    K = K_FACTORY.copy(); tau_ff_c = TAU_FF_INC
    plant_ff = TAU_FF_INC
    X = np.zeros((n, 4)); X[0] = [0.003, 0, 0, 0]
    U = np.zeros(n); events = []
    err_win = []; cooldown = 0; mismatch_on = False
    for i in range(1, n):
        t = i*dt
        if t >= 12.0 and not mismatch_on:
            plant_ff = TAU_FF_INC*(1+MISMATCH_S3)
            mismatch_on = True
            events.append((t, "mismatch +50%"))
        u = float(-(K @ X[i-1]) + tau_ff_c)
        U[i] = np.clip(u, -TAU_MAX, TAU_MAX)
        dX = A@X[i-1] + B.flatten()*(U[i] - plant_ff)
        X[i] = X[i-1] + dX*dt
        if abs(X[i,0]) > 0.5:
            print(f"[{t:6.2f}s] 状态发散，终止（检查模型参数）"); X[i:] = X[i]; break
        # ---- 模拟 MCU 滑窗触发（500ms 窗，mean|e|>4mm，3s 冷却）----
        err_win.append(abs(X[i,0]-TARGET))
        if len(err_win) > 500: err_win.pop(0)
        if cooldown > 0: cooldown -= 1
        if (mismatch_on and len(err_win) == 500 and np.mean(err_win) > ERR_THR and cooldown == 0):
            chrom, K_new, J, ms = eaga_online()
            e_ss = X[i,0] - TARGET
            tau_ff_new = estimate_tau_ff(K, e_ss, tau_ff_c)
            ok, why = safety_precheck(K_new, tau_ff_new)
            print(f"[{t:6.2f}s] 触发！当前偏置 {e_ss*100:+.2f}cm → 重优化 {ms:.0f}ms "
                  f"(J={J:.3f}) → " + (f"切换，tau_ff {tau_ff_c:+.4f}→{tau_ff_new:+.4f}"
                                        if ok else f"被拒:{why}"))
            events.append((t, f"trigger→reopt({ms:.0f}ms)→{'switch' if ok else 'reject:'+why}"))
            if ok:
                K = K_new; tau_ff_c = tau_ff_new
                globals()["incumbent_chrom"] = chrom.copy()
            cooldown = 3000
    t_arr = np.arange(n)*dt
    with open("sae_selftest_log.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_s", "x_m", "phi_rad", "u_nm"])
        for i in range(0, n, 10):
            w.writerow([f"{t_arr[i]:.3f}", f"{X[i,0]:.6f}", f"{X[i,2]:.6f}", f"{U[i]:.5f}"])
    ess_off   = X[int(20/dt):int(21/dt),0].mean()   # 失配后、切换前（若12.x s已切换则另取）
    ess_final = X[int(26/dt):,0].mean()
    print(f"\n[selftest] 末段稳态偏置 ≈ {ess_final*100:.3f} cm（切换成功时应≈0）")
    print("[selftest] 事件链：" + "  |  ".join(f"{t:.1f}s {e}" for t, e in events))
    print("[selftest] 已保存 sae_selftest_log.csv —— t-x 曲线即 CHIL 主图")
    try:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(9, 4.5))
        plt.plot(t_arr, X[:,0]*100, lw=1.2)
        for t, e in events:
            plt.axvline(t, color="r", ls="--", alpha=0.5)
        plt.xlabel("t (s)"); plt.ylabel("x (cm)"); plt.grid(alpha=0.3)
        plt.title("SAE-EAGA-LQR selftest: mismatch -> trigger -> switch -> bias-free")
        plt.tight_layout(); plt.savefig("sae_selftest.png", dpi=150)
        print("[selftest] 已保存 sae_selftest.png")
    except Exception as ex:
        print("[selftest] 画图跳过：", ex)


# ============================================================================
#  identify 模式：系统辨识（模型不可信时，用真实数据认参数）
#  固件开 SAE_ID_MODE 后发 [AA 55 5B][x,dx,phi_err,dphi,tau 5×f32][sum] @100Hz
# ============================================================================
def parse_id_stream(buf):
    while len(buf) >= 24:
        if buf[0] != 0xAA:
            del buf[0]; continue
        if len(buf) >= 2 and buf[1] != 0x55:
            del buf[0]; continue
        if len(buf) < 24: break
        if buf[2] == 0x5B and (sum(buf[:23]) & 0xFF) == buf[23]:
            return struct.unpack("<5f", bytes(buf[3:23])), buf[24:]
        del buf[0]
    return None, buf

def run_identify():
    import serial
    DUR = float(input("采集时长（建议 60~120 秒）: ") or "90")
    ser = serial.Serial(COM_PORT, BAUD, timeout=0.05)
    print(f"[ID] {COM_PORT} 已打开，采集 {DUR:.0f}s ... 确认固件 SAE_ID_MODE=1 且杆上无球（第一轮）")
    rows = []; buf = bytearray(); t0 = time.time()
    while time.time() - t0 < DUR:
        d = ser.read(128)
        if d: buf += d
        st, buf = parse_id_stream(buf)
        if st: rows.append(st)
    ser.close()
    print(f"[ID] 采集 {len(rows)} 帧，保存 sae_id_log.csv")
    with open("sae_id_log.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["x","dx","phi_err","dphi","tau"]); w.writerows(rows)
    if len(rows) < 500:
        print("[ID] 数据太少，检查链路"); return
    D = np.array(rows)
    dt = 0.01
    # ---- 杆通道：phi_ddot = a·τ − b·φ̇ + g·φ ----
    phi_ddot = np.gradient(D[:,3], dt)
    R = np.column_stack([D[:,4], -D[:,3], D[:,2]])
    coef, *_ = np.linalg.lstsq(R, phi_ddot, rcond=None)
    a, b, g = coef
    # ---- 球通道：x_ddot = c·φ + d·ẋ（若第一轮无球，c 不可辨识，跳过）----
    x_ddot = np.gradient(D[:,1], dt)
    R2 = np.column_stack([D[:,2], D[:,1]])
    c, d = np.linalg.lstsq(R2, x_ddot, rcond=None)[0]
    # ---- 重力前馈实测：脉冲为零段的平均 tau 残差 ≈ 0（杆自由），平衡保持力矩另测 ----
    print("\n[ID] ===== 辨识结果 =====")
    print(f"  杆通道: phi_ddot = {a:+.2f}·tau  {-abs(b):+.3f}·dphi  {g:+.2f}·phi_err")
    print(f"       → J_eff = 1/a = {1/a:.5f} kg·m²   b = {b:.4f} N·m·s/rad")
    print(f"  球通道: x_ddot = {c:+.3f}·phi_err {d:+.3f}·dx   (无球轮忽略此行)")
    print(f"  符号核对: a{'>0' if a>0 else '<0'}, c{'<0（与固件 K1 负一致 ✓）' if c<0 else '>0（与固件 K1 负矛盾！）'}")
    print("\n[ID] 把这两个值填进脚本配置区：")
    print(f"     J_ROD = {1/a:.5f}")
    print(f"     B_FRIC = {b:.5f}")
    print(f"     并把 plant_AB 里 (5/7)*G_GRAV 替换为辨识的耦合系数（符号按上行核对结果）")

# ============================================================================
if __name__ == "__main__":
    np.random.seed(42)
    if MODE == "real":
        run_real()
    elif MODE == "identify":
        run_identify()
    else:
        run_selftest()