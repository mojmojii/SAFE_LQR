#include "test_2.h"
#include "test_1.h"
#include "main.h"
#include "bsp_can.h"
#include "struct_typedef.h"
#include "cmsis_os.h"
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

// ============================================================================
//  机构参数（当前机器：a=0.17 h=0.025 r=0.04 l=0.05 L=0.25）
// ============================================================================
#define MECH_A      (0.17f)
#define MECH_H      (0.025f)
#define MECH_R      (0.04f)
#define MECH_L_ROD  (0.05f)
#define MECH_L      (0.25f)

// ============================================================================
//  LQR 参数（4 状态：x, dx, phi, dphi；积分外置，老结构）
//  当前机构重算：Q11=1000, R=50（MATLAB lqr，2026-08-03）
//  ※ 以下 #define 现在作为"出厂默认增益"，运行时可被 SAE 在线更新替换
// ============================================================================
#define K1        (-8.972f)       // x 增益4.572
#define K2        (-2.147f)       // dx 增益
#define K3        (0.606f)        // phi 增益
#define K4        (0.0456f)       // dphi 增益

#define PHI0      (4.023f)       // 摆杆水平时电机编码器读数 [rad]（实测）3.9395f
#define TAU_FF    (0.1500f)       // 【S3 失配注入版 v3】前两版均不够：0.0356 被摩擦全吸收；0.08 超出摩擦
                                  //  的部分仅剩 0.006，只能拖偏 ~0.7mm，仍低于 4mm 触发线。
                                  //  0.15：净剩 0.076 N·m，球稳定偏 ~8mm（>4mm 触发、<1cm kick 死区、
                                  //  <1.5cm 积分带下限——三者都错开）。为 TAU_MAX 的 1/4，安全。
                                  //  实验后改回 0.0237f
#define TRACKING_START_TAU_FF (-0.010f) /* PA3/PA5 寻迹期间向下的额外平衡前馈。 */

// ============================================================================
//  外置积分（老结构：不在 LQR 增益向量里，单独加）
// ============================================================================
#define Ki        (8.0f)          // 积分增益（有静差就加大，抖动就减小）
// 积分工作带：0.5cm~4cm 内累积；之外冻结（防大偏差乱卷），之内停积保持（防零点抖振）
#define INT_BAND_HI (0.04f)
#define INT_BAND_LO (0.003f)      // 0.3cm（2026-08-07 深夜实测：摩擦停靠点 ~4.9mm 与 5mm 带沿重叠，
                                  //  积分冻结→残余偏差反复点火→估计器被骗着过冲到 -0.15；
                                  //  降到 3mm 让积分收尾最后几 mm，球进 4mm 触发线内即停）
#define XI_MAX    (0.1f)          // 积分限幅

// 摩擦补偿三件套（2026-08-07 实测：静摩擦 τ_s≈0.05~0.06 N·m，球卡 -3.2cm 事件）
// ※ TAU_KICK / TAU_COUL 为估值，堵转法标定后替换
#define TAU_KICK  (0.06f)        // 突破脉冲：卡住时给，取 τ_s×0.9，一动就撤
#define TAU_COUL  (0.045f)        // 库仑摩擦前馈：动摩擦 τ_c，跟随速度方向，停归零
#define OMEGA_EPS (0.05f)         // tanh 平滑带宽，防零点抖振

#define DT        (0.001f)        // 控制周期 [s]

// ============================================================================
//  保护参数
// ============================================================================
#define TAU_MAX     (0.6f)        // 峰值限幅
#define TAU_RATED   (0.6f)        // 额定限幅
#define PHI_SOFT_LIM (0.58f)      // 软限幅：偏离平衡点 ±0.58 rad（当前机构死区×0.8）
#define PHI_HARD_LIM (0.72f)      // 硬死区：±0.72 rad（当前机构可达边界）
#define POS_FENCE_HI  (6.09f)    // 绝对编码器上限围栏（机械极限 6.117，留 ~4° 余量）
#define FENCE_PULL    (-0.3f)    // 越围栏后的固定回拉力矩（小力矩，往工作区方向拉）
// ============================================================================
//  目标参数
// ============================================================================
#define TARGET_POSITIVE          (0.052f)
#define TARGET_NEGATIVE          (-0.055f)
#define TARGET_POSITION_TOLERANCE (0.005f)
#define TARGET_SPEED_TOLERANCE   (0.01f)
#define POSITIVE_TIMEOUT_MS      (3000U)
#define TRAJECTORY_TIMEOUT_MS    (4500U)

// ============================================================================
//  SAE-EAGA-LQR 运行时框架（论文机制单片机落地）
//  串口链路已接：触发→上行请求帧→PC 暖启动 EAGA→下行新增益→轻检→混合切换。
// ============================================================================
#define BLEND_STEPS     (50U)      // 无扰切换：50ms 线性混合（1kHz 下 50 步）
#define SAFE_POOL_N     (5U)       // 安全池容量 N_safe = 5
#define ERR_TRIG        (0.004f)   // 触发阈值：窗内 mean|e1| > 4mm（S3 实验：错开 5mm 积分死区，避免死区抖动反复点火；平时可改回 0.002f）
#define COOLDOWN_STEPS  (3000U)    // 触发冷却 3s，防频繁重优化

typedef struct { float k[4]; float tau_ff; } GainSet_t;

typedef struct {
    GainSet_t gain;
    float fitness;
    float p[4][4];
    uint8_t valid;
} SafeController_t;

#define UPDATE_PERIOD_STEPS (500U)
#define VSS_INITIAL (0.04f)
#define VSS_FLOOR (1.0e-4f)
#define VSS_ALPHA (0.01f)
#define RUNTIME_GAIN_SCALE (0.9f)
#define FACTORY_MODEL_FITNESS (0.0254812f)
#define GAIN_FRAME_LEGACY_LEN (24U)
#define GAIN_FRAME_EXT_LEN (68U)

static SafeController_t g_safe_pool[SAFE_POOL_N];
static uint8_t g_safe_pool_count = 0U;
static float g_gain_new_fitness = FLT_MAX;
// Policy-evaluation Lyapunov matrix for the manually tuned factory gain.
static float g_p_cur[4][4] = {
    {1148.872706f, 246.428820f, -33.536201f, -0.213145f},
    {246.428820f, 80.026878f, -14.929501f, -0.244699f},
    {-33.536201f, -14.929501f, 3.975809f, 0.069808f},
    {-0.213145f, -0.244699f, 0.069808f, 0.006560f}
};
static float g_p_old[4][4];
static float g_p_new[4][4];
static float g_v_ss = VSS_INITIAL;
static uint16_t g_periodic_steps = 0U;
static uint8_t g_pending_pool_admit = 0U;

static GainSet_t g_gain_cur  = {{K1, K2, K3, K4}, TAU_FF};  // 现役增益（默认=出厂）
static GainSet_t g_gain_old  = {{K1, K2, K3, K4}, TAU_FF};  // 切换前旧增益（混合用）
static GainSet_t g_gain_new  = {{0}};                        // 待切换增益（外部写入）
static volatile uint8_t g_new_pending = 0;    // 有待切换增益，控制循环验证后混合
static volatile uint8_t g_blending    = 0;    // 1 = 正在 50ms 混合
static uint16_t g_blend_cnt   = 0;
static uint16_t g_cooldown    = 0;
volatile uint8_t sae_update_request = 0;   // 触发标志：调试器/OLED 可读，外部置 0 清除

// ============================================================================
//  系统辨识模式（临时用：模型参数实测）
//  1=辨识：电机脱离控制，只发 ±0.04/0.06 N·m 脉冲 + 100Hz 上报状态，PC 端
//          MODE="identify" 采集拟合。杆上先不放球（第1轮杆通道），再放球（第2轮）。
//  0=正常运行（默认）。辨识完必须改回 0 重新烧录！
// ============================================================================
#define SAE_ID_MODE   0

// SAE 统计（调试器 Live Watch 直接观察，无需任何外设）
volatile uint32_t sae_stat_triggers  = 0;   // 触发次数
volatile uint32_t sae_stat_rejected  = 0;   // 安全检查拦截次数
volatile uint32_t sae_stat_rollbacks = 0;   // 屏障回滚次数
volatile uint32_t sae_stat_switches  = 0;   // 成功切换次数

// ============================================================================
//  SAE 对外接口：任何地方拿到新增益后调这个函数即可发起切换
//  （现在没人调 = 纯固定 LQR，行为与旧固件逐位一致）
// ============================================================================
void sae_submit_gain(float k1, float k2, float k3, float k4, float tau_ff)
{
    if (g_blending) {
        sae_stat_rejected++;
        return;
    }
    GainSet_t gn = {{k1, k2, k3, k4}, tau_ff};
    g_gain_new = gn;
    g_gain_new_fitness = FLT_MAX;
    memcpy(g_p_new, g_p_cur, sizeof(g_p_new));
    g_new_pending = 1U;
}

// ============================================================================
//  SAE 串口链路（PC ↔ MCU，115200 8N1）
//  ※ 使用前先改这里：extern 的句柄名必须和 Core/Src/usart.c 里定义的一致
//    （上次编译报 huart1 undefined，说明你的工程不叫 huart1，可能是 huart2/3/6）
// ============================================================================
extern UART_HandleTypeDef huart8;                 // UART8（原陀螺仪串口，已征用为 HIL 链路）
#define SAE_HUART   huart8

// 下行帧：0xA5 legacy=24 B；0xA6 scored controller + fitness + symmetric P=68 B.
// 在 UART 接收中断/IDLE 回调里对每个字节调用一次
void sae_rx_byte(uint8_t b) {
    static uint8_t buf[GAIN_FRAME_EXT_LEN];
    static uint8_t idx = 0U;
    static uint8_t expected_len = 0U;
    if (idx == 0 && b != 0xAA) return;
    if (idx == 1U && b != 0x55) { idx = (b == 0xAA) ? 1U : 0U; return; }
    buf[idx] = b;
    if (idx == 2U) {
        if (b == 0xA5) expected_len = GAIN_FRAME_LEGACY_LEN;
        else if (b == 0xA6) expected_len = GAIN_FRAME_EXT_LEN;
        else { idx = 0U; expected_len = 0U; return; }
    }
    idx++;
    if (expected_len == 0U || idx < expected_len) return;
    idx = 0U;
    uint8_t sum = 0U;
    for (uint8_t i = 0U; i < (uint8_t)(expected_len - 1U); i++) {
        sum = (uint8_t)(sum + buf[i]);
    }
    if (sum != buf[expected_len - 1U]) return;
    if (g_blending) {
        sae_stat_rejected++;
        return;
    }
    memcpy(&g_gain_new, &buf[3], sizeof(GainSet_t));
    if (buf[2] == 0xA6) {
        memcpy(&g_gain_new_fitness, &buf[23], sizeof(float));
        float p_upper[10];
        memcpy(p_upper, &buf[27], sizeof(p_upper));
        memset(g_p_new, 0, sizeof(g_p_new));
        const uint8_t ri[10] = {0,0,0,0,1,1,1,2,2,3};
        const uint8_t ci[10] = {0,1,2,3,1,2,3,2,3,3};
        for (uint8_t i = 0U; i < 10U; i++) {
            g_p_new[ri[i]][ci[i]] = p_upper[i];
            g_p_new[ci[i]][ri[i]] = p_upper[i];
        }
    } else {
        g_gain_new_fitness = FLT_MAX;
        memcpy(g_p_new, g_p_cur, sizeof(g_p_new));
    }
    g_new_pending = 1;                    // 不在中断里切换，交控制循环处理
}

// 上行帧（MCU→PC）：[0xAA][0x55][0x5A][x,dx,phi_err,dphi,xi,target,tau 7×f32][sum] 共 32 字节
// 附带积分 xi / 目标 target / 实际输出 tau：PC 据此区分摩擦偏置与前馈失配
static void sae_send_update_request(float x, float dx, float phi_err, float dphi,
                                    float xi, float target, float tau_now) {
    uint8_t buf[32];
    float st[7] = {x, dx, phi_err, dphi, xi, target, tau_now};
    buf[0] = 0xAA; buf[1] = 0x55; buf[2] = 0x5A;
    memcpy(&buf[3], st, 28);
    uint8_t sum = 0;
    for (int i = 0; i < 31; i++) sum = (uint8_t)(sum + buf[i]);
    buf[31] = sum;
    HAL_UART_Transmit(&SAE_HUART, buf, 32, 10);
}

// ============================================================================
//  备份级联 PID（保留旧版，完全不动）
// ============================================================================
#define BALL_ANGLE_OFFSET_MAX    (0.25f)
#define BALL_POSITION_INT_SEP    (0.03f)
#define BALL_ANGLE_INT_SEP       (0.15f)
#define VISION_LPF_ALPHA          (0.10f)

static const fp32 BALL_POSITION_PID[3] = {2.0f, 0.0f, 0.05f};
static const fp32 BALL_ANGLE_PID[3] = {1.5f, 0.05f, 0.02f};
static pid_type_def ball_position_pid;
static pid_type_def ball_angle_pid;

typedef struct
{
    float output;
    uint8_t initialized;
} FirstOrderFilter_t;

static FirstOrderFilter_t vision_position_filter = {0};
static FirstOrderFilter_t vision_speed_filter = {0};

uint64_t test_time;

// ============================================================================
//  内部状态
// ============================================================================
typedef struct {
    float xi;
    float dx_f;
    float dphi_f;
    uint8_t sat;
} BalState_t;

static BalState_t g = {0};
typedef enum {
    K1_PROFILE_RUNTIME = 0,
    K1_PROFILE_FIXED,
    K1_PROFILE_ADJUSTABLE
} K1Profile_t;

static K1Profile_t g_k1_profile = K1_PROFILE_RUNTIME;
static float g_k1_fixed = K1;
static float g_k1_adjust_ratio = 0.0f;

// ============================================================================
//  工具函数
// ============================================================================
static inline float clamp(float v, float lo, float hi) {
    return (v > hi) ? hi : ((v < lo) ? lo : v);
}

// 保险：角度差 wrap 到 [-pi, +pi]。当前 PHI0=3.9395 离跳变点很远，
// 正常工作永远不会触发，效果和直接相减完全一样；留着只是兜底。
static inline float wrap_pi(float a) {
    while (a >  3.14159265f) a -= 6.28318531f;
    while (a < -3.14159265f) a += 6.28318531f;
    return a;
}

static float first_order_filter_calc(FirstOrderFilter_t *filter, float input)
{
    if (filter->initialized == 0U)
    {
        filter->output = input;
        filter->initialized = 1U;
    }
    else
    {
        filter->output += VISION_LPF_ALPHA * (input - filter->output);
    }
    return filter->output;
}

static uint8_t mech_ok(float phi)
{
    float delta = wrap_pi(phi - PHI0);
    return (fabsf(delta) < PHI_HARD_LIM) ? 1 : 0;
}

// ============================================================================
//  SAE-EAGA-LQR 运行时函数
// ============================================================================

// 轻量运行前检查（论文 offline pre-check 的 MCU 简化版：兜物理边界）
static uint8_t gain_sane(const GainSet_t *gn) {
    for (int i = 0; i < 4; i++) {
        if (!isfinite(gn->k[i]) || fabsf(gn->k[i]) > 500.0f) return 0;
    }
    if (!isfinite(gn->tau_ff) || fabsf(gn->tau_ff) > 0.3f) return 0;
    // Match the host pre-check's 3 cm candidate-evaluation envelope.
    if (fabsf(gn->k[0]) * 0.03f + fabsf(gn->tau_ff) > 0.8f * TAU_MAX) return 0;
    return 1;
}

static uint8_t matrix_spd4(const float p[4][4])
{
    float l[4][4] = {{0.0f}};
    for (uint8_t i = 0U; i < 4U; i++) {
        for (uint8_t j = 0U; j <= i; j++) {
            float s = p[i][j];
            for (uint8_t k = 0U; k < j; k++) s -= l[i][k] * l[j][k];
            if (i == j) {
                if (!isfinite(s) || s <= 1.0e-6f) return 0U;
                l[i][j] = sqrtf(s);
            } else {
                l[i][j] = s / l[j][j];
            }
        }
    }
    return 1U;
}

static uint8_t safe_pool_can_admit(float fitness)
{
    if (!isfinite(fitness)) return 0U;
    if (g_safe_pool_count < SAFE_POOL_N) return 1U;
    return fitness < g_safe_pool[g_safe_pool_count - 1U].fitness;
}

static uint8_t safe_pool_admit(const GainSet_t *gain, float fitness,
                              const float p[4][4])
{
    if (!safe_pool_can_admit(fitness) || !matrix_spd4(p)) return 0U;
    uint8_t pos = 0U;
    while (pos < g_safe_pool_count && g_safe_pool[pos].fitness <= fitness) pos++;
    uint8_t new_count = (g_safe_pool_count < SAFE_POOL_N)
                      ? (uint8_t)(g_safe_pool_count + 1U) : SAFE_POOL_N;
    for (uint8_t i = new_count - 1U; i > pos; i--) {
        g_safe_pool[i] = g_safe_pool[i - 1U];
    }
    g_safe_pool[pos].gain = *gain;
    g_safe_pool[pos].fitness = fitness;
    memcpy(g_safe_pool[pos].p, p, sizeof(g_safe_pool[pos].p));
    g_safe_pool[pos].valid = 1U;
    g_safe_pool_count = new_count;
    return 1U;
}

static float lyapunov_value(float e1, float e2, float e3, float e4)
{
    const float x[4] = {e1, e2, e3, e4};
    float p[4][4];
    float a = g_blending ? ((float)g_blend_cnt / (float)BLEND_STEPS) : 0.0f;
    for (uint8_t i = 0U; i < 4U; i++) {
        for (uint8_t j = 0U; j < 4U; j++) {
            p[i][j] = g_blending
                    ? (1.0f-a)*g_p_old[i][j] + a*g_p_new[i][j]
                    : g_p_cur[i][j];
        }
    }
    float v = 0.0f;
    for (uint8_t i = 0U; i < 4U; i++) {
        for (uint8_t j = 0U; j < 4U; j++) v += x[i] * p[i][j] * x[j];
    }
    return v;
}

static void gain_rollback(void) {                      // 屏障触发：回滚最近安全增益
    if (g_safe_pool_count > 0U) {
        g_gain_cur = g_safe_pool[0].gain;
        memcpy(g_p_cur, g_safe_pool[0].p, sizeof(g_p_cur));
    }
    g_blending = 0;
    g_blend_cnt = 0;
    g_pending_pool_admit = 0U;
    sae_stat_rollbacks++;
}

// ============================================================================
//  核心控制（老结构：4 状态 LQR + 重力前馈 + 外置积分）
//  输入：x(m), dx(m/s), phi(rad), dphi(rad/s), target(m)
//  返回：力矩(N·m)，直接塞给达妙 MIT t_ff
//  ※ SAE 改动仅两处：第 5 步改用运行时混合增益；新增第 8 步触发检测与屏障
// ============================================================================
float balance_control(float x, float dx, float phi, float dphi, float target)
{
    // 0. 绝对围栏：编码器永不许超过机械极限点
    if (phi > POS_FENCE_HI) {
        return FENCE_PULL;      // 顶到就固定给小回拉力矩，跳过 LQR 和积分
    }
    // 1. 低通滤波
    g.dx_f   = 0.3f * dx   + 0.7f * g.dx_f;
    g.dphi_f = 0.3f * dphi + 0.7f * g.dphi_f;

    // 2. 误差
    float e1 = x - target;
    float e2 = g.dx_f;
    float e3 = wrap_pi(phi - PHI0);
    float e4 = g.dphi_f;

    // 3. 软限幅：偏离工作点过远，固定小力矩往平衡点拉
    if (e3 > PHI_SOFT_LIM || e3 < -PHI_SOFT_LIM) {
        float tau_pull = (e3 > 0.0f) ? -0.15f : 0.15f;
        return clamp(tau_pull, -TAU_MAX, TAU_MAX);
    }

    // 4. 硬死区保护（连杆够不着时）
    if (!mech_ok(phi)) {
        return clamp(TAU_FF, -TAU_MAX, TAU_MAX);
    }

    // 5. LQR + 重力前馈 + 外置积分（SAE：运行时增益 + 50ms 无扰混合）
    float k1 = g_gain_cur.k[0], k2 = g_gain_cur.k[1];
    float k3 = g_gain_cur.k[2], k4 = g_gain_cur.k[3];
    float tau_ff = g_gain_cur.tau_ff;
    if (g_blending) {
        float a = (float)g_blend_cnt / (float)BLEND_STEPS;   // 0→1 线性混合
        k1 = (1.0f-a)*g_gain_old.k[0] + a*g_gain_new.k[0];
        k2 = (1.0f-a)*g_gain_old.k[1] + a*g_gain_new.k[1];
        k3 = (1.0f-a)*g_gain_old.k[2] + a*g_gain_new.k[2];
        k4 = (1.0f-a)*g_gain_old.k[3] + a*g_gain_new.k[3];
        tau_ff = (1.0f-a)*g_gain_old.tau_ff + a*g_gain_new.tau_ff; // Kr_hat 同步切换
    }
    // 专用目标模式基于已混合的 k1 计算，避免绕过 50 ms 无扰切换。
    float effective_k1 = k1;
    if (g_k1_profile == K1_PROFILE_FIXED) {
        effective_k1 = g_k1_fixed;
    } else if (g_k1_profile == K1_PROFILE_ADJUSTABLE) {
        effective_k1 = k1 + (7.2f - k1) * g_k1_adjust_ratio;
    }
    float tau_feedback = -(effective_k1*e1 + k2*e2 + k3*e3 + k4*e4);

    // ---- 5.5 摩擦补偿：卡住给突破脉冲，动起来换动摩擦前馈 ----
    float ae1 = fabsf(e1);
    // 卡住判定：位置偏差>1cm 且球和杆都基本不动（真·被静摩擦卡住）
    uint8_t stuck = (ae1 > 0.01f) && (fabsf(dx) < 0.002f) && (fabsf(g.dphi_f) < 0.1f);
    // 脉冲方向取反馈需求方向（控制器想往哪推就往哪帮），不依赖速度符号
    float tau_demand = -(effective_k1 * e1);
    float tau_fric;
    if (stuck) {
        tau_fric = (tau_demand > 0.0f) ? TAU_KICK : -TAU_KICK;   // 突破脉冲（τ_s×0.9）
    } else {
        tau_fric = TAU_COUL * tanhf(g.dphi_f / OMEGA_EPS);        // 动摩擦前馈（动才给，停归零）
    }
    float tau_raw = tau_feedback + tau_ff + Ki * g.xi + tau_fric;
    float gain_scale = (fabsf(tau_raw) > 0.8f * TAU_MAX)
                     ? RUNTIME_GAIN_SCALE : 1.0f;
    float tau = gain_scale * tau_feedback + tau_ff + Ki * g.xi + tau_fric;

    // 6. 峰值限幅
    float tau_sat = clamp(tau, -TAU_MAX, TAU_MAX);

    // 7. 积分工作带（0.5cm~4cm）+ 抗饱和
    uint8_t freeze = (ae1 > INT_BAND_HI)    // 误差太大：冻结，防大偏差时积分乱卷
                  || (ae1 < INT_BAND_LO)    // 误差太小：停止累积（保持已存值），防零点抖振
                  || (fabsf(tau) >= TAU_MAX - 0.001f);  // 饱和：冻结
    if (!freeze) {
        g.xi += e1 * DT;
        g.xi = clamp(g.xi, -XI_MAX, XI_MAX);
    }
    g.sat = (fabsf(tau) >= TAU_MAX - 0.001f);

    // 7.5 挣脱泄放：球真的连续滚动 = 静摩擦已克服，爬坡攒的积分快速泄掉防过冲
    // ※ 2026-08-07 实测修复v2：kick 微动使 stuck 每拍抖动，!stuck 门控失效，xi 仍被泄光。
    //   改为"持续运动"判定：|dx|>1cm/s 连续 20ms 才泄——kick 抖动是瞬时的，真挣脱是持续的
    static uint8_t move_cnt = 0U;
    if (fabsf(dx) > 0.01f) {
        if (move_cnt < 20U) move_cnt++;
    } else {
        move_cnt = 0U;
    }
    if (move_cnt >= 20U) {
        g.xi *= 0.99f;        // 每拍泄 1%（1kHz 下 ~70ms 泄到一半）
    }

    // 8. SAE: Lyapunov switching barrier plus periodic/state-norm triggers.
    if (g_blending) {
        float V = lyapunov_value(e1, e2, e3, e4);
        if (V > 1.5f * g_v_ss) {
            gain_rollback();
            float tau_safe = -(g_gain_cur.k[0]*e1 + g_gain_cur.k[1]*e2
                             + g_gain_cur.k[2]*e3 + g_gain_cur.k[3]*e4)
                           + g_gain_cur.tau_ff + Ki*g.xi + tau_fric;
            return clamp(tau_safe, -TAU_MAX, TAU_MAX);
        }
    }
    if (!g_blending && !g.sat && fabsf(e1) <= ERR_TRIG && fabsf(e2) < 0.02f) {
        float Vss_sample = lyapunov_value(e1, e2, e3, e4);
        g_v_ss = clamp((1.0f - VSS_ALPHA)*g_v_ss + VSS_ALPHA*Vss_sample,
                       VSS_FLOOR, VSS_INITIAL);
    }
    uint8_t periodic_due = (++g_periodic_steps >= UPDATE_PERIOD_STEPS);
    if (periodic_due) g_periodic_steps = 0U;
    float sensor_norm = sqrtf(e1*e1 + 0.1f*e2*e2
                            + 0.05f*e3*e3 + 0.01f*e4*e4);
    if ((periodic_due || sensor_norm > ERR_TRIG) && g_cooldown == 0U) {
        sae_update_request = 1U;
        g_cooldown = COOLDOWN_STEPS;
        sae_stat_triggers++;
    }
    if (g_cooldown > 0U) g_cooldown--;

    return tau_sat;
}

// ============================================================================
//  初始化
// ============================================================================
void balance_init(void)
{
    g.xi = 0.0f;
    g.dx_f = 0.0f;
    g.dphi_f = 0.0f;
    g.sat = 0;
    memset(g_p_old, 0, sizeof(g_p_old));
    memcpy(g_p_old, g_p_cur, sizeof(g_p_cur));
    memcpy(g_p_new, g_p_cur, sizeof(g_p_cur));
    g_safe_pool_count = 0U;
    (void)safe_pool_admit(&g_gain_cur, FACTORY_MODEL_FITNESS, g_p_cur);
    g_v_ss = VSS_INITIAL;
    g_periodic_steps = 0U;
    g_pending_pool_admit = 0U;
    g_new_pending = 0U;
    g_blending = 0U;
    g_blend_cnt = 0U;
    g_cooldown = 0U;
}

// ============================================================================
//  备份级联 PID（保留旧版，完全不动）
// ============================================================================
void ball_cascade_pid_init(void)
{
    PID_init(&ball_position_pid, PID_POSITION, BALL_POSITION_PID,
             BALL_ANGLE_OFFSET_MAX, 0.10f, BALL_POSITION_INT_SEP);
    PID_init(&ball_angle_pid, PID_POSITION, BALL_ANGLE_PID,
             TAU_MAX, 0.25f, BALL_ANGLE_INT_SEP);
}

void ball_cascade_pid_reset(void)
{
    PID_clear(&ball_position_pid);
    PID_clear(&ball_angle_pid);
}

float ball_cascade_pid_calc(float position, float motor_angle, float target_position)
{
    const float target_angle_offset =
        PID_calc(&ball_position_pid, position, target_position);
    return PID_calc(&ball_angle_pid, motor_angle, PHI0 + target_angle_offset);
}

void balance_reset(void)
{
    g.xi = 0.0f;
}

// ============================================================================
//  全局变量（保留旧版）
// ============================================================================
float x_m   = 0;
float dx_m  = 0;
float phi_rad  = 0;
float dphi_rad = 0;
float tau = 0;

extern volatile float can_a1_position;
extern volatile float can_a1_speed;

volatile LQR_TargetCommand_t lqr_target_command = LQR_TARGET_CENTER;  // 【S3 实验】上电即定点 target=0；恢复比赛行为改回 LQR_TARGET_NONE
volatile float lqr_adjustable_target = 0.0f;
volatile uint8_t trajectory_complete = 0U;
volatile uint8_t trajectory_timeout = 0U;
volatile uint32_t trajectory_elapsed_ms = 0U;

typedef enum
{
    TRAJECTORY_HOLD_CENTER = 0,
    TRAJECTORY_MOVE_POSITIVE,
    TRAJECTORY_MOVE_NEGATIVE
} TrajectoryState_t;

// ============================================================================
//  任务主循环（老结构：阶跃定点目标）
//  ※ SAE 改动仅两处：专用 K1 策略基于混合增益；第 3 步前处理待切换增益
// ============================================================================
void StartTask_2(void const *pvParameters)
{
    LQR_TargetCommand_t applied_command = LQR_TARGET_NONE;
    float control_x;
    float control_dx;
    float target = 0.0f;
    uint32_t cmd_t0 = 0U;
    uint8_t pin1_flash_done = 0U;

    (void)pvParameters;
    HAL_Delay(1000);
    DM_MIT_send_qidon(&hcan1,1);
    HAL_Delay(500);
    balance_init();
    ball_cascade_pid_init();

    while (1)
    {
        // ==================== 1. 读传感器 ====================
        x_m = first_order_filter_calc(&vision_position_filter, can_a1_position);
        dx_m = first_order_filter_calc(&vision_speed_filter, can_a1_speed);

        phi_rad  = feedbackFrame[0].POS;
        dphi_rad = feedbackFrame[0].VEL;

#if SAE_ID_MODE
        // ==================== 辨识模式：脉冲激励 + 100Hz 上报 ====================
        {
            static uint32_t id_t = 0;
            float tau_id = 0.0f;
            uint32_t ph = (id_t / 1000U) % 8U;      // 8 秒一个循环
            switch (ph) {
                case 2: tau_id =  0.04f; break;      // +0.04 N·m 持续 1s
                case 4: tau_id = -0.04f; break;      // -0.04
                case 6: tau_id =  0.06f; break;      // +0.06
                case 7: tau_id = -0.06f; break;      // -0.06
                default: tau_id = 0.0f; break;
            }
            float e3_id = wrap_pi(phi_rad - PHI0);
            if (fabsf(e3_id) > 0.3f) tau_id = 0.0f;  // 安全截止
            static uint8_t div10 = 0;
            if (++div10 >= 10U) {
                div10 = 0;
                uint8_t buf[24];
                float st[5] = {x_m, dx_m, e3_id, dphi_rad, tau_id};
                buf[0]=0xAA; buf[1]=0x55; buf[2]=0x5B;
                memcpy(&buf[3], st, 20);
                uint8_t sum=0; for(int i=0;i<23;i++) sum+=buf[i];
                buf[23]=sum;
                HAL_UART_Transmit(&SAE_HUART, buf, 24, 5);
            }
            DM_MIT_send(&hcan1, 1, 0.0f, 0.0f, 0.0f, 0.0f, tau_id);
            test_time++;
            HAL_Delay(1);
            continue;
        }
#endif

        // ==================== 2. 目标（阶跃定点，老结构） ====================
        LQR_TargetCommand_t command = lqr_target_command;
        if (command != applied_command)
        {
            balance_reset();
            ball_cascade_pid_reset();
            cmd_t0 = HAL_GetTick();
            applied_command = command;
            pin1_flash_done = 0U;
        }
        trajectory_elapsed_ms = HAL_GetTick() - cmd_t0;
        if (command == LQR_TARGET_PIN1_SEQUENCE && trajectory_elapsed_ms >= 5000U && !pin1_flash_done)
        {
            oled_timing_enabled = 0U;
            oled_stop_flash_requested = 1U;
            pin1_flash_done = 1U;
        }

        control_x = x_m;
        control_dx = dx_m;
        g_k1_profile = K1_PROFILE_RUNTIME;
        g_k1_adjust_ratio = 0.0f;
        switch (command)
        {
            case LQR_TARGET_CENTER:
                target = 0.0f;
                break;
            case LQR_TARGET_POSITIVE:
                target = TARGET_POSITIVE;
                break;
            case LQR_TARGET_NEGATIVE:
                target = TARGET_NEGATIVE;
                break;
            case LQR_TARGET_PIN1_SEQUENCE:
                if (trajectory_elapsed_ms < 2500U) {
                    target = TARGET_POSITIVE;
                    g_k1_profile = K1_PROFILE_FIXED;
                    g_k1_fixed = -7.00f;
                } else {
                    target = TARGET_NEGATIVE;
                }
                break;
            case LQR_TARGET_ADJUSTABLE:
                target = lqr_adjustable_target;
                if (target > 0.0f)
                {
                    g_k1_profile = K1_PROFILE_ADJUSTABLE;
                    g_k1_adjust_ratio = clamp(target / 0.125f, 0.0f, 1.0f);
                }
                break;
            case LQR_TARGET_NONE:
            default:
                target = 0.0f;
                control_x = 0.0f;
                control_dx = 0.0f;
                break;
        }

        // ==================== 2.4 【轨迹实验模式】周期 ±50mm 平滑轨迹 ====================
        // 期刊实验 E-TRAJ：固定目标模式下叠加周期轨迹，考察失配下的跟踪能力。
        // 1 = 启用（轨迹实验两组都烧这版）；0 = 恢复定点（S3 实验/比赛行为）
#define TRAJ_EXPERIMENT   (1U)
#define TRAJ_AMP          (0.050f)   // 轨迹幅值 ±50 mm
#define TRAJ_HOLD_MS      (4000U)    // 每端停留 4 s
#define TRAJ_TRANS_MS     (2000U)    // 余弦过渡 2 s
#if TRAJ_EXPERIMENT
        {
            const uint32_t period = 2U * (TRAJ_HOLD_MS + TRAJ_TRANS_MS);   // 12 s
            uint32_t ph = HAL_GetTick() % period;
            float dir;
            if (ph < TRAJ_HOLD_MS) {                                 dir = 1.0f;   // 停在 +50mm
            }
            else if (ph < TRAJ_HOLD_MS + TRAJ_TRANS_MS) {                        // + → −
                float s = (float)(ph - TRAJ_HOLD_MS) / (float)TRAJ_TRANS_MS;
                dir = 1.0f - (1.0f - cosf(s * 3.14159265f));         // 余弦过渡 1→-1
            }
            else if (ph < 2U * TRAJ_HOLD_MS + TRAJ_TRANS_MS) {       dir = -1.0f;  // 停在 −50mm
            }
            else {                                                            // − → +
                float s = (float)(ph - 2U * TRAJ_HOLD_MS - TRAJ_TRANS_MS) / (float)TRAJ_TRANS_MS;
                dir = -1.0f + (1.0f - cosf(s * 3.14159265f));        // −1→+1
            }
            target = TRAJ_AMP * dir;
        }
#endif

        // ==================== 2.5 SAE：待切换增益处理 ====================
        if (g_new_pending)
        {
            g_new_pending = 0U;
            if (!g_blending &&
                gain_sane(&g_gain_new) &&
                isfinite(g_gain_new_fitness) &&
                matrix_spd4(g_p_new) &&
                safe_pool_can_admit(g_gain_new_fitness))
            {
                g_gain_old  = g_gain_cur;
                memcpy(g_p_old, g_p_cur, sizeof(g_p_cur));
                g_blending  = 1U;
                g_blend_cnt = 0U;
                g_pending_pool_admit = 1U;
            }
            else
            {
                sae_stat_rejected++;       // 不合格直接丢弃，现役增益继续服役
            }
        }
        if (g_blending && ++g_blend_cnt >= BLEND_STEPS)
        {
            if (g_pending_pool_admit &&
                safe_pool_admit(&g_gain_new, g_gain_new_fitness, g_p_new))
            {
                g_gain_cur = g_gain_new;
                memcpy(g_p_cur, g_p_new, sizeof(g_p_cur));
                sae_stat_switches++;
            }
            else
            {
                sae_stat_rejected++;
            }
            g_pending_pool_admit = 0U;
            g_blending = 0U;
            g_blend_cnt = 0U;
        }
        if (sae_update_request)
        {
            sae_update_request = 0;
            sae_send_update_request(control_x, control_dx,
                                    wrap_pi(phi_rad - PHI0), dphi_rad,
                                    g.xi, target, tau);
        }

        // ==================== 2.5 实验数据流：20Hz 连续上报（0x5C 帧） ====================
        // 论文主图需要 x(t) 连续曲线，触发帧只有事件散点不够。
        // 帧格式与 0x5A 相同（7×f32），仅帧头不同；PC 端只记录、不触发重优化。
        // 带宽：32B × 20Hz = 640B/s，115200 波特绰绰有余。
        {
            static uint8_t div50 = 0;
            if (++div50 >= 50U) {
                div50 = 0;
                uint8_t sbuf[32];
                float sst[7] = {control_x, control_dx, wrap_pi(phi_rad - PHI0),
                                dphi_rad, g.xi, target, tau};
                sbuf[0]=0xAA; sbuf[1]=0x55; sbuf[2]=0x5C;
                memcpy(&sbuf[3], sst, 28);
                uint8_t ssum=0; for(int i=0;i<31;i++) ssum+=sbuf[i];
                sbuf[31]=ssum;
                HAL_UART_Transmit(&SAE_HUART, sbuf, 32, 5);
            }
        }

        // ==================== 3. 计算力矩 ====================
        if (command == LQR_TARGET_NONE)
        {
            /* Keep the gravity compensation active while the LQR motor is stopped. */
            tau = TAU_FF;
        }
        else
        {
            tau = balance_control(control_x, control_dx, phi_rad, dphi_rad, target);
        }

        if (tracking_enabled && (tracking_run_mode == 1U || tracking_run_mode == 2U))
        {
            tau = clamp(tau + TRACKING_START_TAU_FF, -TAU_MAX, TAU_MAX);
        }

        // ==================== 4. 发送达妙 MIT ====================
        DM_MIT_send(&hcan1, 1, 0.0f, 0.0f, 0.0f, 0.0f, tau);

        // ==================== 5. 周期对齐 ====================
        test_time++;
        HAL_Delay(1);
    }
}
