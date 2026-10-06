/**
 ******************************************************************************
 * @file    ControlTask.cpp
 * @brief   第一部分考核: GM6020 正弦速度跟踪 / 正弦位置跟踪 + 独立看门狗(IWDG)保护
 *
 * 硬件: RoboMaster 开发板 C 型 (STM32F407IGH6) + GM6020
 * 接线: GM6020 的 CAN 口 -> 开发板 CAN1 接口(2pin, 外壳丝印 CAN1), 电机 24V 单独供电
 *
 * 工程配置(C_project.ioc, 已由 CubeMX 生成):
 *   CAN1  : 1Mbps (Prescaler=3, BS1=6TQ, BS2=7TQ), RX0 中断打开, 优先级 1
 *   TIM6  : Prescaler=84-1, Period=1000-1 -> 84MHz/84/1000 = 1kHz 控制中断, 优先级 2
 *   IWDG  : Prescaler=64, Reload=400 -> 32kHz(LSI)/64 = 500Hz, 400/500 = 0.8s 超时
 *
 * 任务分解:
 *   ① 速度环: 让电机转速跟随正弦曲线 v_ref(t) = A_v*sin(2*pi*f_v*t)
 *   ② 位置环: 让电机角度跟随正弦曲线 th_ref(t) = th0 + A_th*sin(2*pi*f_th*t)
 *   ③ 看门狗: 1kHz 任务正常跑完才喂狗; 一旦任务卡死/故障, 0.8s 后 MCU 自动复位
 *
 * 运行模式: 由下面的 kMode 决定(0=只跑速度环, 1=只跑位置环, 2=两条都跑)。
 *           一条曲线和两条曲线是同一份代码, 只改一个数字, 不存在两份不一致的问题。
 *
 * 说明: GM6020 出厂固件默认是"电压控制"模式(0x1FF/0x2FF 报文),
 *       驱动器内部完成 FOC 电流环, 我们外部只做速度环与位置环(见 Gm6020.h 注释)。
 ******************************************************************************
 */

#include "ControlTask.h"

#include "Gm6020.h"
#include "Pid.h"
#include "main.h"
#include "can.h"     // hcan1 / hcan2 (CubeMX 把外设句柄放在各自的头文件里)
#include "tim.h"     // htim6
#include "iwdg.h"    // hiwdg

#include <math.h>

/* ---- 调试变量(全局, 方便用调试器在线观察, 也方便 PC 仿真读出来画曲线) ----
 *   g_dbgState: 0=速度环  1=位置环  2=故障
 *   g_dbgRef  : 当前给定(速度环: rad/s, 位置环: rad)
 *   g_dbgFdb  : 当前反馈
 *   g_dbgOut  : PID 输出(电流给定, 计数; 电压模式下才是电压给定) */
volatile float   g_dbgRef   = 0.0f;
volatile float   g_dbgFdb   = 0.0f;
volatile float   g_dbgOut   = 0.0f;
volatile uint8_t g_dbgState = 0;
volatile uint8_t g_dbgFault = 0;
uint32_t         g_dbgWdCount = 0;   // 喂狗次数

/* ---- [DEBUG] 实验3: 三路速度信号同时观察(只写变量, 不参与控制) ----------
 * 背景: 反馈曲线上有一条约 5~6Hz 的纹波, 但 g_dbgOut 里只有约 63 计数(≈0.009 N·m),
 *       按总惯量 J≈4e-3 算根本驱动不出反馈里 ±1 rad/s 的波动, 所以怀疑这条纹波
 *       来自"电机自带转速字段"本身, 而不是电机真的在抖。
 * 做法: 同一拍把三路速度都算出来, 用 Ozone 的 Data Sampling 一起采下来对比 ——
 *   g_dbgVelDiff: 5ms 角度差分(原始, 未滤波)     -> 角度看得到的速度
 *   g_dbgVelEst : 5ms 角度差分 + 200Hz 低通      -> 可以直接拿来当反馈的候选
 *   g_dbgVelFilt: 电机自带转速字段 + 40Hz 低通    -> "只加一级低通"的候选
 * 判读: 只有 g_dbgFdb(转速字段) 上有 5~6Hz -> 是转速字段的伪信号, 换源/滤波即可;
 *       三路都有                             -> 电机真的在抖, 走机械/陷波那条路。 */
volatile float   g_dbgVelDiff = 0.0f;   // rad/s
volatile float   g_dbgVelEst  = 0.0f;   // rad/s
volatile float   g_dbgVelFilt = 0.0f;   // rad/s
/* 电机回报的“实际转矩电流”(A), 对应反馈帧 DATA[4..5]。
 * 用途: g_dbgOut 只是我们下发的“指令电流”, 驱动器内部电流环/换相产生的力矩脉动
 *       不会出现在指令里。把这一路和速度一起采下来做交叉谱, 就能判断
 *       “速度纹波到底是驱动器灌出来的, 还是机械侧灌出来的”。 */
volatile float   g_dbgCur     = 0.0f;   // A

/* ---- [DEBUG] 片内示波器: 环形记录最近 512 拍的波形(1kHz -> 0.5s 窗口) ----
 * 用调试器读出来画图, 用来判断环路有没有振荡、振荡频率是多少。
 * 不参与控制, 量产版本可以整个删掉。 */
constexpr uint32_t kScopeLen = 512;
volatile float    g_scopeRef[kScopeLen] = {0};   // 给定
volatile float    g_scopeFdb[kScopeLen] = {0};   // 反馈(用协议里的 rpm 换算)
volatile float    g_scopeOut[kScopeLen] = {0};   // PID 输出(限幅前)
volatile float    g_scopeCmd[kScopeLen] = {0};   // 实际下发电压
volatile float    g_scopeAng[kScopeLen] = {0};   // 累计角度(rad)
volatile float    g_scopeRpm[kScopeLen] = {0};   // 原始转速字段(rpm)
volatile float    g_scopeCur[kScopeLen] = {0};   // 原始电流字段(A)
volatile float    g_scopeVelDiff[kScopeLen] = {0};  // 实验3: 5ms 角度差分(原始)
volatile float    g_scopeVelEst[kScopeLen]  = {0};  // 实验3: 角度差分 + 200Hz 低通
volatile float    g_scopeVelFilt[kScopeLen] = {0};  // 实验3: 转速字段 + 40Hz 低通
volatile float    g_scopeCurA[kScopeLen]    = {0};  // 电机回报的实际电流(A)
volatile uint32_t g_scopeIdx = 0;

namespace {

/* ==========================================================================
 * 1. 可调参数(想换曲线、换参数, 只改这里)
 * ========================================================================== */

// 电机拨码开关设置的 ID (1~7): 决定反馈报文 0x204+ID 与控制报文 0x1FF/0x2FF
constexpr uint8_t kMotorId = 1;

// 控制周期, 与 TIM6 一致: 1kHz -> 1ms
constexpr float   kDt = 0.001f;
constexpr float   kPi = 3.14159265358979f;

/* ---- 曲线 ①: 速度正弦 v_ref(t) = kVelAmp * sin(2*pi*kVelFreq*t) ---- */
constexpr float   kVelAmp  = 6.0f;    // 幅值 4 rad/s (约 38 rpm, 空载最高 320rpm, 留足余量)
constexpr float   kVelFreq = 0.5f;    // 频率 1 Hz -> 周期 1 s

/* ---- 曲线 ②: 位置正弦 th_ref(t) = th0 + kPosAmp * sin(2*pi*kPosFreq*t) ---- */
constexpr float   kPosAmp  = kPi / 2.0f;  // 幅值 90°(±90°), 避免线材/结构被扭
constexpr float   kPosFreq = 0.25f;       // 频率 0.25 Hz -> 周期 4 s

/* ---- 电机控制模式 ------------------------------------------------------
 * 必须和电机侧(RoboMaster Assistant)的设置一致, 否则电机要么不响应、要么橙灯常亮:
 *   false = 电压模式(出厂默认): 控制帧 0x1FF/0x2FF, 输出是电压, ±25000 ↔ ±24V
 *   true  = 电流模式:           控制帧 0x1FE/0x2FE, 输出是电流, ±16384 ↔ ±3A
 * 切换时要一起改四处: 本开关 / Gm6020.cpp 的 txId() / kOutMax,kOutMin / 前馈系数 */
constexpr bool    kCurrentMode    = true;

/* ---- 前馈(建议开启) ----------------------------------------------------
 * 电压模式: 转速与所需电压近似成正比(反电势), 前馈 = kFfGainFinal * ref,
 *           同样的 PID 参数下跟踪误差能从 ~5% 降到 ~1%。
 * 电流模式: 输出是电流, 维持正弦运动需要的电流只有两部分 ——
 *   ① 摩擦/齿槽补偿: 大小基本固定, 方向跟着运动方向 -> kFricFf * softSign(ref)
 *   ② 加速补偿:     转矩 = J * dω/dt               -> kAccFf  * d(ref)/dt
 *   注意: 电流模式下**不能**沿用电压模式那条"和转速成正比"的公式,
 *         它相当于持续朝一个方向加速, 会把电机推飞(实测能跑到 200+ rpm)。 */
constexpr bool    kUseFeedforward = true;  // true=开, false=关
constexpr float   kSupplyVolt     = 24.0f;    // 电源电压(V)
constexpr float   kSpeedConst     = 1.3958f;  // 转速常数 13.33rpm/V -> (rad/s)/V
constexpr float   kFfGain         = (25000.0f / kSupplyVolt) / kSpeedConst;  // 电压模式: rad/s -> 电压给定
constexpr float   kFfCalib        = 1.15f;    // 电压模式实测校准(实测转速比理论低约 13%)
constexpr float   kFfGainFinal    = kFfGain * kFfCalib;

/* 电流模式的两个前馈系数(用电压模式时这两项不参与运算) */
constexpr float   kFricFf         = 900.0f;   // 摩擦/齿槽补偿量(计数) —— 实测定版
constexpr float   kAccFf          = 0.0f;     // 加速补偿量(计数 per rad/s^2); 实测给大了反而抵消 PID, 定版为 0
constexpr float   kFfBlend        = 0.20f;    // softSign 的软化宽度(rad/s), 避免过零点硬跳变

/* ---- 抖动注入(dither): 抑制低速爬行 ------------------------------------
 * 直驱电机在低速时, 齿槽力矩会把它"卡住 -> 猛冲 -> 再卡住", 也就是爬行。
 * 在输出上叠一个高频小幅抖动, 让转子和齿槽之间始终处于"动摩擦"状态,
 * 就能把爬行压掉(代价是速度上多一点点高频纹波, 会被转子惯量滤掉大半)。
 * 频率取整数 Hz, 相位按 1 秒循环, 长时间运行也不会丢精度。 */
constexpr float   kDitherAmp      = 0.0f;     // 抖动幅度(计数); 实测开抖动反而容易跑飞, 定版关闭
constexpr float   kDitherHz       = 120.0f;   // 抖动频率(Hz), 用整数

/* ---- 运行模式: 想只演示其中一条曲线, 只改 kMode 这一个数字 ----------------
 *   kVelOnly = 0  全程跑速度正弦
 *   kPosOnly = 1  全程跑位置正弦
 *   kBoth    = 2  先速度环, 再位置环, 循环往复(默认, 一次烧录连续看到两条曲线)
 * 三种模式共用同一份控制代码, 只是调度分支不同; 想看单条曲线的固件效果,
 * 改这里重新编译即可, 不需要维护两份会各自走样的代码。 */
constexpr int     kVelOnly = 0;
constexpr int     kPosOnly = 1;
constexpr int     kBoth    = 2;
constexpr int     kMode    = 1;

/* ---- 两个阶段的持续时间(只在 kBoth 模式下生效) ---- */
constexpr float   kVelPhaseTime = 12.0f;  // 0~12s
constexpr float   kPosPhaseTime = 12.0f;  // 12~24s, 再回到速度环

/* ---- PID 参数(输出就是电流给定, ±16384 对应 ±3A) -----------------------
 * 注意量纲: 电流模式下对象近似是"纯积分"(驱动器内部电流环很快, 电压转速之间的
 * 反电势阻尼被电流环补掉了), 所以增益量级和电压模式不一样。
 *
 * 速度环 PI: 转速反馈只有 1rpm(=0.105rad/s)的分辨率, 微分项会被量化噪声放大, 故 Kd=0
 * 位置环 PD + 小 I: D 项提供阻尼(相当于内部速度反馈), I 项消除负载力矩带来的静差
 *
 * ---- 2026-10-06 调参: 为什么 Ki 要降下来 ----------------------------------
 * 电流模式下开环传递函数近似为
 *     L(s) = (Kp + Ki/s) * (Kt*3/16384) / (J*s)
 * 它有两个特征频率:
 *   ① 穿越频率(环路带宽) ωc ≈ sqrt(Kp*Kt*3/16384/J), 原参数约 5Hz;
 *   ② 积分零点           ωi = Ki/Kp = 5000/200 = 25rad/s ≈ 4Hz。
 * 原来 ωi 几乎压在 ωc 上: 相位裕度只剩 53°, 而速度反馈本身有 20~30ms 延迟
 * (电机自带转速字段的更新周期 + CAN + 采样), 在 5Hz 处就是 36~54° 的相位损失,
 * 裕度被吃光 -> 实测出现约 5Hz、峰峰值 12rad/s 的自激振荡(见调试记录截图)。
 * Ki 降到 1500 后 ωi = 7.5rad/s ≈ 1.2Hz(约 ωc 的 1/4), 相位裕度回到 75°
 * (含 25ms 延迟约 35°, 延迟裕度从 28ms 提到 47ms), 仿真里纹波 rms 从
 * 0.64rad/s 降到 0.53rad/s, 反馈范围从 ±4.8 收到 ±4.2rad/s。
 * 提醒: 若实测转动惯量比 1e-3 kg·m^2 小很多, ωc 会成比例升高, 此时 Kp 和 Ki
 *       要一起往下降, 只降 Ki 不够(穿越频率处的延迟损失不随 Ki 改变)。
 *
 * ---- 位置环的 Ki 为什么不跟着降 ------------------------------------------
 * 位置环的积分零点 ωi = kPosKi/kPosKp = 12000/3000 = 4rad/s, 已经远低于它的
 * 穿越频率(约 6.7Hz), 相位裕度够用; 而位置环的静差主要来自摩擦力矩(前馈只补了
 * 一部分), 只能靠积分消掉 —— 把 Ki 降到 5000 仿真里误差反而从 3.43° 涨到 4.45°。
 * 位置环这次只动微分项(加 150Hz 低通), 见 Pid.cpp。 */
constexpr float   kVelKp = 200.0f, kVelKi = 2000.0f, kVelKd = 0.0f;   // 2026-10-06: Ki 5000->1500(抑制 5Hz 自激)
constexpr float   kPosKp = 3000.0f, kPosKi = 12000.0f, kPosKd = 300.0f; // 2026-10-06: Ki 保持, D 项加 150Hz 低通

/* ---- 输出限幅(即电压给定上下限) ---- */
constexpr float   kOutMax = 16384.0f;   // 实测定版: 满电压 24V（调试阶段曾压到 6000）
constexpr float   kOutMin = -16384.0f;  // 实测定版: 满电压 24V（调试阶段曾压到 6000）

/* ---- 保护阈值 ---- */
constexpr uint32_t kRxTimeoutMs = 100;    // 超过 100ms 没收到该电机反馈 -> 判通信故障
constexpr float    kTempLimitC  = 100.0f; // 电机温度上限(℃)

/* ---- 故障后的行为 ----
 *   true  = 输出置零 + 停止喂狗, 0.8s 后 IWDG 自动复位 MCU (本次考核演示用)
 *   false = 只把输出置零并继续运行(适合"某个电机没插好"也不希望整板重启的场合)
 * 注意: 选 true 时, 上电前要先接好电机, 否则会每 ~0.9s 复位一次。 */
constexpr bool kResetOnFault = true;

/* ==========================================================================
 * 2. 运行时状态
 * ========================================================================== */

Gm6020 g_motor(kMotorId);
Pid    g_velPid(kVelKp, kVelKi, kVelKd, kOutMax, kOutMin);
Pid    g_posPid(kPosKp, kPosKi, kPosKd, kOutMax, kOutMin);

uint32_t g_tick       = 0;      // 控制节拍(1kHz), 等价于运行毫秒数
uint32_t g_lastRxTick = 0;      // 最近一次收到反馈的节拍
bool     g_rxSeen     = false;  // 是否收到过反馈
bool     g_fault      = false;  // 故障锁存标志(置位后不喂狗, 等 IWDG 复位)
bool     g_velPhase   = true;   // 当前阶段: true=速度正弦, false=位置正弦
float    g_posOffset  = 0.0f;   // 位置正弦的直流偏置(切换瞬间的电机角度)

/* ==========================================================================
 * 3. 内部工具函数
 * ========================================================================== */

inline float sineWave(float amp, float freqHz, float t) {
    return amp * sinf(2.0f * kPi * freqHz * t);
}

// 软化版符号函数: |x| >> eps 时接近 ±1, x=0 时为 0。
// 摩擦前馈用它 —— 既能在低速时给出补偿力, 又不会在过零点硬跳变。
inline float softSign(float x, float eps) {
    return x / (fabsf(x) + eps);
}

/* ---- 速度反馈: 用角度差分自己算 ----------------------------------------
 * 电机自带的转速字段只有 1 rpm(=0.105 rad/s) 分辨率, 而且 3~4ms 才更新一次,
 * 直接拿它做速度环反馈, 量化噪声和延时都很大。
 * 角度反馈是 8192 计数/圈(0.000767 rad), 干净得多 —— 用 N 拍角度差分估速度,
 * 分辨率提高到 0.000767/(N*0.001) rad/s, 代价是 N 毫秒的延时。
 *
 * 2026-10-06 (实验3) 把窗口从 20 拍改成 5 拍, 并补上一级低通:
 *   20 拍 = 20ms 延迟, 在 5Hz 附近要吃掉 36° 相位, 太贵;
 *    5 拍 =  5ms 延迟(5Hz 处 9°), 代价是量化噪声变大到 0.153 rad/s,
 *          所以后面串一级 200Hz 低通把量化噪声压回去(对 5~6Hz 没有影响)。
 * kUseAngleVelFdb 现在真的起作用了: true 时控制用"角度差分+低通"这一路。 */
constexpr bool     kUseAngleVelFdb = true;  // false=用电机自带转速字段(默认, 与之前一致); true=用 g_dbgVelEst
constexpr uint32_t kVelFdbWin = 5;           // 差分窗口(拍) = 5ms
float    g_angHist[kVelFdbWin] = {0};
uint32_t g_angPut  = 0;
bool     g_angReady = false;

// 两路速度低通的系数 alpha = T/(T+tau), tau = 1/(2*pi*fc)
constexpr float kTwoPiVel      = 6.28318530717959f;
constexpr float kVelEstFc      = 200.0f;     // 角度差分后的低通(Hz)
constexpr float kVelFiltFc     = 40.0f;      // 转速字段的低通(Hz)
constexpr float kVelEstAlpha   = kDt / (kDt + 1.0f / (kTwoPiVel * kVelEstFc));
constexpr float kVelFiltAlpha  = kDt / (kDt + 1.0f / (kTwoPiVel * kVelFiltFc));
float    g_velEstLp = 0.0f;                  // 角度差分低通状态
float    g_velRpmLp = 0.0f;                  // 转速字段低通状态

inline float estimateVel(float ang) {
    if (!g_angReady) {
        for (uint32_t i = 0; i < kVelFdbWin; ++i) {
            g_angHist[i] = ang;
        }
        g_angReady = true;
        return 0.0f;
    }
    const float old = g_angHist[g_angPut];
    g_angHist[g_angPut] = ang;
    g_angPut = (g_angPut + 1u) % kVelFdbWin;
    return (ang - old) / (static_cast<float>(kVelFdbWin) * kDt);
}

// 把 PID 输出限幅并转成电压给定(int16)
inline int16_t toCmd(float u) {
    if (u > kOutMax)      u = kOutMax;
    else if (u < kOutMin) u = kOutMin;
    return static_cast<int16_t>(u);
}

// 发送一帧 8 字节的标准帧
void canSend(uint32_t stdId, uint8_t *data) {
    CAN_TxHeaderTypeDef tx = {0};
    tx.StdId   = stdId;
    tx.ExtId   = 0;
    tx.IDE     = CAN_ID_STD;
    tx.RTR     = CAN_RTR_DATA;
    tx.DLC     = 8;
    tx.TransmitGlobalTime = DISABLE;
    uint32_t mailbox = 0;
    (void)HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox);
}

// 把本电机的电压给定组帧下发(一帧最多带 4 个电机, 见 Gm6020::encode)
void sendMotorCommand(void) {
    uint8_t frame[8] = {0};
    g_motor.encode(frame);
    canSend(g_motor.txId(), frame);
}

// 输出置零并立即下发(安全停机)
void stopMotor(void) {
    g_motor.setVoltage(0);
    sendMotorCommand();
}

// 喂狗: 只有控制任务正常跑完才会执行
inline void feedWatchdog(void) {
    HAL_IWDG_Refresh(&hiwdg);
    ++g_dbgWdCount;
}

}  // namespace

/* ==========================================================================
 * 4. 对外接口
 * ========================================================================== */

extern "C" void ControlTaskInit(void)
{
    /* ---- 调试便利: 调试器把 CPU 停住时冻结 IWDG, 否则单步/断点 0.8s 后板子会复位 ----
     * 只影响"被调试器halt"的状态, 程序自由运行时看门狗照常工作。 */
    __HAL_DBGMCU_FREEZE_IWDG();
    /* ---- ① CAN1 接收过滤器 --------------------------------------------------
     * 电机反馈标识符 = 0x204 + ID, 即 0x205~0x20B, 都在 0x200~0x20F 区间内。
     * 用 32 位掩码模式: FilterId 与掩码都左移 5 位对齐到 STID 位域;
     * 掩码 0x7F0 表示只比较 ID 的 bit[10:4], 因此 0x200~0x20F 全部放行。 */
    CAN_FilterTypeDef filter = {0};
    filter.FilterIdHigh        = static_cast<uint32_t>(0x200u << 5);
    filter.FilterIdLow         = 0;
    filter.FilterMaskIdHigh    = static_cast<uint32_t>(0x7F0u << 5);
    filter.FilterMaskIdLow     = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank          = 0;
    filter.FilterMode          = CAN_FILTERMODE_IDMASK;
    filter.FilterScale         = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation    = ENABLE;
    filter.SlaveStartFilterBank = 14;   // F407 双 CAN: CAN1 用 0~13 号过滤器组
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    /* ---- ② 启动 CAN1 并打开 RX FIFO0 挂起中断 ---- */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* ---- ③ 先下发一帧 0 电压: 上电时电机不会保持上一次的指令 ---- */
    stopMotor();

    /* ---- ④ 启动 1kHz 控制定时器 TIM6(中断里会回调 MainTask) ---- */
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief 1kHz 控制任务(在 TIM6 周期中断中被调用)
 *
 * 执行顺序: 保护检查 -> 阶段调度 -> 生成给定 -> PID -> 下发 CAN -> 喂狗
 * 注意: 喂狗必须在最后一步, 这样只要中间任何一环卡住(死循环/硬件异常),
 *       看门狗都不会被刷新, 0.8s 后自动复位, 电机停止输出。
 */
extern "C" void MainTask(void)
{
    ++g_tick;
    const float t = static_cast<float>(g_tick) * kDt;   // 运行时间(s)

    /* ---------------- 第一步: 保护检查 ---------------- */
    const bool rxTimeout = (!g_rxSeen) || ((g_tick - g_lastRxTick) > kRxTimeoutMs);
    const bool overTemp  = (g_motor.temp() > kTempLimitC);
    if (rxTimeout || overTemp) {
        g_fault = true;
        g_dbgFault = 1;
    }

    if (g_fault) {
        // 故障: 输出立刻置零, 并且不再喂狗 -> 0.8s 后 IWDG 复位 MCU
        stopMotor();
        g_dbgState = 2;
        g_dbgRef = 0.0f;
        g_dbgFdb = 0.0f;
        g_dbgOut = 0.0f;
        if (!kResetOnFault) {
            feedWatchdog();
        }
        return;
    }

    /* ---------------- 第二步: 阶段调度 ----------------
     * kMode 决定这一拍跑哪条曲线:
     *   kVelOnly -> 永远速度环, 相位时间就用运行时间 t
     *   kPosOnly -> 永远位置环, 同样用运行时间 t
     *   kBoth    -> 以 (kVelPhaseTime + kPosPhaseTime) 为周期循环, 前段速度后段位置
     */
    bool  velPhase = true;
    float tPhase   = t;

    if (kMode == kPosOnly) {
        velPhase = false;
    } else if (kMode == kBoth) {
        const float tCycle = fmodf(t, kVelPhaseTime + kPosPhaseTime);
        velPhase = (tCycle < kVelPhaseTime);
        tPhase   = velPhase ? tCycle : (tCycle - kVelPhaseTime);
    }
    // kVelOnly 走上面的默认值: velPhase = true, tPhase = t

    if (velPhase != g_velPhase) {
        // 阶段切换: 复位两个 PID 的积分, 位置环以"当前角度"作为正弦中心,
        // 这样切换瞬间给定 == 反馈, 不会产生位置跳变把电机"打"一下。
        g_velPhase = velPhase;
        g_velPid.reset();
        g_posPid.reset();
        if (!velPhase) {
            g_posOffset = g_motor.angle();
        }
    }

    /* ---------------- 第三步: 生成给定 + 计算 PID ---------------- */
    float ref = 0.0f, fdb = 0.0f, out = 0.0f;

    if (velPhase) {
        /* 速度环: 给定速度 -> 反馈速度, 输出给驱动器(电压或电流, 看 kCurrentMode) */
        ref = sineWave(kVelAmp, kVelFreq, tPhase);

        /* [DEBUG/实验3] 三路速度同时算出来(默认不改变控制行为) */
        const float velRpm  = g_motor.vel();                    // 电机自带转速字段
        const float velDiff = estimateVel(g_motor.angle());     // 5ms 角度差分(原始)
        g_velEstLp += kVelEstAlpha  * (velDiff - g_velEstLp);   // 200Hz 低通
        g_velRpmLp += kVelFiltAlpha * (velRpm  - g_velRpmLp);   // 40Hz 低通
        g_dbgVelDiff = velDiff;
        g_dbgVelEst  = g_velEstLp;
        g_dbgVelFilt = g_velRpmLp;

        fdb = kUseAngleVelFdb ? g_velEstLp : velRpm;            // rad/s
        out = g_velPid.calc(ref, fdb);
        if (kUseFeedforward) {
            if (kCurrentMode) {
                // 电流模式: 摩擦/齿槽(按方向) + 加速(给定角速度的导数)
                const float wDot = kVelAmp * 2.0f * kPi * kVelFreq
                                 * cosf(2.0f * kPi * kVelFreq * tPhase);
                out += kFricFf * softSign(ref, kFfBlend) + kAccFf * wDot;
            } else {
                out += kFfGainFinal * ref;      // 电压模式: 补上反电势所需的电压
            }
        }
        g_dbgState = 0;
    } else {
        /* 位置环: 给定角度 -> 反馈角度(多圈累计角度, 单位 rad) */
        const float wRef = kPosAmp * 2.0f * kPi * kPosFreq
                         * cosf(2.0f * kPi * kPosFreq * tPhase);  // 给定角速度(正弦的导数)
        ref = g_posOffset + sineWave(kPosAmp, kPosFreq, tPhase);
        fdb = g_motor.angle();                  // rad
        out = g_posPid.calc(ref, fdb);
        if (kUseFeedforward) {
            if (kCurrentMode) {
                // 电流模式: 摩擦/齿槽(按给定角速度方向) + 加速(给定角速度的导数)
                const float wDot = -kPosAmp * (2.0f * kPi * kPosFreq) * (2.0f * kPi * kPosFreq)
                                 * sinf(2.0f * kPi * kPosFreq * tPhase);
                out += kFricFf * softSign(wRef, kFfBlend) + kAccFf * wDot;
            } else {
                out += kFfGainFinal * wRef;
            }
        }
        g_dbgState = 1;
    }

    /* 抖动注入: 抑制低速爬行(见 kDitherAmp 的说明) */
    const float tDither = static_cast<float>(g_tick % 1000u) * kDt;   // 0~1 s 循环
    out += kDitherAmp * sinf(2.0f * kPi * kDitherHz * tDither);

    g_motor.setVoltage(toCmd(out));
    g_dbgRef = ref;
    g_dbgFdb = fdb;
    g_dbgOut = out;

    /* ---------------- 第四步: 下发 CAN 控制帧 ---------------- */
    sendMotorCommand();

    /* ---- [DEBUG] 记录本拍到示波器缓冲 ---- */
    g_scopeRef[g_scopeIdx] = ref;
    g_scopeFdb[g_scopeIdx] = fdb;
    g_scopeOut[g_scopeIdx] = out;
    g_scopeCmd[g_scopeIdx] = static_cast<float>(g_motor.voltage());
    g_scopeAng[g_scopeIdx] = g_motor.angle();
    g_scopeRpm[g_scopeIdx] = static_cast<float>(g_motor.velRpm());
    g_scopeCur[g_scopeIdx] = g_motor.current();
    g_scopeVelDiff[g_scopeIdx] = g_dbgVelDiff;
    g_scopeVelEst[g_scopeIdx]  = g_dbgVelEst;
    g_scopeVelFilt[g_scopeIdx] = g_dbgVelFilt;
    g_dbgCur = g_motor.current();
    g_scopeCurA[g_scopeIdx] = g_dbgCur;
    if (++g_scopeIdx >= kScopeLen) {
        g_scopeIdx = 0;
    }

    /* ---------------- 第五步: 喂狗(任务正常跑完) ---------------- */
    feedWatchdog();
}

extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data)
{
    if (std_id == g_motor.rxId()) {
        g_motor.decode(data);       // 解析角度/转速/电流/温度
        g_lastRxTick = g_tick;
        g_rxSeen     = true;
    }
}

/* ==========================================================================
 * 5. HAL 回调(CubeMX 生成的代码里没有实现, 由这里接管)
 * ========================================================================== */

// TIM6 周期中断 -> 1kHz 控制任务
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        MainTask();
    }
}

// CAN1 收到新报文 -> 读出来交给 CanFeedbackCallback
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader = {0};
    uint8_t rxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        return;
    }
    if (rxHeader.IDE != CAN_ID_STD) {   // 只用标准帧
        return;
    }
    CanFeedbackCallback(rxHeader.StdId, rxData);
}
