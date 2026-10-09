/*
 * velocity_control.c - Custom H-frame velocity controller for multirotor interceptors
 *
 * 方案B：INAV 仅保留 ANGLE 姿态内环；本模块在飞控上自建三轴速度环。
 *
 * 架构：
 *   - 外部指令 H 系（x 前 / y 右 / z 天，cm/s）
 *   - 实测速度取 posEstimator.est.vel（地固 NEU），用航向投影到 H 系
 *   - 水平两轴：速度误差 --PI--> 期望水平加速度(cm/s^2) --> atan2 求目标 pitch/roll
 *   - 垂直轴：速度误差 --PI--> 油门修正(µs)，叠加悬停油门
 *   - 绕推力轴的滚转：yaw 通道把机体系 +Y 压回水平（xoz 平面竖直），见下
 *   - 200Hz 计算，结果缓存；每个 PID(1kHz) 周期末尾覆盖 rcCommand
 *
 * 注意：以下 PID 增益为【起始值】，必须在台架/试飞中整定，非经验证的最终值。
 *
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"
#include "common/time.h"

#include "drivers/time.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/mixer.h"
#include "flight/velocity_control.h"

#include "fc/control_profile.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "navigation/navigation_pos_estimator_private.h"

/* ---------------- 参数（起始值，需整定） ---------------- */

#define VEL_TIMEOUT_US              500000      // 速度指令超时：500ms 无更新则解除接管
#define VEL_FEEDBACK_LPF_HZ         15.0f       // 反馈速度环内低通截止频率
/* 水平加速度输出上限（cm/s²）。它直接决定**巡航倾角**：φ_des = atan(aFwd/(g+a_v))，
 * 而本机"推力方向 = 加速度方向"（docs/ue_operations.md §6.2），所以倾角越大推力越接近水平、
 * 速度越快。原来取 350（0.36g≈20°）时，20 m/s 巡航配平只需 ~11°（0.19g），
 * 速度只能在 ~20 m/s 附近饱和。
 * 取 1470 = **1.5g ≈ 56° 倾角**：55 m/s 巡航的阻力约 1.44g（由 20 m/s 时 0.19g 按 v² 外推），
 * 1.5g 刚好够用；再大受**垂向权限**限制 —— 倾角 θ 下保持高度需要 T = g/cosθ，
 * 56° 时 1.79g、60° 时 2.0g，而垂速环向上的权限是 +150 µs ≈ 2.25g（60° 以上会顶住）。
 * 机身姿态本身还能到 80°（eeprom `max_angle_inclination_pit=80`，§6.2），
 * 但"保持高度"的物理上限就在 60° 附近。 */
#define VEL_MAX_HORIZ_ACC_CMSS      3500.0f
#define VEL_MAX_HORIZ_VEL_CMS       12000.0f    // 水平速度目标上限（>90m/s，留余量）
#define VEL_MAX_VERT_VEL_CMS        3000.0f     // 垂直速度目标上限（±30m/s）

/* 绕推力轴的滚转保持（见文件末尾的说明）。增益单位 dps/deg。 */
#define VEL_ROLL_HOLD_KP            3.0f
#define VEL_ROLL_HOLD_SIGN          1.0f        // 若实测该自由度发散，改 -1
/* 推力轴水平分量低于此值时"滚转保持"退化（悬停），不作用。 */
#define VEL_ROLL_HOLD_MIN_TILT      0.15f

/* 自动 yaw：把机头转向**指令水平速度方向**（H 系里该方位 -> 0），dps per deg。
 * 符号：psiCmd > 0 = 指令在右侧 => 右偏航 = 负的 yaw 角速率（本机 yaw 正 = 左）。 */
#define VEL_YAW_TURN_KP             2.5f
/* "先对准、再前推"的对准带（deg）：方位误差大于它时**完全不给水平推力**，只原地转向。
 * 见 taskVelocityControl() 里 aFwd 那段的实测故障链：按 cos 缩放不足以消除激励。 */
#define VEL_ALIGN_BAND_DEG          15.0f
/* 指令方位一阶低通时间常数（s）：压掉近距离位置量化造成的目标方位跳变（见 psiCmd 处）。
 * 太大则变方位航段对准变慢，0.5 s 对 30 dps 的对准几乎无影响。 */
#define VEL_YAW_DIR_LPF_TAU_S       0.5f
/* 偏航角速率硬限幅（dps）。不许侧倾时转太快会把车甩出去：body 系恒定的斜向指令
 * 按物理就是一个圆，实测 112 dps 时转弯诱导滚转把 xoz 冲到 0.63、随后整体翻转。
 * 现在"先对准再前推"是**严格**的（方位误差 > VEL_ALIGN_BAND_DEG 时 aFwd 恒为 0），
 * 转向全部在**悬停原地**完成，不再有"边飞边转"的弧线，所以角速率只影响转向耗时、
 * 不影响航迹形状：180° 最坏 6 s，相对 500~1000 m 航段可忽略。
 * 取 30 dps（不是 60）：实测 leg1（播种航向 90°、目标正北）在 60 dps + 6.7° 倾角
 * 同时作用下 3 s 内抬头 75°、翻滚到 42 m/s（VC 日志 pd=6.7 pa=0.2 ryaw=60.0
 * -> pa=-11.8 -> -39.7 -> -75.4 -> 105.5）。转向留出裕度。 */
#define VEL_YAW_MAX_DPS             30.0f
/* 期望倾角的最大变化率（deg/s）。低空保护解除、航段切换时 φ_des 会从 0 阶跃到
 * 十几度，内环速率环被阶跃激励会振荡——实测 Test1 起飞后 5~9 s 直接翻过去
 * （pa 从 -15° 跳到 +95°、yaw 1 s 内转 228°，而当时 pd 只有 0.7~1.5°、thr 也只
 * 比悬停高 30 µs，即不是推力耦合、是内环被阶跃激励）。限 30°/s 后 350 cm/s²
 * 的指令约 0.7 s 到位，不影响正常机动。 */
#define VEL_PHI_SLEW_DPS            30.0f
/* 指令水平速度低于此值时方位无意义（悬停），保持当前航向（cm/s）。
 * 试过压到 10（想在最后几米继续对准），实测**反而变差**：末端一直在修正方位，
 * 位置量化（±0.4~0.7 m，见 docs/ue_operations.md §4.10）让目标方位在近距离来回跳，
 * 飞机在目标附近画圈 —— leg1 最近接近从 0.74 m 退化到 9.33 m、横向残差 15.3 m。
 * 回到 30：末端不修正方位、直接飞过去，反而稳。
 *
 * 2026-10-09 试过提到 800（≈32 m 处开始冻结航向），**实测更差**：leg1 1.09→4.01、
 * leg2 0.37→5.25 m。原因是冻结航向后"指向目标"的方位修正也一起没了，而指令仍在
 * 随目标方位转动 ⇒ 机体系指令变成以横向为主，前向通道收油、飞机带着动量横着漂过去。
 * 末端残差真正的病根是"50° 左右倾角下靠转头做横向修正"（见 §7.7.2/§7.7.4），
 * 要靠**降低末端速度**去解，不是靠冻结航向。回到 100。 */
#define VEL_YAW_MIN_TGT_CMS         100.0f
/* 俯仰层：带符号倾角误差(deg) -> 低头为正的角速率(dps) 的比例增益。
 *
 * 实测（2026-10-09，fly_vel=55 的高速巡航）：增益 1.0 时**实测倾角长期落后期望
 * 8~10°**（`pd=36~44° / pa=27~33°`），而倾角就是推力方向 ⇒ 水平分量只有
 * g·tan30°≈0.58g，正好被 29.5 m/s 的阻力吃掉，速度卡在 29.5 m/s 上不去。
 * 落后的原因是配平气动俯仰力矩（§6.2 的静稳定裕度）由内环速率环的积分项扛着，
 * 外环 P 只有 1.0 时给出的补偿角速率太小（10° -> 10 dps）。
 * 提到 3.0：10° 误差给 30 dps，倾角能跟上期望、速度才上得去。
 * 旧注释里"4.0/1.5 会诱发内环振荡（实测起飞后翻转）"是**符号错误时期**的结论
 * （当时俯仰外环是正反馈，见 phiActDeg 处），两个符号修好之后不再适用。 */
#define VEL_ATT_KP                  3.0f

/* ---------------- 内部 PID 结构 ---------------- */

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prevError;
    float iLimit;       // 积分项限幅（输出单位）
    float dLimit;       // 微分项变化率限幅
    float outMin;
    float outMax;
} velPid_t;

static float velPidApply(velPid_t *p, float target, float measurement, float dt)
{
    const float error = target - measurement;

    // P
    const float pTerm = p->kp * error;

    // I（含抗积分饱和限幅）
    if (p->ki > 0.0f && dt > 0.0f) {
        p->integrator += p->ki * error * dt;
        p->integrator = constrainf(p->integrator, -p->iLimit, p->iLimit);
    }

    // D（默认 kd=0；对误差微分并限变化率，抑制噪声）
    float dTerm = 0.0f;
    if (p->kd != 0.0f && dt > 0.0f) {
        float deriv = (error - p->prevError) / dt;
        deriv = constrainf(deriv, -p->dLimit, p->dLimit);
        dTerm = p->kd * deriv;
    }
    p->prevError = error;

    return constrainf(pTerm + p->integrator + dTerm, p->outMin, p->outMax);
}

/* ---------------- 模块状态 ---------------- */

static struct {
    bool        active;
    timeUs_t    lastCmdUs;          // 最近一次速度指令时间戳
    timeUs_t    lastRunUs;          // 最近一次任务执行时间戳（用于 dt）

    // 速度目标（H 系，x 前 / y 右 / z 天，cm/s）
    float       targetFwd;
    float       targetRight;
    float       targetUp;

    // 指令方位的一阶低通状态（单位方向分量；对分量滤波，±180° 环绕天然安全）
    float       dirFwd;
    float       dirRight;

    // 控制器
    velPid_t    pidFwd;
    velPid_t    pidRight;
    velPid_t    pidUp;

    // 反馈滤波器
    pt1Filter_t fbFwd;
    pt1Filter_t fbRight;
    pt1Filter_t fbUp;

    // 缓存输出（rcCommand 单位）
    int16_t     outRoll;
    int16_t     outPitch;
    int16_t     outYaw;
    int16_t     outThrottle;

    // 诊断（1Hz 遥测用）
    float       rollHoldPhiDeg;
    float       phiDesPrevDeg;   // 上一周期期望倾角（速率限幅用）
} vel = {
    /* 水平速度环：kp = 0.35（单位 cm/s -> cm/s^2）。
     * 姿态律改成"带符号俯仰"之后这个环才真正闭环，原先按坏律标定的 0.7 偏热：
     * 5 m/s 指令下倾角需求 ~10°，但 30 m/s 的过渡误差会直接顶到限幅、把倾角
     * 打到最大并诱发振荡（实测 4~5 段出现过 pa=178.9° 的翻转）。 */
    /* 前向速度环：kp 0.35 -> 0.8，为了让"高速巡航时加速度指令打到限幅"。
     * 实测 kp=0.35、目标 55 m/s 时，2360 cm/s 的速度误差只给出 ~830 cm/s²
     * （0.85g），φ_des 只有 34°，净加速度 ~0.5 m/s²，一段航程里追不到 55
     * （实测稳定在 31 m/s）。kp=0.8 时 2360 cm/s 误差给出 1888 -> 被限到
     * VEL_MAX_HORIZ_ACC_CMSS=1470（1.5g），φ_des≈47°，可用加速度 ~0.6g。 */
    .pidFwd = { .kp = 4.0f, .ki = 0.15f, .kd = 0.05f, .iLimit = 200.0f, .dLimit = 5000.0f,
                .outMin = -VEL_MAX_HORIZ_ACC_CMSS, .outMax = VEL_MAX_HORIZ_ACC_CMSS },
    // 水平右向
    .pidRight = { .kp = 0.35f, .ki = 0.15f, .kd = 0.05f, .iLimit = 200.0f, .dLimit = 5000.0f,
                  .outMin = -VEL_MAX_HORIZ_ACC_CMSS, .outMax = VEL_MAX_HORIZ_ACC_CMSS },
    // 垂直：输出油门修正 µs（outMin/outMax 在运行时按 hover/idle/max 设置）
    /* 增益按"µs ↔ 加速度"标定：本机推重比 ≈5，油门行程 ~1000 µs 对应 ~5g，
     * 即 1 µs ≈ 0.005g ≈ 0.049 m/s²。速度环取时间常数 τ≈0.5 s => a=err/τ，
     * 则 1 cm/s 误差对应 0.02/0.049 ≈ 0.4 µs —— kp=0.4。
     * 原 1.5 意味着 300 cm/s 误差要 450 µs ≈ 2.2g：垂速环长期 bang-bang，
     * 实测悬停时 fbUp 在 ±4 m/s 之间自激（真值高度也 ±1~2 m 抖）。
     * 积分项负责消掉 hover 油门的静态偏差（小信号死区），取 τi≈4 s。 */
    .pidUp = { .kp = 0.4f, .ki = 0.1f, .kd = 0.1f, .iLimit = 200.0f, .dLimit = 2000.0f,
               .outMin = -300.0f, .outMax = 300.0f },
};

/* ---------------- 辅助 ---------------- */

static void clearIntegrators(void)
{
    vel.pidFwd.integrator = 0.0f;
    vel.pidRight.integrator = 0.0f;
    vel.pidUp.integrator = 0.0f;
    vel.pidFwd.prevError = 0.0f;
    vel.pidRight.prevError = 0.0f;
    vel.pidUp.prevError = 0.0f;
    vel.phiDesPrevDeg = 0.0f;
}

/* ---------------- 公开 API ---------------- */

void velocityControlReset(void)
{
    vel.active = false;
    clearIntegrators();
}

void velocityControlSetTarget(float vx, float vy, float vz)
{
    // H 系：x 前 / y 右 / z 天（向上为正）
    vel.targetFwd   = constrainf(vx, -VEL_MAX_HORIZ_VEL_CMS, VEL_MAX_HORIZ_VEL_CMS);
    vel.targetRight = constrainf(vy, -VEL_MAX_HORIZ_VEL_CMS, VEL_MAX_HORIZ_VEL_CMS);
    vel.targetUp    = constrainf(vz, -VEL_MAX_VERT_VEL_CMS, VEL_MAX_VERT_VEL_CMS);
    vel.lastCmdUs = micros();
}

bool velocityControlIsActive(void)
{
    return vel.active;
}

/*
 * 绕推力轴的滚转保持：把机体系 +Y 压回水平。
 *
 * 为什么需要：本链路 iNav 只有 xyz 速度指令，没有 yaw 指令。而"绕推力轴的自转"
 * 是一个既不产生平移、也不被速度环约束的自由度（线框系的 yaw 通道 = 机体系绕推力轴
 * 的滚转）。它漂走会让十字翼的一对翼离开水平 —— 升力失向、相机地平线滚转。
 *
 * 判据：机体系 +Y 水平 <=> xoz 平面竖直。机体系 +Y 在世界竖直方向上的分量为
 * rMat[2][1]（rMat 是体→地）。这里不直接对 rMat[2][1] 做比例（那在推力轴接近
 * 水平时会退化），而是构造"期望的 +Y 方向"再取绕推力轴的夹角，全姿态可用：
 *
 *   t  = 推力轴 = -机体系 +Z（线框系把推力轴读作 up）
 *   ẑ  = 世界竖直（本固件 rMat 的口径：天 = (0,0,-1)）
 *   期望 +Y 必须同时垂直于 t 和 ẑ  =>  dy = normalize(t × ẑ) = normalize(ty, -tx, 0)
 *   期望 +Y 取 ±dy 中**离当前更近**的那个（只约束滚转，不指定绕推力轴转到哪一侧）
 *   误差角 phi = 从当前 +Y 到该期望方向绕 t 的带符号夹角，|phi| ≤ 90°
 *   绕 t 的角速率 = K * phi；t = -bodyZ，故写进 rcCommand[YAW] 的 bodyZ 速率取反
 *
 * 推力轴接近竖直（悬停）时 |t_h| → 0，此时 +Y 本来就水平、该自由度无定义，
 * 直接不作用（dy 退化）。同理它只在推力轴倾斜时才真正有权限。
 */
static float rollHoldRateDps(const float maxRollDps)
{
    /* 机体系 +Y 的世界"下"分量 rMat[2][1]：xoz 面竖直 <=> 它为 0。
     * iNav 约定 roll 正 = 右翼下沉 = +Y 更朝下 = rMat[2][1] 增大，
     * 所以要把它拉回 0 需要**负**的滚转角速率 —— 符号无歧义
     * （不再依赖 rMatᵀ/绕轴符号链），而且走 ROLL 通道，
     * 不再和"自动 yaw 转向"抢同一个通道。 */
    const float offDeg = RADIANS_TO_DEGREES(asinf(constrainf(rMat[2][1], -1.0f, 1.0f)));
    vel.rollHoldPhiDeg = offDeg;

    return constrainf(-VEL_ROLL_HOLD_KP * offDeg, -maxRollDps, maxRollDps);
}

void taskVelocityControl(timeUs_t currentTimeUs)
{
    // 指令新鲜度 & 解锁状态判据
    const bool fresh = (currentTimeUs - vel.lastCmdUs) < VEL_TIMEOUT_US;
    if (!ARMING_FLAG(ARMED)) {
        vel.active = false;
        clearIntegrators();
        vel.lastRunUs = currentTimeUs;
        return;
    }
    /* 命令断流（> VEL_TIMEOUT_US）**不再撒手**：本环占着舵面，撒手后 rcCommand 会回到
     * adapter 发的 neutral（杆量中位 + **idle 油门**）⇒ 没有推力 ⇒ 自由落体。
     * 目标置零 = 倾角 0 + 悬停油门 = 原地悬停，与"全 0 命令"的行为一致。
     * 只有未解锁才真正解除接管（上面那支）。 */
    if (!fresh) {
        vel.targetFwd   = 0.0f;
        vel.targetRight = 0.0f;
        vel.targetUp    = 0.0f;
    }

    // 计算 dt
    float dt = 0.0f;
    if (vel.lastRunUs > 0) {
        dt = (float)(currentTimeUs - vel.lastRunUs) * 1e-6f;
    }
    vel.lastRunUs = currentTimeUs;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.005f;   // 异常时回退到名义 200Hz 周期
    }

    /* 航向：`attitude.raw[YAW]` 直接就是"地速航迹"所在的航向（实测标定，2026-10-09）。
     * 判据是 1 Hz 诊断里的 fb 与坐标差分算出的真实航迹是否对得上：
     *   leg6（1000 m/az300）真实位移航迹 299°、地速 15.9 m/s；
     *   取 psi = attitude.raw[YAW] 时 fb=(15.5,-4.3)（≈沿机头前飞，对）；
     *   取 psi = -attitude.raw[YAW] 时 fb=(-0.3,-16.0)（**说飞机在横着飞**，错 107°），
     *   而当时指令 tgt=(2000,-3) —— 速度环于是把"指令 20 m/s 前飞"对上一个"横向 16 m/s"
     *   的假反馈，整段不减速（planner 把指令降到 0.4 m/s 也不理），最后冲过目标。
     * 所以速度反馈这里**不加负号**。
     *
     * 注意：俯仰实测角 φ_act 用的是同一个 psi、但取反（见下面 phiActDeg）—— 两者要的符号
     * 不同是因为它们投影的是**不同的轴**：fb 投的是"地速矢量"，φ_act 投的是"推力轴"
     * （推力轴 = 线框 +Z = 机身 +X，与线框 X/机头不是同一根轴，见 docs/ue_operations.md §4.2）。
     * 两个符号各自都用实测标定过，别"顺手统一"。 */
    const float psi = DECIDEGREES_TO_RADIANS(attitude.raw[YAW]);
    const float cpsi = cosf(psi);
    const float spsi = sinf(psi);

    // 反馈：地固 NEU -> H 系（x 前 / y 右 / z 天）
    const float vN = posEstimator.est.vel.v[X];
    const float vE = posEstimator.est.vel.v[Y];
    const float vU = posEstimator.est.vel.v[Z];
    const float fbFwdRaw   =  vN * cpsi + vE * spsi;
    const float fbRightRaw = -vN * spsi + vE * cpsi;
    const float fbUpRaw    =  vU;

    // 首次激活：bumpless transfer，反馈滤波器初始化为当前速度
    if (!vel.active) {
        pt1FilterReset(&vel.fbFwd, fbFwdRaw);
        pt1FilterReset(&vel.fbRight, fbRightRaw);
        pt1FilterReset(&vel.fbUp, fbUpRaw);
        // 方位低通状态按当前指令方向初始化（无水平指令时朝前），避免起步瞬间偏航
        const float tgtInit = sqrtf(vel.targetFwd * vel.targetFwd
                                  + vel.targetRight * vel.targetRight);
        vel.dirFwd   = (tgtInit > VEL_YAW_MIN_TGT_CMS) ? vel.targetFwd / tgtInit : 1.0f;
        vel.dirRight = (tgtInit > VEL_YAW_MIN_TGT_CMS) ? vel.targetRight / tgtInit : 0.0f;
        clearIntegrators();
        vel.active = true;
    }

    // 环内 15Hz 低通
    const float fbFwd   = pt1FilterApply4(&vel.fbFwd,   fbFwdRaw,   VEL_FEEDBACK_LPF_HZ, dt);
    const float fbRight = pt1FilterApply4(&vel.fbRight, fbRightRaw, VEL_FEEDBACK_LPF_HZ, dt);
    const float fbUp    = pt1FilterApply4(&vel.fbUp,    fbUpRaw,    VEL_FEEDBACK_LPF_HZ, dt);

    /* ---- 水平速度环：输出期望加速度（cm/s^2），指令与反馈同为 H 系 ---- */
    float aFwd   = velPidApply(&vel.pidFwd,   vel.targetFwd,   fbFwd,   dt);
    float aRight = velPidApply(&vel.pidRight, vel.targetRight, fbRight, dt);

    // 限制合成水平加速度幅值
    float aMag = sqrtf(aFwd * aFwd + aRight * aRight);
    if (aMag > VEL_MAX_HORIZ_ACC_CMSS) {
        const float scale = VEL_MAX_HORIZ_ACC_CMSS / aMag;
        aFwd *= scale;
        aRight *= scale;
        aMag = VEL_MAX_HORIZ_ACC_CMSS;
    }

    /*
     * ---- 期望姿态：机头垂直平面内的"带符号俯仰" ----
     *
     * 需求 1 要求 xoz 平面始终竖直，等价于**不许侧倾**；于是姿态误差只剩一个
     * 自由度：机头垂直平面内的带符号倾角 φ（正 = 低头/前倾）。
     *   俯仰通道：φ_des 来自"沿机头方向"的加速度需求，φ_act 由推力轴实测得到
     *   滚转通道：只做机翼拉平（rMat[2][1] -> 0）
     *
     * 这里**不能**用"倾角幅值 + 方位"分解（旧实现）。方位误差在倾角恰好相等时
     * 完全不可观测，会稳定在"倾角幅值正确、方向反 180°"的倒飞平衡点上。实测正是
     * 这个平衡点：指令低头 35.5°、实际 pitch = -32.5（抬头）、GPS 航迹 282° 而
     * 机头 102°、|fb| 30 m/s 反向、倾角长期顶在限幅。带符号的 φ 天然区分前倾与
     * 后仰，不存在该平衡点（旧实现把方位项 (void) 掉了，等于永远不修正倾角方向）。
     *
     * φ 是**带符号**的：aFwd < 0（速度过冲，需要减速）时自然给出后仰角，靠姿态刹车。
     * 这不会退回"倒飞"：机头始终由下面的偏航环对准**指令速度方向**（与反向的速度
     * 误差无关），所以后仰只出现在刹车过渡里，不会出现"机头朝前、速度朝后"的持续反向飞行。
     */
    /* 实测推力轴（rMat 系即地固 NED；t = -机体系 Z = 推力方向） */
    const float tN = -rMat[0][2], tE = -rMat[1][2], tD = -rMat[2][2];
    /* 推力轴沿机头水平投影的分量。
     *
     * **这个投影用镜像的 psi，且整体再取反**（两处符号都是实测标定，2026-10-09）：
     * 上面的速度反馈用 psi 原值，这里用 -psi，两者**不能统一**（投影的轴不同：
     * 那里投地速矢量，这里投推力轴 = 线框 +Z = 机身 +X，见 ue_operations.md §4.2）。
     *
     * 标定证据（Test1 逐目标重启，1 Hz 诊断）：
     *  - 用 psi 原值：leg1（方位 0°）俯仰环稳定 ✔，但 leg2（60°）/leg6（300°）在
     *    水平指令一介入时正反馈发散 —— leg6 原始记录 pd=14.1 pa=-4.9 -> -15.2 -> -30.0
     *    （指令一直低头、实测报抬头），而同期 fb=358（3.6 m/s 沿机头前飞）证明飞机
     *    **物理上确实在低头**；随后 thup 掉到 -0.116（推力轴接近水平）翻滚、gv 冲到
     *    -34 m/s，横向跑飞 280 m。
     *  - 用 -psi + 整体取反（当前实现）：leg1 稳定 ✔，leg6 的 pa 与 pd 同号跟随 ✔，
     *    六个方位（0/60/120/180/240/300）全部飞到目标、不再翻滚。
     * 镜像的物理来源：`attitude.raw[YAW]` 是**负的**罗盘航向（§4.4 第 2 条），
     * 而线框机头 X 与推力轴 Z 不是同一根轴，两者对该符号的敏感度相反。 */
    const float tAlongNose = tN * cpsi - tE * spsi;   /* = 用 -psi 投影 */
    const float phiActDeg = -RADIANS_TO_DEGREES(atan2f(tAlongNose, -tD));
    float phiDesDeg = 0.0f;   /* 在垂速环之后按当前推力折算，见那里的说明 */

    /* ---- 垂直速度环：输出油门（µs） ---- */
    const float idle  = (float)getThrottleIdleValue();
    const float maxT  = (float)getMaxThrottle();
    const float hover = currentBatteryProfile->nav.mc.hover_throttle;

    // 油门修正范围：**向下权限必须很小**，向上可以给足。
    // 推力 ∝ (油门-idle)²，悬停点=1g、idle=0g，所以修正量的"g 值"极不对称：
    //   -300 µs（= idle-hover，旧实现）→ 0g      = 自由落体
    //   -150 µs                        → ~0.25g  = 自由落体
    //   -30 µs                         → ~0.8g   = 最多 2 m/s² 下落
    //   +22 µs                         → 1.03g   = 实测稳态 5 m/s 爬升只需要这么多
    //   +150 µs                        → ~2.2g   = 爬升权限绰绰有余
    // 实测故障链：爬到 29 m 后水平指令介入，垂速指令由 5 m/s 掉到 0.5 m/s，
    // 垂速环猛收油门 → 推力塌到 1/4 → 3.6 s 掉 35 m 撞地、之后一直贴地滑行
    // （adapter 日志 alt=463/22.9 → 440/-5.2 → 440/0.1 gs=5.3 就是这段）。
    // 所以向下只给 ~0.8g，向上给满；积分限幅同样收窄避免下侧卷绕。
    vel.pidUp.outMin = fmaxf(idle - hover, -30.0f);
    vel.pidUp.outMax = fminf(maxT - hover, 250.0f);
    vel.pidUp.iLimit = 60.0f;

    const float thrCorrection = velPidApply(&vel.pidUp, vel.targetUp, fbUp, dt);
    const float throttle = constrainf(hover + thrCorrection, idle, maxT);

    /* ---- 指令方位 psiCmd：机头对准**指令水平速度方向** ----
     * 横向需求只能靠"转向"实现（不许侧倾，xoz 面才竖直，需求 1）。
     * 必须用"指令方向"（与反馈无关，不会自激），不能用"速度误差方向"：
     * 横向误差靠转向永远消不掉 —— 不侧倾就产生不了横向加速度，机头只是一直
     * 追着那个消不掉的横向误差转，实测 ryaw 长期饱和、pa 在 ±180° 之间翻滚。 */
    const float tgtHorizCms = sqrtf(vel.targetFwd * vel.targetFwd
                                  + vel.targetRight * vel.targetRight);
    /* **指令方位一阶低通**（实测标定，2026-10-09 末制导端局）：
     * 近距离时位置量化很粗（东 0.73 m / 北 0.21 m，docs/ue_operations.md §4.10），
     * 3 m 处一个量化格就是 ±14° 的"目标方位"跳变；而方位环增益 2.5 会把 12° 直接
     * 放大成**饱和的 30 dps 偏航** —— 实测 leg1 末端 ryaw 顶到 30、航向从 350° 甩到
     * 264°、横向速度冲到 3.2 m/s，横向残差 12 m、最近接近只有 4.94 m（同一段刹车本身
     * 是好的：pd 从 13.9° 一路到 -11.3° 抬头，把 fb 从 16.8 压到 2 m/s）。
     * 对**单位方向分量**做一阶低通（不是对角度做），±180° 环绕无需特殊处理。 */
    if (tgtHorizCms > VEL_YAW_MIN_TGT_CMS) {
        const float alpha = dt / (VEL_YAW_DIR_LPF_TAU_S + dt);
        vel.dirFwd   += alpha * (vel.targetFwd   / tgtHorizCms - vel.dirFwd);
        vel.dirRight += alpha * (vel.targetRight / tgtHorizCms - vel.dirRight);
    }
    const float psiCmd = (tgtHorizCms > VEL_YAW_MIN_TGT_CMS)
                       ? atan2f(vel.dirRight, vel.dirFwd)
                       : 0.0f;   /* 指令为 0（悬停）时保持当前航向 */

    /* **先对准、再前推**（严格版）：方位误差还大时**完全不给水平推力**，只原地转向。
     *
     * 实测故障链（Test1 leg1，每目标重启一次，可复现）：飞到 27 m 后水平指令一介入，
     * 播种航向 90°、目标正北（方位误差 -59°），60 dps 偏航与 6.7° 倾角同时作用 ——
     * 1 Hz 采样 pa 依次 -11.8° / -39.7° / -75.4°（指令一直低头 14°→6°，即机身**抬头**
     * 发散），推力轴转平后翻滚，gv 冲到 42 m/s，随后撞地贴地滑行整个航段。
     * 旧实现按 cos(方位误差) 缩放（此处 cos(-59°)=0.52）只把倾角减半，激励仍然存在。
     * 现在误差 > VEL_ALIGN_BAND_DEG 时 aFwd 直接置 0：地面系下永远"先转到位、再前推"，
     * 两种激励不再重叠。倾角从 0 起来的阶跃由 VEL_PHI_SLEW_DPS 磨圆。 */
    if (fabsf(RADIANS_TO_DEGREES(psiCmd)) > VEL_ALIGN_BAND_DEG) {
        aFwd = 0.0f;
    }

    /* ---- 期望倾角 φ_des：必须按**当前实际推力**折算，不能固定用 g ----
     *
     * 推力矢量：竖直分量 T·cosφ = g + a_v，水平分量 T·sinφ = a_h
     *   =>  tanφ = a_h / (g + a_v)
     * 所以 tan(φ_des) = aFwd / (g + a_v)，而 a_v 来自垂速环给出的油门量。
     * 推力按螺旋桨律 T ∝ (油门-idle)²（转速∝油门-idle），以悬停点为 1g 归一：
     *   T = g·((throttle-idle)/(hover-idle))²
     *
     * 旧实现直接把 a_v 当 0（固定除 g）。爬升/掉高时油门被垂速环顶到 2~4g，
     * 同一个倾角产生的水平加速度被放大约 3 倍：实测起飞+大横向指令那几拍冲到
     * 67 m/s（指令 20），随后速度环猛刹、姿态摆到 ±170° 翻滚。
     * 按上式折算后，高油门下倾角自动变小，水平加速度回到 aFwd 的量级。
     * 下限取 1g：油门低于悬停时**不多给**倾角（不让它比原来更激进）。 */
    {
        const float thrAboveIdle   = fmaxf(throttle - idle, 0.0f);
        const float hoverAboveIdle = fmaxf(hover - idle, 1.0f);
        const float thrustRatio    = thrAboveIdle / hoverAboveIdle;
        const float aVertCmss      = fmaxf(GRAVITY_CMSS * thrustRatio * thrustRatio, GRAVITY_CMSS);
        const float phiTargetDeg   = RADIANS_TO_DEGREES(atan2f(aFwd, aVertCmss));

        /* 期望倾角速率限幅：把"指令切换/低空保护解除"造成的阶跃磨圆，
         * 避免内环速率环被阶跃激励（见 VEL_PHI_SLEW_DPS 的说明）。 */
        const float dPhiMax = VEL_PHI_SLEW_DPS * dt;
        phiDesDeg = constrainf(phiTargetDeg,
                               vel.phiDesPrevDeg - dPhiMax,
                               vel.phiDesPrevDeg + dPhiMax);
        vel.phiDesPrevDeg = phiDesDeg;
    }

    const uint8_t rateRll = currentControlProfile->stabilized.rates[FD_ROLL];
    const uint8_t ratePit = currentControlProfile->stabilized.rates[FD_PITCH];
    const uint8_t rateYaw = currentControlProfile->stabilized.rates[FD_YAW];
    const float maxRllDps = rateRll * 10.0f;
    const float maxPitDps = ratePit * 10.0f;

    /* ---- 偏航输出：psiCmd（上面已按指令方向算好）→ 角速率，硬限幅 ---- */
    const float rateYawCmd = constrainf(-VEL_YAW_TURN_KP * RADIANS_TO_DEGREES(psiCmd),
                                        -VEL_YAW_MAX_DPS, VEL_YAW_MAX_DPS);

    /* ---- 滚转/俯仰通道 ----
     * 滚转：只做机翼拉平（rMat[2][1] -> 0），不带任何方向项 → xoz 面始终竖直。
     * 俯仰：带符号倾角误差 -> 低头为正的角速率（rcCommand[PITCH] 正 = 低头）。 */
    const float rateRollCmd  = constrainf(rollHoldRateDps(maxRllDps), -maxRllDps, maxRllDps);
    const float ratePitchCmd = constrainf(VEL_ATT_KP * (phiDesDeg - phiActDeg),
                                          -maxPitDps, maxPitDps);

    vel.outRoll     = pidRateToRcCommand(rateRollCmd, rateRll);
    vel.outPitch    = pidRateToRcCommand(ratePitchCmd, ratePit);
    vel.outYaw      = pidRateToRcCommand(rateYawCmd, rateYaw);
    vel.outThrottle = (int16_t)throttle;

    /* ---- 低频诊断（1 Hz，只在接管时打）：验证"xoz 平面竖直 / x 始终朝下" ----
     * xoz = rMat[2][1]，机体系 +Y 的世界"下"分量；xoz≈0 即 xoz 平面竖直。
     * thup = 推力轴的世界"下"分量（= -rMat[2][2]）；>0 表示推力轴朝下（车翻了）。
     * pd/pa = 期望/实测的带符号倾角（正 = 低头）；稳定时 pa≈pd>0 且 |pa| 远小于 90°，
     * 即"x 始终朝下"。phi = 滚转保持的误差角。验证完可删。 */
    {
        static timeMs_t dbgLastMs = 0;
        const timeMs_t dbgNowMs = millis();
        if (dbgNowMs - dbgLastMs >= 1000) {
            dbgLastMs = dbgNowMs;
            printf("VC xoz=%.3f thup=%.3f phi=%.1f pd=%.1f pa=%.1f ryaw=%.1f thr=%.0f/%.0f "
                   "tgt=%.0f/%.0f/%.0f fb=%.0f/%.0f/%.0f "
                   "fl=0x%08lx eph=%.0f gv=%.0f/%.0f ev=%.0f/%.0f hdg=%d yaw=%.0f\n",
                   (double)rMat[2][1], (double)tD, (double)vel.rollHoldPhiDeg,
                   (double)phiDesDeg, (double)phiActDeg, (double)rateYawCmd,
                   (double)throttle, (double)hover,
                   (double)vel.targetFwd, (double)vel.targetRight, (double)vel.targetUp,
                   (double)fbFwd, (double)fbRight, (double)fbUp,
                   (unsigned long)posEstimator.flags, (double)posEstimator.est.eph,
                   (double)posEstimator.gps.vel.x, (double)posEstimator.gps.vel.y,
                   (double)posEstimator.est.vel.v[X], (double)posEstimator.est.vel.v[Y],
                   isImuHeadingValid() ? 1 : 0, (double)RADIANS_TO_DEGREES(psi));
        }
    }

    /* 关掉 ANGLE/HORIZON/ANGLEHOLD：否则 rcCommand[ROLL/PITCH] 会被当成角度目标，
     * 又走回上面那条 euler 奇异的路。模式标志每周期由 RX 重算，而本函数在其后调用
     * （fc_core.c processPilotAndFailSafeActions 末尾），故本周期有效。 */
    DISABLE_FLIGHT_MODE(ANGLE_MODE);
    DISABLE_FLIGHT_MODE(HORIZON_MODE);
    DISABLE_FLIGHT_MODE(ANGLEHOLD_MODE);
}

void velocityControlApplyToRcCommand(void)
{
    if (vel.active) {
        rcCommand[ROLL]     = vel.outRoll;
        rcCommand[PITCH]    = vel.outPitch;
        rcCommand[YAW]      = vel.outYaw;
        rcCommand[THROTTLE] = vel.outThrottle;
    }
}
