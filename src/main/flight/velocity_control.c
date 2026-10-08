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
#define VEL_MAX_HORIZ_ACC_CMSS      1500.0f     // 水平加速度输出上限（~1.5g，对应约56.8°倾角）
#define VEL_MAX_HORIZ_VEL_CMS       12000.0f    // 水平速度目标上限（>90m/s，留余量）
#define VEL_MAX_VERT_VEL_CMS        3000.0f     // 垂直速度目标上限（±30m/s）

/* 绕推力轴的滚转保持（见文件末尾的说明）。增益单位 dps/deg。 */
#define VEL_ROLL_HOLD_KP            3.0f
#define VEL_ROLL_HOLD_SIGN          1.0f        // 若实测该自由度发散，改 -1
/* 推力轴水平分量低于此值时"滚转保持"退化（悬停），不作用。 */
#define VEL_ROLL_HOLD_MIN_TILT      0.15f

/* 姿态层：方向误差 -> 机体角速率(dps) 的比例增益 */
#define VEL_ATT_KP                  4.0f
/* 推力轴水平分量低于此值时"方位"退化（推力轴接近竖直），只做倾角。 */
#define VEL_ATT_MIN_HORIZ           0.15f

/*
 * rMat 的地球系与"真地球"差一个绕东轴的镜像（北分量取反）。
 *
 * 本仿真实测（docs/tg_wander_turn_hover.md §5.1）：指令方位 331° 时 rMat 解出的
 * 推力轴方位也是 331°，而 GPS 速度航向是 212°；180-331 = -151 ≡ 209° ≈ 212°，
 * 即车飞的正是推力关于东西轴的镜像。所以期望方向 d 也要翻北分量，才能与
 * t（来自 rMat）同处一个系。改 0 即关掉这个补偿。
 */
#define VEL_ATT_MIRROR_NORTH        1

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
} vel = {
    // 水平前向：输出 cm/s^2
    .pidFwd = { .kp = 3.0f, .ki = 1.5f, .kd = 0.0f, .iLimit = 500.0f, .dLimit = 5000.0f,
                .outMin = -VEL_MAX_HORIZ_ACC_CMSS, .outMax = VEL_MAX_HORIZ_ACC_CMSS },
    // 水平右向
    .pidRight = { .kp = 3.0f, .ki = 1.5f, .kd = 0.0f, .iLimit = 500.0f, .dLimit = 5000.0f,
                  .outMin = -VEL_MAX_HORIZ_ACC_CMSS, .outMax = VEL_MAX_HORIZ_ACC_CMSS },
    // 垂直：输出油门修正 µs（outMin/outMax 在运行时按 hover/idle/max 设置）
    .pidUp = { .kp = 1.5f, .ki = 0.8f, .kd = 0.0f, .iLimit = 300.0f, .dLimit = 2000.0f,
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
static float rollHoldRateDps(const float maxYawDps)
{
    const float tx = -rMat[0][2];
    const float ty = -rMat[1][2];
    const float tz = -rMat[2][2];

    // dy = normalize(t × ẑ), ẑ = (0,0,1)  =>  (ty, -tx, 0)
    const float dyx = ty;
    const float dyy = -tx;
    const float dyMag = sqrtf(dyx * dyx + dyy * dyy);
    if (dyMag < VEL_ROLL_HOLD_MIN_TILT) {
        vel.rollHoldPhiDeg = 0.0f;
        return 0.0f;
    }

    const float ux = dyx / dyMag;
    const float uy = dyy / dyMag;

    // 当前机体系 +Y
    const float byx = rMat[0][1];
    const float byy = rMat[1][1];
    const float byz = rMat[2][1];

    // sin(phi) = ((bodyY × dy) · t), cos(phi) = bodyY · dy
    const float crossX = byy * 0.0f - byz * uy;
    const float crossY = byz * ux - byx * 0.0f;
    const float crossZ = byx * uy - byy * ux;
    const float sinPhi = crossX * tx + crossY * ty + crossZ * tz;
    const float cosPhi = byx * ux + byy * uy;

    const float phiDeg = RADIANS_TO_DEGREES(atan2f(sinPhi, cosPhi));

    /*
     * 只约束滚转：把 +Y 拉到**最近的**那个水平解，不强制绕推力轴转到哪一侧。
     * ±dy 给出同一个水平面、但绕推力轴差 180°；本机绕推力轴几乎没有权限
     * （只有桨阻），硬拉远端解会变成一次 180° 慢滚。取 |phi| ≤ 90° 的那个。
     */
    float phiWrappedDeg = phiDeg;
    if (phiWrappedDeg > 90.0f) {
        phiWrappedDeg -= 180.0f;
    } else if (phiWrappedDeg < -90.0f) {
        phiWrappedDeg += 180.0f;
    }
    vel.rollHoldPhiDeg = phiWrappedDeg;

    // 绕 t 的速率 -> bodyZ 速率（t = -bodyZ）
    const float rateAboutT = VEL_ROLL_HOLD_KP * phiWrappedDeg;
    const float rateBodyZ = VEL_ROLL_HOLD_SIGN * -rateAboutT;

    return constrainf(rateBodyZ, -maxYawDps, maxYawDps);
}

void taskVelocityControl(timeUs_t currentTimeUs)
{
    // 指令新鲜度 & 解锁状态判据
    const bool fresh = (currentTimeUs - vel.lastCmdUs) < VEL_TIMEOUT_US;
    if (!fresh || !ARMING_FLAG(ARMED)) {
        vel.active = false;
        clearIntegrators();
        vel.lastRunUs = currentTimeUs;
        return;
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

    // H 系航向：罗盘航向（0=N、顺时针），与 ue.c 的注入、algo/planning 口径一致
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
     * ---- 期望姿态：用旋转矢量，不用 euler ----
     *
     * iNav 角度环的误差是逐轴相减（flight/pid.c pidLevel: angleTarget - attitude.raw[axis]），
     * 在 pitch ≈ ±90° 处奇异、roll/yaw 互相耦合。后续会有 pitch > 80° 的位形，
     * 所以这里不走 ANGLE 模式，改成纯矢量 / 旋转矩阵：
     *
     *   1) 由水平加速度求期望**推力轴方向** d（单位矢量），不是三个欧拉角
     *   2) 实际推力轴 t 直接从 rMat 读出（t = -机体系 +Z；线框系把推力轴读作 up）
     *   3) 方向误差分解成"绕世界竖直的方位"与"绕水平轴的倾角"两半：
     *      方位只改航向、倾角只改俯仰 —— 这样转弯不会把机头抬起来
     *      （旧的最短路径实现会让 x 轴在连续转弯中朝上）
     *   4) 地系角速度 -> rMatᵀ -> 机体系角速率 -> rcCommand[ROLL/PITCH]
     *
     * 全程只有矢量与旋转矩阵，θ=90° 处既不奇异也不退化。
     */
    const float thetaDeg = RADIANS_TO_DEGREES(atan2f(aMag, GRAVITY_CMSS));   /* 期望倾角 */
    const float psiCmd = atan2f(aRight, aFwd);                               /* H 系里期望加速度的方位 */

    /* H 系基矢量（算在 rMat 的地球系里）；psi 就是本函数反馈投影用的那个航向 */
    const float xhN = cpsi,  xhE = spsi;      /* x̂ = (cos psi, sin psi) */
    const float yhN = -spsi, yhE = cpsi;      /* ŷ = (-sin psi, cos psi) */
    const float cpc = cosf(psiCmd);
    const float spc = sinf(psiCmd);
    const float dhN = xhN * cpc + yhN * spc;
    const float dhE = xhE * cpc + yhE * spc;

    const float sinTh = sinf(DEGREES_TO_RADIANS(thetaDeg));
    const float cosTh = cosf(DEGREES_TO_RADIANS(thetaDeg));

    /* 期望推力轴（天 = (0,0,-1)）。VEL_ATT_MIRROR_NORTH 见常量处的说明。 */
    const float dN = VEL_ATT_MIRROR_NORTH ? -(sinTh * dhN) : (sinTh * dhN);
    const float dE = sinTh * dhE;
    const float dD = -cosTh;

    /* 实测推力轴（rMat 系） */
    const float tN = -rMat[0][2], tE = -rMat[1][2], tD = -rMat[2][2];

    /* 方位误差绕世界竖直 up=(0,0,-1)；倾角误差绕水平轴 a_tilt=normalize(th×up) */
    const float thx = tN, thy = tE;
    const float dhx = dN, dhy = dE;
    const float thMag = sqrtf(thx * thx + thy * thy);
    const float dhMag = sqrtf(dhx * dhx + dhy * dhy);

    const float elevT = RADIANS_TO_DEGREES(asinf(constrainf(-tD, -1.0f, 1.0f)));
    const float elevD = RADIANS_TO_DEGREES(asinf(constrainf(-dD, -1.0f, 1.0f)));
    const float dElev = elevD - elevT;

    float dPsi = 0.0f;
    if (thMag > VEL_ATT_MIN_HORIZ && dhMag > VEL_ATT_MIN_HORIZ) {
        dPsi = RADIANS_TO_DEGREES(atan2f(-(thx * dhy - thy * dhx), thx * dhx + thy * dhy));
    }

    float weN = 0.0f, weE = 0.0f, weD = 0.0f;
    if (thMag > VEL_ATT_MIN_HORIZ) {
        weD += -dPsi;                                    /* 方位：绕 up */
        weN += dElev * (-thy / thMag);                   /* 倾角：绕 a_tilt */
        weE += dElev * (thx / thMag);
    } else if (dhMag > VEL_ATT_MIN_HORIZ) {
        /* 推力轴已接近竖直：方位无定义，只把推力轴朝 dh 掰过去 */
        weN += (-dElev) * (dhy / dhMag);
        weE += (-dElev) * (-dhx / dhMag);
    }

    /* 地系 -> 机体系：乘 rMatᵀ */
    const float rvBx = rMat[0][0] * weN + rMat[1][0] * weE + rMat[2][0] * weD;
    const float rvBy = rMat[0][1] * weN + rMat[1][1] * weE + rMat[2][1] * weD;

    /* ---- 垂直速度环：输出油门（µs） ---- */
    const float idle  = (float)getThrottleIdleValue();
    const float maxT  = (float)getMaxThrottle();
    const float hover = currentBatteryProfile->nav.mc.hover_throttle;

    // 油门修正范围 = [idle-hover, max-hover]；积分限幅随之
    vel.pidUp.outMin = idle - hover;
    vel.pidUp.outMax = maxT - hover;
    vel.pidUp.iLimit = fminf(fabsf(vel.pidUp.outMin), fabsf(vel.pidUp.outMax));

    const float thrCorrection = velPidApply(&vel.pidUp, vel.targetUp, fbUp, dt);
    const float throttle = constrainf(hover + thrCorrection, idle, maxT);

    /* ---- 绕推力轴的滚转保持 ---- */
    const uint8_t rateYaw = currentControlProfile->stabilized.rates[FD_YAW];
    const float maxYawDps = rateYaw * 10.0f;
    const float rateYawCmd = rollHoldRateDps(maxYawDps);

    /* ---- 方向误差 -> 机体角速率指令 ---- */
    const uint8_t rateRll = currentControlProfile->stabilized.rates[FD_ROLL];
    const uint8_t ratePit = currentControlProfile->stabilized.rates[FD_PITCH];
    const float maxRllDps = rateRll * 10.0f;
    const float maxPitDps = ratePit * 10.0f;

    const float rateRollCmd  = constrainf(VEL_ATT_KP * rvBx, -maxRllDps, maxRllDps);
    const float ratePitchCmd = constrainf(VEL_ATT_KP * rvBy, -maxPitDps, maxPitDps);

    vel.outRoll     = pidRateToRcCommand(rateRollCmd, rateRll);
    vel.outPitch    = pidRateToRcCommand(ratePitchCmd, ratePit);
    vel.outYaw      = pidRateToRcCommand(rateYawCmd, rateYaw);
    vel.outThrottle = (int16_t)throttle;

    /* ---- 低频诊断（1 Hz，只在接管时打）：验证"xoz 平面竖直 / x 不朝上" ----
     * xoz = rMat[2][1]，机体系 +Y 的世界"下"分量；xoz≈0 即 xoz 平面竖直。
     * thup = 推力轴的世界"下"分量（= -rMat[2][2]）；>0 表示推力轴朝上（x 朝上）。
     * phi = 滚转保持的误差角。验证完可删。 */
    {
        static timeMs_t dbgLastMs = 0;
        const timeMs_t dbgNowMs = millis();
        if (dbgNowMs - dbgLastMs >= 1000) {
            dbgLastMs = dbgNowMs;
            printf("VC xoz=%.3f thup=%.3f phi=%.1f th=%.1f ryaw=%.1f "
                   "tgt=%.0f/%.0f/%.0f fb=%.0f/%.0f/%.0f\n",
                   (double)rMat[2][1], (double)tD, (double)vel.rollHoldPhiDeg,
                   (double)thetaDeg, (double)rateYawCmd,
                   (double)vel.targetFwd, (double)vel.targetRight, (double)vel.targetUp,
                   (double)fbFwd, (double)fbRight, (double)fbUp);
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
