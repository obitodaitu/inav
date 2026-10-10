/*
 * velocity_control.h - Custom H-frame velocity controller for multirotor interceptors
 *
 * 方案B：INAV 仅保留 ANGLE 姿态内环；本模块在飞控上自建三轴速度环。
 *
 * 数据流：
 *   上位机 --(MSP2, H 系三轴速度 cm/s)--> velocityControlSetTarget()
 *   200Hz 周期任务 taskVelocityControl() 运行 PID，缓存 roll/pitch/yaw/throttle 输出
 *   每个 PID(1kHz) 周期末尾 velocityControlApplyToRcCommand() 覆盖 rcCommand
 *
 * 坐标约定：
 *   指令与反馈都在 **H 系**（与 algo 侧 inav/vel_cmd、planning、rcCommand 口径一致）：
 *     x = 机头水平投影（前）  y = 右  z = 天
 *   单位 cm/s。
 *
 *   实测速度来自 posEstimator.est.vel（地固 NEU），由本模块用航向投影到 H 系。
 *   不引入 NED：INAV 的 rcCommand 接口本就是 H 系，algo 侧也是 H 系。
 *
 *   rcCommand 符号（INAV 固有约定，见 flight/pid.c computePidLevelTarget 的注释）：
 *     rcCommand[PITCH] 正 = 低头，rcCommand[ROLL] 正 = 右滚。
 *   前向加速 -> 机头下俯 -> pitch 目标角取**正**。
 *
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"

/* 重置控制器：清积分、失能接管。在解锁状态变化/失联时调用 */
void velocityControlReset(void);

/*
 * 由 MSP2 速度指令调用：设置 H 系三轴速度目标。
 *   vx = 前, vy = 右, vz = 天，单位 cm/s；z 向上为正。
 * 同时刷新指令时间戳，用作超时判据。
 */
void velocityControlSetTarget(float vx, float vy, float vz);

/* 200Hz 周期任务：读取速度反馈、运行三轴 PID、缓存姿态/油门输出 */
void taskVelocityControl(timeUs_t currentTimeUs);

/*
 * 每个 PID 周期调用（必须在 rcCommand 全部计算完成之后）：
 * 若控制器有效（指令新鲜、已解锁）则覆盖 rcCommand[ROLL/PITCH/YAW/THROTTLE]。
 */
void velocityControlApplyToRcCommand(void);

/* 控制器是否正在接管（指令新鲜且已解锁） */
bool velocityControlIsActive(void);
