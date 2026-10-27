#ifndef TRAJECTORY_LIBRARY_H
#define TRAJECTORY_LIBRARY_H

#include <stdint.h>

#define TRAJ_SEGMENTS 50

/* ================= Rat ID 정의 ================= */
typedef enum
{
    RAT_TAG_11 = 0,
    RAT_TAG_8,
    RAT_TAG_7,
    RAT_TAG_5,
    RAT_TAG_6,
    RAT_TAG_12,
    RAT_COUNT          // 항상 마지막에 위치 (rat 개수 자동 계산)
} RatID;

/* ================= Breakpoints ================= */
extern const float break_points[TRAJ_SEGMENTS];

/* ================= Trajectory Coefficients ================= */
/* 구조: [rat][segment][a,b,c,d] */
extern const float trajectory_coeffs[RAT_COUNT][TRAJ_SEGMENTS][4];

/* ================= Trajectory Generator ================= */
float generate_trajectory(float x, RatID rat_id);

#endif