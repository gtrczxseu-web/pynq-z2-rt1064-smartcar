#ifndef PLANNER_TYPES_H
#define PLANNER_TYPES_H

#include <stdint.h>
#include "planner_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t r;
    int16_t c;
} PlannerPoint;

typedef struct {
    char grid[PLANNER_ROWS][PLANNER_COLS];
} PlannerGrid;

/* Legacy/general planner result used by solver_api/openart_adapter.
 * Stage28 final single-action code uses PlannerControllerResult in planner_controller_api.h,
 * but this struct is intentionally kept small and compatible with earlier modules.
 */
typedef struct {
    int success;                         /* 1 = success, 0 = failure */
    char reason[64];                     /* failure/success reason */
    char mode[32];                       /* normal / normal_step / bomb_step / done / fail */
    char path[PLANNER_MAX_PATH_LEN];     /* U/D/L/R path */
    int path_len;
} PlannerResult;

typedef struct {
    char dir;    /* N/S/E/W */
    int steps;
} PlannerMcuCommand;

typedef struct {
    PlannerMcuCommand commands[PLANNER_MAX_MCU_COMMANDS];
    int count;
} PlannerMcuCommandList;

static inline int planner_point_equal(PlannerPoint a, PlannerPoint b)
{
    return a.r == b.r && a.c == b.c;
}

static inline int planner_point_valid(PlannerPoint p)
{
    return p.r >= 0 && p.r < PLANNER_ROWS && p.c >= 0 && p.c < PLANNER_COLS;
}

#ifdef __cplusplus
}
#endif

#endif /* PLANNER_TYPES_H */
