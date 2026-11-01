#ifndef PLANNER_CONTROLLER_API_H
#define PLANNER_CONTROLLER_API_H

#include "planner_types.h"
#include "sokoban_simulator.h"
#include "multi_bomb_step_solver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PLAN_CONTROLLER_MODE_NORMAL_STEP = 0,
    PLAN_CONTROLLER_MODE_BOMB_STEP   = 1,
    PLAN_CONTROLLER_MODE_DONE        = 2,
    PLAN_CONTROLLER_MODE_FAIL        = 3
} PlannerControllerMode;

typedef struct {
    int success;
    PlannerControllerMode mode_enum;
    char mode[32];
    char reason[64];

    char path[PLANNER_MAX_PATH_LEN];
    int path_len;

    int requires_reobserve;
    char action[32];

    char full_planned_path[PLANNER_MAX_PATH_LEN];
    char discarded_path[PLANNER_MAX_PATH_LEN];
    char bomb_path[PLANNER_MAX_PATH_LEN];

    PlannerGrid predicted_grid;
    PlannerPoint player_before;
    PlannerPoint predicted_player_after;
    PlannerPoint consumed_goal;

    int boxes_before;
    int boxes_after;
    int goals_before;
    int goals_after;
    int bombs_before;
    int bombs_after;

    PlannerResult normal_result;
    MultiBombStepResult bomb_result;
} PlannerControllerResult;

/* Stage28 final single-action entry point.
 * Returns one action only: normal_step, bomb_step, done, or fail.
 */
int planner_controller_plan_next_action(const PlannerGrid *grid,
                                        int max_bomb_depth,
                                        PlannerControllerResult *out_result);

/* Helper for main.c printf/WiFi logs. */
const char *planner_controller_mode_to_string(PlannerControllerMode mode);

#ifdef __cplusplus
}
#endif

#endif /* PLANNER_CONTROLLER_API_H */
