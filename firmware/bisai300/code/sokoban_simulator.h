#ifndef SOKOBAN_SIMULATOR_H
#define SOKOBAN_SIMULATOR_H

#include "planner_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int success;
    int consumed;
    char reason[48];
    PlannerGrid grid;
    PlannerPoint player;
    char executed_path[PLANNER_MAX_PATH_LEN];
    char remaining_path[PLANNER_MAX_PATH_LEN];
    PlannerPoint consumed_goal;
    int boxes_left;
    int goals_left;
} SokobanSimResult;

/* Stage24: simulate a planned U/D/L/R path and stop immediately after the
 * first box enters a goal. The consumed box and goal both disappear.
 */
int sokoban_simulate_until_first_consumption(const PlannerGrid *grid,
                                             const char *path,
                                             SokobanSimResult *out_result);

/* Full consuming simulation, mainly for PC/MCU regression tests. */
int sokoban_simulate_consuming_solution(const PlannerGrid *grid,
                                        const char *path,
                                        SokobanSimResult *out_result);

#ifdef __cplusplus
}
#endif

#endif /* SOKOBAN_SIMULATOR_H */
