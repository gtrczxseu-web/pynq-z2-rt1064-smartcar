#ifndef BOMB_TASK_BUILDER_H
#define BOMB_TASK_BUILDER_H

#include "planner_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PLANNER_MAX_BOMB_TASKS
#define PLANNER_MAX_BOMB_TASKS 512
#endif

#define BOMB_MAX_AFFECTED_WALLS 9

typedef struct {
    int task_id;
    PlannerPoint bomb;
    PlannerPoint target_wall;
    PlannerPoint blast_center;
    PlannerPoint affected_walls[BOMB_MAX_AFFECTED_WALLS];
    int affected_wall_count;
    int distance_score;
} BombTask;

/* C version of bomb_task_builder.py.
 * Enumerates every bomb x internal-wall pair and sorts tasks by:
 * 1) shorter Manhattan distance first;
 * 2) when distance ties, more affected walls first.
 */
int bomb_task_builder_generate(const PlannerGrid *grid,
                               BombTask tasks[],
                               int max_tasks,
                               int *out_task_count);

#ifdef __cplusplus
}
#endif

#endif /* BOMB_TASK_BUILDER_H */
