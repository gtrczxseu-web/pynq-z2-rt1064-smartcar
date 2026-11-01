#ifndef BOMB_PUSH_PLANNER_H
#define BOMB_PUSH_PLANNER_H

#include "bomb_task_builder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int success;
    char reason[48];
    char path[PLANNER_MAX_PATH_LEN];      /* U/D/L/R player movement path */
    int path_len;
    PlannerPoint final_player;            /* player position after pushing bomb into target_wall */
    PlannerPoint final_bomb;              /* equals task.target_wall when success */
} BombPushResult;

/* C version of bomb_push_planner.py.
 * BFS state: (player_r, player_c, bomb_r, bomb_c).
 * Last push may move bomb into task.target_wall; ordinary bomb movement may not enter walls.
 */
int bomb_push_planner_solve(const PlannerGrid *grid,
                            const BombTask *task,
                            BombPushResult *out_result);

#ifdef __cplusplus
}
#endif

#endif /* BOMB_PUSH_PLANNER_H */
