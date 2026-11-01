#ifndef BOMB_THEN_BOX_SOLVER_H
#define BOMB_THEN_BOX_SOLVER_H

#include "bomb_blast_executor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int success;                                      /* 1 = found useful bomb plan, 0 = failed */
    char reason[48];                                  /* ok / no_bomb_tasks / no_valid_bomb_then_box_plan / ... */
    char mode[32];                                    /* bomb_then_box */

    char bomb_path[PLANNER_MAX_PATH_LEN];             /* path to execute on real car now */
    int bomb_path_len;

    char box_path_after_blast[PLANNER_MAX_PATH_LEN];   /* only for validation/debug; real car should re-detect before using it */
    int box_path_after_blast_len;

    char full_path_simulated[PLANNER_MAX_PATH_LEN];    /* bomb_path + box_path_after_blast, PC-style simulation only */
    int full_path_simulated_len;

    int total_cost;                                    /* len(bomb_path) + len(box_path_after_blast) */
    BombTask bomb_task;
    BombPushResult bomb_push_result;
    PlannerGrid blast_grid;
} BombThenBoxResult;

/* C version of bomb_then_box_solver.py.
 * Full simulation logic:
 *   generate bomb tasks -> push bomb to target wall -> simulate blast -> run normal box solver.
 * The result stores both bomb_path and after-blast box_path, but the real car should execute
 * only bomb_path and then wait for a fresh OpenART map.
 */
int bomb_then_box_solver_solve(const PlannerGrid *grid,
                               BombThenBoxResult *out_result);

#ifdef __cplusplus
}
#endif

#endif /* BOMB_THEN_BOX_SOLVER_H */
