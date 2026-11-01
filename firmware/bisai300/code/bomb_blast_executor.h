#ifndef BOMB_BLAST_EXECUTOR_H
#define BOMB_BLAST_EXECUTOR_H

#include "bomb_push_planner.h"

#ifdef __cplusplus
extern "C" {
#endif

/* C version of bomb_blast_executor.py.
 * Applies the selected bomb task after bomb_push_planner succeeds.
 * - Original bomb disappears.
 * - target_wall and affected internal walls become '-'.
 * - Border walls remain unchanged.
 * - Old player '@' is cleared.
 * - final_player becomes '@'.
 */
int bomb_blast_executor_apply(const PlannerGrid *grid,
                              const BombTask *task,
                              const BombPushResult *push_result,
                              PlannerGrid *out_grid);

#ifdef __cplusplus
}
#endif

#endif /* BOMB_BLAST_EXECUTOR_H */
