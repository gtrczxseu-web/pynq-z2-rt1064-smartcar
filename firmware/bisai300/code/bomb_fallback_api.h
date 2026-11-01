#ifndef BOMB_FALLBACK_API_H
#define BOMB_FALLBACK_API_H

#include "planner_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Stage25 helper: ordinary planning must not use bombs, so bombs are
 * converted to walls only in the temporary normal-planning grid.
 */
void bomb_fallback_make_normal_planning_grid(const PlannerGrid *grid, PlannerGrid *out_grid);
int bomb_fallback_is_normal_no_solution(const PlannerResult *normal_result);
int bomb_fallback_has_bomb(const PlannerGrid *grid);

#ifdef __cplusplus
}
#endif

#endif /* BOMB_FALLBACK_API_H */
