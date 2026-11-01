#ifndef OPENART_ADAPTER_H
#define OPENART_ADAPTER_H

#include "planner_types.h"
#include "planner_controller_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Compatibility wrapper for older main.c/tests.
 * New Stage28 projects should call planner_controller_plan_next_action() directly.
 * This wrapper maps Stage28 result into legacy PlannerResult:
 *   normal_step / bomb_step / done / fail
 * path is only the current executable step, never an old full path.
 */
int openart_adapter_solve_grid(const PlannerGrid *grid,
                               int enable_bomb_solver,
                               PlannerResult *out_result);

/* Direct Stage28 adapter entry for callers that still want an OpenART-named API. */
int openart_adapter_plan_next_action(const PlannerGrid *grid,
                                     int max_bomb_depth,
                                     PlannerControllerResult *out_result);

/* Convenience wrapper for raw OpenART symbol grids. */
int openart_adapter_solve_raw_grid(const char raw_grid[PLANNER_ROWS][PLANNER_COLS],
                                   int enable_bomb_solver,
                                   PlannerResult *out_result);

int openart_adapter_plan_next_action_raw(const char raw_grid[PLANNER_ROWS][PLANNER_COLS],
                                         int max_bomb_depth,
                                         PlannerControllerResult *out_result);

#ifdef __cplusplus
}
#endif

#endif /* OPENART_ADAPTER_H */
