#ifndef SOLVER_API_H
#define SOLVER_API_H

#include "planner_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* C version of solver_api.py / Stage22.
 * Provides a stable ordinary-box-planning entry point around solver.c.
 * It validates the map and returns a normalized PlannerResult.
 */
int solver_api_solve_map(const PlannerGrid *grid, PlannerResult *out_result);

/* Utility helpers used by higher-level Stage24/28 controller code. */
int solver_api_normalize_grid(const PlannerGrid *grid, PlannerGrid *out_grid, char *reason, int reason_size);
int solver_api_count_symbol(const PlannerGrid *grid, char symbol);
int solver_api_find_player(const PlannerGrid *grid, PlannerPoint *out_player);
int solver_api_has_symbol(const PlannerGrid *grid, char symbol);

#ifdef __cplusplus
}
#endif

#endif /* SOLVER_API_H */
