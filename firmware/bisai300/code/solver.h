#ifndef SOLVER_H
#define SOLVER_H

#include "planner_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Equivalent to Python solve_subject1_best_path(state, verbose=False).
 * Returns 1 when a U/D/L/R path is found, 0 otherwise.
 */
int solver_solve_subject1_best_path(const PlannerGrid *state_grid,
                                    char *out_path,
                                    int out_path_size);

/* Equivalent to Python solve_single_box_bfs(submap). */
int solver_solve_single_box_bfs(const PlannerGrid *submap,
                                char *out_path,
                                int out_path_size);

/* Utility functions corresponding to simulator/solver helpers. */
void solver_clone_grid(const PlannerGrid *src, PlannerGrid *dst);
int solver_extract_positions(const PlannerGrid *grid,
                             PlannerPoint *player,
                             PlannerPoint boxes[], int *box_count,
                             PlannerPoint targets[], int *target_count);
int solver_simulate_path(const PlannerGrid *initial_grid,
                         const char *path,
                         PlannerPoint player,
                         PlannerPoint box,
                         PlannerPoint target,
                         PlannerGrid *out_grid,
                         PlannerPoint *out_player);

#ifdef __cplusplus
}
#endif

#endif /* SOLVER_H */
