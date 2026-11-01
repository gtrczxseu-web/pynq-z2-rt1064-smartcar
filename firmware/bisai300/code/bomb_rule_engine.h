#ifndef BOMB_RULE_ENGINE_H
#define BOMB_RULE_ENGINE_H

#include "planner_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* C version of bomb_rule_engine.py.
 * Rule summary:
 * - A bomb must be pushed into an internal wall.
 * - The target wall is the blast center.
 * - Blast range is 3 x 3 centered on target_wall.
 * - Only internal walls are removed.
 * - Border walls are never removable.
 */

int bomb_rule_is_border_cell(int r, int c);
int bomb_rule_is_border_wall(const PlannerGrid *grid, PlannerPoint p);
int bomb_rule_is_internal_wall(const PlannerGrid *grid, PlannerPoint p);
int bomb_rule_is_valid_target_wall(const PlannerGrid *grid, PlannerPoint target_wall);

/* Fill affected_walls with removable internal walls in the 3x3 blast area.
 * Returns affected wall count, or -1 when target_wall is invalid.
 */
int bomb_rule_get_affected_walls(const PlannerGrid *grid,
                                 PlannerPoint target_wall,
                                 PlannerPoint affected_walls[],
                                 int max_affected_walls);

#ifdef __cplusplus
}
#endif

#endif /* BOMB_RULE_ENGINE_H */
