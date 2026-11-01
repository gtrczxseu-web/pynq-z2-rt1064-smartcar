#include "bomb_rule_engine.h"

static int in_bounds(int r, int c)
{
    return r >= 0 && r < PLANNER_ROWS && c >= 0 && c < PLANNER_COLS;
}

int bomb_rule_is_border_cell(int r, int c)
{
    if(!in_bounds(r, c)) {
        return 0;
    }
    return r == 0 || r == PLANNER_ROWS - 1 || c == 0 || c == PLANNER_COLS - 1;
}

int bomb_rule_is_border_wall(const PlannerGrid *grid, PlannerPoint p)
{
    if(grid == 0 || !planner_point_valid(p)) {
        return 0;
    }
    return bomb_rule_is_border_cell(p.r, p.c) && grid->grid[p.r][p.c] == '#';
}

int bomb_rule_is_internal_wall(const PlannerGrid *grid, PlannerPoint p)
{
    if(grid == 0 || !planner_point_valid(p)) {
        return 0;
    }
    if(bomb_rule_is_border_cell(p.r, p.c)) {
        return 0;
    }
    return grid->grid[p.r][p.c] == '#';
}

int bomb_rule_is_valid_target_wall(const PlannerGrid *grid, PlannerPoint target_wall)
{
    return bomb_rule_is_internal_wall(grid, target_wall);
}

int bomb_rule_get_affected_walls(const PlannerGrid *grid,
                                 PlannerPoint target_wall,
                                 PlannerPoint affected_walls[],
                                 int max_affected_walls)
{
    int count = 0;
    int dr, dc;

    if(grid == 0 || affected_walls == 0 || max_affected_walls <= 0) {
        return -1;
    }

    if(!bomb_rule_is_valid_target_wall(grid, target_wall)) {
        return -1;
    }

    for(dr = -1; dr <= 1; dr++) {
        for(dc = -1; dc <= 1; dc++) {
            PlannerPoint p;
            p.r = (int16_t)(target_wall.r + dr);
            p.c = (int16_t)(target_wall.c + dc);

            if(!planner_point_valid(p)) {
                continue;
            }
            if(!bomb_rule_is_internal_wall(grid, p)) {
                continue;
            }
            if(count < max_affected_walls) {
                affected_walls[count] = p;
            }
            count++;
        }
    }

    if(count > max_affected_walls) {
        return max_affected_walls;
    }
    return count;
}
