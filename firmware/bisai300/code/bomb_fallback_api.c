#include "bomb_fallback_api.h"
#include <string.h>

void bomb_fallback_make_normal_planning_grid(const PlannerGrid *grid, PlannerGrid *out_grid)
{
    int r, c;
    if(grid == 0 || out_grid == 0) {
        return;
    }
    memcpy(out_grid, grid, sizeof(*out_grid));
    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            if(out_grid->grid[r][c] == '*') {
                out_grid->grid[r][c] = '#';
            }
        }
    }
}

int bomb_fallback_is_normal_no_solution(const PlannerResult *normal_result)
{
    if(normal_result == 0) {
        return 0;
    }
    if(normal_result->success) {
        return 0;
    }
    if(strcmp(normal_result->reason, "normal_no_solution") == 0) {
        return 1;
    }
    if(strstr(normal_result->reason, "no_solution") != 0) {
        return 1;
    }
    if(strstr(normal_result->reason, "no solution") != 0) {
        return 1;
    }
    return 0;
}

int bomb_fallback_has_bomb(const PlannerGrid *grid)
{
    int r, c;
    if(grid == 0) {
        return 0;
    }
    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            if(grid->grid[r][c] == '*') {
                return 1;
            }
        }
    }
    return 0;
}
