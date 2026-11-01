#include "bomb_blast_executor.h"
#include "bomb_rule_engine.h"
#include <string.h>

static void clone_grid(const PlannerGrid *src, PlannerGrid *dst)
{
    memcpy(dst->grid, src->grid, sizeof(dst->grid));
}

static void clear_all_players(PlannerGrid *grid)
{
    int r, c;
    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            if(grid->grid[r][c] == '@') {
                grid->grid[r][c] = '-';
            }
        }
    }
}

static void clear_if_valid(PlannerGrid *grid, PlannerPoint p)
{
    if(planner_point_valid(p)) {
        grid->grid[p.r][p.c] = '-';
    }
}

int bomb_blast_executor_apply(const PlannerGrid *grid,
                              const BombTask *task,
                              const BombPushResult *push_result,
                              PlannerGrid *out_grid)
{
    int i;

    if(grid == 0 || task == 0 || push_result == 0 || out_grid == 0) {
        return 0;
    }

    if(!push_result->success) {
        return 0;
    }

    if(!planner_point_equal(push_result->final_bomb, task->target_wall)) {
        return 0;
    }

    clone_grid(grid, out_grid);

    /* Remove the old player marker first. */
    clear_all_players(out_grid);

    /* The original bomb position becomes empty after the bomb has been pushed away. */
    clear_if_valid(out_grid, task->bomb);

    /* Remove target wall and every affected internal wall. Border walls are protected. */
    if(bomb_rule_is_internal_wall(grid, task->target_wall)) {
        clear_if_valid(out_grid, task->target_wall);
    }

    for(i = 0; i < task->affected_wall_count && i < BOMB_MAX_AFFECTED_WALLS; i++) {
        PlannerPoint p = task->affected_walls[i];
        if(bomb_rule_is_internal_wall(grid, p)) {
            clear_if_valid(out_grid, p);
        }
    }

    if(!planner_point_valid(push_result->final_player)) {
        return 0;
    }

    out_grid->grid[push_result->final_player.r][push_result->final_player.c] = '@';
    return 1;
}
