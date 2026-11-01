#include "openart_adapter.h"
#include "openart_symbol_mapper.h"
#include "solver_api.h"
#include <string.h>

static void result_clear(PlannerResult *res)
{
    if(res == 0) {
        return;
    }
    memset(res, 0, sizeof(*res));
}

static void result_set(PlannerResult *res, int success,
                       const char *reason,
                       const char *mode,
                       const char *path)
{
    result_clear(res);
    if(res == 0) {
        return;
    }
    res->success = success;
    if(reason != 0) {
        strncpy(res->reason, reason, sizeof(res->reason) - 1);
        res->reason[sizeof(res->reason) - 1] = '\0';
    }
    if(mode != 0) {
        strncpy(res->mode, mode, sizeof(res->mode) - 1);
        res->mode[sizeof(res->mode) - 1] = '\0';
    }
    if(path != 0) {
        strncpy(res->path, path, sizeof(res->path) - 1);
        res->path[sizeof(res->path) - 1] = '\0';
        res->path_len = (int)strlen(res->path);
    }
}

static int grid_has_char(const PlannerGrid *grid, char target)
{
    int r, c;
    if(grid == 0) {
        return 0;
    }
    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            if(grid->grid[r][c] == target) {
                return 1;
            }
        }
    }
    return 0;
}

int openart_adapter_plan_next_action(const PlannerGrid *grid,
                                     int max_bomb_depth,
                                     PlannerControllerResult *out_result)
{
    return planner_controller_plan_next_action(grid, max_bomb_depth, out_result);
}

int openart_adapter_solve_grid(const PlannerGrid *grid,
                               int enable_bomb_solver,
                               PlannerResult *out_result)
{
    PlannerControllerResult controller;

    result_clear(out_result);

    if(grid == 0 || out_result == 0) {
        return 0;
    }

    /* Preserve the old adapter's explicit opt-in behavior for bomb maps. */
    if(grid_has_char(grid, '*') && !enable_bomb_solver) {
        result_set(out_result, 0, "unsupported_bomb_map", "unsupported_bomb_map", "");
        return 0;
    }

    if(!planner_controller_plan_next_action(grid,
            enable_bomb_solver ? PLANNER_MAX_BOMB_CHAIN_DEPTH : 0,
            &controller)) {
        result_set(out_result, 0, controller.reason, controller.mode, "");
        return 0;
    }

    result_set(out_result,
               controller.success,
               controller.reason,
               controller.mode,
               controller.path);
    return controller.success;
}

int openart_adapter_solve_raw_grid(const char raw_grid[PLANNER_ROWS][PLANNER_COLS],
                                   int enable_bomb_solver,
                                   PlannerResult *out_result)
{
    PlannerGrid mapped;
    if(!openart_symbol_mapper_raw_to_solver(raw_grid, &mapped)) {
        result_set(out_result, 0, "raw_map_convert_failed", "mapper", "");
        return 0;
    }
    return openart_adapter_solve_grid(&mapped, enable_bomb_solver, out_result);
}

int openart_adapter_plan_next_action_raw(const char raw_grid[PLANNER_ROWS][PLANNER_COLS],
                                         int max_bomb_depth,
                                         PlannerControllerResult *out_result)
{
    PlannerGrid mapped;
    if(!openart_symbol_mapper_raw_to_solver(raw_grid, &mapped)) {
        if(out_result != 0) {
            memset(out_result, 0, sizeof(*out_result));
            out_result->success = 0;
            out_result->mode_enum = PLAN_CONTROLLER_MODE_FAIL;
            strncpy(out_result->mode, "fail", sizeof(out_result->mode) - 1);
            strncpy(out_result->reason, "raw_map_convert_failed", sizeof(out_result->reason) - 1);
        }
        return 0;
    }
    return planner_controller_plan_next_action(&mapped, max_bomb_depth, out_result);
}
