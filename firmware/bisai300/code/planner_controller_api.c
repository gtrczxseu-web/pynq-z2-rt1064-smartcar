#include "planner_controller_api.h"
#include "solver_api.h"
#include "bomb_fallback_api.h"
#include <string.h>

static void set_text(char *dst, int dst_size, const char *src)
{
    if(dst == 0 || dst_size <= 0) {
        return;
    }
    if(src == 0) {
        src = "";
    }
    strncpy(dst, src, (unsigned int)(dst_size - 1));
    dst[dst_size - 1] = '\0';
}

const char *planner_controller_mode_to_string(PlannerControllerMode mode)
{
    switch(mode) {
        case PLAN_CONTROLLER_MODE_NORMAL_STEP:
            return "normal_step";
        case PLAN_CONTROLLER_MODE_BOMB_STEP:
            return "bomb_step";
        case PLAN_CONTROLLER_MODE_DONE:
            return "done";
        case PLAN_CONTROLLER_MODE_FAIL:
        default:
            return "fail";
    }
}

static void result_clear(PlannerControllerResult *res)
{
    if(res == 0) {
        return;
    }
    memset(res, 0, sizeof(*res));
    res->success = 0;
    res->mode_enum = PLAN_CONTROLLER_MODE_FAIL;
    set_text(res->mode, (int)sizeof(res->mode), "fail");
    set_text(res->reason, (int)sizeof(res->reason), "unknown");
    res->player_before.r = -1;
    res->player_before.c = -1;
    res->predicted_player_after.r = -1;
    res->predicted_player_after.c = -1;
    res->consumed_goal.r = -1;
    res->consumed_goal.c = -1;
}

static void result_fail(PlannerControllerResult *res, const char *reason)
{
    if(res == 0) {
        return;
    }
    res->success = 0;
    res->mode_enum = PLAN_CONTROLLER_MODE_FAIL;
    set_text(res->mode, (int)sizeof(res->mode), "fail");
    set_text(res->reason, (int)sizeof(res->reason), reason);
    set_text(res->action, (int)sizeof(res->action), "none");
    res->requires_reobserve = 0;
    res->path[0] = '\0';
    res->path_len = 0;
}

static int validate_controller_grid(const PlannerGrid *grid,
                                    PlannerGrid *out_state,
                                    PlannerControllerResult *res)
{
    char reason[48];
    int boxes;
    int goals;
    int players;

    if(!solver_api_normalize_grid(grid, out_state, reason, (int)sizeof(reason))) {
        result_fail(res, reason);
        return 0;
    }

    players = solver_api_count_symbol(out_state, '@');
    if(players == 0) {
        result_fail(res, "no_player");
        return 0;
    }
    if(players != 1) {
        result_fail(res, "invalid_player_count");
        return 0;
    }

    boxes = solver_api_count_symbol(out_state, '$');
    goals = solver_api_count_symbol(out_state, '.');
    if(boxes != goals) {
        result_fail(res, "box_goal_count_mismatch");
        return 0;
    }

    return 1;
}

static void fill_counts(const PlannerGrid *grid, PlannerControllerResult *res, int before)
{
    if(grid == 0 || res == 0) {
        return;
    }
    if(before) {
        res->boxes_before = solver_api_count_symbol(grid, '$');
        res->goals_before = solver_api_count_symbol(grid, '.');
        res->bombs_before = solver_api_count_symbol(grid, '*');
    } else {
        res->boxes_after = solver_api_count_symbol(grid, '$');
        res->goals_after = solver_api_count_symbol(grid, '.');
        res->bombs_after = solver_api_count_symbol(grid, '*');
    }
}

static int make_done(const PlannerGrid *state, PlannerControllerResult *res)
{
    res->success = 1;
    res->mode_enum = PLAN_CONTROLLER_MODE_DONE;
    set_text(res->mode, (int)sizeof(res->mode), "done");
    set_text(res->reason, (int)sizeof(res->reason), "ok");
    set_text(res->action, (int)sizeof(res->action), "task_complete");
    res->requires_reobserve = 0;
    res->path[0] = '\0';
    res->path_len = 0;
    memcpy(&res->predicted_grid, state, sizeof(res->predicted_grid));
    solver_api_find_player(state, &res->predicted_player_after);
    fill_counts(state, res, 0);
    return 1;
}

static int make_normal_step(const PlannerGrid *state,
                            const PlannerResult *normal,
                            PlannerControllerResult *res)
{
    SokobanSimResult sim;
    int boxes_before;
    int goals_before;
    int bombs_before;

    if(normal == 0 || !normal->success || normal->path[0] == '\0') {
        result_fail(res, "empty_normal_path_with_remaining_boxes");
        return 0;
    }

    if(!sokoban_simulate_until_first_consumption(state, normal->path, &sim)) {
        res->normal_result = *normal;
        result_fail(res, "normal_execution_failed");
        return 0;
    }
    if(!sim.consumed) {
        res->normal_result = *normal;
        result_fail(res, "normal_plan_without_consumption");
        return 0;
    }

    boxes_before = solver_api_count_symbol(state, '$');
    goals_before = solver_api_count_symbol(state, '.');
    bombs_before = solver_api_count_symbol(state, '*');

    if(sim.boxes_left != boxes_before - 1) {
        res->normal_result = *normal;
        result_fail(res, "invalid_predicted_box_update");
        return 0;
    }
    if(sim.goals_left != goals_before - 1) {
        res->normal_result = *normal;
        result_fail(res, "invalid_predicted_goal_update");
        return 0;
    }
    if(solver_api_count_symbol(&sim.grid, '*') != bombs_before) {
        res->normal_result = *normal;
        result_fail(res, "normal_step_changed_bomb_count");
        return 0;
    }

    res->success = 1;
    res->mode_enum = PLAN_CONTROLLER_MODE_NORMAL_STEP;
    set_text(res->mode, (int)sizeof(res->mode), "normal_step");
    set_text(res->reason, (int)sizeof(res->reason), "ok");
    set_text(res->action, (int)sizeof(res->action), "execute_normal_step");
    res->requires_reobserve = 1;
    set_text(res->path, (int)sizeof(res->path), sim.executed_path);
    res->path_len = (int)strlen(res->path);
    set_text(res->full_planned_path, (int)sizeof(res->full_planned_path), normal->path);
    set_text(res->discarded_path, (int)sizeof(res->discarded_path), sim.remaining_path);
    memcpy(&res->predicted_grid, &sim.grid, sizeof(res->predicted_grid));
    res->predicted_player_after = sim.player;
    res->consumed_goal = sim.consumed_goal;
    res->normal_result = *normal;
    fill_counts(state, res, 1);
    fill_counts(&sim.grid, res, 0);
    return 1;
}

static int make_bomb_step(const PlannerGrid *state,
                          const MultiBombStepResult *bomb,
                          PlannerControllerResult *res)
{
    int boxes_before;
    int goals_before;
    int bombs_before;

    if(bomb == 0 || !bomb->success || bomb->bomb_path[0] == '\0') {
        result_fail(res, "empty_bomb_step_path");
        return 0;
    }

    boxes_before = solver_api_count_symbol(state, '$');
    goals_before = solver_api_count_symbol(state, '.');
    bombs_before = solver_api_count_symbol(state, '*');

    if(solver_api_count_symbol(&bomb->blast_grid, '$') != boxes_before) {
        result_fail(res, "bomb_step_changed_box_count");
        return 0;
    }
    if(solver_api_count_symbol(&bomb->blast_grid, '.') != goals_before) {
        result_fail(res, "bomb_step_changed_goal_count");
        return 0;
    }
    if(solver_api_count_symbol(&bomb->blast_grid, '*') != bombs_before - 1) {
        result_fail(res, "invalid_predicted_bomb_update");
        return 0;
    }

    res->success = 1;
    res->mode_enum = PLAN_CONTROLLER_MODE_BOMB_STEP;
    set_text(res->mode, (int)sizeof(res->mode), "bomb_step");
    set_text(res->reason, (int)sizeof(res->reason), "ok");
    set_text(res->action, (int)sizeof(res->action), "execute_bomb_step");
    res->requires_reobserve = 1;
    set_text(res->path, (int)sizeof(res->path), bomb->bomb_path);
    set_text(res->bomb_path, (int)sizeof(res->bomb_path), bomb->bomb_path);
    res->path_len = (int)strlen(res->path);
    memcpy(&res->predicted_grid, &bomb->blast_grid, sizeof(res->predicted_grid));
    solver_api_find_player(&bomb->blast_grid, &res->predicted_player_after);
    res->bomb_result = *bomb;
    fill_counts(state, res, 1);
    fill_counts(&bomb->blast_grid, res, 0);
    return 1;
}

int planner_controller_plan_next_action(const PlannerGrid *grid,
                                        int max_bomb_depth,
                                        PlannerControllerResult *out_result)
{
    PlannerGrid state;
    PlannerGrid normal_grid;
    PlannerResult normal;
    MultiBombStepResult bomb;
    int boxes;
    int goals;
    int bombs;

    result_clear(out_result);

    if(grid == 0 || out_result == 0) {
        if(out_result != 0) {
            result_fail(out_result, "null_argument");
        }
        return 0;
    }

    if(!validate_controller_grid(grid, &state, out_result)) {
        return 0;
    }

    solver_api_find_player(&state, &out_result->player_before);
    fill_counts(&state, out_result, 1);

    boxes = out_result->boxes_before;
    goals = out_result->goals_before;
    bombs = out_result->bombs_before;

    if(boxes == 0 && goals == 0) {
        return make_done(&state, out_result);
    }

    bomb_fallback_make_normal_planning_grid(&state, &normal_grid);
    if(solver_api_solve_map(&normal_grid, &normal)) {
        return make_normal_step(&state, &normal, out_result);
    }

    out_result->normal_result = normal;
    if(!bomb_fallback_is_normal_no_solution(&normal)) {
        result_fail(out_result, "normal_failure_not_bomb_eligible");
        return 0;
    }

    if(bombs <= 0) {
        result_fail(out_result, "normal_no_solution_and_no_bomb");
        return 0;
    }

    if(!multi_bomb_step_solver_solve_one_step(&state, max_bomb_depth, &bomb)) {
        out_result->bomb_result = bomb;
        result_fail(out_result, bomb.reason);
        return 0;
    }

    return make_bomb_step(&state, &bomb, out_result);
}
