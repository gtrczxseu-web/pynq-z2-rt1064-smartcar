#include "bomb_then_box_solver.h"
#include "bomb_task_builder.h"
#include "bomb_push_planner.h"
#include "bomb_blast_executor.h"
#include "solver.h"
#include <string.h>

static void result_clear(BombThenBoxResult *res)
{
    if(res == 0) {
        return;
    }
    memset(res, 0, sizeof(*res));
    res->success = 0;
    res->total_cost = 0;
}

static void result_set_reason(BombThenBoxResult *res, const char *reason)
{
    if(res == 0 || reason == 0) {
        return;
    }
    strncpy(res->reason, reason, sizeof(res->reason) - 1);
}

static void result_set_mode(BombThenBoxResult *res, const char *mode)
{
    if(res == 0 || mode == 0) {
        return;
    }
    strncpy(res->mode, mode, sizeof(res->mode) - 1);
}

static int path_join(const char *a, const char *b, char *out, int out_size)
{
    int len_a;
    int len_b;

    if(a == 0 || b == 0 || out == 0 || out_size <= 0) {
        return 0;
    }

    len_a = (int)strlen(a);
    len_b = (int)strlen(b);
    if(len_a + len_b >= out_size) {
        out[0] = '\0';
        return 0;
    }

    strcpy(out, a);
    strcpy(out + len_a, b);
    return 1;
}

static int is_better_candidate(const BombThenBoxResult *candidate,
                               const BombThenBoxResult *best)
{
    if(candidate == 0 || !candidate->success) {
        return 0;
    }

    if(best == 0 || !best->success) {
        return 1;
    }

    if(candidate->total_cost < best->total_cost) {
        return 1;
    }

    if(candidate->total_cost > best->total_cost) {
        return 0;
    }

    /* Tie-breakers preserve the Python-stage intent: prefer a shorter bomb action first,
     * then prefer the blast that removes more internal walls.
     */
    if(candidate->bomb_path_len < best->bomb_path_len) {
        return 1;
    }
    if(candidate->bomb_path_len > best->bomb_path_len) {
        return 0;
    }

    return candidate->bomb_task.affected_wall_count > best->bomb_task.affected_wall_count;
}

int bomb_then_box_solver_solve(const PlannerGrid *grid,
                               BombThenBoxResult *out_result)
{
    BombTask tasks[PLANNER_MAX_BOMB_TASKS];
    int task_count = 0;
    int i;
    BombThenBoxResult best;

    result_clear(out_result);

    if(grid == 0 || out_result == 0) {
        return 0;
    }

    if(!bomb_task_builder_generate(grid, tasks, PLANNER_MAX_BOMB_TASKS, &task_count)) {
        result_set_reason(out_result, "bomb_task_build_failed");
        result_set_mode(out_result, "bomb_then_box");
        return 0;
    }

    if(task_count <= 0) {
        result_set_reason(out_result, "no_bomb_tasks");
        result_set_mode(out_result, "bomb_then_box");
        return 0;
    }

    result_clear(&best);

    for(i = 0; i < task_count; i++) {
        BombPushResult push_result;
        PlannerGrid blast_grid;
        char box_path[PLANNER_MAX_PATH_LEN];
        char full_path[PLANNER_MAX_PATH_LEN];
        int bomb_len;
        int box_len;
        BombThenBoxResult candidate;

        if(!bomb_push_planner_solve(grid, &tasks[i], &push_result)) {
            continue;
        }

        if(!bomb_blast_executor_apply(grid, &tasks[i], &push_result, &blast_grid)) {
            continue;
        }

        if(!solver_solve_subject1_best_path(&blast_grid, box_path, sizeof(box_path))) {
            continue;
        }

        if(!path_join(push_result.path, box_path, full_path, sizeof(full_path))) {
            continue;
        }

        bomb_len = (int)strlen(push_result.path);
        box_len = (int)strlen(box_path);

        result_clear(&candidate);
        candidate.success = 1;
        result_set_reason(&candidate, "ok");
        result_set_mode(&candidate, "bomb_then_box");
        strncpy(candidate.bomb_path, push_result.path, sizeof(candidate.bomb_path) - 1);
        strncpy(candidate.box_path_after_blast, box_path, sizeof(candidate.box_path_after_blast) - 1);
        strncpy(candidate.full_path_simulated, full_path, sizeof(candidate.full_path_simulated) - 1);
        candidate.bomb_path_len = bomb_len;
        candidate.box_path_after_blast_len = box_len;
        candidate.full_path_simulated_len = bomb_len + box_len;
        candidate.total_cost = bomb_len + box_len;
        candidate.bomb_task = tasks[i];
        candidate.bomb_push_result = push_result;
        solver_clone_grid(&blast_grid, &candidate.blast_grid);

        if(is_better_candidate(&candidate, &best)) {
            best = candidate;
        }
    }

    if(best.success) {
        *out_result = best;
        return 1;
    }

    result_set_reason(out_result, "no_valid_bomb_then_box_plan");
    result_set_mode(out_result, "bomb_then_box");
    return 0;
}
