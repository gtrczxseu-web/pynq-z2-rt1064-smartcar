#include "multi_bomb_step_solver.h"
#include "bomb_fallback_api.h"
#include "solver_api.h"
#include <string.h>

static void set_reason(char *dst, int dst_size, const char *reason)
{
    if(dst == 0 || dst_size <= 0) {
        return;
    }
    if(reason == 0) {
        reason = "unknown";
    }
    strncpy(dst, reason, (unsigned int)(dst_size - 1));
    dst[dst_size - 1] = '\0';
}

static void result_clear(MultiBombStepResult *res)
{
    if(res == 0) {
        return;
    }
    memset(res, 0, sizeof(*res));
    strncpy(res->mode, "fail", sizeof(res->mode) - 1);
    strncpy(res->reason, "unknown", sizeof(res->reason) - 1);
}

static int count_symbol(const PlannerGrid *grid, char symbol)
{
    return solver_api_count_symbol(grid, symbol);
}

static int normal_attempt(const PlannerGrid *grid, PlannerResult *out_normal)
{
    PlannerGrid normal_grid;
    bomb_fallback_make_normal_planning_grid(grid, &normal_grid);
    return solver_api_solve_map(&normal_grid, out_normal);
}

static int normal_goal_evaluator(const PlannerGrid *grid,
                                 void *context,
                                 int *out_remaining_cost)
{
    PlannerResult normal;
    (void)context;

    if(out_remaining_cost != 0) {
        *out_remaining_cost = 0;
    }

    if(!normal_attempt(grid, &normal)) {
        return 0;
    }

    if(out_remaining_cost != 0) {
        *out_remaining_cost = normal.path_len;
    }
    return 1;
}

static int make_candidate(const PlannerGrid *state,
                          const BombTask *task,
                          const BombPushResult *push,
                          PlannerGrid *out_blast_grid)
{
    int bombs_before;
    int bombs_after;

    if(state == 0 || task == 0 || push == 0 || out_blast_grid == 0) {
        return 0;
    }
    if(!push->success || push->path_len <= 0) {
        return 0;
    }

    bombs_before = count_symbol(state, '*');
    if(!bomb_blast_executor_apply(state, task, push, out_blast_grid)) {
        return 0;
    }
    bombs_after = count_symbol(out_blast_grid, '*');
    return bombs_after == bombs_before - 1;
}

typedef struct {
    int nodes;
    int task_attempts;
    int max_nodes;
    int max_task_attempts;
    int truncated;
} SearchBudget;

typedef struct {
    BombTask first_task;
    BombPushResult first_push;
    PlannerGrid first_blast_grid;
} FirstStep;

typedef struct {
    int success;
    int total_cost;
    int bomb_path_cost;
    int remaining_cost;
    int bomb_steps;
    int first_path_len;
    int first_affected_walls;
} SearchScore;

/*
 * 递归层的大对象放到静态工作区，避免 256 个 BombTask 与 4KB 路径缓冲
 * 在 MCU 调用栈上按炸弹深度重复分配。求解器由主循环串行调用，因此无需重入。
 */
static BombTask g_task_workspace[PLANNER_MAX_BOMB_CHAIN_DEPTH + 1][PLANNER_MAX_BOMB_TASKS];
static BombPushResult g_push_workspace[PLANNER_MAX_BOMB_CHAIN_DEPTH + 1];
static PlannerGrid g_blast_workspace[PLANNER_MAX_BOMB_CHAIN_DEPTH + 1];

static void score_clear(SearchScore *score)
{
    if(score != 0) {
        memset(score, 0, sizeof(*score));
    }
}

static int score_better(const SearchScore *candidate, const SearchScore *best)
{
    if(candidate == 0 || !candidate->success) {
        return 0;
    }
    if(best == 0 || !best->success) {
        return 1;
    }
    if(candidate->total_cost != best->total_cost) {
        return candidate->total_cost < best->total_cost;
    }
    if(candidate->bomb_path_cost != best->bomb_path_cost) {
        return candidate->bomb_path_cost < best->bomb_path_cost;
    }
    if(candidate->first_path_len != best->first_path_len) {
        return candidate->first_path_len < best->first_path_len;
    }
    return candidate->first_affected_walls > best->first_affected_walls;
}

enum {
    SEARCH_REASON_NONE = 0,
    SEARCH_REASON_NULL = 1,
    SEARCH_REASON_NODE_LIMIT = 2,
    SEARCH_REASON_DEPTH_LIMIT = 3,
    SEARCH_REASON_NO_BOMB = 4,
    SEARCH_REASON_TASK_BUILD = 5,
    SEARCH_REASON_TASK_LIMIT = 6,
    SEARCH_REASON_NO_SEQUENCE = 7
};

static int recursive_search(const PlannerGrid *state,
                            int steps_left,
                            int recursion_level,
                            SearchBudget *budget,
                            MultiBombGoalEvaluator evaluator,
                            void *context,
                            SearchScore *out_score,
                            FirstStep *out_first,
                            int is_root,
                            int *out_reason_code)
{
    BombTask *tasks;
    SearchScore best;
    int task_count = 0;
    int remaining_cost = 0;
    int i;

    score_clear(out_score);
    score_clear(&best);

    if(out_reason_code != 0) {
        *out_reason_code = SEARCH_REASON_NONE;
    }

    if(state == 0 || budget == 0 || evaluator == 0 || out_score == 0 ||
       recursion_level < 0 || recursion_level > PLANNER_MAX_BOMB_CHAIN_DEPTH) {
        if(out_reason_code != 0) {
            *out_reason_code = SEARCH_REASON_NULL;
        }
        return 0;
    }

    if(budget->nodes >= budget->max_nodes) {
        budget->truncated = 1;
        if(out_reason_code != 0) {
            *out_reason_code = SEARCH_REASON_NODE_LIMIT;
        }
        return 0;
    }
    budget->nodes++;

    if(evaluator(state, context, &remaining_cost)) {
        out_score->success = 1;
        out_score->total_cost = remaining_cost;
        out_score->remaining_cost = remaining_cost;
        return 1;
    }

    if(steps_left <= 0) {
        if(out_reason_code != 0) {
            *out_reason_code = SEARCH_REASON_DEPTH_LIMIT;
        }
        return 0;
    }

    if(count_symbol(state, '*') <= 0) {
        if(out_reason_code != 0) {
            *out_reason_code = SEARCH_REASON_NO_BOMB;
        }
        return 0;
    }

    tasks = g_task_workspace[recursion_level];

    if(!bomb_task_builder_generate(state, tasks, PLANNER_MAX_BOMB_TASKS, &task_count)) {
        if(out_reason_code != 0) {
            *out_reason_code = SEARCH_REASON_TASK_BUILD;
        }
        return 0;
    }

    for(i = 0; i < task_count; i++) {
        BombPushResult *push = &g_push_workspace[recursion_level];
        PlannerGrid *blast_grid = &g_blast_workspace[recursion_level];
        SearchScore child;
        SearchScore candidate;
        int child_reason = SEARCH_REASON_NONE;

        if(budget->nodes >= budget->max_nodes ||
           budget->task_attempts >= budget->max_task_attempts) {
            budget->truncated = 1;
            if(out_reason_code != 0) {
                *out_reason_code = (budget->nodes >= budget->max_nodes) ?
                                   SEARCH_REASON_NODE_LIMIT :
                                   SEARCH_REASON_TASK_LIMIT;
            }
            break;
        }
        budget->task_attempts++;

        if(!bomb_push_planner_solve(state, &tasks[i], push)) {
            continue;
        }
        if(!make_candidate(state, &tasks[i], push, blast_grid)) {
            continue;
        }

        score_clear(&child);
        if(!recursive_search(blast_grid,
                             steps_left - 1,
                             recursion_level + 1,
                             budget,
                             evaluator,
                             context,
                             &child,
                             out_first,
                             0,
                             &child_reason)) {
            continue;
        }

        score_clear(&candidate);
        candidate.success = 1;
        candidate.total_cost = push->path_len + child.total_cost;
        candidate.bomb_path_cost = push->path_len + child.bomb_path_cost;
        candidate.remaining_cost = child.remaining_cost;
        candidate.bomb_steps = 1 + child.bomb_steps;
        candidate.first_path_len = push->path_len;
        candidate.first_affected_walls = tasks[i].affected_wall_count;

        if(score_better(&candidate, &best)) {
            best = candidate;
            if(is_root && out_first != 0) {
                out_first->first_task = tasks[i];
                out_first->first_push = *push;
                memcpy(&out_first->first_blast_grid, blast_grid, sizeof(*blast_grid));
            }
        }
    }

    if(best.success) {
        *out_score = best;
        return 1;
    }

    if(out_reason_code != 0 && *out_reason_code == SEARCH_REASON_NONE) {
        *out_reason_code = SEARCH_REASON_NO_SEQUENCE;
    }
    return 0;
}

static void reason_from_code(int code, char *out, int out_size)
{
    const char *reason = "no_sequence_within_depth";

    if(code == SEARCH_REASON_NULL) {
        reason = "null_argument";
    } else if(code == SEARCH_REASON_NODE_LIMIT) {
        reason = "search_node_limit";
    } else if(code == SEARCH_REASON_DEPTH_LIMIT) {
        reason = "depth_limit";
    } else if(code == SEARCH_REASON_NO_BOMB) {
        reason = "no_bomb_left";
    } else if(code == SEARCH_REASON_TASK_BUILD) {
        reason = "build_bomb_tasks_failed";
    } else if(code == SEARCH_REASON_TASK_LIMIT) {
        reason = "task_attempt_limit";
    }

    set_reason(out, out_size, reason);
}

int multi_bomb_step_solver_solve_for_goal(const PlannerGrid *grid,
                                          int max_bomb_depth,
                                          MultiBombGoalEvaluator evaluator,
                                          void *context,
                                          MultiBombStepResult *out_result)
{
    int bombs_before;
    int initial_cost = 0;
    int depth;
    int final_reason = SEARCH_REASON_NONE;

    result_clear(out_result);

    if(grid == 0 || evaluator == 0 || out_result == 0) {
        if(out_result != 0) {
            set_reason(out_result->reason, (int)sizeof(out_result->reason), "null_argument");
        }
        return 0;
    }

    if(evaluator(grid, context, &initial_cost)) {
        set_reason(out_result->reason, (int)sizeof(out_result->reason), "goal_already_satisfied");
        return 0;
    }

    bombs_before = count_symbol(grid, '*');
    if(bombs_before <= 0) {
        set_reason(out_result->reason, (int)sizeof(out_result->reason), "no_bomb");
        return 0;
    }

    if(max_bomb_depth <= 0) {
        max_bomb_depth = bombs_before;
    }
    if(max_bomb_depth > bombs_before) {
        max_bomb_depth = bombs_before;
    }
    if(max_bomb_depth > PLANNER_MAX_BOMB_CHAIN_DEPTH) {
        max_bomb_depth = PLANNER_MAX_BOMB_CHAIN_DEPTH;
    }

    for(depth = 1; depth <= max_bomb_depth; depth++) {
        SearchBudget budget;
        SearchScore score;
        FirstStep first;
        int reason_code = SEARCH_REASON_NONE;

        memset(&first, 0, sizeof(first));
        score_clear(&score);
        budget.nodes = 0;
        budget.task_attempts = 0;
        budget.max_nodes = PLANNER_MAX_MULTI_BOMB_NODES;
        budget.max_task_attempts = PLANNER_MAX_MULTI_BOMB_TASK_ATTEMPTS;
        budget.truncated = 0;

        if(recursive_search(grid,
                            depth,
                            0,
                            &budget,
                            evaluator,
                            context,
                            &score,
                            &first,
                            1,
                            &reason_code) &&
           score.success && score.bomb_steps > 0) {
            out_result->success = 1;
            strncpy(out_result->mode, "bomb_step", sizeof(out_result->mode) - 1);
            set_reason(out_result->reason, (int)sizeof(out_result->reason), "ok");
            strncpy(out_result->bomb_path, first.first_push.path, sizeof(out_result->bomb_path) - 1);
            out_result->bomb_path[sizeof(out_result->bomb_path) - 1] = '\0';
            strncpy(out_result->path, out_result->bomb_path, sizeof(out_result->path) - 1);
            out_result->path[sizeof(out_result->path) - 1] = '\0';
            out_result->bomb_path_len = (int)strlen(out_result->bomb_path);
            out_result->path_len = out_result->bomb_path_len;
            out_result->bomb_task = first.first_task;
            out_result->bomb_push_result = first.first_push;
            memcpy(&out_result->blast_grid, &first.first_blast_grid, sizeof(out_result->blast_grid));
            out_result->planned_bomb_steps = score.bomb_steps;
            out_result->nodes_searched = budget.nodes;
            out_result->task_attempts = budget.task_attempts;
            out_result->depth_used = depth;
            out_result->preview_total_cost = score.total_cost;
            out_result->preview_bomb_path_cost = score.bomb_path_cost;
            out_result->preview_remaining_cost = score.remaining_cost;
            out_result->search_truncated = budget.truncated;
            return 1;
        }

        final_reason = reason_code;
    }

    reason_from_code(final_reason, out_result->reason, (int)sizeof(out_result->reason));
    out_result->success = 0;
    strncpy(out_result->mode, "fail", sizeof(out_result->mode) - 1);
    return 0;
}

int multi_bomb_step_solver_solve_one_step(const PlannerGrid *grid,
                                          int max_bomb_depth,
                                          MultiBombStepResult *out_result)
{
    PlannerResult initial_normal;

    result_clear(out_result);
    if(grid == 0 || out_result == 0) {
        if(out_result != 0) {
            set_reason(out_result->reason, (int)sizeof(out_result->reason), "null_argument");
        }
        return 0;
    }

    if(normal_attempt(grid, &initial_normal)) {
        set_reason(out_result->reason, (int)sizeof(out_result->reason), "normal_solution_already_exists");
        return 0;
    }
    if(!bomb_fallback_is_normal_no_solution(&initial_normal)) {
        set_reason(out_result->reason, (int)sizeof(out_result->reason), "normal_failure_not_bomb_eligible");
        return 0;
    }

    return multi_bomb_step_solver_solve_for_goal(grid,
                                                  max_bomb_depth,
                                                  normal_goal_evaluator,
                                                  0,
                                                  out_result);
}
