#ifndef MULTI_BOMB_STEP_SOLVER_H
#define MULTI_BOMB_STEP_SOLVER_H

#include "bomb_blast_executor.h"
#include "planner_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PLANNER_MAX_BOMB_CHAIN_DEPTH
#define PLANNER_MAX_BOMB_CHAIN_DEPTH 3
#endif
#ifndef PLANNER_MAX_MULTI_BOMB_NODES
#define PLANNER_MAX_MULTI_BOMB_NODES 400
#endif
#ifndef PLANNER_MAX_MULTI_BOMB_TASK_ATTEMPTS
#define PLANNER_MAX_MULTI_BOMB_TASK_ATTEMPTS 2000
#endif

/*
 * 通用爆破后置条件。
 * 返回 1 表示当前地图已经满足上层任务；out_remaining_cost 返回从当前地图
 * 完成后续任务的估计移动格数，用于同炸弹数量下比较完整方案距离。
 */
typedef int (*MultiBombGoalEvaluator)(const PlannerGrid *grid,
                                      void *context,
                                      int *out_remaining_cost);

typedef struct {
    int success;
    char reason[64];
    char mode[32];
    char path[PLANNER_MAX_PATH_LEN];       /* same as bomb_path */
    int path_len;
    char bomb_path[PLANNER_MAX_PATH_LEN];  /* first bomb only: execute now */
    int bomb_path_len;
    PlannerGrid blast_grid;                /* predicted grid after first bomb */
    BombTask bomb_task;
    BombPushResult bomb_push_result;
    int planned_bomb_steps;                /* number of bombs in preview sequence */
    int nodes_searched;
    int task_attempts;
    int depth_used;

    /* 完整预览成本：所有炸弹路径 + 后置任务路径。 */
    int preview_total_cost;
    int preview_bomb_path_cost;
    int preview_remaining_cost;
    int search_truncated;
} MultiBombStepResult;

/*
 * 通用多炸弹搜索入口。按 1..max_depth 递增搜索，因此优先炸弹数量更少；
 * 同一深度内比较 evaluator 给出的完整总距离，只返回第一颗炸弹供实车执行。
 */
int multi_bomb_step_solver_solve_for_goal(const PlannerGrid *grid,
                                          int max_bomb_depth,
                                          MultiBombGoalEvaluator evaluator,
                                          void *context,
                                          MultiBombStepResult *out_result);

/*
 * 兼容原 Stage27 接口：目标是“爆破后普通推箱地图可解”。
 * 实车仍然只执行第一颗炸弹，然后重新获取地图。
 */
int multi_bomb_step_solver_solve_one_step(const PlannerGrid *grid,
                                          int max_bomb_depth,
                                          MultiBombStepResult *out_result);

#ifdef __cplusplus
}
#endif

#endif /* MULTI_BOMB_STEP_SOLVER_H */
