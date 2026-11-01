#include "bomb_push_planner.h"
#include "bomb_rule_engine.h"
#include "planner_bfs_workspace.h"
#include <string.h>

/* Direction order matches solver.c / Python solver: U, D, L, R. */
static const char DIR_CHARS[4] = {'U', 'D', 'L', 'R'};
static const int8_t DIR_DR[4] = {-1, 1, 0, 0};
static const int8_t DIR_DC[4] = {0, 0, -1, 1};

/* BFS workspace is shared with solver.c to fit MCU RAM. */

static int in_bounds(int r, int c)
{
    return r >= 0 && r < PLANNER_ROWS && c >= 0 && c < PLANNER_COLS;
}

static PlannerPoint make_point(int r, int c)
{
    PlannerPoint p;
    p.r = (int16_t)r;
    p.c = (int16_t)c;
    return p;
}

static int state_index(int pr, int pc, int br, int bc)
{
    int player_cell = pr * PLANNER_COLS + pc;
    int bomb_cell = br * PLANNER_COLS + bc;
    return player_cell * PLANNER_CELL_COUNT + bomb_cell;
}

static void decode_state_index(uint16_t idx, int *pr, int *pc, int *br, int *bc)
{
    int player_cell = idx / PLANNER_CELL_COUNT;
    int bomb_cell = idx % PLANNER_CELL_COUNT;

    *pr = player_cell / PLANNER_COLS;
    *pc = player_cell % PLANNER_COLS;
    *br = bomb_cell / PLANNER_COLS;
    *bc = bomb_cell % PLANNER_COLS;
}

static void push_result_clear(BombPushResult *res)
{
    if(res == 0) {
        return;
    }
    memset(res, 0, sizeof(*res));
    res->final_player.r = -1;
    res->final_player.c = -1;
    res->final_bomb.r = -1;
    res->final_bomb.c = -1;
}

static void push_result_set_reason(BombPushResult *res, const char *reason)
{
    if(res == 0 || reason == 0) {
        return;
    }
    strncpy(res->reason, reason, sizeof(res->reason) - 1);
}

static int find_player(const PlannerGrid *grid, PlannerPoint *player)
{
    int r, c;
    if(grid == 0 || player == 0) {
        return 0;
    }
    player->r = -1;
    player->c = -1;
    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            if(grid->grid[r][c] == '@') {
                player->r = (int16_t)r;
                player->c = (int16_t)c;
                return 1;
            }
        }
    }
    return 0;
}

static int is_current_bomb_cell(int r, int c, int br, int bc)
{
    return r == br && c == bc;
}

static int is_own_bomb_original_cell(int r, int c, PlannerPoint original_bomb)
{
    return r == original_bomb.r && c == original_bomb.c;
}

static int is_other_bomb_cell(const PlannerGrid *grid,
                              int r, int c,
                              PlannerPoint original_bomb,
                              int cur_br, int cur_bc)
{
    if(!in_bounds(r, c)) {
        return 0;
    }
    if(grid->grid[r][c] != '*') {
        return 0;
    }
    if(is_current_bomb_cell(r, c, cur_br, cur_bc)) {
        return 0;
    }
    if(is_own_bomb_original_cell(r, c, original_bomb)) {
        return 0;
    }
    return 1;
}

static int player_can_stand_on(const PlannerGrid *grid,
                               int r, int c,
                               PlannerPoint original_bomb,
                               int cur_br, int cur_bc)
{
    char ch;
    if(!in_bounds(r, c)) {
        return 0;
    }
    if(is_current_bomb_cell(r, c, cur_br, cur_bc)) {
        return 0;
    }
    if(is_other_bomb_cell(grid, r, c, original_bomb, cur_br, cur_bc)) {
        return 0;
    }

    ch = grid->grid[r][c];
    if(is_own_bomb_original_cell(r, c, original_bomb) && !is_current_bomb_cell(r, c, cur_br, cur_bc)) {
        return 1;
    }
    return ch == '-' || ch == '.' || ch == '@';
}

static int bomb_can_move_to_normal_cell(const PlannerGrid *grid,
                                        int r, int c,
                                        PlannerPoint original_bomb,
                                        int cur_br, int cur_bc)
{
    char ch;
    if(!in_bounds(r, c)) {
        return 0;
    }
    if(is_current_bomb_cell(r, c, cur_br, cur_bc)) {
        return 0;
    }
    if(is_other_bomb_cell(grid, r, c, original_bomb, cur_br, cur_bc)) {
        return 0;
    }

    ch = grid->grid[r][c];
    if(is_own_bomb_original_cell(r, c, original_bomb) && !is_current_bomb_cell(r, c, cur_br, cur_bc)) {
        return 1;
    }
    return ch == '-' || ch == '.' || ch == '@';
}

static int reconstruct_path(int end_idx, char *out_path, int out_path_size)
{
    char rev_path[PLANNER_MAX_PATH_LEN];
    int len = 0;
    int idx = end_idx;
    int i;

    if(out_path == 0 || out_path_size <= 0) {
        return 0;
    }
    out_path[0] = '\0';

    while(planner_bfs_parent[idx] != PLANNER_BFS_PARENT_NONE) {
        if(len >= PLANNER_MAX_PATH_LEN - 1) {
            return 0;
        }
        rev_path[len++] = planner_bfs_parent_action[idx];
        idx = planner_bfs_parent[idx];
    }

    if(len >= out_path_size) {
        return 0;
    }

    for(i = 0; i < len; i++) {
        out_path[i] = rev_path[len - 1 - i];
    }
    out_path[len] = '\0';
    return 1;
}

int bomb_push_planner_solve(const PlannerGrid *grid,
                            const BombTask *task,
                            BombPushResult *out_result)
{
    PlannerPoint player;
    int head = 0;
    int tail = 0;
    int start_idx;

    push_result_clear(out_result);

    if(grid == 0 || task == 0 || out_result == 0) {
        return 0;
    }

    if(!bomb_rule_is_valid_target_wall(grid, task->target_wall)) {
        push_result_set_reason(out_result, "invalid_target_wall");
        return 0;
    }

    if(!planner_point_valid(task->bomb) || grid->grid[task->bomb.r][task->bomb.c] != '*') {
        push_result_set_reason(out_result, "invalid_bomb");
        return 0;
    }

    if(!find_player(grid, &player)) {
        push_result_set_reason(out_result, "missing_player");
        return 0;
    }

    planner_bfs_workspace_clear();

    start_idx = state_index(player.r, player.c, task->bomb.r, task->bomb.c);
    planner_bfs_queue[tail] = (uint16_t)start_idx;
    planner_bfs_visited[start_idx] = 1;
    planner_bfs_parent[start_idx] = PLANNER_BFS_PARENT_NONE;
    tail++;

    while(head < tail) {
        uint16_t cur_idx_u16 = planner_bfs_queue[head++];
        int cur_idx = (int)cur_idx_u16;
        int cur_pr, cur_pc, cur_br, cur_bc;
        int d;

        decode_state_index(cur_idx_u16, &cur_pr, &cur_pc, &cur_br, &cur_bc);

        if(cur_br == task->target_wall.r && cur_bc == task->target_wall.c) {
            if(!reconstruct_path(cur_idx, out_result->path, sizeof(out_result->path))) {
                push_result_set_reason(out_result, "path_too_long");
                return 0;
            }
            out_result->success = 1;
            out_result->path_len = (int)strlen(out_result->path);
            out_result->final_player = make_point(cur_pr, cur_pc);
            out_result->final_bomb = make_point(cur_br, cur_bc);
            push_result_set_reason(out_result, "ok");
            return 1;
        }

        for(d = 0; d < 4; d++) {
            int npr = cur_pr + DIR_DR[d];
            int npc = cur_pc + DIR_DC[d];
            int nbr = cur_br;
            int nbc = cur_bc;
            int new_idx;

            if(!in_bounds(npr, npc)) {
                continue;
            }

            if(npr == cur_br && npc == cur_bc) {
                nbr = cur_br + DIR_DR[d];
                nbc = cur_bc + DIR_DC[d];

                if(nbr == task->target_wall.r && nbc == task->target_wall.c) {
                    /* Final push: bomb may enter the selected internal wall. */
                } else {
                    if(!bomb_can_move_to_normal_cell(grid, nbr, nbc,
                                                     task->bomb, cur_br, cur_bc)) {
                        continue;
                    }
                }
            } else {
                if(!player_can_stand_on(grid, npr, npc, task->bomb, cur_br, cur_bc)) {
                    continue;
                }
            }

            new_idx = state_index(npr, npc, nbr, nbc);
            if(!planner_bfs_visited[new_idx]) {
                if(tail >= PLANNER_SINGLE_STATE_COUNT) {
                    push_result_set_reason(out_result, "queue_full");
                    return 0;
                }
                planner_bfs_visited[new_idx] = 1;
                planner_bfs_parent[new_idx] = (uint16_t)cur_idx;
                planner_bfs_parent_action[new_idx] = DIR_CHARS[d];
                planner_bfs_queue[tail] = (uint16_t)new_idx;
                tail++;
            }
        }
    }

    push_result_set_reason(out_result, "no_bomb_push_path");
    return 0;
}
