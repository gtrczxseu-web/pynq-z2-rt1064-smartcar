#include "solver_api.h"
#include "solver.h"
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

static void planner_result_clear(PlannerResult *res)
{
    if(res == 0) {
        return;
    }
    memset(res, 0, sizeof(*res));
    strncpy(res->mode, "fail", sizeof(res->mode) - 1);
    strncpy(res->reason, "unknown", sizeof(res->reason) - 1);
}

static void planner_result_fail(PlannerResult *res, const char *reason)
{
    planner_result_clear(res);
    if(res != 0) {
        res->success = 0;
        strncpy(res->mode, "fail", sizeof(res->mode) - 1);
        set_reason(res->reason, (int)sizeof(res->reason), reason);
    }
}

static void planner_result_success(PlannerResult *res, const char *path)
{
    int len;
    char temp_path[PLANNER_MAX_PATH_LEN + 1];

    if(path == 0) {
        path = "";
    }

    strncpy(temp_path, path, sizeof(temp_path) - 1);
    temp_path[sizeof(temp_path) - 1] = '\0';

    planner_result_clear(res);
    if(res == 0) {
        return;
    }
    res->success = 1;
    strncpy(res->mode, "normal", sizeof(res->mode) - 1);
    strncpy(res->reason, "ok", sizeof(res->reason) - 1);
    strncpy(res->path, temp_path, sizeof(res->path) - 1);
    res->path[sizeof(res->path) - 1] = '\0';
    len = (int)strlen(res->path);
    res->path_len = len;
}

static int valid_cell_symbol(char ch)
{
    if(ch == ' ') {
        return 1;
    }
    if(ch == '#' || ch == '-' || ch == '@' || ch == '$' || ch == '.' || ch == '*' || ch == 'X') {
        return 1;
    }
    return 0;
}

int solver_api_count_symbol(const PlannerGrid *grid, char symbol)
{
    int r, c;
    int count = 0;
    if(grid == 0) {
        return 0;
    }
    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            if(grid->grid[r][c] == symbol) {
                count++;
            }
        }
    }
    return count;
}

int solver_api_has_symbol(const PlannerGrid *grid, char symbol)
{
    return solver_api_count_symbol(grid, symbol) > 0;
}

int solver_api_find_player(const PlannerGrid *grid, PlannerPoint *out_player)
{
    int r, c;
    int count = 0;
    PlannerPoint p;

    if(grid == 0 || out_player == 0) {
        return 0;
    }

    out_player->r = -1;
    out_player->c = -1;

    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            if(grid->grid[r][c] == '@') {
                p.r = (int16_t)r;
                p.c = (int16_t)c;
                *out_player = p;
                count++;
            }
        }
    }

    return count == 1;
}

int solver_api_normalize_grid(const PlannerGrid *grid, PlannerGrid *out_grid, char *reason, int reason_size)
{
    int r, c;
    int players;

    if(grid == 0 || out_grid == 0) {
        set_reason(reason, reason_size, "grid_is_null");
        return 0;
    }

    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            char ch = grid->grid[r][c];
            if(!valid_cell_symbol(ch)) {
                set_reason(reason, reason_size, "invalid_cell_symbol");
                return 0;
            }
            if(ch == ' ') {
                ch = '-';
            }
            /* X is treated as a wall in the C planner. */
            if(ch == 'X') {
                ch = '#';
            }
            out_grid->grid[r][c] = ch;
        }
    }

    players = solver_api_count_symbol(out_grid, '@');
    if(players == 0) {
        set_reason(reason, reason_size, "no_player");
        return 0;
    }
    if(players != 1) {
        set_reason(reason, reason_size, "invalid_player_count");
        return 0;
    }

    set_reason(reason, reason_size, "ok");
    return 1;
}

int solver_api_solve_map(const PlannerGrid *grid, PlannerResult *out_result)
{
    PlannerGrid norm;
    char reason[48];
    char solved_path[PLANNER_MAX_PATH_LEN + 1];
    int boxes;
    int goals;
    int ok;

    planner_result_clear(out_result);
    solved_path[0] = '\0';

    if(!solver_api_normalize_grid(grid, &norm, reason, (int)sizeof(reason))) {
        planner_result_fail(out_result, reason);
        return 0;
    }

    boxes = solver_api_count_symbol(&norm, '$');
    goals = solver_api_count_symbol(&norm, '.');

    if(boxes == 0) {
        planner_result_fail(out_result, "no_box");
        return 0;
    }
    if(goals == 0) {
        planner_result_fail(out_result, "no_goal");
        return 0;
    }
    if(boxes != goals) {
        planner_result_fail(out_result, "box_goal_count_mismatch");
        return 0;
    }

    ok = solver_solve_subject1_best_path(&norm, solved_path, (int)sizeof(solved_path));
    if(!ok || solved_path[0] == '\0') {
        planner_result_fail(out_result, "normal_no_solution");
        return 0;
    }

    planner_result_success(out_result, solved_path);
    return 1;
}
