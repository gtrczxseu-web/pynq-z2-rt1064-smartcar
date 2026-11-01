#include "sokoban_simulator.h"
#include "solver_api.h"
#include <string.h>

static const char DIR_CHARS[4] = {'U', 'D', 'L', 'R'};
static const int8_t DIR_DR[4] = {-1, 1, 0, 0};
static const int8_t DIR_DC[4] = {0, 0, -1, 1};

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

static void sim_result_clear(SokobanSimResult *res)
{
    if(res == 0) {
        return;
    }
    memset(res, 0, sizeof(*res));
    res->player.r = -1;
    res->player.c = -1;
    res->consumed_goal.r = -1;
    res->consumed_goal.c = -1;
    set_reason(res->reason, (int)sizeof(res->reason), "unknown");
}

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

static int direction_index(char action)
{
    int i;
    for(i = 0; i < 4; i++) {
        if(DIR_CHARS[i] == action) {
            return i;
        }
    }
    return -1;
}

static int is_goal_point(const PlannerPoint goals[], int goal_count, PlannerPoint p)
{
    int i;
    for(i = 0; i < goal_count; i++) {
        if(planner_point_equal(goals[i], p)) {
            return 1;
        }
    }
    return 0;
}

static int remove_goal_point(PlannerPoint goals[], int *goal_count, PlannerPoint p)
{
    int i;
    if(goals == 0 || goal_count == 0) {
        return 0;
    }
    for(i = 0; i < *goal_count; i++) {
        if(planner_point_equal(goals[i], p)) {
            int j;
            for(j = i; j < *goal_count - 1; j++) {
                goals[j] = goals[j + 1];
            }
            (*goal_count)--;
            return 1;
        }
    }
    return 0;
}

static int collect_goals(const PlannerGrid *grid, PlannerPoint goals[], int *goal_count)
{
    int r, c;
    if(grid == 0 || goals == 0 || goal_count == 0) {
        return 0;
    }
    *goal_count = 0;
    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            if(grid->grid[r][c] == '.') {
                if(*goal_count >= PLANNER_MAX_TARGETS) {
                    return 0;
                }
                goals[*goal_count] = make_point(r, c);
                (*goal_count)++;
            }
        }
    }
    return 1;
}

static void restore_or_clear_player_cell(PlannerGrid *grid,
                                         PlannerPoint player,
                                         const PlannerPoint goals[],
                                         int goal_count)
{
    if(is_goal_point(goals, goal_count, player)) {
        grid->grid[player.r][player.c] = '.';
    } else {
        grid->grid[player.r][player.c] = '-';
    }
}

static void copy_path_prefix(char *dst, int dst_size, const char *path, int count)
{
    int i;
    if(dst == 0 || dst_size <= 0) {
        return;
    }
    if(path == 0 || count <= 0) {
        dst[0] = '\0';
        return;
    }
    if(count >= dst_size) {
        count = dst_size - 1;
    }
    for(i = 0; i < count; i++) {
        dst[i] = path[i];
    }
    dst[count] = '\0';
}

static void copy_path_suffix(char *dst, int dst_size, const char *path, int start)
{
    int len;
    if(dst == 0 || dst_size <= 0) {
        return;
    }
    if(path == 0) {
        dst[0] = '\0';
        return;
    }
    len = (int)strlen(path);
    if(start < 0) {
        start = 0;
    }
    if(start >= len) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, path + start, (unsigned int)(dst_size - 1));
    dst[dst_size - 1] = '\0';
}

static void fill_counts(SokobanSimResult *res)
{
    if(res == 0) {
        return;
    }
    res->boxes_left = solver_api_count_symbol(&res->grid, '$');
    res->goals_left = solver_api_count_symbol(&res->grid, '.');
}

int sokoban_simulate_until_first_consumption(const PlannerGrid *grid,
                                             const char *path,
                                             SokobanSimResult *out_result)
{
    PlannerGrid state;
    PlannerPoint player;
    PlannerPoint goals[PLANNER_MAX_TARGETS];
    int goal_count = 0;
    int index;

    sim_result_clear(out_result);

    if(grid == 0 || path == 0 || out_result == 0) {
        if(out_result != 0) {
            set_reason(out_result->reason, (int)sizeof(out_result->reason), "null_argument");
        }
        return 0;
    }

    memcpy(&state, grid, sizeof(state));

    if(!solver_api_find_player(&state, &player)) {
        memcpy(&out_result->grid, &state, sizeof(state));
        set_reason(out_result->reason, (int)sizeof(out_result->reason), "invalid_player_count");
        return 0;
    }

    if(!collect_goals(&state, goals, &goal_count)) {
        memcpy(&out_result->grid, &state, sizeof(state));
        set_reason(out_result->reason, (int)sizeof(out_result->reason), "too_many_goals");
        return 0;
    }

    for(index = 0; path[index] != '\0'; index++) {
        int d = direction_index(path[index]);
        int nr, nc;
        char next_cell;

        if(d < 0) {
            memcpy(&out_result->grid, &state, sizeof(state));
            out_result->player = player;
            copy_path_prefix(out_result->executed_path, sizeof(out_result->executed_path), path, index);
            copy_path_suffix(out_result->remaining_path, sizeof(out_result->remaining_path), path, index);
            set_reason(out_result->reason, (int)sizeof(out_result->reason), "invalid_action");
            fill_counts(out_result);
            return 0;
        }

        nr = player.r + DIR_DR[d];
        nc = player.c + DIR_DC[d];
        if(!in_bounds(nr, nc)) {
            memcpy(&out_result->grid, &state, sizeof(state));
            out_result->player = player;
            copy_path_prefix(out_result->executed_path, sizeof(out_result->executed_path), path, index);
            copy_path_suffix(out_result->remaining_path, sizeof(out_result->remaining_path), path, index);
            set_reason(out_result->reason, (int)sizeof(out_result->reason), "player_out_of_bounds");
            fill_counts(out_result);
            return 0;
        }

        next_cell = state.grid[nr][nc];

        if(next_cell == '$') {
            int br = nr + DIR_DR[d];
            int bc = nc + DIR_DC[d];
            char box_front;

            if(!in_bounds(br, bc)) {
                memcpy(&out_result->grid, &state, sizeof(state));
                out_result->player = player;
                copy_path_prefix(out_result->executed_path, sizeof(out_result->executed_path), path, index);
                copy_path_suffix(out_result->remaining_path, sizeof(out_result->remaining_path), path, index);
                set_reason(out_result->reason, (int)sizeof(out_result->reason), "box_out_of_bounds");
                fill_counts(out_result);
                return 0;
            }

            box_front = state.grid[br][bc];
            if(!(box_front == '-' || box_front == '.')) {
                memcpy(&out_result->grid, &state, sizeof(state));
                out_result->player = player;
                copy_path_prefix(out_result->executed_path, sizeof(out_result->executed_path), path, index);
                copy_path_suffix(out_result->remaining_path, sizeof(out_result->remaining_path), path, index);
                set_reason(out_result->reason, (int)sizeof(out_result->reason), "box_front_blocked");
                fill_counts(out_result);
                return 0;
            }

            restore_or_clear_player_cell(&state, player, goals, goal_count);
            player = make_point(nr, nc);
            state.grid[nr][nc] = '@';

            if(is_goal_point(goals, goal_count, make_point(br, bc))) {
                remove_goal_point(goals, &goal_count, make_point(br, bc));
                state.grid[br][bc] = '-';
                memcpy(&out_result->grid, &state, sizeof(state));
                out_result->success = 1;
                out_result->consumed = 1;
                out_result->player = player;
                out_result->consumed_goal = make_point(br, bc);
                copy_path_prefix(out_result->executed_path, sizeof(out_result->executed_path), path, index + 1);
                copy_path_suffix(out_result->remaining_path, sizeof(out_result->remaining_path), path, index + 1);
                set_reason(out_result->reason, (int)sizeof(out_result->reason), "consumed");
                fill_counts(out_result);
                return 1;
            }

            state.grid[br][bc] = '$';
        } else if(next_cell == '-' || next_cell == '.') {
            restore_or_clear_player_cell(&state, player, goals, goal_count);
            player = make_point(nr, nc);
            state.grid[nr][nc] = '@';
        } else {
            memcpy(&out_result->grid, &state, sizeof(state));
            out_result->player = player;
            copy_path_prefix(out_result->executed_path, sizeof(out_result->executed_path), path, index);
            copy_path_suffix(out_result->remaining_path, sizeof(out_result->remaining_path), path, index);
            set_reason(out_result->reason, (int)sizeof(out_result->reason), "player_front_blocked");
            fill_counts(out_result);
            return 0;
        }
    }

    memcpy(&out_result->grid, &state, sizeof(state));
    out_result->success = 1;
    out_result->consumed = 0;
    out_result->player = player;
    copy_path_prefix(out_result->executed_path, sizeof(out_result->executed_path), path, (int)strlen(path));
    out_result->remaining_path[0] = '\0';
    set_reason(out_result->reason, (int)sizeof(out_result->reason), "path_finished_without_consumption");
    fill_counts(out_result);
    return 1;
}

int sokoban_simulate_consuming_solution(const PlannerGrid *grid,
                                        const char *path,
                                        SokobanSimResult *out_result)
{
    PlannerGrid state;
    char remaining[PLANNER_MAX_PATH_LEN];
    char executed_all[PLANNER_MAX_PATH_LEN];
    int rounds = 0;
    int consumed_count = 0;

    sim_result_clear(out_result);
    if(grid == 0 || path == 0 || out_result == 0) {
        if(out_result != 0) {
            set_reason(out_result->reason, (int)sizeof(out_result->reason), "null_argument");
        }
        return 0;
    }

    memcpy(&state, grid, sizeof(state));
    strncpy(remaining, path, sizeof(remaining) - 1);
    remaining[sizeof(remaining) - 1] = '\0';
    executed_all[0] = '\0';

    while(remaining[0] != '\0') {
        SokobanSimResult step;
        int old_remaining_len;
        int new_len;
        rounds++;
        if(rounds > PLANNER_MAX_PATH_LEN) {
            memcpy(&out_result->grid, &state, sizeof(state));
            set_reason(out_result->reason, (int)sizeof(out_result->reason), "simulation_round_limit");
            return 0;
        }

        old_remaining_len = (int)strlen(remaining);
        if(!sokoban_simulate_until_first_consumption(&state, remaining, &step)) {
            *out_result = step;
            return 0;
        }

        memcpy(&state, &step.grid, sizeof(state));
        if(strlen(executed_all) + strlen(step.executed_path) < sizeof(executed_all)) {
            strcat(executed_all, step.executed_path);
        }
        if(step.consumed) {
            consumed_count++;
            strncpy(remaining, step.remaining_path, sizeof(remaining) - 1);
            remaining[sizeof(remaining) - 1] = '\0';
        } else {
            remaining[0] = '\0';
        }
        new_len = (int)strlen(remaining);
        if(new_len > 0 && new_len >= old_remaining_len) {
            memcpy(&out_result->grid, &state, sizeof(state));
            set_reason(out_result->reason, (int)sizeof(out_result->reason), "simulation_made_no_progress");
            return 0;
        }
    }

    memcpy(&out_result->grid, &state, sizeof(state));
    out_result->success = (solver_api_count_symbol(&state, '$') == 0 && solver_api_count_symbol(&state, '.') == 0);
    out_result->consumed = consumed_count > 0;
    out_result->boxes_left = solver_api_count_symbol(&state, '$');
    out_result->goals_left = solver_api_count_symbol(&state, '.');
    if(solver_api_find_player(&state, &out_result->player)) {
        ;
    }
    strncpy(out_result->executed_path, executed_all, sizeof(out_result->executed_path) - 1);
    out_result->executed_path[sizeof(out_result->executed_path) - 1] = '\0';
    out_result->remaining_path[0] = '\0';
    if(out_result->success) {
        set_reason(out_result->reason, (int)sizeof(out_result->reason), "ok");
        return 1;
    }
    set_reason(out_result->reason, (int)sizeof(out_result->reason), "path_finished_with_remaining_objects");
    return 0;
}
