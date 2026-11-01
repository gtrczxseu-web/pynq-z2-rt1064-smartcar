#include "solver.h"
#include "planner_bfs_workspace.h"
#include <string.h>
/* Direction order intentionally matches Python dict insertion order: U, D, L, R. */
static const char DIR_CHARS[4] = {'U', 'D', 'L', 'R'};
static const int8_t DIR_DR[4] = {-1, 1, 0, 0};
static const int8_t DIR_DC[4] = {0, 0, -1, 1};

/* BFS workspace is shared with bomb_push_planner.c to fit MCU RAM. */

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
    int box_cell = br * PLANNER_COLS + bc;
    return player_cell * PLANNER_CELL_COUNT + box_cell;
}

static void decode_state_index(uint16_t idx, int *pr, int *pc, int *br, int *bc)
{
    int player_cell = idx / PLANNER_CELL_COUNT;
    int box_cell = idx % PLANNER_CELL_COUNT;

    *pr = player_cell / PLANNER_COLS;
    *pc = player_cell % PLANNER_COLS;
    *br = box_cell / PLANNER_COLS;
    *bc = box_cell % PLANNER_COLS;
}

static int is_static_free(const PlannerGrid *grid, int r, int c)
{
    char ch;
    if(!in_bounds(r, c)) {
        return 0;
    }
    ch = grid->grid[r][c];
    return ch == '-' || ch == '.' || ch == '@';
}

static int is_target_cell(const PlannerPoint targets[], int target_count, PlannerPoint p)
{
    int i;
    for(i = 0; i < target_count; i++) {
        if(planner_point_equal(targets[i], p)) {
            return 1;
        }
    }
    return 0;
}


static void clear_cell(PlannerGrid *grid, PlannerPoint pos,
                       const PlannerPoint targets[], int target_count)
{
    if(is_target_cell(targets, target_count, pos)) {
        grid->grid[pos.r][pos.c] = '.';
    } else {
        grid->grid[pos.r][pos.c] = '-';
    }
}

static void place_player(PlannerGrid *grid, PlannerPoint pos)
{
    grid->grid[pos.r][pos.c] = '@';
}

static void place_box(PlannerGrid *grid, PlannerPoint pos)
{
    grid->grid[pos.r][pos.c] = '$';
}

void solver_clone_grid(const PlannerGrid *src, PlannerGrid *dst)
{
    memcpy(dst->grid, src->grid, sizeof(dst->grid));
}

int solver_extract_positions(const PlannerGrid *grid,
                             PlannerPoint *player,
                             PlannerPoint boxes[], int *box_count,
                             PlannerPoint targets[], int *target_count)
{
    int r, c;
    int has_player = 0;

    *box_count = 0;
    *target_count = 0;
    player->r = -1;
    player->c = -1;

    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            char ch = grid->grid[r][c];
            if(ch == '@') {
                *player = make_point(r, c);
                has_player = 1;
            } else if(ch == '$') {
                if(*box_count >= PLANNER_MAX_BOXES) {
                    return 0;
                }
                boxes[*box_count] = make_point(r, c);
                (*box_count)++;
            } else if(ch == '.') {
                if(*target_count >= PLANNER_MAX_TARGETS) {
                    return 0;
                }
                targets[*target_count] = make_point(r, c);
                (*target_count)++;
            }
        }
    }

    return has_player;
}

static void collect_targets(const PlannerGrid *grid, PlannerPoint targets[], int *target_count)
{
    int r, c;
    *target_count = 0;
    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            if(grid->grid[r][c] == '.') {
                if(*target_count < PLANNER_MAX_TARGETS) {
                    targets[*target_count] = make_point(r, c);
                    (*target_count)++;
                }
            }
        }
    }
}

static int find_single_elements(const PlannerGrid *grid,
                                PlannerPoint *player,
                                PlannerPoint *box,
                                PlannerPoint *target)
{
    int r, c;
    int has_player = 0, has_box = 0, has_target = 0;
    player->r = player->c = -1;
    box->r = box->c = -1;
    target->r = target->c = -1;

    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            char ch = grid->grid[r][c];
            if(ch == '@') {
                *player = make_point(r, c);
                has_player = 1;
            } else if(ch == '$') {
                *box = make_point(r, c);
                has_box = 1;
            } else if(ch == '.') {
                *target = make_point(r, c);
                has_target = 1;
            }
        }
    }

    return has_player && has_box && has_target;
}

static int reconstruct_bfs_path(int end_index, char *out_path, int out_path_size)
{
    char rev_path[PLANNER_MAX_PATH_LEN];
    int len = 0;
    int idx = end_index;
    int i;

    if(out_path_size <= 0) {
        return 0;
    }

    while(planner_bfs_parent[idx] != PLANNER_BFS_PARENT_NONE) {
        if(len >= PLANNER_MAX_PATH_LEN - 1) {
            out_path[0] = '\0';
            return 0;
        }
        rev_path[len++] = planner_bfs_parent_action[idx];
        idx = planner_bfs_parent[idx];
    }

    if(len >= out_path_size) {
        out_path[0] = '\0';
        return 0;
    }

    for(i = 0; i < len; i++) {
        out_path[i] = rev_path[len - 1 - i];
    }
    out_path[len] = '\0';
    return 1;
}

static int solver_solve_single_box_bfs_ordered(
    const PlannerGrid *submap,
    const uint8_t dir_order[4],
    char *out_path,
    int out_path_size)

{
    PlannerPoint player, box, target;
    int head = 0;
    int tail = 0;
    int start_idx;

    if(out_path_size <= 0) {
        return 0;
    }
    out_path[0] = '\0';

    if(!find_single_elements(submap, &player, &box, &target)) {
        return 0;
    }

    planner_bfs_workspace_clear();

    start_idx = state_index(player.r, player.c, box.r, box.c);
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

        /* Python solve_single_box_bfs returns immediately when the box reaches target.
         * It does not continue searching for fewer compressed commands.
         */
        if(cur_br == target.r && cur_bc == target.c) {
            return reconstruct_bfs_path(cur_idx, out_path, out_path_size);
        }

        /* Direction order matches Python DIRS insertion order: U, D, L, R. */
        int k;

        for(k = 0; k < 4; k++){
           d = dir_order[k];
            int npr = cur_pr + DIR_DR[d];
            int npc = cur_pc + DIR_DC[d];
            int nbr = cur_br;
            int nbc = cur_bc;
            int new_idx;

            if(npr == cur_br && npc == cur_bc) {
                nbr = cur_br + DIR_DR[d];
                nbc = cur_bc + DIR_DC[d];
                if(!is_static_free(submap, nbr, nbc)) {
                    continue;
                }
            } else {
                if(!is_static_free(submap, npr, npc)) {
                    continue;
                }
            }

            new_idx = state_index(npr, npc, nbr, nbc);
            if(!planner_bfs_visited[new_idx]) {
                if(tail >= PLANNER_SINGLE_STATE_COUNT) {
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

    return 0;
}

int solver_solve_single_box_bfs(const PlannerGrid *submap,
                                char *out_path,
                                int out_path_size)
{
    static const uint8_t default_order[4] = {
        0, 1, 2, 3
    };

    return solver_solve_single_box_bfs_ordered(
        submap,
        default_order,
        out_path,
        out_path_size);
}

static int build_submap_from_grid(const PlannerGrid *current_grid,
                                  PlannerPoint active_box,
                                  PlannerPoint active_target,
                                  PlannerGrid *submap)
{
    PlannerPoint player;
    PlannerPoint boxes[PLANNER_MAX_BOXES];
    PlannerPoint targets[PLANNER_MAX_TARGETS];
    int box_count = 0;
    int target_count = 0;
    int r, c, i;

    solver_clone_grid(current_grid, submap);

    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            char ch = submap->grid[r][c];
            if(ch == '$' || ch == '.') {
                submap->grid[r][c] = '-';
            }
        }
    }

    submap->grid[active_target.r][active_target.c] = '.';

    if(!solver_extract_positions(current_grid, &player, boxes, &box_count, targets, &target_count)) {
        return 0;
    }

    for(i = 0; i < box_count; i++) {
        PlannerPoint b = boxes[i];
        if(planner_point_equal(b, active_box)) {
            submap->grid[b.r][b.c] = '$';
        } else {
            submap->grid[b.r][b.c] = '#';
        }
    }

    return 1;
}

int solver_simulate_path(const PlannerGrid *initial_grid,
                         const char *path,
                         PlannerPoint player,
                         PlannerPoint box,
                         PlannerPoint target,
                         PlannerGrid *out_grid,
                         PlannerPoint *out_player)
{
    PlannerGrid grid;
    PlannerPoint all_targets[PLANNER_MAX_TARGETS];
    int target_count = 0;
    int step;

    solver_clone_grid(initial_grid, &grid);
    collect_targets(&grid, all_targets, &target_count);

    for(step = 0; path[step] != '\0'; step++) {
        int d = -1;
        int i;
        int npr, npc;

        for(i = 0; i < 4; i++) {
            if(path[step] == DIR_CHARS[i]) {
                d = i;
                break;
            }
        }
        if(d < 0) {
            return 0;
        }

        npr = player.r + DIR_DR[d];
        npc = player.c + DIR_DC[d];

        if(npr == box.r && npc == box.c) {
            PlannerPoint new_box = make_point(box.r + DIR_DR[d], box.c + DIR_DC[d]);
            if(!is_static_free(&grid, new_box.r, new_box.c)) {
                return 0;
            }
            clear_cell(&grid, player, all_targets, target_count);
            clear_cell(&grid, box, all_targets, target_count);
            player = box;
            box = new_box;
            place_player(&grid, player);
            place_box(&grid, box);
        } else {
            if(!is_static_free(&grid, npr, npc)) {
                return 0;
            }
            clear_cell(&grid, player, all_targets, target_count);
            player = make_point(npr, npc);
            place_player(&grid, player);
        }
    }

    if(!planner_point_equal(box, target)) {
        return 0;
    }

    grid.grid[target.r][target.c] = '-';
    place_player(&grid, player);

    if(out_grid != 0) {
        solver_clone_grid(&grid, out_grid);
    }
    if(out_player != 0) {
        *out_player = player;
    }
    return 1;
}

/* The old exhaustive (n!)^2 implementation is retained as disabled reference.
 * Subject 1 now uses the bounded Beam Search implementation below.
 */
#if 0
/* Recursive ordered selection generator, equivalent to Python ordered_selections(seq, count).
 * This callback-based form avoids storing all permutations at once.
 */
typedef int (*SelectionCallback)(const PlannerPoint order[], int count, void *user);

static int ordered_selections_recursive(const PlannerPoint seq[], int seq_count,
                                        int count,
                                        uint8_t used[],
                                        PlannerPoint current[],
                                        int depth,
                                        SelectionCallback cb,
                                        void *user)
{
    int i;
    if(depth == count) {
        return cb(current, count, user);
    }

    for(i = 0; i < seq_count; i++) {
        if(!used[i]) {
            used[i] = 1;
            current[depth] = seq[i];
            if(!ordered_selections_recursive(seq, seq_count, count, used, current, depth + 1, cb, user)) {
                return 0;
            }
            used[i] = 0;
        }
    }
    return 1;
}

static int ordered_selections(const PlannerPoint seq[], int seq_count,
                              int count,
                              SelectionCallback cb,
                              void *user)
{
    uint8_t used[PLANNER_MAX_OBJECTS];
    PlannerPoint current[PLANNER_MAX_OBJECTS];
    memset(used, 0, sizeof(used));
    if(count < 0 || count > seq_count || count > PLANNER_MAX_OBJECTS) {
        return 0;
    }
    return ordered_selections_recursive(seq, seq_count, count, used, current, 0, cb, user);
}

typedef struct
{
    int steps;          /* ????? */
    int segments;       /* ?????????? */
    int one_cell;       /* N1/E1/S1/W1 ???? */
    int short_segments; /* ???? <= 2 ??????????? */
} Subject1PathMetrics;

static void subject1_measure_path(const char *path,
                                  Subject1PathMetrics *metrics)
{
    int i;
    int run_len;
    char last_dir;

    memset(metrics, 0, sizeof(*metrics));

    if(path == NULL || path[0] == '\0')
    {
        return;
    }

    last_dir = path[0];
    run_len = 1;
    metrics->steps = 1;
    metrics->segments = 1;

    for(i = 1; path[i] != '\0'; i++)
    {
        metrics->steps++;

        if(path[i] == last_dir)
        {
            run_len++;
        }
        else
        {
            if(run_len == 1)
            {
                metrics->one_cell++;
            }
            if(run_len <= 2)
            {
                metrics->short_segments++;
            }

            last_dir = path[i];
            run_len = 1;
            metrics->segments++;
        }
    }

    if(run_len == 1)
    {
        metrics->one_cell++;
    }
    if(run_len <= 2)
    {
        metrics->short_segments++;
    }
}

static int subject1_metrics_better(
    const Subject1PathMetrics *candidate,
    const Subject1PathMetrics *best)
{
    /*
     * ???????????¦È????
     * ???????????????¦²??????????
     */
    if(candidate->segments != best->segments)
    {
        return candidate->segments < best->segments;
    }

    if(candidate->one_cell != best->one_cell)
    {
        return candidate->one_cell < best->one_cell;
    }

    if(candidate->short_segments != best->short_segments)
    {
        return candidate->short_segments < best->short_segments;
    }

    return candidate->steps < best->steps;
}

typedef struct {
    PlannerGrid original_grid;
    PlannerPoint target_order[PLANNER_MAX_OBJECTS];
    int pair_count;
    char *best_path;
    int best_path_size;
    Subject1PathMetrics best_metrics;
    int has_best;
} TargetLoopContext;

typedef struct {
    TargetLoopContext *ctx;
    PlannerPoint box_order[PLANNER_MAX_OBJECTS];
} BoxTargetContext;

static int try_assignment(const PlannerPoint box_order[],
                          const PlannerPoint target_order[],
                          int pair_count,
                          const PlannerGrid *original_grid,
                          char *out_total,
                          int out_total_size)
{
    PlannerGrid current_grid;
    int total_len = 0;
    int i;

    if(out_total_size <= 0) {
        return 0;
    }
    out_total[0] = '\0';
    solver_clone_grid(original_grid, &current_grid);

    for(i = 0; i < pair_count; i++) {
        PlannerPoint active_box = box_order[i];
        PlannerPoint active_target = target_order[i];
        PlannerGrid submap;
        static char sub_path[PLANNER_MAX_PATH_LEN];
        PlannerPoint current_player;
        PlannerPoint boxes[PLANNER_MAX_BOXES];
        PlannerPoint targets[PLANNER_MAX_TARGETS];
        int box_count = 0;
        int target_count = 0;
        PlannerGrid new_grid;

        if(!build_submap_from_grid(&current_grid, active_box, active_target, &submap)) {
            return 0;
        }

        if(!solver_solve_single_box_bfs(&submap, sub_path, sizeof(sub_path))) {
            return 0;
        }

        if(!solver_extract_positions(&current_grid, &current_player, boxes, &box_count, targets, &target_count)) {
            return 0;
        }

        if(!solver_simulate_path(&current_grid, sub_path, current_player,
                                 active_box, active_target, &new_grid, 0)) {
            return 0;
        }

        if(total_len + (int)strlen(sub_path) >= out_total_size) {
            return 0;
        }
        strcpy(out_total + total_len, sub_path);
        total_len += (int)strlen(sub_path);
        solver_clone_grid(&new_grid, &current_grid);
    }

    return 1;
}

static int target_order_callback(const PlannerPoint target_order[],
                                 int count,
                                 void *user)
{
    BoxTargetContext *btc = (BoxTargetContext *)user;
    TargetLoopContext *ctx = btc->ctx;

    /*
     * ??t?????ï…????????????????4096????
     * ????œý??????????????????????¨¢?
     */
    static char total_path[PLANNER_MAX_PATH_LEN];

    Subject1PathMetrics candidate_metrics;
    int path_len;

    if(!try_assignment(btc->box_order,
                       target_order,
                       count,
                       &ctx->original_grid,
                       total_path,
                       sizeof(total_path)))
    {
        return 1;
    }

    path_len = (int)strlen(total_path);
    subject1_measure_path(total_path, &candidate_metrics);

    if((!ctx->has_best) ||
       subject1_metrics_better(&candidate_metrics,
                               &ctx->best_metrics))
    {
        if(path_len >= ctx->best_path_size)
        {
            return 1;
        }

        strcpy(ctx->best_path, total_path);
        ctx->best_metrics = candidate_metrics;
        ctx->has_best = 1;
    }

    return 1;
}
typedef struct {
    TargetLoopContext *ctx;
    const PlannerPoint *targets;
    int target_count;
} BoxLoopContext;

static int box_order_callback(const PlannerPoint box_order[], int count, void *user)
{
    BoxLoopContext *blc = (BoxLoopContext *)user;
    BoxTargetContext btc;
    int i;
    btc.ctx = blc->ctx;
    for(i = 0; i < count; i++) {
        btc.box_order[i] = box_order[i];
    }
    return ordered_selections(blc->targets, blc->target_count, count,
                              target_order_callback, &btc);
}

int solver_solve_subject1_best_path(const PlannerGrid *state_grid,
                                    char *out_path,
                                    int out_path_size)
{
    PlannerPoint player;
    PlannerPoint boxes[PLANNER_MAX_BOXES];
    PlannerPoint targets[PLANNER_MAX_TARGETS];
    int box_count = 0;
    int target_count = 0;
    int pair_count;
    TargetLoopContext tlc;
    BoxLoopContext blc;

    if(out_path_size <= 0) {
        return 0;
    }
    out_path[0] = '\0';

    if(!solver_extract_positions(state_grid, &player, boxes, &box_count, targets, &target_count)) {
        return 0;
    }

    if(box_count == 0 || target_count == 0) {
        return 0;
    }

    pair_count = box_count < target_count ? box_count : target_count;
    if(pair_count > PLANNER_MAX_OBJECTS) {
        return 0;
    }

    solver_clone_grid(state_grid, &tlc.original_grid);
    tlc.pair_count = pair_count;
    tlc.best_path = out_path;
    tlc.best_path_size = out_path_size;
    memset(&tlc.best_metrics, 0, sizeof(tlc.best_metrics));
    tlc.has_best = 0;

    blc.ctx = &tlc;
    blc.targets = targets;
    blc.target_count = target_count;

    if(!ordered_selections(boxes, box_count, pair_count, box_order_callback, &blc)) {
        return 0;
    }

    if(!tlc.has_best) {
        out_path[0] = '\0';
        return 0;
    }

    return 1;
}
#endif

/*
 * Subject 1 bounded Beam Search.
 *
 * Each node is a different predicted map after completing a different
 * box/goal sequence.  Only the best SUBJECT1_BEAM_WIDTH nodes survive each
 * depth, so five or more boxes do not trigger factorial enumeration.
 */
typedef struct {
    int steps;
    int segments;
    int one_cell;
    int short_segments;
} Subject1BeamMetrics;

typedef struct {
    PlannerGrid grid;
    char path[SUBJECT1_BEAM_PATH_LEN];
    int path_len;
    int depth;
    int first_path_len;
    PlannerPoint first_box;
    PlannerPoint first_target;
    Subject1BeamMetrics metrics;
    uint8_t box_order[PLANNER_MAX_BOXES];
    uint8_t target_order[PLANNER_MAX_TARGETS];
    uint16_t action_end[PLANNER_MAX_BOXES];
    int remaining_estimate;
    int score;
} Subject1BeamNode;

/* Large work buffers must not live on the MCU stack. */
static Subject1BeamNode g_subject1_beam_a[SUBJECT1_BEAM_WIDTH];
static Subject1BeamNode g_subject1_beam_b[SUBJECT1_BEAM_WIDTH];
static Subject1BeamNode g_subject1_candidate;
static char g_subject1_sub_path[SUBJECT1_BEAM_PATH_LEN];
static PlannerGrid g_subject1_submap;
static PlannerGrid g_subject1_new_grid;
static char g_subject1_push_sequence[SUBJECT1_BEAM_PATH_LEN];
static char g_subject1_walk_path[SUBJECT1_BEAM_PATH_LEN];
static char g_subject1_optimized_path[SUBJECT1_BEAM_PATH_LEN];
static char g_subject1_first_path[SUBJECT1_BEAM_PATH_LEN];

/* Direction-aware free-walk Dijkstra workspace. */
static uint8_t g_subject1_walk_visited[PLANNER_ROWS][PLANNER_COLS][5];
static uint16_t g_subject1_walk_dist[PLANNER_ROWS][PLANNER_COLS][5];
static uint8_t g_subject1_walk_parent_cell[PLANNER_ROWS][PLANNER_COLS][5];
static int8_t g_subject1_walk_parent_dir[PLANNER_ROWS][PLANNER_COLS][5];
static char g_subject1_walk_parent_action[PLANNER_ROWS][PLANNER_COLS][5];

static void subject1_beam_measure_path(const char *path,
                                       Subject1BeamMetrics *metrics)
{
    int i;
    int run_len;
    char last_dir;

    memset(metrics, 0, sizeof(*metrics));
    if(path == 0 || path[0] == '\0') {
        return;
    }

    metrics->steps = 1;
    metrics->segments = 1;
    last_dir = path[0];
    run_len = 1;

    for(i = 1; path[i] != '\0'; i++) {
        metrics->steps++;
        if(path[i] == last_dir) {
            run_len++;
        } else {
            if(run_len == 1) {
                metrics->one_cell++;
            }
            if(run_len <= 2) {
                metrics->short_segments++;
            }
            last_dir = path[i];
            run_len = 1;
            metrics->segments++;
        }
    }

    if(run_len == 1) {
        metrics->one_cell++;
    }
    if(run_len <= 2) {
        metrics->short_segments++;
    }
}

static int subject1_beam_path_cost(const char *path)
{
    Subject1BeamMetrics metrics;
    subject1_beam_measure_path(path, &metrics);
    return metrics.steps * SUBJECT1_COST_STEP +
           metrics.segments * SUBJECT1_COST_SEGMENT +
           metrics.one_cell * SUBJECT1_COST_ONE_CELL +
           metrics.short_segments * SUBJECT1_COST_SHORT_SEGMENT;
}

static int subject1_walkable(const PlannerGrid *grid, int r, int c)
{
    char ch;
    if(grid == 0 || !in_bounds(r, c)) {
        return 0;
    }
    ch = grid->grid[r][c];
    return ch == '-' || ch == '.' || ch == '@';
}

/* Find a free-walk path that prices a direction change higher than one extra
 * grid step.  The active box remains an obstacle in the supplied submap.
 */
static int subject1_solve_free_path(const PlannerGrid *grid,
                                    int target_r,
                                    int target_c,
                                    int next_push_dir,
                                    char *out_path,
                                    int out_size)
{
    PlannerPoint player;
    PlannerPoint boxes[PLANNER_MAX_BOXES];
    PlannerPoint targets[PLANNER_MAX_TARGETS];
    int box_count = 0;
    int target_count = 0;
    const int DIR_NONE = 4;
    const int INF = 65535;
    int start_r;
    int start_c;
    int r;
    int c;
    int d;
    int i;
    int goal_dir = -1;
    int len = 0;

    if(grid == 0 || out_path == 0 || out_size <= 0) {
        return 0;
    }
    out_path[0] = '\0';

    if(!solver_extract_positions(grid, &player, boxes, &box_count,
                                 targets, &target_count)) {
        return 0;
    }
    if(box_count != 1 || target_count != 1 ||
       !subject1_walkable(grid, target_r, target_c)) {
        return 0;
    }

    start_r = player.r;
    start_c = player.c;
    if(start_r == target_r && start_c == target_c) {
        return 1;
    }

    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            for(d = 0; d < 5; d++) {
                g_subject1_walk_visited[r][c][d] = 0;
                g_subject1_walk_dist[r][c][d] = INF;
                g_subject1_walk_parent_cell[r][c][d] = 255;
                g_subject1_walk_parent_dir[r][c][d] = -1;
                g_subject1_walk_parent_action[r][c][d] = 0;
            }
        }
    }
    g_subject1_walk_dist[start_r][start_c][DIR_NONE] = 0;

    while(1) {
        int best_r = -1;
        int best_c = -1;
        int best_d = -1;
        int best_cost = INF;

        for(r = 0; r < PLANNER_ROWS; r++) {
            for(c = 0; c < PLANNER_COLS; c++) {
                for(d = 0; d < 5; d++) {
                    if(!g_subject1_walk_visited[r][c][d] &&
                       g_subject1_walk_dist[r][c][d] < best_cost) {
                        best_cost = g_subject1_walk_dist[r][c][d];
                        best_r = r;
                        best_c = c;
                        best_d = d;
                    }
                }
            }
        }

        if(best_r < 0) {
            break;
        }
        g_subject1_walk_visited[best_r][best_c][best_d] = 1;
       

        for(i = 0; i < 4; i++) {
            int nr = best_r + DIR_DR[i];
            int nc = best_c + DIR_DC[i];
            int new_cost;

            if(!subject1_walkable(grid, nr, nc)) {
                continue;
            }
            new_cost = best_cost + SUBJECT1_WALK_STEP_COST;
            if(best_d != DIR_NONE && best_d != i) {
                new_cost += SUBJECT1_WALK_TURN_COST;
            }
            if(new_cost < g_subject1_walk_dist[nr][nc][i]) {
                g_subject1_walk_dist[nr][nc][i] = new_cost;
                g_subject1_walk_parent_cell[nr][nc][i] =
                    (uint8_t)(best_r * PLANNER_COLS + best_c);
                g_subject1_walk_parent_dir[nr][nc][i] = (int8_t)best_d;
                g_subject1_walk_parent_action[nr][nc][i] = DIR_CHARS[i];
            }
        }
    }
{
    int best_goal_cost = INF;

    goal_dir = -1;

    for(d = 0; d < 4; d++)
    {
        int goal_cost = g_subject1_walk_dist[target_r][target_c][d];

        if(goal_cost >= INF)
        {
            continue;
        }

        /*
         * ?????¦Ë????????????????????
         * §³??????????¦±?
         */
        if(d != next_push_dir)
        {
            goal_cost += SUBJECT1_WALK_TURN_COST;
        }

        if(goal_cost < best_goal_cost)
        {
            best_goal_cost = goal_cost;
            goal_dir = d;
        }
    }
}
    if(goal_dir < 0) {
        return 0;
    }

    r = target_r;
    c = target_c;
    d = goal_dir;
    while(!(r == start_r && c == start_c && d == DIR_NONE)) {
        int pr;
        int pc;
        int pd;
        if(d < 0 || len >= SUBJECT1_BEAM_PATH_LEN - 1) {
            return 0;
        }
        g_subject1_sub_path[len++] =
            g_subject1_walk_parent_action[r][c][d];
        pr = g_subject1_walk_parent_cell[r][c][d] / PLANNER_COLS;
        pc = g_subject1_walk_parent_cell[r][c][d] % PLANNER_COLS;
        pd = g_subject1_walk_parent_dir[r][c][d];
        r = pr;
        c = pc;
        d = pd;
    }
    if(len >= out_size) {
        return 0;
    }
    for(i = 0; i < len; i++) {
        out_path[i] = g_subject1_sub_path[len - 1 - i];
    }
    out_path[len] = '\0';
    return 1;
}

/* Preserve the BFS push-direction sequence, but rebuild every free-walk gap
 * using the direction-aware planner above.  The input must be a one-box,
 * one-goal submap produced by build_submap_from_grid().
 */
static int subject1_optimize_walk_segments(const PlannerGrid *grid,
                                           char *path,
                                           int path_size)
{
    PlannerPoint player;
    PlannerPoint boxes[PLANNER_MAX_BOXES];
    PlannerPoint targets[PLANNER_MAX_TARGETS];
    PlannerGrid work;
    int box_count = 0;
    int target_count = 0;
    int sim_pr;
    int sim_pc;
    int sim_br;
    int sim_bc;
    int goal_r;
    int goal_c;
    int push_count = 0;
    int optimized_len = 0;
    int raw_len;
    int r;
    int c;
    int i;

    if(grid == 0 || path == 0 || path_size <= 0 || path[0] == '\0') {
        return 0;
    }
    if(!solver_extract_positions(grid, &player, boxes, &box_count,
                                 targets, &target_count) ||
       box_count != 1 || target_count != 1) {
        return 0;
    }

    sim_pr = player.r;
    sim_pc = player.c;
    sim_br = boxes[0].r;
    sim_bc = boxes[0].c;
    goal_r = targets[0].r;
    goal_c = targets[0].c;
    raw_len = (int)strlen(path);

    for(i = 0; i < raw_len; i++) {
        int dir = -1;
        int next_pr;
        int next_pc;
        int k;
        for(k = 0; k < 4; k++) {
            if(path[i] == DIR_CHARS[k]) {
                dir = k;
                break;
            }
        }
        if(dir < 0) {
            return 0;
        }
        next_pr = sim_pr + DIR_DR[dir];
        next_pc = sim_pc + DIR_DC[dir];
        if(next_pr == sim_br && next_pc == sim_bc) {
            if(push_count >= SUBJECT1_BEAM_PATH_LEN - 1) {
                return 0;
            }
            g_subject1_push_sequence[push_count++] = path[i];
            sim_br += DIR_DR[dir];
            sim_bc += DIR_DC[dir];
        }
        sim_pr = next_pr;
        sim_pc = next_pc;
    }
    g_subject1_push_sequence[push_count] = '\0';
    if(push_count <= 0 || sim_br != goal_r || sim_bc != goal_c) {
        return 0;
    }

    sim_pr = player.r;
    sim_pc = player.c;
    sim_br = boxes[0].r;
    sim_bc = boxes[0].c;
    g_subject1_optimized_path[0] = '\0';

    for(i = 0; i < push_count; i++) {
        int dir = -1;
        int stand_r;
        int stand_c;
        int walk_len;
        int k;
        for(k = 0; k < 4; k++) {
            if(g_subject1_push_sequence[i] == DIR_CHARS[k]) {
                dir = k;
                break;
            }
        }
        if(dir < 0) {
            return 0;
        }
        stand_r = sim_br - DIR_DR[dir];
        stand_c = sim_bc - DIR_DC[dir];

        solver_clone_grid(grid, &work);
        for(r = 0; r < PLANNER_ROWS; r++) {
            for(c = 0; c < PLANNER_COLS; c++) {
                if(work.grid[r][c] == '@' || work.grid[r][c] == '$') {
                    work.grid[r][c] = '-';
                }
            }
        }
        work.grid[goal_r][goal_c] = '.';
        work.grid[sim_br][sim_bc] = '$';
        work.grid[sim_pr][sim_pc] = '@';

        if(!subject1_solve_free_path(&work,
                             stand_r,
                             stand_c,
                             dir,
                             g_subject1_walk_path,
                             (int)sizeof(g_subject1_walk_path))) {
            return 0;
        }
        walk_len = (int)strlen(g_subject1_walk_path);
        if(optimized_len + walk_len + 1 >= path_size ||
           optimized_len + walk_len + 1 >= SUBJECT1_BEAM_PATH_LEN) {
            return 0;
        }
        memcpy(g_subject1_optimized_path + optimized_len,
               g_subject1_walk_path, (unsigned int)walk_len);
        optimized_len += walk_len;
        g_subject1_optimized_path[optimized_len++] =
            g_subject1_push_sequence[i];
        g_subject1_optimized_path[optimized_len] = '\0';

        for(k = 0; k < walk_len; k++) {
            int wd;
            for(wd = 0; wd < 4; wd++) {
                if(g_subject1_walk_path[k] == DIR_CHARS[wd]) {
                    sim_pr += DIR_DR[wd];
                    sim_pc += DIR_DC[wd];
                    break;
                }
            }
            if(wd >= 4) {
                return 0;
            }
        }
        if(sim_pr != stand_r || sim_pc != stand_c) {
            return 0;
        }
        sim_pr = sim_br;
        sim_pc = sim_bc;
        sim_br += DIR_DR[dir];
        sim_bc += DIR_DC[dir];
    }

    if(sim_br != goal_r || sim_bc != goal_c) {
        return 0;
    }
    if(subject1_beam_path_cost(g_subject1_optimized_path) <
       subject1_beam_path_cost(path)) {
        strcpy(path, g_subject1_optimized_path);
    }
    return 1;
}

static int subject1_beam_abs(int value)
{
    return value < 0 ? -value : value;
}

/* A cheap future estimate used to rank partial nodes.  Exact reachability is
 * still checked by the single-box BFS before a child enters the beam.
 */
static int subject1_beam_remaining_estimate(const PlannerGrid *grid)
{
    PlannerPoint player;
    PlannerPoint boxes[PLANNER_MAX_BOXES];
    PlannerPoint targets[PLANNER_MAX_TARGETS];
    int box_count = 0;
    int target_count = 0;
    int total = 0;
    int b;
    int t;

    if(!solver_extract_positions(grid, &player, boxes, &box_count,
                                 targets, &target_count)) {
        return 1000000;
    }
    if(box_count != target_count) {
        return 1000000;
    }

    for(b = 0; b < box_count; b++) {
        int best = 1000000;
        for(t = 0; t < target_count; t++) {
            int distance =
                subject1_beam_abs(boxes[b].r - targets[t].r) +
                subject1_beam_abs(boxes[b].c - targets[t].c);
            if(distance < best) {
                best = distance;
            }
        }
        if(best < 1000000) {
            total += best;
        }
    }
    return total;
}

static void subject1_beam_update_score(Subject1BeamNode *node)
{
    node->remaining_estimate =
        subject1_beam_remaining_estimate(&node->grid);
    node->score =
        node->metrics.steps * SUBJECT1_COST_STEP +
        node->metrics.segments * SUBJECT1_COST_SEGMENT +
        node->metrics.one_cell * SUBJECT1_COST_ONE_CELL +
        node->metrics.short_segments * SUBJECT1_COST_SHORT_SEGMENT +
        node->remaining_estimate * SUBJECT1_COST_REMAINING;
}

static int subject1_beam_node_better(const Subject1BeamNode *a,
                                     const Subject1BeamNode *b)
{
    if(a->score != b->score) {
        return a->score < b->score;
    }
    if(a->metrics.one_cell != b->metrics.one_cell) {
        return a->metrics.one_cell < b->metrics.one_cell;
    }
    if(a->metrics.segments != b->metrics.segments) {
        return a->metrics.segments < b->metrics.segments;
    }
    if(a->metrics.short_segments != b->metrics.short_segments) {
        return a->metrics.short_segments < b->metrics.short_segments;
    }
    return a->metrics.steps < b->metrics.steps;
}

static int subject1_beam_same_state(const Subject1BeamNode *a,
                                    const Subject1BeamNode *b)
{
    return memcmp(&a->grid, &b->grid, sizeof(a->grid)) == 0;
}

static void subject1_beam_insert(Subject1BeamNode nodes[],
                                 int *node_count,
                                 const Subject1BeamNode *candidate)
{
    int i;
    int worst;

    /* Equivalent predicted maps have identical futures in the planner. */
    for(i = 0; i < *node_count; i++) {
        if(subject1_beam_same_state(&nodes[i], candidate)) {
            if(subject1_beam_node_better(candidate, &nodes[i])) {
                nodes[i] = *candidate;
            }
            return;
        }
    }

    if(*node_count < SUBJECT1_BEAM_WIDTH) {
        nodes[*node_count] = *candidate;
        (*node_count)++;
        return;
    }

    worst = 0;
    for(i = 1; i < *node_count; i++) {
        if(subject1_beam_node_better(&nodes[worst], &nodes[i])) {
            worst = i;
        }
    }
    if(subject1_beam_node_better(candidate, &nodes[worst])) {
        nodes[worst] = *candidate;
    }
}
#define SUBJECT1_ROUTE_VARIANTS 3

static const uint8_t g_subject1_dir_orders
    [SUBJECT1_ROUTE_VARIANTS][4] =
{
    {0, 1, 2, 3}, /* ????? */
    {2, 3, 0, 1}, /* ???????? */
    {0, 1, 3, 2}  /* ???????¡ê???????? */
};
static int subject1_beam_expand_node(const Subject1BeamNode *parent,
                                     Subject1BeamNode next_nodes[],
                                     int *next_count)
{
    PlannerPoint player;
    PlannerPoint boxes[PLANNER_MAX_BOXES];
    PlannerPoint targets[PLANNER_MAX_TARGETS];
    int box_count = 0;
    int target_count = 0;
    int produced = 0;
    int b;
    int t;
    int variant;

    if(!solver_extract_positions(&parent->grid, &player, boxes, &box_count,
                                 targets, &target_count)) {
        return 0;
    }
    if(box_count <= 0 || box_count != target_count) {
        return 0;
    }

    for(b = 0; b < box_count; b++) {
        for(t = 0; t < target_count; t++) {
           Subject1BeamMetrics action_metrics;
            int sub_len;

            if(!build_submap_from_grid(&parent->grid, boxes[b], targets[t],
                           &g_subject1_submap)) {
    continue;
}

for(variant = 0;
    variant < SUBJECT1_ROUTE_VARIANTS;
    variant++)
{
    if(!solver_solve_single_box_bfs_ordered(
            &g_subject1_submap,
            g_subject1_dir_orders[variant],
            g_subject1_sub_path,
            (int)sizeof(g_subject1_sub_path))) {
        continue;
    }

    if(!solver_simulate_path(&parent->grid,
                             g_subject1_sub_path,
                             player,
                             boxes[b],
                             targets[t],
                             &g_subject1_new_grid,
                             0)) {
        continue;
    }

    sub_len = (int)strlen(g_subject1_sub_path);

    if(sub_len <= 0 ||
       parent->path_len + sub_len >= SUBJECT1_BEAM_PATH_LEN) {
        continue;
    }

    g_subject1_candidate = *parent;

    solver_clone_grid(&g_subject1_new_grid,
                      &g_subject1_candidate.grid);

    memcpy(g_subject1_candidate.path + parent->path_len,
           g_subject1_sub_path,
           (unsigned int)(sub_len + 1));

    g_subject1_candidate.path_len =
        parent->path_len + sub_len;

    g_subject1_candidate.depth =
        parent->depth + 1;

        
    g_subject1_candidate.box_order[parent->depth] =
    (uint8_t)(boxes[b].r * PLANNER_COLS + boxes[b].c);

    g_subject1_candidate.target_order[parent->depth] =
    (uint8_t)(targets[t].r * PLANNER_COLS + targets[t].c);

    g_subject1_candidate.action_end[parent->depth] =
    (uint16_t)g_subject1_candidate.path_len;
    
    if(parent->depth == 0) {
        g_subject1_candidate.first_path_len = sub_len;
        g_subject1_candidate.first_box = boxes[b];
        g_subject1_candidate.first_target = targets[t];
    }

   subject1_beam_measure_path(g_subject1_sub_path,
                           &action_metrics);

g_subject1_candidate.metrics.steps +=
    action_metrics.steps;

g_subject1_candidate.metrics.segments +=
    action_metrics.segments;

g_subject1_candidate.metrics.one_cell +=
    action_metrics.one_cell;

g_subject1_candidate.metrics.short_segments +=
    action_metrics.short_segments;

    subject1_beam_update_score(&g_subject1_candidate);

    subject1_beam_insert(next_nodes,
                         next_count,
                         &g_subject1_candidate);

    produced = 1;
}
        }
    }
    return produced;
}

/*
 * Rebuild every action of the selected Beam plan.
 *
 * Beam expansion stays cheap because it still uses the three BFS variants.
 * Only the final winning plan is rebuilt here.  Each box action receives the
 * direction-aware walk optimization before the complete route is returned to
 * the Level 1 cache.
 */
static int subject1_emit_selected_path(const PlannerGrid *state_grid,
                                       const Subject1BeamNode *selected,
                                       char *out_path,
                                       int out_path_size)
{
    PlannerGrid work_grid;
    PlannerPoint player;
    PlannerPoint boxes[PLANNER_MAX_BOXES];
    PlannerPoint targets[PLANNER_MAX_TARGETS];
    int box_count = 0;
    int target_count = 0;
    int raw_start = 0;
    int out_len = 0;
    int action;

    if(state_grid == 0 || selected == 0 || out_path == 0 ||
       out_path_size <= 0 || selected->depth <= 0 ||
       selected->depth > PLANNER_MAX_BOXES ||
       selected->path_len <= 0 ||
       selected->path_len >= SUBJECT1_BEAM_PATH_LEN) {
        return 0;
    }
    out_path[0] = '\0';
    solver_clone_grid(state_grid, &work_grid);

    for(action = 0; action < selected->depth; action++) {
        PlannerPoint active_box;
        PlannerPoint active_target;
        int raw_end = (int)selected->action_end[action];
        int raw_len;
        int optimized_len;
        uint8_t box_cell = selected->box_order[action];
        uint8_t target_cell = selected->target_order[action];

        if(raw_end <= raw_start || raw_end > selected->path_len) {
            return 0;
        }
        raw_len = raw_end - raw_start;
        if(raw_len <= 0 || raw_len >= SUBJECT1_BEAM_PATH_LEN) {
            return 0;
        }

        active_box.r = (int16_t)(box_cell / PLANNER_COLS);
        active_box.c = (int16_t)(box_cell % PLANNER_COLS);
        active_target.r = (int16_t)(target_cell / PLANNER_COLS);
        active_target.c = (int16_t)(target_cell % PLANNER_COLS);

        if(!planner_point_valid(active_box) ||
           !planner_point_valid(active_target) ||
           work_grid.grid[active_box.r][active_box.c] != '$' ||
           work_grid.grid[active_target.r][active_target.c] != '.') {
            return 0;
        }

        memcpy(g_subject1_first_path,
               selected->path + raw_start,
               (unsigned int)raw_len);
        g_subject1_first_path[raw_len] = '\0';

        if(!build_submap_from_grid(&work_grid,
                                   active_box,
                                   active_target,
                                   &g_subject1_submap)) {
            return 0;
        }
        subject1_optimize_walk_segments(&g_subject1_submap,
                                        g_subject1_first_path,
                                        (int)sizeof(g_subject1_first_path));

        optimized_len = (int)strlen(g_subject1_first_path);
        if(optimized_len <= 0 ||
           out_len + optimized_len >= out_path_size ||
           out_len + optimized_len >= SUBJECT1_BEAM_PATH_LEN) {
            return 0;
        }

        if(!solver_extract_positions(&work_grid,
                                     &player,
                                     boxes,
                                     &box_count,
                                     targets,
                                     &target_count) ||
           !solver_simulate_path(&work_grid,
                                 g_subject1_first_path,
                                 player,
                                 active_box,
                                 active_target,
                                 &g_subject1_new_grid,
                                 0)) {
            return 0;
        }

        memcpy(out_path + out_len,
               g_subject1_first_path,
               (unsigned int)optimized_len);
        out_len += optimized_len;
        out_path[out_len] = '\0';

        solver_clone_grid(&g_subject1_new_grid, &work_grid);
        raw_start = raw_end;
    }

    return raw_start == selected->path_len && out_len > 0;
}

int solver_solve_subject1_best_path(const PlannerGrid *state_grid,
                                    char *out_path,
                                    int out_path_size)
{
    Subject1BeamNode *current = g_subject1_beam_a;
    Subject1BeamNode *next = g_subject1_beam_b;
    Subject1BeamNode *swap;
    PlannerPoint player;
    PlannerPoint boxes[PLANNER_MAX_BOXES];
    PlannerPoint targets[PLANNER_MAX_TARGETS];
    int box_count = 0;
    int target_count = 0;
    int current_count = 1;
    int next_count;
    int depth;
    int i;
    int best;

    if(state_grid == 0 || out_path == 0 || out_path_size <= 0) {
        return 0;
    }
    out_path[0] = '\0';

    if(!solver_extract_positions(state_grid, &player, boxes, &box_count,
                                 targets, &target_count)) {
        return 0;
    }
    if(box_count <= 0 || box_count != target_count ||
       box_count > PLANNER_MAX_OBJECTS) {
        return 0;
    }

    memset(g_subject1_beam_a, 0, sizeof(g_subject1_beam_a));
    memset(g_subject1_beam_b, 0, sizeof(g_subject1_beam_b));
    solver_clone_grid(state_grid, &current[0].grid);
    current[0].path[0] = '\0';
    current[0].path_len = 0;
    current[0].depth = 0;
    subject1_beam_update_score(&current[0]);

    for(depth = 0; depth < box_count; depth++) {
        next_count = 0;
        for(i = 0; i < current_count; i++) {
            subject1_beam_expand_node(&current[i], next, &next_count);
        }
        if(next_count <= 0) {
            out_path[0] = '\0';
            return 0;
        }
        swap = current;
        current = next;
        next = swap;
        current_count = next_count;
    }

    best = 0;
    for(i = 1; i < current_count; i++) {
        if(subject1_beam_node_better(&current[i], &current[best])) {
            best = i;
        }
    }
    if(current[best].path_len <= 0 ||
       current[best].path_len >= out_path_size) {
        out_path[0] = '\0';
        return 0;
    }

    if(!subject1_emit_selected_path(state_grid, &current[best],
                                    out_path, out_path_size)) {
        memcpy(out_path, current[best].path,
               (unsigned int)(current[best].path_len + 1));
    }
    return 1;
}
