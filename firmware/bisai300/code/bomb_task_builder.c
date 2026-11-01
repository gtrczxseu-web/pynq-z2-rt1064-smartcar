#include "bomb_task_builder.h"
#include "bomb_rule_engine.h"

static int manhattan_distance(PlannerPoint a, PlannerPoint b)
{
    int dr = a.r - b.r;
    int dc = a.c - b.c;
    if(dr < 0) {
        dr = -dr;
    }
    if(dc < 0) {
        dc = -dc;
    }
    return dr + dc;
}

static int collect_bombs(const PlannerGrid *grid, PlannerPoint bombs[], int max_bombs)
{
    int r, c;
    int count = 0;
    if(grid == 0 || bombs == 0 || max_bombs <= 0) {
        return 0;
    }
    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            if(grid->grid[r][c] == '*') {
                if(count < max_bombs) {
                    bombs[count].r = (int16_t)r;
                    bombs[count].c = (int16_t)c;
                }
                count++;
            }
        }
    }
    if(count > max_bombs) {
        return max_bombs;
    }
    return count;
}

static int task_better_than(const BombTask *a, const BombTask *b)
{
    if(a->distance_score != b->distance_score) {
        return a->distance_score < b->distance_score;
    }
    if(a->affected_wall_count != b->affected_wall_count) {
        return a->affected_wall_count > b->affected_wall_count;
    }
    if(a->bomb.r != b->bomb.r) {
        return a->bomb.r < b->bomb.r;
    }
    if(a->bomb.c != b->bomb.c) {
        return a->bomb.c < b->bomb.c;
    }
    if(a->target_wall.r != b->target_wall.r) {
        return a->target_wall.r < b->target_wall.r;
    }
    return a->target_wall.c < b->target_wall.c;
}

static void sort_tasks(BombTask tasks[], int count)
{
    int i, j;
    for(i = 1; i < count; i++) {
        BombTask key = tasks[i];
        j = i - 1;
        while(j >= 0 && task_better_than(&key, &tasks[j])) {
            tasks[j + 1] = tasks[j];
            j--;
        }
        tasks[j + 1] = key;
    }
}

int bomb_task_builder_generate(const PlannerGrid *grid,
                               BombTask tasks[],
                               int max_tasks,
                               int *out_task_count)
{
    PlannerPoint bombs[PLANNER_MAX_BOMBS];
    int bomb_count;
    int r, c, b;
    int count = 0;
    int next_task_id = 0;

    if(out_task_count != 0) {
        *out_task_count = 0;
    }

    if(grid == 0 || tasks == 0 || max_tasks <= 0) {
        return 0;
    }

    bomb_count = collect_bombs(grid, bombs, PLANNER_MAX_BOMBS);
    if(bomb_count <= 0) {
        return 1;
    }

    for(b = 0; b < bomb_count; b++) {
        for(r = 1; r < PLANNER_ROWS - 1; r++) {
            for(c = 1; c < PLANNER_COLS - 1; c++) {
                PlannerPoint wall;
                PlannerPoint affected[BOMB_MAX_AFFECTED_WALLS];
                int affected_count;

                wall.r = (int16_t)r;
                wall.c = (int16_t)c;

                if(!bomb_rule_is_valid_target_wall(grid, wall)) {
                    continue;
                }

                affected_count = bomb_rule_get_affected_walls(grid, wall,
                                                              affected,
                                                              BOMB_MAX_AFFECTED_WALLS);
                if(affected_count <= 0) {
                    continue;
                }

                if(count >= max_tasks) {
                    /* Static MCU buffer is full. Return the best prefix we can store. */
                    sort_tasks(tasks, count);
                    if(out_task_count != 0) {
                        *out_task_count = count;
                    }
                    return 1;
                }

                tasks[count].task_id = next_task_id++;
                tasks[count].bomb = bombs[b];
                tasks[count].target_wall = wall;
                tasks[count].blast_center = wall;
                tasks[count].affected_wall_count = affected_count;
                tasks[count].distance_score = manhattan_distance(bombs[b], wall);
                for(int i = 0; i < affected_count && i < BOMB_MAX_AFFECTED_WALLS; i++) {
                    tasks[count].affected_walls[i] = affected[i];
                }
                count++;
            }
        }
    }

    sort_tasks(tasks, count);
    if(out_task_count != 0) {
        *out_task_count = count;
    }
    return 1;
}
