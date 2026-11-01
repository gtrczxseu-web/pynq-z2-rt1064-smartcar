#include "planner_bfs_workspace.h"
#include <string.h>

uint16_t planner_bfs_queue[PLANNER_SINGLE_STATE_COUNT];
uint16_t planner_bfs_parent[PLANNER_SINGLE_STATE_COUNT];
char planner_bfs_parent_action[PLANNER_SINGLE_STATE_COUNT];
uint8_t planner_bfs_visited[PLANNER_SINGLE_STATE_COUNT];

void planner_bfs_workspace_clear(void)
{
    memset(planner_bfs_visited, 0, sizeof(planner_bfs_visited));
    memset(planner_bfs_parent, 0xFF, sizeof(planner_bfs_parent));
    memset(planner_bfs_parent_action, 0, sizeof(planner_bfs_parent_action));
}
