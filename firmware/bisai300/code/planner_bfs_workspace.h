#ifndef PLANNER_BFS_WORKSPACE_H
#define PLANNER_BFS_WORKSPACE_H

#include <stdint.h>
#include "planner_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Shared BFS workspace for solver.c and bomb_push_planner.c.
 *
 * Original Python used dynamic deque/set objects. The first C port used one
 * full BFS workspace in solver.c and another full workspace in bomb_push_planner.c.
 * On RT1064/IAR that exceeds RAM, so both BFS users share this buffer.
 *
 * Each BFS state index is in [0, PLANNER_SINGLE_STATE_COUNT), and for the
 * fixed 12 x 16 map PLANNER_SINGLE_STATE_COUNT = 36864, so uint16_t is enough.
 */
#if PLANNER_SINGLE_STATE_COUNT > 65535
#error "PLANNER_SINGLE_STATE_COUNT must fit in uint16_t for planner_bfs_workspace"
#endif

#define PLANNER_BFS_PARENT_NONE ((uint16_t)0xFFFFu)

extern uint16_t planner_bfs_queue[PLANNER_SINGLE_STATE_COUNT];
extern uint16_t planner_bfs_parent[PLANNER_SINGLE_STATE_COUNT];
extern char planner_bfs_parent_action[PLANNER_SINGLE_STATE_COUNT];
extern uint8_t planner_bfs_visited[PLANNER_SINGLE_STATE_COUNT];

void planner_bfs_workspace_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* PLANNER_BFS_WORKSPACE_H */
