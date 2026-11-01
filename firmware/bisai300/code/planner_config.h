#ifndef PLANNER_CONFIG_H
#define PLANNER_CONFIG_H

/*
 * Planner fixed limits for the 12 x 16 smart-car grid.
 * These values replace Python's dynamic list/set/dict capacity with static MCU-safe buffers.
 */
#define PLANNER_ROWS                12
#define PLANNER_COLS                16
#define PLANNER_CELL_COUNT          (PLANNER_ROWS * PLANNER_COLS)

/* Path length upper bound. Single-box BFS state count is 192 * 192 = 36864.
 * A valid shortest path will never need to exceed the number of unique states.
 */
#define PLANNER_MAX_PATH_LEN        4096

/* Object limits. If the camera produces more objects than these limits, the C solver returns failure.
 * They are intentionally explicit so MCU memory use stays predictable.
 */
#define PLANNER_MAX_OBJECTS         8
#define PLANNER_MAX_BOXES           PLANNER_MAX_OBJECTS
#define PLANNER_MAX_TARGETS         PLANNER_MAX_OBJECTS
#define PLANNER_MAX_BOMBS           PLANNER_MAX_OBJECTS

/* Single-box BFS state space: player_cell * box_cell. */
#define PLANNER_SINGLE_STATE_COUNT  (PLANNER_CELL_COUNT * PLANNER_CELL_COUNT)

/* UART text parser buffer. 12 rows * 16 chars plus line breaks and tags. */
#define PLANNER_TEXT_BUFFER_SIZE    512

/* MCU compressed path command limit. */
#define PLANNER_MAX_MCU_COMMANDS    256

/* Path selection weights for real-car execution.
 * A compressed direction segment such as E,6 counts as one command, so command count
 * is weighted much higher than raw grid steps. The solver still uses raw steps as a
 * tie-breaker when command counts are equal.
 */
#define PLANNER_PATH_COST_COMMAND_WEIGHT 10000
#define PLANNER_PATH_COST_STEP_WEIGHT    1

/* Subject 1 bounded task-level Beam Search. */
#define SUBJECT1_BEAM_WIDTH              8
#define SUBJECT1_BEAM_PATH_LEN           512
#define SUBJECT1_COST_STEP               1
#define SUBJECT1_COST_SEGMENT            8
#define SUBJECT1_COST_ONE_CELL           6
#define SUBJECT1_COST_SHORT_SEGMENT      2
#define SUBJECT1_COST_REMAINING          1
#define SUBJECT1_WALK_STEP_COST          100
#define SUBJECT1_WALK_TURN_COST          300

/* Stage27/28 multi-bomb preview limits.
 * The controller searches 1..PLANNER_MAX_BOMB_CHAIN_DEPTH bombs, but only returns
 * the first bomb path for real-car execution. Keep these limits explicit so MCU
 * search time and stack/RAM use remain controllable.
 */
#define PLANNER_MAX_BOMB_CHAIN_DEPTH        3
#define PLANNER_MAX_MULTI_BOMB_NODES        400
#define PLANNER_MAX_MULTI_BOMB_TASK_ATTEMPTS 2000

/* Candidate bomb task cap per preview state. This overrides the fallback default
 * in bomb_task_builder.h and prevents large stack frames on MCU.
 */
#define PLANNER_MAX_BOMB_TASKS              256

#endif /* PLANNER_CONFIG_H */
