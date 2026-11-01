#ifndef PATH_WAYPOINT_CONVERTER_H
#define PATH_WAYPOINT_CONVERTER_H

#include "planner_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Equivalent to Python compress_path_to_uart_lines(path).
 * Converts U/D/L/R path into N/S/E/W repeated-step commands.
 */
int path_compress_to_mcu_commands(const char *path, PlannerMcuCommandList *out_list);

/* Formats commands as <PLAN> text, useful for debugging or legacy UART protocol. */
int path_format_plan_text(const PlannerMcuCommandList *cmds, char *out_text, int out_size);

/* Single direction conversion. Returns 1 on success, 0 on invalid input. */
int path_udlr_to_nsew(char action, char *out_dir);

#ifdef __cplusplus
}
#endif

#endif /* PATH_WAYPOINT_CONVERTER_H */
