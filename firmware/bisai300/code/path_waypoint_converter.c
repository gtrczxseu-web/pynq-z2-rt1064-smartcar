#include "path_waypoint_converter.h"
#include <stdio.h>
#include <string.h>

int path_udlr_to_nsew(char action, char *out_dir)
{
    switch(action) {
        case 'U': *out_dir = 'N'; return 1;
        case 'D': *out_dir = 'S'; return 1;
        case 'L': *out_dir = 'W'; return 1;
        case 'R': *out_dir = 'E'; return 1;
        default: return 0;
    }
}

int path_compress_to_mcu_commands(const char *path, PlannerMcuCommandList *out_list)
{
    int i;
    char cur;
    char cur_dir;
    int count;

    if(out_list == 0) {
        return 0;
    }
    out_list->count = 0;

    if(path == 0 || path[0] == '\0') {
        return 1;
    }

    cur = path[0];
    if(!path_udlr_to_nsew(cur, &cur_dir)) {
        return 0;
    }
    count = 1;

    for(i = 1; path[i] != '\0'; i++) {
        char d;
        if(!path_udlr_to_nsew(path[i], &d)) {
            return 0;
        }
        if(path[i] == cur) {
            count++;
        } else {
            if(out_list->count >= PLANNER_MAX_MCU_COMMANDS) {
                return 0;
            }
            out_list->commands[out_list->count].dir = cur_dir;
            out_list->commands[out_list->count].steps = count;
            out_list->count++;
            cur = path[i];
            cur_dir = d;
            count = 1;
        }
    }

    if(out_list->count >= PLANNER_MAX_MCU_COMMANDS) {
        return 0;
    }
    out_list->commands[out_list->count].dir = cur_dir;
    out_list->commands[out_list->count].steps = count;
    out_list->count++;

    return 1;
}

int path_format_plan_text(const PlannerMcuCommandList *cmds, char *out_text, int out_size)
{
    int used = 0;
    int i;

    if(cmds == 0 || out_text == 0 || out_size <= 0) {
        return 0;
    }

    used += snprintf(out_text + used, (size_t)(out_size - used), "<PLAN>\n");
    if(used < 0 || used >= out_size) {
        return 0;
    }

    for(i = 0; i < cmds->count; i++) {
        used += snprintf(out_text + used, (size_t)(out_size - used), "%c,%d\n",
                         cmds->commands[i].dir,
                         cmds->commands[i].steps);
        if(used < 0 || used >= out_size) {
            return 0;
        }
    }

    used += snprintf(out_text + used, (size_t)(out_size - used), "</PLAN>\n");
    if(used < 0 || used >= out_size) {
        return 0;
    }

    return 1;
}
