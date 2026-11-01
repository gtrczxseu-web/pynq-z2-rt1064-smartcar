#include "openart_symbol_mapper.h"

char openart_symbol_mapper_char(char raw_ch)
{
    switch(raw_ch) {
        case 'G': return '#';
        case 'R': return '*';
        case 'P': return '.';
        case 'Y': return '$';
        case 'C': return '@';
        case 'L': return '@';
        case '-': return '-';
        case '?': return '-';
        case '#': return '#';
        case '*': return '*';
        case '.': return '.';
        case '$': return '$';
        case '@': return '@';
        default:  return '-';
    }
}

int openart_symbol_mapper_raw_to_solver(const char raw_grid[PLANNER_ROWS][PLANNER_COLS],
                                        PlannerGrid *out_grid)
{
    int r, c;
    int player_written = 0;

    if(raw_grid == 0 || out_grid == 0) {
        return 0;
    }

    for(r = 0; r < PLANNER_ROWS; r++) {
        for(c = 0; c < PLANNER_COLS; c++) {
            char mapped = openart_symbol_mapper_char(raw_grid[r][c]);
            if(mapped == '@') {
                if(player_written) {
                    mapped = '-';
                } else {
                    player_written = 1;
                }
            }
            out_grid->grid[r][c] = mapped;
        }
    }

    /* Keep outer border as wall, matching OpenART mapper rule. */
    for(c = 0; c < PLANNER_COLS; c++) {
        out_grid->grid[0][c] = '#';
        out_grid->grid[PLANNER_ROWS - 1][c] = '#';
    }
    for(r = 0; r < PLANNER_ROWS; r++) {
        out_grid->grid[r][0] = '#';
        out_grid->grid[r][PLANNER_COLS - 1] = '#';
    }

    return 1;
}
