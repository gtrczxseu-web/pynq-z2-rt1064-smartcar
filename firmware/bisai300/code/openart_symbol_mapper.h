#ifndef OPENART_SYMBOL_MAPPER_H
#define OPENART_SYMBOL_MAPPER_H

#include "planner_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Convert raw OpenART symbols G/R/P/Y/C/L/-/? to solver-standard characters.
 * Unknown '?' is converted to '-' for runtime tolerance, matching current OpenART practice.
 */
int openart_symbol_mapper_raw_to_solver(const char raw_grid[PLANNER_ROWS][PLANNER_COLS],
                                        PlannerGrid *out_grid);

char openart_symbol_mapper_char(char raw_ch);

#ifdef __cplusplus
}
#endif

#endif /* OPENART_SYMBOL_MAPPER_H */
