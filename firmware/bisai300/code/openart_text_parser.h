#ifndef OPENART_TEXT_PARSER_H
#define OPENART_TEXT_PARSER_H

#include "planner_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char buffer[PLANNER_TEXT_BUFFER_SIZE];
    int len;
    int in_map;
    int map_ready;
    PlannerGrid grid;
} OpenartTextParser;

void openart_text_parser_init(OpenartTextParser *parser);

/* Feed one byte from UART. Returns 1 exactly when a complete valid map has just been parsed. */
int openart_text_parser_feed_char(OpenartTextParser *parser, char ch);

/* Parse a complete text block containing <MAP>...</MAP>, MAP:, MAP_NOW:, or raw 12 map lines. */
int openart_text_parser_parse_text(const char *text, PlannerGrid *out_grid);

#ifdef __cplusplus
}
#endif

#endif /* OPENART_TEXT_PARSER_H */
