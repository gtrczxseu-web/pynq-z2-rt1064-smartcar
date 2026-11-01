#include "openart_text_parser.h"
#include "openart_symbol_mapper.h"
#include <string.h>

static int is_map_char(char ch)
{
    return ch == '#' || ch == '-' || ch == '@' || ch == '$' || ch == '.' || ch == '*' ||
           ch == 'G' || ch == 'R' || ch == 'P' || ch == 'Y' || ch == 'C' || ch == 'L' || ch == '?';
}

static int line_is_map_row(const char *line, int len)
{
    int i;
    if(len != PLANNER_COLS) {
        return 0;
    }
    for(i = 0; i < len; i++) {
        if(!is_map_char(line[i])) {
            return 0;
        }
    }
    return 1;
}

static int parse_lines_from_text(const char *text, PlannerGrid *out_grid)
{
    char raw[PLANNER_ROWS][PLANNER_COLS];
    int row_count = 0;
    const char *p = text;

    while(*p != '\0' && row_count < PLANNER_ROWS) {
        char line[PLANNER_COLS + 4];
        int len = 0;

        while(*p == '\r' || *p == '\n') {
            p++;
        }
        while(*p != '\0' && *p != '\r' && *p != '\n') {
            if(len < (int)sizeof(line) - 1) {
                line[len++] = *p;
            }
            p++;
        }
        line[len] = '\0';

        if(line_is_map_row(line, len)) {
            int c;
            for(c = 0; c < PLANNER_COLS; c++) {
                raw[row_count][c] = line[c];
            }
            row_count++;
        }
    }

    if(row_count != PLANNER_ROWS) {
        return 0;
    }

    return openart_symbol_mapper_raw_to_solver(raw, out_grid);
}

int openart_text_parser_parse_text(const char *text, PlannerGrid *out_grid)
{
    const char *start;
    const char *end;
    char inner[PLANNER_TEXT_BUFFER_SIZE];
    int len;

    if(text == 0 || out_grid == 0) {
        return 0;
    }

    start = strstr(text, "<MAP>");
    if(start != 0) {
        start += 5;
        end = strstr(start, "</MAP>");
        if(end == 0) {
            return 0;
        }
        len = (int)(end - start);
        if(len >= (int)sizeof(inner)) {
            return 0;
        }
        memcpy(inner, start, (size_t)len);
        inner[len] = '\0';
        return parse_lines_from_text(inner, out_grid);
    }

    start = strstr(text, "MAP_NOW:");
    if(start != 0) {
        start += 8;
        return parse_lines_from_text(start, out_grid);
    }

    start = strstr(text, "MAP:");
    if(start != 0) {
        start += 4;
        return parse_lines_from_text(start, out_grid);
    }

    return parse_lines_from_text(text, out_grid);
}

void openart_text_parser_init(OpenartTextParser *parser)
{
    if(parser == 0) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
}

int openart_text_parser_feed_char(OpenartTextParser *parser, char ch)
{
    if(parser == 0) {
        return 0;
    }

    if(parser->len < PLANNER_TEXT_BUFFER_SIZE - 1) {
        parser->buffer[parser->len++] = ch;
        parser->buffer[parser->len] = '\0';
    } else {
        parser->len = 0;
        parser->buffer[0] = '\0';
        parser->in_map = 0;
        return 0;
    }

    if(strstr(parser->buffer, "<MAP>") != 0) {
        parser->in_map = 1;
    }

    if(parser->in_map && strstr(parser->buffer, "</MAP>") != 0) {
        if(openart_text_parser_parse_text(parser->buffer, &parser->grid)) {
            parser->map_ready = 1;
            parser->len = 0;
            parser->buffer[0] = '\0';
            parser->in_map = 0;
            return 1;
        }
        parser->len = 0;
        parser->buffer[0] = '\0';
        parser->in_map = 0;
    }

    return 0;
}
