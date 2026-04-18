/*
 * json.c — minimal recursive-descent JSON parser
 *
 * Supports: objects, arrays, strings, numbers (including scientific
 * notation handled by strtod), booleans, null, and // line comments.
 */
#include "json.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

/* ------------------------------------------------------------------ parser state */

typedef struct {
    const char *src;   /* full input text */
    const char *p;     /* current position */
    int         line;  /* 1-based line number for error messages */
    int         error; /* set to 1 on first error */
} Parser;

/* ------------------------------------------------------------------ forward decls */
static JsonNode *parse_value(Parser *ps);

/* ------------------------------------------------------------------ helpers */

static void skip_whitespace_and_comments(Parser *ps)
{
    for (;;) {
        /* skip ordinary whitespace */
        while (*ps->p && (unsigned char)*ps->p <= ' ') {
            if (*ps->p == '\n') ps->line++;
            ps->p++;
        }
        /* skip // line comments */
        if (ps->p[0] == '/' && ps->p[1] == '/') {
            while (*ps->p && *ps->p != '\n') ps->p++;
            continue;
        }
        break;
    }
}

static void parse_error(Parser *ps, const char *msg)
{
    if (!ps->error) {
        fprintf(stderr, "[json] parse error at line %d: %s\n", ps->line, msg);
        ps->error = 1;
    }
}

static JsonNode *alloc_node(void)
{
    JsonNode *n = (JsonNode *)calloc(1, sizeof(JsonNode));
    return n;
}

/* ------------------------------------------------------------------ string parsing */

/*
 * parse_string — consume a JSON "..." string starting at ps->p (which
 * must already point at the opening quote).  Returns heap-allocated
 * NUL-terminated string, or NULL on error.
 */
static char *parse_string(Parser *ps)
{
    if (*ps->p != '"') {
        parse_error(ps, "expected '\"'");
        return NULL;
    }
    ps->p++;  /* skip opening quote */

    /* first pass: measure length */
    const char *start = ps->p;
    size_t len = 0;
    const char *q = ps->p;
    while (*q && *q != '"') {
        if (*q == '\\') {
            q++;
            if (!*q) break;
        }
        len++;
        q++;
    }

    char *buf = (char *)malloc(len + 1);
    if (!buf) { parse_error(ps, "out of memory"); return NULL; }

    /* second pass: copy with escape handling */
    char *out = buf;
    while (*ps->p && *ps->p != '"') {
        if (*ps->p == '\\') {
            ps->p++;
            switch (*ps->p) {
            case '"':  *out++ = '"';  break;
            case '\\': *out++ = '\\'; break;
            case '/':  *out++ = '/';  break;
            case 'b':  *out++ = '\b'; break;
            case 'f':  *out++ = '\f'; break;
            case 'n':  *out++ = '\n'; break;
            case 'r':  *out++ = '\r'; break;
            case 't':  *out++ = '\t'; break;
            case 'u': {
                /* simplified: consume 4 hex digits, output '?' */
                int k;
                for (k = 0; k < 4 && ps->p[1] && isxdigit((unsigned char)ps->p[1]); k++)
                    ps->p++;
                *out++ = '?';
                break;
            }
            default:
                *out++ = *ps->p;
                break;
            }
            if (*ps->p) ps->p++;
        } else {
            if (*ps->p == '\n') ps->line++;
            *out++ = *ps->p++;
        }
    }
    *out = '\0';
    (void)start;

    if (*ps->p == '"') ps->p++; /* skip closing quote */
    else parse_error(ps, "unterminated string");

    return buf;
}

/* ------------------------------------------------------------------ value parsers */

static JsonNode *parse_object(Parser *ps)
{
    if (*ps->p != '{') { parse_error(ps, "expected '{'"); return NULL; }
    ps->p++;

    JsonNode *node = alloc_node();
    node->type = JSON_OBJECT;

    JsonNode *last_child = NULL;

    skip_whitespace_and_comments(ps);
    if (*ps->p == '}') { ps->p++; return node; }

    for (;;) {
        skip_whitespace_and_comments(ps);
        if (ps->error) break;
        if (*ps->p != '"') { parse_error(ps, "expected key string"); break; }

        char *key = parse_string(ps);
        if (!key || ps->error) break;

        skip_whitespace_and_comments(ps);
        if (*ps->p != ':') { free(key); parse_error(ps, "expected ':'"); break; }
        ps->p++;
        skip_whitespace_and_comments(ps);

        JsonNode *child = parse_value(ps);
        if (!child || ps->error) { free(key); break; }
        child->key = key;

        if (last_child) last_child->next = child;
        else            node->first_child = child;
        last_child = child;

        skip_whitespace_and_comments(ps);
        if (*ps->p == ',') {
            ps->p++;
            /* allow trailing comma before '}' */
            skip_whitespace_and_comments(ps);
            if (*ps->p == '}') break;
            continue;
        }
        break;
    }

    skip_whitespace_and_comments(ps);
    if (!ps->error && *ps->p == '}') ps->p++;
    else if (!ps->error) parse_error(ps, "expected '}'");

    return node;
}

static JsonNode *parse_array(Parser *ps)
{
    if (*ps->p != '[') { parse_error(ps, "expected '['"); return NULL; }
    ps->p++;

    JsonNode *node = alloc_node();
    node->type = JSON_ARRAY;

    JsonNode *last_child = NULL;

    skip_whitespace_and_comments(ps);
    if (*ps->p == ']') { ps->p++; return node; }

    for (;;) {
        skip_whitespace_and_comments(ps);
        if (ps->error) break;

        JsonNode *child = parse_value(ps);
        if (!child || ps->error) break;

        if (last_child) last_child->next = child;
        else            node->first_child = child;
        last_child = child;

        skip_whitespace_and_comments(ps);
        if (*ps->p == ',') {
            ps->p++;
            /* allow trailing comma before ']' */
            skip_whitespace_and_comments(ps);
            if (*ps->p == ']') break;
            continue;
        }
        break;
    }

    skip_whitespace_and_comments(ps);
    if (!ps->error && *ps->p == ']') ps->p++;
    else if (!ps->error) parse_error(ps, "expected ']'");

    return node;
}

static JsonNode *parse_value(Parser *ps)
{
    skip_whitespace_and_comments(ps);
    if (ps->error) return NULL;

    char c = *ps->p;

    if (c == '{') return parse_object(ps);
    if (c == '[') return parse_array(ps);

    if (c == '"') {
        JsonNode *node = alloc_node();
        node->type = JSON_STRING;
        node->string = parse_string(ps);
        if (ps->error) { json_free(node); return NULL; }
        return node;
    }

    /* true / false / null */
    if (strncmp(ps->p, "true", 4) == 0) {
        ps->p += 4;
        JsonNode *node = alloc_node();
        node->type = JSON_BOOL;
        node->boolean = 1;
        return node;
    }
    if (strncmp(ps->p, "false", 5) == 0) {
        ps->p += 5;
        JsonNode *node = alloc_node();
        node->type = JSON_BOOL;
        node->boolean = 0;
        return node;
    }
    if (strncmp(ps->p, "null", 4) == 0) {
        ps->p += 4;
        JsonNode *node = alloc_node();
        node->type = JSON_NULL;
        return node;
    }

    /* number */
    if (c == '-' || (c >= '0' && c <= '9')) {
        char *end;
        double val = strtod(ps->p, &end);
        if (end == ps->p) { parse_error(ps, "invalid number"); return NULL; }
        ps->p = end;
        JsonNode *node = alloc_node();
        node->type = JSON_NUMBER;
        node->number = val;
        return node;
    }

    parse_error(ps, "unexpected character");
    return NULL;
}

/* ------------------------------------------------------------------ public API */

JsonNode *json_parse(const char *text)
{
    Parser ps;
    ps.src   = text;
    ps.p     = text;
    ps.line  = 1;
    ps.error = 0;

    skip_whitespace_and_comments(&ps);
    JsonNode *root = parse_value(&ps);
    if (ps.error) {
        json_free(root);
        return NULL;
    }
    return root;
}

JsonNode *json_parse_file(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[json] cannot open '%s'\n", path);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *buf = (char *)malloc((size_t)sz + 1);
    if (!buf) {
        fclose(f);
        fprintf(stderr, "[json] out of memory reading '%s'\n", path);
        return NULL;
    }

    size_t got = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[got] = '\0';

    JsonNode *root = json_parse(buf);
    free(buf);
    return root;
}

void json_free(JsonNode *root)
{
    if (!root) return;
    /* free all siblings */
    JsonNode *n = root;
    while (n) {
        JsonNode *next = n->next;
        free(n->key);
        free(n->string);
        json_free(n->first_child);
        free(n);
        n = next;
    }
}

JsonNode *json_get(const JsonNode *obj, const char *key)
{
    if (!obj || obj->type != JSON_OBJECT) return NULL;
    JsonNode *child = obj->first_child;
    while (child) {
        if (child->key && strcmp(child->key, key) == 0) return child;
        child = child->next;
    }
    return NULL;
}

JsonNode *json_idx(const JsonNode *arr, int i)
{
    if (!arr || arr->type != JSON_ARRAY) return NULL;
    JsonNode *child = arr->first_child;
    int idx = 0;
    while (child) {
        if (idx == i) return child;
        idx++;
        child = child->next;
    }
    return NULL;
}

double json_num(const JsonNode *node, double def)
{
    if (!node || node->type != JSON_NUMBER) return def;
    return node->number;
}

const char *json_str(const JsonNode *node, const char *def)
{
    if (!node || node->type != JSON_STRING) return def;
    return node->string;
}

int json_bool(const JsonNode *node, int def)
{
    if (!node || node->type != JSON_BOOL) return def;
    return node->boolean;
}
