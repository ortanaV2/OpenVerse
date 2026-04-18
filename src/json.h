/*
 * json.h — minimal recursive-descent JSON parser
 *
 * Extensions beyond standard JSON:
 *   - // line comments
 *
 * All nodes are heap-allocated. Call json_free() on the root to release
 * the entire tree.
 */
#pragma once
#include <stddef.h>

typedef enum {
    JSON_NULL,
    JSON_BOOL,
    JSON_NUMBER,
    JSON_STRING,
    JSON_ARRAY,
    JSON_OBJECT
} JsonType;

typedef struct JsonNode JsonNode;
struct JsonNode {
    JsonType    type;
    char       *key;           /* non-null when this node is an object member */
    double      number;
    char       *string;        /* heap-allocated, valid for JSON_STRING        */
    int         boolean;       /* valid for JSON_BOOL                          */
    JsonNode   *first_child;   /* first child for JSON_OBJECT / JSON_ARRAY     */
    JsonNode   *next;          /* next sibling                                 */
};

/*
 * json_parse_file — read a file and parse it.
 * Returns NULL on I/O error or parse failure (error printed to stderr).
 */
JsonNode   *json_parse_file(const char *path);

/*
 * json_parse — parse a NUL-terminated string.
 * Returns NULL on failure (error printed to stderr).
 * The caller may free the input string after this call returns.
 */
JsonNode   *json_parse(const char *text);

/* json_free — recursively free the entire tree rooted at node. */
void        json_free(JsonNode *root);

/* json_get — find a direct child of an object by key. Returns NULL if not found. */
JsonNode   *json_get(const JsonNode *obj, const char *key);

/* json_idx — return the i-th element (0-based) of an array. Returns NULL if out of range. */
JsonNode   *json_idx(const JsonNode *arr, int i);

/* Accessors with defaults — safe even when node is NULL or wrong type. */
double      json_num (const JsonNode *node, double def);
const char *json_str (const JsonNode *node, const char *def);
int         json_bool(const JsonNode *node, int def);
