/*
 * gl_utils.h — OpenGL helper utilities (shader loading, buffer creation)
 */
#pragma once
#include "common.h"

/*
 * Load, compile, and link a GLSL program from two source files.
 * Returns the program ID, or 0 on failure.
 */
GLuint gl_shader_load(const char *vert_path, const char *frag_path);

/*
 * Create a VAO bound to the calling code's subsequent attribute setup.
 * The VAO is left bound on return.
 */
GLuint gl_vao_create(void);

/*
 * Create a VBO and upload data.
 *   usage: GL_STATIC_DRAW, GL_DYNAMIC_DRAW, etc.
 * The buffer is left bound to GL_ARRAY_BUFFER on return.
 */
GLuint gl_vbo_create(size_t bytes, const void *data, GLenum usage);

/*
 * Create an EBO (index buffer) and upload data.
 * Left bound to GL_ELEMENT_ARRAY_BUFFER on return.
 */
GLuint gl_ebo_create(size_t bytes, const unsigned int *data);
