/*
 * gl_utils.c — thin wrappers around common OpenGL object creation
 *
 * All functions return 0 / NULL on failure and print a diagnostic to stderr.
 * Ownership: the caller is responsible for deleting returned objects
 * (glDeleteProgram, glDeleteVertexArrays, glDeleteBuffers).
 *
 * VAO/VBO binding convention:
 *   gl_vao_create() leaves the new VAO bound.
 *   gl_vbo_create() / gl_ebo_create() leave the new buffer bound to its
 *   target. The caller sets up vertex attribute pointers, then unbinds
 *   the VAO with glBindVertexArray(0).
 */
#include "gl_utils.h"

/* ---------------------------------------------------------------- private */

/* Read an entire file into a heap-allocated NUL-terminated string.
 * Uses binary mode so line endings are preserved as-is for GLSL. */
static char *read_file(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "[GL] cannot open '%s'\n", path); return NULL; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    rewind(f);
    char *buf = (char *)malloc(sz + 1);
    if (!buf) { fclose(f); return NULL; }
    fread(buf, 1, sz, f);
    buf[sz] = '\0';
    fclose(f);
    return buf;
}

/* Compile a single shader stage and return its handle, or 0 on failure.
 * The info log (up to 1 KB) is printed to stderr on compile error.
 * path is used only for the error message — it is not re-read here. */
static GLuint compile_shader(GLenum type, const char *src, const char *path) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof(log), NULL, log);
        fprintf(stderr, "[GL] shader compile error (%s):\n%s\n", path, log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

/* ---------------------------------------------------------------- public */

/* Load, compile, and link a vertex+fragment shader pair from disk.
 * Shader objects are deleted after linking — only the program handle survives.
 * Returns the linked program, or 0 on any failure. */
GLuint gl_shader_load(const char *vert_path, const char *frag_path) {
    char *vsrc = read_file(vert_path);
    char *fsrc = read_file(frag_path);
    if (!vsrc || !fsrc) { free(vsrc); free(fsrc); return 0; }

    GLuint vs = compile_shader(GL_VERTEX_SHADER,   vsrc, vert_path);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fsrc, frag_path);
    free(vsrc); free(fsrc);
    if (!vs || !fs) { glDeleteShader(vs); glDeleteShader(fs); return 0; }

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    /* Shader objects are no longer needed once the program is linked */
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(prog, sizeof(log), NULL, log);
        fprintf(stderr, "[GL] program link error (%s / %s):\n%s\n",
                vert_path, frag_path, log);
        glDeleteProgram(prog);
        return 0;
    }
    return prog;
}

/* Create and bind a VAO.  The caller must set up vertex attribute pointers
 * before calling glBindVertexArray(0). */
GLuint gl_vao_create(void) {
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    return vao;
}

/* Create and bind a VBO, optionally uploading initial data.
 * data may be NULL for a zero-initialised or to-be-filled buffer.
 * usage is typically GL_STATIC_DRAW or GL_DYNAMIC_DRAW. */
GLuint gl_vbo_create(size_t bytes, const void *data, GLenum usage) {
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)bytes, data, usage);
    return vbo;
}

/* Create and bind an EBO (index buffer) with static data.
 * Must be called while a VAO is bound so the binding is captured. */
GLuint gl_ebo_create(size_t bytes, const unsigned int *data) {
    GLuint ebo;
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)bytes, data, GL_STATIC_DRAW);
    return ebo;
}

GLuint gl_surf_to_tex(SDL_Surface *surf, int *w, int *h) {
    SDL_Surface *c = SDL_ConvertSurfaceFormat(surf, SDL_PIXELFORMAT_ABGR8888, 0);
    SDL_FreeSurface(surf);
    if (!c) return 0;
    *w = c->w; *h = c->h;
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, c->w, c->h,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, c->pixels);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    SDL_FreeSurface(c);
    return tex;
}
