/*
 * gl_utils.c — OpenGL helpers: shader loading, buffer creation
 */
#include "gl_utils.h"

/* ------------------------------------------------------------------ internal */
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

/* ------------------------------------------------------------------ public */
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

GLuint gl_vao_create(void) {
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    return vao;
}

GLuint gl_vbo_create(size_t bytes, const void *data, GLenum usage) {
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)bytes, data, usage);
    return vbo;
}

GLuint gl_ebo_create(size_t bytes, const unsigned int *data) {
    GLuint ebo;
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)bytes, data, GL_STATIC_DRAW);
    return ebo;
}
