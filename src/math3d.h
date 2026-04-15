/*
 * math3d.h — header-only 3D math library (column-major Mat4, Vec3/Vec4)
 *
 * Convention: OpenGL column-major.  Element access: m[col*4 + row].
 * Matrix layout (rows shown for readability):
 *   m[ 0] m[ 4] m[ 8] m[12]
 *   m[ 1] m[ 5] m[ 9] m[13]
 *   m[ 2] m[ 6] m[10] m[14]
 *   m[ 3] m[ 7] m[11] m[15]
 */
#pragma once
#include <math.h>
#include <string.h>

typedef float Mat4[16];
typedef float Vec3[3];
typedef float Vec4[4];

/* ------------------------------------------------------------------ Vec3 */
static inline void  vec3_set(Vec3 v, float x, float y, float z)
    { v[0]=x; v[1]=y; v[2]=z; }
static inline void  vec3_copy(Vec3 d, const Vec3 s) { memcpy(d,s,12); }
static inline void  vec3_add(Vec3 r, const Vec3 a, const Vec3 b)
    { r[0]=a[0]+b[0]; r[1]=a[1]+b[1]; r[2]=a[2]+b[2]; }
static inline void  vec3_sub(Vec3 r, const Vec3 a, const Vec3 b)
    { r[0]=a[0]-b[0]; r[1]=a[1]-b[1]; r[2]=a[2]-b[2]; }
static inline void  vec3_scale(Vec3 r, const Vec3 v, float s)
    { r[0]=v[0]*s; r[1]=v[1]*s; r[2]=v[2]*s; }
static inline float vec3_dot(const Vec3 a, const Vec3 b)
    { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
static inline float vec3_len(const Vec3 v)
    { return sqrtf(vec3_dot(v,v)); }
static inline void  vec3_normalize(Vec3 v)
    { float l=vec3_len(v); if(l>1e-9f){v[0]/=l;v[1]/=l;v[2]/=l;} }
static inline void  vec3_cross(Vec3 r, const Vec3 a, const Vec3 b) {
    r[0]=a[1]*b[2]-a[2]*b[1];
    r[1]=a[2]*b[0]-a[0]*b[2];
    r[2]=a[0]*b[1]-a[1]*b[0];
}

/* ------------------------------------------------------------------ Vec4 */
static inline void mat4_mul_vec4(Vec4 r, const Mat4 m, const Vec4 v) {
    r[0]=m[0]*v[0]+m[4]*v[1]+m[ 8]*v[2]+m[12]*v[3];
    r[1]=m[1]*v[0]+m[5]*v[1]+m[ 9]*v[2]+m[13]*v[3];
    r[2]=m[2]*v[0]+m[6]*v[1]+m[10]*v[2]+m[14]*v[3];
    r[3]=m[3]*v[0]+m[7]*v[1]+m[11]*v[2]+m[15]*v[3];
}

/* ------------------------------------------------------------------ Mat4 */
static inline void mat4_identity(Mat4 m) {
    memset(m,0,64);
    m[0]=m[5]=m[10]=m[15]=1.0f;
}
static inline void mat4_copy(Mat4 d, const Mat4 s) { memcpy(d,s,64); }

/* r = a * b  (both column-major) */
static inline void mat4_mul(Mat4 r, const Mat4 a, const Mat4 b) {
    int c,rw,k;
    for(c=0;c<4;c++) for(rw=0;rw<4;rw++) {
        float s=0; for(k=0;k<4;k++) s+=a[k*4+rw]*b[c*4+k];
        r[c*4+rw]=s;
    }
}

/* perspective projection (fovY in degrees, aspect = width/height) */
static inline void mat4_perspective(Mat4 m, float fov_y_deg, float aspect,
                                    float near_p, float far_p) {
    float f=1.0f/tanf(fov_y_deg*0.5f*(float)(3.14159265358979323846/180.0));
    memset(m,0,64);
    m[0] = f/aspect;
    m[5] = f;
    m[10]= -(far_p+near_p)/(far_p-near_p);
    m[11]= -1.0f;
    m[14]= -(2.0f*far_p*near_p)/(far_p-near_p);
}

/* lookAt view matrix */
static inline void mat4_lookAt(Mat4 m,
                                const Vec3 eye, const Vec3 center,
                                const Vec3 world_up) {
    Vec3 f,r,u;
    vec3_sub(f,center,eye);  vec3_normalize(f);
    vec3_cross(r,f,world_up); vec3_normalize(r);
    vec3_cross(u,r,f);
    /* Column-major storage of the rotation + translation view matrix */
    m[0]=r[0]; m[4]=r[1]; m[8] =r[2]; m[12]=-vec3_dot(r,eye);
    m[1]=u[0]; m[5]=u[1]; m[9] =u[2]; m[13]=-vec3_dot(u,eye);
    m[2]=-f[0];m[6]=-f[1];m[10]=-f[2]; m[14]= vec3_dot(f,eye);
    m[3]=0;    m[7]=0;    m[11]=0;     m[15]=1.0f;
}

/* Extract camera right/up from a view matrix (world-space direction vectors) */
static inline void mat4_get_right(const Mat4 m, Vec3 r)
    { r[0]=m[0]; r[1]=m[4]; r[2]=m[8]; }
static inline void mat4_get_up   (const Mat4 m, Vec3 u)
    { u[0]=m[1]; u[1]=m[5]; u[2]=m[9]; }
static inline void mat4_get_fwd  (const Mat4 m, Vec3 f)
    { f[0]=-m[2]; f[1]=-m[6]; f[2]=-m[10]; }

/* Strip translation from a view matrix (for skybox rotation-only rendering) */
static inline void mat4_strip_translation(Mat4 dst, const Mat4 src) {
    mat4_copy(dst, src);
    dst[12]=dst[13]=dst[14]=0.0f;
}

/*
 * Project a world-space point to screen space.
 * vp      = projection * view  (or any MVP)
 * vp_w/h  = viewport width/height (pixels)
 * sx/sy   = output screen pixels  (y=0 at bottom, OpenGL convention)
 * Returns 0 if point is behind the camera.
 */
static inline int mat4_project(const Mat4 vp,
                                float wx, float wy, float wz,
                                int vp_w, int vp_h,
                                float *sx, float *sy) {
    float cx=vp[0]*wx+vp[4]*wy+vp[8] *wz+vp[12];
    float cy=vp[1]*wx+vp[5]*wy+vp[9] *wz+vp[13];
    float cw=vp[3]*wx+vp[7]*wy+vp[11]*wz+vp[15];
    if(cw<=0.0f) return 0;
    *sx=(cx/cw+1.0f)*0.5f*(float)vp_w;
    *sy=(cy/cw+1.0f)*0.5f*(float)vp_h;
    return 1;
}
