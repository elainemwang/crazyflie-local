// 0x 0y 1x 1y
// read from file, do math, write to file
static void vec_cross_product(const vec3d a, const vec3d b, vec3d res) {
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
}

static float vec_dot(const vec3d a, const vec3d b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vec_add(const vec3d a, const vec3d b, vec3d r) {
  r[0] = a[0] + b[0];
  r[1] = a[1] + b[1];
  r[2] = a[2] + b[2];
}

static float vec_length(const vec3d vec) {
    float pow = vec_dot(vec, vec);

    float res;
    arm_sqrt_f32(pow, &res);
    return res;
}
