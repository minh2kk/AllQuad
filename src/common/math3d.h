// AllQuad - 3D math helpers (vectors, quaternions)
// Frames: body = FRD (x forward, y right, z down), earth = NED.
#pragma once
#include <math.h>
#include <stdint.h>

// hot helpers of the 1-2 kHz flight loop: always inlined, so they run from the caller's memory
// (IRAM on the ESP32) instead of one out-of-line copy in flash, possibly compiled -Os
#define AQ_HOT __attribute__((always_inline))

namespace fc {

constexpr float PI_F = 3.14159265358979f;
constexpr float DEG2RAD = PI_F / 180.0f;
constexpr float RAD2DEG = 180.0f / PI_F;
constexpr float GRAVITY = 9.80665f;

inline float constrainf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float squaref(float v) { return v * v; }
inline AQ_HOT float wrapPi(float a) {
  while (a > PI_F) a -= 2.0f * PI_F;
  while (a < -PI_F) a += 2.0f * PI_F;
  return a;
}

struct Vec3 {
  float x = 0, y = 0, z = 0;
  Vec3() = default;
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
  Vec3 operator-() const { return {-x, -y, -z}; }
  Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
  Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
  Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
  float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
  float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
  float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  AQ_HOT float norm() const { return sqrtf(x * x + y * y + z * z); }
  Vec3 normalized() const {
    float n = norm();
    return n > 1e-9f ? (*this) * (1.0f / n) : Vec3();
  }
};

// Unit quaternion describing rotation body -> earth.
struct Quat {
  float w = 1, x = 0, y = 0, z = 0;
  Quat() = default;
  Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

  static Quat fromEuler(float roll, float pitch, float yaw) {
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
  }
  AQ_HOT Quat operator*(const Quat& o) const {
    return {w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
  }
  Quat conj() const { return {w, -x, -y, -z}; }
  AQ_HOT void normalize() {
    float n = sqrtf(w * w + x * x + y * y + z * z);
    if (n < 1e-9f) { w = 1; x = y = z = 0; return; }
    float k = 1.0f / n;
    w *= k; x *= k; y *= k; z *= k;
  }
  // body -> earth
  AQ_HOT Vec3 rotate(const Vec3& v) const {
    Vec3 u(x, y, z);
    Vec3 t = u.cross(v) * 2.0f;
    return v + t * w + u.cross(t);
  }
  // earth -> body
  AQ_HOT Vec3 rotateInv(const Vec3& v) const { return conj().rotate(v); }

  float roll() const { return atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)); }
  float pitch() const { return asinf(constrainf(2.0f * (w * y - z * x), -1.0f, 1.0f)); }
  AQ_HOT float yaw() const { return atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)); }
  // cos of angle between body z and earth z (1 = level)
  AQ_HOT float cosTilt() const { return 1.0f - 2.0f * (x * x + y * y); }
};

}  // namespace fc
