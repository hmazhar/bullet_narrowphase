#ifndef SUPPORT_H
#define SUPPORT_H

#include "../narrowphase/data_structures.h"

inline real3 GetSupportPoint_Sphere(const real3& B, const real3& n) {
  // real3 b = real3(B.x);
  // return b * b * n / length(b * n);
  // the ellipsoid support function provides a cleaner solution for some reason
  return B.x * n;
}
inline real3 GetSupportPoint_Triangle(const real3& A, const real3& B, const real3& C, const real3& n) {
  real dist = dot(A, n);
  real3 point = A;

  if (dot(B, n) > dist) {
    dist = dot(B, n);
    point = B;
  }

  if (dot(C, n) > dist) {
    dist = dot(C, n);
    point = C;
  }

  return point;
}
inline real3 GetSupportPoint_Box(const real3& B, const real3& n) {
  real3 result = R3(0, 0, 0);
  result.x = sign(n.x) * B.x;
  result.y = sign(n.y) * B.y;
  result.z = sign(n.z) * B.z;
  return result;
}
inline real3 GetSupportPoint_Ellipsoid(const real3& B, const real3& n) {

  real3 normal = n;
  real3 result = B * B * normal / length(B * normal);
  // cout << result.x << " " << result.y << " " << result.z<<endl;
  return result;

  //
  //  real3 norm=normalize(n);
  //  real3 dim=(norm*norm)/(B*B);
  //  real k = sqrt(1/(dim.x+dim.y+dim.z));
  //  return k*norm;
}

inline real3 GetSupportPoint_Cylinder(const real3& B, const real3& n) {
  //   real3 u = R3(0, 1, 0);
  //   real3 w = n - (dot(u, n)) * u;
  //   real3 result;
  //
  //   if (length(w) != 0) {
  //      result = sign(dot(u, n)) * B.y * u + B.x * normalize(w);
  //   } else {
  //      result = sign(dot(u, n)) * B.y * u;
  //   }
  // return result;
  real s = sqrt(n.x * n.x + n.z * n.z);
  real3 tmp;
  if (s != 0) {
    tmp.x = n.x * B.x / s;
    tmp.y = n.y < 0.0 ? -B.y : B.y;
    tmp.z = n.z * B.x / s;
  } else {
    tmp.x = B.x;
    tmp.y = n.y < 0.0 ? -B.y : B.y;
    tmp.z = 0;
  }
  return tmp;
}
inline real3 GetSupportPoint_Plane(const real3& B, const real3& n) {
  real3 result = B;

  if (n.x < 0)
    result.x = -result.x;

  if (n.y < 0)
    result.y = -result.y;

  return result;
}
inline real3 GetSupportPoint_Cone(const real3& B, const real3& n) {
  real radius = B.x;
  real height = B.y;

  real m_sinAngle = (radius / sqrt(radius * radius + height * height));

  if (n.y > length(n) * m_sinAngle) {
    return R3(0, height / 2.0, 0);
  } else {
    real s = sqrt(n.x * n.x + n.z * n.z);
    if (s > 1e-9) {
      real3 tmp;
      tmp.x = n.x * B.x / s;
      tmp.y = -height / 2.0;
      tmp.z = n.z * B.z / s;
      return tmp;
    } else {
      real3 tmp;
      tmp.x = 0;
      tmp.y = -height / 2.0;
      tmp.z = 0;
      return tmp;
    }
  }
}
inline real3 GetSupportPoint_Seg(const real3& B, const real3& n) {
  real3 result = R3(0, 0, 0);
  result.x = sign(n.x) * B.x;

  return result;
}
inline real3 GetSupportPoint_Capsule(const real3& B, const real3& n) { return GetSupportPoint_Seg(B, n) + GetSupportPoint_Sphere(R3(B.y), n); }

inline real3 GetSupportPoint_Disk(const real3& B, const real3& n) {
  real3 n2 = R3(n.x, n.y, 0);
  n2 = normalize(n2);

  real3 result = B.x * n2;

  return result;
}

inline real3 GetSupportPoint_Rect(const real3& B, const real3& n) {
  real3 result = R3(0, 0, 0);
  result.x = sign(n.x) * B.x;
  result.z = sign(n.z) * B.z;
  return result;
}

inline real3 GetSupportPoint_RoundedBox(const real3& B, const real3& C, const real3& n) { return GetSupportPoint_Box(B, n) + GetSupportPoint_Sphere(R3(C.x), n); }
inline real3 GetSupportPoint_RoundedCylinder(const real3& B, const real3& C, const real3& n) { return GetSupportPoint_Cylinder(B, n) + GetSupportPoint_Sphere(R3(C.x), n); }
inline real3 GetSupportPoint_RoundedCone(const real3& B, const real3& C, const real3& n) { return GetSupportPoint_Cone(B, n) + GetSupportPoint_Sphere(R3(C.x), n); }

inline real3 GetSupportPoint_Convex(const real3& B, const real3* convex_data, const real3& n) {
  real max_dot_p = -LARGE_REAL;
  real dot_p;
  real3 point = convex_data[int(B.y)];
  for (int i = B.y; i < B.y + B.x; i++) {
    dot_p = convex_data[i].dot(n);
    if (dot_p > max_dot_p) {
      max_dot_p = dot_p;
      point = convex_data[i];
    }
  }
  return point + n * B.z;
}

inline real3 GetCenter_Sphere() { return ZERO_VECTOR; }
inline real3 GetCenter_Triangle(const real3& A, const real3& B, const real3& C) { return R3((A.x + B.x + C.x) / 3.0f, (A.y + B.y + C.y) / 3.0f, (A.z + B.z + C.z) / 3.0f); }
inline real3 GetCenter_Box() { return ZERO_VECTOR; }
inline real3 GetCenter_Ellipsoid() { return ZERO_VECTOR; }
inline real3 GetCenter_Cylinder() { return ZERO_VECTOR; }
inline real3 GetCenter_Plane() { return ZERO_VECTOR; }
inline real3 GetCenter_Cone(const real3& B) { return ZERO_VECTOR; }

inline real3 SupportVert(const ConvexShape& Shape, const real3& n, const real& envelope) {
  real3 localSupport;

  switch (Shape.type) {
    case SPHERE:
      localSupport = GetSupportPoint_Sphere(Shape.B, n);
      break;
    case ELLIPSOID:
      localSupport = GetSupportPoint_Ellipsoid(Shape.B, n);
      break;
    case BOX:
      localSupport = GetSupportPoint_Box(Shape.B, n);
      break;
    case CYLINDER:
      localSupport = GetSupportPoint_Cylinder(Shape.B, n);
      break;
    case CONE:
      localSupport = GetSupportPoint_Cone(Shape.B, n);
      break;
    case CAPSULE:
      localSupport = GetSupportPoint_Capsule(Shape.B, n);
      break;
    case ROUNDEDBOX:
      localSupport = GetSupportPoint_RoundedBox(Shape.B, Shape.C, n);
      break;
    case ROUNDEDCYL:
      localSupport = GetSupportPoint_RoundedCylinder(Shape.B, Shape.C, n);
      break;
    case ROUNDEDCONE:
      localSupport = GetSupportPoint_RoundedCone(Shape.B, Shape.C, n);
      break;
  }
  // The collision margin is applied as a compound support.
  // A sphere with a radius equal to the collision envelope is swept around the
  // entire shape.
  localSupport += GetSupportPoint_Sphere(envelope, n);
  return localSupport;
}

inline real3 LocalSupportVert(const ConvexShape& Shape, const real3& n, const real& envelope) {
  real3 rotated_n = quatRotateT(n, Shape.R);
  return SupportVert(Shape, rotated_n, envelope);
}

inline real3 TransformSupportVert(const ConvexShape& Shape, const real3& n, const real& envelope) {
  real3 localSupport;

  switch (Shape.type) {
    case TRIANGLEMESH: {
      // Triangles are handles differently than other convex shapes but they
      // still have an envelope around them. Prior to this there was no way
      // to define envelopes for triangle meshes.
      // Note that even with this change, large penetrations might cause the
      // object to be pushed into the triangle mesh. If you need true
      // inside/outside handling please use a convex decomposition
      return GetSupportPoint_Triangle(Shape.A, Shape.B, Shape.C, n) + GetSupportPoint_Sphere(envelope, n);
      break;
    }
    default:
      localSupport = LocalSupportVert(Shape, n, envelope);
      break;
  }

  return TransformLocalToParent(Shape.A, Shape.R, localSupport);
}

inline void GetBoundingSphere(const ConvexShape& Shape,  real3& center, real& disc) {
  center = real3(0);
  switch (Shape.type) {
    case SPHERE:
      disc = Shape.B.x;
      break;
    case ELLIPSOID:
      disc = max3(Shape.B);
      break;
    case BOX:
      disc = length(Shape.B);
      break;
    case CYLINDER:
      disc = sqrt(Shape.B.x * Shape.B.x + Shape.B.y * Shape.B.y);
      break;
    case CONE:
      disc = sqrt(Shape.B.x * Shape.B.x + Shape.B.y * Shape.B.y);
      break;
    case CAPSULE:
      disc = Shape.B.x + Shape.B.y;
      break;
    case ROUNDEDBOX:
      disc = length(Shape.B) + Shape.C.x;
      break;
    case ROUNDEDCYL:
      disc = sqrt(Shape.B.x * Shape.B.x + Shape.B.y * Shape.B.y) + Shape.C.x;
      break;
    case ROUNDEDCONE:
      disc = sqrt(Shape.B.x * Shape.B.x + Shape.B.y * Shape.B.y) + Shape.C.x;
      break;
  }
}

inline real GetAngularMotionDisc(const ConvexShape& Shape) {
  real3 center;
  real disc;
  GetBoundingSphere(Shape, center, disc);
  disc += (center).length();
  return disc;
}

#endif
