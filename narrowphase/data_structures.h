
#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include "../math/collision_math.h"

#define MANIFOLD_SIZE 3

enum ShapeType {
  SPHERE,
  ELLIPSOID,
  BOX,
  CYLINDER,
  CONVEXHULL,
  TRIANGLEMESH,
  BARREL,
  CAPSULE,        // Currently implemented in parallel only
  CONE,           // Currently implemented in parallel only
  ROUNDEDBOX,     // Currently implemented in parallel only
  ROUNDEDCYL,     // Currently implemented in parallel only
  ROUNDEDCONE,    // Currently implemented in parallel only
  CONVEX,         // Currently implemented in parallel only
  FLUID           // Currently implemented in parallel only
};

struct ConvexShape {
  ShapeType type;    // type of shape
  real3 A;           // location
  real3 B;           // dimensions
  real3 C;           // extra
  quaternion R;      // rotation
  real margin;
  ConvexShape():margin(0.04){}
};

struct ContactPoint {
  real3 pointA, pointB, normal;
  real depth;
  ContactPoint(){}
  ContactPoint(real3 pa, real3 pb, real3 norm, real d) {
    pointA = pa;
    pointB = pb;
    normal = norm;
    depth = d;
  }
};

struct ContactManifold {
  ContactPoint points[MANIFOLD_SIZE];

  unsigned int num_contact_points;
  ContactManifold() { num_contact_points = 0; }

  int getCacheEntry(ContactPoint& newPoint) {

    real shortestDist = ZERO_EPSILON * 2;    // TODO: SHOULD THIS BE SOMETHIGN ELSE
    int size = num_contact_points;
    int nearestPoint = -1;
    for (int i = 0; i < size; i++) {
      const ContactPoint& mp = points[i];

      real3 diffA = mp.pointA - newPoint.pointA;
      const real distToManiPoint = diffA.dot(diffA);
      if (distToManiPoint < shortestDist) {
        shortestDist = distToManiPoint;
        nearestPoint = i;
      }
    }
    return nearestPoint;
  }

  void replaceContactPoint(ContactPoint& newPt, int& insertIndex) { points[insertIndex] = newPt; }
  int addManifoldPoint(ContactPoint& newPt) {

    if (num_contact_points == MANIFOLD_SIZE) {
      points[0] = newPt;
      return 0;
    } else {
      points[num_contact_points] = newPt;
      num_contact_points++;
      return num_contact_points - 1;
    }
  }

  void addContactPoint(const ConvexShape& shapeA, const ConvexShape& shapeB, const real3& normalOnBInWorld, const real3& pointInWorld, const real& depth) {

    real3 pointA = pointInWorld + normalOnBInWorld * depth;
    real3 localA = TransformParentToLocal(shapeA.A, shapeA.R, pointA);
    real3 localB = TransformParentToLocal(shapeB.A, shapeB.R, pointInWorld);

    ContactPoint newPt(localA, localB, normalOnBInWorld, depth);
    newPt.pointA = pointA;
    newPt.pointB = pointInWorld;
    int insertIndex = getCacheEntry(newPt);
    if (insertIndex >= 0) {
      replaceContactPoint(newPt, insertIndex);
    } else {
      insertIndex = addManifoldPoint(newPt);
    }
  }
};

#endif
