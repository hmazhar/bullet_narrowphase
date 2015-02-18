/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the
use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated
but is not required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
GJK-EPA collision solver by Nathanael Presson, 2008
*/

#ifndef GJK_EPA
#define GJK_EPA
#include "data_structures.h"

struct sResults {
  enum eStatus {
    Separated,   /* Shapes does not penetrate                                   */
    Penetrating, /* Shapes are penetrating                                  */
    GJK_Failed,  /* GJK phase fail, no big issue, shapes are probably just 'touching' */
    EPA_Failed   /* EPA phase fail, bigger problem, need to save parameters, and debug */
  } status;
  real3 witnesses[2];
  real3 normal;
  real distance;
};

#define SIMDSQRT12 0.7071067811865475244008443621048490
#define SIMD_PI 3.1415926535897932384626433832795028842
#define SIMD_2_PI 2*SIMD_PI
template <class T>
inline void PlaneSpace1(const T& n, T& p, T& q) {
  if (fabs(n[2]) > SIMDSQRT12) {
    // choose p in y-z plane
    real a = n[1] * n[1] + n[2] * n[2];
    real k = 1.0 / sqrt(a);
    p[0] = 0;
    p[1] = -n[2] * k;
    p[2] = n[1] * k;
    // set q = n x p
    q[0] = a * k;
    q[1] = -n[0] * p[2];
    q[2] = n[0] * p[1];
  } else {
    // choose p in x-y plane
    real a = n[0] * n[0] + n[1] * n[1];
    real k = 1.0 / sqrt(a);
    p[0] = -n[1] * k;
    p[1] = n[0] * k;
    p[2] = 0;
    // set q = n x p
    q[0] = -n[2] * p[1];
    q[1] = n[2] * p[0];
    q[2] = a * k;
  }
}

bool Distance(const ConvexShape& shape0, const ConvexShape& shape1, const real3& guess, sResults& results);

bool Penetration(const ConvexShape& shape0, const ConvexShape& shape1, const real3& guess, sResults& results);

bool Collide(const ConvexShape& shape0, const ConvexShape& shape1,ContactManifold & manifold, real margin);

bool calcPenDepth(const ConvexShape& shape0, const ConvexShape& shape1, sResults& results);

#endif
