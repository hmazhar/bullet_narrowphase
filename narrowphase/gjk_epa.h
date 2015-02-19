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
#define SIMD_2_PI 2 * SIMD_PI



template <class T>
inline void PlaneSpace1(const T& n, T& p, T& q) {
  if (fabs(n.z) > SIMDSQRT12) {
    // choose p in y-z plane
    real a = n.y * n.y + n.z * n.z;
    real k = 1.0 / sqrt(a);
    p.x = 0;
    p.y = -n.z * k;
    p.z = n.y * k;
    // set q = n x p
    q.x = a * k;
    q.y = -n.x * p.z;
    q.z = n.x * p.y;
  } else {
    // choose p in x-y plane
    real a = n.x * n.x + n.y * n.y;
    real k = 1.0 / sqrt(a);
    p.x = -n.y * k;
    p.y = n.x * k;
    p.z = 0;
    // set q = n x p
    q.x = -n.z * p.y;
    q.y = n.z * p.x;
    q.z = a * k;
  }
}

bool Distance(const ConvexShape& shape0, const ConvexShape& shape1, const real3& guess, sResults& results);

bool Penetration(const ConvexShape& shape0, const ConvexShape& shape1, const real3& guess, sResults& results);

bool Collide(const ConvexShape& shape0, const ConvexShape& shape1, ContactManifold& manifold, real3 & m_cachedSeparatingAxis, real margin);
void PerturbedCollide(const ConvexShape& shapeA, const ConvexShape& shapeB, ContactManifold& manifold, real margin, real3 m_cachedSeparatingAxis);
bool calcPenDepth(const ConvexShape& shape0, const ConvexShape& shape1, sResults& results);

#endif
