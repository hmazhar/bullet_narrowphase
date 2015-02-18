#include <iostream>
#include <fstream>
#include <string>
using namespace std;
#include "narrowphase/data_structures.h"
#include "narrowphase/gjk_epa.h"

int main(int argc, char *argv[]) {
  real envelope = 0;

  {
    ContactManifold manifold;
    cout << "  two ellipsoids touching perfectly" << endl;
    real3 p, n(0, 0, 0);
    real d = 0;
    ConvexShape shapeA, shapeB;
    shapeA.type = BOX;
    shapeA.A = real3(2, 2, 0);
    shapeA.B = real3(1, 1, 1);
    shapeA.C = real3(0);
    shapeA.R = real4(1, 1, 0, 0);

    shapeB.type = BOX;
    shapeB.A = real3(2, 0, 0);
    shapeB.B = real3(1, 1, 1);
    shapeB.C = real3(0);
    shapeB.R = real4(1, 0, 0, 0);

    Collide(shapeA, shapeB, manifold, envelope);
    Collide(shapeA, shapeB, manifold, envelope);
    Collide(shapeA, shapeB, manifold, envelope);

    for (int i = 0; i < manifold.num_contact_points; i++) {
      cout <<manifold.points[i].normal<<manifold.points[i].pointA<<manifold.points[i].pointB<<manifold.points[i].depth<<endl;
    }

  }

  return 0;
}
