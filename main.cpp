#include <iostream>
#include <fstream>
#include <string>
using namespace std;
#include "narrowphase/data_structures.h"
#include "narrowphase/gjk_epa.h"
#include "collision/bullet/btBulletCollisionCommon.h"

void BulletTest() {

  btCollisionConfiguration* bt_collision_configuration;
  btCollisionDispatcher* bt_dispatcher;
  btBroadphaseInterface* bt_broadphase;
  btCollisionWorld* bt_collision_world;

  double scene_size = 500;
  unsigned int max_objects = 16000;

  bt_collision_configuration = new btDefaultCollisionConfiguration();
  bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);

  btScalar sscene_size = (btScalar)scene_size;
  btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
  btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);
  // This is one type of broadphase, bullet has others that might be faster depending on the application
  bt_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, max_objects, 0, true);    // true for disabling raycast accelerator

  bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);
  // Create two collision objects
  btCollisionObject* sphere_A = new btCollisionObject();
  btCollisionObject* sphere_B = new btCollisionObject();
  // Move each to a specific location
  sphere_A->getWorldTransform().setOrigin(btVector3((btScalar)2, (btScalar)1.5, (btScalar)0));
  sphere_B->getWorldTransform().setOrigin(btVector3((btScalar)2, (btScalar)0, (btScalar)0));
  // Create a sphere with a radius of 1
  btCylinderShape sphere_shape(btVector3(1, 1, 1));
  // Set the shape of each collision object
  sphere_A->setCollisionShape(&sphere_shape);
  sphere_B->setCollisionShape(&sphere_shape);
  // Add the collision objects to our collision world
  bt_collision_world->addCollisionObject(sphere_A);
  bt_collision_world->addCollisionObject(sphere_B);

  // Perform collision detection
  bt_collision_world->performDiscreteCollisionDetection();

  int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
  // For each contact manifold
  for (int i = 0; i < numManifolds; i++) {
    btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
    btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
    btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
    contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());
    int numContacts = contactManifold->getNumContacts();
    // For each contact point in that manifold
    for (int j = 0; j < numContacts; j++) {
      // Get the contact information
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      btVector3 ptA = pt.getPositionWorldOnA();
      btVector3 ptB = pt.getPositionWorldOnB();
      double ptdist = pt.getDistance();

      real3 PA = real3(ptA.x(), ptA.y(), ptA.z());
      real3 PB = real3(ptB.x(), ptB.y(), ptB.z());
      real3 N = real3(pt.m_normalWorldOnB.x(), pt.m_normalWorldOnB.y(), pt.m_normalWorldOnB.z());
      std::cout << N << PA << PB << ptdist << std::endl;
    }
  }
}

int main(int argc, char* argv[]) {
  real envelope = .04;

  if (argc > 1) {

    BulletTest();
  } else {
    ContactManifold manifold;
    real3 p, n(0, 0, 0);
    real d = 0;
    ConvexShape shapeA, shapeB;
    shapeA.type = CYLINDER;
    shapeA.A = real3(2, 1.5, 0);
    shapeA.B = real3(1, 1, 1);
    shapeA.C = real3(0);
    shapeA.R = real4(1, 0, 0, 0);

    shapeB.type = CYLINDER;
    shapeB.A = real3(2, 0, 0);
    shapeB.B = real3(1, 1, 1);
    shapeB.C = real3(0);
    shapeB.R = real4(1, 0, 0, 0);
    real3 sep_axis;

    Collide(shapeA, shapeB, manifold, sep_axis, envelope);

    for (int i = 0; i < manifold.num_contact_points; i++) {
      std::cout << manifold.points[i].normal << manifold.points[i].pointA << manifold.points[i].pointB << manifold.points[i].depth << std::endl;
    }
    ////    if () {
    ////      cout << sep_axis << endl;
    ////      PerturbedCollide(shapeA, shapeB, manifold, envelope, sep_axis);
    ////    }
  }
  return 0;
}
