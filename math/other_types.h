// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: definition of other types such as int3 and int2
// =============================================================================

#ifndef OTHER_TYPES_H
#define OTHER_TYPES_H
#define S2 _make_short2
#define I3 _make_int3
#define I2 _make_int2

typedef unsigned int uint;

struct bool2 {
  bool x, y;
  bool2() : x(0), y(0) {}
  bool2(bool a, bool b) : x(a), y(b) {}
};


#endif
