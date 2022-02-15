#include "TrajectoryUtilities.h"

struct Trajectories {
  Trajectory<Splines::CatmullRom> trajectory;
  void build() {
    trajectory.push_back({
      {0,1}, {1,1}, {3,3}, // forward
      {3,4}, {1,1}, {1,0} // back
    });
    trajectory.build(0.01);
  }
};