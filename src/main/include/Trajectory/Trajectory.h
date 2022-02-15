#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Splines/CatmullRom.h"

template<typename SplineType>
class Trajectory {
 public:

  Trajectory(std::string name = "<Trajectory>") : _name(name) {}
  Trajectory(std::vector<Splines::Waypoint> waypoints, std::string name = "<Trajectory>") : _name(name) {
    _trajectory.points = waypoints;
  }

  void push_back(Splines::Waypoint waypoint) {
    _trajectory.points.push_back(waypoint);
  }

  void push_back(std::vector<Splines::Waypoint> waypoints) {
    for (auto point : waypoints) {
      push_back(point);
    }
  }

  void pop_back() {
    _trajectory.points.pop_back();
  }

  void build(double stepSize = 0.0001) {
    _sType.setStepSize(stepSize);
    if (_sType.calculateSpline(_trajectory) == -1) {
      std::cout << "Error while calculating spline in " << _name << std::endl;
    }
  }

  /**
   * @brief Get the Angle along the spline using distance traveled
   * 
   * @param dist 
   * @param rads 
   * @return double 
   */
  double getAngle(double dist, bool rads = false) {
    double t = _sType.getDist2t(dist, _trajectory);
    if (rads) {
      return _sType.getSplineAngleRad(t, _trajectory);
    } else {
      return _sType.getSplineAngleDeg(t, _trajectory);
    }
  }

  /**
   * @brief Get the Coords along the spline using distance traveled
   * 
   * @param dist 
   * @return Splines::Waypoint 
   */
  Splines::Waypoint getCoords(double dist) {
    double t = _sType.getDist2t(dist, _trajectory);
    return _sType.getSplinePoint(t, _trajectory).waypoint;
  }

  Splines::Spline &getRawTrajectory() { 
    return _trajectory; 
  }

  SplineType getType() {
    return _sType;
  }

  void print() {
    std::cout << "-- Trajectory: " << _name << " --" << std::endl;
    std::cout << "|  Type: " << typeid(SplineType).name() << std::endl;
    std::cout << "   |  N Points: " << _trajectory.points.size() << std::endl;
    std::cout << "   |  Coords: ";
    for (auto point : _trajectory.points) {
      std::cout << "{" << point.x << "," << point.y << "}, ";
    }
    std::cout << std::endl;

    std::cout << "   |  Segments: " << _trajectory.segmentNum << std::endl;
    std::cout << "   |  Length: " << _trajectory.totalLength << std::endl;
  }


  void simulate(double distanceStepSize = 0.1) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "-- Simulating Moving Along Trajectory " << _name << " -- " << std::endl;
    for (double step = 0; step < _trajectory.totalLength; step += distanceStepSize) {
      Splines::Waypoint position = getCoords(step);
      double angle = getAngle(step);
      std::cout << "Distance: " << step << " | ";
      std::cout << "Coords: {" << position.x << "," << position.y << "} | ";
      std::cout << "Angle: " << angle << std::endl;
    }
  }

  /**
   * Used for wrapping around a range. E.g, if current angle is 175 and goal is -180. Both 180 and -180 are the same.
   * So don't spin the robot all the way around. instead just go to 180 degrees
   */
  double wrap(double val, double range = 0) {
    if (range > 0) {
      val = std::fmod(val, range);
      if (std::abs(val) > (range/2.0)) {
        return (val > 0) ? val - range : val + range;
      }
    }

    return val;
  }

  /**
   * @brief At Waypoint,
   * @param waypoint add waypoint number (object relative, e.g waypoint 0 is where the object starts from)
   * @param distande current object distance along trajectory
   * @param continous return true just once when it has reached/passed the waypoint
   * Returns true if object is at waypoint, or if set to do so, return true if object has passed waypoint
   */
  bool atWaypoint(int waypoint, double distance, bool once = false) {
    if (waypoint < 0 || waypoint > (int)_trajectory.points.size()-1) {
      std::cout << "At Waypoint <" << waypoint << "> Out of Scope" << std::endl;
    } else {
      auto &point = _trajectory.points[waypoint];

      bool passedPoint = distance >= point.totalLength ? true : false;
      if (!point.complete) {
        point.complete = passedPoint;
        return point.complete;
      } else {
        return once ? false : point.complete;
      }
    }

    return false;
  }

 private:
  Splines::Spline _trajectory;
  SplineType _sType;

  std::string _name;
};

#endif