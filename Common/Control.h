#include "TrajectoryUtilities.h"

struct Trajectories {
  Trajectory<Splines::CatmullRom> trajectory;

  double maxLinearSpeed = 0.25; // percentage -> 0.25
  double maxTurnSpeed = 0.15; // percentage -> 0.15
  double rate = 0.075; // percentage per second

  double delta_pos = 0, current_pos = 0, last_pos = 0;

  double last_linearOutput = 0, last_angularOutput = 0;

  bool complete = false;
  bool scheduleGains = false;
  bool tuneMode = false;

  PID::PIDGains linearGains{"Linear Gains", 0.2, 0.01, 0};
  PID::PIDGains linearGainsFine{"Linear Gains Fine", 0,0,0};

  PID::PIDGains angleGains{"Angle Gains", 0.02, 0.5, 0.0001}; // 0.002, 0.5, 0.0001
  PID::PIDGains angleGainsFine{ "Fine Angle Gains", 0, 0, 0 };

  PID::PIDController linearController{linearGains};
  PID::PIDController angleController{angleGains};

  double constrain(double value, double low, double high) {
    return value < low ? low : value > high ? high : value;
  }

  double linearAcceleration(double targetSpeed, double lastSpeed, double rate, double maxSpeed, double dt) {
    double valueRate = (targetSpeed-lastSpeed); // delta speed
    if ( fabs((lastSpeed+valueRate)) > fabs((lastSpeed+(rate*dt))) ) {
      double desiredSpeed = lastSpeed+(rate*dt);
      return fabs(desiredSpeed) > maxSpeed ? constrain(desiredSpeed, -maxSpeed, maxSpeed) : lastSpeed+(rate*dt);
    } else {
      return targetSpeed;
    }
  }

  void build() {

    // Trajectory must start where robot starts (1,4)
    trajectory.push_back({
      {0,4}, // Start Ctrl Pnt

      {1,4}, {3,5}, // forward
      {3,6}, {1,4}, // back
      
      {1,3} // End Ctrl Pnt
    });

    trajectory.build(0.01);
    angleController.setWrap(180);
    linearController.setSetpoint(trajectory.getRawTrajectory().totalLength);

    // Set izones
    angleController.setIZone(10);
    linearController.setIZone(10);
  }

  void setScheduleGains(double distance, double gyro, double &dt) {
    if (scheduleGains) {
      if (distance > trajectory.getRawTrajectory().totalLength-0.5) {
        linearController.scheduleGains(linearGainsFine);
      } else {
        linearController.scheduleDefaultGains();
      }

      if (gyro > trajectory.getAngle(distance)-5 && gyro < trajectory.getAngle(distance)+5) {
        angleController.scheduleGains(angleGainsFine);
      } else {
        angleController.scheduleDefaultGains();
      }
    }
  }

  /**
   * @brief Tune PID Controllers
   * 
   * @return std::pair<double, double> 
   */
  std::pair<double, double> tunePID(double distance, double gyro, double dt, bool sim = false) {
    // Angular Ouptut
    angleController.setSetpoint(90);
    double angleOutput = angleController.calculate(gyro, dt);
    std::cout << "Goal: 90, Gyro: " << gyro << std::endl;
    return {0+angleOutput, 0-angleOutput};
  }


  std::pair<double, double> calculate(double distance, double gyro, double dt, bool sim = false) {
    current_pos = distance;
    delta_pos = current_pos - last_pos;
    double speed = delta_pos / dt;

    double leftPower = 0, rightPower = 0;

    setScheduleGains(distance, gyro, dt); // schedule new gains if allowed

    if (distance < trajectory.getRawTrajectory().totalLength) {
      

      // Linear Output
      double linearOutput = linearController.calculate(distance, dt);
      linearOutput = linearAcceleration(linearOutput, last_linearOutput, rate, maxLinearSpeed, dt);
      leftPower += linearOutput;
      rightPower += linearOutput;

      // Angular Ouptut
      angleController.setSetpoint(trajectory.getAngle(distance));
      double angleOutput = angleController.calculate(gyro, dt);
      // angleOutput = linearAcceleration(angleOutput, last_angularOutput, 1, maxTurnSpeed, dt);
      leftPower += angleOutput;
      rightPower -= angleOutput;

      // std::cout << "Goal Distance: " << trajectory.getRawTrajectory().totalLength << ", Distance: " << distance << ", Goal Gyro: " << trajectory.getAngle(distance) << ", Gyro: " << gyro << ", Coords: (" << trajectory.getCoords(distance).x << ", " << trajectory.getCoords(distance).y << ")\n";
      // std::cout << "Speed: " << speed << ", Delta Position: " << delta_pos << std::endl;


      last_linearOutput = linearOutput;
      last_angularOutput = angleOutput;
      last_pos = current_pos;
    } else {
      if (!complete) {
        std::cout << "\n\nAuto Complete\n\n";
        complete = true;
      }
    }

    return {leftPower, rightPower};
  }

  std::pair<double, double> follow(double distance, double gyro, double dt, bool sim = false) {
    std::pair<double, double> output;
    if (tuneMode) {
      output = tunePID(distance, gyro, dt, sim);
    } else {
      output = calculate(distance, gyro, dt, sim);
    }

    return {output.first, output.second};
  }
};