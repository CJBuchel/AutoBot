#ifndef CONTROL_THEORY_H
#define CONTROL_THEORY_H

#include "PID.h"
// #include "Trajectory/Trajectory.h"

/**
 * @brief Abstracted theory for movement
 * The abstracted/simplified goal of movement is to get from A to B,
 * where that point of A and B has been processed via some other means (trajectories).
 * 
 * Plan the control of movement getting from A through to B
 */
class ControlTheory {
 public:
  enum class State {
    kIDLE = 0,
    kMOVING
  };

  class Target {
   public:
    Target(std::string name, double initialTarget, PID::PIDGains gains) : _name(name) {
      _gains = new PID::PIDGains(gains);
      _controller = new PID::PIDController(*_gains, initialTarget);
    }

    PID::PIDGains *getGains() {
      return _gains;
    }

    PID::PIDController *getController() {
      return _controller;
    }

    void setMarginOfError(double error) {
      _marginOfError = error;
    }

    void setFeedForward(double ff) {
      _feedforward = ff;
    }

    void setCurrentPosition(double current) {
      _currentPosition = current;
    }

    double getOutput() {
      return _output;
    }

    void calculate(double dt) {
      _output = _controller->calculate(_currentPosition, dt, _feedforward);
    }
  
   private:
    std::string _name;
    PID::PIDGains *_gains;
    PID::PIDController *_controller;

    double _marginOfError = 0;
    double _output = 0;
    double _currentPosition = 0;
    double _feedforward = 0;
  };

  ControlTheory(double maxSpeed, double maxAcceleration) {
    _maxSpeed = maxSpeed;
    _maxAcceleration = maxAcceleration;
  }

  void createTarget(Target target) {
    _targets.push_back(target);
  }

  void createTargets(std::vector<Target> targets) {
    for (auto &target : targets) {
      createTarget(target);
    }
  }

  std::vector<Target> &getTargets() {
    return _targets;
  }

  Target &getTarget(int index) {
    return _targets[index];
  }

  void start() {
    _state = State::kMOVING;
  }

  void stop() {
    _state = State::kIDLE;
  }

  /**
   * @brief Get the Completion of the motion
   * 
   * @return true 
   * @return false 
   */
  bool getComplete() {
    return _complete;
  }

  /**
   * @brief Reset the motion
   * Values are all dropped to zero and could begin again
   */
  void reset() {
    _complete = false;
  }

  /**
   * @brief Set the Margin Of Error for the object
   * Being at the target can never be perfect in practice,
   * a margin of error helps to specify when the motion can be complete
   * if it's within the margin of error;
   * (+- error)
   * @param error
   */
  void setMarginOfError(double error) {
    _marginOfError = error;
  }

  void updateTargetMovement(double dt) {
    for (auto &target : _targets) {
      target.calculate(dt);
    }
  }

  void update(double dt) {
    switch (_state) {
      case State::kIDLE:
        break;

      case State::kMOVING:
        updateTargetMovement(dt);
        break;

      default:
        _state = State::kIDLE;
        break;
    }
  }

 private:
  State _state{ State::kIDLE };
  bool _complete = false;
  double _marginOfError = 0;

  double _maxSpeed;
  double _maxAcceleration;

  std::vector<Target> _targets;
};

#endif