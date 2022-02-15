#ifndef CONTROL_THEORY_PID_H
#define CONTROL_THEORY_PID_H

#include <string>
#include <cmath>

namespace PID {

  /**
   * @brief Gains for PID controller
   * 
   */
  class PIDGains {
   public:
    PIDGains(std::string name, double kP = 0, double kI = 0, double kD = 0, double kF = 0) : _name(name), _kP(kP), _kI(kI), _kD(kD), _kF(kF) {}
    PIDGains(const PIDGains &other) : PIDGains(other._name, other._kP, other._kI, other._kD, other._kF) {}

    void scheduleGains(double kP, double kI, double kD, double kF = 0) {
      _kP = kP;
      _kI = kI;
      _kD = kD;
      _kF = kF;
    }

    double GetkP() {
      return _kP;
    }

    double GetkI() {
      return _kI;
    }

    double GetkD() {
      return _kD;
    }

    double GetkF() {
      return _kF;
    }


    std::string getName() {
      return _name;
    }

   private:
    std::string _name;
    double _kP, _kI, _kD, _kF;
  };



  class PIDController {
   public:
    PIDController(PIDGains &gains, double setpoint = 0) : _gains(gains) {
      setSetpoint(setpoint, true);
    }

    void setSetpoint(double setpoint, bool reset = true) {
      if (reset) resetValues();
      _setpoint = setpoint;
      if (_threshAvgSet == false) {
        _threshAvgPos = setpoint * 0.05;
        _threshAvgVel = setpoint * 0.05;
      }
    }

    void setIZone(double threshIZone) {
      _threshIZone = threshIZone;
    }

    void setIsDoneThreshold(double threshAvgPos, double threshAvgVel) {
      _threshAvgPos = threshAvgPos;
      _threshAvgVel = threshAvgVel;
      _threshAvgSet = true;
    }


    double getSetpoint() {
      return _setpoint;
    }

    void setWrap(double range) {
      _wrapRange = range;
    }

    double calculate(double processingVariable, double dt, double feedforward = 0.0) {
      double error = wrap(_setpoint - processingVariable);

      if (_threshIZone > 0 && std::abs(error) > _threshIZone) _integral = 0; // I zone
      else _integral += error * dt; // Calc I
      _derivative = dt > 0 ? (error - _lastError) / dt : 0; // Calc D

      double output = _gains.GetkP() * error + _gains.GetkI() * _integral + _gains.GetkD() * _derivative + _gains.GetkF() * feedforward;
      _lastError = error;
      
      return output;
    }

   protected:
    void resetValues() {
      _integral = 0;
      _derivative = 0;
      _lastError = 0;
    }

    PIDGains &getGains() {
      return _gains;
    }

   private:
    PIDGains &_gains;

    double wrap(double val) {
      if (_wrapRange > 0) {
        val = std::fmod(val, _wrapRange);
        if (std::abs(val) > (_wrapRange / 2.0)) {
          return (val > 0) ? val - _wrapRange : val + _wrapRange;
        }
      }

      return val;
    }

    double _setpoint;

    double _integral;
    double _derivative;
    double _lastError;

    double _threshIZone = -1;
    double _threshAvgPos = -1, _threshAvgVel = -1;
    double _wrapRange = -1;
    bool _threshAvgSet = false; // Used to check if _threshAvg has already been manually set.
  };
}

#endif