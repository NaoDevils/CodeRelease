#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_


/**
* @class PIDController
*
* A class for controlling the speed of angular motors
* combining three different controllers.
*/
template<class V = float> class PIDController
{
private:
  /** The constant factor for a proportional control part (gain) */
  V pControlFactor = V();
  /** The constant factor for an integral control part (reset) */
  V iControlFactor = V();
  /** The constant factor for a derivative control part (rate) */
  V dControlFactor = V();
  /** The time step length of a simulation control step*/
  V timeStep = V() + 1;
  /** The sum of errorValues */
  V errorSum = V();
  /** The error the previous pass */
  V previousError = V();

public:
  /** Constructor */
  PIDController() {}

  /**
   * Returns the controller output.
   * @param measuredValue The current value of the joint
   * @param setpoint point The desired value of the joint
   * @return The controller output
   */
  V getControllerOutput(V measuredValue, V setpoint)
  {
    V error = setpoint - measuredValue;
    V pControlValue = pControlFactor * error;
    errorSum += error;
    V iControlValue = iControlFactor * timeStep * errorSum;
    V dControlValue = (dControlFactor * (error - previousError)) / timeStep;
    previousError = error;
    return pControlValue + iControlValue + dControlValue;
  }

  /** 
   * Sets the time step length of a simulation control step
   * @param value the time step length
   */
  void setTimeStep(V value){timeStep = value;};

  /**
  * Updates the all control factors
  * @param p The new p value
  * @param i The new p value
  * @param d The new p value
  */
  void updateControlFactors(V p, V i, V d) {pControlFactor = p; iControlFactor = i; dControlFactor = d;}
};

#endif //PIDCONTROLLER_H_
