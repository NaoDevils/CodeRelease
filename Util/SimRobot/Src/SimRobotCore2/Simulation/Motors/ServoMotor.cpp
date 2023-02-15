/**
* @file Simulation/Motors/ServoMotor.cpp
* Implementation of class ServoMotor
* @author Colin Graf
*/

#include <cmath>

#include "Simulation/Motors/ServoMotor.h"
#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "Simulation/Actuators/Joint.h"
#include "Simulation/Axis.h"
#include "CoreModule.h"
#include "Platform/Assert.h"
#include "Tools/Math.h"

ServoMotor::ServoMotor() : maxVelocity(0), maxForce(0), stiffness(100)
{
  Simulation::simulation->scene->actuators.push_back(this);

  positionSensor.sensorType = SimRobotCore2::SensorPort::floatSensor;
  positionSensor.dimensions.push_back(1);
}

void ServoMotor::create(Joint* joint)
{
  ASSERT(dJointGetType(joint->joint) == dJointTypeHinge || dJointGetType(joint->joint) == dJointTypeSlider);
  this->joint = joint;
  positionSensor.servoMotor = this;
  if (dJointGetType(joint->joint) == dJointTypeHinge)
  {
    dJointSetHingeParam(joint->joint, dParamFMax, maxForce * (stiffness / 100.f));
    lastPos = static_cast<float>(dJointGetHingeAngle(joint->joint));
  }
  else
    dJointSetSliderParam(joint->joint, dParamFMax, maxForce * (stiffness / 100.f));
}

void ServoMotor::act()
{
  float currentPos = (dJointGetType(joint->joint) == dJointTypeHinge
          ? static_cast<float>(dJointGetHingeAngle(joint->joint))
          : static_cast<float>(dJointGetSliderPosition(joint->joint)) + (joint->axis->deflection ? joint->axis->deflection->offset : 0.f));

  if (dJointGetType(joint->joint) == dJointTypeHinge)
  {
    const float diff = normalize(currentPos - normalize(lastPos));
    currentPos = lastPos + diff;
    lastPos = currentPos;
  }

  float setpoint = this->setpoint;
  float velocity = maxVelocity;
  const float newVel = controller.getOutput(currentPos, setpoint);
  if (std::abs(newVel) < maxVelocity)
    velocity = newVel;
  else if (setpoint - currentPos < 0.f)
    velocity = -maxVelocity;


  float force = maxForce * (stiffness / 100.f);
  if (dJointGetType(joint->joint) == dJointTypeHinge)
  {
    dJointSetHingeParam(joint->joint, dParamFMax, force);
    dJointSetHingeParam(joint->joint, dParamVel, velocity);
  }
  else
  {
    dJointSetSliderParam(joint->joint, dParamFMax, force);
    dJointSetSliderParam(joint->joint, dParamVel, velocity);
  }
}

float ServoMotor::Controller::getOutput(float currentPos, float setpoint)
{
  const float deltaTime = Simulation::simulation->scene->stepLength;
  const float error = setpoint - currentPos;
  errorSum += error * deltaTime;
  float p_portion = error;
  float i_portion = errorSum;
  float d_portion = (error - lastError) / deltaTime;
  float result = p * p_portion + i * i_portion + d * d_portion;
  lastError = error;
  return result;
}

void ServoMotor::setValue(float value)
{
  setpoint = value;
  Axis::Deflection* deflection = joint->axis->deflection;
  if (deflection)
  {
    if (setpoint > deflection->max)
      setpoint = deflection->max;
    else if (setpoint < deflection->min)
      setpoint = deflection->min;
  }
}

void ServoMotor::setStiffness(int value)
{
  if (value != -1)
  {
    stiffness = value;
    if (stiffness > 100)
      stiffness = 100;
    else if (stiffness < 20)
      stiffness = 20;
  }
}

bool ServoMotor::getMinAndMax(float& min, float& max) const
{
  Axis::Deflection* deflection = joint->axis->deflection;
  if (deflection)
  {
    min = deflection->min;
    max = deflection->max;
    return true;
  }
  return false;
}

void ServoMotor::PositionSensor::updateValue()
{
  data.floatValue = (dJointGetType(servoMotor->joint->joint) == dJointTypeHinge
          ? static_cast<float>(dJointGetHingeAngle(servoMotor->joint->joint))
          : static_cast<float>(dJointGetSliderPosition(servoMotor->joint->joint)) + (servoMotor->joint->axis->deflection ? servoMotor->joint->axis->deflection->offset : 0.f));
  if (dJointGetType(servoMotor->joint->joint) == dJointTypeHinge)
  {
    const float diff = normalize(data.floatValue - normalize(servoMotor->lastPos));
    data.floatValue = servoMotor->lastPos + diff;
  }
}

bool ServoMotor::PositionSensor::getMinAndMax(float& min, float& max) const
{
  Axis::Deflection* deflection = servoMotor->joint->axis->deflection;
  if (deflection)
  {
    min = deflection->min;
    max = deflection->max;
    return true;
  }
  return false;
}

void ServoMotor::registerObjects()
{
  if (dJointGetType(joint->joint) == dJointTypeHinge)
    positionSensor.unit = unit = QString::fromUtf8("°");
  else
    positionSensor.unit = unit = "m";
  positionSensor.fullName = joint->fullName + ".position";
  fullName = joint->fullName + ".position";

  CoreModule::application->registerObject(*CoreModule::module, positionSensor, joint);
  CoreModule::application->registerObject(*CoreModule::module, *this, joint);
}
