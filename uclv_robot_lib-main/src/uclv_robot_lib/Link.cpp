/*

    Robot Class

    Copyright 2023 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "uclv_robot_lib/Link.hpp"
#include "uclv_robot_lib/exceptions.hpp"
#include <iostream>

namespace uclv::robot
{
/*======CONSTRUCTORS======*/

// Full Constructor
Link::Link(double a, double alpha, double d, double theta, double robot2dh_offset, bool robot2dh_flip,
           double Joint_Hard_limit_lower, double Joint_Hard_limit_higher, double Joint_Soft_limit_lower,
           double Joint_Soft_limit_higher, double hard_velocity_limit, double soft_velocity_limit,
           const std::string& name)
{
  _a = a;
  _alpha = alpha;
  _d = d;
  _theta = theta;
  _robot2dh_offset = robot2dh_offset;
  _robot2dh_flip = robot2dh_flip;
  setHardJointLimits(Joint_Hard_limit_lower, Joint_Hard_limit_higher);
  setSoftJointLimits(Joint_Soft_limit_lower, Joint_Soft_limit_higher);
  setHardVelocityLimit(hard_velocity_limit);
  setSoftVelocityLimit(soft_velocity_limit);
  _name = name;
}

Link::Link(double a, double alpha, double d, double theta, double robot2dh_offset, bool robot2dh_flip,
           double Joint_Hard_limit_lower, double Joint_Hard_limit_higher, double hard_velocity_limit,
           const std::string& name)
  : Link(a, alpha, d, theta, robot2dh_offset, robot2dh_flip, Joint_Hard_limit_lower, Joint_Hard_limit_higher,
         Joint_Hard_limit_lower, Joint_Hard_limit_higher, hard_velocity_limit, hard_velocity_limit, name)
{
}

/*======END CONSTRUCTORS======*/

//========Varie=======//

void Link::checkLowerHigher(double lower, double higher)
{
  if (lower > higher)
  {
    throw robot::runtime_error("[Link] Error in checkLowerHigher( double lower, double higher ): lower > higher ");
  }
}

Eigen::Isometry3d Link::A_internal(double theta, double d) const
{
  double sa = sin(_alpha);
  double ca = cos(_alpha);

  double st = sin(theta);
  double ct = cos(theta);

  // clang-format off

  return Eigen::Isometry3d(
    Eigen::Matrix4d{
      {  ct, -st * ca,  st * sa, _a * ct }, 
      {  st,  ct * ca, -ct * sa, _a * st }, 
      { 0.0,       sa,       ca,       d }, 
      { 0.0,      0.0,      0.0,     1.0 } 
      }
    );
  // clang-format on
}

//===================//

//========GETTERS==============//

/*
    return the link length
*/
double Link::getDH_a() const
{
  return _a;
}

/*
    return the link twist
*/
double Link::getDH_alpha() const
{
  return _alpha;
}

/*
    return the link offset
    Note:
        - For prismatic link this is the joint variable,
          in that case this function returns NaN
*/
double Link::getDH_d() const
{
  return _d;
}

/*
    return the link angle
    Note:
        - For revolute link this is the joint variable,
          in that case this function returns NaN
*/
double Link::getDH_theta() const
{
  return _theta;
}

/*
    Return the offset between the robot and DH convention
*/
double Link::getRobot2DH_offset() const
{
  return _robot2dh_offset;
}

/*
    Return the sign between robot and DH conventions
*/
bool Link::getRobot2DH_flip() const
{
  return _robot2dh_flip;
}

/*
    TODO
*/
Eigen::Vector2d Link::getSoftJointLimits() const
{
  return { _Joint_Soft_limit_lower, _Joint_Soft_limit_higher };
}

/*
    TODO
*/
Eigen::Vector2d Link::getHardJointLimits() const
{
  return { _Joint_Hard_limit_lower, _Joint_Hard_limit_higher };
}

/*
    TODO
*/
double Link::getSoftVelocityLimit() const
{
  return _soft_velocity_limit;
}

/*
    TODO
*/
double Link::getHardVelocityLimit() const
{
  return _hard_velocity_limit;
}

/*
    Return the Joint Name
*/
const std::string& Link::getName() const
{
  return _name;
}

//======END GETTERS===========//

//========SETTERS==============//

/*
    TODO
*/
void Link::setDH_a(double a)
{
  _a = a;
}

/*
    TODO
*/
void Link::setDH_alpha(double alpha)
{
  _alpha = alpha;
}

/*
    TODO
    ERROR IF LINK IS PRISMATIC
*/
void Link::setDH_d(double d)
{
  _d = d;
}

/*
    TODO
    ERROR IF LINK IS REVOLUTE
*/
void Link::setDH_theta(double theta)
{
  _theta = theta;
}

/*
    TODO
*/
void Link::setRobot2DH_offset(double offset)
{
  _robot2dh_offset = offset;
}

/*
    TODO
*/
void Link::setRobot2DH_flip(bool flip)
{
  _robot2dh_flip = flip;
}

/*
    TODO
*/
void Link::setSoftJointLimits(double lower, double higher)
{
  checkLowerHigher(lower, higher);
  _Joint_Soft_limit_lower = lower;
  _Joint_Soft_limit_higher = higher;
}

/*
    TODO
*/
void Link::setSoftJointLimits(const Eigen::Ref<Eigen::Vector2d>& limits)
{
  setSoftJointLimits(limits[0], limits[1]);
}

/*
    TODO
*/
void Link::setHardJointLimits(double lower, double higher)
{
  checkLowerHigher(lower, higher);
  _Joint_Hard_limit_lower = lower;
  _Joint_Hard_limit_higher = higher;
}

/*
    TODO
*/
void Link::setHardJointLimits(const Eigen::Ref<Eigen::Vector2d>& limits)
{
  setHardJointLimits(limits[0], limits[1]);
}

/*
    TODO
*/
void Link::setHardVelocityLimit(double velocity_limit)
{
  if (velocity_limit < 0.0)
  {
    throw robot::runtime_error("[Link] Error in setHardVelocityLimit( double velocity_limit): velocity_limit<0.0");
  }
  _hard_velocity_limit = velocity_limit;
}

/*
    TODO
*/
void Link::setSoftVelocityLimit(double velocity_limit)
{
  if (velocity_limit < 0.0)
  {
    throw robot::runtime_error("[Link] Error in setSoftVelocityLimit( double velocity_limit): velocity_limit<0.0");
  }
  _soft_velocity_limit = velocity_limit;
}

/*
    TODO
*/
void Link::setName(const std::string& name)
{
  _name = name;
}

//======END SETTERS===========//

/*
    TODO
*/
void Link::display() const
{
  // clang-format off

  std::cout <<

      "Link [" << _name << "]" << "\n"
       <<

      "Type: " << type() << "\n"
       <<

      "Kinematic parameters (DH):" << "\n"
       << "a = " << _a << "\n"
       << "alpha = " << _alpha << "\n"
       << "theta = " << _theta << "\n"
       <<

      "Robot2DH Conversion:" << "\n"
       << "offset = " << _robot2dh_offset << " | flip = " << (_robot2dh_flip ? "yes" : "no") << "\n"
       <<

      "SoftLimits:" << "\n"
       << "[" << _Joint_Soft_limit_lower << " | " << _Joint_Soft_limit_higher << "]" << "\n"
       <<

      "HardLimits:" << "\n"
       << "[" << _Joint_Hard_limit_lower << " | " << _Joint_Hard_limit_higher << "]" << "\n"
       <<

      "Soft Velocity Limit: " << _soft_velocity_limit << "\n"
       << "Hard Velocity Limit: " << _hard_velocity_limit
      ;  // End COUT

  // clang-format on
}

/*
    return True if the input q_R (in Robot convention) exceeds the softLimits
*/
bool Link::exceededSoftJointLimits(double q_R) const
{
  return (q_R <= _Joint_Soft_limit_lower || q_R >= _Joint_Soft_limit_higher);
}

/*
    return True if the input q_Robot (in Robot convention) exceeds the HardRobotLimits
*/
bool Link::exceededHardJointLimits(double q_R) const
{
  return (q_R <= _Joint_Hard_limit_lower || q_R >= _Joint_Hard_limit_higher);
}

/*
    return True if the input q_vel exceeds the velocity soft limit
*/
bool Link::exceededSoftVelocityLimit(double q_vel) const
{
  return (abs(q_vel) >= _soft_velocity_limit);
}

/*
    return True if the input q_vel exceeds the velocity hard limit
*/
bool Link::exceededHardVelocityLimit(double q_vel) const
{
  return (abs(q_vel) >= _hard_velocity_limit);
}

/*
    TODO
*/
double Link::joint_Robot2DH(double q_Robot) const
{
  if (_robot2dh_flip)
    return (-q_Robot + _robot2dh_offset);
  else
    return (q_Robot + _robot2dh_offset);
}

/*
    TODO
*/
double Link::joint_DH2Robot(double q_DH) const
{
  if (_robot2dh_flip)
    return -(q_DH - _robot2dh_offset);
  else
    return (q_DH - _robot2dh_offset);
}

/*
    TODO
*/
double Link::jointvel_Robot2DH(double q_vel_Robot) const
{
  if (_robot2dh_flip)
    return -q_vel_Robot;
  else
    return q_vel_Robot;
}

/*
    TODO
*/
double Link::jointvel_DH2Robot(double q_vel_DH) const
{
  if (_robot2dh_flip)
    return -q_vel_DH;
  else
    return q_vel_DH;
}

}  // namespace uclv::robot