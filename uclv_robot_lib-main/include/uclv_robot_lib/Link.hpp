/*

    Robot Link

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

#pragma once

#include <Eigen/Dense>

namespace uclv::robot
{
//! Abstract Class for a Link
class Link
{
private:
  Link();  // No Default Constructor

protected:
  // Kinematic parameters (DH)
  double _a;      // link length
  double _alpha;  // link twist
  double _d;      // link offset
  double _theta;  // link angle

  //////////////////////////////////////////////////

  // Robot-DH Conversion
  double _robot2dh_offset;  // Offset between robot and DH convention
  double _robot2dh_flip;    // joint in Robot convention moves in opposite direction
  ///////////////////////////////////////////////

  // Safety Vars
  double _Joint_Soft_limit_lower, _Joint_Soft_limit_higher;  // SoftLimits in Robot convention
  double _Joint_Hard_limit_lower, _Joint_Hard_limit_higher;  // HardLimits in Robot convention
  double _hard_velocity_limit;                               // Hard Velocity Limits in Robot convention
  double _soft_velocity_limit;                               // Hard Velocity Limits in Robot convention
  //////////////////////////////////////////////

  std::string _name;  // joint name

  /*======CONSTRUCTORS======*/

  //! Full Constructor
  Link(double a, double alpha, double d, double theta, double robot2dh_offset, bool robot2dh_flip,
       double Joint_Hard_limit_lower, double Joint_Hard_limit_higher, double Joint_Soft_limit_lower,
       double Joint_Soft_limit_higher, double hard_velocity_limit, double soft_velocity_limit,
       const std::string& name = "joint_no_name");

  Link(double a, double alpha, double d, double theta, double robot2dh_offset = 0.0, bool robot2dh_flip = false,
       double Joint_Hard_limit_lower = -INFINITY, double Joint_Hard_limit_higher = INFINITY,
       double hard_velocity_limit = INFINITY, const std::string& name = "joint_no_name");

  // Link( const Link& rl) = default;

  /*======END CONSTRUCTORS======*/

  //========Varie=======//

  static void checkLowerHigher(double lower, double higher);

  virtual Eigen::Isometry3d A_internal(double theta, double d) const;

  //===================//

public:
  //========GETTERS==============//

  /*!
      return the link length
  */
  virtual double getDH_a() const;

  /*!
      return the link twist
  */
  virtual double getDH_alpha() const;

  /*!
      return the link offset
      Note:
          - For prismatic link this is the joint variable,
            in that case this function returns NaN
  */
  virtual double getDH_d() const;

  /*!
      return the link angle
      Note:
          - For revolute link this is the joint variable,
            in that case this function returns NaN
  */
  virtual double getDH_theta() const;

  /*!
      Return the offset between the robot and DH convention
  */
  virtual double getRobot2DH_offset() const;

  /*!
      Return the sign between robot and DH conventions
  */
  virtual bool getRobot2DH_flip() const;

  /*!
      TODO
  */
  virtual Eigen::Vector2d getSoftJointLimits() const;

  /*!
      TODO
  */
  virtual Eigen::Vector2d getHardJointLimits() const;

  /*!
      TODO
  */
  virtual double getSoftVelocityLimit() const;

  /*!
      TODO
  */
  virtual double getHardVelocityLimit() const;

  /*!
      Return the Joint Name
  */
  virtual const std::string& getName() const;

  /*!
      Clone the object
  */
  virtual Link* clone() const = 0;

  //======END GETTERS===========//

  //========SETTERS==============//

  /*!
      TODO
  */
  virtual void setDH_a(double a);

  /*!
      TODO
  */
  virtual void setDH_alpha(double alpha);

  /*!
      TODO
      ERROR IF LINK IS PRISMATIC
  */
  virtual void setDH_d(double d);

  /*!
      TODO
      ERROR IF LINK IS REVOLUTE
  */
  virtual void setDH_theta(double theta);

  /*!
      TODO
  */
  virtual void setRobot2DH_offset(double offset);

  /*!
      TODO
  */
  virtual void setRobot2DH_flip(bool flip);

  /*!
      TODO
  */
  virtual void setSoftJointLimits(double lower, double higher);

  /*!
      TODO
  */
  virtual void setSoftJointLimits(const Eigen::Ref<Eigen::Vector2d>& limits);

  /*!
      TODO
  */
  virtual void setHardJointLimits(double lower, double higher);

  /*!
      TODO
  */
  virtual void setHardJointLimits(const Eigen::Ref<Eigen::Vector2d>& limits);

  /*!
      TODO
  */
  virtual void setHardVelocityLimit(double velocity_limit);

  /*!
      TODO
  */
  virtual void setSoftVelocityLimit(double velocity_limit);

  /*!
      TODO
  */
  virtual void setName(const std::string& name);

  //======END SETTERS===========//

  /*!
      TODO
  */
  virtual void display() const;

  /*!
      Retrun the joint type
      'p' = prismatic
      'r' = revolute
  */
  virtual char type() const = 0;

  /*!
      Compute the link transform matrix
      input q_DH in DH convention
  */
  virtual Eigen::Isometry3d A(double q_DH) const = 0;

  /*!
      return True if the input q_R (in Robot convention) exceeds the softLimits
  */
  virtual bool exceededSoftJointLimits(double q_R) const;

  /*!
      return True if the input q_R (in Robot convention) exceeds the HardLimits
  */
  virtual bool exceededHardJointLimits(double q_R) const;

  /*!
      return True if the input q_vel exceeds the velocity soft limit
  */
  virtual bool exceededSoftVelocityLimit(double q_vel) const;

  /*!
      return True if the input q_vel exceeds the velocity hard limit
  */
  virtual bool exceededHardVelocityLimit(double q_vel) const;

  /*!
      TODO
  */
  virtual double joint_Robot2DH(double q_Robot) const;

  /*!
      TODO
  */
  virtual double joint_DH2Robot(double q_DH) const;

  /*!
      TODO
  */
  virtual double jointvel_Robot2DH(double q_vel_Robot) const;

  /*!
      TODO
  */
  virtual double jointvel_DH2Robot(double q_vel_DH) const;

};  // end class

}  // namespace uclv
