/*

    Robot Revolute Link

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

#include "uclv_robot_lib/links/Revolute.hpp"
#include "uclv_robot_lib/exceptions.hpp"

namespace uclv::robot::link
{
/*=============CONSTRUCTORS===========*/

// Full Constructor
Revolute::Revolute(double a, double alpha, double d, double robot2dh_offset, bool robot2dh_flip,
                   double Joint_Hard_limit_lower, double Joint_Hard_limit_higher, double Joint_Soft_limit_lower,
                   double Joint_Soft_limit_higher, double hard_velocity_limit, double soft_velocity_limit,
                   const std::string& name)
  : Link(a, alpha, d, NAN, robot2dh_offset, robot2dh_flip, Joint_Hard_limit_lower, Joint_Hard_limit_higher,
         Joint_Soft_limit_lower, Joint_Soft_limit_higher, hard_velocity_limit, soft_velocity_limit, name)
{
}

Revolute::Revolute(double a, double alpha, double d, double robot2dh_offset, bool robot2dh_flip,
                   double Joint_Hard_limit_lower, double Joint_Hard_limit_higher, double hard_velocity_limit,
                   const std::string& name)
  : Link(a, alpha, d, NAN, robot2dh_offset, robot2dh_flip, Joint_Hard_limit_lower, Joint_Hard_limit_higher,
         hard_velocity_limit, name)
{
}

/*=======END CONSTRUCTORS===========*/

/*
    Clone the object
*/
Revolute* Revolute::clone() const
{
  return new Revolute(*this);
}

/*
        return the link angle
        Note:
            - For revolute link this is the joint variable,
            in that case this function returns NaN
*/
double Revolute::getDH_theta() const
{
  return NAN;
}

/*
        TODO
        ERROR IF LINK IS REVOLUTE
*/
void Revolute::setDH_theta(double)
{
  throw robot::runtime_error("[Revolute] Error in setDH_theta( double theta ): Cannot set theta for Revolute");
}

/*
    TODO
*/
void Revolute::display() const
{
  Link::display();
}

/*
        Retrun the joint type
        'p' = prismatic
        'r' = revolute
*/
char Revolute::type() const
{
  return 'r';
}

/*
    Compute the link transform matrix
    input q_DH in DH convention
*/
Eigen::Isometry3d Revolute::A(double q_DH) const
{
  return A_internal(q_DH, _d);
}

bool isRevolute(const Link& l)
{
  return (l.type() == 'r');
}

}  // namespace uclv