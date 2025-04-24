/*

    Robot Prismatic Link

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

#include "uclv_robot_lib/links/Prismatic.hpp"
#include "uclv_robot_lib/exceptions.hpp"

namespace uclv::robot::link
{
/*=============CONSTRUCTORS===========*/

// Full Constructor
Prismatic::Prismatic(double a, double alpha, double theta, double robot2dh_offset, bool robot2dh_flip,
                     double Joint_Hard_limit_lower, double Joint_Hard_limit_higher, double Joint_Soft_limit_lower,
                     double Joint_Soft_limit_higher, double hard_velocity_limit, double soft_velocity_limit,
                     const std::string& name)
  : Link(a, alpha, NAN, theta, robot2dh_offset, robot2dh_flip, Joint_Hard_limit_lower, Joint_Hard_limit_higher,
         Joint_Soft_limit_lower, Joint_Soft_limit_higher, hard_velocity_limit, soft_velocity_limit, name)
{
}

Prismatic::Prismatic(double a, double alpha, double theta, double robot2dh_offset, bool robot2dh_flip,
                     double Joint_Hard_limit_lower, double Joint_Hard_limit_higher, double hard_velocity_limit,
                     const std::string& name)
  : Link(a, alpha, NAN, theta, robot2dh_offset, robot2dh_flip, Joint_Hard_limit_lower, Joint_Hard_limit_higher,
         hard_velocity_limit, name)
{
}

/*=======END CONSTRUCTORS===========*/

/*
    Clone the object
*/
Prismatic* Prismatic::clone() const
{
  return new Prismatic(*this);
}

/*
        return the link angle
        Note:
            - For revolute link this is the joint variable,
            in that case this function returns NaN
*/
double Prismatic::getDH_d() const
{
  return NAN;
}

/*
        TODO
        ERROR IF LINK IS REVOLUTE
*/
void Prismatic::setDH_d(double)
{
  throw robot::runtime_error("[Prismatic] Error in setDH_d( double d ): Cannot set d for Prismatic");
}

/*
    TODO
*/
void Prismatic::display() const
{
  Link::display();
}

/*
        Retrun the joint type
        'p' = prismatic
        'r' = revolute
*/
char Prismatic::type() const
{
  return 'p';
}

/*
    Compute the link transform matrix
    input q_DH in DH convention
*/
Eigen::Isometry3d Prismatic::A(double q_DH) const
{
  return A_internal(_theta, q_DH);
}

bool isPrismatic(const Link& l)
{
  return (l.type() == 'p');
}

}  // namespace uclv::robot::link