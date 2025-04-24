/*
    Robot Class for the Franka Emika Panda

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

#include "uclv_robot_lib/robots/FrankaEmikaPanda.hpp"
#include "uclv_robot_lib/links/Revolute.hpp"

namespace uclv::robot
{
/*=========CONSTRUCTORS=========*/

/*
    Full constructor
*/
FrankaEmikaPanda::FrankaEmikaPanda(const Eigen::Isometry3d& n_T_e, const std::string& name)
  : SerialLink(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.333)), n_T_e, name)
{
  _model = FRANKA_EMIKA_PANDA_MODEL_STR;
  // L1
  push_back_link(std::make_shared<link::Revolute>(
      // a,   alpha,     d,
      //-0.0003, -1.5714, 0.0,
      0.0, -M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -2.8973, 2.8973,
      // hard_velocity_limit
      2.1750,
      // string name
      "P1"));

  // L2
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      //-0.0003, 1.5697,-0.0011,
      0.0, M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -1.7628, 1.7628,
      // hard_velocity_limit
      2.1750,
      // string name
      "P2"));
  // L3
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      //0.0833, 1.5701, 0.3134,
      0.0825, M_PI / 2.0, 0.316,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -2.8973, 2.8973,
      // hard_velocity_limit
      2.1750,
      // string name
      "P3"));
  // L4
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      //-0.0810, -1.5700, 0.0,
      -0.0825, -M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -3.0718,-0.0698, 
      // hard_velocity_limit
      2.1750,
      // string name
      "P4"));
  // L5
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      //0.0006, 1.5741, 0.3817,
      0.0, M_PI / 2.0, 0.384,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -2.8973, 2.8973,
      // hard_velocity_limit
      2.61,
      // string name
      "P5"));
  // L6
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      //0.0837, 1.5741, -0.00027,
      0.088, M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -0.0175, 3.7525,
      // hard_velocity_limit
      2.61,
      // string name
      "P6"));
  // L7
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      //-0.0226, -0.00024, 0.107,
      0.0, 0.0, 0.107,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -2.8973, 2.8973,
      // hard_velocity_limit
      2.61,
      // string name
      "P7"));


    //Note: the volocity depends on the configuration of the robot, in this file we have only upper and lower limits.
}

/*
    Constructor with name only
*/
FrankaEmikaPanda::FrankaEmikaPanda(const std::string& name) : FrankaEmikaPanda(Eigen::Isometry3d::Identity(), name)
{
}

/*
    Empty constructor
*/
FrankaEmikaPanda::FrankaEmikaPanda() : FrankaEmikaPanda("PANDA_NO_NAME")
{
}

}  // namespace uclv::robot

/*=========END CONSTRUCTORS=========*/