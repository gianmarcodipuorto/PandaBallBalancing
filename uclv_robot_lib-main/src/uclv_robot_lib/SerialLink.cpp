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

#include "uclv_robot_lib/SerialLink.hpp"
#include "uclv_robot_lib/exceptions.hpp"
#include <iostream>
#include <iomanip>

namespace uclv::robot
{
/*=========CONSTRUCTORS=========*/

/*
    Default constructor
    Robot with no links
*/
SerialLink::SerialLink()
{
  _b_T_0.setIdentity();
  _n_T_e.setIdentity();
  _name = "Robot_No_Name";
  _model = "Robot_No_Model";
}

SerialLink::SerialLink(const std::string& name)
{
  _b_T_0.setIdentity();
  _n_T_e.setIdentity();
  _name = name;
  _model = "Robot_No_Model";
}

/*
    Full constructor
*/
SerialLink::SerialLink(const std::vector<std::shared_ptr<Link>>& links, const Eigen::Isometry3d& b_T_0,
                       const Eigen::Isometry3d& n_T_e, const std::string& name)
  : _b_T_0(b_T_0), _links(links), _n_T_e(n_T_e), _name(name)
{
}

/*
    Constuctor without links
    usefull to use robot.push_back_link(...)
*/
SerialLink::SerialLink(const Eigen::Isometry3d& b_T_0, const Eigen::Isometry3d& n_T_e, const std::string& name)
  : _b_T_0(b_T_0), _n_T_e(n_T_e), _name(name)
{
}

/*
    Copy Constructor
    NB: the link objects are shared!!
*/
SerialLink::SerialLink(const SerialLink& robot)
{
  _b_T_0 = robot._b_T_0;
  _n_T_e = robot._n_T_e;
  _name = robot._name;
  _model = robot._model;
  _links = robot._links;
}

/*=====END CONSTRUCTORS=========*/

/*=======HELPS=========*/

/*
    Display robot in smart way
*/
void SerialLink::display()
{
  std::array<int, 7> w{ 2, 5, 4, 7, 7, 7, 7 };

  std::cout <<

      "Robot [" << _model << "] " << _name << "\n"
            <<

      "DH Table: "
            << "\n";

  for (int i = 0; i < 7; i++)
  {
    if (i == 0)
      std::cout << "╔";
    for (int j = 0; j < w[i]; j++)
    {
      std::cout << "═";
    }
    if (i == 6)
      std::cout << "╗";
    else
      std::cout << "╦";
  }
  std::cout << "\n"
            << std::setw(1) << "║" << std::setw(w[0]) << "#" << std::setw(1) << "║" << std::setw(w[1]) << "Name"
            << std::setw(1) << "║" << std::setw(w[2]) << "Type" << std::setw(1) << "║" << std::setw(w[3]) << "a"
            << std::setw(1) << "║" << std::setw(w[4]) << "alpha" << std::setw(1) << "║" << std::setw(w[5]) << "theta"
            << std::setw(1) << "║" << std::setw(w[6]) << "d" << std::setw(1) << "║"
            << "\n";
  for (int i = 0; i < 7; i++)
  {
    if (i == 0)
      std::cout << "╠";
    for (int j = 0; j < w[i]; j++)
    {
      std::cout << "═";
    }
    if (i == 6)
      std::cout << "╣";
    else
      std::cout << "╬";
  }
  std::cout << "\n";
  for (unsigned int i = 0; i < _links.size(); i++)
  {
    std::cout << std::setw(1) << "║" << std::setw(w[0]) << i + 1 << std::setw(1) << "║" << std::setw(w[1])
              << _links[i]->getName() << std::setw(1) << "║" << std::setw(w[2]) << _links[i]->type() << std::setw(1)
              << "║" << std::setw(w[3]) << std::setprecision(w[3] - 2) << _links[i]->getDH_a() << std::setw(1) << "║"
              << std::setw(w[4]) << std::setprecision(w[3] - 2) << _links[i]->getDH_alpha() << std::setw(1) << "║"
              << std::setw(w[5]) << std::setprecision(w[3] - 2) << _links[i]->getDH_theta() << std::setw(1) << "║"
              << std::setw(w[6]) << std::setprecision(w[3] - 2) << _links[i]->getDH_d() << std::setw(1) << "║"
              << "\n";
  }
  for (int i = 0; i < 7; i++)
  {
    if (i == 0)
      std::cout << "╚";
    for (int j = 0; j < w[i]; j++)
    {
      std::cout << "═";
    }
    if (i == 6)
      std::cout << "╝";
    else
      std::cout << "╩";
  }
  std::cout << "\n"
            << "0_T_b = "
            << "\n"
            << _b_T_0.matrix() << "\n"
            << "n_T_e = "
            << "\n"
            << _n_T_e.matrix() << "\n";
}

/*
    Display robot position
    Input in DH Convention
*/
void SerialLink::dispPosition(const Eigen::VectorXd& q_DH)
{
  assert((std::size_t)q_DH.size() == getNumJoints() && "SerialLink::dispPosition: invalid joint size");
  auto bTe = fkine(q_DH);
  std::cout << "========================="
            << "\n"
            << "Robot[ " << _model << " ]: " << _name << "\n"
            << "Teb = "
            << "\n"
            << bTe.matrix() << "\n"
            << "========================="
            << "\n";
}

/*=======END HELPS=====*/

/*=========GETTERS=========*/

/*
    get number of joints
*/
std::size_t SerialLink::getNumJoints() const
{
  return _links.size();
}

/*
    Get Transformation matrix of link_0 w.r.t. base frame
*/
const Eigen::Isometry3d& SerialLink::bT0() const
{
  return _b_T_0;
}

/*
    get Vector of links
    this function makes a copy
*/
const std::vector<std::shared_ptr<Link>>& SerialLink::links() const
{
  return _links;
}

/*
    Get Transformation matrix of link_0 w.r.t. base frame
*/
const Eigen::Isometry3d& SerialLink::nTe() const
{
  return _n_T_e;
}

/*
    Get robot name
*/
const std::string& SerialLink::name() const
{
  return _name;
}

/*
    Get robot model
*/
const std::string& SerialLink::model() const
{
  return _model;
}

/*
    Get i-th joint name
*/
const std::string& SerialLink::joint_name(int i) const
{
  return _links[i]->getName();
}

/*
    get a string of joint names given the bitmap
*/
std::string SerialLink::jointsNameFromBitMask(const std::vector<bool>& jointMask) const
{
  std::string out("");
  for (unsigned int i = 0; i < _links.size(); i++)
  {
    if (jointMask[i])
    {
      out += _links[i]->getName() + "|";
    }
  }
  if (out.size() > 0)
    return out.substr(0, out.size() - 1);
  else
    return out;
}

Eigen::VectorXd SerialLink::getCenterOfSoftJointLimits() const
{
  Eigen::VectorXd centers = Eigen::VectorXd::Zero(getNumJoints());
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    const auto& link = _links[i];
    ;
    centers[i] = (link->getSoftJointLimits()[1] + link->getSoftJointLimits()[0]) / 2.0;
  }
  return centers;
}

/*
    Clone the object
*/
SerialLink* SerialLink::clone() const
{
  return new SerialLink(*this);
}

/*=========END GETTERS=========*/

/*=========SETTERS=========*/

/*
    Set Transformation matrix of link_0 w.r.t. base frame
*/
Eigen::Isometry3d& SerialLink::bT0()
{
  return _b_T_0;
}

/*
    Set vector of links
*/
std::vector<std::shared_ptr<Link>>& SerialLink::links()
{
  return _links;
}

/*
    Add a link to the kinematic chain
*/
void SerialLink::push_back_link(const std::shared_ptr<Link>& link)
{
  _links.push_back(link);
}

/*
    overloaded operator: Add a link to the kinematic chain
*/
SerialLink& SerialLink::operator+=(const std::shared_ptr<Link>& link)
{
  push_back_link(link);
  return *this;
}

/*
    overloaded operator: Constuct a new Robot object and add a link to the
   kinematic chain
*/
SerialLink SerialLink::operator+(const std::shared_ptr<Link>& link) const
{
  SerialLink out = SerialLink(*this);
  out.push_back_link(link);
  return out;
}

/*
    Remove last link of the chain
*/
void SerialLink::pop_back_link()
{
  _links.pop_back();  // delete?
}

/*
    Set Transformation matrix of endeffector w.r.t. link_n frame
*/
Eigen::Isometry3d& SerialLink::nTe()
{
  return _n_T_e;
}

/*
    Set Robot Name
*/
std::string& SerialLink::name()
{
  return _name;
}

/*
    Set Robot Model
*/
std::string& SerialLink::model()
{
  return _model;
}

/*=========END SETTERS=========*/

/*=========CONVERSIONS=========*/

/*
    Transform joints from robot to DH convention
*/
Eigen::VectorXd SerialLink::joints_Robot2DH(const Eigen::VectorXd& q_Robot) const
{
  assert((std::size_t)q_Robot.size() == getNumJoints() && "SerialLink::joints_Robot2DH: invalid joint size");
  Eigen::VectorXd q_DH(getNumJoints());
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    q_DH[i] = _links[i]->joint_Robot2DH(q_Robot[i]);
  }
  return q_DH;
}

/*
    Transform joints from HD to robot convention
*/
Eigen::VectorXd SerialLink::joints_DH2Robot(const Eigen::VectorXd& q_DH) const
{
  assert((std::size_t)q_DH.size() == getNumJoints() && "SerialLink::joints_DH2Robot: invalid joint size");
  Eigen::VectorXd q_Robot(getNumJoints());
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    q_Robot[i] = _links[i]->joint_DH2Robot(q_DH[i]);
  }
  return q_Robot;
}

/*
    Transform joints velocity from robot to DH convention
*/
Eigen::VectorXd SerialLink::jointsvel_Robot2DH(const Eigen::VectorXd& q_dot_Robot) const
{
  assert((std::size_t)q_dot_Robot.size() == getNumJoints() && "SerialLink::jointsvel_Robot2DH: invalid joint size");
  Eigen::VectorXd q_dot_DH(getNumJoints());
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    q_dot_DH[i] = _links[i]->jointvel_Robot2DH(q_dot_Robot[i]);
  }
  return q_dot_DH;
}

/*
    Transform joints from DH to robot convention
*/
Eigen::VectorXd SerialLink::jointsvel_DH2Robot(const Eigen::VectorXd& q_dot_DH) const
{
  assert((std::size_t)q_dot_DH.size() == getNumJoints() && "SerialLink::jointsvel_DH2Robot: invalid joint size");
  Eigen::VectorXd q_dot_Robot(getNumJoints());
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    q_dot_Robot[i] = _links[i]->jointvel_DH2Robot(q_dot_DH[i]);
  }
  return q_dot_Robot;
}

/*!
      Transform jacobian from DH to Robot convention
*/
inline void SerialLink::jacobian_DH2Robot(const Eigen::Matrix<double, 6, Eigen::Dynamic>& J_DH,
                                          Eigen::Matrix<double, 6, Eigen::Dynamic>& J_R) const
{
  J_R = J_DH;
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    if (_links[i]->getRobot2DH_flip())
    {
      J_R.col(i) = -J_R.col(i);
    }
  }
}

inline void SerialLink::jacobian_DH2Robot(Eigen::Matrix<double, 6, Eigen::Dynamic>& J) const
{
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    if (_links[i]->getRobot2DH_flip())
    {
      J.col(i) = -J.col(i);
    }
  }
}

/*!
    Transform jacobian from Robot to DH convention
*/
inline void SerialLink::jacobian_Robot2DH(const Eigen::Matrix<double, 6, Eigen::Dynamic>& J_R,
                                          Eigen::Matrix<double, 6, Eigen::Dynamic>& J_DH) const
{
  jacobian_DH2Robot(J_R, J_DH);  // its the same algorithm
}

inline void SerialLink::jacobian_Robot2DH(Eigen::Matrix<double, 6, Eigen::Dynamic>& J) const
{
  jacobian_DH2Robot(J);  // its the same algorithm
}

/*=========END CONVERSIONS=========*/

/*=========SAFETY=========*/

/*
    Check Hard Limits
    Return a logic vector, if the i-th element is true then the i-th link has
   violated the limits
*/
std::vector<bool> SerialLink::checkHardJointLimits(const Eigen::VectorXd& q_Robot) const
{
  assert((std::size_t)q_Robot.size() == getNumJoints() && "SerialLink::checkHardJointLimits: invalid joint size");
  std::vector<bool> out(getNumJoints());
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    out[i] = (_links[i]->exceededHardJointLimits(q_Robot[i]));
  }
  return out;
}

/*
    Check Hard Limits
    Return true if any joint has violated the limits
*/
bool SerialLink::exceededHardJointLimits(const Eigen::VectorXd& q_Robot) const
{
  assert((std::size_t)q_Robot.size() == getNumJoints() && "SerialLink::exceededHardJointLimits: invalid joint size");
  std::vector<bool> b_vec = checkHardJointLimits(q_Robot);
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    if (b_vec[i])
    {
      return true;
    }
  }
  return false;
}

/*
    Check Soft Limits
    Return a logic vector, if the i-th element is true then the i-th link has
   violated the limits
*/
std::vector<bool> SerialLink::checkSoftJointLimits(const Eigen::VectorXd& q_R) const
{
  assert((std::size_t)q_R.size() == getNumJoints() && "SerialLink::checkSoftJointLimits: invalid joint size");
  std::vector<bool> out(getNumJoints());
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    out[i] = (_links[i]->exceededSoftJointLimits(q_R[i]));
  }
  return out;
}

/*
    Check Soft Limits
    Return true if any joint has violated the limits
*/
bool SerialLink::exceededSoftJointLimits(const Eigen::VectorXd& q_R) const
{
  assert((std::size_t)q_R.size() == getNumJoints() && "SerialLink::exceededSoftJointLimits: invalid joint size");
  std::vector<bool> b_vec = checkSoftJointLimits(q_R);
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    if (b_vec[i])
    {
      return true;
    }
  }
  return false;
}

/*
    Check HARD Velocity Limits
    Return a logic vector, if the i-th element is true then the i-th link has
   violated the limits
*/
std::vector<bool> SerialLink::checkHardVelocityLimits(const Eigen::VectorXd& q_dot) const
{
  assert((std::size_t)q_dot.size() == getNumJoints() && "SerialLink::checkHardVelocityLimits: invalid joint size");
  std::vector<bool> out(getNumJoints());
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    out[i] = (_links[i]->exceededHardVelocityLimit(q_dot[i]));
  }
  return out;
}

/*
    Check HARD Velocity Limits
    Return true if any joint has violated the limits
*/
bool SerialLink::exceededHardVelocityLimits(const Eigen::VectorXd& q_dot) const
{
  assert((std::size_t)q_dot.size() == getNumJoints() && "SerialLink::exceededHardVelocityLimits: invalid joint size");
  std::vector<bool> b_vec = checkHardVelocityLimits(q_dot);
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    if (b_vec[i])
    {
      return true;
    }
  }
  return false;
}

/*
    Check SOFT Velocity Limits
    Return a logic vector, if the i-th element is true then the i-th link has
   violated the limits
*/
std::vector<bool> SerialLink::checkSoftVelocityLimits(const Eigen::VectorXd& q_dot) const
{
  assert((std::size_t)q_dot.size() == getNumJoints() && "SerialLink::checkSoftVelocityLimits: invalid joint size");
  std::vector<bool> out(getNumJoints());
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    out[i] = (_links[i]->exceededSoftVelocityLimit(q_dot[i]));
  }
  return out;
}

/*
    Check SOFT Velocity Limits
    Return true if any joint has violated the limits
*/
bool SerialLink::exceededSoftVelocityLimits(const Eigen::VectorXd& q_dot) const
{
  assert((std::size_t)q_dot.size() == getNumJoints() && "SerialLink::exceededSoftVelocityLimits: invalid joint size");
  std::vector<bool> b_vec = checkSoftVelocityLimits(q_dot);
  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    if (b_vec[i])
    {
      return true;
    }
  }
  return false;
}

/*=========END SAFETY=========*/

/*========FKINE=========*/

/*
    Internal fkine
    This function compute the fkine to joint "n_joint" given the last
   transformation to joint n_joint-1
    - q_DH_j is the joint position of the i-th link
    - b_T_j_1 is the transformation of the link j-1 w.r.t base frame
*/
inline Eigen::Isometry3d SerialLink::fkine_internal(const double& q_DH_j, const Eigen::Isometry3d& b_T_j_1,
                                                    unsigned int n_joint) const
{
  assert(n_joint < getNumJoints() && "SerialLink::fkine_internal: invalid joint number");
  return (b_T_j_1 * _links[n_joint]->A(q_DH_j));
}

/*
    fkine to n_joint-th link
    j_T_f will be post multiplyed to the result
    if n_joint = NUM_JOINT+1 then the result is b_T_e*j_T_f
*/
inline Eigen::Isometry3d SerialLink::fkine(const Eigen::VectorXd& q_DH, unsigned int n_joint,
                                           const Eigen::Isometry3d& j_T_f) const
{
  assert(n_joint <= (getNumJoints() + 1) && "SerialLink::fkine: invalid joint number");
  assert((std::size_t)q_DH.size() <= getNumJoints() && "SerialLink::fkine: invalid joint size");
  return (fkine(q_DH, n_joint) * j_T_f);
}

/*
  fkine to n_joint-th link
  if n_joint = NUM_JOINT+1 then the result is b_T_e
*/
inline Eigen::Isometry3d SerialLink::fkine(const Eigen::VectorXd& q_DH, unsigned int n_joint) const
{
  assert(n_joint <= (getNumJoints() + 1) && "SerialLink::fkine: invalid joint number");
  assert((std::size_t)q_DH.size() <= getNumJoints() && "SerialLink::fkine: invalid joint size");

  // Start from frame 0
  Eigen::Isometry3d b_T_j = _b_T_0;

  // check if the final frame is {end-effector}
  bool ee = false;
  if (n_joint == (getNumJoints() + 1))
  {
    n_joint--;
    ee = true;
  }

  for (unsigned int i = 0; i < n_joint; i++)
  {
    b_T_j = fkine_internal(q_DH[i], b_T_j, i);
  }

  // if the final frame is the {end-effector} then add it
  if (ee)
  {
    b_T_j = b_T_j * _n_T_e;
  }

  return b_T_j;
}

/*
    fkine to the end-effector
*/
inline Eigen::Isometry3d SerialLink::fkine(const Eigen::VectorXd& q_DH) const
{
  assert((std::size_t)q_DH.size() <= getNumJoints() && "SerialLink::fkine: invalid joint size");
  return fkine(q_DH, getNumJoints() + 1);
}

/*
    The resul of this function is the matrix b_T_f
    where f is a given frame defined by the input e_T_f
    - e_T_f is the transformation of the frame {f} w.r.t. frame {end-effector}
*/
inline Eigen::Isometry3d SerialLink::fkine(const Eigen::VectorXd& q_DH, const Eigen::Isometry3d& e_T_f) const
{
  assert((std::size_t)q_DH.size() <= getNumJoints() && "SerialLink::fkine: invalid joint size");
  return (fkine(q_DH) * e_T_f);
}

/*
    This function return all the transformation up to link "n_joint"
    The return is a vector of size n_joint
    if n_joint=NUM_JOINT+1 then the output will be a vector of size n_joint as
   well, but the last element is b_T_e
*/
inline std::vector<Eigen::Isometry3d> SerialLink::fkine_all(const Eigen::VectorXd& q_DH, unsigned int n_joint) const
{
  assert(n_joint <= (getNumJoints() + 1) && "SerialLink::fkine: invalid joint number");
  assert((std::size_t)q_DH.size() <= getNumJoints() && "SerialLink::fkine: invalid joint size");

  std::vector<Eigen::Isometry3d> out;
  out.reserve(n_joint + 1);

  // Start from frame 0
  out.push_back(_b_T_0);

  // check if the final frame is {end-effector}
  bool ee = false;
  if (n_joint == (getNumJoints() + 1))
  {
    n_joint--;
    ee = true;
  }

  for (unsigned int i = 0; i < n_joint; i++)
  {
    out.push_back(fkine_internal(q_DH[i], out.back(), i));
  }

  // if the final frame is the {end-effector} then add it to the last element
  if (ee)
  {
    out.back() = out.back() * _n_T_e;
  }

  return out;
}

/*========END FKINE=========*/

/*========Jacobians=========*/

/*
    Internal computation of the position part of the jacobian in the frame {f}
    The input is a vector of all transformation the considered joints i.e.
    [ b_T_0 , b_T_1, ... , (b_T_j*j_T_f) ] (size = joints+1)
*/
Eigen::Matrix<double, 3, Eigen::Dynamic> SerialLink::jacob_p_internal(const std::vector<Eigen::Isometry3d>& all_T) const
{
  assert(all_T.size() > 0 && "SerialLink::jacob_p_internal: invalid input size");
  assert(all_T.size() <= getNumJoints() + 1 && "SerialLink::jacob_p_internal: invalid input size");

  int numQ = all_T.size() - 1;

  Eigen::Matrix<double, 3, Eigen::Dynamic> Jp = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, numQ);

  auto p_e = all_T.back().translation();

  for (int i = 0; i < numQ; i++)
  {
    // auto z_i_1 = all_T[i].rotation().col(2);
    auto z_i_1 = all_T[i].matrix().block<3, 1>(0, 2);

    switch (_links[i]->type())
    {
      case 'p':  // Prismatic
      {
        Jp.col(i) = z_i_1;
        break;
      }

      case 'r':  // Revolute
      {
        auto p_i_1 = all_T[i].translation();
        Jp.col(i) = z_i_1.cross(p_e - p_i_1);
        break;
      }

      default: {
        throw uclv::robot::runtime_error("SerialLink::jacob_p_internal: invalid joint type: _links[" +
                                         std::to_string(i) + "].type()=" + _links[i]->type());
      }
    }
  }

  return Jp;
}

/*
    Internal computation of the orientation part of the geometric jacobian in
   the frame {f} The input is a vector of all transformation the considered
   joints i.e. [ b_T_0 , b_T_1, ... , (b_T_j*j_T_f) ] (size = joints+1)
*/
Eigen::Matrix<double, 3, Eigen::Dynamic>
SerialLink::jacob_o_geometric_internal(const std::vector<Eigen::Isometry3d>& all_T) const
{
  assert(all_T.size() > 0 && "SerialLink::jacob_p_internal: invalid input size");
  assert(all_T.size() <= getNumJoints() + 1 && "SerialLink::jacob_p_internal: invalid input size");

  int numQ = all_T.size() - 1;

  Eigen::Matrix<double, 3, Eigen::Dynamic> Jo_geometric = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, numQ);

  for (int i = 0; i < numQ; i++)
  {
    switch (_links[i]->type())
    {
      case 'p':  // Prismatic
      {
        // Commented because it is already initialized to zero
        // Jo_geometric.col(i).setZero();
        break;
      }

      case 'r':  // Revolute
      {
        // auto z_i_1 = all_T[i].matrix().block<3, 1>(0, 2);
        // Jo_geometric.col(i) = z_i_1;
        // Done in one line:
        Jo_geometric.col(i) = all_T[i].matrix().block<3, 1>(0, 2);
        break;
      }

      default: {
        throw uclv::robot::runtime_error("SerialLink::jacob_o_geometric_internal: invalid joint type: _links[" +
                                         std::to_string(i) + "].type()=" + _links[i]->type());
      }
    }
  }

  return Jo_geometric;
}

/*
    Compute the position part of the jacobian in frame {f} w.r.t. base frame
   (pag 111) The jacobian is computed using the first n_joint joints. The matrix
   j_T_f defines the frame {f}: this is the transformation of frame {j} w.r.t.
   frame of the joint n_joint If n_joint is n_joint+1 the frame {end-effector}
   is considered as frame of the last joint
*/
inline Eigen::Matrix<double, 3, Eigen::Dynamic> SerialLink::jacob_p(const Eigen::VectorXd& q_DH, unsigned int n_joint,
                                                                    const Eigen::Isometry3d& j_T_f) const
{
  auto all_T = fkine_all(q_DH, n_joint);
  all_T.back() = all_T.back() * j_T_f;
  return jacob_p_internal(all_T);
}

/*
    Compute the position part of the jacobian in frame of joint n_joint w.r.t.
   base frame (pag 111) The jacobian is computed using the first n_joint joints.
    If n_joint is n_joint+1 the frame {end-effector} is considered as frame of
   the last joint
*/
inline Eigen::Matrix<double, 3, Eigen::Dynamic> SerialLink::jacob_p(const Eigen::VectorXd& q_DH,
                                                                    unsigned int n_joint) const
{
  return jacob_p_internal(fkine_all(q_DH, n_joint));
}

/*
    Compute the position part of the jacobian in frame {end-effector} w.r.t.
   base frame (pag 111)
*/
inline Eigen::Matrix<double, 3, Eigen::Dynamic> SerialLink::jacob_p(const Eigen::VectorXd& q_DH) const
{
  return jacob_p(q_DH, getNumJoints() + 1);
}

/*
    Compute the orientation part of the geometric jacobian in frame {f} w.r.t.
   base frame (pag 111) The jacobian is computed using the first n_joint joints.
    The matrix j_T_f defines the frame {f}: this is the transformation of frame
   {j} w.r.t. frame of the joint n_joint If n_joint is n_joint+1 the frame
   {end-effector} is considered as frame of the last joint
*/
inline Eigen::Matrix<double, 3, Eigen::Dynamic> SerialLink::jacob_o_geometric(const Eigen::VectorXd& q_DH,
                                                                              unsigned int n_joint,
                                                                              const Eigen::Isometry3d& j_T_f) const
{
  auto all_T = fkine_all(q_DH, n_joint);
  all_T.back() = all_T.back() * j_T_f;
  return jacob_o_geometric_internal(all_T);
}

/*
    Compute the orientation part of the geometric jacobian in frame of joint
   n_joint w.r.t. base frame (pag 111) The jacobian is computed using the first
   n_joint joints. If n_joint is n_joint+1 the frame {end-effector} is
   considered as frame of the last joint
*/
inline Eigen::Matrix<double, 3, Eigen::Dynamic> SerialLink::jacob_o_geometric(const Eigen::VectorXd& q_DH,
                                                                              unsigned int n_joint) const
{
  return jacob_o_geometric_internal(fkine_all(q_DH, n_joint));
}

/*
    Compute the orientation part of the geometric jacobian in frame
   {end-effector} w.r.t. base frame (pag 111)
*/
inline Eigen::Matrix<double, 3, Eigen::Dynamic> SerialLink::jacob_o_geometric(const Eigen::VectorXd& q_DH) const
{
  return jacob_o_geometric(q_DH, getNumJoints() + 1);
}

/*
    Compute the geometric jacobian in frame {f} w.r.t. base frame (pag 111)
    The jacobian is computed using the first n_joint joints.
    The matrix j_T_f defines the frame {f}: this is the transformation of frame
   {j} w.r.t. frame of the joint n_joint If n_joint is n_joint+1 the frame
   {end-effector} is considered as frame of the last joint
*/
inline Eigen::Matrix<double, 6, Eigen::Dynamic> SerialLink::jacob_geometric(const Eigen::VectorXd& q_DH,
                                                                            unsigned int n_joint,
                                                                            const Eigen::Isometry3d& j_T_f) const
{
  auto all_T = fkine_all(q_DH, n_joint);
  all_T.back() = all_T.back() * j_T_f;

  if (n_joint == getNumJoints() + 1)
    n_joint--;

  Eigen::Matrix<double, 6, Eigen::Dynamic> J_geo(6, n_joint);

  J_geo.block<3, Eigen::Dynamic>(0, 0, 3, n_joint) = jacob_p_internal(all_T);
  J_geo.block<3, Eigen::Dynamic>(3, 0, 3, n_joint) = jacob_o_geometric_internal(all_T);

  return J_geo;
}

/*
    Compute the geometric jacobian in frame of joint n_joint w.r.t. base frame
   (pag 111) The jacobian is computed using the first n_joint joints. If n_joint
   is n_joint+1 the frame {end-effector} is considered as frame of the last
   joint
*/
inline Eigen::Matrix<double, 6, Eigen::Dynamic> SerialLink::jacob_geometric(const Eigen::VectorXd& q_DH,
                                                                            unsigned int n_joint) const
{
  auto all_T = fkine_all(q_DH, n_joint);

  if (n_joint == getNumJoints() + 1)
    n_joint--;

  Eigen::Matrix<double, 6, Eigen::Dynamic> J_geo(6, n_joint);

  J_geo.block<3, Eigen::Dynamic>(0, 0, 3, n_joint) = jacob_p_internal(all_T);
  J_geo.block<3, Eigen::Dynamic>(3, 0, 3, n_joint) = jacob_o_geometric_internal(all_T);

  return J_geo;
}

/*
    Compute the geometric jacobian in frame {end-effector} w.r.t. base frame
   (pag 111)
*/
inline Eigen::Matrix<double, 6, Eigen::Dynamic> SerialLink::jacob_geometric(const Eigen::VectorXd& q_DH) const
{
  return jacob_geometric(q_DH, getNumJoints() + 1);
}

/*
    Ginven the jacobian b_J in frame {b} and the rotation matrix u_R_b of frame
   {b} w.r.t. frame {u}, compute the jacobian w.r.t frame {u} (pag 113) The
   jacobian b_J can be the position part (3xQ), the orientation part (3xQ) or
   the full jacobian (6xQ)
*/
Eigen::MatrixXd SerialLink::change_jacob_frame(Eigen::MatrixXd b_J, const Eigen::Matrix3d& u_R_b)
{
  Eigen::Quaterniond a;
  switch (b_J.rows())
  {
    case 3: {
      return u_R_b * b_J;
      // break;
    }

    case 6: {
      auto num_cols = b_J.cols();
      b_J.block<3, Eigen::Dynamic>(0, 0, 3, num_cols) = u_R_b * b_J.block<3, Eigen::Dynamic>(0, 0, 3, num_cols);
      b_J.block<3, Eigen::Dynamic>(3, 0, 3, num_cols) = u_R_b * b_J.block<3, Eigen::Dynamic>(3, 0, 3, num_cols);
      return b_J;
      break;
    }

    default: {
      throw uclv::robot::runtime_error("SerialLink::change_jacob_frame: invalid b_J Matrix rows dimension [" +
                                       std::to_string(b_J.rows()) + "]");
    }
  }
}

/*========END Jacobians=========*/

/*==========Operators========*/

/*
  overloaded operator +
  Construct a new Robot object with link1 as the first link and link2 as second
  link
*/
SerialLink operator+(const std::shared_ptr<Link>& link1, const std::shared_ptr<Link>& link2)
{
  SerialLink out = SerialLink();
  out.push_back_link(link1);
  out.push_back_link(link2);
  return out;
}

}  // namespace uclv::robot