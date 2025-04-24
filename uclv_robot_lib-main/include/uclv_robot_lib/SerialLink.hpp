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

#pragma once

#include <uclv_robot_lib/Link.hpp>
#include <vector>
#include <memory>
// #include <iomanip>
// #include "sun_math_toolbox/PortingFunctions.h"
// #include "sun_math_toolbox/UnitQuaternion.h"

namespace uclv::robot
{
//! The SerialLink Class
class SerialLink
{
private:
protected:
  //! Transformation matrix of link_ w.r.t. base frame
  Eigen::Isometry3d _b_T_0;  // T_0^b
  //! Links
  std::vector<std::shared_ptr<Link>> _links;
  //! Transformation matrix of effector w.r.t. link_n frame
  Eigen::Isometry3d _n_T_e;  // T_e^n

  //! Name of the robot
  std::string _name;

  //! Model of the robot
  std::string _model;

public:
  /*=========CONSTRUCTORS=========*/

  /*!
      Default constructor

      Robot with no links
  */
  SerialLink();

  SerialLink(const std::string& name);

  /*!
      Full constructor
  */
  SerialLink(const std::vector<std::shared_ptr<Link>>& links, const Eigen::Isometry3d& b_T_0,
             const Eigen::Isometry3d& n_T_e, const std::string& name);

  /*!
      Constuctor without links

      usefull to use robot.push_back_link(...)
  */
  SerialLink(const Eigen::Isometry3d& b_T_0, const Eigen::Isometry3d& n_T_e, const std::string& name);
  /*
      Copy Constructor
      NB: the link objects are shared!!
 */
  SerialLink(const SerialLink& robot);

  /*=====END CONSTRUCTORS=========*/

  /*=======HELPS=========*/

public:
  /*!
      Display robot in smart way
  */
  virtual void display();

  /*!
      Display robot position
      Input in DH Convention
  */
  virtual void dispPosition(const Eigen::VectorXd& q_DH);

  /*=======END HELPS=====*/

  /*=========GETTERS=========*/

  /*!
      get number of joints
  */
  virtual std::size_t getNumJoints() const;

  /*!
      Get Transformation matrix of link_0 w.r.t. base frame
  */
  virtual const Eigen::Isometry3d& bT0() const;

  /*!
      get Vector of links
  */
  virtual const std::vector<std::shared_ptr<Link>>& links() const;

  /*!
      Get Transformation matrix of link_0 w.r.t. base frame
  */
  virtual const Eigen::Isometry3d& nTe() const;

  /*!
      Get robot name
  */
  virtual const std::string& name() const;

  /*!
      Get robot model
  */
  virtual const std::string& model() const;

  /*!
      Get i-th joint name
  */
  virtual const std::string& joint_name(int i) const;

  /*!
      get a string of joint names given the bitmap
  */
  virtual std::string jointsNameFromBitMask(const std::vector<bool>& jointMask) const;

  virtual Eigen::VectorXd getCenterOfSoftJointLimits() const;

  /*!
      Clone the object
  */
  virtual SerialLink* clone() const;

  /*=========END GETTERS=========*/

  /*=========SETTERS=========*/

  /*!
      Set Transformation matrix of link_0 w.r.t. base frame
  */
  virtual Eigen::Isometry3d& bT0();

  /*!
      Set vector of links
  */
  virtual std::vector<std::shared_ptr<Link>>& links();

  /*!
      Add a link to the kinematic chain
  */
  virtual void push_back_link(const std::shared_ptr<Link>& link);

  /*!
      overloaded operator: Add a link to the kinematic chain
  */
  virtual SerialLink& operator+=(const std::shared_ptr<Link>& link);

  /*!
      overloaded operator: Constuct a new Robot object and add a link to the kinematic chain
  */
  virtual SerialLink operator+(const std::shared_ptr<Link>& link) const;

  /*!
      Remove last link of the chain
  */
  virtual void pop_back_link();

  /*!
      Set Transformation matrix of endeffector w.r.t. link_n frame
  */
  virtual Eigen::Isometry3d& nTe();

  /*!
      Set Robot Name
  */
  virtual std::string& name();

  /*!
      Set Robot Model
  */
  virtual std::string& model();

  /*=========END SETTERS=========*/

  /*=========CONVERSIONS=========*/

  /*!
      Transform joints from robot to DH convention
  */
  virtual Eigen::VectorXd joints_Robot2DH(const Eigen::VectorXd& q_Robot) const;

  /*!
      Transform joints from HD to robot convention
  */
  virtual Eigen::VectorXd joints_DH2Robot(const Eigen::VectorXd& q_DH) const;

  /*!
      Transform joints velocity from robot to DH convention
  */
  virtual Eigen::VectorXd jointsvel_Robot2DH(const Eigen::VectorXd& q_dot_Robot) const;

  /*!
      Transform joints from DH to robot convention
  */
  virtual Eigen::VectorXd jointsvel_DH2Robot(const Eigen::VectorXd& q_dot_DH) const;

  /*!
      Transform jacobian from DH to Robot convention
  */
  virtual void jacobian_DH2Robot(const Eigen::Matrix<double, 6, Eigen::Dynamic>& J_DH,
                                 Eigen::Matrix<double, 6, Eigen::Dynamic>& J_R) const;

  virtual void jacobian_DH2Robot(Eigen::Matrix<double, 6, Eigen::Dynamic>& J) const;

  /*!
      Transform jacobian from Robot to DH convention
  */
  virtual void jacobian_Robot2DH(const Eigen::Matrix<double, 6, Eigen::Dynamic>& J_R,
                                 Eigen::Matrix<double, 6, Eigen::Dynamic>& J_DH) const;

  virtual void jacobian_Robot2DH(Eigen::Matrix<double, 6, Eigen::Dynamic>& J) const;

  /*=========END CONVERSIONS=========*/

  /*=========SAFETY=========*/

  /*!
      Check Hard Limits

      Return a logic vector, if the i-th element is true then the i-th link has violated the limits
  */
  virtual std::vector<bool> checkHardJointLimits(const Eigen::VectorXd& q_Robot) const;

  /*!
      Check Hard Limits

      Return true if any joint has violated the limits
  */
  virtual bool exceededHardJointLimits(const Eigen::VectorXd& q_Robot) const;

  /*!
      Check Soft Limits

      Return a logic vector, if the i-th element is true then the i-th link has violated the limits
  */
  virtual std::vector<bool> checkSoftJointLimits(const Eigen::VectorXd& q_Robot) const;

  /*!
      Check Soft Limits

      Return true if any joint has violated the limits
  */
  virtual bool exceededSoftJointLimits(const Eigen::VectorXd& q_R) const;

  /*!
      Check Hard Velocity Limits

      Return a logic vector, if the i-th element is true then the i-th link has violated the limits
  */
  virtual std::vector<bool> checkHardVelocityLimits(const Eigen::VectorXd& q_dot) const;

  /*!
      Check Velocity Limits

      Return true if any joint has violated the limits
  */
  virtual bool exceededHardVelocityLimits(const Eigen::VectorXd& q_dot) const;

  virtual std::vector<bool> checkSoftVelocityLimits(const Eigen::VectorXd& q_dot) const;

  /*!
      Check SOFT Velocity Limits

      Return true if any joint has violated the limits
  */
  virtual bool exceededSoftVelocityLimits(const Eigen::VectorXd& q_dot) const;

  /*=========END SAFETY=========*/

  /*========FKINE=========*/

protected:
  /*!
      Internal fkine

      This function compute the fkine to joint "n_joint" given the last transformation to joint n_joint-1
      - q_DH_j is the joint position of the i-th link
      - b_T_j_1 is the transformation of the link j-1 w.r.t base frame
  */
  virtual Eigen::Isometry3d fkine_internal(const double& q_DH_j, const Eigen::Isometry3d& b_T_j_1,
                                           unsigned int n_joint) const;

public:
  /*!
      fkine to n_joint-th link

      j_T_f will be post multiplyed to the result

      if n_joint = NUM_JOINT+1 then the result is b_T_e*j_T_f
  */
  virtual Eigen::Isometry3d fkine(const Eigen::VectorXd& q_DH, unsigned int n_joint,
                                  const Eigen::Isometry3d& j_T_f) const;

  /*!
    fkine to n_joint-th link
    if n_joint = NUM_JOINT+1 then the result is b_T_e
  */
  virtual Eigen::Isometry3d fkine(const Eigen::VectorXd& q_DH, unsigned int n_joint) const;

  /*!
      fkine to the end-effector
  */
  virtual Eigen::Isometry3d fkine(const Eigen::VectorXd& q_DH) const;

  /*!
      The resul of this function is the matrix b_T_f
      where f is a given frame defined by the input e_T_f
      - e_T_f is the transformation of the frame {f} w.r.t. frame {end-effector}
  */
  virtual Eigen::Isometry3d fkine(const Eigen::VectorXd& q_DH, const Eigen::Isometry3d& e_T_f) const;

  /*!
      This function return all the transformation up to link "n_joint"

      The return is a vector of size n_joint

      if n_joint=NUM_JOINT+1 then the output will be a vector of size n_joint as well, but the last element is b_T_e
  */
  virtual std::vector<Eigen::Isometry3d> fkine_all(const Eigen::VectorXd& q_DH, unsigned int n_joint) const;

  /*========END FKINE=========*/

  /*========Jacobians=========*/

protected:
  /*!
      Internal computation of the position part of the jacobian in the frame {f}
      The input is a vector of all transformation the considered joints i.e.
      [ b_T_0 , b_T_1, ... , (b_T_j*j_T_f) ] (size = joints+1)
  */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> jacob_p_internal(const std::vector<Eigen::Isometry3d>& all_T) const;

  /*!
      Internal computation of the orientation part of the geometric jacobian in the frame {f}
      The input is a vector of all transformation the considered joints i.e.
      [ b_T_0 , b_T_1, ... , (b_T_j*j_T_f) ] (size = joints+1)
  */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic>
  jacob_o_geometric_internal(const std::vector<Eigen::Isometry3d>& all_T) const;

public:
  /*!
      Compute the position part of the jacobian in frame {f} w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      The matrix j_T_f defines the frame {f}: this is the transformation of frame {j} w.r.t. frame of the joint n_joint
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> jacob_p(const Eigen::VectorXd& q_DH, unsigned int n_joint,
                                                           const Eigen::Isometry3d& j_T_f) const;

  /*!
      Compute the position part of the jacobian in frame of joint n_joint w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> jacob_p(const Eigen::VectorXd& q_DH, unsigned int n_joint) const;

  /*!
      Compute the position part of the jacobian in frame {end-effector} w.r.t. base frame (pag 111)
  */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> jacob_p(const Eigen::VectorXd& q_DH) const;

  /*!
      Compute the orientation part of the geometric jacobian in frame {f} w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      The matrix j_T_f defines the frame {f}: this is the transformation of frame {j} w.r.t. frame of the joint n_joint
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> jacob_o_geometric(const Eigen::VectorXd& q_DH, unsigned int n_joint,
                                                                     const Eigen::Isometry3d& j_T_f) const;

  /*!
      Compute the orientation part of the geometric jacobian in frame of joint n_joint w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> jacob_o_geometric(const Eigen::VectorXd& q_DH,
                                                                     unsigned int n_joint) const;

  /*!
      Compute the orientation part of the geometric jacobian in frame {end-effector} w.r.t. base frame (pag 111)
  */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> jacob_o_geometric(const Eigen::VectorXd& q_DH) const;

  /*!
      Compute the geometric jacobian in frame {f} w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      The matrix j_T_f defines the frame {f}: this is the transformation of frame {j} w.r.t. frame of the joint n_joint
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic> jacob_geometric(const Eigen::VectorXd& q_DH, unsigned int n_joint,
                                                                   const Eigen::Isometry3d& j_T_f) const;

  /*!
      Compute the geometric jacobian in frame of joint n_joint w.r.t. base frame (pag 111)
      The jacobian is computed using the first n_joint joints.
      If n_joint is n_joint+1 the frame {end-effector} is considered as frame of the last joint
  */
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic> jacob_geometric(const Eigen::VectorXd& q_DH,
                                                                   unsigned int n_joint) const;

  /*!
      Compute the geometric jacobian in frame {end-effector} w.r.t. base frame (pag 111)
  */
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic> jacob_geometric(const Eigen::VectorXd& q_DH) const;

  /*!
      Ginven the jacobian b_J in frame {b} and the rotation matrix u_R_b of frame {b} w.r.t. frame {u},
      compute the jacobian w.r.t frame {u} (pag 113)
      The jacobian b_J can be the position part (3xQ), the orientation part (3xQ) or the full jacobian (6xQ)
  */
  static Eigen::MatrixXd change_jacob_frame(Eigen::MatrixXd b_J, const Eigen::Matrix3d& u_R_b);

  /*========END Jacobians=========*/

};  // END CLASS

/*==========Operators========*/

/*!
  overloaded operator +

  Construct a new Robot object with link1 as the first link and link2 as second link
*/
SerialLink operator+(const std::shared_ptr<Link>& link1, const std::shared_ptr<Link>& link2);

}  // namespace uclv::robot
