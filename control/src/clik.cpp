//utility
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <uclv_robot_lib/robots/FrankaEmikaPanda.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include <stdio.h>
#include <string.h>
#include <cstring>
#include <thread>
#include <chrono>

//interfacce
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"

//Eigen
#include <Eigen/Dense>

// tf2
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


Eigen::Quaterniond quaternionContinuity(const Eigen::Quaterniond& q, const Eigen::Quaterniond& oldQ)
{
  auto tmp = q.vec().transpose() * oldQ.vec();
  if (tmp < -0.01)
  {
    Eigen::Quaterniond out(q);
    out.vec() = -out.vec();
    out.w() = -out.w();
    return out;
  }
  return q;
}

class CLIKNode : public rclcpp::Node
{
  protected:
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
      rclcpp::TimerBase::SharedPtr clik_timer_;
      rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_clik_;

      // Variabili di controllo
      double Ts_ = 0.017;             // <-- periodo di campionamento del clik
      double clik_gain_ = 60;        // <-- guadagno del clik

      //definizione del robot
      std::shared_ptr<uclv::robot::SerialLink> robot_;

      //rappresenta la posizione "interna" del robot, al generico passo
      sensor_msgs::msg::JointState joint_current;

      // oldQ usato per la continuità del quaternione, inizializzato all'identità
      Eigen::Quaterniond oldQuaternion_ = Eigen::Quaterniond::Identity();

      //trasformata terna racchetta in terna flangia
      Eigen::Matrix4d T_7_racchetta_clik;   //costante

      //Definisco il vettore per la posizione desiderata    
      Eigen::Vector3d position_des_;
      Eigen::Quaterniond quaternion_des_;

      //subscriber per il topic /desired_pose
      rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_posa_;

      //subscriber per il topic di abort
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_abort_;

      //Salvo gli angoli desiderati
      geometry_msgs::msg::PointStamped angoli_desiderati;
      rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr angoli_des_;

      geometry_msgs::msg::PointStamped angoli_attuati;
      rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr angoli_act_;

      geometry_msgs::msg::PointStamped angoli_reali;
      rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr angoli_real_;

      //Per i parametri
      std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
      int realtime_priority_ = 1;
      std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> cb_handles_;
      std::vector<double> joint_vel_limits_;

      //Parte ereditata dal controller
      //trasformate necessarie all'eleaborazione
      Eigen::Matrix4d T_7_racchetta_con;
      Eigen::Matrix4d b_T_rDesiderata;
      Eigen::Matrix4d b_T_rinit;
      Eigen::Matrix3d b_R_rinit;

      //Posizione iniziale
      sensor_msgs::msg::JointState q_init;

      //tf2
      std::unique_ptr<tf2_ros::Buffer> tf_buffer;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener; 

      //variabili utili
      int p=16;
      int n=0;
      std_msgs::msg::Float64MultiArray out_msg;

      //Variabili per il test
      sensor_msgs::msg::JointState joint_real;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;

      //Variabili per il filtro
      double alpha;
      double beta;
      double t_pub=0.001;
      double tau_f=0.00159;
      std_msgs::msg::Float64MultiArray old_out_msg;
      std_msgs::msg::Float64MultiArray old_out_msg_filtered;
      std_msgs::msg::Float64MultiArray out_msg_filtered;


    public:
      CLIKNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("clik", opt)
      { 
          // Dichiarazione dei parametri e controllo sul realtime kernel
          declare_ros_parameters();
          initRealTime();
          
          auto qos=rclcpp::SensorDataQoS();
          qos.keep_last(1);
          
          using namespace std::placeholders;

          //Definizione della trasformata della racchetta rispetto alla flangia quella del clik
          T_7_racchetta_clik<<  1, 0, 0, 0, 
                                0, 1, 0, 0,
                                0, 0, 1, 0.197,
                                0, 0, 0, 1.0; 
          Eigen::Isometry3d _7_T_racchetta=Eigen::Isometry3d(T_7_racchetta_clik);
          
          //Definizione del robot tramite libreria
          robot_ = std::make_shared<uclv::robot::FrankaEmikaPanda>(_7_T_racchetta,"Panda");

          // Il publisher per i comandi in velocità
          cmd_pub_=this->create_publisher<sensor_msgs::msg::JointState>("/cmd/joint_position",qos);
          vel_pub_=this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_velocity_controller/command",qos);
          angoli_des_=this->create_publisher<geometry_msgs::msg::PointStamped>("angoli_des_",qos);
          angoli_act_=this->create_publisher<geometry_msgs::msg::PointStamped>("angoli_act_",qos);
          angoli_real_=this->create_publisher<geometry_msgs::msg::PointStamped>("angoli_real_",qos);

          // timer of the main control loop (uso un timer che garantisce la giusta frequenza al clik)
          clik_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1)), std::bind(&CLIKNode::clik_control_cycle, this));               
          clik_timer_->cancel(); //non avvio il timer del clik finchè non ho letto la configurazione dei giunti iniziale
          
          //inizializzo il server
          start_clik_ = this->create_service<std_srvs::srv::SetBool>("clik/start_stop", std::bind(&CLIKNode::set_clik_, this, _1, _2));

          //Subscriber per il topic /desired_pose
          sub_posa_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/desired_pose", qos, std::bind(&CLIKNode::sub_callback, this, _1));
          
          //Subscriber per il topic di abort
          sub_abort_ = this->create_subscription<std_msgs::msg::String>("/abort", qos, std::bind(&CLIKNode::abort, this, _1));

          //parte presa dal controller
          T_7_racchetta_con<< 1, 0, 0, 0, 
                              0, 1, 0, 0,
                              0, 0, 1, 0.304,
                              0, 0, 0, 1.0; 
          //giunti iniziali
          q_init.position.resize(7);
      
          //istanzio elementi per l'uso del topic tf
          tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
          tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

          out_msg.data.resize(7);
          //out_msg.data=[0,0,0,0,0,0,0];

          old_out_msg.data.resize(7);
          out_msg_filtered.data.resize(7);
          old_out_msg_filtered.data.resize(7);
          alpha=t_pub/(t_pub+2*tau_f); //0.384615384615385
          beta= (t_pub-2*tau_f)/(t_pub+2*tau_f); //-0.230769230769231;

          sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>("/franka_state_controller/joint_states", qos, std::bind(&CLIKNode::joint_callback, this, _1));
          angoli_desiderati.point.y=0;
          angoli_desiderati.point.z=0;
          RCLCPP_INFO_STREAM(this->get_logger(), "CLIK Node started");


      }

      ~CLIKNode() = default;

    protected:
            
      void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
      {
          joint_real=*msg;
      }
      
      //callback per leggere la posa che viene pubblicata dal nodo di controllo e che deve essere attuata sul robot
      void sub_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
      {
          angoli_desiderati.point.y=msg->point.y;
          angoli_desiderati.point.z=msg->point.z;
      }

      //callback per arrestare il movimento perchè la pallina è caduta
      void abort(const std_msgs::msg::String::SharedPtr msg)
      {
        std::cout<<msg->data<<std::endl;
        clik_timer_->cancel();
        abort_inverse_kinematics(joint_current.position.size());
      }
      
      //callback per inizializzare il clik che inizialmente è disattivato
      void set_clik_(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)
      {
        using namespace std::chrono_literals;
        if(req->data==false)
        {
            clik_timer_->cancel();
            RCLCPP_INFO_STREAM(this->get_logger(),"clik fermato");
            res->message="stop";
            res->success=true;
            return;
        }

        //se viene chiamato con true
        if (!rclcpp::wait_for_message<sensor_msgs::msg::JointState>(joint_current, shared_from_this(), "/franka_state_controller/joint_states", 10s))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current joint state");
            res->message="stop";
            res->success=false;
            return;
        }

        Eigen::Map<const Eigen::VectorXd> q_R(joint_current.position.data(), joint_current.position.size());
        auto q_DH = robot_->joints_Robot2DH(q_R);
  
        //Roba del controller
        q_init.position=joint_current.position;
        tf2_msgs::msg::TFMessage trasformata; 
        trasformata.transforms.resize(1);
        //calcolo la trasformata
        try
        {
            trasformata.transforms[0] = tf_buffer->lookupTransform("panda_link0","panda_link7", tf2::TimePointZero, 5s);
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "ERRORE NEL CALCOLO DELLA 1° TRASFORMATA " << ex.what());
        }
        Eigen::Quaterniond b_quat_rinit(trasformata.transforms[0].transform.rotation.w,trasformata.transforms[0].transform.rotation.x,trasformata.transforms[0].transform.rotation.y,trasformata.transforms[0].transform.rotation.z);
        b_quat_rinit.normalize();
        Eigen::Matrix3d b_R_7init=b_quat_rinit.toRotationMatrix();
        Eigen::Vector3d b_p_7init(trasformata.transforms[0].transform.translation.x,trasformata.transforms[0].transform.translation.y,trasformata.transforms[0].transform.translation.z);

        Eigen::Matrix4d b_T_7init;
        b_T_7init.block<3,3>(0,0)=b_R_7init;
        b_T_7init.block<3,1>(0,3)=b_p_7init;
        b_T_7init.block<1,4>(3,0)=Eigen::Vector4d(0,0,0,1);

        b_T_rinit=b_T_7init*T_7_racchetta_con;
        b_R_rinit=b_T_rinit.block<3,3>(0,0);

        position_des_=b_T_rinit.block<3,1>(0,3);
        quaternion_des_=Eigen::Quaterniond(b_R_rinit);
        quaternion_des_.normalize();
        oldQuaternion_=quaternion_des_;

        //attivo il clik
        clik_timer_->reset();
        res->success=true;
        res->message="start";
      }

      void clik_control_cycle()
      {
        /*if(n>200&&n<500)
        {
          angoli_desiderati.point.y=0.002;
          angoli_desiderati.point.z=0.009;
        }*/

        /*double f1=M_PI/16; 
        double f2=M_PI/20;
        n++;
        if(n<10000)
          {
            angoli_desiderati.point.y=(M_PI/8)*sin(2*M_PI*f1*n*0.001);
            angoli_desiderati.point.z=(M_PI/8)*sin(2*M_PI*f2*n*0.001);
          }
          angoli_desiderati.header.stamp=this->now();
          angoli_des_->publish(angoli_desiderati);
        */

        p++;
        if(p%17==0)
        {
          p=0;
          angoli_desiderati.point.y=-angoli_desiderati.point.y;

          Eigen::AngleAxisd rotz(angoli_desiderati.point.z, Eigen::Vector3d::UnitZ());
          Eigen::AngleAxisd roty(angoli_desiderati.point.y, Eigen::Vector3d::UnitY());
          Eigen::AngleAxisd rotx(0, Eigen::Vector3d::UnitX());

          Eigen::Quaternion<double> quat = rotx * roty * rotz; 

          quat.normalize();
          //quat=quaternionContinuity(quat,oldQuaternion_);
          //quat.normalize();
          Eigen::Matrix3d rinit_R_des = quat.matrix();

          Eigen::Matrix3d b_R_rDesiderata=b_R_rinit*rinit_R_des;
          Eigen::Vector3d b_p_rDesiderata=b_T_rinit.block<3,1>(0,3);
          position_des_=b_p_rDesiderata;

          // Estraggo il quaternione
          Eigen::Quaterniond quaternion_temp(b_R_rDesiderata);
          quaternion_des_=quaternion_temp;
          // Assicuro la continuità del quaternion
          quaternion_des_.normalize();
          quaternion_des_ = quaternionContinuity(quaternion_des_, oldQuaternion_);
          quaternion_des_.normalize();


          Eigen::Map<const Eigen::VectorXd> q_R(joint_current.position.data(), joint_current.position.size());
          auto q_DH = robot_->joints_Robot2DH(q_R);
          const Eigen::Isometry3d& b_T_r = robot_->fkine(q_DH);
          
          Eigen::Vector3d position_temp = b_T_r.translation();
          Eigen::Matrix3d rotation_temp_ = b_T_r.rotation();

          Eigen::Matrix4d b_T_r_matrix;
          b_T_r_matrix.block<3,3>(0,0)=rotation_temp_;
          b_T_r_matrix.block<3,1>(0,3)=position_temp;
          b_T_r_matrix.block<1,4>(3,0)=Eigen::Vector4d(0,0,0,1);

          //Estraggo il quaternione
          Eigen::Quaterniond quaternion(rotation_temp_);
          quaternion.normalize();
          // Assicuro la continuità del quaternione
          quaternion = quaternionContinuity(quaternion, oldQuaternion_);
          quaternion.normalize();
          oldQuaternion_ = quaternion;  // <-- per il prossimo ciclo

          //Calcolo l'errore
          Eigen::Matrix<double, 6, 1> error;
          error.block<3, 1>(0, 0) = position_des_ - position_temp;

          // Errore in orientamento è la parte vettoriale di Qd*inv(Q)
          Eigen::Quaterniond deltaQ = quaternion_des_ * quaternion.inverse();
          deltaQ.normalize();

          // errore in orientamento (setto il blocco di dimensioni 3x1 che parte dalla posizione (3,0))
          error.block<3, 1>(3, 0) = deltaQ.vec();

          //vel_e è il termine (v_des + gamma*error) con v_des pari a zero
          Eigen::Matrix<double, 6, 1> vel_e =clik_gain_ * error;
          
          auto jacobian = robot_->jacob_geometric(q_DH);
          robot_->jacobian_DH2Robot(jacobian);
          
          // Calcolo pinv(jacobian)*vel_e, ovvero q_dot
          Eigen::VectorXd q_dot(7);
          q_dot = jacobian.completeOrthogonalDecomposition().solve(vel_e);
      
          // check velocity limits
          if ((Eigen::Index)joint_vel_limits_.size() != q_dot.size())
          {       
            abort_inverse_kinematics(q_dot.size());
          }
          for (int i = 0; i < q_dot.size(); i++)
          {                 
            if (fabs(q_dot(i)) > joint_vel_limits_[i])
            {
              abort_inverse_kinematics(q_dot.size());
            }
          }

          Eigen::VectorXd q(7);
          for(int i=0;i<7;i++){
            q[i]=joint_current.position[i];
          }
          
          //calcolo il valore della q interna del clik (mi serve per la successiva iterazione)
          q = q + q_dot * Ts_; 
          
          for(int i=0;i<q_dot.size();i++){
            joint_current.position[i]=q(i);
          }

          //cmd_pub_->publish(joint_current); //solo per provare sul simulatore poi bisogna toglierla        
                
          out_msg.data.resize(7);
          for(int i=0;i<q_dot.size();i++)
          {
            out_msg.data[i]=q_dot(i);
          }

        }   
        
        calcolo_angoli_attuati(joint_current,1);
        angoli_attuati.header.stamp=this->now();
        angoli_act_->publish(angoli_attuati);

        calcolo_angoli_attuati(joint_real,2);
        angoli_reali.header.stamp=joint_real.header.stamp;
        angoli_real_->publish(angoli_reali);
        
        //Filtraggio del messaggio
        for(int i=0;i<(int)out_msg.data.size();i++)
        {
          out_msg_filtered.data[i]=alpha*(out_msg.data[i]+old_out_msg.data[i])-beta*old_out_msg_filtered.data[i];
        }
        old_out_msg.data=out_msg.data;
        old_out_msg_filtered.data=out_msg_filtered.data;

        vel_pub_->publish(out_msg_filtered);

      }



      void abort_inverse_kinematics(int num_joints)
      {
        // publish zero vel
        for (int i = 0; i < 5; i++)
        {
          auto vel_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
          vel_msg->data.resize(num_joints);
          
          for(int j=0;j<num_joints;j++)
          {
            vel_msg->data[j]=0;
          }
          vel_pub_->publish(*vel_msg);
        }
      }

      void calcolo_angoli_attuati(sensor_msgs::msg::JointState q, int n)
      {
        //ricevo i giunti attuali e calcolo gli angoli attuati con la cinematica diretta
        Eigen::Map<const Eigen::VectorXd> q_R(q.position.data(), q.position.size());
        auto q_DH = robot_->joints_Robot2DH(q_R);
        const Eigen::Isometry3d& b_T_r = robot_->fkine(q_DH);
        Eigen::Matrix3d rotation_temp_ = b_T_r.rotation();
        //calcolo la matrice di rotazione della base in terna racchetta iniziale (inversa di b_T_rinit)
        Eigen::Matrix3d rinit_R_b=b_R_rinit.transpose();
        
        Eigen::Matrix3d rinit_R_act = rinit_R_b * rotation_temp_;
        Eigen::Quaterniond quat(rinit_R_act);
        quat.normalize();

        if(n==1)
        {
          // Calcolo l'angolo attorno a Y
          angoli_attuati.point.y = std::asin(rinit_R_act(0, 2));
          // Calcolo l'angolo attorno a Z
          angoli_attuati.point.z = std::atan2(-rinit_R_act(0, 1), rinit_R_act(0, 0));

          // Stampo i valori degli angoli
          //RCLCPP_INFO(this->get_logger(), "Angolo Y (attuato): %f", angoli_attuati.point.y);
          //RCLCPP_INFO(this->get_logger(), "Angolo Z (attuato): %f", angoli_attuati.point.z);
        }

        if(n==2)
        {
          // Calcolo l'angolo attorno a Y
          angoli_reali.point.y = std::asin(rinit_R_act(0, 2));
          // Calcolo l'angolo attorno a Z
          angoli_reali.point.z = std::atan2(-rinit_R_act(0, 1), rinit_R_act(0, 0));
          // Stampo i valori degli angoli
          //RCLCPP_INFO(this->get_logger(), "Angolo Y (reale): %f", angoli_reali.point.y);
          //RCLCPP_INFO(this->get_logger(), "Angolo Z (reale): %f", angoli_reali.point.z);
        }
      }

      void initRealTime()
      {
        if ((realtime_priority_ > 0) && realtime_tools::has_realtime_kernel())
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "Trajectory generator set REALTIME to " << realtime_priority_);
          if (!realtime_tools::configure_sched_fifo(realtime_priority_))
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Trajectory generator SET REALTIME FAILED ");
            throw std::runtime_error("REALTIME REQUIRED, BUT FAILED TO SET");
          }
          RCLCPP_INFO_STREAM(this->get_logger(), "Trajectory generator SET REALTIME OK ");
        }
      }

      void declare_ros_parameters()
        {
          param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
          cb_handles_.clear();

          {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "realtime_priority";
            desc.description = "Realtime priority, if <0 (default) no realtime is explicitely set";
            desc.additional_constraints = "";
            desc.read_only = true;
            desc.integer_range.resize(1);
            desc.integer_range[0].from_value = INT32_MIN;
            desc.integer_range[0].to_value = 99;
            desc.integer_range[0].step = 0;
            realtime_priority_ = this->declare_parameter(desc.name, 1, desc);
          }

          {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "joint_vel_limits";
            desc.description = "Joint vel limits on each joint. It is a mandatory parameter";
            desc.additional_constraints = "";
            desc.read_only = false; 
            this->declare_parameter(desc.name, std::vector<double>({2.1750,2.1750,2.1750,2.1750,2.61,2.61,2.61}), desc);
            cb_handles_.insert(
                { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
                  joint_vel_limits_ = p.as_double_array();
                  RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
                }) });
          }
        }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CLIKNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
