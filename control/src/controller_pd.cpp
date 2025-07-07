//utility
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include <thread>
#include <chrono>

//interfacce
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

//Eigen
#include <Eigen/Dense>

class ControllerNode : public rclcpp::Node
{
    protected:
        //Publisher degli angoli calcolati dall'equazione alle differenze (angolo y è quello intorno alle y e idem per la z)
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_posa_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_des_;
        
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_pos;

        //servizio per avviare il controllore
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_controller_;

        //publisher per fermare tutto
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr abort_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        //Variabili di stato

        float y;
        float z;
        Eigen::Vector4d error_y;
        Eigen::Vector4d error_z;
        Eigen::Vector4d phi;
        Eigen::Vector4d theta;
        float phi_temp;
        float theta_temp;
        float y_des;
        float z_des;
        
        //variabile di utility
        bool start=false;
        bool ricevuto=false;
        //variabile incrementale
        int p=0;

        //numerare e denominatore del controllore discreto
        Eigen::Vector4d num;
        Eigen::Vector4d den;

        //Per i parametri
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        int realtime_priority_;
        std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> cb_handles_;

    public:
        ControllerNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("ControllerNode", opt)
        { 
            using namespace std::placeholders;
            // Dichiarazione dei parametri e controllo sul realtime kernel
            declare_ros_parameters();
            initRealTime();

            auto qos=rclcpp::SensorDataQoS();
            qos.keep_last(1);

            // Il publisher per i punti della traiettoria su /desired_pose
            pub_posa_=this->create_publisher<geometry_msgs::msg::PointStamped>("/desired_pose",qos);
            pub_des_=this->create_publisher<geometry_msgs::msg::PointStamped>("/des_traj",qos);


            abort_pub_=this->create_publisher<std_msgs::msg::String>("/abort",qos);

            subscriber_pos = this->create_subscription<geometry_msgs::msg::PointStamped>("ball_position", qos, std::bind(&ControllerNode::position_callback, this, _1));

            //inizializzo il server
            start_controller_ = this->create_service<std_srvs::srv::SetBool>("controller/start", std::bind(&ControllerNode::controller_cb_start, this, _1, _2));

            timer_= this->create_wall_timer(std::chrono::milliseconds(int(17)), std::bind(&ControllerNode::controllo_cb, this));
            timer_->cancel();

            y_des=0;
            z_des=0;

            //inizializzo a zero le variabili di errore e di angolo
            error_y.setZero();
            error_z.setZero();
            phi.setZero();
            theta.setZero();   
            
            //inizializzo numeratore e denominatore per il controllore a 17 ms
            num<<-0.01984,-0.01994,0.01966,0.01975;
            den<<1,-2.53,2.134,-0.5998; 
        }

        ~ControllerNode() = default;

    protected:
        void controller_cb_start(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)
        {
            if(req->data==false)
            {
                start=false;
                RCLCPP_INFO_STREAM(this->get_logger(),"controllo fermato");
                res->message="stop";
                res->success=true;
                return;
            }
            timer_->reset();
            start=true;
            res->success=true;
            res->message="start controller";
        }

        void position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
        {
            if(start){
                y=msg->point.y;
                z=msg->point.z;
                //stampo la posizione della pallina
                //RCLCPP_INFO_STREAM(this->get_logger(),"y: "<<y);
                //RCLCPP_INFO_STREAM(this->get_logger(),"z: "<<z);
                if(!ricevuto)
                {
                    ricevuto=true;
                    error_y[0]=y_des-y;
                    error_z[0]=z_des-z;
                }
                if(sqrt(y * y + z * z) > 0.10) //if(fabs(y)>0.10||fabs(z)>0.10)
                {   
                    //stampo la posizione della pallina
                    RCLCPP_INFO_STREAM(this->get_logger(),"y: "<<y);
                    RCLCPP_INFO_STREAM(this->get_logger(),"z: "<<z);
                    auto message=std_msgs::msg::String();
                    message.data="STOP!";
                    start=false;
                    abort_pub_->publish(message);
                }
            } 
        }

        void controllo_cb()
        { 

            geometry_msgs::msg::PointStamped des_msg;
            des_msg.point.y=y_des;
            des_msg.point.z=z_des;
            des_msg.header.stamp=this->now();
            pub_des_->publish(des_msg);
            using namespace std::chrono_literals;
            if(p==0)
            {
                error_y[0]=y_des-y;
                error_z[0]=z_des-z;

                phi[0]=num[0]*error_y[0];
                theta[0]=num[0]*error_z[0];
                p++;
                phi_temp=phi[0];
                theta_temp=theta[0];
            }
            else if(p==1)
            {
                error_y[1]=y_des-y;
                error_z[1]=z_des-z;

                phi[1]=-den[1]*phi[0]+num[0]*error_y[1]+num[1]*error_y[0];
                theta[1]=-den[1]*theta[0]+num[0]*error_z[1]+num[1]*error_z[0];
                p++;
                phi_temp=phi[1];
                theta_temp=theta[1];
            }
            else if(p==2)
            {
                error_y[2]=y_des-y;
                error_z[2]=z_des-z;

                phi[2]=-den[1]*phi[1]-den[2]*phi[0]+num[0]*error_y[2]+num[1]*error_y[1]+num[2]*error_y[0];
                theta[2]=-den[1]*theta[1]-den[2]*theta[0]+num[0]*error_z[2]+num[1]*error_z[1]+num[2]*error_z[0];
                p++;
                phi_temp=phi[2];
                theta_temp=theta[2];
            }
            else if(p==3)
            {
                error_y[3]=y_des-y;
                error_z[3]=z_des-z;

                phi[3]=-den[1]*phi[2]-den[2]*phi[1]-den[3]*phi[0]+num[0]*error_y[3]+num[1]*error_y[2]+num[2]*error_y[1]+num[3]*error_y[0];
                theta[3]=-den[1]*theta[2]-den[2]*theta[1]-den[3]*theta[0]+num[0]*error_z[3]+num[1]*error_z[2]+num[2]*error_z[1]+num[3]*error_z[0];
                p++;
                phi_temp=phi[3];
                theta_temp=theta[3];
            }
            else
            {
                for(int i=1; i<4;i++)
                {
                    error_y[i-1]=error_y[i];
                    error_z[i-1]=error_z[i];
                    phi[i-1]=phi[i];
                    theta[i-1]=theta[i];
                }
                error_y[3]=y_des-y;
                error_z[3]=z_des-z;

                phi[3]=-den[1]*phi[2]-den[2]*phi[1]-den[3]*phi[0]+num[0]*error_y[3]+num[1]*error_y[2]+num[2]*error_y[1]+num[3]*error_y[0];
                theta[3]=-den[1]*theta[2]-den[2]*theta[1]-den[3]*theta[0]+num[0]*error_z[3]+num[1]*error_z[2]+num[2]*error_z[1]+num[3]*error_z[0];
                phi_temp=phi[3];
                theta_temp=theta[3];
            }
        
            geometry_msgs::msg::PointStamped out_msg;
            out_msg.header.stamp = this->now();
            out_msg.point.y=theta_temp;
            out_msg.point.z=phi_temp;
            pub_posa_->publish(out_msg);
        }

        void initRealTime()
        {
            if ((realtime_priority_ > 0) && realtime_tools::has_realtime_kernel())
            {
            RCLCPP_INFO_STREAM(this->get_logger(), "set REALTIME to " << realtime_priority_);
            if (!realtime_tools::configure_sched_fifo(realtime_priority_))
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "SET REALTIME FAILED ");
                throw std::runtime_error("REALTIME REQUIRED, BUT FAILED TO SET");
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "SET REALTIME OK ");
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
            }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}