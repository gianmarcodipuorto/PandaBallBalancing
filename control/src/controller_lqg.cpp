//utility
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include <thread>
#include <chrono>
#include <cstring>
#include <string>

//interfacce
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

//Eigen
#include <Eigen/Dense>


class ControllerLQG : public rclcpp::Node
{
    protected:
        //Publisher degli angoli calcolati dall'equazione alle differenze (angolo y è quello intorno alle y e idem per la z)
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_posa_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_des_;

        //subscriber per il topic /position con all'interno la posizione della y e delle z
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_pos;

        //servizio per avviare il controllore
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_controller_;

        //publisher per fermare tutto
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr abort_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        //variabile di utility
        bool start=false;
        bool ricevuto=false;
        double t_camp=0.017;

        float phi;
        float theta;
        //Matrici per Kalman
        Eigen::MatrixXd GdA = Eigen::MatrixXd::Zero(8, 8);
        Eigen::MatrixXd GdB = Eigen::MatrixXd::Zero(8, 1);
        Eigen::MatrixXd GdC = Eigen::MatrixXd::Zero(1, 8);
        Eigen::MatrixXd GdD = Eigen::MatrixXd::Zero(1, 1);
        Eigen::MatrixXd W = Eigen::MatrixXd::Zero(8, 8);
        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(1, 1);

        Eigen::MatrixXd Phat_y = Eigen::MatrixXd::Zero(8, 8);
        Eigen::MatrixXd S_y = Eigen::MatrixXd::Zero(1, 1);
        Eigen::MatrixXd K_gain_y = Eigen::MatrixXd::Zero(8, 8);
        Eigen::VectorXd xhat_y = Eigen::VectorXd::Zero(8);
        Eigen::VectorXd yhat = Eigen::VectorXd::Zero(1);

        Eigen::MatrixXd Phat_z = Eigen::MatrixXd::Zero(8, 8);
        Eigen::MatrixXd S_z = Eigen::MatrixXd::Zero(1, 1);
        Eigen::MatrixXd K_gain_z = Eigen::MatrixXd::Zero(8, 8);
        Eigen::VectorXd xhat_z = Eigen::VectorXd::Zero(8);
        Eigen::VectorXd zhat = Eigen::VectorXd::Zero(1);
        float z;
        float y;
        std::vector<double> K= {-4.1615, -1.6530, 1.1867, 2.4560, 0.1677, 0.1593, 0.1509, 0.1425};
        //test
        //std::vector<double> K= {-13.9635, -3.6076, 2.3767, 5.0863, 0.3317, 0.3035, 0.2755, 0.2476}; //Q=100000,1,1,1,0,0,0,0 R=400 test 1
        //std::vector<double> K= {-14.8670, -3.7738, 2.4748, 5.3060, 0.3451, 0.3152, 0.2853, 0.2557}; //Q=200000,1,1,1,0,0,0,0 R=700 test 2
        //std::vector<double> K= {-8.9685, -2.6400, 1.7938, 3.7921, 0.2515, 0.2334, 0.2153, 0.1974};  //Q=10000,0,1,1,1,1,0,0 R=100 test 3
        //std::vector<double> K= {-10.7232, -2.9906, 2.0074, 4.2641, 0.2809, 0.2593, 0.2377, 0.2163};   //Q=10000,0,1,1,1,1,0,0 R=700 test 4

        Eigen::Map<Eigen::VectorXd> K_lqr = Eigen::Map<Eigen::VectorXd>(K.data(), K.size());

        //variabile incrementale
        int q=0;
        int p=0;
        double tau=0.5;

        //per la circonferenza
        double y_des=0;
        double z_des=0;
        double vel_y_des=0;
        double vel_z_des=0;

        //per l'onda quadra
        bool onda=false;
        double y_des_filtered=y_des;
        double z_des_filtered=z_des;

        //Per i parametri
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        int realtime_priority_;
        std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> cb_handles_;
        
    public:

        ControllerLQG(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("ControllerLQG", opt)
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
            subscriber_pos = this->create_subscription<geometry_msgs::msg::PointStamped>("ball_position", qos, std::bind(&ControllerLQG::position_callback, this, _1));

            //inizializzo il server
            start_controller_ = this->create_service<std_srvs::srv::SetBool>("controller/start", std::bind(&ControllerLQG::controller_cb_start, this, _1, _2));

            //inizializzo il timer
            timer_= this->create_wall_timer(std::chrono::milliseconds(int(17)), std::bind(&ControllerLQG::controllo_cb, this));
            timer_->cancel();

            //inizializzo a zero le variabili di errore e di angolo
            phi=0;
            theta=0;

            //inizalizzo il sistema per Kalman
            inizializza_sistema_Kalman();

        }

        ~ControllerLQG() = default;

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
            start=true;
            res->success=true;
            res->message="start controller";
        }

        void position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
        {
            if(start){
                y=msg->point.y;
                z=msg->point.z;

                if(!ricevuto)
                {
                    ricevuto=true;
                    xhat_y[0]=y;
                    xhat_z[0]=z;
                    timer_->reset();

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
            
            // Aggiorna la traiettoria desiderata
            //update_desired_trajectory("square_wave");
            //update_desired_trajectory("circle");

            double soft_start_weight=1-exp(-(p*t_camp)/tau);

            geometry_msgs::msg::PointStamped des_msg;
            des_msg.point.y=y_des;
            des_msg.point.z=z_des;
            des_msg.header.stamp=this->now();
            pub_des_->publish(des_msg);
            
            p++;
            q++;
            //phi=Kalman(0,soft_start_weight);
            phi = Kalman(&xhat_y, &Phat_y, &S_y, &K_gain_y, &yhat, &y, &y_des_filtered, soft_start_weight, &vel_y_des);
            //theta=Kalman(1,soft_start_weight);
            theta = Kalman(&xhat_z, &Phat_z, &S_z, &K_gain_z, &zhat, &z, &z_des_filtered, soft_start_weight, &vel_z_des);

            geometry_msgs::msg::PointStamped out_msg;
            out_msg.point.y=theta;
            out_msg.point.z=phi;
            out_msg.header.stamp=this->now();

            pub_posa_->publish(out_msg);
        }
        
        void update_desired_trajectory(std::string trajectory_type)
        {
            // Aggiorna la traiettoria desiderata in base al tipo di traiettoria
            if (trajectory_type == "square_wave")
            {
                double alpha = t_camp / (0.5 + t_camp); // Coefficiente del filtro
                //onda quadra
                if(q*t_camp>=10.0)
                {
                    if(onda==true)
                    {
                        y_des=0.04;
                        onda=false;
                        q=0;
                        //p=0;
                    }else
                    {
                        y_des=-0.04;
                        onda=true;
                        q=0;
                        //p=0;
                    }
                }
                // Calcola i valori desiderati intermedi
                y_des_filtered += alpha * (y_des - y_des_filtered);
                z_des_filtered += alpha * (z_des - z_des_filtered);
            }
            else if (trajectory_type == "circle")
            {
                y_des=0.05*cos(2*M_PI*0.05*p*t_camp);
                z_des=0.05*sin(2*M_PI*0.05*p*t_camp);
                vel_y_des=-0.05*2*M_PI*0.05*sin(2*M_PI*0.05*p*t_camp);
                vel_z_des=0.05*2*M_PI*0.05*cos(2*M_PI*0.05*p*t_camp);
                y_des_filtered = y_des;
                z_des_filtered = z_des;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Tipo di traiettoria non riconosciuto");
            }
        }


        double Kalman(Eigen::VectorXd* xhat, Eigen::MatrixXd* Phat, Eigen::MatrixXd* S, Eigen::MatrixXd* K_gain, Eigen::VectorXd* outhat,  float* measurement, double* desired, double soft_start_weight, double* vel_des)
        {
            Eigen::VectorXd measurement_vec(1);
            measurement_vec(0) = *measurement;
            auto xhat_temp = *xhat;
            (*xhat)[0]-=*desired; 
            (*xhat)[1]-=*vel_des;
            auto u=-K_lqr.dot(*xhat); 
            (*xhat)[0]+=*desired;
            (*xhat)[1]+=*vel_des;
            //double u=-K_lqr.dot(xhat_temp);
            u=u*soft_start_weight;
            // Prediction step
            *xhat = GdA * (*xhat) + GdB * u;
            *Phat = GdA * (*Phat) * GdA.transpose() + W;
            *outhat = GdC * (*xhat) + GdD * u;

            // Update step
            *S = GdC * (*Phat) * GdC.transpose()+ V;
            *K_gain= (*Phat) * GdC.transpose() * S->inverse();
            
            // Update the state estimate and covariance
            *xhat = *xhat + (*K_gain) * (measurement_vec - *outhat);
            *Phat = *Phat - (*K_gain) * GdC * (*Phat);

            return u;
        }


        void inizializza_sistema_Kalman()
        {
            GdA(0,0)=1;
            GdA(0,1)=0.0170000000000000;
            GdA(0,3)=-0.00465318080357143;
            GdA(1,1)=1;
            GdA(1,3)=-0.547433035714286;
            GdA(2,2)=0.0730621448838791;
            GdA(2,3)=-0.270971779676069; 
            GdA(2,4)=0.0589634592575126;
            GdA(3,2)=0.471707674060101;
            GdA(3,3)=0.801434288653152;
            GdA(3,4)=0.043207898789074;
            GdA(4,5)=1;
            GdA(5,6)=1;
            GdA(6,7)=1;
            
            GdB(7,0)=1;

            GdC(0,0)=1;

            GdD(0,0)=0;

            W.diagonal().setConstant(1e-8);
            V(0,0)=1e-6;
        }

        void initRealTime()        
        {
            if ((realtime_priority_ > 0) && realtime_tools::has_realtime_kernel())
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Realtime kernel detected and priority is set.");
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Set REALTIME to " << realtime_priority_);
            if (!realtime_tools::configure_sched_fifo(realtime_priority_))
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "SET REALTIME FAILED ");
                throw std::runtime_error("REALTIME REQUIRED, BUT FAILED TO SET");
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
  auto node = std::make_shared<ControllerLQG>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}