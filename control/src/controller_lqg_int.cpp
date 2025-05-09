//utility
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include <thread>
#include <chrono>
#include <cstring>
//interfacce
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

//Eigen
#include <Eigen/Dense>


class ControllerLQR : public rclcpp::Node
{
    protected:
        //Publisher degli angoli calcolati dall'equazione alle differenze (angolo y è quello intorno alle y e idem per la z)
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_posa_;

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

        std::vector<double> K_new= {-2.0804, -0.9602,0.7068, 1.4486, 0.1002, 0.0963, 0.0924, 0.0885};
        Eigen::Map<Eigen::VectorXd> K_new_ = Eigen::Map<Eigen::VectorXd>(K_new.data(), K_new.size());

        std::vector<double> K_int= {-1.7778};
        Eigen::Map<Eigen::VectorXd> K_int_ = Eigen::Map<Eigen::VectorXd>(K_int.data(), K_int.size());

        //variabile incrementale
        int p=0;
        double tau=0.5;

        //variabili per tustin
        double z_int_old=0;
        double y_int_old=0;
        double t_camp=0.017;
        double error_y_old=0;
        double error_z_old=0;
        double y_des=0;
        double z_des=0;

        //Per i parametri
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        int realtime_priority_;
        std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> cb_handles_;
        
    public:

        ControllerLQR(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("ControllerLQR", opt)
        { 
            using namespace std::placeholders;
            // Dichiarazione dei parametri e controllo sul realtime kernel
            declare_ros_parameters();
            initRealTime();

            auto qos=rclcpp::SensorDataQoS();
            qos.keep_last(1);

            // Il publisher per i punti della traiettoria su /desired_pose
            pub_posa_=this->create_publisher<geometry_msgs::msg::PointStamped>("/desired_pose",qos);
            abort_pub_=this->create_publisher<std_msgs::msg::String>("/abort",qos);
            subscriber_pos = this->create_subscription<geometry_msgs::msg::PointStamped>("ball_position", qos, std::bind(&ControllerLQR::position_callback, this, _1));

            //inizializzo il server
            start_controller_ = this->create_service<std_srvs::srv::SetBool>("controller/start", std::bind(&ControllerLQR::controller_cb_start, this, _1, _2));

            //inizializzo il timer
            timer_= this->create_wall_timer(std::chrono::milliseconds(int(17)), std::bind(&ControllerLQR::controllo_cb, this));
            timer_->cancel();

            //inizializzo a zero le variabili di errore e di angolo
            phi=0;
            theta=0;

            //inizalizzo il sistema per Kalman
            inizializza_sistema_Kalman();

        }

        ~ControllerLQR() = default;

    protected:

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
                    error_z_old=z_des-z;
                    error_y_old=y_des-y;
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
            
            double soft_start_weight=1-exp(-(p*t_camp)/tau);
            p++;
            //theta=Kalman(0,smoother);
            //phi=Kalman(1,smoother);
            phi = Kalman(&xhat_y, &Phat_y, &S_y, &K_gain_y, &yhat, &y, &y_des, &y_int_old, soft_start_weight, &error_y_old);
            theta = Kalman(&xhat_z, &Phat_z, &S_z, &K_gain_z, &zhat, &z, &z_des,&z_int_old, soft_start_weight, &error_z_old);

            geometry_msgs::msg::PointStamped out_msg;
            out_msg.point.y=phi;
            out_msg.point.z=theta;
            pub_posa_->publish(out_msg);
        }

        void controller_cb_start(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)
        {
            using namespace std::chrono_literals;
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


        double Kalman(Eigen::VectorXd* xhat, Eigen::MatrixXd* Phat, Eigen::MatrixXd* S, Eigen::MatrixXd* K_gain, Eigen::VectorXd* outhat, float* measurement, double* desired, double* int_old, double soft_start_weight,double* error_old)
        {
            Eigen::VectorXd measurement_vec(1);
            measurement_vec(0) = *measurement;
            double error = *desired - *measurement;
            double integrate = *int_old + (t_camp / 2.0) * (error + *error_old);
            *int_old = integrate;
            *error_old = error;

            //auto xhat_temp = *xhat;
            (*xhat)[0] -= *desired;
            Eigen::VectorXd int_vec(1);
            int_vec(0) = integrate;
            auto u1=-K_new_.dot(*xhat);
            auto u2=K_int_.dot(int_vec);
            auto u=u1+u2;
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

        /*double Kalman(int mode, double smoother)
        {
            if(mode==0)
            {
                Eigen::VectorXd y_vec(1);
                y_vec(0) = y;
                double error_y = y_des - y;
                double y_int = y_int_old + (t_camp / 2.0) * (error_y + error_y_old);
                y_int_old = y_int;
                error_y_old = error_y;
                Eigen::VectorXd y_int_vec(1);
                y_int_vec(0) = y_int;
                auto u1=-K_new_.dot(xhat_y);
                auto u2=K_int_.dot(y_int_vec);
                auto u=u1+u2;
                u=u*smoother;
              
                // Prediction step
                xhat_y = GdA * xhat_y + GdB * u;
                Phat_y = GdA * Phat_y * GdA.transpose() + W;
                yhat = GdC * xhat_y + GdD * u;

                // Update step
                S_y = GdC * Phat_y * GdC.transpose()+ V;
                K_gain_y= Phat_y * GdC.transpose() * S_y.inverse();
                
                // Update the state estimate and covariance
                xhat_y = xhat_y + K_gain_y * (y_vec - yhat);
                Phat_y = Phat_y-K_gain_y*GdC*Phat_y;

                
                return u;
            }
            else if(mode==1)
            {
                Eigen::VectorXd z_vec(1);
                z_vec(0) = z;
                double error_z = z_des - z;
                double z_int = z_int_old + (t_camp / 2.0) * (error_z + error_z_old);
                z_int_old = z_int;
                error_z_old = error_z;


                Eigen::VectorXd z_int_vec(1);
                z_int_vec(0) = z_int;
                auto u1=-K_new_.dot(xhat_z);
                auto u2=K_int_.dot(z_int_vec);
                auto u=u1+u2;
                u=u*smoother;
                // Prediction step
                xhat_z = GdA * xhat_z + GdB * u;
                Phat_z = GdA * Phat_z * GdA.transpose() + W;
                zhat = GdC * xhat_z + GdD * u;

                // Update step
                S_z = GdC * Phat_z * GdC.transpose()+ V;
                K_gain_z= Phat_z * GdC.transpose() * S_z.inverse();
                
                // Update the state estimate and covariance
                xhat_z = xhat_z + K_gain_z * (z_vec - zhat);
                Phat_z = Phat_z-K_gain_z*GdC*Phat_z;
                
                return u;
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(),"mode non valido");
                return 0;
            }

        }*/


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
  auto node = std::make_shared<ControllerLQR>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}