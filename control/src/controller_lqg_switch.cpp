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
#include "geometry_msgs/msg/point_stamped.hpp"
#include "interfaces/srv/set_des.hpp"

//Eigen
#include <Eigen/Dense>

//costanti
constexpr double MAX_POSITION = 0.1; // Valore massimo per y e z
constexpr double SAMPLING_TIME = 0.017; // Tempo di campionamento

class ControllerLQR : public rclcpp::Node
{
    protected:
        //Publisher degli angoli calcolati dall'equazione alle differenze (angolo y è quello intorno alle y e idem per la z)
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_posa_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_des_;

        //subscriber per il topic /position con all'interno la posizione della y e delle z
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_pos;

        //servizio per avviare il controllore
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_controller_;

        //servizio per cambiare il setPoint
        rclcpp::Service<interfaces::srv::SetDes>::SharedPtr set_des_;

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
        std::vector<double> K_new= {-2.0804, -0.9602,0.7068, 1.4486, 0.1002, 0.0963, 0.0924, 0.0885}; // Q=1000,0,1,1,1,1,0,0,1 R=1000
        //test
        //std::vector<double> K_new= {-6.4000, -2.0624, 1.4303, 2.9993, 0.2012, 0.1885, 0.1758, 0.1633};   // Q=40000,0,1,1,1,1,0,0,1 R=1000 test1
        //std::vector<double> K_new= {-10.4694, -2.8526, 1.9122, 4.0628, 0.2677, 0.2472, 0.2270, 0.2071};  //Q= 10000,0,1,1,1,1,0,0,1 R=100  test2
        //std::vector<double> K_new= {-2.1422, -1.0384,0.7683, 1.5718, 0.1090, 0.1048, 0.1006, 0.0964}; // Q=1000,100,1,1,0,0,0,0,1 R=1000  test3
        //std::vector<double> K_new= {-2.463759821338347, -1.502920737546973,1.134740653012466, 2.305680133240116, 0.161305336866996, 0.155776073130742, 0.149661854010936, 0.142527206049657}; // Q=1000,1000,1,1,0,0,0,0,1 R=1000 test4
        Eigen::Map<Eigen::VectorXd> K_new_ = Eigen::Map<Eigen::VectorXd>(K_new.data(), K_new.size());

        std::vector<double> K_int= {-1.7778};   // Q=1000,0,1,1,1,1,0,0,1 R=1000
        //test
        //std::vector<double> K_int= {-1.7124}; / Q=40000,0,1,1,1,1,0,0,1 R=1000   test1
        //std::vector<double> K_int= {-5.2500}; //Q= 10000,0,1,1,1,1,0,0,1 R=100    test2
        //std::vector<double> K_int= {-1.731208663999560};   // Q=1000,1000,1,1,0,0,0,0,1 R=1000  test3
        Eigen::Map<Eigen::VectorXd> K_int_ = Eigen::Map<Eigen::VectorXd>(K_int.data(), K_int.size());

        //variabile incrementale
        int p=0;
        int q=0;
        double tau=0.5;

        //variabili per tustin
        double z_int_old=0;
        double y_int_old=0;
        double t_camp=0.017;
        double error_y_old=0;
        double error_z_old=0;
        double y_des=0;
        double z_des=0;
        bool integrator_on=true;
        Eigen::VectorXd y_int_vec = Eigen::VectorXd::Zero(1);
        Eigen::VectorXd z_int_vec = Eigen::VectorXd::Zero(1);

        //per l'onda quadra
        bool onda=false;
        double y_des_filtered=y_des;
        double z_des_filtered=z_des;

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
            pub_des_=this->create_publisher<geometry_msgs::msg::PointStamped>("/des_traj",qos);
            abort_pub_=this->create_publisher<std_msgs::msg::String>("/abort",qos);
            subscriber_pos = this->create_subscription<geometry_msgs::msg::PointStamped>("ball_position", qos, std::bind(&ControllerLQR::position_callback, this, _1));

            //inizializzo il server
            start_controller_ = this->create_service<std_srvs::srv::SetBool>("controller/start", std::bind(&ControllerLQR::controller_cb_start, this, _1, _2));

            //inizializzo il service server per cambiare il setpoint
            set_des_=this->create_service<interfaces::srv::SetDes>("controller/set_des",std::bind(&ControllerLQR::set_des_cb, this, _1, _2));

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
                if(integrator_on==true && fabs(y_des-y)<0.005 && fabs(z_des-z)<0.005)
                {
                    integrator_on=false;
                    
                    RCLCPP_INFO_STREAM(this->get_logger(),"integratore OFF");
                }
                else if(integrator_on==false && (fabs(y_des-y)>0.014 || fabs(z_des-z)>0.014))
                {
                    integrator_on=true;
                   
                    RCLCPP_INFO_STREAM(this->get_logger(),"integratore ON");
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
            //update_desired_trajectory("diamond");
            //update_desired_trajectory("circle");

            double soft_start_weight=1-exp(-(p*t_camp)/tau);
        
            geometry_msgs::msg::PointStamped des_msg;
            des_msg.point.y=y_des_filtered;
            des_msg.point.z=z_des_filtered;
            des_msg.header.stamp=this->now();
            pub_des_->publish(des_msg);
            q++;
            p++;
            //theta=Kalman(0,soft_start_weight);
            //phi=Kalman(1,soft_start_weight);

            theta = Kalman(&xhat_y, &Phat_y, &S_y, &K_gain_y, &yhat, &y, &y_des_filtered, &y_int_old, soft_start_weight, &error_y_old, &y_int_vec);
            phi = Kalman(&xhat_z, &Phat_z, &S_z, &K_gain_z, &zhat, &z, &z_des_filtered,&z_int_old, soft_start_weight, &error_z_old, &z_int_vec);

            geometry_msgs::msg::PointStamped out_msg;
            out_msg.point.y=phi;
            out_msg.point.z=theta;
            out_msg.header.stamp=this->now();
            pub_posa_->publish(out_msg);
        }

        void update_desired_trajectory(std::string trajectory_type)
        {
            // Aggiorna la traiettoria desiderata in base al tipo di traiettoria
            if (trajectory_type == "square_wave")
            {
                //onda quadra
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
                y_des_filtered = y_des; 
                z_des_filtered = z_des; 
            }
            else if (trajectory_type == "diamond")
            {
                double alpha = t_camp / (0.5 + t_camp); // Coefficiente del filtro

                if (q * t_camp < 15.0) {
                    y_des = 0.04;
                    z_des = 0.0;
                } else if (q * t_camp < 30.0) {
                    y_des = 0.0;
                    z_des = 0.04;
                } else if (q * t_camp < 45.0) {
                    y_des = -0.04;
                    z_des = 0.0;
                } else if (q * t_camp < 60.0) {
                    y_des = 0.0;
                    z_des = -0.04;
                } else {
                    q = 0; // Reset q
                }
                

                // Calcola i valori desiderati intermedi
                y_des_filtered += alpha * (y_des - y_des_filtered);
                z_des_filtered += alpha * (z_des - z_des_filtered);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Tipo di traiettoria non riconosciuto");
            }
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

        void set_des_cb(const interfaces::srv::SetDes::Request::SharedPtr req, interfaces::srv::SetDes::Response::SharedPtr res)
        {
            if(req->y_des>0.10 || req->y_des<-0.10 || req->z_des>0.10 || req->z_des<-0.10)
            {
                res->success=false;
                RCLCPP_INFO_STREAM(this->get_logger(), "valore non valido");
                return;
            }
            y_des=req->y_des;
            z_des=req->z_des;
            p=0;
            res->success=true;
            RCLCPP_INFO_STREAM(this->get_logger(), "setpoint cambiato");
        }

        double Kalman(Eigen::VectorXd* xhat, Eigen::MatrixXd* Phat, Eigen::MatrixXd* S, Eigen::MatrixXd* K_gain, Eigen::VectorXd* outhat, float* measurement, double* desired,double* int_old, double soft_start_weight,double* error_old, Eigen::VectorXd* int_vec)
        {
            double u=0;
            Eigen::VectorXd measurement_vec(1);
            measurement_vec(0) = *measurement;
            double error = *desired - *measurement;

            if(integrator_on==true)
            {
                double integrate = *int_old + (t_camp / 2.0) * (error + *error_old);
                *int_old = integrate;
                *error_old = error;
                (*int_vec)[0] = integrate;
                (*xhat)[0]-=*desired; 
                auto u1=-K_new_.dot(*xhat);
                (*xhat)[0]+=*desired;  
                auto u2=K_int_.dot(*int_vec);
                u=u1+u2;
                u=u*soft_start_weight;
            }
            else
            {
                (*xhat)[0]-=*desired; 
                auto u1=-K_new_.dot(*xhat); 
                (*xhat)[0]+=*desired; 
                auto u2=K_int_.dot(*int_vec);
                u=u1+u2;
                u=u*soft_start_weight;
            }
            
    
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
            //W(0,0)=1e-6;
            //W(1,1)=1e-6;
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