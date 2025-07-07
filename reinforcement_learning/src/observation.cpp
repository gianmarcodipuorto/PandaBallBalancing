#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/wait_for_message.hpp>
//Eigen
#include <Eigen/Dense>

class ObservationNode : public rclcpp::Node
{
    protected:
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_observation_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_pos;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_action;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_angoli_reali;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_des_;

        geometry_msgs::msg::PointStamped angoli_reali;
        geometry_msgs::msg::PointStamped angoli_reali_old;

        geometry_msgs::msg::PointStamped vel_angoli;
        geometry_msgs::msg::PointStamped old_vel_angoli;

        geometry_msgs::msg::PointStamped ball_position;
        geometry_msgs::msg::PointStamped old_ball_position;

        geometry_msgs::msg::PointStamped vel_ball;
        geometry_msgs::msg::PointStamped old_vel_ball;

        geometry_msgs::msg::PointStamped action;
        rclcpp::TimerBase::SharedPtr timer_;
        //numerare e denominatore del derivatore della pallina
        Eigen::Vector2d num_pallina;
        Eigen::Vector2d den_pallina;

        Eigen::Vector2d num_angoli;
        Eigen::Vector2d den_angoli;
        int p=0,q=0;
        double Ts=0.017; // Tempo di campionamento in s

        //per le traiettorie
        float y_des=0, z_des=0, y_des_filtered=0, z_des_filtered=0;
        bool onda=true;



        
    public:
        ObservationNode() : Node("observation_node")
        {
            using namespace std::placeholders;

            auto qos = rclcpp::SensorDataQoS();
            qos.keep_last(1);

            // Sottoscrizione al topic degli angoli reali
            sub_angoli_reali=this->create_subscription<geometry_msgs::msg::PointStamped>("real_angles", qos, std::bind(&ObservationNode::angoli_reali_cb, this, _1));
            // Publisher per l'osservazione
            pub_observation_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/observation", qos);
            //Susbriber posizione della palla
            subscriber_pos = this->create_subscription<geometry_msgs::msg::PointStamped>("ball_position", qos, std::bind(&ObservationNode::position_callback, this, _1));
            //Subscriber per l'azione
            subscriber_action = this->create_subscription<geometry_msgs::msg::PointStamped>("/action", qos, std::bind(&ObservationNode::action_callback, this, _1));
            // Timer per il ciclo di pubblicazione
            timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(17)), std::bind(&ObservationNode::publish_cycle, this));  

            pub_des_=this->create_publisher<geometry_msgs::msg::PointStamped>("/des_traj",qos);



            // Inizializzo le variabili
            angoli_reali_old.point.y = 0.0;
            angoli_reali_old.point.z = 0.0;

            angoli_reali.point.y = 0.0;
            angoli_reali.point.z = 0.0;
            
            vel_angoli.point.y = 0.0;
            vel_angoli.point.z = 0.0;

            ball_position.point.y = 0.0;
            ball_position.point.z = 0.0;

            old_ball_position.point.y = 0.0;
            old_ball_position.point.z = 0.0;

            vel_ball.point.y = 0.0;
            vel_ball.point.z = 0.0;

            old_vel_ball.point.y = 0.0;
            old_vel_ball.point.z = 0.0;

            action.point.y = 0.0;
            action.point.z = 0.0;


            num_pallina<<15.971205666309787,-15.971205666309787; // Coefficienti del numeratore per la velocità della pallina
            den_pallina<<1,-0.728489503672734; // Coefficienti del denominatore per la velocità della pallina

            num_angoli<<478.1144472213765,-478.1144472213765; // Coefficienti del numeratore per la velocità angolare
            den_angoli<<1,-0.521885552778623; // Coefficienti del denominatore per la velocità angolare
        }

        void publish_cycle()
        {
            
            // Calcolo l'osservazione
            std_msgs::msg::Float64MultiArray observation_msg;
            observation_msg.data.resize(10); // 10 elementi come da specifica

            //float y_des=0.05*cos(2*M_PI*0.1*p*0.017);
            //float z_des=0.05*sin(2*M_PI*0.1*p*0.017);

            // Aggiorna la traiettoria desiderata
            //update_desired_trajectory("square_wave");
            //update_desired_trajectory("diamond");
            //update_desired_trajectory("circle");

            p++;
            q++;
            geometry_msgs::msg::PointStamped des_msg;
            des_msg.point.y=y_des;
            des_msg.point.z=z_des;
            des_msg.header.stamp=this->now();
            pub_des_->publish(des_msg);
            //costruisco l'osservazione
            observation_msg.data[0] = angoli_reali.point.z; // Angolo Z reale
            observation_msg.data[1] = vel_angoli.point.z; // Velocità angolare Z
            observation_msg.data[2] = angoli_reali.point.y; // Angolo Y reale
            observation_msg.data[3] = vel_angoli.point.y; // Velocità angolare Y
            if(sqrt(ball_position.point.y*ball_position.point.y+ ball_position.point.z*ball_position.point.z)>0.10)
            {
                observation_msg.data[4] = 1;// Posizione Y della pallina
                observation_msg.data[5] = 1; // Posizione Z della pallina
            }
            else
            {
                observation_msg.data[4] = ball_position.point.y-y_des_filtered;// Posizione Y della pallina
                observation_msg.data[5] = ball_position.point.z-z_des_filtered; // Posizione Z della pallina
            }
            observation_msg.data[6] = vel_ball.point.y/10; // Velocità Y della pallina
            observation_msg.data[7] = vel_ball.point.z/10; // Velocità Z della pallina
            observation_msg.data[8] = action.point.y; // Azione Y
            observation_msg.data[9] = action.point.z; // Azione Z

            // Pubblico l'osservazione
            pub_observation_->publish(observation_msg);
        }

        void calcolo_velocità_palla()
        {
            // Calcolo la velocità della pallina
            vel_ball.point.y = num_pallina(0) * ball_position.point.y + num_pallina(1) * old_ball_position.point.y - den_pallina(1) * old_vel_ball.point.y;
            vel_ball.point.z = num_pallina(0) * ball_position.point.z + num_pallina(1) * old_ball_position.point.z - den_pallina(1) * old_vel_ball.point.z;
            old_vel_ball.point.y = vel_ball.point.y;
            old_vel_ball.point.z = vel_ball.point.z;
        }

        void position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
        {
            ball_position.point.y=msg->point.y;
            ball_position.point.z=msg->point.z;
            calcolo_velocità_palla();
            old_ball_position.point.y = ball_position.point.y;
            old_ball_position.point.z = ball_position.point.z;
            
        }


        void action_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
        {
            action.point.y = msg->point.y;
            action.point.z = msg->point.z;
        }

        void calcolo_velocità_angoli()
        {
            // Calcolo la velocità angolare
            vel_angoli.point.y = num_angoli(0) * angoli_reali.point.y + num_angoli(1) * angoli_reali_old.point.y - den_angoli(1) * old_vel_angoli.point.y;
            vel_angoli.point.z = num_angoli(0) * angoli_reali.point.z + num_angoli(1) * angoli_reali_old.point.z - den_angoli(1) * old_vel_angoli.point.z;
            old_vel_angoli.point.y = vel_angoli.point.y;
            old_vel_angoli.point.z = vel_angoli.point.z;
        }

        void angoli_reali_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
        {
            angoli_reali.point.y = msg->point.y;
            angoli_reali.point.z = msg->point.z;
            // Calcolo la velocità angolare
            calcolo_velocità_angoli();
            // Aggiorno gli angoli reali vecchi
            angoli_reali_old.point.y = angoli_reali.point.y;
            angoli_reali_old.point.z = angoli_reali.point.z;

        }

        void update_desired_trajectory(std::string trajectory_type)
        {
            // Aggiorna la traiettoria desiderata in base al tipo di traiettoria
            if (trajectory_type == "square_wave")
            {
                //onda quadra
                double alpha = Ts / (0.5 + Ts); // Coefficiente del filtro
                if(q*Ts>=10.0)
                {
                    if(onda==true)
                    {
                        y_des=0.04;
                        onda=false;
                        q=0;
                    }else
                    {
                        y_des=-0.04;
                        onda=true;
                        q=0;
                    }
                }
                // Calcola i valori desiderati intermedi
                y_des_filtered += alpha * (y_des - y_des_filtered);
                z_des_filtered += alpha * (z_des - z_des_filtered);
            }
            else if (trajectory_type == "circle")
            {
                y_des=0.05*cos(2*M_PI*0.05*p*Ts);
                z_des=0.05*sin(2*M_PI*0.05*p*Ts);
                y_des_filtered = y_des; 
                z_des_filtered = z_des; 
            }
            else if (trajectory_type == "diamond")
            {
                double alpha = Ts / (0.5 + Ts); // Coefficiente del filtro

                if (q * Ts < 15.0) {
                    y_des = 0.04;
                    z_des = 0.0;
                } else if (q * Ts < 30.0) {
                    y_des = 0.0;
                    z_des = 0.04;
                } else if (q * Ts < 45.0) {
                    y_des = -0.04;
                    z_des = 0.0;
                } else if (q * Ts < 60.0) {
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

};
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObservationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
