#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdlib.h>
#include <omp.h>
// Dichiarazione funzioni generate da MATLAB Codegen
extern "C" {
void mySACPolicy_initialize();
void mySACPolicy_terminate();
void mySACPolicy(const double* observation1, double* action1);
}

class AgentNode : public rclcpp::Node
{
    private:
        // Dimensione stato e azione
        size_t obs_size = 10;
        size_t action_size = 2;
        std::vector<double> prev_velocity = {0.0, 0.0};
        std::vector<double> velocity = {0.0, 0.0};
        std::vector<double> position = {0.0, 0.0};
        double Ts = 0.017; 
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr observation_subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr angoli_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr velocity_pub_;
        //servizio per avviare il controllore
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_controller_;
        //publisher per fermare tutto
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr abort_pub_;
        bool start = false;
        bool ricevuto = false; 
        std::vector<double> state;
        int p=0;

        public:
        AgentNode() : Node("agent_node")
        {
            omp_set_num_threads(1); // Imposta il numero di thread OpenMP a 1 per evitare conflitti
            // Inizializza agente
            mySACPolicy_initialize();
         
            
            auto qos=rclcpp::SensorDataQoS();
            qos.keep_last(1);

            using namespace std::placeholders;
            // Sottoscrizione a topic "state"
            observation_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/observation", qos, std::bind(&AgentNode::state_callback, this, std::placeholders::_1));
            // timer ogni 17 ms
            timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(17)), std::bind(&AgentNode::timer_callback, this));               
            timer_->cancel();

            abort_pub_=this->create_publisher<std_msgs::msg::String>("/abort",qos);

            start_controller_ = this->create_service<std_srvs::srv::SetBool>("controller/start", std::bind(&AgentNode::controller_cb_start, this, _1, _2));

            //publisher 
            angoli_pub_=this->create_publisher<geometry_msgs::msg::PointStamped>("/desired_pose",qos);
            velocity_pub_=this->create_publisher<geometry_msgs::msg::PointStamped>("/action", qos);

        }

        ~AgentNode()
        {
            // Termina agente
            mySACPolicy_terminate();
        }

    private:
        void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {   
            if (start){
                // Stato ricevuto
                state = msg->data;
                //state = std::vector<double>(10, 0.0);
                if(!ricevuto)
                {
                    ricevuto = true;
                    timer_->reset();
                }
                if(fabs(state[4])==1||fabs(state[5])==1)
                {   
                    auto message=std_msgs::msg::String();
                    message.data="STOP!";
                    start=false;
                    abort_pub_->publish(message);
                }
            }
        }

        void timer_callback()
        {   //prendo il tempo
            //auto now = this->now();
            mySACPolicy(state.data(), velocity.data());
            //controllo e stampo il tempo di esecuzione
            //auto elapsed_time = this->now() - now;
            //std::cout << "Tempo di esecuzione: " << elapsed_time.seconds() * 1000.0 << " ms\n";
            geometry_msgs::msg::PointStamped action_msg;
            action_msg.header.stamp = this->now();
            action_msg.point.y = velocity[0]; // Assegna la prima azione alla coordinata y
            action_msg.point.z = velocity[1]; // Assegna la seconda azione alla coordinata z
            velocity_pub_->publish(action_msg);
            // Integro l'azione con Tustin 
            for (size_t i = 0; i < action_size; ++i) {
                position[i] += (Ts / 2.0) * (velocity[i] + prev_velocity[i]);
                prev_velocity[i] = velocity[i];
            }
            // Crea messaggio PointStamped
            geometry_msgs::msg::PointStamped msg;
            
            msg.header.stamp = this->now();
            msg.point.y = position[0]; // Assegna la prima azione alla coordinata y
            msg.point.z = position[1]; // Assegna la seconda azione alla coordinata z

            // Pubblica il messaggio
            angoli_pub_->publish(msg);
            //auto elapsed_time = this->now() - now;
            //RCLCPP_INFO(this->get_logger(), "Tempo di esecuzione: %f ms", elapsed_time.seconds() * 1000.0);


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
};
int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<AgentNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
