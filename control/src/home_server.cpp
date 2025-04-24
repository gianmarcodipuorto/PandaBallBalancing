//utility
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

//interfacce
#include <sensor_msgs/msg/joint_state.hpp>
#include "control/quintic.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

//Per l'azione
#include <rclcpp_action/rclcpp_action.hpp>
#include <interfaces/action/home.hpp>

class HomeAction: public rclcpp::Node
{
    
    using Home = interfaces::action::Home;
    using GoalHandleHome = rclcpp_action::ServerGoalHandle<Home>;

    protected:
        rclcpp_action::Server<Home>::SharedPtr action_server_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_posa_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
        sensor_msgs::msg::JointState q0_,qf;

        std::vector<std::string> joint_names_={"panda_joint1",
                                                "panda_joint2",
                                                "panda_joint3",
                                                "panda_joint4",
                                                "panda_joint5",
                                                "panda_joint6",
                                                "panda_joint7"};
        

    public:
        HomeAction(const rclcpp::NodeOptions opt=rclcpp::NodeOptions()):Node("home_action", opt)
        {
            using namespace std::placeholders;
            q0_.position.resize(joint_names_.size());
            q0_.position={0,0,0,0,0,0,0};
            qf.position.resize(joint_names_.size());
            qf.position[0]=-0.217350;
            qf.position[1]=-1.297263;
            qf.position[2]=1.653361;
            qf.position[3]=-1.267326;
            qf.position[4]=-0.30480;
            qf.position[5]=1.4763787;
            qf.position[6]=-1.594;

            vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_velocity_controller/command", 1);       

            this->action_server_=rclcpp_action::create_server<Home>(
                this,
                "home",
                std::bind(&HomeAction::handle_goal, this, _1, _2),
                std::bind(&HomeAction::handle_cancel, this, _1),
                std::bind(&HomeAction::handle_accepted, this, _1)
            );
            RCLCPP_INFO(this->get_logger(), "Home action server started");
        }
        
        ~HomeAction()=default;

    protected:
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Home::Goal> goal)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request with duration: "<<builtin_interfaces::msg::to_yaml(goal->duration));
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleHome> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleHome> goal_handle)
        {
            using namespace std::placeholders;
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
            // Start the action
            std::thread{std::bind(&HomeAction::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleHome> goal_handle)
        {
            using namespace std::chrono_literals;
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            auto goal = goal_handle->get_goal();
            if(!rclcpp::wait_for_message<sensor_msgs::msg::JointState>(q0_,shared_from_this(),"/franka_state_controller/joint_states",10s))
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(),"non è stato possibile leggere la posizione attuale del robot");
                    auto result= std::make_shared<Home::Result>();
                    result->success=false;
                    goal_handle->abort(result);
                    return;
                }
            
            rclcpp::Rate loop_rate(1000); // 1 kHz
            rclcpp::Time t0 = this->now();
            rclcpp::Duration t(0, 0);

            double traj_duration=rclcpp::Duration(goal->duration).seconds();

            // Ciclo di "generazione traiettoria"
            while (rclcpp::ok() && t.seconds() <= traj_duration)
            {
                t = this->now() - t0;

                //controlla la preemption dell'azione
                if(goal_handle->is_canceling())
                {
                    RCLCPP_INFO(this->get_logger(), "Cancelling goal");
                    auto result = std::make_shared<Home::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    return;
                }

                // riempio il goal
                std_msgs::msg::Float64MultiArray velocity;
                velocity.data.resize(joint_names_.size());

                // riempio il vettore usando la funzione quintic
                for (size_t i = 0; i < joint_names_.size(); ++i)
                {
                    velocity.data[i] = quintic(t.seconds(), q0_.position[i], qf.position[i], traj_duration);
                }


                vel_pub_->publish(velocity);
                loop_rate.sleep();
            }

            // Se l'azione è completata
            if(rclcpp::ok())
            {
                auto result= std::make_shared<Home::Result>();
                result->success=true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }

        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HomeAction>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}