//utility
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

//interfacce
#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <interfaces/action/home.hpp>


int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    using SetBool=std_srvs::srv::SetBool;
    using namespace std::chrono_literals;
    using Home = interfaces::action::Home;

    //creo un nodo
    auto node= std::make_shared<rclcpp::Node>("handler_node");

    //inizializzo l'action client
    auto action_client= rclcpp_action::create_client<Home>(node,"home");
    while(!action_client->wait_for_action_server(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the action server. Exiting.");
            return 0;   
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "action server not available, waiting again...");
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Home Server UP");
    //creo un goal
    auto goal= Home::Goal();
    goal.duration = rclcpp::Duration(5s);
    auto future_goal_handle= action_client->async_send_goal(goal);
    if(rclcpp::spin_until_future_complete(node,future_goal_handle)!=rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, GOAL NOT SENT...");
        return -1;
    }
    //se arrivo qui il servizio è terminato e posso vedere se è terminato con successo
    auto future_result = action_client->async_get_result(future_goal_handle.get());
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, RESULT NOT AVAILABLE...");
        return -1;
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Home Raggiunta");

    //aspetta che qualcuno prema un tasto
    std::cout<<"Premi un tasto per continuare..."<<std::endl;
    std::cin.get();
    
    //inizializzo i service client
    auto start_clik= node->create_client<SetBool>("clik/start_stop");
    auto start_controller= node->create_client<SetBool>("controller/start");
  
    while (!start_clik->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;   
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Clik Server UP");

    {
    auto start_clik_request= std::make_shared<std_srvs::srv::SetBool::Request>();
    start_clik_request->data=true;
    auto future_result=start_clik->async_send_request(start_clik_request);
    if(rclcpp::spin_until_future_complete(node,future_result)!=rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
            return -1;
        }
        //se arrivo qui il servizio è terminato e posso vedere se è terminato con successo
        if(!future_result.get()->success)
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE NOT SUCCESSFUL... message: " << future_result.get()->message);
            return -1;
        }
        RCLCPP_INFO_STREAM(node->get_logger(), "Clik Avviato");
    }


    while (!start_controller->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;   
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Controller Server UP");

    {
    auto start_controller_request= std::make_shared<std_srvs::srv::SetBool::Request>();
    start_controller_request->data=true;
    auto future_result=start_controller->async_send_request(start_controller_request);
    if(rclcpp::spin_until_future_complete(node,future_result)!=rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
            return -1;
        }
        //se arrivo qui il servizio è terminato e posso vedere se è terminato con successo
        if(!future_result.get()->success)
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE NOT SUCCESSFUL... message: " << future_result.get()->message);
            return -1;
        }
        RCLCPP_INFO_STREAM(node->get_logger(), "Clik Avviato");
    }

    RCLCPP_INFO_STREAM(node->get_logger(), "Controller Avviato"); 



}