#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include <chrono>
#include <cstdlib>
#include <memory>


/**
 * @class ModelSpawnerNode
 * 
 * Implementation of ROS NODE
 *
 * Ros node that subscribes to /xml_topic 
 *  
 * @see [rclcpp](https://docs.ros2.org/foxy/api/rclcpp/index.html)  
 */
class ModelSpawnerNode : public rclcpp::Node
{
public: 
    ModelSpawnerNode() : Node("add_models") {

      client = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
      subscription_ =  this->create_subscription<std_msgs::msg::String>("xml_topic",10,std::bind(&ModelSpawnerNode::GetXmlMessage, this, _1));
    }

private:

  /**
   * Callback method called when a message is published through /xml_topic
   * the message is then passed as a request to /spawn_entity gazebo service
   */
    void GetXmlMessage(const std_msgs::msg::String::SharedPtr msg){


    //RCLCPP_INFO(rclcpp::get_logger("spawning"), "XML message: '%s'", msg->data.c_str());


    ///@def request A pointer to create request message
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();  
    //request->name = <name of model> 
    request-> xml = msg->data;
    
  /**
   * Callback method called when there is an available response from the service
   * success returns true if spawn successful
   * status_message returns a string if comments available 
   */
    using ServiceResponseFuture =   rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture;
    auto response_received_callback = [](ServiceResponseFuture future) {
    RCLCPP_INFO(rclcpp::get_logger("spawning"), "Got result: %s with code: %d", (future.get()->status_message).c_str(), future.get()->success) ;    
    };

    auto result = client->async_send_request(request, response_received_callback);

    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client;

};

/**
 *main function
 * 
 * Instantiates and run the ros node
 */
int main(int argc, char **argv){ 

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ModelSpawnerNode>();
  rclcpp::spin(node);
  //return 0;
}




