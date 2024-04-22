#include "tinyxml2.h"
#include <gazebo/gui/GuiIface.hh>
#include <ignition/math/Plane.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <string>
#include <vector>

#include <gazebo/common/MouseEvent.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/rendering.hh>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <gazebo_ros/node.hpp>

#include "labelCreator.h"





namespace gazebo {


/**
 * @class SystemGUI
 * 
 * Implementation of a system plugin for gazebo which inherits from SystemPlugin
 *
 * System plugin that gets the scroll click location,
 * Gazebo ros node that publishes the name of the model
 * to be spawned as well as the xml stream of the model,
 * it also subscribes to /selection topic to get the class
 * or element of the model (Sidewalk, Oak_tree, Grass) 
 * 
 * @throws  Plugin.hh:223 thrown if failed to resolve RegisterPlugin. The plugin is not registered.
 * @throws  parser.cc:468 thrown if Error parsing XML. Error reading Element value
 * 
 * @see [Gazebo ros](http://www.lxshaw.com/tech/ros/2021/05/22/gazebo%E5%85%A5%E9%97%A8%EF%BC%88%E4%B9%9D%EF%BC%89%EF%BC%9A%E4%B8%80%E4%B8%AA%E6%9B%B4%E5%A5%BD%E7%9A%84ros2-gazebo%E6%8F%92%E4%BB%B6%E6%A8%A1%E6%9D%BF/)
 * @see [System Plugin](https://classic.gazebosim.org/tutorials?tut=system_plugin&cat=write_plugin)  
 */
class SystemGUI : public SystemPlugin {

public:
  std::vector<ignition::math::Vector3d> points;  
  ignition::math::Vector3<double> ground_plane_normal =
      ignition::math::Vector3<double>(0.0, 0.0, 1.0);
  ignition::math::Planed ground_plane =
      ignition::math::Planed(ground_plane_normal);

  std::string classifier = "";



private:
  gazebo_ros::Node::SharedPtr ros_node_; // Pointer GazeboROS node
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr xmlModelPublisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr modelNamePublisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr modelSubscriber;


  // Destructor
public:
  virtual ~SystemGUI() {
    gui::MouseEventHandler::Instance()->RemovePressFilter("glwidget");
  }


  
public:

 /**
  * @brief Method that inits gazebo ros node and class attributes. 
  * It also sets the document with the xml data
  * 
  * @param main parameters of main function 
  */
  void Load(int /*_argc*/, char ** /*_argv*/) {

    
    char fileDelete[] = "SavedData.sdf";
    int result = remove(fileDelete); 
    std::string classifier = "";  

    // Register callback
    gui::MouseEventHandler::Instance()->AddPressFilter( "glwidget", boost::bind(&SystemGUI::OnMousePress, this, _1));

    

    /// @def ros_node_ inits gazebo_ros interfaces
    this->ros_node_ = gazebo_ros::Node::Get();
    /// @def xmlModelPublisher_ creates a publisher to /xml_topic
    this->xmlModelPublisher_ = this->ros_node_->create_publisher<std_msgs::msg::String>("xml_topic", 10);
    /// @def modelSubscriber creates a subscriber to /selection 
    this->modelSubscriber = this->ros_node_->create_subscription<std_msgs::msg::String>("selection",10, std::bind(&SystemGUI::message_selection,this,std::placeholders::_1));
    /// @def modelNamePublisher creates a publisher to /name_sender topic
    this->modelNamePublisher = this->ros_node_->create_publisher<std_msgs::msg::String>("name_sender",10);

  }  


 /**
  * @brief Callback method called when the class of the model is published.
  * It sets classifier attribute to the model class. * 
  * 
  * 
  * @param msg string-type message with the selected class i.e. Sidewalk, Grass, Building.
  */
  void message_selection (const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("polygon_parser"), "Model to be spawned: '%s'", msg->data.c_str());
    this-> classifier = msg->data.c_str();
    }

 /**
  * @brief Callback method. It handles mouse events. It sets
  * the points array attribute to the world's coordinates when
  * scroll click pressed. This method  handles the logic of the
  * rendering of the model
  * 
  * @param _event A reference to the event of the form: NO_BUTTON, LEFT, MIDDLE  
  */  
  bool OnMousePress(const common::MouseEvent &_event) {
    ignition::math::Vector3d intersection;
 
    if (!gui::get_active_camera()->WorldPointOnPlane(
            _event.Pos().X(), _event.Pos().Y(), ground_plane, intersection)) {
      return false;
    }
    if (_event.Button() == common::MouseEvent::RIGHT) {
      RenderModel();
      points.clear();
      return true;
    }
    if (_event.Button() == common::MouseEvent::MIDDLE) {
      points.emplace_back(intersection);
      std::cout << "Mouse press detected! " << intersection.X() << " "
                << intersection.Y() << std::endl;
    }
    return true;
  }

 /**
  * @brief Method that publishes the XML message of the model. It populates
  * SavedData.sdf documents with the elements to be spawned.
  *   
  */  
  void RenderModel() {  

    if (points.size() == 0) {
      std::cout << "no points selected\n";
      return;
    }

  /// @def modelName container with the model name to be published
  auto modelName =  std_msgs::msg::String();


  /// @def dataSave xmlDocument declaration
  tinyxml2::XMLDocument dataSave; 
  dataSave.LoadFile("SavedData.sdf");

  // Create new XML model
  std::string name = labelCreator::NewXmlModel(dataSave,this->classifier.c_str(), points);

   
  modelName.data = name;
  this->modelNamePublisher->publish(modelName); 
  tinyxml2::XMLError eResult1 = dataSave.SaveFile("SavedData.sdf");
  tinyxml2::XMLPrinter printer(0, false, 0);
  dataSave.Print(&printer);    
  

  auto xmlMen = std_msgs::msg::String();

  std::string s; 
  s.assign(printer.CStr()) ; 
  std::stringstream modelPrint;
  s.erase(std::remove(s.begin(), s.end(), '\n'), s.end());
  modelPrint <<  "<sdf version=\"1.6\">" << s  << "</sdf>";
  printer.ClearBuffer();
  xmlMen.data = modelPrint.str();
  this->xmlModelPublisher_->publish(xmlMen);   

  }
};
// Register plugin 
GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}

