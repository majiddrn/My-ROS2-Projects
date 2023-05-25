#include <chrono>
#include <memory>
#include <string>
#include <cstdio>
#include <string.h>
#include <any>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "yinyang_interfaces/action/m_action.hpp"
#include "yinyang_interfaces/srv/message.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::placeholders;
using namespace std::chrono;
using MAction = yinyang_interfaces::action::MAction;

rclcpp::Node::SharedPtr node;
int32_t m_i = 0;

std::string messages[8] = {"Hi Yin, I am Yang the opposite of you.",
  "Yes, Yin; we ourselves, do not mean anything since we are only employed to express a relation",
  "Precisely, Yin; we are used to describe how things function in relation to each other and to the universe.",
  "For what is and what is not beget each other.",
  "High and low place each other.",
  "Before and behind follow each other.",
  "And you fade into the darkness.",
  "Well, bye Yin"};

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
rclcpp::Client<yinyang_interfaces::srv::Message>::SharedPtr client_yin;
rclcpp::Service<yinyang_interfaces::srv::Message>::SharedPtr service;
yinyang_interfaces::srv::Message::Response::SharedPtr yin_response;
rclcpp_action::Client<MAction>::SharedPtr action_client;
rclcpp::TimerBase::SharedPtr timer;


void feedback_callback(rclcpp_action::ClientGoalHandle<MAction>::SharedPtr goal_handle, const std::shared_ptr<const MAction::Feedback> feedback) {
  RCLCPP_INFO(node->get_logger(), "Received feedback message with field value: %d", feedback->opacity);
}

void result_callback(const rclcpp_action::ClientGoalHandle<MAction>::WrappedResult& result) {
  switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node->get_logger(), "Action succeeded with result: %s", result.result->res.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Action was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node->get_logger(), "Action was canceled");
        break;
      default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");

  }
}


void timer_callback() {
  if (m_i >= 7) {
    
    if (m_i == 7){                   // Because have no segmentation fault.
      if (strstr(messages[m_i].c_str(), "bye")) {
        auto goal = MAction::Goal();
        goal.msg = "bye";
        auto send_goal_options = rclcpp_action::Client<MAction>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&feedback_callback, _1, _2);
        send_goal_options.result_callback = std::bind(&result_callback, _1);
  
        auto goal_handle_future = action_client->async_send_goal(goal, send_goal_options);
      
        m_i++;
      }
    } else 
      m_i++;
  }
}

void response_callback(rclcpp::Client<yinyang_interfaces::srv::Message>::SharedFuture future) {

  auto status = future.wait_for(1s);
  if (status == std::future_status::ready) {
    RCLCPP_INFO(node->get_logger(), "Result: %i",
                future.get()->checksum_r);
  } else {
    RCLCPP_INFO(node->get_logger(), "Service In-Progress...");
  }
}

void service_callback(
  const std::shared_ptr<yinyang_interfaces::srv::Message::Request> request,
  std::shared_ptr<yinyang_interfaces::srv::Message::Response> response){

  auto logger = node->get_logger();

  std_msgs::msg::String message;
  message.data = "Yin said: " + request->msg + ", " + std::to_string(request->length) + ", " + std::to_string(request->checksum);

  publisher->publish(message);
  RCLCPP_INFO(node->get_logger(), "data:%s", message.data.c_str());

  int checksum_y = 0;

  for (size_t i = 0; i < request->msg.length(); i++)
    checksum_y += request->msg[i];

  response->checksum_r = checksum_y;

  if (m_i <= 6) {
    RCLCPP_INFO_ONCE(logger, "BFORE:%d", m_i);
    auto m = std::make_shared<yinyang_interfaces::srv::Message::Request>();
    m->name = "Yang";
    m->msg = messages[m_i];
    m->length = messages[m_i].length();
    m->checksum = checksum_y;
    RCLCPP_INFO_ONCE(node->get_logger(), "m_i: %d", m_i);
    auto future_result = client_yin->async_send_request(m, std::bind(&response_callback, _1));
    m_i++;

  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("yangnode");

  auto timer = node->create_wall_timer(std::chrono::milliseconds(250), &timer_callback);

  publisher = node->create_publisher<std_msgs::msg::String>("/conversation", 50);
  client_yin = node->create_client<yinyang_interfaces::srv::Message>("yin_service");
  service = node->create_service<yinyang_interfaces::srv::Message>("yang_service",
    std::bind(&service_callback, _1, _2)
  );

  node->declare_parameter("shout", "false");
  node->declare_parameter("opacity", 100);

  action_client = rclcpp_action::create_client<MAction>(node, "yin_action_server");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}