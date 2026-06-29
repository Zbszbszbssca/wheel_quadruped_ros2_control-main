//
// Created by tlab-uav on 24-9-13.
//

#include "joystick_input/JoystickInput.h"

using std::placeholders::_1;

JoystickInput::JoystickInput() : Node("joysick_input_node") {
    publisher_ = create_publisher<control_input_msgs::msg::Inputs>("control_input", 10);
    subscription_ = create_subscription<
        sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickInput::joy_callback, this, _1));
}

void JoystickInput::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
    int FSM_EN=msg->axes[7];
    if(FSM_EN==1)
    {


        inputs_.lx = 0;
        inputs_.ly = 0;
        inputs_.rx = 0;
        inputs_.ry = 0;

        if (msg->axes[4]==-1 && msg->axes[5]==-1&&msg->axes[6]==-1) {
            inputs_.command = 1; // 1
        } else if (msg->axes[4]==0 && msg->axes[5]==-1&&msg->axes[6]==-1) {
            inputs_.command = 2; // 2
        } else if (msg->axes[4]==1 && msg->axes[5]==-1&&msg->axes[6]==-1) {
            inputs_.command = 3; // 3
        } else if (msg->axes[4]==-1 && msg->axes[5]==0&&msg->axes[6]==-1) {
            inputs_.command = 4; // 4
        } else if (msg->axes[4]==0 && msg->axes[5]==0&&msg->axes[6]==-1) {
            inputs_.command = 5; // 5
        } else if (msg->axes[4]==1 && msg->axes[5]==0&&msg->axes[6]==-1) {
            inputs_.command = 6; // 6
        } else if (msg->axes[4]==-1 && msg->axes[5]==1&&msg->axes[6]==-1) {
            inputs_.command = 7; // 7
        } else if (msg->axes[4]==0 && msg->axes[5]==1&&msg->axes[6]==-1) {
            inputs_.command = 8; // 8
        } else if (msg->axes[4]==-1 && msg->axes[5]==-1&&msg->axes[6]==0) {
            inputs_.command = 9; // START
        } 

    }else {
        inputs_.command = 0;
        inputs_.lx = -msg->axes[0];
        inputs_.ly = msg->axes[1];
        inputs_.rx = -msg->axes[3];
        inputs_.ry = msg->axes[2];
    }
    publisher_->publish(inputs_);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickInput>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
