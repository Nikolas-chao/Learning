#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TurtleControlNode :public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;      // 发布者的智能指针
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;      // 订阅者的智能指针
        double target_x_{1.0};
        double target_y_{1.0};
        double k_{1.0}; // 比例系数
        double max_speed_{3.0}; // 最大速度

    public:
        explicit TurtleControlNode(const std::string &node_name):Node(node_name){
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
            subscriber_= this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&TurtleControlNode::on_pose_recevived_,this,std::placeholders::_1));
            
        }

        // 参数：订阅到数据的共享指针   
        void on_pose_recevived_(const turtlesim::msg::Pose::SharedPtr pose){
            // step1:> 获取当前位置
            auto current_x = pose->x;
            auto current_y = pose->y;
            RCLCPP_INFO(this->get_logger(),"当前位置“:x=%f,y=%f",current_x,current_y);
            // step2:> 计算当前位置与目标位置的距离差和角度差 两点之间距离公式 
            auto distance = std::sqrt(
                (target_x_-current_x)*(target_x_-current_x)+
                (target_y_-current_y)*(target_y_-current_y)
            );
            // 利用反切公式计算当前点与目标点的角度差
            auto angle = std::atan2((target_y_-current_y),(target_x_-current_x)) - pose->theta;

            //step3:> 控制策略

            auto msg = geometry_msgs::msg::Twist();

            if(distance>0.1){
                if(fabs(angle)>0.2){
                    msg.angular.z = fabs(angle);
                }else{
                    msg.linear.x = k_*distance;
                }
            }

            // step4:> 线速度和角速度限制
            if(msg.linear.x>max_speed_){
                msg.linear.x = max_speed_;
            } 

            publisher_->publish(msg);

        }


};



int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<TurtleControlNode>("turtle_control");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
