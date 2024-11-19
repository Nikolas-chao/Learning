#include <QApplication>
#include <QLabel>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <status_interfaces/msg/system_status.hpp>

using SystemStatus = status_interfaces::msg::SystemStatus;


class SysStatusDisplay : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Subscription<SystemStatus>::SharedPtr subscriber_;
    QLabel *label_;
public:
    SysStatusDisplay(/* args */):Node("sys_status_display"){
        label_ = new QLabel(QString::fromStdString("Hello Qt!"));
        subscriber_ = this->create_subscription<SystemStatus>("sys_status",10,[&](const SystemStatus::SharedPtr msg)->void{
            label_->setText(get_qstr_from_msg(msg));    
        });
        label_->setText(get_qstr_from_msg(std::make_shared<SystemStatus>()));
        label_->show();
    };

    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg){
        std::stringstream show_str;
        show_str << "===========系统状态可视化===========\n"
        << "时间: "<< msg->stamp.sec<<"\ts\n"
        << "主机名: "<<msg->host_name<<"\t\n"
        << "cpu使用率: "<<msg->cpu_percent<<"\t%\n"
        << "内存使用率: "<<msg->memory_percent<<"\t%\n"
        << "内存总大小: "<<msg->memory_total<<"\tMB\n"
        << "剩余内存大小: "<<msg->memory_available<<"\tMB\n"
        << "网络发送量: "<<msg->net_sent<<"\tMB\n"
        << "网络接收量: "<<msg->net_recv<<"\tMB\n"
        << "==================================";

        return QString::fromStdString(show_str.str());
    };

};






int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    QApplication app(argc,argv);
    auto node = std::make_shared<SysStatusDisplay>();
    std::thread spin_thread([&]()->void{
        rclcpp::spin(node);
    });  
    spin_thread.detach();

  
    app.exec();    // 执行应用
    return 0;
}
