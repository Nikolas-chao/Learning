#include <iostream>
#include <memory>

int main()
{
    auto p1 = std::make_shared<std::string>("This is a str");
    // make_shared 是一个模板类，对应的返回值是对应类的共享指针
    // std::make_shared<数据类型/类>(参数)

    std::cout<<"p1的引用记数"<<p1.use_count()<<",指向的内存地址："<<p1.get()<<std::endl;
    auto p2 = p1;
    std::cout<<"p1的引用记数"<<p1.use_count()<<",指向的内存地址："<<p1.get()<<std::endl;
    std::cout<<"p2的引用记数"<<p2.use_count()<<",指向的内存地址："<<p2.get()<<std::endl;

    p1.reset(); // 释放引用，不指向内存地址
    std::cout<<"p1的引用记数"<<p1.use_count()<<",指向的内存地址："<<p1.get()<<std::endl;
    std::cout<<"p2的引用记数"<<p2.use_count()<<",指向的内存地址："<<p2.get()<<std::endl;

    std::cout<<"p2的指向内存地址数据:"<<p2->c_str()<<std::endl;
    return 0;
}
