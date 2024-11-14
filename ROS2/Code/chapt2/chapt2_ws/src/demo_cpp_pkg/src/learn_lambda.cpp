#include <iostream>
#include <algorithm>


// 匿名函数：lambda表达式

int main(){
    auto add = [](int a,int b) -> int{return a+b;};
    int sum = add(200,12);

    auto print_sum = [sum]()->void{
        std::cout<<"sum:"<<sum<<std::endl;
    };

    print_sum();

    return 0;
}