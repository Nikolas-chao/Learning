#include <iostream>
#include <functional>   // 函数包装器头文件

// 自由函数
void save_with_free_fun(const std::string &file_name){
    std::cout<<"这是自由函数:"<<file_name<<std::endl;
}

// 成员函数

class FileSave
{
private:
    /* data */
public:
    FileSave(/* args */) = default;
    ~FileSave() = default;

    void save_with_member_fun(const std::string & file_name){
        std::cout<<"成员函数:"<<file_name<<std::endl;
    };
};



int main()
{
    FileSave filesave;
    // 匿名函数--Lambda函数
    auto save_with_lambda_fun = [](const std::string &file_name)->void{
        std::cout<<"这是匿名函数:"<<file_name<<std::endl;
    };

    // 正常调用
    save_with_free_fun("file.txt");
    filesave.save_with_member_fun("file.txt");
    save_with_lambda_fun("file.txt");
    // 自由函数放入函数包装器
    std::function<void(const std::string&)> save1 = save_with_free_fun;

    // 成员函数放入函数包装器
    std::function<void(const std::string&)> save2 = std::bind(&FileSave::save_with_member_fun,&filesave,std::placeholders::_1);

    // 匿名函数放入函数包装器
    std::function<void(const std::string&)> save3 = save_with_lambda_fun;

    // 通过函数包装器调用
    save1("file.txt");
    save2("file.txt");
    save3("file.txt");

    return 0;
}
