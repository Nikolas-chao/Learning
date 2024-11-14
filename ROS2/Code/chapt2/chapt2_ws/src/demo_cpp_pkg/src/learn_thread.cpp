#include <thread>   // 多线程
#include <chrono>   // 时间
#include <functional>   // 函数包装器
#include "cpp-httplib/httplib.h"    //下载相关

// 终端输入：python3 -m http.server 
// 开启http服务器

class Download
{
private:
    /* data */
public:
    void download(const std::string& host,const std::string& path,const std::function<void(const std::string&,const std::string&)> &callback_word_count){
        std::cout<<"线程编号"<<std::this_thread::get_id()<<std::endl;
        httplib::Client cli(host);
        auto response = cli.Get(path);
        if(response && response->status == 200){ // http请求成功的返回值
            callback_word_count(path,response->body);
        }


    };

    void start_download(const std::string& host,const std::string& path,const std::function<void(const std::string&,const std::string&)> &callback_word_count){
        auto download_fun = std::bind(&Download::download,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
        std::thread thread(download_fun,host,path,callback_word_count);
        thread.detach();
    };
};


int main()
{
    auto d = Download();

    auto word_count = [](const std::string& path,const std::string& result)->void{
        std::cout<<"下载完成:"<<path<<":"<<result.length()<<"->"<<result.substr(0,9) << std::endl;
    };

    d.start_download("http://0.0.0.0:8000","/nocel1.txt",word_count);
    d.start_download("http://0.0.0.0:8000","/nocel2.txt",word_count);
    d.start_download("http://0.0.0.0:8000","/nocel3.txt",word_count);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000*2));
    return 0;
}
