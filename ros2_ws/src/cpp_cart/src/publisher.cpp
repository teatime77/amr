#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cassert>
#include <cstddef>
#include <cstdint>

// 【C++】簡単なソケット通信プログラムを作ってみた。 | 情報学部生の気ままなブログ
//      https://tora-k.com/2019/08/27/socket-c/
#include <iostream> //標準入出力
#include <sys/socket.h> //アドレスドメイン
#include <sys/types.h> //ソケットタイプ
#include <arpa/inet.h> //バイトオーダの変換に利用
#include <unistd.h> //close()に利用



#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

#ifndef BYTE
#define BYTE    unsigned char
#endif

#define RANGES_SIZE     1024 * 8

enum DataType {
    ydlidar,
    imu,
    encoder
};

struct IMUdata {
    char  mark[4];    
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
};

int little_int(BYTE* dt, int idx){
    return int(dt[idx] + 256 * (int)dt[idx + 1]);
}

class MinimalPublisher : public rclcpp::Node
{
    int sockfd;
    BYTE data[1024*8];
    int dataCnt = 0;

    float ranges[RANGES_SIZE];
    int nRanges = 0;

    float prevFSA = NAN;
    float prevLSA = NAN;

    clock_t startTime;
    clock_t prevTime;

public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

        //ソケットの生成
        sockfd = socket(AF_INET, SOCK_STREAM, 0); //アドレスドメイン, ソケットタイプ, プロトコル
        if(sockfd < 0){ //エラー処理

            RCLCPP_INFO(this->get_logger(), "socket error");
            std::cout << "Error socket:" << std::strerror(errno); //標準出力
            exit(1); //異常終了
        }

        //アドレスの生成
        struct sockaddr_in addr; //接続先の情報用の構造体(ipv4)
        memset(&addr, 0, sizeof(struct sockaddr_in)); //memsetで初期化
        addr.sin_family = AF_INET; //アドレスファミリ(ipv4)
        addr.sin_port = htons(80); //ポート番号,htons()関数は16bitホストバイトオーダーをネットワークバイトオーダーに変換
        addr.sin_addr.s_addr = inet_addr("192.168.0.120"); //IPアドレス,inet_addr()関数はアドレスの翻訳

        //ソケット接続要求
        int sts = connect(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)); //ソケット, アドレスポインタ, アドレスサイズ
        if(sts == 0){

            RCLCPP_INFO(this->get_logger(), "connected");

        }
        else{
            RCLCPP_INFO(this->get_logger(), "connect error");

            return;
        }

        //データ送信
        char s_str[] = "HelloWorld!"; //送信データ格納用
        int size = send(sockfd, s_str, strlen(s_str) + 1, 0); //送信



        RCLCPP_INFO(this->get_logger(), "send '%d'", size);

        startTime = clock();
    }

    ~MinimalPublisher(){
        //ソケットクローズ
        close(sockfd);
    }

    void publish_scan(){
        clock_t current_time = clock();

        double stamp = (current_time - startTime) / CLOCKS_PER_SEC;

        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

        scan_msg->header.stamp.sec = floor(stamp);
        scan_msg->header.stamp.nanosec = 1000 * 1000 * 1000 * (stamp - scan_msg->header.stamp.sec);
        scan_msg->header.frame_id = "laser_frame";
        scan_msg->angle_min = prevFSA * M_PI / 180.0;
        scan_msg->angle_max = prevLSA * M_PI / 180.0;
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / nRanges;
        scan_msg->scan_time = (current_time - prevTime) / CLOCKS_PER_SEC;
        scan_msg->time_increment = scan_msg->scan_time / nRanges;
        scan_msg->range_min = 0.0;
        scan_msg->range_max = 10.0;

        scan_msg->ranges.resize(nRanges);
        scan_msg->intensities.resize(0);
        for(int i=0; i < nRanges; i++) {
            scan_msg->ranges[i] = ranges[i];
        }        

        publisher_->publish(*scan_msg);

        RCLCPP_INFO(this->get_logger(), "pub stamp:%d:%d time:%.3lf nRanges:%d FSA-LSA:%d %d", scan_msg->header.stamp.sec, scan_msg->header.stamp.nanosec, scan_msg->scan_time, nRanges, int(prevFSA), int(prevLSA));

        prevTime = current_time;
    }

    void readHead(BYTE* dt, int LSN){
        // int CT  = dt[2] & 0x01;

        int iFSA = little_int(dt, 4);
        int iLSA = little_int(dt, 6);
        int CS  = little_int(dt, 8);

        float FSA = (iFSA >> 1) / 64.0;
        float LSA = (iLSA >> 1) / 64.0;
        RCLCPP_INFO(this->get_logger(), "CT:%X FSA:%d LSA:%d", dt[2], int(FSA), int(LSA));

        if(LSA <= FSA){
            return;
        }

        int cs = 0x55AA ^ little_int(dt, 2) ^ iFSA ^ iLSA;
        for(int i = 0; i < LSN; i++){

            int r = little_int(dt, 10 + 2 * i);

            // # X2 DEVELOPMENT MANUAL: Distance solution formula:
            // #   https://www.ydlidar.com/Public/upload/files/2022-06-21/YDLIDAR%20X2%20Development%20Manual%20V1.2(211228).pdf
            ranges[nRanges + i] = 0.001 * float(r) / 4.0;

            cs ^= r;
        }
        if(CS != cs){

            RCLCPP_INFO(this->get_logger(), "CS error %d <> %d -----------------------------------------------", CS, cs);
        }

        if(std::isnan(prevFSA)){
            // 最初の場合

            prevFSA = FSA;

        }
        else if(FSA < prevLSA){
            // 角度が減った場合

            if(nRanges != 0){

                
                publish_scan();
                nRanges = 0;
            }

            prevFSA = FSA;
        }

        prevLSA = LSA;

        nRanges += LSN;
        assert(nRanges < RANGES_SIZE);
    }

    int find_head(DataType& data_type){
        for(int i = 0; i < dataCnt - 10; i++){
            if(data[i] == 0xAA && data[i + 1] == 0x55){

                int LSN = data[i + 3];
                if(i + 10 + 2 * LSN <= dataCnt){

                    data_type = DataType::ydlidar;
                    return i;
                }
                else{
                    return -1;
                }
            }
            else if(data[i] == 0xBB && data[i + 1] == 0x66 && i + int(sizeof(IMUdata)) < dataCnt){
                data_type = DataType::imu;
                return i;
            }
        }

        return -1;
    }

private:
    void timer_callback()
    {

        //データ受信
        char buf[1024*8]; //受信データ格納用
        int size = recv(sockfd, buf, sizeof(buf), 0); //受信

        assert(dataCnt + size < (int)sizeof(data));
        memcpy(data + dataCnt, buf, size);
        dataCnt += size;

        while (10 <= dataCnt){
            DataType data_type;

            int idx = find_head(data_type);

            if(idx == -1){
                return;
            }

            int data_len;

            switch (data_type){
            case DataType::ydlidar:{

                int LSN = data[idx + 3];
                readHead(data + idx, LSN);

                data_len = 10 + 2 * LSN;
                break;
            }

            case DataType::imu:{
                IMUdata imu_dt;
                
                data_len = sizeof(IMUdata);
                memcpy(&imu_dt, data + idx, data_len);

                RCLCPP_INFO(this->get_logger(), "acc:(%.1f, %.1f, %.1f) gyro:(%.1f, %.1f, %.1f) temp:%.1f", 
                    imu_dt.acc_x, imu_dt.acc_y, imu_dt.acc_z, 
                    imu_dt.gyro_x, imu_dt.gyro_y, imu_dt.gyro_z, 
                    imu_dt.temp);
                break;
            }            
            default:
                assert(false);
                break;
            }

            memmove(data, data + idx + data_len, dataCnt - (idx + data_len));
            dataCnt -= idx + data_len;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}