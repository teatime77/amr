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
#include "std_msgs/msg/float32.hpp"
#include "cart_interfaces/srv/motor_pwm.hpp"

void init_qt(int argc, char * argv[]);
void process_qt();

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

#ifndef BYTE
#define BYTE    unsigned char
#endif

#define PULSE_PER_ROUND -907.5625
#define WHEEL_RADIUS    0.03

#define RANGES_SIZE     1024 * 8
#define MIN_PWM         60

const float Kp = 200.0;
float CartVel = 0;
float VelR;
float VelL;
int   pwmL = 0;
int   pwmR = 0;
bool  timer_called = false;

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

const float wheel_separation = 0.2;

int encR, encL;

void sub(float Vl, float Vr, float& R, float& omega){
    if(Vl == Vr){
        R = std::numeric_limits<float>::infinity();
        omega = 0;
    }
    else if(Vl == -Vr){
        R = 0;
        omega = Vl / (0.5 * wheel_separation);
    }
    else if(Vl == 0){
        R = 0.5 * wheel_separation;
        omega = Vr / wheel_separation;
    }
    else if(Vr == 0){

        R = 0.5 * wheel_separation;
        omega = - Vl / wheel_separation;
    }
    else{

    }
}

void ForwardDiffBot(float R, float omega, float dt, float x, float y, float theta, float& x2, float& y2, float& theta2){
    float ICCx = x - R * sin(theta);
    float ICCy = y + R * cos(theta);

    float Rx = x - ICCx;
    float Ry = y - ICCy;

    float omega_dt = omega * dt;

    x2 = cos(omega_dt) * Rx - sin(omega_dt) * Ry + ICCx;
    y2 = sin(omega_dt) * Rx + cos(omega_dt) * Ry + ICCy;

    theta2 = theta + omega_dt;
}

struct EncoderData {
    char  mark[4];
    int   msec;
    int   counts[2];
};

int little_int(BYTE* dt, int idx){
    return int(dt[idx] + 256 * (int)dt[idx + 1]);
}

void setMotorPWM(const std::shared_ptr<cart_interfaces::srv::MotorPWM::Request> request,
          std::shared_ptr<cart_interfaces::srv::MotorPWM::Response>      /*response*/){

    (void)request;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nleft: %d" " right: %d", request->left, request->right);
}

class MinimalPublisher : public rclcpp::Node
{
    int sockfd;
    bool connected = false;
    BYTE data[1024*8];
    int dataCnt = 0;

    float ranges[RANGES_SIZE];
    int nRanges = 0;

    float prevFSA = NAN;
    float prevLSA = NAN;

    clock_t startTime;
    clock_t prevTime;

    rclcpp::Service<cart_interfaces::srv::MotorPWM>::SharedPtr service;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wheelVecR_sub;

public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
        enc_L_pub = this->create_publisher<std_msgs::msg::Float32>("encoder_L", 10);
        enc_R_pub = this->create_publisher<std_msgs::msg::Float32>("encoder_R", 10);

        wheelVecR_sub = this->create_subscription<std_msgs::msg::Float32>("cartVel", 10, std::bind(&MinimalPublisher::cartVel_callback, this, std::placeholders::_1));

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

            connected = true;
            RCLCPP_INFO(this->get_logger(), "connected");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "connect error");

            return;
        }

        sendPWM(0, 0);

        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));

        service = this->create_service<cart_interfaces::srv::MotorPWM>("set_motor_pwm", &setMotorPWM);

        startTime = clock();
    }

    ~MinimalPublisher(){
        //ソケットクローズ
        close(sockfd);
    }

    void sendPWM(int pwm_l, int pwm_r){
        static short prev_l, prev_r;

        if(prev_l == pwm_l && prev_r == pwm_r){
            return;
        }

        char buf[6];

        buf[0] = '\xFF';
        buf[1] = '\xEE';

        *(short*)(buf + 2) = (short)pwm_l;
        *(short*)(buf + 4) = (short)pwm_r;

        assert(sizeof(short) == 2);
 
        if(connected){

            send(sockfd, buf, 2 + 2 + 2, 0); //送信
        }

        prev_l = pwm_l;
        prev_r = pwm_r;
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

        scan_pub->publish(*scan_msg);

        // RCLCPP_INFO(this->get_logger(), "pub stamp:%d:%d time:%.3lf nRanges:%d FSA-LSA:%d %d", scan_msg->header.stamp.sec, scan_msg->header.stamp.nanosec, scan_msg->scan_time, nRanges, int(prevFSA), int(prevLSA));

        prevTime = current_time;
    }

    void readHead(BYTE* dt, int LSN){
        // int CT  = dt[2] & 0x01;

        int iFSA = little_int(dt, 4);
        int iLSA = little_int(dt, 6);
        int CS  = little_int(dt, 8);

        float FSA = (iFSA >> 1) / 64.0;
        float LSA = (iLSA >> 1) / 64.0;
        // RCLCPP_INFO(this->get_logger(), "CT:%X FSA:%d LSA:%d", dt[2], int(FSA), int(LSA));

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
            else if(data[i] == 0xCC && data[i + 1] == 0x77 && i + int(sizeof(EncoderData)) < dataCnt){
                data_type = DataType::encoder;
                return i;
            }
        }

        return -1;
    }

private:
    void cartVel_callback(const std_msgs::msg::Float32 & msg) const{
        CartVel = msg.data;
        RCLCPP_INFO(this->get_logger(), "cart vel:%.2f", CartVel);
    }

    void timer_callback()
    {
        timer_called = true;

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

                // RCLCPP_INFO(this->get_logger(), "acc:(%.1f, %.1f, %.1f) gyro:(%.1f, %.1f, %.1f) temp:%.1f", 
                //     imu_dt.acc_x, imu_dt.acc_y, imu_dt.acc_z, 
                //     imu_dt.gyro_x, imu_dt.gyro_y, imu_dt.gyro_z, 
                //     imu_dt.temp);
                break;
            }            

            case DataType::encoder:{
                EncoderData enc_dt;
                
                data_len = sizeof(EncoderData);
                memcpy(&enc_dt, data + idx, data_len);

                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                enc_dt.counts[0] *= -1;
                enc_dt.counts[1] *= -1;
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                encR += enc_dt.counts[0];
                encL += enc_dt.counts[1];

                auto msg_R = std_msgs::msg::Float32();
                auto msg_L = std_msgs::msg::Float32();

                // 変位角(radian)
                float thetaR = - 2.0 * M_PI * enc_dt.counts[0] / PULSE_PER_ROUND;
                float thetaL =   2.0 * M_PI * enc_dt.counts[1] / PULSE_PER_ROUND;

                // 角速度(radian/S)
                float omegaR = 1000.0 * thetaR / float(enc_dt.msec);
                float omegaL = 1000.0 * thetaL / float(enc_dt.msec);

                // 速度(m/S)
                VelR = WHEEL_RADIUS * omegaR;
                VelL = WHEEL_RADIUS * omegaL;

                msg_R.data = VelR;
                msg_L.data = VelL;
 
                enc_R_pub->publish(msg_R);
                enc_L_pub->publish(msg_L);

                // RCLCPP_INFO(this->get_logger(), "enc: %d msec %d %d", enc_dt.msec, enc_dt.counts[0], enc_dt.counts[1]);
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
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr enc_L_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr enc_R_pub;

    
    size_t count_;
};

static std::shared_ptr<MinimalPublisher>    node;

int calcPWM(int current_pwm, float current_vel, float target_vel){
    if(target_vel == 0){
        return 0;
    }

    float   P = target_vel - current_vel;

    int pwm = current_pwm + int(Kp * P);
    if(0 < target_vel){
        return std::max(MIN_PWM, std::min(255, pwm));
    }
    else{
        
        return std::max(-255, std::min(-MIN_PWM, pwm));
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    init_qt(argc, argv);

    node = std::make_shared<MinimalPublisher>();
    while (true){
        rclcpp::spin_some(node);
        process_qt();

        if(timer_called){
            timer_called = false;

            pwmR = calcPWM(pwmR, VelR, CartVel);
            pwmL = calcPWM(pwmL, VelL, CartVel);

            RCLCPP_INFO(node->get_logger(), "R PWM:%d vel:%.2f target:%.2f", pwmR, VelR, CartVel);
            RCLCPP_INFO(node->get_logger(), "L PWM:%d vel:%.2f target:%.2f", pwmL, VelL, CartVel);

            node->sendPWM(pwmL, -pwmR);
        }
   }
    
    rclcpp::shutdown();
    return 0;
}
