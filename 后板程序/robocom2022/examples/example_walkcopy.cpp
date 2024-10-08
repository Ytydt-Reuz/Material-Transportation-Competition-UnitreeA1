/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include "unitree_legged_sdk/a1_task.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>   
#include <netinet/in.h>   
#include <unistd.h>   
#include <errno.h>   

#define MID_LINE 280
#define WIDTH 160
#define YAW_FIX_SPEED 0.2f

typedef enum task_stage
{
    /* data */
    walk = 0,
    turn,
    dump_material,
    climb_stair,
    avoidance,
    finish
}task_stage_e;


int turn_flag = 0;
int turn_count = 0;
int cx=0,cy=280;
int motiontime_temp = 0;
task_stage_e task_stage;

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);

    // printf("%f %f %f %f %f\n", state.imu.rpy[1], state.imu.rpy[2], state.position[0], state.position[1], state.velocity[0]);

    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    /*set mode*/
    if(cy<150 && !turn_flag){
        task_stage = turn;
    }
    else if(cy>150 && !turn_flag){
        task_stage = walk;
    }

    if(task_stage == walk){
        cmd.mode=2;
        cmd.gaitType=1;
        cmd.velocity[0]=1.0f;

        if(cx - MID_LINE > 50){//跑直线
            cmd.velocity[1] = -0.2f;
        }
        else if(cx - MID_LINE < -50){
            cmd.velocity[1] = 0.2f;
        }
        else{
            cmd.velocity[1] = 0.0f;
        }

        //顺时针减，逆时针加
        switch(turn_count){
            case 0:
                if(state.imu.rpy[2] - 0.0f > 0.06f){
                    cmd.yawSpeed = -YAW_FIX_SPEED;
                }
                else if(state.imu.rpy[2] - 0.0f < -0.06f){
                    cmd.yawSpeed = YAW_FIX_SPEED;
                }
                else{
                    cmd.yawSpeed = 0.0f;
                }
                break;
            case 1:
                if(state.imu.rpy[2] - (-1.57f) > 0.06f){
                    cmd.yawSpeed = -YAW_FIX_SPEED;
                }
                else if(state.imu.rpy[2] - (-1.57f) < -0.06f){
                    cmd.yawSpeed = YAW_FIX_SPEED;
                }
                else{
                    cmd.yawSpeed = 0.0f;
                }
                break;
            case 2:
                if(state.imu.rpy[2] < 0){
                    if(state.imu.rpy[2] - (-3.14f) > 0.06f){
                        cmd.yawSpeed = -YAW_FIX_SPEED;
                    }
                    else{
                        cmd.yawSpeed = 0.0f;
                    }
                }
                else if(state.imu.rpy[2] > 0){
                    if(state.imu.rpy[2] - 3.14f < -0.06f){
                        cmd.yawSpeed = YAW_FIX_SPEED;
                    }
                    else{
                        cmd.yawSpeed = 0.0f;
                    }
                }
                else{
                    cmd.yawSpeed = 0.0f;
                }
                break;
            case 3:
                if(state.imu.rpy[2] - 1.57f > 0.06f){
                    cmd.yawSpeed = -YAW_FIX_SPEED;
                }
                else if(state.imu.rpy[2] - 1.57f < -0.06f){
                    cmd.yawSpeed = YAW_FIX_SPEED;
                }
                else{
                    cmd.yawSpeed = 0.0f;
                }
                break;
            default:
                cmd.yawSpeed = 0.0f;
                break;
        }

    }
    else if(task_stage == turn){ // 降低速度拐弯
        if(!turn_flag){
            motiontime_temp = motiontime;
        }
        turn_flag = 1;
        cmd.mode=2;
        cmd.gaitType=1;
        cmd.velocity[1] = 0.0f;
        cmd.velocity[0] = 0.12f;
        if(motiontime - motiontime_temp < 2400){
            cmd.yawSpeed = -1.0f;
        }
        else{
            cmd.yawSpeed = 0.0f;
            turn_count++;
            turn_flag = 0;
        }
    }
    else if(task_stage == dump_material){

    }
    else if(task_stage == climb_stair){
        cmd.mode=2;
        cmd.gaitType=3;
        cmd.footRaiseHeight = 0.8f;

    }
    else if(task_stage == avoidance){

    }
    else{

    }

    // if(motiontime <= 3500)
    // {
    //     cmd.mode=2;
    //     cmd.gaitType=1;
    //     cmd.velocity[0]=0.45f;
    //     cmd.yawSpeed = 1.0f*(cx-MID_VALUE)/10.0f;
    //     // cmd.yawSpeed = 0.35f;//逆时针为正，顺时针为负
    // }
    // else{
    //     cmd.mode=0;
    //     cmd.gaitType=0;
    //     cmd.velocity[0]=0.0f;
    //     cmd.yawSpeed = 0.0f;
    // }

    udp.SetSend(cmd);
}

int fourCharToInt(unsigned char a, unsigned char b,unsigned char c,unsigned char d){
	 int ret_val = 0;
	ret_val  =  a;
	ret_val <<= 8;
	ret_val |=  b;
	ret_val <<= 8;
	ret_val |=  c;
	ret_val <<= 8;
	ret_val |=  d;

	return ret_val;
}

void UDP_Recv2(void)//接受来自视觉的识别信息
{
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
    addr_serv.sin_family = AF_INET;//使用IPV4地址
    addr_serv.sin_port = htons(12000);//端口
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
    len = sizeof(addr_serv);

    /* 绑定socket */
    if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
    {
        perror("bind error:");
        exit(1);
    }


    int send_num;
    char send_buf[20] = "i am server!";
    char recv_buf[8]={0};
    struct sockaddr_in addr_client;
    

    while(1){
        read(sock_fd, recv_buf, sizeof(recv_buf));
        cx = fourCharToInt(recv_buf[3], recv_buf[2], recv_buf[1], recv_buf[0]);
        cy = fourCharToInt(recv_buf[7], recv_buf[6], recv_buf[5], recv_buf[4]);
        std::cout<<"cx: "<<cx<<" cy: "<<cy<<std::endl;
        // std::cout<<"forward:"<<1.0f*(cx-MID_LINE)/10.0f<<std::endl;

    }
}

void UDP_Recv3(void)//接受来自视觉的识别信息
{
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
    addr_serv.sin_family = AF_INET;//使用IPV4地址
    addr_serv.sin_port = htons(12001);//端口
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
    len = sizeof(addr_serv);

    /* 绑定socket */
    if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
    {
        perror("bind error:");
        exit(1);
    }


    int send_num;
    char send_buf[20] = "i am server!";
    char recv_buf[8]={0};
    struct sockaddr_in addr_client;
    

    while(1){
        read(sock_fd, recv_buf, sizeof(recv_buf));
        // cx = fourCharToInt(recv_buf[3], recv_buf[2], recv_buf[1], recv_buf[0]);
        // cy = fourCharToInt(recv_buf[7], recv_buf[6], recv_buf[5], recv_buf[4]);
        std::cout<<"cx: "<<cx<<" cy: "<<cy<<std::endl;

    }
}


int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));
    LoopFunc loop_udpRecv2("udp_recv2",custom.dt,3, boost::bind(&UDP_Recv2));
    LoopFunc loop_udpRecv3("udp_recv3",custom.dt,3, boost::bind(&UDP_Recv3));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    loop_udpRecv2.start();
    loop_udpRecv3.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
