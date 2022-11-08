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
#include <thread>
#define MID_LINE 307
#define WIDTH 160
#define YAW_FIX_SPEED 0.35f
#define ANGLE_ERR_LIMIT 0.05f
#define WALK_SPEED 1.5f
#define CY_LIMIT 195

//1500 1s左右

typedef enum task_stage
{
    /* data */
    walk = 0,
    turn,
    dump_material,
    climb_stair,
    avoidance,
    stay,
    finish
}task_stage_e;

typedef union dist_ee{
    float f;
    char c[4];
}dist_e;


int turn_flag = 0;
int area_flag = 0;
int turn_count = 0;
int cx=0,cy=280;
float dist = 0;
string area_type;
int motiontime_temp = 0;
int area_motiontime_temp = 0;
float yaw_zero_value=0.0f;
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
    void Walk_straight();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

//Socket接收信息
void Custom::UDPRecv()
{
    udp.Recv();
}

//Socket发送信息
void Custom::UDPSend()
{  
    udp.Send();
}

/*  控制直线行走函数  主要依靠的参数是偏转角度imu  即陀螺仪反馈的角度信息imu              */
/*  在遇到相应的偏航情况时，执行相对应的方向修正。                                       */
/*  当前行走角度-该方向的正确行走角度=当前偏离角度                                       */
/*  通过当前偏离角度，和设定的最大角度误差值的正负两个方向进行比较，判定是否需要修正。     */
/*  yaw_zero_value 按指定路线直线行走的角度                                            */

void Custom::Walk_straight()
{
    //旋转方向：顺时针减，逆时针加
    //case：标志着行走的阶段
    switch(turn_count){
        /*在阶段0 的行走过程中 对角度的修正 */
        case 0:         
            if(state.imu.rpy[2] - 0.0f - yaw_zero_value > ANGLE_ERR_LIMIT){
                printf("trun right\n");
                cmd.yawSpeed = -YAW_FIX_SPEED;
            }
            else if(state.imu.rpy[2] - 0.0f - yaw_zero_value < -ANGLE_ERR_LIMIT){
                printf("trun left\n");
                cmd.yawSpeed = YAW_FIX_SPEED;
            }
            else{
                cmd.yawSpeed = 0.0f;
            }
            break;
        case 2:
        /*在阶段1 的行走过程中 对角度的修正 */
        case 1:
            if(state.imu.rpy[2] - (-1.57f) - yaw_zero_value > ANGLE_ERR_LIMIT){
                printf("trun right\n");
                cmd.yawSpeed = -YAW_FIX_SPEED;
            }
            else if(state.imu.rpy[2] - (-1.57f) - yaw_zero_value < -ANGLE_ERR_LIMIT){
                printf("trun left\n");
                cmd.yawSpeed = YAW_FIX_SPEED;
            }
            else{
                cmd.yawSpeed = 0.0f;
            }
            break;
        case 4:
        /*在阶段3 的行走过程中 对角度的修正 */
        case 3:
            if(state.imu.rpy[2] < 0){
                if(state.imu.rpy[2] - (-3.14f) - yaw_zero_value > ANGLE_ERR_LIMIT){
                    cmd.yawSpeed = -YAW_FIX_SPEED;
                }
                else{
                    cmd.yawSpeed = 0.0f;
                }
            }
            else if(state.imu.rpy[2] > 0){
                if(state.imu.rpy[2] - 3.14f - yaw_zero_value < -ANGLE_ERR_LIMIT){
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
        case 6:
        /*在阶段5 的行走过程中 对角度的修正 */
        case 5:
            if(state.imu.rpy[2] - 1.57f - yaw_zero_value > ANGLE_ERR_LIMIT){
                cmd.yawSpeed = -YAW_FIX_SPEED;
            }
            else if(state.imu.rpy[2] - 1.57f - yaw_zero_value < -ANGLE_ERR_LIMIT){
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

/*
根据任务需要，我们定义了当前A1机器狗的状态，共有7个状态，分别为：停止、行走、倾倒、转向、过台阶、避障、分别完成对应的任务。
当前的状态由于视觉识别识别的信息所决定
在每个状态下的运动 都会对机器狗进行联合调控 保持正确行走
*/

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);

    // printf("%f %f %f %f %f\n", state.imu.rpy[1], state.imu.rpy[2], state.position[0], state.position[1], state.velocity[0]);
    //cout<<"imustate"<<state.imu.rpy[2]<<endl;
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

    //printf("%f\n",cy);
    /*set mode*/
    if(task_stage != stay){
        if(cy < CY_LIMIT && !turn_flag && !area_flag){
            
            if(turn_count % 2 == 0 && turn_count != 6){
                task_stage = turn;
                printf("now turn\n");
                
            }
            else{
                if(turn_count == 1){
                    task_stage = dump_material;
                    printf("now dump_material\n");
                    
                }
                else if(turn_count == 3){
                    task_stage = climb_stair;
                }
                else if(turn_count == 5){
                    task_stage = avoidance;
                }
                else if(turn_count == 6){
                    task_stage = finish;
                }
                else{
                    task_stage = turn;
                }

            }
            // if(turn_count==6){
            //     task_stage = stay;
            // }
        }else if(cy >= CY_LIMIT && !turn_flag && !area_flag){
            task_stage = walk;
        }

    }
    
    if(task_stage == walk){
        // printf(" ");
        // printf("speed: %f,yaw: %f\n", state.velocity[0],state.velocity[1]);
        // printf("ang: %f\n",state.yawSpeed);
        static float yaw_zero_value_temp = state.imu.rpy[2]; //每次启动设置陀螺仪零点值
        yaw_zero_value = yaw_zero_value_temp;

        cmd.mode=2;
        cmd.gaitType=1;
        // cmd.footRaiseHeight = 0.09f;
        // cmd.velocity[0]=0.1f;
        cmd.velocity[0] = WALK_SPEED;

        if(cx - MID_LINE > 30){//跑直线
            cmd.velocity[1] = -0.25f;  //向右
            cout<<"move right"<<endl;
            //printf("vel1: %f\n",state.velocity[1]);

        }
        else if(cx - MID_LINE < -30){
            cmd.velocity[1] = 0.25f;
            cout<<"move left"<<endl;
            //printf("vel1: %f\n",state.velocity[1]);
        }
        else{
            cmd.velocity[1] = 0.0f;
        }

        Walk_straight();

    }
    else if(task_stage == turn){ // 降低速度拐弯
        if(!turn_flag){
            motiontime_temp = motiontime;
            cmd.velocity[0] = 0.0f;
        }
        turn_flag = 1;
        cmd.mode=2;
        cmd.gaitType=1;
        cmd.velocity[1] = 0.0f;
        
        if(motiontime - motiontime_temp < 800){
            // if(motiontime - motiontime_temp>700){
            //     cmd.velocity[0] = 0.3f;
            // }
            cmd.yawSpeed = -2.5f;
        }
        else{
            cmd.yawSpeed = 0.0f;
            if(cy > CY_LIMIT){
                turn_count++;
                turn_flag = 0;
            }


        }
    }
    else if(task_stage == dump_material){
        if(!area_flag){
            printf("进入倾倒物料状态\n");
            area_motiontime_temp = motiontime;
            cmd.yawSpeed = 0.0f;
            cmd.velocity[1] = 0.0f;
        }
        
        area_flag = 1;
        cmd.mode=2;
        cmd.gaitType=1;
        // cmd.velocity[1] = 0.0f;
        // cmd.yawSpeed = 0.0f;

        if(motiontime - area_motiontime_temp < 900){
            cmd.velocity[0] = WALK_SPEED;
            Walk_straight();

        }
        //切换模式缓冲
        else if(motiontime - area_motiontime_temp >= 900 && motiontime - area_motiontime_temp < 1300){
            cmd.velocity[0] = 0.0f;
              
        }
        else if(motiontime - area_motiontime_temp >= 1300 && motiontime - area_motiontime_temp < 1900){
            cmd.mode = 1;
            cmd.gaitType = 0;
            cmd.velocity[0] = 0.0f;
            cmd.euler[1] = -1.0f;
        }
        //缓冲
        else if(motiontime - area_motiontime_temp >= 1900 && motiontime - area_motiontime_temp < 2000){
            cmd.euler[1] = 0.0f;
        }
        else if(motiontime - area_motiontime_temp >= 2000 && motiontime - area_motiontime_temp < 2700)
        {
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[0] = WALK_SPEED;
            Walk_straight();
        }
        else{
            area_flag = 0;
            turn_count++;
        }

    }
    else if(task_stage == climb_stair){
       if(!area_flag){
            printf("进入爬楼梯状态\n");
            area_motiontime_temp = motiontime;
            cmd.yawSpeed = 0.0f;
            cmd.velocity[1] = 0.0f;
        }
        area_flag = 1;
        cmd.mode=2;
        cmd.gaitType=3;
        cmd.speedLevel = 0;
        cmd.footRaiseHeight = 0.14f;
 
        if(motiontime - area_motiontime_temp <= 2100){
            cmd.velocity[0] = 1.0f; 
            Walk_straight();
        }
        else if(motiontime - area_motiontime_temp > 2100 && motiontime - area_motiontime_temp <= 2900)
        {
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[0] = 1.0f;
            Walk_straight();
        }
        else{
            area_flag = 0;
            turn_count++;
        }


    }
    else if(task_stage == avoidance){
       if(!area_flag){
            printf("进入避障状态\n");
            area_motiontime_temp = motiontime;
            cmd.yawSpeed = 0.0f;
            cmd.velocity[1] = 0.0f;
        }
        area_flag = 1;
        cmd.mode=2;
        cmd.gaitType=1;

        if(motiontime - area_motiontime_temp < 600){ // 直走
            cmd.velocity[0] = WALK_SPEED;
            cmd.velocity[1] = 0.0f;
            Walk_straight();
        }
        else if(motiontime - area_motiontime_temp >= 600 && motiontime - area_motiontime_temp <1200){
            cmd.velocity[0] = 0.0f;
        }
        else if(motiontime - area_motiontime_temp >= 1200 && motiontime - area_motiontime_temp < 1900){
            //慢前走并右拐
            cmd.velocity[0] = 0.0f; 
            cmd.velocity[1] = 1.0f;

            //Walk_straight();

        }
        else if(motiontime - area_motiontime_temp >= 1900 && motiontime - area_motiontime_temp <= 3820)
        {
            //zhizou
            cmd.velocity[0] = 0.9f;
            cmd.velocity[1] = 0.0f;
            Walk_straight();
        }
        else if(motiontime - area_motiontime_temp > 3820 && motiontime - area_motiontime_temp <= 4220){
            cmd.velocity[0] = 0.0f;
            cmd.velocity[1] = 0.0f;
        }
        else if(motiontime - area_motiontime_temp > 4220 && motiontime - area_motiontime_temp <= 4920)
        {
            //慢直走并左靠
            cmd.velocity[0] = 0.0f;
            cmd.velocity[1] = -1.0f;
            Walk_straight();
        }
        else if(motiontime - area_motiontime_temp > 4920 && motiontime - area_motiontime_temp <= 5020)
        {
            cmd.velocity[0] = WALK_SPEED;
            cmd.velocity[1] = 0.0f;
            Walk_straight();

        }
        else{
            if(cy > CY_LIMIT){
                area_flag = 0;
                turn_count++;
            }
            else{
                cmd.velocity[0] = WALK_SPEED;
                cmd.velocity[1] = 0.0f;
                Walk_straight();

            }
        }

    }
    else if(task_stage == finish){
       if(!area_flag){
            printf("finish\n");
            area_motiontime_temp = motiontime;
            cmd.yawSpeed = 0.0f;
            cmd.velocity[1] = 0.0f;
        }
        area_flag = 1;
        cmd.mode=2;
        cmd.gaitType=1;
 
        if(motiontime - area_motiontime_temp <= 1040){
            cmd.velocity[0] = 1.0f; 
            cmd.velocity[1] = 0.0f;

            //Walk_straight();
        }
        else{
            area_flag = 0;
            turn_count++;
            
            cmd.velocity[0] = 0.0f;
            cmd.velocity[1] = 0.0f;
            task_stage = stay;
        }
    }
    else{
            cmd.velocity[0] = 0.0f;
            cmd.velocity[1] = 0.0f;
            cmd.yawSpeed = 0.0f;

    }


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
        //std::cout<<"cx: "<<cx<<" cy: "<<cy<<std::endl;
        // std::cout<<"forward:"<<1.0f*(cx-MID_LINE)/10.0f<<std::endl;

    }
}

void UDP_Recv3(void)    //接收深度学习的深度和类别信息
{
    dist_e dist_rev;

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
    char recv_buf[9]={0};
    struct sockaddr_in addr_client;
    string type_temp;
    

    while(1){
        read(sock_fd, recv_buf, sizeof(recv_buf));
        dist_rev.c[0] = recv_buf[0];dist_rev.c[1] = recv_buf[1];dist_rev.c[2] = recv_buf[2];dist_rev.c[3] = recv_buf[3];
        dist=dist_rev.f;
        // dist = fourCharToInt(recv_buf[3], recv_buf[2], recv_buf[1], recv_buf[0]);
        type_temp = "";
        for(int i=4;i<8;++i){
            type_temp+=recv_buf[i];
        }
        area_type = type_temp;
        std::cout<<"dist: "<<dist<<" area_type: "<<area_type<<std::endl;

    }
}


int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue...." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));
    //LoopFunc loop_udpRecv2("udp_recv2",custom.dt,3, boost::bind(&UDP_Recv2));
    LoopFunc loop_udpRecv3("udp_recv3",custom.dt,3, boost::bind(&UDP_Recv3));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    // loop_udpRecv2.start();
    thread test1(UDP_Recv2);
    loop_udpRecv3.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
