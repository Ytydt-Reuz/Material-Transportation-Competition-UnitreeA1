/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/a1_task.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

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
    a1_task_t a1_task;
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

    if(motiontime == 4)
    {
        a1_task.stage = walk1;
        a1_task.finish_flag = 0;
    }

    // if(walk_reset(&cmd,state) && !a1_task.finish_flag)
    // {
    //     a1_task.stage  = walk1;
    //     a1_task.finish_flag = 1;
    // }
    // else if(!walk_reset(&cmd,state) && !a1_task.finish_flag){
    //     a1_task.stage =finish;
    // }

    if(a1_task.stage == walk1)
    {
        printf("walk1_statge, now_x:%f, now_y:%f, now_ang:%f\n", state.position[0], state.position[1], state.imu.rpy[2]);
        if(walk1_task(&cmd, state) == true)
        {
            a1_task.stage = dump_material;
        }
    }
    else if(a1_task.stage == dump_material)
    {
        if(dump_material_task(&cmd, state) == true)
        {
            a1_task.stage = walk2;
        }
    }
    // else if(a1_task.stage == walk2)
    // {
    //     if(!walk2_task(&cmd, state) == 1)
    //     {
    //         a1_task.stage = climb_stair;
    //     }
    // }
    // else if(a1_task.stage == climb_stair)
    // {
    //     if(!climb_stair_task(&cmd, state) == 1)
    //     {
    //         a1_task.stage = walk3;
    //     }
    // }
    // else if(a1_task.stage == walk3)
    // {
    //     if(!walk3_task(&cmd, state) == 1)
    //     {
    //         a1_task.stage = finish;
    //     }
    // }
    // else if(a1_task.stage == finish)
    // {
    //     finish_task(&cmd, state);
    // }




    // if(motiontime > 1000 && motiontime < 3000){
    //     cmd.mode = 2;
    //     cmd.gaitType = 3;
    //     cmd.velocity[0] = 0.4f; // -1  ~ +1
    //     cmd.yawSpeed = 0.0f;
    //     cmd.footRaiseHeight = 0.12;
    //     // printf("walk\n");
    // }
    // if(fabs(state.position[0] - cmd.postion[0]) > 0.1){
    //     cmd.mode = 3;
    //     cmd.gaitType = 1;
    //     cmd.speedLevel = 1;
    //     cmd.postion[0] = 1.8f;
    //     cmd.postion[1] = 0.0f;
    //     cmd.footRaiseHeight = 0.08;
    //     // printf("walk\n");
    // }

    // if(motiontime>3000 ){
    //     cmd.mode = 1;
    // }

    udp.SetSend(cmd);
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

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
