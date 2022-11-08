#include "unitree_legged_sdk/a1_task.h"
#include <math.h>
#include <unistd.h>


using namespace UNITREE_LEGGED_SDK;

void fix_position(HighCmd* cmd,HighState state, float positon0, float positon1, float yaw)
{}

bool walk_reset(HighCmd* cmd, HighState state)
{
    cmd->mode = 3;
    cmd->gaitType = 1;

    cmd->postion[0] = 0.0f;
    cmd->postion[1] = 0.0f;

    if(fabs(cmd->postion[0] - state.position[0]) < 0.2f && fabs(cmd->postion[1] - state.position[1]) < 0.5f)
    {
        return true;
    }
    return false;
}


bool walk1_task(HighCmd* cmd, HighState state)
{
    static unsigned char walk1_stage = 0;
    bool check[3] = {0};
    printf("wal1阶段:%i\n",walk1_stage);

    cmd->mode = 2;
    cmd->gaitType = 1;

    if(walk1_stage == 0)  //直走
    {
        cmd->postion[0] = SHORTSIDE;
        cmd->postion[1] = 0.0f;
        cmd->velocity[0] = 0.5f;

        if(fabs(cmd->postion[0] - state.position[0]) < 0.08f && fabs(cmd->postion[1] - state.position[1]) < 0.5f)
        {
            check[walk1_stage++] = 1;
        }
        else if(cmd->postion[0] > state.position[0])
        {
            cmd->velocity[0] = 0.5f;
        }
        else if(cmd->postion[0] < state.position[0])
        {
            cmd->velocity[0] = -0.5f;
        }
    }
    else if(walk1_stage == 1)  //该转弯了
    {
        cmd->mode = 2;
        
        // cmd->euler[2] = -PI/2.0f;
        fix_position(cmd, state, 5.1f, 0.0f, 0.0f);
        cmd->velocity[0] = 0.0f;
        cmd->yawSpeed = -1.5f;

        if(fabs((-PI/2.0f) - state.imu.rpy[2]) < fabs(-PI/2.0f)/20.0f)
        {
            check[walk1_stage++] = 1;
        }

    }
    else if(walk1_stage == 2)
    {
        cmd->mode = 2;
        cmd->velocity[0] = 0.5f;
        cmd->yawSpeed = 0.0f;

        cmd->postion[0] = SHORTSIDE;
        cmd->postion[1] = -LONGSIDE/2.0f;
        fix_position(cmd, state, cmd->postion[0], cmd->postion[1], 0.0f);

        if(fabs(cmd->postion[0] - state.position[0]) < 0.1f && fabs(cmd->postion[1] - state.position[1]) < 0.5f)
        {
            check[walk1_stage++] = 1;
        }
        else if(cmd->postion[1] > state.position[1])
        {
            cmd->velocity[0] = 0.5f;
        }
        else if(cmd->postion[1] < state.position[1])
        {
            cmd->velocity[0] = -0.5f;
        }

    }

    if(walk1_stage == 4)
    {
        return true;
    }
    return false;

}

bool dump_material_task(HighCmd* cmd, HighState state)
{
    cmd->mode = 1;
    cmd->gaitType = 0;
    cmd->euler[0] = -0.3f; //roll
    sleep(3);
    cmd->euler[0] = -0.0f;

    return true;
    
}

bool walk2_task(HighCmd* cmd, HighState state)
{
    static unsigned char walk2_stage = 0;
    bool check[4] = {0};

    cmd->mode = 3;
    cmd->gaitType = 1;

    fix_position(cmd, state, 5.1f, 4.45f, 0.0f);

    if(walk2_stage == 0)  //直走
    {
        cmd->postion[0] = SHORTSIDE;
        cmd->postion[1] = LONGSIDE;
        if(fabs(cmd->postion[0] - state.position[0]) < 0.1f && fabs(cmd->postion[1] - state.position[1]) < 0.1f)
        {
            check[walk2_stage++] = 1;
        }
    }
    else if(walk2_stage == 1)  //该转弯了
    {
        cmd->euler[2] = -PI;
        fix_position(cmd, state, 5.1f, 0.0f, 0.0f);

        if(fabs(cmd->euler[2] - state.imu.rpy[2]) < cmd->euler[2]/10.0f)
        {
            check[walk2_stage++] = 1;
        }

    }
    else if(walk2_stage == 2)
    {
        cmd->postion[0] = SHORTSIDE/2.0f;
        cmd->postion[1] = LONGSIDE;
        fix_position(cmd, state, cmd->postion[0], cmd->postion[0], 0.0f);

        if(fabs(cmd->postion[0] - state.position[0]) < 0.1f && fabs(cmd->postion[1] - state.position[1]) < 0.1f)
        {
            check[walk2_stage++] = 1;
        }

    }

    if(walk2_stage == 4)
    {
        return true;
    }
    return false;

}

bool climb_stair_task(HighCmd* cmd, HighState state)
{
    cmd->mode = 3;
    cmd->gaitType = 3;
    cmd->footRaiseHeight = 0.8f;
    cmd->speedLevel = 1;

    cmd->postion[0] += 1.8f;
    cmd->postion[1] = LONGSIDE;

    if(fabs(cmd->postion[0] - state.position[0]) < 0.1f && fabs(cmd->postion[1] - state.position[1]) < 0.1f)
    {
        return true;
    }
    return false;


}

bool walk3_task(HighCmd* cmd, HighState state)
{
    static unsigned char walk3_stage = 0;
    bool check[4] = {0};

    cmd->mode = 3;
    cmd->gaitType = 1;

    fix_position(cmd, state, 5.1f, 4.45f, 0.0f);

    if(walk3_stage == 0)  //直走
    {
        cmd->postion[0] = 0.0f;
        cmd->postion[1] = LONGSIDE;
        if(fabs(cmd->postion[0] - state.position[0]) < 0.1f && fabs(cmd->postion[1] - state.position[1]) < 0.1f)
        {
            check[walk3_stage++] = 1;
        }
    }
    else if(walk3_stage == 1)  //该转弯了
    {
        cmd->euler[2] = PI/2.0f;
        fix_position(cmd, state, 5.1f, 0.0f, 0.0f);

        if(fabs(cmd->euler[2] - state.imu.rpy[2]) < cmd->euler[2]/10.0f)
        {
            check[walk3_stage++] = 1;
        }

    }
    else if(walk3_stage == 2)
    {
        cmd->postion[0] = 0.0f;
        cmd->postion[1] = 0.0f;
        fix_position(cmd, state, cmd->postion[0], cmd->postion[0], 0.0f);

        if(fabs(cmd->postion[0] - state.position[0]) < 0.1f && fabs(cmd->postion[1] - state.position[1]) < 0.1f)
        {
            check[walk3_stage++] = 1;
        }

    }

    if(walk3_stage == 4)
    {
        return true;
    }
    return false;

}

bool finish_task(HighCmd* cmd, HighState state)
{
    cmd->mode = 0;
    cmd->gaitType = 0;
}