#ifndef _A1_TASK_H_
#define _A1_TASK_H_

#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

#define LONGSIDE 1.2f//8.6f
#define SHORTSIDE 1.2f//5.2f
#define PI 3.1415926f

typedef enum task_stage
{
    /* data */
    walk = 0,
    turn,
    dump_material,
    climb_stair,
    finish
}task_stage_e;


typedef struct a1_task_state
{
    /* data */
    task_stage_e stage;
    bool finish_flag;

}a1_task_t;
bool walk_reset(HighCmd* cmd, HighState state);

bool walk1_task(HighCmd* cmd, HighState state);
bool dump_material_task(HighCmd* cmd, HighState state);
bool walk2_task(HighCmd* cmd, HighState state);
bool climb_stair_task(HighCmd* cmd, HighState state);
bool walk3_task(HighCmd* cmd, HighState state);
bool finish_task(HighCmd* cmd, HighState state);


#endif