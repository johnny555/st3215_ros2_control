// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "jointstate.h"
#include "string.h"


#define JOINT_NUM 4
#define ARRAY_LEN 10

JointState::JointState()
{   

    rosidl_runtime_c__String string_buffer[JOINT_NUM];

    joint_msg_.name.data = (rosidl_runtime_c__String *) malloc(JOINT_NUM*sizeof(rosidl_runtime_c__String));
    joint_msg_.name.size = 4;
    joint_msg_.name.capacity = JOINT_NUM;

    for (int i = 0; i < JOINT_NUM; ++i)
    {
        joint_msg_.name.data[i].data = (char*) malloc(5);
        joint_msg_.name.data[i].capacity = 5;
        sprintf(joint_msg_.name.data[i].data, "motor_%d", i+1);
        joint_msg_.name.data[i].size = strlen(joint_msg_.name.data[i].data);
    }

    double joint_states_pos_buffer[] = {0,0,0,0};
    double joint_states_vel_buffer[] = {0,0,0,0};
    double joint_states_eff_buffer[] = {0,0,0,0};

    joint_msg_.position.data = joint_states_pos_buffer;
    joint_msg_.position.size = 4;
    joint_msg_.position.capacity = 4;

    joint_msg_.velocity.data = joint_states_vel_buffer;
    joint_msg_.velocity.size = 4;
    joint_msg_.velocity.capacity = 4;

    joint_msg_.effort.data = joint_states_eff_buffer;
    joint_msg_.effort.size = 4;
    joint_msg_.effort.capacity = 4;

}

void JointState::update(JointData* joint_data, int joint_len)
{

    for (int i = 0; i < joint_len; i++) {
        joint_msg_.position.data[i] = joint_data[i].position;
        joint_msg_.velocity.data[i] = joint_data[i].velocity;
        joint_msg_.effort.data[i] = joint_data[i].effort;
    }
    
}

sensor_msgs__msg__JointState JointState::getData()
{
    return joint_msg_;
}

