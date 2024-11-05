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

#ifndef jointstate_H
#define jointstate_H

#include <Arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/joint_state.h>
#include "config.h"

#ifndef POSE_COV
#define POSE_COV { 0.0001, 0.0001, 0, 0, 0, 0.0001 }
#endif
#ifndef TWIST_COV
#define TWIST_COV { 0.00001, 0.00001, 0, 0, 0, 0.00001 }
#endif


typedef struct  {
    int id;
    float position;
    float velocity;
    float effort;
} JointData;

class JointState
{
    public:
        JointState();
        void update(JointData* joint_data, int joint_len);
        sensor_msgs__msg__JointState getData();

    private:

        sensor_msgs__msg__JointState joint_msg_;

};

#endif
