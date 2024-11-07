#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include <sensor_msgs/msg/joint_state.h>
#include "jointstate.h"
#include "syslog.h"
#include "secrets.hpp"
#include <map>
#include <memory>


#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif

#ifndef CONTROL_TIMER
#define CONTROL_TIMER 20 // 50Hz
#endif

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)


// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19
#include <SCServo.h>

void rclErrorLoop()
{
    while(true)
    {
        //Serial.println("ROS Error setup");
    }
}

std::map<String, int> name_to_id = {{"m1", 3}, {"m2",6}, {"m3",9}, {"m4",12}, {"m5", 15}, {"m6",18}};

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

rcl_publisher_t joint_state_publisher;
rcl_subscription_t joint_state_subscriber;

sensor_msgs__msg__JointState joint_msg;
sensor_msgs__msg__JointState joint_msg_sub;

bool joint_msg_recieved = false;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;



SMS_STS sms_sts;


bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
        #if (_POSIX_TIMERS > 0)
                // Get time in milliseconds or nanoseconds
                int64_t time_ns = rmw_uros_epoch_nanos();
            timespec tp;
            tp.tv_sec = time_ns / 1000000000;
            tp.tv_nsec = time_ns % 1000000000;
            clock_settime(CLOCK_REALTIME, &tp);
        #else
            unsigned long long ros_time_ms = rmw_uros_epoch_millis();
            // now we can find the difference between ROS time and uC time
            time_offset = ros_time_ms - millis();
        #endif
        return true;
    }
    return false;
}


struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

int get_motor_id(rosidl_runtime_c__String& name) {
    return name_to_id[String(name.data)];
}

rosidl_runtime_c__String conv_string_to_c_string(const String& str, char* buffer) 
{
    rosidl_runtime_c__String result;
    result.data = buffer;
    str.toCharArray(result.data, str.length() + 1);
    result.size = str.length();
    result.capacity = 20;

    return result;
}

bool names_initialized = false;

// This buffer is needed so we don't get a memory leak. It allows us to reuse this memory over and over. 
char joint_name_buffer[10][20];

void readServoIntoJointMsg()
{
    int i = 0;

    for ( auto [name, id] : name_to_id)
    {

        joint_msg.name.data[i] = conv_string_to_c_string(name, joint_name_buffer[i]);
        joint_msg.position.data[i] = sms_sts.ReadPos(id);
        joint_msg.velocity.data[i] = sms_sts.ReadSpeed(id);
        joint_msg.effort.data[i] = sms_sts.ReadCurrent(id);

        ++i;
    }

    joint_msg.position.size = i;
    joint_msg.name.size = i;
    joint_msg.velocity.size = i;
    joint_msg.effort.size = i;
}

void joint_callback(const void * msgin)
{
    joint_msg_recieved = true;
    // We can pass, because all we want is to set the memory.
    // sensor_msgs__msg__JointState *msg = (sensor_msgs__msg__JointState *) msgin;

    //for (int i = 0; i < msg->name.size; ++i) {
      //  desired_joint_data[i].position = msg->position.data[i];
        
    //}
    //desired_joint_data[0].position = msg->position.data[0];
    

}



s16 compute_motor_position(int id, double in_pos)
{
    return (s16)in_pos;
}

void move_motors()
{
    if (joint_msg_recieved) {
        int id; 
        for (int i = 0; i < joint_msg_sub.name.size; ++i)
        {
            id = get_motor_id(joint_msg_sub.name.data[i]);            
            sms_sts.RegWritePosEx(id, compute_motor_position(id, joint_msg_sub.position.data[i]), 3400, 50); // TODO: Maybe change sign of speed depending on which is closer.
            sms_sts.RegWriteAction();
        }
    
    }
}

void publishData()
{
    struct timespec time_stamp = getTime();

    joint_msg.header.stamp.sec = time_stamp.tv_sec;
    joint_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_msg, NULL));
}


void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        move_motors();
        readServoIntoJointMsg();
        publishData();
    }
}


bool createEntities()
{

    allocator = rcl_get_default_allocator();
    // init memory for msgs

    static micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_ros2_type_sequence_capacity = 6; // There are atleast 6 motors in the joint states. 

    RCCHECK( micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &joint_msg,
            conf));
    
    RCCHECK( micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &joint_msg_sub,
            conf));

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "st3215_control", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_state"));

    RCCHECK(rclc_subscription_init_default(
        &joint_state_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "desired_joint_state"
    ));

    const unsigned int control_timeout = CONTROL_TIMER;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));


    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
 
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &joint_state_subscriber,
        &joint_msg_sub,
        &joint_callback,
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();

    return true;
}

bool destroyEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    //TODO: make sure to clear joint_msgs to avoid leaks.

    RCSOFTCHECK(rcl_publisher_fini(&joint_state_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&joint_state_subscriber, &node));

    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node))
    RCSOFTCHECK(rclc_support_fini(&support));

    return true;
}


void setup()
{
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    Serial.begin(115200);
    sms_sts.pSerial = &Serial1;

    set_microros_serial_transports(Serial);

    delay(1000);
}

void loop () {
    switch (state)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            syslog(LOG_INFO, "%s agent available %lu", __FUNCTION__, millis());
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT)
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            syslog(LOG_INFO, "%s agent disconnected %lu", __FUNCTION__, millis());
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}