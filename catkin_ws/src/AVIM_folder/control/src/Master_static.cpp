#include<iostream>
#include<ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include<math.h>
#include "object_detection/points_objects.h"
#include<string>
#include <vector>
#include<map>
#include <chrono>
#include <thread>
#include <std_msgs/Bool.h>

int LANE_DRIVING = 0;
int FOLLOWING = 1;
int MOVING_LEFT = 2;
int PASSING = 3;
int MOVING_RIGHT = 4;
int MOVING_RIGHT_LANE = 5;
int STOP_AT_SIGN = 6;

static std::map<int, std::string>task_names{
     { LANE_DRIVING,       "Lane driving"},
     { FOLLOWING,          "Following"},
     { MOVING_LEFT,        "Moving to left lane"},
     { PASSING,            "Passing obstacle" },
     { MOVING_RIGHT,       "Returning right lane"},
     { MOVING_RIGHT_LANE,  "Returning right lane to lane"},
     { STOP_AT_SIGN,       "Stopping at STOP sign"}
};


class Task{
    public:
        std::string name;
        int ID;

        Task(int task_identifier){
            this->ID = task_identifier;
            this->name = task_names[this->ID];
        }
};



class Master{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber distance_center;
        ros::Subscriber object_detection;
        ros::Subscriber stop_sign_sub;
        ros::Publisher angle_pub;
        ros::Publisher speed_pub;
        ros::Publisher left_signal_pub;
        ros::Publisher right_signal_pub;
        ros::Publisher blinkers_pub;
        ros::Publisher rear_lights_pub;   // luces de freno traseras

        std_msgs::Int16 angle_message;
        std_msgs::Int16 speed_message;

        std::vector<geometry_msgs::Point> found_objects;
        int num_found_objects;
        int dist_now;
        int dist_last;
        int angle_now;
        int angle_last;
        float kp_angle;
        float kd_angle;
        float error_angle;
        int u_angle;
        int angle_pd;
        int speed_pid;
        float kp_speed;
        int u_speed;
        bool passing_enabled;
        int time_long;
        std::vector<Task> task_pile; 
        Task *last_task;
        int count;
        int count_pass;
        int max_waiting_time;
        float dist_to_keep;
        int vel_decreasing_factor;
        int mid_speed;
        bool stop_sign_detected;
        bool stop_initiated;
        std::chrono::steady_clock::time_point start;
        std::chrono::steady_clock::time_point stop_start_time;
        std::chrono::steady_clock::time_point end;

        bool left_signal_on;
        bool right_signal_on;
        bool blinkers_on;
        bool rear_lights_on;   // estado actual de luces traseras de freno
   
    public:
        Master(Task task, bool PASSING_ENABLED, int MAX_WAIT_TIME, float DIST_TO_KEEP){
            distance_center  = nh_.subscribe("/distance_center_line", 1, &Master::dist_center_clbk, this);
            object_detection = nh_.subscribe("/objects_points",        1, &Master::object_detec_clbk, this);
            stop_sign_sub    = nh_.subscribe("/stop_sign_detected",    1, &Master::stop_sign_callback, this);

            angle_pub        = nh_.advertise<std_msgs::Int16>("/AutoModelMini/manual_control/steering", 1000);
            speed_pub        = nh_.advertise<std_msgs::Int16>("/AutoModelMini/manual_control/speed",    1000);
            left_signal_pub  = nh_.advertise<std_msgs::Bool>("/lights/left_signal",  1);
            right_signal_pub = nh_.advertise<std_msgs::Bool>("/lights/right_signal", 1);
            blinkers_pub     = nh_.advertise<std_msgs::Bool>("/lights/blinkers",     1);
            rear_lights_pub  = nh_.advertise<std_msgs::Bool>("/lights/rear",         1);

            num_found_objects  = 0;
            dist_now           = 0;
            dist_last          = 0;
            angle_now          = 0;
            angle_last         = 0;
            kp_angle           = 1.15;
            kd_angle           = 0.045;
            error_angle        = 0.0;
            u_angle            = 0;
            speed_pid          = 0;
            kp_speed           = 2.4462;
            u_speed            = 0;
            count              = 0;
            count_pass         = 0;
            mid_speed          = 1035;
            vel_decreasing_factor = -15;
            dist_to_keep       = DIST_TO_KEEP;
            max_waiting_time   = MAX_WAIT_TIME;
            passing_enabled    = PASSING_ENABLED;
            stop_sign_detected = false;
            stop_initiated     = false;
            left_signal_on     = false;
            right_signal_on    = false;
            blinkers_on        = false;
            rear_lights_on     = false;   // luces traseras apagadas al inicio

            this->add_task(task);
        }

        // ── Light helpers ────────────────────────────────────────────────────

        void set_left_signal(bool state){
            if (left_signal_on == state) return;
            left_signal_on = state;
            std_msgs::Bool msg; msg.data = state;
            left_signal_pub.publish(msg);
            ROS_INFO(state ? "← Left signal ON" : "← Left signal OFF");
        }

        void set_right_signal(bool state){
            if (right_signal_on == state) return;
            right_signal_on = state;
            std_msgs::Bool msg; msg.data = state;
            right_signal_pub.publish(msg);
            ROS_INFO(state ? "→ Right signal ON" : "→ Right signal OFF");
        }

        void set_blinkers(bool state){
            if (blinkers_on == state) return;
            blinkers_on = state;
            std_msgs::Bool msg; msg.data = state;
            blinkers_pub.publish(msg);
            ROS_INFO(state ? "⚠ Blinkers ON" : "⚠ Blinkers OFF");
        }

        // Luces traseras de freno: se encienden si speed_pid == 0
        void update_rear_brake_lights(){
            bool braking = (speed_pid == 0);
            if (rear_lights_on == braking) return;
            rear_lights_on = braking;
            std_msgs::Bool msg; msg.data = braking;
            rear_lights_pub.publish(msg);
            ROS_INFO(braking ? "🔴 Brake lights ON" : "🔴 Brake lights OFF");
        }

        // ── Callbacks ────────────────────────────────────────────────────────

        void stop_sign_callback(const std_msgs::Bool::ConstPtr& msg){
            stop_sign_detected = msg->data;
            if (stop_sign_detected)
                ROS_INFO("STOP SIGN DETECTED!");
        }

        void dist_center_clbk(const std_msgs::Int16& dis_now_center){
            dist_now = static_cast<int>(dis_now_center.data);
            this->run();
            this->publish_policies();
        }

        void object_detec_clbk(const object_detection::points_objects::ConstPtr& msg){
            found_objects.clear();
            if (msg->points.empty()){
                num_found_objects = 0;
                return;
            }
            std::map<float, float> points_order;
            for (auto point : msg->points)
                points_order.insert(std::pair<float, float>(point.x, point.y));

            auto first_point = points_order.begin();
            geometry_msgs::Point point_msg;
            point_msg.x = first_point->first;
            point_msg.y = first_point->second;
            found_objects.push_back(point_msg);
            num_found_objects = found_objects.size();
        }

        // ── Task management ──────────────────────────────────────────────────

        void add_task(Task task){ this->task_pile.push_back(task); }

        void remove_task(){
            if (this->task_pile.size() > 1)
                this->task_pile.pop_back();
        }

        Task get_current_task(void){ return this->task_pile.back(); }

        void task_assigner(void){
            Task current_task = get_current_task();

            if (stop_sign_detected && current_task.ID == LANE_DRIVING){
                this->add_task(Task(STOP_AT_SIGN));
                stop_sign_detected = false;
                return;
            }

            if (this->num_found_objects > 0){
                for (auto obstacle : this->found_objects){
                    if ((obstacle.y > 70.0) && (obstacle.y < 110.0) && (obstacle.x < 119.0)){
                        if (current_task.ID == LANE_DRIVING){
                            this->add_task(Task(FOLLOWING));
                            last_task = new Task(FOLLOWING);
                            break;
                        }
                    }
                    if ((current_task.ID == FOLLOWING) && (this->count > this->max_waiting_time)){
                        count_pass += 1;
                        this->count = 0;
                        this->add_task(Task(MOVING_RIGHT_LANE));
                        this->add_task(Task(MOVING_RIGHT));
                        this->add_task(Task(PASSING));
                        this->add_task(Task(MOVING_LEFT));
                        break;
                    }
                }
            }
        }
        
        void task_solver(void){
            Task current_task = this->get_current_task();
            ROS_INFO_STREAM("[Current task]: " << current_task.name);

            // ── STOP AT SIGN ─────────────────────────────────────────────────
            if (current_task.ID == STOP_AT_SIGN){
                if (!stop_initiated){
                    stop_initiated  = true;
                    stop_start_time = std::chrono::steady_clock::now();
                    set_blinkers(true);
                    ROS_INFO("Stopping at STOP sign");
                }
                on_lane_stop();
                update_rear_brake_lights();   // enciende luces traseras al frenar

                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - stop_start_time).count();
                if (elapsed > 2000){
                    set_blinkers(false);
                    this->remove_task();
                    stop_initiated = false;
                    ROS_INFO("Resuming after STOP sign");
                }
                return;
            }

            // ── LANE DRIVING ─────────────────────────────────────────────────
            if (current_task.ID == LANE_DRIVING){
                set_left_signal(false);
                set_right_signal(false);
                set_blinkers(false);
                for (auto obstacle : this->found_objects){
                    if ((obstacle.y > 55.0 && obstacle.y < 125.0) && (obstacle.x <= 200.0)){
                        mid_speed = 535;
                        break;
                    }
                }
                on_lane();
            }

            // ── FOLLOWING ────────────────────────────────────────────────────
            else if (current_task.ID == FOLLOWING){
                if (count_pass > 4) count_pass = 0;
                if (this->num_found_objects == 0){
                    on_lane();
                    this->remove_task();
                }
                else{
                    for (auto obstacle : this->found_objects){
                        if ((obstacle.y >= 225.0 && obstacle.y <= 315.0) || last_task->ID == MOVING_RIGHT_LANE){
                            on_lane();
                            this->count = 0;
                            this->remove_task();
                            break;
                        }
                        else if (obstacle.x > (this->dist_to_keep + 10.0)){
                            this->count = 0;
                            if (count_pass == 2 || count_pass == 3)
                                on_lane_front_object_cross(obstacle.x);
                            else
                                on_lane_front_object(obstacle.x);
                            break;
                        }
                        else if (obstacle.x < (this->dist_to_keep - 10)){
                            if (this->passing_enabled)
                                this->count += 1;
                            if (count_pass == 2 || count_pass == 3)
                                on_lane_front_object_cross(obstacle.x);
                            else
                                on_lane_front_object(obstacle.x);
                            break;
                        }
                        else{
                            if (this->passing_enabled)
                                this->count += 1;
                            on_lane_stop();
                            break;
                        }
                    }
                }
            }

            // ── MOVING LEFT ──────────────────────────────────────────────────
            else if (current_task.ID == MOVING_LEFT){
                set_left_signal(true);
                set_right_signal(false);
                for (auto obstacle : this->found_objects){
                    if ((obstacle.y >= 60.0) && (obstacle.y <= 135.0)){
                        if (count_pass == 4){ speed_pid = -250; angle_pd = 170; }
                        else               { speed_pid = -200; angle_pd = 180; }
                        break;
                    }
                    else if ((obstacle.y >= 0.0 && obstacle.y < 60.0) || (obstacle.y >= 315.0 && obstacle.y <= 360.0)){
                        set_left_signal(false);
                        on_lane();
                        this->remove_task();
                        break;
                    }
                }
            }

            // ── PASSING ──────────────────────────────────────────────────────
            else if (current_task.ID == PASSING){
                set_left_signal(true);
                set_right_signal(false);
                for (auto obstacle : this->found_objects){
                    if ((obstacle.y >= 270.0) && (obstacle.y <= 345.0)){
                        speed_pid = -300; angle_pd = 20;
                        this->remove_task();
                        start = std::chrono::steady_clock::now();
                        break;
                    }
                    else{
                        if (count_pass > 2){ speed_pid = -200; angle_pd = 65; }
                        else              { on_lane(); }
                    }
                }
            }

            // ── MOVING RIGHT ─────────────────────────────────────────────────
            else if (current_task.ID == MOVING_RIGHT){
                set_right_signal(true);
                set_left_signal(false);
                time_long = 2500;
                if      (count_pass == 4) time_long = 3500;
                else if (count_pass == 2) time_long = 1700;
                else if (count_pass == 3) time_long = 1850;
                end = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() > time_long){
                    set_right_signal(false);
                    on_lane_right();
                    this->remove_task();
                    start = std::chrono::steady_clock::now();
                }
                else{ speed_pid = -200; angle_pd = 35; }
            }

            // ── MOVING RIGHT LANE ────────────────────────────────────────────
            else if (current_task.ID == MOVING_RIGHT_LANE){
                set_right_signal(true);
                set_left_signal(false);
                end = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() > 7500){
                    set_right_signal(false);
                    on_lane_right();
                    this->remove_task();
                    mid_speed = 1035;
                }
                else{ on_lane_right(); }
            }

            last_task = new Task(current_task.ID);

            // ── Actualizar luces de freno según velocidad resultante ──────────
            update_rear_brake_lights();
        }

        void run(void){
            this->task_assigner();
            this->task_solver();
        }

        // ── Motion helpers ───────────────────────────────────────────────────

        void on_lane(void){
            u_angle  = static_cast<int>(kp_angle * static_cast<float>(dist_now) + kd_angle * static_cast<float>(dist_now - dist_last));
            angle_pd = 90 + u_angle;
            if (angle_pd <= 45)  angle_pd = 45;
            else if (angle_pd >= 135) angle_pd = 135;
            u_speed  = static_cast<int>(kp_speed * dist_now);
            speed_pid = -mid_speed + abs(u_speed);
            if (speed_pid < -mid_speed) speed_pid = -mid_speed;
            else if (speed_pid > 0)     speed_pid = 0;
            dist_last = dist_now;
        }

        void on_lane_right(void){
            u_angle  = static_cast<int>(kp_angle * static_cast<float>(dist_now) + kd_angle * static_cast<float>(dist_now - dist_last));
            angle_pd = 90 + u_angle;
            if (angle_pd <= 0)   angle_pd = 0;
            else if (angle_pd >= 180) angle_pd = 180;
            speed_pid = -150;
            dist_last = dist_now;
        }

        void on_lane_front_object_cross(float obstacle_dist){
            speed_pid = decrement_speed(obstacle_dist);
            if (speed_pid < -535)      speed_pid = -535;
            else if (speed_pid >= 250) speed_pid = 250;
            angle_pd = 90;
        }

        void on_lane_front_object(float obstacle_dist){
            u_angle  = static_cast<int>(kp_angle * static_cast<float>(dist_now) + kd_angle * static_cast<float>(dist_now - dist_last));
            angle_pd = 90 + u_angle;
            if (angle_pd <= 45)  angle_pd = 45;
            else if (angle_pd >= 135) angle_pd = 135;
            speed_pid = decrement_speed(obstacle_dist);
            if (speed_pid < -535)      speed_pid = -535;
            else if (speed_pid >= 250) speed_pid = 250;
            dist_last = dist_now;
        }

        int decrement_speed(float obstacle_dist){
            int dist_to_obstacle = static_cast<int>(obstacle_dist);
            int distance_diff    = dist_to_obstacle - dist_to_keep;
            return vel_decreasing_factor * distance_diff;
        }

        void on_lane_stop(void){
            u_angle  = static_cast<int>(kp_angle * static_cast<float>(dist_now) + kd_angle * static_cast<float>(dist_now - dist_last));
            angle_pd = 90 + u_angle;
            if (angle_pd <= 45)  angle_pd = 45;
            else if (angle_pd >= 135) angle_pd = 135;
            speed_pid = 0;
            dist_last = dist_now;
        }

        void publish_policies(){
            angle_message.data = angle_pd;
            angle_pub.publish(angle_message);
            speed_message.data = speed_pid;
            speed_pub.publish(speed_message);
        }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "Master");
    Master *control = new Master(Task(LANE_DRIVING), true, 5, 115);
    ros::spin();
    return 0;
}
