/*
 * The MIT License (MIT)
 * Copyright (c) 2011 William Woodall <wjwwood@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>

#include <sys/fcntl.h>
#include <iomanip>
#include "serialPathConfig.h" // SERIAL_PATH を定義

// #include "ros/ros.h"
// #include <tf/transform_broadcaster.h>
// #include "geometry_msgs/Twist.h"
// #include "nav_msgs/Odometry.h"
// #include "segway_rmp/SegwayStatusStamped.h"

#include "segwayrmp/segwayrmp.h"

#include <vector>
#include <linux/joystick.h>
#define JOY_DEV "/dev/input/js0"


// #include <boost/thread.hpp>

// #include <std_msgs/String.h>
// #include <std_msgs/Float64.h>
// #include <sensor_msgs/Joy.h>
//
// #include "segway_rmp/VelocityStatus.h"
// #include "segway_rmp/AccelCmd.h"
// #include "segway_rmp/jyja.h"

class SegwayRMPNode;

static SegwayRMPNode * segwayrmp_node_instance;

static double radians_to_degrees = 180.0 / M_PI;
static double degrees_to_radians = M_PI / 180.0;

// Message Wrappers
void handleDebugMessages(const std::string &msg) {printf("%s",msg.c_str());}
void handleInfoMessages(const std::string &msg) {printf("%s",msg.c_str());}
void handleErrorMessages(const std::string &msg) {printf("%s",msg.c_str());}

// void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr &ss);
void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss); //  removed '&' by Ojima

const double dt = 1.0/20.0;

class Lavel {
public:
    double linear_vel, angular_vel;
};


// class BanAccel {
//     int* latch;
//     int section, reverse;
//     double t, ct, total_time, T1, T2, T3;
//     double vel, a;
//     double vel_limit;
//     double x;
//     ros::Publisher vel_pub;
// public:
//     BanAccel(ros::Publisher& vel_pub, int* latch): vel_pub(vel_pub), latch(latch) {}
//     void setup(double T2, double a, double vel_limit, int reverse) {
//         // this->x = x;
//         // this->total_time = 2.0*std::sqrt(x/a);
//         // this->vel_limit = std::sqrt(x*a);
//         // this->vel = 0;
//         // this->a = a;
//         // this->t = 0;
//         // this->ct = 0;
//         // this->section = 1;
//         this->reverse = reverse;
//         this->vel_limit = vel_limit;
//         this->a = a;
//         this->T1 = vel_limit / a;
//         this->T3 = vel_limit / a;
//         this->T2 = T2;
//         this->total_time = this->T1 + this->T2 + this->T3;
//         this->x =  (this->T2 + (this->T1 + this->T2 + this->T3)) * vel_limit / 2;
//         this->vel = 0;
//         this->t = 0;
//         this->ct = 0;
//         this->section = 1;
//     }
//     Lavel controller() {
//         switch (this->section) {
//             case 1:
//                 section_1();
//                 break;
//             case 2:
//                 section_2();
//                 break;
//             case 3:
//                 section_3();
//                 break;
//         }
//         segway_rmp::VelocityStatus vs;
//         vs.ros_time = ros::Time::now().toSec();
//         vs.section = this->section;
//         vs.t = this->ct;
//         vs.total_time = this->total_time;
//         vs.velocity = this->vel;
//         vs.vm = this->vel_limit;
//         vs.am = this->a;
//         vs.x = this->x;
//         vs.T1 = this->T1;
//         vs.T2 = this->T2;
//         vs.T3 = this->T3;
//         vel_pub.publish(vs);
//         Lavel la;
//         la.linear_vel = this->vel;
//         la.angular_vel = 0;
//         if (reverse) {
//             la.linear_vel = -la.linear_vel;
//         }
//         return la;
//     }
//     void section_1() {
//         if (this->t < this->T1) {
//             this->vel = this->a * this->t;
//             this->t += dt;
//             this->ct += dt;
//         }
//         else {
//             this->t = 0;
//             this->section = 2;
//             return section_2();
//         }
//         return;
//     }
//     void section_2() {
//         if (this->t < this->T2) {
//             this->vel = this->vel_limit;
//             this->t += dt;
//             this->ct += dt;
//         }
//         else {
//             this->t = 0;
//             this->section = 3;
//             return section_3();
//         }
//     }
//     void section_3() {
//         if (this->t < this->T3) {
//             this->vel = this->vel_limit - this->a * this->t;
//             this->t += dt;
//             this->ct += dt;
//         }
//         else {
//             this->vel = 0;
//             *(this->latch) = 0;
//             return;
//         }
//         return;
//     }
// };

// ROS Node class
class SegwayRMPNode {
public:
    SegwayRMPNode() {
        // n = new ros::NodeHandle("~");
        this->segway_rmp = NULL;
        this->first_odometry = true;
        this->last_forward_displacement = 0.0;
        this->last_yaw_displacement = 0.0;
        this->odometry_x = 0.0;
        this->odometry_y = 0.0;
        this->odometry_w = 0.0;
        this->linear_vel = 0.0;
        this->angular_vel = 0.0;
        this->target_linear_vel = 0.0;
        this->target_angular_vel = 0.0;
        this->initial_integrated_forward_position = 0.0;
        this->initial_integrated_left_wheel_position = 0.0;
        this->initial_integrated_right_wheel_position = 0.0;
        this->initial_integrated_turn_position = 0.0;
        this->latch = 3;
        this->lin = 0;
        this->ang = 0;
        this->before_target_linear_vel = 0;
        this->linear_vel_feedback = 0;
        this->zero_judge = 0;
        this->gain = 1.4;
        this->obstacle_detected = false;
        this->motors_enabled = false;
        this->recover_motors_enabled = false;
        this->reset_odometry = false;
        this->joy_control = false;
    }

    ~SegwayRMPNode() {
        this->disconnect();
    }

    void disconnect() {
        if (this->segway_rmp != NULL)
            delete this->segway_rmp;
        this->segway_rmp = NULL;
    }

    void momo_serial_read() {
        int fd_read = open(SERIAL_PATH, O_RDONLY); // SERIAL_PATH は serialPathConfig.h.in にて定義されている。
        while (true) {
            // if (this->connected) {
                int req_size = 100*sizeof(uint8_t);
                uint8_t buf_ptr[100] = {
                    0
                };
                int read_size = read(fd_read, buf_ptr, req_size);
                if (read_size < 0) {
                    continue;
                }
                printf("read %d byte: %02x %02x %02x %02x\n", read_size, buf_ptr[0], buf_ptr[1], buf_ptr[2], buf_ptr[3]);
                // printf("read %d byte: %08x\n", read_size, buf_ptr[0]);
                if (read_size == 4) {
                    if (buf_ptr[0] == 0x43 && buf_ptr[1] == 0x00) {
                        // this->ang = 50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                        // this->lin = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                        this->ang = 50*(int8_t)buf_ptr[2] /127.0;
                        this->lin = 1.0*(int8_t)buf_ptr[3] /127.0;
                        this->latch = 3;
                        this->jyja_arrival_time = std::chrono::system_clock::now();
                    }
                    else if ((buf_ptr[0] & 0xff000000) == 0xab000000) {
                        // segway_rmp::AccelCmd msg;
                        // msg.T2 = 20*(int8_t)((buf_ptr[0] & 0x00ff0000) >> 12)/127.0;
                        // msg.a = 0.5*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                        // msg.vel_limit = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                        // msg.reverse = 1;
                        // accel_pub.publish(msg);
                    }
                    else if ((buf_ptr[0] & 0xff000000) == 0xaf000000) {
                        // segway_rmp::AccelCmd msg;
                        // msg.T2 = 20*(int8_t)((buf_ptr[0] & 0x00ff0000) >> 12)/127.0;
                        // msg.a = 0.5*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                        // msg.vel_limit = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                        // msg.reverse = 0;
                        // accel_pub.publish(msg);
                    }
                    else if (buf_ptr[0] == 0x99999999) {
                        // std::cout << "segway_rmp_node を終了\n";
                        // std_msgs::String msg;
                        // msg.data = "quit";
                        // halt_pub.publish(msg);
                    }
                }
            // }
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return;
    }

    void joy_read() {
        int joy_fd = -1;

        while (true) {
            if (joy_fd < 0) {
                printf("[%d] connecting to joystick ...\n", joy_fd);
                joy_fd = open(JOY_DEV, O_RDONLY);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            else {
                break;
            }
        }

        int num_of_axis = 0, num_of_buttons = 0;
        char name_of_joystick[80];

        ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
        ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
        ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

        std::vector<char> joy_button;
        std::vector<double> joy_axis;

        joy_button.resize(num_of_buttons,0);
        joy_axis.resize(num_of_axis,0);
        std::cout << "Joystick:  " << name_of_joystick << '\n';
        std::cout << "  axis: " << num_of_axis << '\n';
        std::cout << "  buttons: " << num_of_buttons << '\n';

        while (true) {
            js_event js;
            read(joy_fd, &js, sizeof(js_event));

            switch (js.type & ~JS_EVENT_INIT) {
                case JS_EVENT_AXIS:
                    if((int)js.number >= joy_axis.size()) {
                        std::cout << "err:" << (int)js.number << '\n';
                        continue;
                    }
                    joy_axis.at((int)js.number) = js.value;
                    break;
                case JS_EVENT_BUTTON:
                    if ((int)js.number >= joy_button.size()) {
                        std::cout << "err:" << (int)js.number << '\n';
                        continue;
                    }
                    joy_button.at((int)js.number) = js.value;
                    break;
            }

            // std::cout << "axis:";
            // for (size_t i(0); i < joy_axis.size(); i++) {
            //     std::cout << " " << std::setw(2) << joy_axis.at(i)/32767.0;
            // }
            // std::cout << '\n';
            //
            // std::cout << "  button: ";
            // for(size_t i(0);i<joy_button.size();++i) {
            //     std::cout << " " << (int)joy_button.at(i);
            // }
            // std::cout << '\n';


            if ((int)joy_button.at(13) == 1) {
                this->latch = 0;
            }
            if ((int)joy_button.at(14) == 1) {
                this->latch = 3;
                this->ang = 0;
                this->lin = 0;
            }

            if (this->latch == 0) {
                this->ang = -50.0*joy_axis.at(0)/32767.0;
                this->lin = -joy_axis.at(3)/32767.0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return;
    }

    void run() {
        if (this->getParameters()) {
            return;
        }

        this->setupSegwayRMP();

        // this->setupROSComms();

        // this->cv = new ChangeVelocity(this->vel_pub, &latch, this->max_linear_vel, this->linear_pos_accel_limit, this->max_angular_vel, this->angular_pos_accel_limit);
        // this->ba = new BanAccel(this->vel_pub, &(this->latch));

        // Setup keep alive timer
        // this->keep_alive_timer = this->n->createTimer(ros::Duration(dt), &SegwayRMPNode::keepAliveCallback, this);
        //
        // ros::AsyncSpinner spinner(1);
        // spinner.start();

        // this->createdTimer = false;

        // this->keep_alive_timer = this->n->createTimer(ros::Duration(dt), &SegwayRMPNode::keepAliveCallback, this);
        // ros::AsyncSpinner spinner(1);
        // spinner.start();
        // this->createdTimer = true;

        // this->odometry_reset_start_time = ros::Time::now();

        boost::thread th_momo_serial_read(&SegwayRMPNode::momo_serial_read, this);
        // boost::thread th_keep_alive_callback(&SegwayRMPNode::keep_alive_callback, this);
        boost::thread th_hoge(&SegwayRMPNode::joy_read, this);

        this->reset_odometry = false;
        this->connected = false;
        while (true) {
            try {
                this->segway_rmp->connect(true);
                this->connected = true;
            } catch (std::exception& e) {
                std::string e_msg(e.what());
                printf("Exception while connecting to the SegwayRMP, check your cables and power buttons.\n");
                printf("    %s\n", e_msg.c_str());
                this->connected = false;
            }
            if (this->spin()) { // ROS is OK, but we aren't connected, wait then try again
                printf("Not connected to the SegwayRMP, will retry in 3 seconds...\n\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                // ros::Duration(3).sleep();
            }
        }
        return;
    }


    bool spin() {
        if (true && this->connected) {
            printf("Segway RMP Ready.\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            this->segway_rmp->resetAllIntegrators();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            // ros::Duration(0.05).sleep();
            this->segway_rmp->setMaxVelocityScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            // ros::Duration(0.05).sleep();
            this->segway_rmp->setMaxAccelerationScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            // ros::Duration(0.05).sleep();
            this->segway_rmp->setMaxTurnScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            // ros::Duration(0.05).sleep();
            this->segway_rmp->setCurrentLimitScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            // ros::Duration(0.05).sleep();
            this->segway_rmp->setBalanceModeLocking(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            // ros::Duration(0.05).sleep();
            this->segway_rmp->setOperationalMode(segwayrmp::tractor);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            // ros::Duration(0.05).sleep();
            this->segway_rmp->setControllerGainSchedule(segwayrmp::heavy);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            // ros::Duration(0.05).sleep();
            // this->keep_alive_timer = this->n->createTimer(ros::Duration(dt), &SegwayRMPNode::keepAliveCallback, this);
            // ros::AsyncSpinner spinner(1);
            // spinner.start();
            while (true && this->connected) {
                // ros::Duration(1).sleep();
                // while (ros::ok()) {
                //     if (this->latch) {
                //         ros::Duration(1).sleep();
                //     }
                //     else {
                //         break;
                //     }
                // }
                // std::cout << "前後に動かす場合は 1 を入力。曲がる場合は 2 を入力。バン加速は 3 を入力。シャットダウンは 4 >> ";
                // int num;
                // std::cin >> num;
                // double x, theta, r, a, max_vel;
                // switch (num) {
                //     case 1:
                //         std::cout << "変位 x (m) を入力 >> ";
                //         std::cin >> x;
                //         cv->setup(x);
                //         this->latch = 1;
                //         break;
                //     case 2:
                //         std::cout << "回転角度 theta (deg) と 回転半径 r (m) を入力\n回転角度 theta (deg) = ";
                //         std::cin >> theta;
                //         std::cout << "回転半径 r (m) = ";
                //         std::cin >> r;
                //         cv->setup(r, theta);
                //         this->latch = 1;
                //         break;
                //     case 3:
                //         std::cout << "最高速度 max_vel (m/s) と 加速度 a (m/s/s) と を入力\n";
                //         std::cout << "max_vel = ";
                //         std::cin >> max_vel;
                //         std::cout << "a = ";
                //         std::cin >> a;
                //         ba->setup(max_vel, a);
                //         this->latch = 2;
                //         break;
                //     case 4:
                //         std::exit(1);
                //         return false;
                // }
                // std::cout << "移動中・・・\n";
                // ros::Duration(100).sleep();

                // boost::mutex::scoped_lock lock(this->m_mutex);


                if (!this->connected || this->reset_odometry) {
                    continue;
                }

                // ROS_INFO("keepAliveCallback");

                if (!this->motors_enabled) {

                    continue;
                }

                if (this->recover_motors_enabled) {
                    this->segway_rmp->resetAllIntegrators();
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    // ros::Duration(0.05).sleep();
                    this->segway_rmp->setMaxVelocityScaleFactor(1.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    // ROS_INFO("setMaxVelocityScaleFactor");
                    // ros::Duration(0.05).sleep();
                    this->segway_rmp->setMaxAccelerationScaleFactor(1.0);
                    // ROS_INFO("setMaxAccelerationScaleFactor");
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    // ros::Duration(0.05).sleep();
                    this->segway_rmp->setMaxTurnScaleFactor(1.0);
                    // ROS_INFO("setMaxTurnScaleFactor");
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    // ros::Duration(0.05).sleep();
                    this->segway_rmp->setCurrentLimitScaleFactor(1.0);
                    // ROS_INFO("setCurrentLimitScaleFactor");
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    // ros::Duration(0.05).sleep();
                    this->segway_rmp->setBalanceModeLocking(true);
                    // ROS_INFO("setBalanceModeLocking");
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    // ros::Duration(0.05).sleep();
                    this->segway_rmp->setOperationalMode(segwayrmp::tractor);
                    // ROS_INFO("setOperationalMode");
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    // ros::Duration(0.05).sleep();
                    this->segway_rmp->setControllerGainSchedule(segwayrmp::heavy);
                    // ROS_INFO("setControllerGainSchedule");
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    // ros::Duration(0.05).sleep();
                    this->recover_motors_enabled = false;
                }

                // boost::mutex::scoped_lock lock(this->m_mutex);
                // ROS_INFO("keepAliveCallback");

                // printf("main loop\n");

                Lavel la;
                if (this->latch == 0) {
                    // if (ros::Time::now() - this->joy_arrival_time > ros::Duration(0.5)) {
                    //     this->lin = 0;
                    //     this->ang = 0;
                    // }
                }
                if (this->latch == 3) {
                    if (std::chrono::system_clock::now() - this->jyja_arrival_time > std::chrono::milliseconds(500)) {
                        this->lin = 0;
                        this->ang = 0;
                        // this->latch = 0;
                    }
                }
                else if (this->latch == 1) {
                    // la = this->cv->controller();
                    // this->lin = la.linear_vel;
                    // this->ang = la.angular_vel;
                }
                else if (this->latch == 2) {
                    // la = this->ba->controller();
                    // this->lin = la.linear_vel + 0.03;
                    this->lin = (this->lin - this->linear_vel_feedback)*0.5 + la.linear_vel;
                    this->ang = la.angular_vel;
                }

                try {
                    // if (this->lin > 1.5) {
                    //     this->lin = 1.5;
                    // }
                    // else if (this->lin < -0.5) {
                    //     // this->lin = -0.5;
                    // }

                    if (this->obstacle_detected) {
                        this->lin = 0;
                    }
                    // printf("%lf, %lf\n", this->ang, this->lin);
                    this->segway_rmp->move(this->lin, this->ang); // add offset 0.03
                } catch (std::exception& e) {
                    std::string e_msg(e.what());
                    printf("Error commanding Segway RMP: %s", e_msg.c_str());
                    this->connected = false;
                    this->disconnect();
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                // ros::Duration(0.05).sleep();
            }
            // ros::Duration(100).sleep();
        }

        if (true) { // Error not shutdown
            return true;
        } else {         // Shutdown
            return false;
        }
    }

    /**
     * This method is called at 20Hz.  Each time it sends a movement
     * command to the Segway RMP.
     */

    /*
    void keepAliveCallback(const ros::TimerEvent& e) {

        if (ros::ok()) {
            if (!this->connected || this->reset_odometry) {
                return;
            }

            ROS_INFO("keepAliveCallback");

            // else if (!this->segway_rmp->no_data_from_segway && this->no_data_from_segway) {
            //     this->segway_rmp->setMaxVelocityScaleFactor(1.0);
            //     ROS_INFO("setMaxVelocityScaleFactor");
            //     ros::Duration(0.05).sleep();
            //     this->segway_rmp->setMaxAccelerationScaleFactor(1.0);
            //     ROS_INFO("setMaxAccelerationScaleFactor");
            //     ros::Duration(0.05).sleep();
            //     this->segway_rmp->setMaxTurnScaleFactor(1.0);
            //     ROS_INFO("setMaxTurnScaleFactor");
            //     ros::Duration(0.05).sleep();
            //     this->segway_rmp->setCurrentLimitScaleFactor(1.0);
            //     ROS_INFO("setCurrentLimitScaleFactor");
            //     ros::Duration(0.05).sleep();
            //     this->segway_rmp->setBalanceModeLocking(true);
            //     ROS_INFO("setBalanceModeLocking");
            //     ros::Duration(0.05).sleep();
            //     this->segway_rmp->setOperationalMode(segwayrmp::tractor);
            //     ROS_INFO("setOperationalMode");
            //     ros::Duration(0.05).sleep();
            //     this->segway_rmp->setControllerGainSchedule(segwayrmp::heavy);
            //     ROS_INFO("setControllerGainSchedule");
            //     ros::Duration(0.05).sleep();
            //     this->no_data_from_segway = false;
            // }
            //
            // if (this->no_data_from_segway) {
            //     return;
            // }

            boost::mutex::scoped_lock lock(this->m_mutex);

            Lavel la;
            if (this->latch == 0) {
                if (this->joy_b4_arrival_time < this->joy_arrival_time) {
                    this->joy_b4_arrival_time = this->joy_arrival_time;
                    zero_judge = 0;
                }
                else {
                    zero_judge += 1;
                    if (zero_judge == 10) {
                        this->lin = 0;
                        this->ang = 0;
                        zero_judge = 0;
                    }
                }
            }
            if (this->latch == 3) {
                if (this->jyja_b4_arrival_time < this->jyja_arrival_time) {
                    this->jyja_b4_arrival_time = this->jyja_arrival_time;
                    zero_judge = 0;
                }
                else {
                    zero_judge += 1;
                    if (zero_judge == 20) {
                        this->lin = 0;
                        this->ang = 0;
                        zero_judge = 0;
                        this->latch = 0;
                    }
                }
            }
            else if (this->latch == 1) {
                la = this->cv->controller();
                this->lin = la.linear_vel;
                this->ang = la.angular_vel;
            }
            else if (this->latch == 2) {
                la = this->ba->controller();
                // this->lin = la.linear_vel + 0.03;
                this->lin = (this->lin - this->linear_vel_feedback)*0.5 + la.linear_vel;
                this->ang = la.angular_vel;
            }

            try {
                // if (this->lin > 1.5) {
                //     this->lin = 1.5;
                // }
                // else if (this->lin < -0.5) {
                //     // this->lin = -0.5;
                // }

                if (this->obstacle_detected) {
                    this->lin = 0;
                }
                this->segway_rmp->move(this->lin, this->ang); // add offset 0.03
            } catch (std::exception& e) {
                std::string e_msg(e.what());
                ROS_ERROR("Error commanding Segway RMP: %s", e_msg.c_str());
                this->connected = false;
                this->disconnect();
            }
        }
    }
    */

    void handleStatus(segwayrmp::SegwayStatus::Ptr &ss_ptr) {
        if (!this->connected)
            return;
        // // Get the time
        // ros::Time current_time = ros::Time::now();
        //
        // this->sss_msg.header.stamp = current_time;
        //
        segwayrmp::SegwayStatus &ss = *(ss_ptr);
        //
        // // Check if an odometry reset is still required
        // if (this->reset_odometry) {
        //   if ((current_time - this->odometry_reset_start_time).toSec() < 0.25) {
        //     return; // discard readings for the first 0.25 seconds
        //   }
        //   if (fabs(ss.integrated_forward_position) < 1e-3 &&
        //       fabs(ss.integrated_turn_position) < 1e-3 &&
        //       fabs(ss.integrated_left_wheel_position) < 1e-3 &&
        //       fabs(ss.integrated_right_wheel_position) < 1e-3) {
        //     this->initial_integrated_forward_position = ss.integrated_forward_position;
        //     this->initial_integrated_left_wheel_position = ss.integrated_left_wheel_position;
        //     this->initial_integrated_right_wheel_position = ss.integrated_right_wheel_position;
        //     this->initial_integrated_turn_position = ss.integrated_turn_position;
        //     ROS_INFO("Integrators reset by Segway RMP successfully");
        //     this->reset_odometry = false;
        //   } else if ((current_time - this->odometry_reset_start_time).toSec() > this->odometry_reset_duration) {
        //     this->initial_integrated_forward_position = ss.integrated_forward_position;
        //     this->initial_integrated_left_wheel_position = ss.integrated_left_wheel_position;
        //     this->initial_integrated_right_wheel_position = ss.integrated_right_wheel_position;
        //     this->initial_integrated_turn_position = ss.integrated_turn_position;
        //     ROS_INFO("Integrator reset by Segway RMP failed. Performing software reset");
        //     this->reset_odometry = false;
        //   } else {
        //     return; // continue waiting for odometry to be reset
        //   }
        // }
        //
        this->linear_vel_feedback = (ss.left_wheel_speed + ss.right_wheel_speed) / 2.0;
        // printf("%lf\n", this->linear_vel_feedback);
        //
        // this->sss_msg.segway.pitch_angle = ss.pitch * degrees_to_radians;
        // this->sss_msg.segway.pitch_rate = ss.pitch_rate * degrees_to_radians;
        // this->sss_msg.segway.roll_angle = ss.roll * degrees_to_radians;
        // this->sss_msg.segway.roll_rate = ss.roll_rate * degrees_to_radians;
        // this->sss_msg.segway.left_wheel_velocity = ss.left_wheel_speed;
        // this->sss_msg.segway.right_wheel_velocity = ss.right_wheel_speed;
        // this->sss_msg.segway.accel = (v2 - v1)/dt;
        // this->sss_msg.segway.yaw_rate = ss.yaw_rate * degrees_to_radians;
        // this->sss_msg.segway.servo_frames = ss.servo_frames;
        // this->sss_msg.segway.left_wheel_displacement =
        //     ss.integrated_left_wheel_position - this->initial_integrated_left_wheel_position;
        // this->sss_msg.segway.right_wheel_displacement =
        //     ss.integrated_right_wheel_position - this->initial_integrated_right_wheel_position;
        // this->sss_msg.segway.forward_displacement =
        //     ss.integrated_forward_position - this->initial_integrated_forward_position;
        // this->sss_msg.segway.yaw_displacement =
        //     (ss.integrated_turn_position - this->initial_integrated_turn_position) * degrees_to_radians;
        // this->sss_msg.segway.left_motor_torque = ss.left_motor_torque;
        // this->sss_msg.segway.right_motor_torque = ss.right_motor_torque;
        // this->sss_msg.segway.operation_mode = ss.operational_mode;
        // this->sss_msg.segway.gain_schedule = ss.controller_gain_schedule;
        // this->sss_msg.segway.ui_battery = ss.ui_battery_voltage;
        // this->sss_msg.segway.powerbase_battery = ss.powerbase_battery_voltage;
        // this->sss_msg.segway.motors_enabled = (bool)(ss.motor_status);
        // this->sss_msg.segway.ros_time = ros::Time::now().toSec();
        // this->sss_msg.segway.send_vel = this->lin;
        // this->sss_msg.segway.target_vel = this->before_target_linear_vel;
        // this->sss_msg.segway.actual_velocity = (ss.left_wheel_speed + ss.right_wheel_speed)*0.5;
        //
        //
        if (!this->motors_enabled && (bool)(ss.motor_status)) {
            this->recover_motors_enabled = true;
        }
        this->motors_enabled = (bool)(ss.motor_status);
        //
        // segway_status_pub.publish(this->sss_msg);
        //
        // // TODO: possibly spin this off in another thread
        //
        // // Grab the newest Segway data
        // float forward_displacement =
        //     (ss.integrated_forward_position - this->initial_integrated_forward_position) *
        //     this->linear_odom_scale;
        // float yaw_displacement =
        //     (ss.integrated_turn_position - this->initial_integrated_turn_position) *
        //     degrees_to_radians * this->angular_odom_scale;
        // float yaw_rate = ss.yaw_rate * degrees_to_radians;
        //
        // // Integrate the displacements over time
        // // If not the first odometry calculate the delta in displacements
        // float vel_x = 0.0;
        // float vel_y = 0.0;
        // if(!this->first_odometry) {
        //     float delta_forward_displacement =
        //         forward_displacement - this->last_forward_displacement;
        //     double delta_time = (current_time-this->last_time).toSec();
        //     // Update accumulated odometries and calculate the x and y components
        //     // of velocity
        //     this->odometry_w = yaw_displacement;
        //     float delta_odometry_x =
        //         delta_forward_displacement * std::cos(this->odometry_w);
        //     vel_x = delta_odometry_x / delta_time;
        //     this->odometry_x += delta_odometry_x;
        //     float delta_odometry_y =
        //         delta_forward_displacement * std::sin(this->odometry_w);
        //     vel_y = delta_odometry_y / delta_time;
        //     this->odometry_y += delta_odometry_y;
        // } else {
        //     this->first_odometry = false;
        // }
        // // No matter what update the previouse (last) displacements
        // this->last_forward_displacement = forward_displacement;
        // this->last_yaw_displacement = yaw_displacement;
        // this->last_time = current_time;
        //
        // // Create a Quaternion from the yaw displacement
        // geometry_msgs::Quaternion quat =
        //     tf::createQuaternionMsgFromYaw(yaw_displacement);
        //
        // // Publish the Transform odom->base_link
        // if (this->broadcast_tf) {
        //     this->odom_trans.header.stamp = current_time;
        //
        //     this->odom_trans.transform.translation.x = this->odometry_x;
        //     this->odom_trans.transform.translation.y = this->odometry_y;
        //     this->odom_trans.transform.translation.z = 0.0;
        //     this->odom_trans.transform.rotation = quat;
        //
        //     //send the transform
        //     this->odom_broadcaster.sendTransform(this->odom_trans);
        // }
        //
        // // Publish Odometry
        // this->odom_msg.header.stamp = current_time;
        // this->odom_msg.pose.pose.position.x = this->odometry_x;
        // this->odom_msg.pose.pose.position.y = this->odometry_y;
        // this->odom_msg.pose.pose.position.z = 0.0;
        // this->odom_msg.pose.pose.orientation = quat;
        // this->odom_msg.pose.covariance[0] = 0.00001;
        // this->odom_msg.pose.covariance[7] = 0.00001;
        // this->odom_msg.pose.covariance[14] = 1000000000000.0;
        // this->odom_msg.pose.covariance[21] = 1000000000000.0;
        // this->odom_msg.pose.covariance[28] = 1000000000000.0;
        // this->odom_msg.pose.covariance[35] = 0.001;
        //
        // this->odom_msg.twist.twist.linear.x = vel_x;
        // this->odom_msg.twist.twist.linear.y = vel_y;
        // this->odom_msg.twist.twist.angular.z = yaw_rate;
        //
        // this->odom_pub.publish(this->odom_msg);
    }

    /**
     * This method is called if a motor command is not received
     * within the segway_motor_timeout interval.  It halts the
     * robot for safety reasons.
     */
    // void motor_timeoutCallback(const ros::TimerEvent& e) {
    //     boost::mutex::scoped_lock lock(m_mutex);
    //     // ROS_INFO("Motor command timeout!  Setting target linear and angular velocities to be zero.");
    //     // this->target_linear_vel = 0.0;
    //     // this->target_angular_vel = 0.0;
    // }

    /**
     * The handler for messages received on the 'cmd_vel' topic.
     */
    // void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //     if (!this->connected)
    //         return;
    //     boost::mutex::scoped_lock lock(m_mutex);
    //     double x = msg->linear.x;
    //     double z = msg->angular.z;
    //     if (this->invert_x) {
    //         x *= -1;
    //     }
    //     if (this->invert_z) {
    //         z *= -1;
    //     }
    //     if (this->max_linear_vel != 0.0) {
    //       if (abs(x) > this->max_linear_vel) {
    //         x = (x > 0) ? this->max_linear_vel : -this->max_linear_vel;
    //       }
    //     }
    //     if (this->max_angular_vel != 0.0) {
    //       if (abs(z) > this->max_angular_vel) {
    //         z = (z > 0) ? this->max_angular_vel : -this->max_angular_vel;
    //       }
    //     }
    //     this->target_linear_vel = x;
    //     this->target_angular_vel = z * radians_to_degrees; // Convert to degrees
    //
    //     // ROS_INFO("Received motor command linear vel = %f, angular vel = %f.",
    //     //    this->target_linear_vel, this->target_angular_vel);
    //
    //     this->motor_timeout_timer =
    //         this->n->createTimer(
    //             ros::Duration(this->segway_motor_timeout),
    //             &SegwayRMPNode::motor_timeoutCallback,
    //             this,
    //             true);
    // }

    // void cmd_accelCallback(const segway_rmp::AccelCmd& msg) {
    //     if (!this->connected) {
    //         return;
    //     }
    //     std::cout << "移動中・・・";
    //     boost::mutex::scoped_lock lock(m_mutex);
    //     if (this->latch == 1 || this->latch == 2) {
    //         return;
    //     }
    //     ba->setup(msg.T2, msg.a, msg.vel_limit, msg.reverse);
    //     this->latch = 2;
    //     return;
    // }

    // void halt_callback(const std_msgs::String& msg) {
    //     this->segway_rmp->shutdown();
    //     ros::Duration(0.3).sleep();
    //     exit(0);
    // }

    // void joy_callback(const sensor_msgs::Joy& msg) {
    //     if (this->connected && (this->latch == 0 || this->latch == 3)) {
    //         this->latch = 0;
    //         // ROS_INFO("%lf, %lf", msg.axes[0], msg.axes[3]);
    //         this->lin = msg.axes[3] * 0.8;
    //         this->ang = msg.axes[0] * 60.0;
    //         this->joy_arrival_time = ros::Time::now();
    //         if (this->lin < 0) {
    //             this->ang = - this->ang;
    //         }
    //         // ros::Duration(0.05).sleep();
    //     }
    //     return;
    // }

    // void jyja_callback(const segway_rmp::jyja& msg) {
    //     if (this->connected && (this->latch == 0 || this->latch == 3)) {
    //         // ROS_INFO("%lf, %lf", msg.leftright, msg.frontrear);
    //         // this->ang = msg.leftright;
    //         // this->lin = msg.frontrear;
    //         // if (this->lin < 0) {
    //         //     this->ang = - this->ang;
    //         // }
    //         this->latch = 3;
    //         this->jyja_arrival_time = ros::Time::now();
    //         if (msg.frontrear >= 0) {
    //             this->lin = (this->before_target_linear_vel - this->linear_vel_feedback)*0.6 + msg.frontrear;
    //             // this->lin = msg.frontrear;
    //             this->ang = msg.leftright;
    //         }
    //         else {
    //             this->lin = (this->before_target_linear_vel - this->linear_vel_feedback)*0.6 + msg.frontrear;
    //             // this->lin = msg.frontrear;
    //             this->ang = -msg.leftright;
    //         }
    //         this->before_target_linear_vel = msg.frontrear;
    //         // ros::Duration(0.05).sleep();
    //     }
    //     return;
    // }

    // void obstacle_callback(const std_msgs::String& msg) {
    //     if (msg.data == "obstacle") {
    //         this->obstacle_detected = true;
    //     }
    //     else {
    //         this->obstacle_detected = false;
    //     }
    // }

private:
    // Function
    // void setupROSComms() {
    //     // Subscribe to command velocities
    //     this->cmd_velSubscriber = n->subscribe("cmd_vel", 1000, &SegwayRMPNode::cmd_velCallback, this);
    //     this->cmd_accelSubscriber = n->subscribe("/accel_cmd/accel", 1000, &SegwayRMPNode::cmd_accelCallback, this);
    //     this->halt_sub = n->subscribe("/accel_cmd/halt", 1000, &SegwayRMPNode::halt_callback, this);
    //     this->joy_sub = n->subscribe("/joy", 100, &SegwayRMPNode::joy_callback, this);
    //     this->jyja_sub = n->subscribe("/accel_cmd/jyja", 100, &SegwayRMPNode::jyja_callback, this);
    //     this->obstacle_sub = n->subscribe("/obstacle", 10, &SegwayRMPNode::obstacle_callback, this);
    //
    //     // Advertise the SegwayStatusStamped
    //     this->segway_status_pub = n->advertise<segway_rmp::SegwayStatusStamped>("segway_status", 1000);
    //     // Advertise the Odometry Msg
    //     this->odom_pub = n->advertise<nav_msgs::Odometry>("odom", 50);
    //
    //     this->vel_pub = n->advertise<segway_rmp::VelocityStatus>("vel", 1000);
    // }

    void setupSegwayRMP() {
        std::stringstream ss;
        ss << "Connecting to Segway RMP via ";
        this->segway_rmp = new segwayrmp::SegwayRMP(this->interface_type, this->segway_rmp_type);
        if (this->interface_type_str == "serial") {
            ss << "serial on serial port: " << this->serial_port;
            this->segway_rmp->configureSerial(this->serial_port);

        } else if (this->interface_type_str == "usb") {
            ss << "usb ";
            if (this->usb_selector == "serial_number") {
                ss << "identified by the device serial number: " << this->serial_number;
                this->segway_rmp->configureUSBBySerial(this->serial_number);
            }
            if (this->usb_selector == "description") {
                ss << "identified by the device description: " << this->usb_description;
                this->segway_rmp->configureUSBByDescription(this->usb_description);
            }
            if (this->usb_selector == "index") {
                ss << "identified by the device index: " << this->usb_index;
                this->segway_rmp->configureUSBByIndex(this->usb_index);
            }
        }
        printf("%s\n", ss.str().c_str());


        // Set the instance variable
        segwayrmp_node_instance = this;

        // Set callbacks for segway data and messages
        this->segway_rmp->setStatusCallback(handleStatusWrapper);
        this->segway_rmp->setLogMsgCallback("debug", handleDebugMessages);
        this->segway_rmp->setLogMsgCallback("info", handleInfoMessages);
        this->segway_rmp->setLogMsgCallback("error", handleErrorMessages);
    }

    int getParameters() {
        // Get Interface Type
        // n->param("interface_type", this->interface_type_str, std::string("serial"));
        this->interface_type_str = std::string("serial");
        // Get Configurations based on Interface Type
        if (this->interface_type_str == "serial") {
            this->interface_type = segwayrmp::serial;
            // n->param("serial_port", this->serial_port, std::string("/dev/ttyUSB0"));
            this->serial_port = std::string("/dev/ttyUSB0");
        } else if (this->interface_type_str == "usb") {
            // this->interface_type = segwayrmp::usb;
            // n->param("usb_selector", this->usb_selector, std::string("serial_number"));
            // if (this->usb_selector == "index") {
            //     n->param("usb_index", this->usb_index, 0);
            // } else if (this->usb_selector == "serial_number") {
            //     n->param("usb_serial_number", this->serial_number, std::string("0403e729"));
            //     if (this->serial_number == std::string("00000000")) {
            //         ROS_WARN("The serial_number parameter is set to the default 00000000, which shouldn't work.");
            //     }
            // } else if (this->usb_selector == "description") {
            //     n->param("usb_description", this->serial_number, std::string("Robotic Mobile Platform"));
            // } else {
            //     ROS_ERROR(
            //         "Invalid USB selector: %s, valid types are 'index', 'serial_number', and 'description'.",
            //         this->usb_selector.c_str());
            //     return 1;
            // }
        } else {
            // ROS_ERROR(
            //     "Invalid interface type: %s, valid interface types are 'serial' and 'usb'.",
            //     this->interface_type_str.c_str());
            return 1;
        }
        // Get Setup Motor Timeout
        // n->param("motor_timeout", this->segway_motor_timeout, 0.5);
        // Get frame id parameter
        // n->param("frame_id", frame_id, std::string("base_link"));
        this->frame_id = std::string("base_link");
        // n->param("odom_frame_id", odom_frame_id, std::string("odom"));
        this->odom_frame_id = std::string("odom");
        // this->sss_msg.header.frame_id = this->frame_id;
        // this->odom_trans.header.frame_id = this->odom_frame_id;
        // this->odom_trans.child_frame_id = this->frame_id;
        // this->odom_msg.header.frame_id = this->odom_frame_id;
        // this->odom_msg.child_frame_id = this->frame_id;
        // // Get cmd_vel inversion parameters
        // n->param("invert_linear_vel_cmds", invert_x, false);
        // n->param("invert_angular_vel_cmds", invert_z, false);
        // // Get option for enable/disable tf broadcasting
        // n->param("broadcast_tf", this->broadcast_tf, true);
        // Get the segway rmp type
        std::string segway_rmp_type_str;
        // n->param("rmp_type", segway_rmp_type_str, std::string("200/400"));
        segway_rmp_type_str = std::string("200/400");
        if (segway_rmp_type_str == "200/400") {
            this->segway_rmp_type = segwayrmp::rmp200;
        } else if (segway_rmp_type_str == "50/100") {
            this->segway_rmp_type = segwayrmp::rmp100;
        } else {
            // ROS_ERROR(
            //     "Invalid rmp type: %s, valid rmp types are '200/400' and '50/100'.",
            //     segway_rmp_type_str.c_str());
            return 1;
        }

        // Get the linear acceleration limits in m/s^2.  Zero means infinite.
        // n->param("linear_pos_accel_limit", this->linear_pos_accel_limit, 0.1);
        // n->param("linear_neg_accel_limit", this->linear_neg_accel_limit, 0.0);
        //
        // // Get the angular acceleration limits in deg/s^2.  Zero means infinite.
        // n->param("angular_pos_accel_limit", this->angular_pos_accel_limit, 10.0);
        // n->param("angular_neg_accel_limit", this->angular_neg_accel_limit, 0.0);
        //
        // // Check for valid acceleration limits
        // if (this->linear_pos_accel_limit < 0) {
        //     ROS_ERROR("Invalid linear positive acceleration limit of %f (must be non-negative).",
        //         this->linear_pos_accel_limit);
        //     return 1;
        // }
        // if (this->linear_neg_accel_limit < 0) {
        //     ROS_ERROR("Invalid linear negative acceleration limit of %f (must be non-negative).",
        //         this->linear_neg_accel_limit);
        //     return 1;
        // }
        // if (this->angular_pos_accel_limit < 0) {
        //     ROS_ERROR("Invalid angular positive acceleration limit of %f (must be non-negative).",
        //         this->angular_pos_accel_limit);
        //     return 1;
        // }
        // if (this->angular_neg_accel_limit < 0) {
        //     ROS_ERROR("Invalid angular negative acceleration limit of %f (must be non-negative).",
        //         this->angular_neg_accel_limit);
        //     return 1;
        // }
        //
        // ROS_INFO("Accel limits: linear: pos = %f, neg = %f, angular: pos = %f, neg = %f.",
        //     this->linear_pos_accel_limit, this->linear_neg_accel_limit,
        //     this->angular_pos_accel_limit, this->angular_neg_accel_limit);
        //
        // // Get velocity limits. Zero means no limit
        // n->param("max_linear_vel", this->max_linear_vel, 0.3);
        // n->param("max_angular_vel", this->max_angular_vel, 9.0);
        //
        // if (this->max_linear_vel < 0) {
        //     ROS_ERROR("Invalid max linear velocity limit of %f (must be non-negative).",
        //         this->max_linear_vel);
        //     return 1;
        // }
        //
        // if (this->max_angular_vel < 0) {
        //     ROS_ERROR("Invalid max angular velocity limit of %f (must be non-negative).",
        //         this->max_angular_vel);
        //     return 1;
        // }
        //
        // ROS_INFO("Velocity limits: linear: %f, angular: %f.",
        //     this->max_linear_vel, this->max_angular_vel);
        //
        // // Convert the linear acceleration limits to have units of (m/s^2)/20 since
        // // the movement commands are sent to the Segway at 20Hz.
        // // this->linear_pos_accel_limit /= 20;
        // // this->linear_neg_accel_limit /= 20;
        //
        // // Convert the angular acceleration limits to have units of (deg/s^2)/20 since
        // // the movement commands are sent to the Segway at 20Hz.
        // // this->angular_pos_accel_limit /= 20;
        // // this->angular_neg_accel_limit /= 20;
        //
        // // Get the scale correction parameters for odometry
        // n->param("linear_odom_scale", this->linear_odom_scale, 1.0);
        // n->param("angular_odom_scale", this->angular_odom_scale, 1.0);
        //
        // // Check if a software odometry reset is required
        // n->param("reset_odometry", this->reset_odometry, false);
        // n->param("odometry_reset_duration", this->odometry_reset_duration, 1.0);

        return 0;
    }

    // Variables

    // ros::NodeHandle * n;
    //
    // ros::Timer keep_alive_timer;
    //
    // ros::Subscriber cmd_velSubscriber, cmd_accelSubscriber, halt_sub;
    // ros::Subscriber  joy_sub, jyja_sub;
    // ros::Publisher segway_status_pub;
    // ros::Publisher odom_pub;
    // ros::Publisher vel_pub;
    // tf::TransformBroadcaster odom_broadcaster;

    segwayrmp::SegwayRMP * segway_rmp;

    std::string interface_type_str;
    segwayrmp::InterfaceType interface_type;
    segwayrmp::SegwayRMPType segway_rmp_type;
    std::string serial_port;
    std::string usb_selector;
    std::string serial_number;
    std::string usb_description;
    int usb_index;

    double segway_motor_timeout;
    // ros::Timer motor_timeout_timer;

    std::string frame_id;
    std::string odom_frame_id;
    bool invert_x, invert_z;
    bool broadcast_tf;

    double linear_vel;
    double angular_vel; // The angular velocity in deg/s

    double before_target_linear_vel;
    double linear_vel_feedback;

    int zero_judge;
    // ros::Time joy_arrival_time;
    std::chrono::system_clock::time_point jyja_arrival_time;

    double target_linear_vel;  // The ideal linear velocity in m/s
    double target_angular_vel; // The ideal angular velocity in deg/s

    double linear_pos_accel_limit;  // The max linear acceleration in (m/s^2)/20
    double linear_neg_accel_limit;  // The max linear deceleration in (m/s^2)/20
    double angular_pos_accel_limit; // The max angular acceleration in (deg/s^2)/20
    double angular_neg_accel_limit; // The max angular deceleration in (deg/s^2)/20

    double linear_odom_scale;       // linear odometry scale correction
    double angular_odom_scale;      // angular odometry scale correction

    double max_linear_vel;  // maximum allowed magnitude of velocity
    double max_angular_vel;

    bool connected;

    // segway_rmp::SegwayStatusStamped sss_msg;
    // geometry_msgs::TransformStamped odom_trans;
    // nav_msgs::Odometry odom_msg;

    bool first_odometry;
    float last_forward_displacement;
    float last_yaw_displacement;
    float odometry_x;
    float odometry_y;
    float odometry_w;
    // ros::Time last_time;

    boost::mutex m_mutex;

    // Hardware reset of integrators can sometimes fail.
    // These help in performing a software reset.
    bool reset_odometry;
    double odometry_reset_duration;
    // ros::Time odometry_reset_start_time;
    double initial_integrated_forward_position;
    double initial_integrated_left_wheel_position;
    double initial_integrated_right_wheel_position;
    double initial_integrated_turn_position;

    bool createdTimer;

    // ChangeVelocity* cv;
    int latch;

    double lin, ang;

    // BanAccel* ba;

    double v1, v2;
    double gain;

    bool obstacle_detected;
    // ros::Subscriber obstacle_sub;

    bool motors_enabled, recover_motors_enabled;

    bool joy_control;

}; // class SegwayRMPNode

// Callback wrapper
// void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr &ss) {
//     segwayrmp_node_instance->handleStatus(ss);
// }

void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss) { // removed '&' by Ojima
    segwayrmp_node_instance->handleStatus(ss);
}

int main(int argc, char **argv) {

    SegwayRMPNode segwayrmp_node;

    segwayrmp_node.run();

    return 0;
}
