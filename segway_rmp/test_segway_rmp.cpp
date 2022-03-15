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

#include <arpa/inet.h>
#include <unistd.h>

int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
struct sockaddr_in addr;
struct My_udp_data {
    double obstacle_rate = 0.0;
};

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


class BanAccel {
    int* latch;
    int section, reverse;
    double t, ct, total_time, T1, T2, T3;
    double vel, a;
    double vel_limit;
    double x;
public:
    BanAccel(int* latch): latch(latch) {}
    void setup(double T2, double a, double vel_limit, int reverse) {
        this->reverse = reverse;
        this->vel_limit = vel_limit;
        this->a = a;
        this->T1 = vel_limit / a;
        this->T3 = vel_limit / a;
        this->T2 = T2;
        this->total_time = this->T1 + this->T2 + this->T3;
        this->x =  (this->T2 + (this->T1 + this->T2 + this->T3)) * vel_limit / 2;
        this->vel = 0;
        this->t = 0;
        this->ct = 0;
        this->section = 1;
    }
    Lavel controller() {
        switch (this->section) {
            case 1:
                section_1();
                break;
            case 2:
                section_2();
                break;
            case 3:
                section_3();
                break;
        }
        Lavel la;
        la.linear_vel = this->vel;
        la.angular_vel = 0;
        if (reverse) {
            la.linear_vel = -la.linear_vel;
        }
        return la;
    }
    void section_1() {
        if (this->t < this->T1) {
            this->vel = this->a * this->t;
            this->t += dt;
            this->ct += dt;
        }
        else {
            this->t = 0;
            this->section = 2;
            return section_2();
        }
        return;
    }
    void section_2() {
        if (this->t < this->T2) {
            this->vel = this->vel_limit;
            this->t += dt;
            this->ct += dt;
        }
        else {
            this->t = 0;
            this->section = 3;
            return section_3();
        }
    }
    void section_3() {
        if (this->t < this->T3) {
            this->vel = this->vel_limit - this->a * this->t;
            this->t += dt;
            this->ct += dt;
        }
        else {
            this->vel = 0;
            *(this->latch) = 3;
            return;
        }
        return;
    }
};

// ROS Node class
class SegwayRMPNode {
public:
    SegwayRMPNode() {
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
        this->latch = 0;
        this->lin = 0;
        this->ang = 0;
        this->before_target_linear_vel = 0;
        this->linear_vel_feedback = 0;
        this->zero_judge = 0;
        this->offset = 0.04;
        this->gain = 0.4;
        this->offset_gain_none_latch = 1;
        this->obstacle_detected = false;
        this->motors_enabled = false;
        this->recover_motors_enabled = false;
        this->reset_odometry = false;
        this->joy_control = false;
        this->d1_count = 0;
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
                if (buf_ptr[0] == 0x43) {
                    // this->ang = 50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                    // this->lin = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                    if (this->latch == 3) {
                        this->ang = 30*(int8_t)buf_ptr[2] /127.0;
                        this->lin = 0.2*(int8_t)buf_ptr[3] /127.0;
                        // printf("%lf, %lf\n", this->ang, this->lin);
                    }
                    this->jyja_arrival_time = std::chrono::system_clock::now();
                }

                else if (buf_ptr[0] == 0xab) {
                    if (this->latch == 3) {
                        this->latch = 2;
                        this->begin_time_point = std::chrono::system_clock::now();
                        this->ba->setup((int8_t)buf_ptr[1]/2.0, (int8_t)buf_ptr[2]/20.0, (int8_t)buf_ptr[3]/20.0, 1);
                    }
                }
                else if (buf_ptr[0] == 0xaf) {
                    if (this->latch == 3) {
                        this->latch = 2;
                        this->begin_time_point = std::chrono::system_clock::now();
                        this->ba->setup((int8_t)buf_ptr[1]/2.0, (int8_t)buf_ptr[2]/20.0, (int8_t)buf_ptr[3]/20.0, 0);
                    }
                }

                else if (buf_ptr[0] == 0x01) {
                    this->offset_gain_none_latch = 1;
                    this->offset = (int8_t)buf_ptr[3]/100.0;
                }
                else if (buf_ptr[0] == 0x02) {
                    this->offset_gain_none_latch = 2;
                    this->gain = (int8_t)buf_ptr[3]/100.0;
                }
                else if (buf_ptr[0] == 0x03) {
                    this->offset_gain_none_latch = 3;
                }

                else if (buf_ptr[0] == 0xd1) {
                    if (this->latch == 2) {
                        this->d1_count = 10;
                        this->ang = 4*(int8_t)buf_ptr[2]/127.0;
                    }
                }

                else if (buf_ptr[0] == 0x99) {
                    this->latch = 0;
                    this->lin = 0;
                    this->ang = 0;
                    // std::cout << "segway_rmp_node を終了\n";
                    // std_msgs::String msg;
                    // msg.data = "quit";
                    // halt_pub.publish(msg);
                }
            }
        }
        return;
    }

    void joy_read() {
        int joy_fd = -1;

        while (true) {
            int search_count = 0;
            while (true) {
                if (joy_fd < 0) {
                    search_count++;
                    printf("[%d] connecting to joystick ...\n", joy_fd);
                    joy_fd = open(JOY_DEV, O_RDONLY);
                    if (joy_fd >= 0) {
                        break;
                    }
                    if (search_count > 3) {
                        this->latch = 0;
                        this->lin = 0;
                        this->ang = 0;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

            int count = 0;

            while (true && count < 10000) {
                js_event js;
                read(joy_fd, &js, sizeof(js_event));

                count++;

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

                // printf("count %d\n", count);
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


                if ((int)joy_button.at(10) && (int)joy_button.at(13)) { //L1 and ○
                    this->latch = 1;
                }
                if ((int)joy_button.at(10) && (int)joy_button.at(12)) { //L1 and △
                    this->latch = 3;
                    this->ang = 0;
                    this->lin = 0;
                }
                if ((int)joy_button.at(14)) { //x
                    this->latch = 0;
                    this->ang = 0;
                    this->lin = 0;
                }

                if (this->latch == 1) {
                    this->ang = -50.0*joy_axis.at(0)/32767.0;
                    this->lin = -1.5*joy_axis.at(3)/32767.0;
                }
            }
            close(joy_fd);
            joy_fd = -1;
        }
        return;
    }

    void udp_read() {
        while (1) {
            struct My_udp_data my_udp_data = {0};
            int recv_size = recv(sockfd, &my_udp_data, sizeof(struct My_udp_data), 0);
            if (my_udp_data.obstacle_rate > 0.4) {
                this->obstacle_detected = true;
            }
            else {
                this->obstacle_detected = false;
            }
            // printf("read %d byte: %lf\n", recv_size, my_udp_data.obstacle_rate);
        }
    }

    void run() {
        this->fd_write = open(SERIAL_PATH, O_WRONLY); // SERIAL_PATH は serialPathConfig.h.in にて定義されている。
        if (this->getParameters()) {
            return;
        }

        this->begin_time_point = std::chrono::system_clock::now();

        this->setupSegwayRMP();

        this->ba = new BanAccel(&(this->latch));

        boost::thread th_momo_serial_read(&SegwayRMPNode::momo_serial_read, this);
        boost::thread th_joy_read(&SegwayRMPNode::joy_read, this);
        boost::thread th_udp_read(&SegwayRMPNode::udp_read, this);

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
            this->segway_rmp->setMaxVelocityScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            this->segway_rmp->setMaxAccelerationScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            this->segway_rmp->setMaxTurnScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            this->segway_rmp->setCurrentLimitScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            this->segway_rmp->setBalanceModeLocking(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            this->segway_rmp->setOperationalMode(segwayrmp::tractor);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            this->segway_rmp->setControllerGainSchedule(segwayrmp::heavy);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            while (true && this->connected) {

                if (!this->connected || this->reset_odometry) {
                    continue;
                }

                if (!this->motors_enabled) {

                    continue;
                }

                if (this->recover_motors_enabled) {
                    this->segway_rmp->resetAllIntegrators();
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    this->segway_rmp->setMaxVelocityScaleFactor(1.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    this->segway_rmp->setMaxAccelerationScaleFactor(1.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    this->segway_rmp->setMaxTurnScaleFactor(1.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    this->segway_rmp->setCurrentLimitScaleFactor(1.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    this->segway_rmp->setBalanceModeLocking(true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    this->segway_rmp->setOperationalMode(segwayrmp::tractor);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    this->segway_rmp->setControllerGainSchedule(segwayrmp::heavy);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    this->recover_motors_enabled = false;
                }

                // boost::mutex::scoped_lock lock(this->m_mutex);

                Lavel la;
                if (this->latch == 0) {
                    this->lin = 0;
                    this->ang = 0;
                }
                else if (this->latch == 1) {
                }
                else if (this->latch == 2) {
                    la = this->ba->controller();
                    if (this->offset_gain_none_latch == 1) {
                        this->lin = la.linear_vel + this->offset;
                    }
                    else if (this->offset_gain_none_latch == 2) {
                        this->lin = (this->lin - this->linear_vel_feedback)*(this->gain) + la.linear_vel;
                    }
                    else if (this->offset_gain_none_latch == 3) {
                        this->lin = la.linear_vel;
                    }

                    // this->ang = la.angular_vel;
                    if (this->d1_count > 0) {
                        this->d1_count++;
                    }
                    if (this->d1_count > 15) {
                        this->d1_count = 0;
                        this->ang = 0;
                    }
                }
                else if (this->latch == 3) {
                    if (std::chrono::system_clock::now() - this->jyja_arrival_time > std::chrono::milliseconds(500)) {
                        this->lin = 0;
                        this->ang = 0;
                    }
                }

                try {
                    if (this->obstacle_detected && this->lin > 0) {
                        this->lin = 0;
                        if (this->latch == 2) {
                            this->latch = 0;
                        }
                    }
                    // printf("%lf, %lf\n", this->ang, this->lin);
                    this->segway_rmp->move(this->lin, this->ang);
                } catch (std::exception& e) {
                    std::string e_msg(e.what());
                    printf("Error commanding Segway RMP: %s", e_msg.c_str());
                    this->connected = false;
                    this->disconnect();
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        if (true) { // Error not shutdown
            return true;
        } else {         // Shutdown
            return false;
        }
    }

    void handleStatus(segwayrmp::SegwayStatus::Ptr &ss_ptr) {
        if (!this->connected) {
            return;
        }
        // // Get the time
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
        int16_t vel = (int16_t)(this->linear_vel_feedback * 10000.0);
        uint8_t h = (uint8_t)((uint16_t)(vel & 0xff00) >> 8);
        uint8_t l = (uint8_t)(vel & 0x00ff);
        int end_time_point = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - this->begin_time_point).count();
        uint8_t hht = (uint8_t)((uint32_t)(end_time_point & 0xff000000) >> 24);
        uint8_t ht = (uint8_t)((uint32_t)(end_time_point & 0x00ff0000) >> 16);
        uint8_t lt = (uint8_t)((uint32_t)(end_time_point & 0x0000ff00) >> 8);
        uint8_t llt = (uint8_t)(end_time_point & 0x000000ff);
        uint8_t buf[7] = {hht, ht, lt, llt, h, l, '\n'};
        write(this->fd_write, &buf, 7);

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

private:

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
        this->interface_type_str = std::string("serial");
        // Get Configurations based on Interface Type
        if (this->interface_type_str == "serial") {
            this->interface_type = segwayrmp::serial;
            this->serial_port = std::string("/dev/ttyUSB0");
        } else if (this->interface_type_str == "usb") {
            this->interface_type = segwayrmp::usb;
            this->usb_selector = std::string("serial_number");
            if (this->usb_selector == "index") {
                this->usb_index = 0;
            } else if (this->usb_selector == "serial_number") {
                this->serial_number = std::string("0403e729");
                if (this->serial_number == std::string("00000000")) {
                    printf("The serial_number parameter is set to the default 00000000, which shouldn't work.\n");
                }
            } else if (this->usb_selector == "description") {
                this->serial_number = std::string("Robotic Mobile Platform");
            } else {
                printf(
                    "Invalid USB selector: %s, valid types are 'index', 'serial_number', and 'description'.\n",
                    this->usb_selector.c_str());
                return 1;
            }
        } else {
            printf(
                "Invalid interface type: %s, valid interface types are 'serial' and 'usb'.\n",
                this->interface_type_str.c_str());
            return 1;
        }
        // Get Setup Motor Timeout
        this->segway_motor_timeout = 0.5;
        // Get frame id parameter
        this->frame_id = std::string("base_link");
        this->odom_frame_id = std::string("odom");

        // Get the segway rmp type
        std::string segway_rmp_type_str;
        segway_rmp_type_str = std::string("200/400");
        if (segway_rmp_type_str == "200/400") {
            this->segway_rmp_type = segwayrmp::rmp200;
        } else if (segway_rmp_type_str == "50/100") {
            this->segway_rmp_type = segwayrmp::rmp100;
        } else {
            printf(
                "Invalid rmp type: %s, valid rmp types are '200/400' and '50/100'.\n",
                segway_rmp_type_str.c_str());
            return 1;
        }


        // // Get the scale correction parameters for odometry
        this->linear_odom_scale = 1.0;
        this->angular_odom_scale = 1.0;
        //
        // // Check if a software odometry reset is required
        this->reset_odometry = false;
        this->odometry_reset_duration = 1.0;

        return 0;
    }

    // Variables

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

    BanAccel* ba;

    double offset, gain;
    int offset_gain_none_latch; // 1: offset, 2: gain, 3: none

    bool obstacle_detected;
    // ros::Subscriber obstacle_sub;

    bool motors_enabled, recover_motors_enabled;

    bool joy_control;

    int fd_write;

    std::chrono::system_clock::time_point begin_time_point;

    int d1_count;

}; // class SegwayRMPNode

// Callback wrapper
// void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr &ss) {
//     segwayrmp_node_instance->handleStatus(ss);
// }

void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss) { // removed '&' by Ojima
    segwayrmp_node_instance->handleStatus(ss);
}

int main(int argc, char **argv) {
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    addr.sin_port = htons(4001);

    bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr));

    SegwayRMPNode segwayrmp_node;

    segwayrmp_node.run();

    return 0;
}
