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
#include <fstream>
#include <ctime>
#include <iostream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>

#include <sys/fcntl.h>
#include <iomanip>
#include "serialPathConfig.h" // SERIAL_PATH を定義

#include "segwayrmp/segwayrmp.h"

#include <vector>
#include <linux/joystick.h>
#define JOY_DEV "/dev/input/js0"

#include <arpa/inet.h>
#include <unistd.h>

int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
struct sockaddr_in addr;
struct My_udp_data {
    char obstacle_detected_in_0_5m = 0;
    char obstacle_detected_in_1_5m = 0;
};

int sockfd_epos = socket(AF_INET, SOCK_DGRAM, 0);
struct sockaddr_in addr_epos;

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
    double section;
};


class BanAccel {
    int* latch;
    int section, reverse;
    double t, ct, total_time, T1, T2, T3;
    double vel, a;
    double vel_limit;
    double x;
    char n;
    char count;
public:
    BanAccel(int* latch): latch(latch), count(0) {}
    void setup(double T2, double a, double vel_limit, int reverse, char n) {
        this->count = 1;
        this->n = n;
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
        la.section = this->section;
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
            if (this->count == this->n) {
                *(this->latch) = 3;
            }
            else {
                this->count++;
                this->t = 0;
                this->section = 1;
                return section_1();
            }
            return;
        }
        return;
    }
};


int latch = 3;
double lin = 0, cmd_linear_vel_from_momo = 0, cmd_linear_vel_from_joystick= 0;
double ang = 0, cmd_angular_vel_from_momo = 0, cmd_angular_vel_from_joystick = 0;
double before_target_linear_vel = 0;
double linear_vel_feedback = 0;
double angular_vel_feedback = 0;
double offset = 0.04;
double gain = 0.4;
int offset_gain_none_latch = 1;
bool obstacle_detected_in_0_5m = false;
bool obstacle_detected_in_1_5m = false;
bool motors_enabled = false;
bool recover_motors_enabled = false;
int d1_count = 0;
bool ofs_closed = true;
int log_margin_count = 0;
int fd_write;
std::chrono::system_clock::time_point begin_time_point;
std::chrono::system_clock::time_point jyja_arrival_time;
bool connected = false;

BanAccel* ba;

double forward_position = 0, turn_position = 0;
double position_x = 0, position_z = 0;
double init_fp = 0, init_tp = 0;

std::ofstream* ofs;

std::mutex m_mutex;

bool stop_auto_moving = false;
double stop_auto_moving_lin = 0;

class MovingPlan {
    int* latch;
    int section;
    double t, ct, total_time, T1, T2, T3;
    double vel, a, ang;
    double vel_limit;
    double x;
    double target_x, target_turn;
    double initial_forward_position, initial_turn_position;
public:
    MovingPlan(int* latch): latch(latch) {}
    void setup(double target_x, double target_turn) {
        this->vel_limit = 0.9;
        this->a = 0.1;
        this->T1 = this->vel_limit / this->a;
        this->T3 = this->vel_limit / this->a;
        this->x = target_x;
        if (this->T1*this->vel_limit > this->x) {
            this->T1 = sqrt(this->x/this->a);
            this->vel_limit = this->a * this->T1;
            this->T2 = 0;
            this->T3 = this->T1;
        }
        else {
            this->T2 = this->x/this->vel_limit - this->T1;
        }
        this->total_time = this->T1 + this->T2 + this->T3;
        this->vel = 0;
        this->t = 0;
        this->ct = 0;
        this->section = 0;
        this->target_turn = target_turn;
        this->initial_forward_position = forward_position;
        this->initial_turn_position = turn_position;
    }
    Lavel controller() {
        switch (this->section) {
            case 0:
                section_0();
                break;
            case 1:
                section_1();
                break;
            case 2:
                section_2();
                break;
            case 3:
                section_3();
                break;
            case 4:
                section_4();
        }
        Lavel la;
        la.linear_vel = this->vel;
        la.angular_vel = this->ang;
        la.section = this->section;
        return la;
    }
    void section_0() {
        if (fabs(turn_position - this->initial_turn_position) < fabs(this->target_turn) - 1) {
            if (target_turn > 0) {
                this->ang = 20;
            }
            else {
                this->ang = -20;
            }
            this->vel = 0;
        }
        else {
            this->ang = 0;
            this->section = 1;
            return section_1();
        }
        return;
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
        if (this->t < this->T2 + 2.0) {
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
            if (this->vel < 0.15) {
                this->section = 4;
                return section_4();
            }
        }
        else {
            this->vel = 0;
            *(this->latch) = 3;
            return;
        }
        return;
    }
    void section_4() {
        if (forward_position < this->initial_forward_position + this->x) {
            this->vel = 0.1;
            return;
        }
        else {
            this->vel = 0;
            *(this->latch) = 3;
            return;
        }
    }
};

MovingPlan* movingplan;
int momo_send_count = 0;

void handleStatus(segwayrmp::SegwayStatus::Ptr ss_ptr) {
    if (!connected) {
        return;
    }
    segwayrmp::SegwayStatus &ss = *(ss_ptr);

    angular_vel_feedback = ss.yaw_rate; // (deg/s)
    int ang_vel = (int32_t)(angular_vel_feedback * 10000);
    uint8_t hha = (uint8_t)((uint32_t)(ang_vel & 0xff000000) >> 24);
    uint8_t ha = (uint8_t)((uint32_t)(ang_vel & 0x00ff0000) >> 16);
    uint8_t la = (uint8_t)((uint32_t)(ang_vel & 0x0000ff00) >> 8);
    uint8_t lla = (uint8_t)(ang_vel & 0x000000ff);

    linear_vel_feedback = (ss.left_wheel_speed + ss.right_wheel_speed) / 2.0; // (m/s)
    int vel = (int32_t)(linear_vel_feedback * 10000.0);
    uint8_t hh = (uint8_t)((uint32_t)(vel & 0xff000000) >> 24);
    uint8_t h = (uint8_t)((uint32_t)(vel & 0x00ff0000) >> 16);
    uint8_t l = (uint8_t)((uint32_t)(vel & 0x0000ff00) >> 8);
    uint8_t ll = (uint8_t)(vel & 0x000000ff);

    int time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - 1658000000000;
    uint8_t hht = (uint8_t)((uint32_t)(time_since_epoch & 0xff000000) >> 24);
    uint8_t ht = (uint8_t)((uint32_t)(time_since_epoch & 0x00ff0000) >> 16);
    uint8_t lt = (uint8_t)((uint32_t)(time_since_epoch & 0x0000ff00) >> 8);
    uint8_t llt = (uint8_t)(time_since_epoch & 0x000000ff);

    // int fp = (int16_t)((ss.integrated_forward_position - init_fp) * 100.0);
    // uint8_t hfp = (uint8_t)((uint16_t)(fp & 0xff00) >> 8);
    // uint8_t lfp = (uint8_t)(fp & 0x00ff);
    //
    int tp = (int16_t)((ss.integrated_turn_position) * 100.0);
    uint8_t htp = (uint8_t)((uint16_t)(tp & 0xff00) >> 8);
    uint8_t ltp = (uint8_t)(tp & 0x00ff);

    int p_x = position_x * 100.0;
    uint8_t hx = (uint8_t)((uint16_t)(p_x & 0xff00) >> 8);
    uint8_t lx = (uint8_t)(p_x & 0x00ff);

    int p_z = position_z * 100.0;
    uint8_t hz = (uint8_t)((uint16_t)(p_z & 0xff00) >> 8);
    uint8_t lz = (uint8_t)(p_z & 0x00ff);

    uint8_t buf[21] = {0x45, hht, ht, lt, llt, hh, h, l, ll, hha, ha, la, lla, (uint8_t)latch, htp, ltp, hx, lx, hz, lz, '\n'};

    momo_send_count++;
    if (momo_send_count > 20) {
        write(fd_write, &buf, 21);
        momo_send_count = 0;
    }

    // if (latch == 2 && !ofs_closed) {
    //     *(ofs) << end_time_point/1000.0 << ' ' << linear_vel_feedback << '\n';
    // }
    // else if (latch == 3 && !ofs_closed && log_margin_count < 200) {
    //     *(ofs) << end_time_point/1000.0 << ' ' << linear_vel_feedback << '\n';
    //     log_margin_count++;
    // }
    // else if (latch == 3 && !ofs_closed && log_margin_count >= 200) {
    //     ofs->close();
    //     ofs_closed = true;
    //     log_margin_count = 0;
    // }

    if (!ofs_closed) {
        char char_array_millisec_since_epoch[32];
        std::sprintf(char_array_millisec_since_epoch, "%.3lf", time_since_epoch/1000.0);
        *ofs << std::string(char_array_millisec_since_epoch) << ' ' << linear_vel_feedback << '\n';
    }

    if (!motors_enabled && (bool)(ss.motor_status)) {
        recover_motors_enabled = true;
    }
    motors_enabled = (bool)(ss.motor_status);


    // printf("forward speed: %.2lf (m/s)\n", linear_vel_feedback);
    // printf("turn speed: %.2lf (deg/s)\n", angular_vel_feedback);
    // printf("plan forward position: %.2lf (m)\n", ss.integrated_forward_position - init_fp);
    // printf("plan turn position: %.2lf (deg)\n", ss.integrated_turn_position - init_tp);
    // printf("ui_battery_voltage: %lf\n", ss.ui_battery_voltage);
    // printf("powerbase_battery_voltage: %lf\n\n", ss.powerbase_battery_voltage);

    double tangent = ss.integrated_forward_position - forward_position;

    position_x = position_x + tangent*cos(ss.integrated_turn_position/180.0*M_PI + M_PI_2);
    position_z = position_z + tangent*sin(ss.integrated_turn_position/180.0*M_PI + M_PI_2);

    // printf("position: x %.2lf(m), z %.2lf(m)\n", position_x, position_z);
    // printf("ui_battery_voltage: %lf\n", ss.ui_battery_voltage);
    // printf("powerbase_battery_voltage: %lf\n\n", ss.powerbase_battery_voltage);

    forward_position = ss.integrated_forward_position;
    turn_position = ss.integrated_turn_position;

    return;
}


void joy_read() {
    int joy_fd = -1;
    // MyQueue my_queue;

    double vel1 = 0.0, vel2 = 0.0;

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
                    // my_queue.reset_zero();
                    latch = 0;
                    lin = 0;
                    ang = 0;
                    cmd_linear_vel_from_momo = 0;
                    cmd_angular_vel_from_momo = 0;
                    cmd_linear_vel_from_joystick = 0;
                    cmd_angular_vel_from_joystick = 0;
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
        // std::cout << "Joystick:  " << name_of_joystick << '\n';
        // std::cout << "  axis: " << num_of_axis << '\n';
        // std::cout << "  buttons: " << num_of_buttons << '\n';

        int count = 0;

        while (true && count < 1000) {
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

            if (count < 30) {
                continue;
            }


            if ((int)joy_button.at(10) && (int)joy_button.at(13)) { //L1 and ○
                latch = 1;
                ang = 0;
                lin = 0;
                cmd_linear_vel_from_momo = 0;
                cmd_angular_vel_from_momo = 0;
                cmd_linear_vel_from_joystick = 0;
                cmd_angular_vel_from_joystick = 0;
            }
            if ((int)joy_button.at(10) && (int)joy_button.at(12)) { //L1 and △
                latch = 3;
                ang = 0;
                lin = 0;
                cmd_linear_vel_from_momo = 0;
                cmd_angular_vel_from_momo = 0;
                cmd_linear_vel_from_joystick = 0;
                cmd_angular_vel_from_joystick = 0;
            }
            if ((int)joy_button.at(14)) { //x
                latch = 0;
                ang = 0;
                lin = 0;
                cmd_linear_vel_from_momo = 0;
                cmd_angular_vel_from_momo = 0;
                cmd_linear_vel_from_joystick = 0;
                cmd_angular_vel_from_joystick = 0;
            }

            if ((int)joy_button.at(8) && (int)joy_button.at(9) && (int)joy_button.at(10) && (int)joy_button.at(11)) {
                system("shutdown -h 0");
            }

            if (latch == 1) {
                // this->ang = -50.0*joy_axis.at(0)/32767.0;
                // if (joy_axis.at(3) < 0) {
                //     joy_lin = -1.5*joy_axis.at(3)/32767.0 * fabs(joy_axis.at(3)/32767.0);
                // }
                // else {
                //     joy_lin = -0.5*joy_axis.at(3)/32767.0 * fabs(joy_axis.at(3)/32767.0);
                // }
                // cmd_angular_vel_from_joystick = -50.0*joy_axis.at(0)/32767.0 * fabs(joy_axis.at(0)/32767.0);
                // this->joy_lin = -1.5*joy_axis.at(3)/32767.0 * fabs(joy_axis.at(3)/32767.0);
                // my_queue.enqueue(-1.5*joy_axis.at(3)/32767.0 * fabs(joy_axis.at(3)/32767.0));
                // this->lin = my_queue.mean();

                double A = 0; // 指令値 (m/s) の最大値
                double k = 0.05;
                double x = -joy_axis.at(3)/32767.0; // joystick の入力値 -1 ~ 1

                if (x > 0) {
                    A = 0.5;
                    cmd_linear_vel_from_joystick = A*((1 - k)*x + k)*x;  // cmd_linear_vel_from_joystick は指令値 (m/s)
                }
                else {
                    A = 0.5;
                    cmd_linear_vel_from_joystick = - A*((1 - k)*(-x) + k)*(-x);  // cmd_linear_vel_from_joystick は指令値 (m/s)
                }

                A = 40; // 指令値 (deg/s) の最大値
                x = -joy_axis.at(0)/32767.0; // joystick の入力値 -1 ~ 1

                if (x > 0) {
                    cmd_angular_vel_from_joystick = A*((1 - k)*x + k)*x;  // cmd_angular_vel_from_joystick は指令値 (deg/s)
                }
                else {
                    cmd_angular_vel_from_joystick = - A*((1 - k)*(-x) + k)*(-x);  // cmd_angular_vel_from_joystick は指令値 (deg/s)
                }

            }
        }
        close(joy_fd);
        joy_fd = -1;
    }
    return;
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
        // printf("read %d byte: 0x%02x %4d %4d %4d\n", read_size, buf_ptr[0], buf_ptr[1], buf_ptr[2], (int8_t)buf_ptr[3]);
        // printf("read %d byte: %08x\n", read_size, buf_ptr[0]);
        if (read_size == 3) {
            if (buf_ptr[0] == 0xe0) {
                uint8_t val[2] = {buf_ptr[1], buf_ptr[2]};
                sendto(sockfd_epos, &val, 2*sizeof(uint8_t), 0, (struct sockaddr *)&addr_epos, sizeof(addr_epos));
            }
        }
        if (read_size == 4) {
            if (buf_ptr[0] == 0x43) {
                // this->ang = 50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                // this->lin = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                if (latch == 3) {
                    cmd_angular_vel_from_momo = 50*(int8_t)buf_ptr[2] /127.0 * std::fabs((int8_t)buf_ptr[2] /127.0);

                    double A = 0; // 指令値 (m/s) の最大値
                    double k = 0.05;
                    double x = (int8_t)buf_ptr[3] /127.0; // 遠隔のjoystick の入力値 -1 ~ 1
                    if (x > 0) {
                        A = 0.4;
                        cmd_linear_vel_from_momo = A*((1 - k)*x + k)*x;  // cmd_linear_vel_from_momo は指令値 (m/s)
                    }
                    else {
                        A = 0.4;
                        cmd_linear_vel_from_momo = - A*((1 - k)*(-x) + k)*(-x);  // cmd_linear_vel_from_momo は指令値 (m/s)
                    }

                    A = 40.0; // 指令値 (deg/s) の最大値
                    k = 0.05;
                    x = (int8_t)buf_ptr[2] /127.0; // 遠隔のjoystick の入力値 -1 ~ 1
                    if (x > 0) {
                        cmd_angular_vel_from_momo = A*((1 - k)*x + k)*x;  // cmd_linear_vel_from_momo は指令値 (m/s)
                    }
                    else {
                        cmd_angular_vel_from_momo = - A*((1 - k)*(-x) + k)*(-x);  // cmd_linear_vel_from_momo は指令値 (m/s)
                    }
                }
                jyja_arrival_time = std::chrono::system_clock::now();
            }
            else if (buf_ptr[0] == 0x44) {
                if (latch == 3) {
                    latch = 4;
                    init_fp = forward_position;
                    init_tp = turn_position;
                    movingplan->setup((int8_t)buf_ptr[2]/10.0, (int8_t)buf_ptr[3]*2.0);
                }
            }
            else if (buf_ptr[0] == 0x46) {
                std::stringstream ss;
                time_t t = time(NULL);
                std::string str = ctime(&t);
                ss << "./log/" << str.substr(0, str.size()-1) << "_data_delay.log";
                ofs = new std::ofstream(ss.str());
                ofs_closed = false;
            }
            else if (buf_ptr[0] == 0x47) {
                ofs_closed = true;
                ofs->close();
            }

            // else if (buf_ptr[0] == 0xab) {
            //     if (latch == 3) {
            //         latch = 2;
            //         begin_time_point = std::chrono::system_clock::now();
            //         ba->setup((int8_t)buf_ptr[1]/2.0, (int8_t)buf_ptr[2]/20.0, (int8_t)buf_ptr[3]/20.0, 1);
            //     }
            // }
            // else if ((buf_ptr[0] & 0xf0) == 0xa0) {
            //     if (latch == 3) {
            //         latch = 2;
            //         log_margin_count = 0;
            //         begin_time_point = std::chrono::system_clock::now();
            //         ba->setup((int8_t)buf_ptr[1]/2.0, (int8_t)buf_ptr[2]/20.0, (int8_t)buf_ptr[3]/20.0, 0, (int8_t)(buf_ptr[0] & 0x0f));
            //         std::stringstream ss;
            //         // ss << "./log/" << std::setfill('0') << std::setw(2) << file_counta << std::setfill(' ') << "actual_velocity_a_" << (int8_t)buf_ptr[2]/20.0 << "_v_" << (int8_t)buf_ptr[3]/20.0 << "_T2_" << (int8_t)buf_ptr[1]/2.0 << ".out";
            //         time_t t = time(NULL);
            //         std::string str = ctime(&t);
            //         ss << "./log/" << str.substr(0, str.size()-1) << "actual_velocity_a_" << (int8_t)buf_ptr[2]/20.0 << "_v_" << (int8_t)buf_ptr[3]/20.0 << "_T2_" << (int8_t)buf_ptr[1]/2.0 << "_count_" << (int)(buf_ptr[0] & 0x0f);
            //         switch (offset_gain_none_latch) {
            //             case 1: ss << "_offset_" << offset << ".out"; break;
            //             case 2: ss << "_gain_" << gain << ".out"; break;
            //             case 3: ss << "_none.out"; break;
            //         }
            //         ofs = new std::ofstream(ss.str());
            //         ofs_closed = false;
            //         *(ofs) << "#accel(m/s^2) " << (int8_t)buf_ptr[2]/20.0 << " max_vel(m/s) " << (int8_t)buf_ptr[3]/20.0 << " max_vel_time(s) " << (int8_t)buf_ptr[1]/2.0;
            //         switch (offset_gain_none_latch) {
            //             case 1: *(ofs) << " offset " << offset; break;
            //             case 2: *(ofs) << " gain " << gain; break;
            //             case 3: *(ofs) << " none "; break;
            //         }
            //         *(ofs) << " count " << (int)(buf_ptr[0] & 0x0f) << '\n';
            //         *(ofs) << "#時刻(s) 実際の速度(m/s)\n";
            //         ofs_closed = false;
            //     }
            // }

            else if (buf_ptr[0] == 0x01) {
                offset_gain_none_latch = 1;
                offset = (int8_t)buf_ptr[3]/100.0;
            }
            else if (buf_ptr[0] == 0x02) {
                offset_gain_none_latch = 2;
                gain = (int8_t)buf_ptr[3]/100.0;
            }
            else if (buf_ptr[0] == 0x03) {
                offset_gain_none_latch = 3;
            }

            else if (buf_ptr[0] == 0xd1) {
                if (latch == 2) {
                    d1_count = 10;
                    ang = 3.5*(int8_t)buf_ptr[2]/127.0;
                }
            }

            else if (buf_ptr[0] == 0x99) {
                if (latch == 4) {
                    stop_auto_moving = true;
                    latch = 3;
                    stop_auto_moving_lin = linear_vel_feedback;
                }
                // std::cout << "segway_rmp_node を終了\n";
                // std_msgs::String msg;
                // msg.data = "quit";
                // halt_pub.publish(msg);
            }
        }
    }
    return;
}

void udp_read() {
    while (1) {
        struct My_udp_data my_udp_data = {0};
        int recv_size = recv(sockfd, &my_udp_data, sizeof(struct My_udp_data), 0);
        obstacle_detected_in_0_5m = my_udp_data.obstacle_detected_in_0_5m;
        obstacle_detected_in_1_5m = my_udp_data.obstacle_detected_in_1_5m;
        // printf("0.5m %d, 2m %d, 3m %d\n", my_udp_data.obstacle_detected_in_0_5m, my_udp_data.obstacle_detected_in_2m, my_udp_data.obstacle_detected_in_3m);
    }
}

void hoge() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    while (1) {
        if (latch == 4) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        double forward, turn;
        printf("前進 << ");
        scanf("%lf", &forward);
        printf("旋回 << ");
        scanf("%lf", &turn);
        latch = 4;
        movingplan->setup(forward, turn);
    }
    return;
}


int main(int argc, char **argv) {
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    addr.sin_port = htons(4001);

    bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr));

    addr_epos.sin_family = AF_INET;
    addr_epos.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr_epos.sin_port = htons(4002);


    fd_write = open(SERIAL_PATH, O_WRONLY); // SERIAL_PATH は serialPathConfig.h.in にて定義されている。


    segwayrmp::SegwayRMP segway_rmp(segwayrmp::serial, segwayrmp::rmp200);
    segway_rmp.configureSerial(std::string("/dev/ttyUSB0"));
    segway_rmp.setStatusCallback(handleStatus);
    segway_rmp.setLogMsgCallback("debug", handleDebugMessages);
    segway_rmp.setLogMsgCallback("info", handleInfoMessages);
    segway_rmp.setLogMsgCallback("error", handleErrorMessages);

    begin_time_point = std::chrono::system_clock::now();

    ba = new BanAccel(&latch);
    movingplan = new MovingPlan(&latch);

    std::thread th_momo_serial_read(momo_serial_read);
    std::thread th_joy_read(joy_read);
    std::thread th_udp_read(udp_read);

    // std::thread th_hoge(hoge);

    connected = false;
    double emergency_brake_lin = 0, slow_start_lin = 0, slow_brake_lin = 0;
    bool slow_start = false, emergency_brake = false, slow_brake = false;
    int stamp = 0;
    int section = 0;
    while (true) {
        try {
            segway_rmp.connect(true);
            connected = true;
        } catch (std::exception& e) {
            std::string e_msg(e.what());
            printf("Exception while connecting to the SegwayRMP, check your cables and power buttons.\n");
            printf("    %s\n", e_msg.c_str());
            connected = false;
        }
        if (connected) {
            printf("Segway RMP Ready.\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            segway_rmp.resetAllIntegrators();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            segway_rmp.setMaxVelocityScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            segway_rmp.setMaxAccelerationScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            segway_rmp.setMaxTurnScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            segway_rmp.setCurrentLimitScaleFactor(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            segway_rmp.setBalanceModeLocking(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            segway_rmp.setOperationalMode(segwayrmp::tractor);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            segway_rmp.setControllerGainSchedule(segwayrmp::heavy);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // while (1) {
            while (connected) {

                if (!motors_enabled) {

                    continue;
                }

                if (recover_motors_enabled) {
                    segway_rmp.resetAllIntegrators();
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    segway_rmp.setMaxVelocityScaleFactor(1.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    segway_rmp.setMaxAccelerationScaleFactor(1.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    segway_rmp.setMaxTurnScaleFactor(1.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    segway_rmp.setCurrentLimitScaleFactor(1.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    segway_rmp.setBalanceModeLocking(true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    segway_rmp.setOperationalMode(segwayrmp::tractor);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    segway_rmp.setControllerGainSchedule(segwayrmp::heavy);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    recover_motors_enabled = false;
                }

                // boost::mutex::scoped_lock lock(m_mutex);
                m_mutex.lock();

                if (latch == 0) {
                    lin = 0;
                    ang = 0;
                    cmd_linear_vel_from_momo = 0;
                    cmd_angular_vel_from_momo = 0;
                    cmd_linear_vel_from_joystick = 0;
                    cmd_angular_vel_from_joystick = 0;
                }
                else if (latch == 1) {
                    ang = cmd_angular_vel_from_joystick;
                    lin = cmd_linear_vel_from_joystick;
                }
                else if (latch == 2) {
                    Lavel la = ba->controller();
                    if (offset_gain_none_latch == 1) {
                        lin = la.linear_vel + offset;
                    }
                    else if (offset_gain_none_latch == 2) {
                        lin = (lin - linear_vel_feedback)*(gain) + la.linear_vel;
                    }
                    else if (offset_gain_none_latch == 3) {
                        lin = la.linear_vel;
                    }

                    // ang = la.angular_vel;
                    if (d1_count >= 10) {
                        d1_count++;
                    }
                    if (d1_count > 15) {
                        d1_count = 0;
                        ang = 0;
                    }
                }
                else if (latch == 3) {
                    if (std::chrono::system_clock::now() - jyja_arrival_time > std::chrono::milliseconds(500)) {
                        lin = 0;
                        ang = 0;
                        cmd_linear_vel_from_momo = 0;
                        cmd_angular_vel_from_momo = 0;
                        cmd_linear_vel_from_joystick = 0;
                        cmd_angular_vel_from_joystick = 0;
                    }
                    else {
                        ang = cmd_angular_vel_from_momo;
                        lin = cmd_linear_vel_from_momo;
                    }
                }
                else if (latch == 4) {
                    Lavel la = movingplan->controller();
                    lin = la.linear_vel;
                    ang = la.angular_vel;
                    section = la.section;
                }


                try {

                    if (stop_auto_moving) {
                        if (stop_auto_moving_lin > 0) {
                            stop_auto_moving_lin = stop_auto_moving_lin - 0.01;
                        }
                        if (stop_auto_moving_lin <= 0) {
                            stop_auto_moving_lin = 0;
                            stop_auto_moving = false;
                        }
                        lin = stop_auto_moving_lin;
                    }


                    if (obstacle_detected_in_0_5m) {
                        if ((latch != 4 && lin > 0) || (latch == 4 && section != 0)) {
                            if (!emergency_brake) {
                                emergency_brake = true;
                                // if (latch == 4) {
                                //     latch = 3;
                                // }
                                if (linear_vel_feedback > 0.4) {
                                    emergency_brake_lin = 0.4;
                                }
                                else {
                                    emergency_brake_lin = linear_vel_feedback;
                                }
                                slow_brake = false;
                                slow_start = false;
                            }
                        }
                        if (lin <= 0) {
                            emergency_brake = false;
                        }
                    }
                    else if (obstacle_detected_in_1_5m) {
                        if (lin > 0.2) {
                            if (!slow_brake) {
                                slow_brake = true;
                                slow_start = false;
                                emergency_brake = false;
                                slow_brake_lin = linear_vel_feedback;
                            }
                        }
                    }
                    else {
                        if (slow_brake || emergency_brake) {
                            slow_brake = false;
                            emergency_brake = false;
                            slow_start = true;
                            slow_start_lin = linear_vel_feedback;
                        }
                    }



                    if (emergency_brake) {
                        stamp++;
                        printf("%d emergency brake\n", stamp);
                        emergency_brake_lin = emergency_brake_lin - 0.01;
                        if (emergency_brake_lin < 0) {
                            emergency_brake_lin = 0;
                            if (latch == 2) {
                                latch = 0;
                            }
                            if (latch == 4) {
                                latch = 3;
                            }
                        }
                        if (lin > emergency_brake_lin) {
                            lin = emergency_brake_lin;
                        }
                    }
                    else if (slow_brake) {
                        stamp++;
                        printf("%d slow brake ", stamp);
                        if (slow_brake_lin > 0.2) {
                            slow_brake_lin -= 0.0075;
                        }
                        else {
                            slow_brake_lin = 0.2;
                        }
                        printf("lin: %lf, slow_brake_lin: %lf\n", lin, slow_brake_lin);
                        if (lin > slow_brake_lin) {
                            lin = slow_brake_lin;
                        }
                    }
                    else if (slow_start) {
                        stamp++;
                        printf("%d slow start\n", stamp);

                        if (slow_start_lin > lin) {
                            slow_start = false;
                        }
                        else {
                            slow_start_lin = slow_start_lin + 0.0025;
                            lin = slow_start_lin;
                        }
                    }

                    printf("latch %d lin %.2lf ang %.2lf\n", latch, lin, ang);
                    segway_rmp.move(lin, ang);
                } catch (std::exception& e) {
                    std::string e_msg(e.what());
                    printf("Error commanding Segway RMP: %s", e_msg.c_str());
                    connected = false;
                }
                m_mutex.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        printf("Not connected to the SegwayRMP, will retry in 3 seconds...\n\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }

    return 0;
}
