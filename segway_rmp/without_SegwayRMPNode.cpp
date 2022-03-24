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
    char obstacle_detected_in_1m = 0;
    char obstacle_detected_in_2m = 0;
    char obstacle_detected_in_3m = 0;
};


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
double lin = 0, momo_lin = 0, joy_lin = 0;
double ang = 0, momo_ang = 0, joy_ang = 0;
double before_target_linear_vel = 0;
double linear_vel_feedback = 0;
int zero_judge = 0;
double offset = 0.04;
double gain = 0.4;
int offset_gain_none_latch = 1;
bool obstacle_detected_in_1m = false;
bool obstacle_detected_in_2m = false;
bool obstacle_detected_in_3m = false;
bool motors_enabled = false;
bool recover_motors_enabled = false;
bool reset_odometry = false;
int d1_count = 0;
bool ofs_closed = true;
int log_margin_count = 0;
int fd_write;
std::chrono::system_clock::time_point begin_time_point;
std::chrono::system_clock::time_point jyja_arrival_time;
bool connected = false;

BanAccel* ba;

std::ofstream* ofs;

std::mutex m_mutex;



void handleStatus(segwayrmp::SegwayStatus::Ptr ss_ptr) {
    if (!connected) {
        return;
    }
    segwayrmp::SegwayStatus &ss = *(ss_ptr);
    linear_vel_feedback = (ss.left_wheel_speed + ss.right_wheel_speed) / 2.0;
    int vel = (int32_t)(linear_vel_feedback * 10000.0);
    uint8_t hh = (uint8_t)((uint32_t)(vel & 0xff000000) >> 24);
    uint8_t h = (uint8_t)((uint32_t)(vel & 0x00ff0000) >> 16);
    uint8_t l = (uint8_t)((uint32_t)(vel & 0x0000ff00) >> 8);
    uint8_t ll = (uint8_t)(vel & 0x000000ff);
    int end_time_point = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - begin_time_point).count();
    uint8_t hht = (uint8_t)((uint32_t)(end_time_point & 0xff000000) >> 24);
    uint8_t ht = (uint8_t)((uint32_t)(end_time_point & 0x00ff0000) >> 16);
    uint8_t lt = (uint8_t)((uint32_t)(end_time_point & 0x0000ff00) >> 8);
    uint8_t llt = (uint8_t)(end_time_point & 0x000000ff);
    uint8_t buf[9] = {hht, ht, lt, llt, hh, h, l, ll, '\n'};
    write(fd_write, &buf, 9);

    if (latch == 2 && !ofs_closed) {
        *(ofs) << end_time_point/1000.0 << ' ' << linear_vel_feedback << '\n';
    }
    else if (latch == 3 && !ofs_closed && log_margin_count < 200) {
        *(ofs) << end_time_point/1000.0 << ' ' << linear_vel_feedback << '\n';
        log_margin_count++;
    }
    else if (latch == 3 && !ofs_closed && log_margin_count >= 200) {
        ofs->close();
        ofs_closed = true;
        log_margin_count = 0;
    }

    printf("linear_vel_feedback: %lf\n", linear_vel_feedback);

    if (!motors_enabled && (bool)(ss.motor_status)) {
        recover_motors_enabled = true;
    }
    motors_enabled = (bool)(ss.motor_status);
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
                    joy_lin = 0;
                    joy_ang = 0;
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
            }
            if ((int)joy_button.at(10) && (int)joy_button.at(12)) { //L1 and △
                latch = 3;
                ang = 0;
                lin = 0;
            }
            if ((int)joy_button.at(14)) { //x
                latch = 0;
                ang = 0;
                lin = 0;
                joy_ang = 0;
                joy_lin = 0;
            }

            if (latch == 1) {
                // this->ang = -50.0*joy_axis.at(0)/32767.0;
                // if (joy_axis.at(3) < 0) {
                //     joy_lin = -1.5*joy_axis.at(3)/32767.0 * fabs(joy_axis.at(3)/32767.0);
                // }
                // else {
                //     joy_lin = -0.5*joy_axis.at(3)/32767.0 * fabs(joy_axis.at(3)/32767.0);
                // }
                joy_ang = -50.0*joy_axis.at(0)/32767.0 * fabs(joy_axis.at(0)/32767.0);
                // this->joy_lin = -1.5*joy_axis.at(3)/32767.0 * fabs(joy_axis.at(3)/32767.0);
                // my_queue.enqueue(-1.5*joy_axis.at(3)/32767.0 * fabs(joy_axis.at(3)/32767.0));
                // this->lin = my_queue.mean();

                double A = 1.0; // 指令値 (m/s) の最大値
                double k = 0.1;
                double x = -joy_axis.at(3)/32767.0; // joystick の入力値 -1 ~ 1

                if (x > 0) {
                    joy_lin = A*((1 - k)*x + k)*x;  // joy_lin は指令値 (m/s)
                }
                else {
                    joy_lin = - A*((1 - k)*x + k)*x; // joy_lin は指令値 (m/s)
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
        printf("read %d byte: %02x %02x %02x %02x\n", read_size, buf_ptr[0], buf_ptr[1], buf_ptr[2], buf_ptr[3]);
        // printf("read %d byte: %08x\n", read_size, buf_ptr[0]);
        if (read_size == 4) {
            if (buf_ptr[0] == 0x43) {
                // this->ang = 50*(int8_t)((buf_ptr[0] & 0x0000ff00) >> 8) /127.0;
                // this->lin = 1.0*(int8_t)(buf_ptr[0] & 0x000000ff)/127.0;
                if (latch == 3) {
                    momo_ang = 50*(int8_t)buf_ptr[2] /127.0 * std::fabs((int8_t)buf_ptr[2] /127.0);
                    if ((int8_t)buf_ptr[3] > 0) {
                        momo_lin = 1.5*(int8_t)buf_ptr[3] /127.0;
                    }
                    else {
                        momo_lin = 0.5*(int8_t)buf_ptr[3] /127.0;
                    }
                    // printf("%lf, %lf\n", ang, lin);
                }
                jyja_arrival_time = std::chrono::system_clock::now();
            }

            // else if (buf_ptr[0] == 0xab) {
            //     if (latch == 3) {
            //         latch = 2;
            //         begin_time_point = std::chrono::system_clock::now();
            //         ba->setup((int8_t)buf_ptr[1]/2.0, (int8_t)buf_ptr[2]/20.0, (int8_t)buf_ptr[3]/20.0, 1);
            //     }
            // }
            else if ((buf_ptr[0] & 0xf0) == 0xa0) {
                if (latch == 3) {
                    latch = 2;
                    log_margin_count = 0;
                    begin_time_point = std::chrono::system_clock::now();
                    ba->setup((int8_t)buf_ptr[1]/2.0, (int8_t)buf_ptr[2]/20.0, (int8_t)buf_ptr[3]/20.0, 0, (int8_t)(buf_ptr[0] & 0x0f));
                    std::stringstream ss;
                    // ss << "./log/" << std::setfill('0') << std::setw(2) << file_counta << std::setfill(' ') << "actual_velocity_a_" << (int8_t)buf_ptr[2]/20.0 << "_v_" << (int8_t)buf_ptr[3]/20.0 << "_T2_" << (int8_t)buf_ptr[1]/2.0 << ".out";
                    time_t t = time(NULL);
                    std::string str = ctime(&t);
                    ss << "./log/" << str.substr(0, str.size()-1) << "actual_velocity_a_" << (int8_t)buf_ptr[2]/20.0 << "_v_" << (int8_t)buf_ptr[3]/20.0 << "_T2_" << (int8_t)buf_ptr[1]/2.0 << "_count_" << (int)(buf_ptr[0] & 0x0f);
                    switch (offset_gain_none_latch) {
                        case 1: ss << "_offset_" << offset << ".out"; break;
                        case 2: ss << "_gain_" << gain << ".out"; break;
                        case 3: ss << "_none.out"; break;
                    }
                    ofs = new std::ofstream(ss.str());
                    ofs_closed = false;
                    *(ofs) << "#accel(m/s^2) " << (int8_t)buf_ptr[2]/20.0 << " max_vel(m/s) " << (int8_t)buf_ptr[3]/20.0 << " max_vel_time(s) " << (int8_t)buf_ptr[1]/2.0;
                    switch (offset_gain_none_latch) {
                        case 1: *(ofs) << " offset " << offset; break;
                        case 2: *(ofs) << " gain " << gain; break;
                        case 3: *(ofs) << " none "; break;
                    }
                    *(ofs) << " count " << (int)(buf_ptr[0] & 0x0f) << '\n';
                    *(ofs) << "#時刻(s) 実際の速度(m/s)\n";
                    ofs_closed = false;
                }
            }

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
                latch = 0;
                lin = 0;
                ang = 0;
                // std::cout << "segway_rmp_node を終了\n";
                // std_msgs::String msg;
                // msg.data = "quit";
                // halt_pub.publish(msg);
            }
        }
    }
    return;
}


int main(int argc, char **argv) {
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    addr.sin_port = htons(4001);

    bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr));

    fd_write = open(SERIAL_PATH, O_WRONLY); // SERIAL_PATH は serialPathConfig.h.in にて定義されている。


    segwayrmp::SegwayRMP segway_rmp(segwayrmp::serial, segwayrmp::rmp200);
    segway_rmp.configureSerial(std::string("/dev/ttyUSB0"));
    segway_rmp.setStatusCallback(handleStatus);
    segway_rmp.setLogMsgCallback("debug", handleDebugMessages);
    segway_rmp.setLogMsgCallback("info", handleInfoMessages);
    segway_rmp.setLogMsgCallback("error", handleErrorMessages);

    begin_time_point = std::chrono::system_clock::now();

    ba = new BanAccel(&latch);

    std::thread th_momo_serial_read(momo_serial_read);
    std::thread th_joy_read(joy_read);
    // std::thread th_udp_read(udp_read);

    // boost::thread th_hoge(&SegwayRMPNode::hoge, this);
    // this->spin();

    connected = false;
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

                Lavel la;
                if (latch == 0) {
                    lin = 0;
                    ang = 0;
                    joy_ang = 0;
                    joy_lin = 0;
                }
                else if (latch == 1) {
                    ang = joy_ang;
                    lin = joy_lin;
                }
                else if (latch == 2) {
                    la = ba->controller();
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
                        momo_ang = 0;
                        momo_lin = 0;
                    }
                    else {
                        ang = momo_ang;
                        lin = lin*0.97 + momo_lin*0.03;
                    }
                }

                try {
                    if (obstacle_detected_in_2m && lin > 0.4) {
                        lin = 0.4;
                    }
                    if (obstacle_detected_in_1m && lin > 0) {
                        lin = 0;
                        if (latch == 2) {
                            latch = 0;
                        }
                    }
                    // if (lin < -0.5) {
                    //     lin = -0.5;
                    // }
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
