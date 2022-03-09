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
#include <unistd.h>

#include <sys/fcntl.h>
#include <iomanip>
#include "serialPathConfig.h" // SERIAL_PATH を定義

#include <time.h>

#include <vector>
#include <linux/joystick.h>
#define JOY_DEV "/dev/input/js0"


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
    }
    return;
}


void hoge() {
    int fd_write = open(SERIAL_PATH, O_WRONLY);
    double vel = -1;
    std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
    while (true) {
        uint8_t h = (uint8_t)((uint16_t)((uint16_t)(vel * 10000) & 0xff00) >> 8);
        uint8_t l = (uint8_t)((uint16_t)(vel * 10000) & 0x00ff);
        uint8_t buf[3] = {h, l, '\n'};
        write(fd_write, &buf, 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
        // printf("%lf\n", (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0);
        vel = vel + 0.001;
    }
}

int latch;
double lin, ang;

void joy_read() {
    int joy_fd = -1;


    while (true) {

        while (true) {
            if (joy_fd < 0) {
                latch = 3;
                lin = 0;
                ang = 0;
                printf("[%d] connecting to joystick ...\n", joy_fd);
                joy_fd = open(JOY_DEV, O_RDONLY);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
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


            if ((int)joy_button.at(0)) {
                latch = 0;
            }
            if ((int)joy_button.at(1)) {
                if (latch == 0) {
                    latch = 3;
                }
                else if (latch == 2) {
                }
                ang = 0;
                lin = 0;
            }

            if (latch == 0) {
                ang = -50.0*joy_axis.at(0)/32767.0;
                lin = -2.0*joy_axis.at(3)/32767.0;
            }
        }
        close(joy_fd);
        joy_fd = -1;
    }
    return;
}

int main(int argc, char **argv) {
    std::thread th_momo_serial_read(momo_serial_read);
    std::thread th_hoge(hoge);
    std::thread th_joy_read(joy_read);

    th_momo_serial_read.join();
    th_hoge.join();
    th_joy_read.join();

    return 0;
}
