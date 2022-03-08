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
    while (true) {
        uint8_t h = (uint8_t)((uint16_t)((uint16_t)(vel * 10000) & 0xff00) >> 8);
        uint8_t l = (uint8_t)((uint16_t)(vel * 10000) & 0x00ff);
        uint8_t buf[3] = {h, l, '\n'};
        write(fd_write, &buf, 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        printf("%lf\n", clock());
        vel = vel + 0.001;
    }
}

int main(int argc, char **argv) {
    std::thread th_momo_serial_read(momo_serial_read);
    std::thread th_hoge(hoge);

    th_momo_serial_read.join();
    th_hoge.join();

    return 0;
}
