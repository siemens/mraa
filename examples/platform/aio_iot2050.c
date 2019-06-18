/*
 * Author: Le Jin <le.jin@siemens.com>
 * Copyright (c) Siemens AG, 2019
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "mraa/aio.h"

#define NUM_CHANNELS (6)

volatile sig_atomic_t flag = 1;

void
sig_handler(int signum)
{
    if (signum == SIGINT) {
        fprintf(stdout, "Exiting...\n");
        flag = 0;
    }
}

int main(int argc, char *argv[])
{
    int i;
    int value;
    float float_value;
    mraa_result_t status = MRAA_SUCCESS;
    mraa_aio_context aio[NUM_CHANNELS];

    signal(SIGINT, sig_handler);
    /* init */
    mraa_init();
    memset(aio, 0, sizeof(mraa_aio_context) * NUM_CHANNELS);
    for(i=0; i<NUM_CHANNELS; i++) {
        aio[i] = mraa_aio_init(i);
        if (aio == NULL) {
            fprintf(stderr, "Failed to initialize AIO\n");
            mraa_deinit();
            return EXIT_FAILURE;
        }
    }

    while (flag) {
        for(i=0; i<NUM_CHANNELS; i++) {
            value = mraa_aio_read(aio[i]);
            float_value = mraa_aio_read_float(aio[i]);
            fprintf(stdout, "A%d: %d, %f\n", i, value, float_value);
        }
        fprintf(stdout, "--------------------\n");
        usleep(500);
    }

    /* close AIO */
    for(i=0; i<NUM_CHANNELS; i++) {
        status = mraa_aio_close(aio[i]);
        if (status != MRAA_SUCCESS) {
            goto err_exit;
        }
    }

    /* deinitialize mraa for the platform (not needed most of the times) */
    mraa_deinit();

    return EXIT_SUCCESS;

err_exit:
    mraa_result_print(status);

    /* deinitialize mraa for the platform (not needed most of the times) */
    mraa_deinit();

    return EXIT_FAILURE;
}
