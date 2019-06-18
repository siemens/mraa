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
#include "mraa/pwm.h"

#define NUM_CHANNELS        (6)
#define PWM_FREQ_US         (1)
#define PWM_DUTY_PERCENT    (0.5)
#define PWM_START_PIN       (4)

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
    mraa_result_t status = MRAA_SUCCESS;
    mraa_pwm_context pwm[NUM_CHANNELS];

    signal(SIGINT, sig_handler);
    /* init */
    mraa_init();
    memset(pwm, 0, sizeof(mraa_pwm_context) * NUM_CHANNELS);
    for(i=0; i<NUM_CHANNELS; i++) {
        pwm[i] = mraa_pwm_init(PWM_START_PIN + i);
        if (pwm[i] == NULL) {
            fprintf(stderr, "Failed to initialize PWM%d\n", i);
            mraa_deinit();
            return EXIT_FAILURE;
        }
    }
    for(i=0; i<NUM_CHANNELS; i++) {
        /* set PWM period */
        status = mraa_pwm_period_us(pwm[i], PWM_FREQ_US);
        if (status != MRAA_SUCCESS) {
            goto err_exit;
        }
        /* write PWM duty cyle */
        status = mraa_pwm_write(pwm[i], PWM_DUTY_PERCENT);
        if (status != MRAA_SUCCESS) {
            goto err_exit;
        }
        /* enable PWM */
        status = mraa_pwm_enable(pwm[i], 1);
        if (status != MRAA_SUCCESS) {
            goto err_exit;
        }
    }
    while (flag) {
    }
    /* close PWM */
    for(i=0; i<NUM_CHANNELS; i++) {
        mraa_pwm_close(pwm[i]);
    }
    /* deinitialize mraa for the platform (not needed most of the times) */
    mraa_deinit();

    return EXIT_SUCCESS;

err_exit:
    mraa_result_print(status);

    /* close PWM */
    for(i=0; i<NUM_CHANNELS; i++) {
        mraa_pwm_close(pwm[i]);
    }

    /* deinitialize mraa for the platform (not needed most of the times) */
    mraa_deinit();

    return EXIT_FAILURE;
}
