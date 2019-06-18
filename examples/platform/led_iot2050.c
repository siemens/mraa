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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "mraa/led.h"

#define LED_TRIGGER "timer"
#define NUM_LEDS (4)

int main(void)
{
    mraa_result_t status = MRAA_SUCCESS;
    mraa_led_context led[NUM_LEDS];
    int i;

    /* initialize mraa for the platform (not needed most of the time) */
    mraa_init();

    //! [Interesting]
    /* initialize LED */
    for(i=0; i<NUM_LEDS; i++) {
        led[i] = mraa_led_init(i);
        if (led == NULL) {
            fprintf(stderr, "Failed to initialize LED\n");
            mraa_deinit();
            return EXIT_FAILURE;
        }
    }

    /* set LED trigger to heartbeat */
    for(i=0; i<NUM_LEDS; i++) {
        status = mraa_led_set_trigger(led[i], LED_TRIGGER);
        if (status != MRAA_SUCCESS) {
            fprintf(stderr, "unable to set LED trigger to: timer\n");
            goto err_exit;
        }
    }

    fprintf(stdout, "LED trigger set to: timer\n");

    /* close LED */
    for(i=0; i<NUM_LEDS; i++) {
        mraa_led_close(led[i]);
    }

    //! [Interesting]
    /* deinitialize mraa for the platform (not needed most of the times) */
    mraa_deinit();

    return EXIT_SUCCESS;

err_exit:
    mraa_result_print(status);

    /* deinitialize mraa for the platform (not needed most of the times) */
    mraa_deinit();

    return EXIT_FAILURE;
}
