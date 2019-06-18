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
#include "mraa/spi.h"

#define SPI_BUS (0)
#define SPI_FREQ 400000
#define BUF_SIZE (64)

int main(int argc, char *argv[])
{
    mraa_result_t status = MRAA_SUCCESS;
    mraa_spi_context spi;
    uint8_t tx_buf[BUF_SIZE];
    uint8_t rx_buf[BUF_SIZE];
    int i;

    /* initialize mraa */
    mraa_init();
    /* initialize SPI bus */
    spi = mraa_spi_init(SPI_BUS);
    if (spi == NULL) {
        fprintf(stderr, "Failed to initialize SPI\n");
        mraa_deinit();
        return EXIT_FAILURE;
    }
    status = mraa_spi_frequency(spi, SPI_FREQ);
    if (status != MRAA_SUCCESS)
        goto error;
    fprintf(stderr, "Start SPI transfer\n");
    for(i=0; i<BUF_SIZE; i++) {
        tx_buf[i] = i;
        rx_buf[i] = 0;
    }
    status = mraa_spi_transfer_buf(spi, tx_buf, rx_buf, BUF_SIZE);
    if(status != MRAA_SUCCESS)
        goto error;
    fprintf(stderr, "SPI Send Data:\n");
    for(i=0; i<BUF_SIZE; i++) {
        fprintf(stderr, "%02X ", tx_buf[i]);
        if(!((i+1) % 8))
            fprintf(stderr, "\n");
    }
    fprintf(stderr, "SPI Recevied Data:\n");
    for(i=0; i<BUF_SIZE; i++) {
        fprintf(stderr, "%02X ", rx_buf[i]);
        if(!((i+1) % 8))
            fprintf(stderr, "\n");
    }
    fprintf(stderr, "\n");
    return EXIT_SUCCESS;
error:
    mraa_result_print(status);
    /* stop spi */
    mraa_spi_stop(spi);
    /* deinitialize mraa */
    mraa_deinit();
    return EXIT_FAILURE;
}