/*

Copyright (c)  2021 Ruslan Migirov <trapi78@gmail.com>
Copyright (c)  2023 Franco Venturi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "ezusb.h"
#include <errno.h>
#include <getopt.h>
#include <libusb.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

unsigned int queuedepth = 16; // Number of requests to queue
unsigned int reqsize = 8;     // Request size in number of packets
unsigned int duration = 100;  // Duration of the test in seconds

const char *firmware = NULL;

static unsigned int ep = 1 | LIBUSB_ENDPOINT_IN;

static int interface_number = 0;
static struct libusb_device_handle *dev_handle = NULL;
unsigned int pktsize;
unsigned int success_count = 0;  // Number of successful transfers
unsigned int failure_count = 0;  // Number of failed transfers
unsigned int transfer_size = 0;  // Size of data transfers performed so far
unsigned int transfer_index = 0; // Write index into the transfer_size array
volatile bool stop_transfers = false; // Request to stop data transfers
volatile int xfers_in_progress = 0;

volatile int sleep_time = 0;

int verbose;
static int randomizer;
static int dither;
static int has_firmware;

static void start_adc(struct libusb_device_handle *dev_handle, unsigned int samplerate, unsigned int xtal, double correction);
static void rational_approximation(double value, uint32_t max_denominator, uint32_t *a, uint32_t *b, uint32_t *c);

static void transfer_callback(struct libusb_transfer *transfer) {
    int size = 0;
    int ret = 0;

    xfers_in_progress--;

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        failure_count++;
        fprintf(stderr, "Transfer callback status %s received %d \
	   bytes.\n",
                libusb_error_name(transfer->status), transfer->actual_length);
    } else {
        size = transfer->actual_length;
        success_count++;
        uint16_t *samples = (uint16_t *)transfer->buffer;
        if (randomizer) {
            for (int i = 0; i < size / 2; i++) {
                samples[i] ^= 0xfffe * (samples[i] & 1);
            }
        }
        ret = write(STDOUT_FILENO, transfer->buffer, transfer->actual_length);
        if (ret < 0) {
            fprintf(stderr, "Error writing to stdout: %s", strerror(errno));
        }
    }
    if (!stop_transfers) {
        if (libusb_submit_transfer(transfer) == 0)
            xfers_in_progress++;
    }
}

// Function to free data buffers and transfer structures
static void free_transfer_buffers(unsigned char **databuffers,
                                  struct libusb_transfer **transfers) {
    // Free up any allocated data buffers
    if (databuffers != NULL) {
        for (unsigned int i = 0; i < queuedepth; i++) {
            if (databuffers[i] != NULL) {
                free(databuffers[i]);
            }
            databuffers[i] = NULL;
        }
        free(databuffers);
    }

    // Free up any allocated transfer structures
    if (transfers != NULL) {
        for (unsigned int i = 0; i < queuedepth; i++) {
            if (transfers[i] != NULL) {
                libusb_free_transfer(transfers[i]);
            }
            transfers[i] = NULL;
        }
        free(transfers);
    }
}

static void sig_stop(int signum) {

    (void)signum;
    fprintf(stderr, "\nAbort. Stopping transfers\n");
    stop_transfers = true;
}
static void printhelp(void) {
    fprintf(stderr, " --verbose, -v      Verbose output\n");
    fprintf(stderr, " --firmware, -f     Firmware file\n");
    fprintf(stderr, " --dither, -d       Enable dithering\n");
    fprintf(stderr, " --rand, -r         Enable output randomization\n");
    fprintf(stderr, " --samplerate, -s   Sample Rate, default 32000000\n");
    fprintf(stderr, " --xtal, -x         Reference clock, default 27000000\n");
    fprintf(stderr, " --correction, -c   Clock correction in ppm, default 0\n");
    fprintf(stderr, " --gainmode, -m     Gain Mode low/high, default high\n");
    fprintf(stderr, " --att, -a          Attenuation, default 0\n");
    fprintf(stderr, " --gain, -g         Gain value, default 3\n");
    fprintf(stderr, " --queuedepth, -q   Queue depth, default 16\n");
    fprintf(stderr,
            " --reqsize, -p      Packets per transfer request, default 8\n");
    fprintf(stderr, " --help, -h         Print this help\n");
}
int main(int argc, char **argv) {

    unsigned int samplerate = 32000000;
    unsigned int xtal = 27000000;
    double correction = 0.0;
    unsigned int gain = 0x83;
    unsigned int att = 0;
    int c;
    while (1) {
        static struct option long_options[] = {
            {"verbose", optional_argument, &verbose, 1},
            {"firmware", required_argument, &has_firmware, 'f'},
            {"dither", no_argument, &dither, 'd'},
            {"rand", no_argument, &randomizer, 'r'},
            {"samplerate", required_argument, 0, 's'},
            {"xtal", required_argument, 0, 'x'},
            {"correction", required_argument, 0, 'c'},
            {"gainmode", required_argument, 0, 'm'},
            {"gain", required_argument, 0, 'g'},
            {"att", required_argument, 0, 'a'},
            {"queuedepth", required_argument, 0, 'q'},
            {"reqsize", required_argument, 0, 'p'},
            {"help", no_argument, 0, 'h'},
            {0, 0, 0, 0}};

        int option_index = 0;
        int gainvalue = 0;

        c = getopt_long(argc, argv, "f:drs:hm:g:a:q:p:", long_options,
                        &option_index);

        if (c == -1)
            break;

        if (c == 0) {
            if (long_options[option_index].flag != 0)
                break;
            c = long_options[option_index].val;
        }
        switch (c) {

        case 'f':
            firmware = optarg;
            break;

        case 'r':
            randomizer = 1;
            break;

        case 'd':
            dither = 1;
            break;

        case 'v':
            if (optarg) {
                verbose = strtoul(optarg, NULL, 10);
            } else {
                verbose = 1;
            }
            break;

        case 's':
            samplerate = strtoul(optarg, NULL, 10);
            if (samplerate < 1000000) {
                fprintf(stderr, "Invalid samplerate %d\n", samplerate);
                printhelp();
                return 0;
            }
            break;

        case 'x':
            xtal = strtoul(optarg, NULL, 10);
            if (xtal < 1000000) {
                fprintf(stderr, "Invalid reference clock %d\n", xtal);
                printhelp();
                return 0;
            }
            break;

        case 'c':
            correction = strtod(optarg, NULL);
            if (fabs(correction) > 10000.0) {
                fprintf(stderr, "Invalid clock correction %lg ppm\n", correction);
                printhelp();
                return 0;
            }
            break;

        case 'm':
            if (strcmp(optarg, "high") == 0) {
                gain |= 0x80;
            } else if (strcmp(optarg, "low") == 0) {
                gain &= ~0x80;
            } else {
                fprintf(stderr, "Invalid gain mode %s\n", optarg);
                printhelp();
                return 0;
            }
            break;

        case 'g':
            gainvalue = strtol(optarg, NULL, 10);
            if (gainvalue < 0 || gainvalue > 127) {
                fprintf(stderr, "Invalid gain value %d\n", gainvalue);
                printhelp();
                return 0;
            }
            gain &= ~0x7f;
            gain |= gainvalue;
            break;

        case 'a':
            att = strtol(optarg, NULL, 10);
            if (att < 0 || att > 63) {
                fprintf(stderr, "Invalid attenuation value %d\n", att);
                printhelp();
                return 0;
            }
            break;
        case 'q':
            queuedepth = strtol(optarg, NULL, 10);
            if (queuedepth < 1 || queuedepth > 64) {
                fprintf(stderr, "Invalid queue depth %d\n", queuedepth);
                printhelp();
                return 0;
            }
            break;
        case 'p':
            reqsize = strtol(optarg, NULL, 10);
            if (reqsize < 1 || reqsize > 64) {
                fprintf(stderr, "Invalid request size %d\n", reqsize);
                printhelp();
                return 0;
            }
            break;
        case 'h':
        case '?':
        default:
            /* getopt_long already printed an error message. */
            printhelp();
            return 0;
        }
    }

    fprintf(stderr, "Firmware: %s\n", firmware);
    fprintf(stderr, "Sample Rate: %u\n", samplerate);
    fprintf(stderr, "Reference clock: %u\n", xtal);
    fprintf(stderr, "Clock correction: %lg\n", correction);
    fprintf(stderr, "Output Randomizer %s, Dither: %s\n",
            randomizer ? "On" : "Off", dither ? "On" : "Off");
    fprintf(stderr, "Gain Mode: %s, Gain: %u, Att: %u\n",
            (gain & 0x80) ? "High" : "Low", gain & 0x7f, att);
    /* code */
    struct libusb_device_descriptor desc;
    struct libusb_device *dev;
    struct libusb_endpoint_descriptor const *endpointDesc;
    struct libusb_ss_endpoint_companion_descriptor *ep_comp;
    struct libusb_config_descriptor *config;
    struct libusb_interface_descriptor const *interfaceDesc;
    int ret;
    int rStatus;

    uint16_t vendor_id;  //= 0x04b4;
    uint16_t product_id; // = 0x00f1;

    struct sigaction sigact;

    sigact.sa_handler = sig_stop;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    (void)sigaction(SIGINT, &sigact, NULL);
    (void)sigaction(SIGTERM, &sigact, NULL);

    struct libusb_transfer **transfers = NULL; // List of transfer structures.
    unsigned char **databuffers = NULL;        // List of data buffers.

    ret = libusb_init(NULL);
    if (ret != 0) {
        fprintf(stderr, "Error initializing libusb: %s\n",
                libusb_error_name(ret));
        exit(1);
    }

    if (firmware) { // there is argument with image file
        vendor_id = 0x04b4;
        product_id = 0x00f3;
        // no firmware. upload the firmware
        dev_handle =
            libusb_open_device_with_vid_pid(NULL, vendor_id, product_id);
        if (!dev_handle) {
            goto has_firmware;
        }

        dev = libusb_get_device(dev_handle);

        if (ezusb_load_ram(dev_handle, firmware, FX_TYPE_FX3, IMG_TYPE_IMG,
                           1) == 0) {
            fprintf(stderr, "Firmware updated\n");
        } else {
            fprintf(stderr,
                    "Firmware upload failed for "
                    "device %d.%d (logical).\n",
                    libusb_get_bus_number(dev), libusb_get_device_address(dev));
        }

        sleep(2);
    }

has_firmware:

    vendor_id = 0x04b4;
    product_id = 0x00f1;
    dev_handle = libusb_open_device_with_vid_pid(NULL, vendor_id, product_id);
    if (!dev_handle) {
        fprintf(stderr,
                "Error or device could not be found, try loading firmware\n");
        goto close;
    }

    ret = libusb_kernel_driver_active(dev_handle, 0);
    if (ret != 0) {
        fprintf(stderr,
                "Kernel driver active. Trying to detach kernel driver\n");
        ret = libusb_detach_kernel_driver(dev_handle, 0);
        if (ret != 0) {
            fprintf(stderr,
                    "Could not detach kernel driver from an interface\n");
            goto close;
        }
    }

    dev = libusb_get_device(dev_handle);

    libusb_get_config_descriptor(dev, 0, &config);

    ret = libusb_claim_interface(dev_handle, interface_number);
    if (ret != 0) {
        fprintf(stderr, "Error claiming interface\n");
        goto end;
    }

    fprintf(stderr, "Successfully claimed interface\n");

    interfaceDesc = &(config->interface[0].altsetting[0]);

    endpointDesc = &interfaceDesc->endpoint[0];

    libusb_get_device_descriptor(dev, &desc);

    libusb_get_ss_endpoint_companion_descriptor(NULL, endpointDesc, &ep_comp);

    pktsize = endpointDesc->wMaxPacketSize * (ep_comp->bMaxBurst + 1);

    libusb_free_ss_endpoint_companion_descriptor(ep_comp);

    databuffers = (unsigned char **)calloc(queuedepth, sizeof(u_char *));
    if (databuffers == NULL) {
        fprintf(stderr, "Could not allocate memory for data buffers\n");
        goto end;
    }

    transfers = (struct libusb_transfer **)calloc(
        queuedepth, sizeof(struct libusb_transfer *));
    if (transfers == NULL) {
        fprintf(stderr, "Could not allocate memory for transfer structures\n");
        goto free_transfer_buf;
    }

    fprintf(stderr, "Queue depth: %d, Request size: %d\n", queuedepth,
            reqsize * pktsize);

    for (unsigned int i = 0; i < queuedepth; i++) {
        databuffers[i] = (unsigned char *)malloc(reqsize * pktsize);
        transfers[i] = libusb_alloc_transfer(0);
        if ((databuffers[i] == NULL) || (transfers[i] == NULL)) {
            goto free_transfer_buf;
        }
    }

    for (unsigned int i = 0; i < queuedepth; i++) {
        libusb_fill_bulk_transfer(transfers[i], dev_handle, ep, databuffers[i],
                                  reqsize * pktsize, transfer_callback,
                                  (void *)&pktsize, 0);
        rStatus = libusb_submit_transfer(transfers[i]);
        if (rStatus == 0)
            xfers_in_progress++;
    }

    /******/
    uint32_t gpio = 0;
    if (dither) {
        gpio |= DITH;
    }
    if (randomizer) {
        gpio |= RANDO;
    }

    usleep(5000);
    command_send(dev_handle, GPIOFX3, gpio);
    usleep(5000);
    argument_send(dev_handle, DAT31_ATT, att);
    usleep(5000);
    argument_send(dev_handle, AD8340_VGA, gain);
    usleep(5000);
    start_adc(dev_handle, samplerate, xtal, correction);
    usleep(5000);
    command_send(dev_handle, STARTFX3, 0);
    usleep(5000);
    command_send(dev_handle, TUNERSTDBY, 0);
    /*******/

    do {
        libusb_handle_events(NULL);

    } while (stop_transfers != true);

    fprintf(stderr, "Test complete. Stopping transfers\n");
    stop_transfers = true;

    while (xfers_in_progress != 0) {
        fprintf(stderr, "%d transfers are pending\n", xfers_in_progress);
        libusb_handle_events(NULL);
        usleep(100000);
    }

    fprintf(stderr, "Transfers completed\n");
    command_send(dev_handle, STOPFX3, 0);

free_transfer_buf:
    free_transfer_buffers(databuffers, transfers);
end:
    if (dev_handle) {
        libusb_release_interface(dev_handle, interface_number);
    }

    if (config) {
        libusb_free_config_descriptor(config);
    }
close:
    if (dev_handle) {
        libusb_close(dev_handle);
    }
    libusb_exit(NULL);

    return 0;
}

// SiLabs Application Note AN619 - Manually Generating an Si5351 Register Map (https://www.silabs.com/documents/public/application-notes/AN619.pdf)
static void start_adc(struct libusb_device_handle *dev_handle, unsigned int samplerate, unsigned int xtal, double correction) {
    if (samplerate == 0) {
        /* power off clock 0 */
        control_send_byte(dev_handle, I2CWFX3, SI5351_ADDR, SI5351_REGISTER_CLK_BASE+0, SI5351_VALUE_CLK_PDN);
        return;
    }

    /* if the requested sample rate is below 1MHz, use an R divider */
    double r_samplerate = samplerate;
    uint8_t rdiv = 0;
    while (r_samplerate < 1e6 && rdiv <= 7) {
        r_samplerate *= 2.0;
        rdiv += 1;
    }
    if (r_samplerate < 1e6) {
        fprintf(stderr, "ERROR - requested sample rate is too low: %d\n", samplerate);
        return;
    }

    /* choose an even integer for the output MS */
    uint32_t output_ms = ((uint32_t)(SI5351_MAX_VCO_FREQ / r_samplerate));
    output_ms -= output_ms % 2;
    if (output_ms < 4 || output_ms > 900) {
        fprintf(stderr, "ERROR - invalid output MS: %d  (samplerate=%d)\n", output_ms, samplerate);
        return;
    }
    double vco_frequency = r_samplerate * output_ms;

    /* feedback MS */
    double xtal_corrected = xtal * (1.0 + 1e-6 * correction);
    double feedback_ms = vco_frequency / xtal_corrected;
    /* find a good rational approximation for feedback_ms */
    uint32_t a;
    uint32_t b;
    uint32_t c;
    rational_approximation(feedback_ms, SI5351_MAX_DENOMINATOR, &a, &b, &c);

    fprintf(stderr, "actual PLL frequency: %d * (1.0 + %lg * 1e-6) * (%d + %d / %d)\n", xtal, correction, a, b, c);

    double actual_ratio = a + (double)b / (double)c;
    double actual_pll_frequency = xtal_corrected * actual_ratio;
    fprintf(stderr, "actual PLL frequency: %lf\n", actual_pll_frequency);

    double actual_samplerate = actual_pll_frequency / output_ms / (1 << rdiv);
    fprintf(stderr, "actual sample rate: %lf / %d = %lf\n", actual_pll_frequency, output_ms * (1 << rdiv), actual_samplerate);
    fprintf(stderr, "sample rate difference: %lf\n", actual_samplerate - samplerate);

    /* configure clock input and PLL */
    uint32_t const b_over_c = 128 * b / c;
    uint32_t const msn_p1 = 128 * a + b_over_c - 512;
    uint32_t const msn_p2 = 128 * b    - c * b_over_c;
    uint32_t const msn_p3 = c;

    uint8_t data_clkin[] = {
        (msn_p3 & 0x0000ff00) >>  8,
        (msn_p3 & 0x000000ff) >>  0,
        (msn_p1 & 0x00030000) >> 16,
        (msn_p1 & 0x0000ff00) >>  8,
        (msn_p1 & 0x000000ff) >>  0,
        (msn_p3 & 0x000f0000) >> 12 | (msn_p2 & 0x000f0000) >> 16,
        (msn_p2 & 0x0000ff00) >>  8,
        (msn_p2 & 0x000000ff) >>  0
    };

    control_send(dev_handle, I2CWFX3, SI5351_ADDR, SI5351_REGISTER_MSNA_BASE, data_clkin, sizeof(data_clkin));

    /* configure clock output */
    /* since the output divider is an even integer a = output_ms, b = 0, c = 1 */
    uint32_t const ms_p1 = 128 * output_ms - 512;
    uint32_t const ms_p2 = 0;
    uint32_t const ms_p3 = 1;

    uint8_t data_clkout[] = {
        (ms_p3 & 0x0000ff00) >>  8,
        (ms_p3 & 0x000000ff) >>  0,
        rdiv << 5 | (ms_p1 & 0x00030000) >> 16,
        (ms_p1 & 0x0000ff00) >>  8,
        (ms_p1 & 0x000000ff) >>  0,
        (ms_p3 & 0x000f0000) >> 12 | (ms_p2 & 0x000f0000) >> 16,
        (ms_p2 & 0x0000ff00) >>  8,
        (ms_p2 & 0x000000ff) >>  0
    };

    control_send(dev_handle, I2CWFX3, SI5351_ADDR, SI5351_REGISTER_MS0_BASE, data_clkout, sizeof(data_clkout));

    /* start clock */
    control_send_byte(dev_handle, I2CWFX3, SI5351_ADDR, SI5351_REGISTER_PLL_RESET, SI5351_VALUE_PLLA_RESET);
    /* power on clock 0 */
    uint8_t const clock_control = SI5351_VALUE_MS_INT | SI5351_VALUE_CLK_SRC_MS | SI5351_VALUE_CLK_DRV_8MA | SI5351_VALUE_MS_SRC_PLLA;
    control_send_byte(dev_handle, I2CWFX3, SI5351_ADDR, SI5351_REGISTER_CLK_BASE+0, clock_control);

    usleep(1000000); // 1s - see SDDC_FX3 firmware
    return;
}

/* best rational approximation:
 *
 *     value ~= a + b/c     (where c <= max_denominator)
 *
 * References:
 * - https://en.wikipedia.org/wiki/Continued_fraction#Best_rational_approximations
 */
static void rational_approximation(double value, uint32_t max_denominator, uint32_t *a, uint32_t *b, uint32_t *c) {
    const double epsilon = 1e-5;

    double af;
    double f0 = modf(value, &af);
    *a = (uint32_t) af;
    *b = 0;
    *c = 1;
    double f = f0;
    double delta = f0;
    /* we need to take into account that the fractional part has a_0 = 0 */
    uint32_t h[] = {1, 0};
    uint32_t k[] = {0, 1};
    for(int i = 0; i < 100; ++i){
        if(f <= epsilon){
            break;
        }
        double anf;
        f = modf(1.0 / f,&anf);
        uint32_t an = (uint32_t) anf;
        for(uint32_t m = (an + 1) / 2; m <= an; ++m){
            uint32_t hm = m * h[1] + h[0];
            uint32_t km = m * k[1] + k[0];
            if(km > max_denominator){
                break;
            }
            double d = fabs((double) hm / (double) km - f0);
            if(d < delta){
                delta = d;
                *b = hm;
                *c = km;
            }
        }
        uint32_t hn = an * h[1] + h[0];
        uint32_t kn = an * k[1] + k[0];
        h[0] = h[1]; h[1] = hn;
        k[0] = k[1]; k[1] = kn;
    }
    return;
}
