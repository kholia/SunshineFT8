#ifndef MY_HEADER_FILE_H
#define MY_HEADER_FILE_H

#include <vector>
#include <map>
#include <string>

using namespace std;

enum si5351_clock {SI5351_CLK0, SI5351_CLK1, SI5351_CLK2};

void si_output_enable(enum si5351_clock clk, uint8_t enable);

// Relay pins
#define relay_1 19
#define relay_2 20
#define PTT_PIN 21
#define LED_PIN 25

struct decoder_results {
    char call[32];
    char loc[7];
    int32_t freq;
    int32_t snr;
};

#define MAX_REPORTS_PER_PACKET 64

void pre_transmit();

#endif
