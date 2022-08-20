#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "ft8/pack.h"
#include "ft8/encode.h"

#include "ft8/decode_ft8.h"
#include "ft8/gen_ft8.h"

#include "util/tx_ft8.h"
#include "util/rx_ft8.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"

#include "hardware/adc.h"
#include "hardware/dma.h"

#include "pico/multicore.h"
#include "hardware/irq.h"

#include "common.h"

// Uncomment if overclocking > 290MHz
#include "hardware/vreg.h"

// RTC was too slow. So I am using hardware_timer's time_us_64!
#include "peripheral_util/pico_si5351/si5351.h"

#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

// Spotting code
#include "pskreporter.h"

// RTC
#include "hardware/rtc.h"
#include "pico/stdlib.h"
#include "pico/util/datetime.h"

#include <map>
bool have_time = false;
time_t epoch;

typedef struct NTP_T_ {
  ip_addr_t ntp_server_address;
  bool dns_request_sent;
  struct udp_pcb *ntp_pcb;
  absolute_time_t ntp_test_time;
  alarm_id_t ntp_resend_alarm;
} NTP_T;

// Created by AA1GD Aug. 25, 2021
// OCTOBER 18, 2021 AT 5:14 PM CDT FIRST ON AIR DECODES WITH THIS
#define MY_CALLSIGN "VU3CER"
#define MY_GRID "MK68"

// GPS and time stuff
#include <TinyGPSPlus.h>
TinyGPSPlus gps;
#include <TimeLib.h>

message_info CurrentStation;

UserSendSelection sendChoices;

message_info message_list[kMax_decoded_messages]; // probably needs to be memset cleared before each decode

int16_t signal_for_processing[num_samples_processed] = {0};

uint32_t handler_max_time = 0;

void core1_irq_handler()
{
  // Copy capture_buf to signal_for_processing array, take fft and save to power
  while (multicore_fifo_rvalid())
  {
    uint32_t handler_start = time_us_32();
    uint16_t idx = multicore_fifo_pop_blocking();

    for (int i = 0; i < nfft; i++) {
      fresh_signal[i] -= DC_BIAS;
    }
    inc_extract_power(fresh_signal);
    uint32_t handler_time = (time_us_32() - handler_start) / 1000;
    if (handler_time > handler_max_time) {
      handler_max_time = handler_time;
    }
    // handler MUST BE under 160 ms.
  }

  multicore_fifo_clear_irq(); // Clear interrupt
}

void core1_runner(void)
{
  // Configure Core 1 Interrupt
  printf("second core running!\n");
  multicore_fifo_clear_irq();
  irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_irq_handler);

  irq_set_enabled(SIO_IRQ_PROC1, true);

  // Infinite While Loop to wait for interrupt
  while (1)
  {
    tight_loop_contents();
  }
}

// Prepare relays for TX
void pre_transmit()
{
  // Move T/R switch to TX (NO) position
  gpio_put(relay_1, 0);
  gpio_put(relay_2, 0);
  sleep_ms(30); // Songle SRD
}

void ptt(int state) {
  if (state == 1) {
    gpio_put(relay_1, 1);
    gpio_put(LED_PIN, 1);
  }
  else {
    gpio_put(relay_1, 0);
    gpio_put(LED_PIN, 0);
  }
}

void sync_time_with_gps_with_timeout()
{
  // digitalWrite(IS_GPS_SYNCED_PIN, LOW);
  bool newData = false;

  Serial.println("GPS Sync Wait...");
  digitalWrite(LED_BUILTIN, LOW);
  Serial2.begin(9600); // https://github.com/earlephilhower/arduino-pico/blob/master/variants/rpipico/pins_arduino.h#L11-L15

  for (unsigned long start = millis(); millis() - start < 32000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
#ifdef debugGPS
      Serial.write(c);
#endif
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
    // http://arduiniana.org/libraries/tinygpsplus/
    if (newData && gps.time.isUpdated() && gps.date.isUpdated() && (gps.location.isValid() && gps.location.age() < 2000)) {
      int16_t Year = gps.date.year();
      int8_t Month = gps.date.month();
      int8_t Day = gps.date.day();
      int8_t Hour = gps.time.hour();
      int8_t Minute = gps.time.minute();
      // int8_t Second = gps.time.second() - 3; // https://github.com/mikalhart/TinyGPSPlus/issues/30
      int8_t Second = gps.time.second();
      datetime_t t = {
        .year  = Year,
        .month = Month,
        .day   = Day,
        .hour  = Hour,
        .min   = Minute,
        .sec   = Second
      };
      // Start the RTC
      rtc_init();
      bool ret = rtc_set_datetime(&t);
      printf("Status of rtc_set_datetime => %d\n", ret);
      setTime(Hour, Minute, Second, Day, Month, Year);
      digitalWrite(LED_BUILTIN, HIGH);
      sleep_ms(1000);
      digitalWrite(LED_BUILTIN, LOW);
      sleep_ms(1000);
      digitalWrite(LED_BUILTIN, HIGH);
      sleep_ms(1000);
      digitalWrite(LED_BUILTIN, LOW);
      sleep_ms(1000);
      Serial.println("GPS Sync Done!");
      return;
    }
  }
  Serial.println("GPS Sync Failed!");
}

#define NTP_SERVER "pool.ntp.org"
#define NTP_MSG_LEN 48
#define NTP_PORT 123
#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_TEST_TIME (30 * 1000)
#define NTP_RESEND_TIME (10 * 1000)

// Called with results of operation
static void ntp_result(NTP_T* state, int status, time_t *result) {
  if (status == 0 && result) {
    struct tm *utc = gmtime(result);
    printf("got ntp response: %02d/%02d/%04d %02d:%02d:%02d\n", utc->tm_mday, utc->tm_mon + 1, utc->tm_year + 1900,
           utc->tm_hour, utc->tm_min, utc->tm_sec);
  }

  if (state->ntp_resend_alarm > 0) {
    cancel_alarm(state->ntp_resend_alarm);
    state->ntp_resend_alarm = 0;
  }
  state->ntp_test_time = make_timeout_time_ms(NTP_TEST_TIME);
  state->dns_request_sent = false;
}

static int64_t ntp_failed_handler(alarm_id_t id, void *user_data);

// Make an NTP request
static void ntp_request(NTP_T *state) {
  // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
  // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
  // these calls are a no-op and can be omitted, but it is a good practice to use them in
  // case you switch the cyw43_arch type later.
  cyw43_arch_lwip_begin();
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
  uint8_t *req = (uint8_t *) p->payload;
  memset(req, 0, NTP_MSG_LEN);
  req[0] = 0x1b;
  udp_sendto(state->ntp_pcb, p, &state->ntp_server_address, NTP_PORT);
  pbuf_free(p);
  cyw43_arch_lwip_end();
}

static int64_t ntp_failed_handler(alarm_id_t id, void *user_data)
{
  NTP_T* state = (NTP_T*)user_data;
  printf("ntp request failed\n");
  ntp_result(state, -1, NULL);
  return 0;
}

// Call back with a DNS result
static void ntp_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
  NTP_T *state = (NTP_T*)arg;
  if (ipaddr) {
    state->ntp_server_address = *ipaddr;
    printf("ntp address %s\n", ip4addr_ntoa(ipaddr));
    ntp_request(state);
  } else {
    printf("ntp dns request failed\n");
    ntp_result(state, -1, NULL);
  }
}

// NTP data received
static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
  NTP_T *state = (NTP_T*)arg;
  uint8_t mode = pbuf_get_at(p, 0) & 0x7;
  uint8_t stratum = pbuf_get_at(p, 1);

  // Check the result
  if (ip_addr_cmp(addr, &state->ntp_server_address) && port == NTP_PORT && p->tot_len == NTP_MSG_LEN &&
      mode == 0x4 && stratum != 0) {
    uint8_t seconds_buf[4] = {0};
    pbuf_copy_partial(p, seconds_buf, sizeof(seconds_buf), 40);
    uint32_t seconds_since_1900 = seconds_buf[0] << 24 | seconds_buf[1] << 16 | seconds_buf[2] << 8 | seconds_buf[3];
    uint32_t seconds_since_1970 = seconds_since_1900 - NTP_DELTA;
    epoch = seconds_since_1970;
    have_time = true;
    ntp_result(state, 0, &epoch);
  } else {
    printf("invalid ntp response\n");
    ntp_result(state, -1, NULL);
  }
  pbuf_free(p);
}

// Perform initialisation
static NTP_T* ntp_init(void) {
  NTP_T *state = (NTP_T *)calloc(1, sizeof(NTP_T));
  if (!state) {
    printf("failed to allocate state\n");
    return NULL;
  }
  state->ntp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  if (!state->ntp_pcb) {
    printf("failed to create pcb\n");
    free(state);
    return NULL;
  }
  udp_recv(state->ntp_pcb, ntp_recv, state);
  return state;
}

// Runs ntp test forever
void run_ntp_test(void) {
  NTP_T *state = ntp_init();
  if (!state)
    return;
  while (true) {
    if (absolute_time_diff_us(get_absolute_time(), state->ntp_test_time) < 0 && !state->dns_request_sent) {

      // Set alarm in case udp requests are lost
      state->ntp_resend_alarm = add_alarm_in_ms(NTP_RESEND_TIME, ntp_failed_handler, state, true);

      // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
      // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
      // these calls are a no-op and can be omitted, but it is a good practice to use them in
      // case you switch the cyw43_arch type later.
      cyw43_arch_lwip_begin();
      int err = dns_gethostbyname(NTP_SERVER, &state->ntp_server_address, ntp_dns_found, state);
      cyw43_arch_lwip_end();

      state->dns_request_sent = true;
      if (err == ERR_OK) {
        ntp_request(state); // Cached result
      } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        printf("dns request failed\n");
        ntp_result(state, -1, NULL);
      }
    }
#if PICO_CYW43_ARCH_POLL
    // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
    // main loop (not from a timer) to check for WiFi driver or lwIP work that needs to be done.
    cyw43_arch_poll();
    sleep_ms(1);
#else
    // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
    // is done via interrupt in the background. This sleep is just an example of some (blocking)
    // work you might be doing.
    sleep_ms(1000);
#endif
    if (have_time) {
      printf("Breaking NTP loop!\n");
      break;
    }
  }
  free(state);
}

static void print_hex(unsigned char *str, int len)
{
  int i;
  for (i = 0; i < len; ++i)
    printf("%02x", str[i]);
}

extern std::map<string, bool> seen;
extern vector<struct decoder_results> dec_results_queue;

PskReporter reporter("VU3CER", "MK68xm", "TestSDR_v2");

void pskUploader() {
  if (dec_results_queue.size() > 0) {
    for (int i = 0; i < MAX_REPORTS_PER_PACKET && dec_results_queue.size() > 0; i++) {
      struct decoder_results dr = dec_results_queue.front();
      reporter.addReceiveRecord(dr.call, dr.freq, dr.snr);
      dec_results_queue.erase(dec_results_queue.begin());
    }
    // reporter.randomIdentifier_ = reporter.randomIdentifier_ + 1;
    seen.clear();
    unsigned char *buf;
    ip_addr_t destination_address;
    //ip_addr_t me;
    IP4_ADDR(&destination_address, 74, 116, 41, 13);
    // IP4_ADDR(&me, 192, 168, 1, 128);
    int length;
    buf = reporter.send(&length);
    printf("Length is %d\n", length);
    print_hex(buf, length);
    printf("\n\n");
    cyw43_arch_lwip_begin();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
    uint8_t *req = (uint8_t *)p->payload;
    memcpy(req, buf, length);
    struct udp_pcb *upcb = udp_new();
    udp_sendto(upcb, p, &destination_address, 4739);
    // udp_sendto(upcb, p, &me, 4739);
    pbuf_free(p);
    udp_remove(upcb);
    cyw43_arch_lwip_end();
  }
}

int main()
{
  // Overclocking the processor
  // 133 MHz is the default, 250 MHz is safe at 1.1V and for flash
  // If using clock > 290MHz, increase voltage and add flash divider
  // See https://raspberrypi.github.io/pico-sdk-doxygen/vreg_8h.html
  // vreg_set_voltage(VREG_VOLTAGE_DEFAULT); // default: VREG_VOLTAGE_1_10 max:VREG_VOLTAGE_1_30
  // vreg_set_voltage(VREG_VOLTAGE_1_30); // default: VREG_VOLTAGE_1_10 max:VREG_VOLTAGE_1_30
  // set_sys_clock_khz(250000, true);
  // set_sys_clock_khz(250000, true);
  // set_sys_clock_khz(290400, true);
  setup_default_uart();

  // initialize GPIO pins
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, 0);
  gpio_init(PTT_PIN);
  gpio_set_dir(PTT_PIN, GPIO_OUT);
  gpio_put(PTT_PIN, 0);
  gpio_init(relay_1);
  gpio_set_dir(relay_1, GPIO_OUT);
  gpio_put(relay_1, 0);
  gpio_init(relay_2);
  gpio_set_dir(relay_2, GPIO_OUT);
  gpio_put(relay_2, 0);

  // start serial connection
  stdio_init_all();

  // setup the adc
  setup_adc();

  // WiFi stuff
  if (cyw43_arch_init_with_country(CYW43_COUNTRY_INDIA)) {
    printf("Failed to initialise!\n");
    return 1;
  }
  cyw43_arch_enable_sta_mode();
  while (1) {
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK, 10000)) {
      printf(".\n");
      continue;
    }
    printf("Connected to WiFi!\n");
    break;
  }

  // Get NTP time
  // run_ntp_test();

  // Spotting stuff
  /* unsigned char *buf;
    ip_addr_t destination_address;
    IP4_ADDR(&destination_address, 74, 116, 41, 13);
    int length;
    reporter.addReceiveRecord("VU3FOE", 14074000, 0);
    buf = reporter.send(&length);
    // printf("Length is %d\n", length);
    // print_hex(buf, length);
    cyw43_arch_lwip_begin();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
    uint8_t *req = (uint8_t *) p->payload;
    memcpy(req, buf, length);
    struct udp_pcb *upcb = udp_new();;
    udp_sendto(upcb, p, &destination_address, 4739);
    pbuf_free(p);
    cyw43_arch_lwip_end(); */
  // cyw43_arch_deinit(); // Nope!

  // Initialize the Si5351 Oct. 10, 2021
  uint32_t rx_freq = 14074000; // Standard FT8 frequency on 20m band
  uint32_t tx_freq = 14075500;
  si_init();
  // Ri is set to divide the given frequency by 4 for VFO 1 (CLK 1)
  // This is to allow Si5351 to make the .25Hz increments for FT8
  SI_SETFREQ(0, rx_freq); // Note: CLK 0 is used as input for CD2003 / TA2003 receiver chip
  si_evaluate();

  // Make hanning window for fft work
  make_window();

  sleep_ms(1000);

  // Start //
  char message[32];
  uint8_t tones[256];
  int offset = 0;
  unsigned char rec_byte[2];
  unsigned int msg_index;
  bool message_available = true;
  bool send = false;
  bool justSent = false; // must recieve right after sending
  bool autosend = true;
  bool cq_only = false;

  // Sync time with GPS
  sync_time_with_gps_with_timeout();
  // sleep_ms(1000);
  /* sync_time_with_gps_with_timeout();
    sleep_ms(1000);
    sync_time_with_gps_with_timeout();
    sleep_ms(1000);
    sync_time_with_gps_with_timeout();
    sleep_ms(1000);
    sync_time_with_gps_with_timeout(); */

  // launch second core
  multicore_launch_core1(core1_runner);

  // decode loop
  int count = 0;
  while (true) {
    if (second() % 15 == 0) { // RX
      printf("RECEIVING FOR 12.8 SECONDS\n\n");
      uint32_t start = time_us_32();
      inc_collect_power();
      uint32_t stop = time_us_32();
      printf("Handler max time: %d\n", handler_max_time);
      printf("Recording time: %d us\n", stop - start);
      uint32_t decode_begin = time_us_32();
      uint8_t num_decoded = decode_ft8(message_list);
      printf("Decoding took this long: %d us\n", time_us_32() - decode_begin);
      decode_begin = time_us_32();
      count = count + 1;
      if (count >= 2) {
        pskUploader();
        count = 0;
      }
    }
#if PICO_CYW43_ARCH_POLL
    // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
    // main loop (not from a timer) to check for WiFi driver or lwIP work that needs to be done.
    printf("PICO_CYW43_ARCH_POLL is active!\n");
    cyw43_arch_poll();
    sleep_ms(1);
#endif
    sleep_ms(100);
  }

  // MAIN PROGRAM LOOP
  /* while (true)
    {
    // CAT
    for (;;) {
      int cmd = getchar_timeout_us(0);
      if (cmd == PICO_ERROR_TIMEOUT) {
        break;
      } else {
        printf("Got serial input!\n");
        if (cmd == 'm') {
          msg_index = 0;
          while (msg_index < sizeof(message)) {
            char c = getchar_timeout_us(0);
            if (c == PICO_ERROR_TIMEOUT)
              break;
            else {
              message[msg_index++] = c;
              message_available = true;
            }
          }
        }
        if (cmd == 'p') {
          if (message_available)
            printf("%s\n", message);
        }
        if (cmd == 'o')
        {
          msg_index = 0;
          // Offset encoded in two bytes
          while (msg_index < 2)
          {
            char c = getchar_timeout_us(0);
            if (c == PICO_ERROR_TIMEOUT)
              break;
            rec_byte[msg_index] = c;
            msg_index++;
          }
          offset = rec_byte[0] + (rec_byte[1] << 8);
          printf("o");
        }
        if (cmd == 't')
        {
          strcpy(message, "VU3CER VU3FOE MK68");
          generate_ft8(message, tones);
          send_ft8(tones, tx_freq, 0);
        }
        if (cmd == '1')
        {
          ptt(1);
        }
        if (cmd == '0')
        {
          ptt(0);
        }
        if (cmd == 'r') {
          // RX
          printf("RECEIVING FOR 12.8 SECONDS\n\n");
          inc_collect_power();
          printf("Handler max time: %d\n", handler_max_time);
          uint32_t decode_begin = time_us_32();
          uint8_t num_decoded = decode_ft8(message_list);
          printf("Decoding took this long: %d us\n", time_us_32() - decode_begin);
          decode_begin = time_us_32();
        }
      }
    }
    sleep_ms(100);
    } */

  return 0;
}
