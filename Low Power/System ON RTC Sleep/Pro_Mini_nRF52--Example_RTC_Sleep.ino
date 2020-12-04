
#define AWAKE_LED_PIN          13      // LED to show when Pro Mini nRF52 is awake.
#define BAUD_RATE              115200  // Serial baud rate.

#define MY_HW_RTC              NRF_RTC2         // Defines which real-time counter to use and
#define MY_HW_RTC_IRQ_HANDLER  RTC2_IRQHandler  // the associated handler and IRQ number. See
#define MY_HW_RTC_IRQN         RTC2_IRQn        // nRF52832 datasheet for more information.

// This flag will change from within the ISR, hence static volatile.
static volatile bool nrf52_rtc_event_triggered;

void setup() {
  pinMode(AWAKE_LED_PIN, OUTPUT);    // Set LED pin to output.
  digitalWrite(AWAKE_LED_PIN, HIGH); // Turn on LED to show we're awake.
  delay(8000);                       // Delay for new upload on power up.

  Serial.begin(BAUD_RATE);
  while (!Serial);
  Serial.println("Pro Mini nRF52 - Example RTC Sleep");

  NRF_POWER->DCDCEN = 1; // Enable DCDC mode for lowest possible power consumption.

  powerDownRAM();        // Power down as much RAM as possible for the lowest possible power consumption.
}

void loop() {
  Serial.println("Going to sleep..."); // Just so you know...
  digitalWrite(AWAKE_LED_PIN, LOW);    // Turn off LED to show we're going to sleep.
  sleep(10000);                        // Low power sleep for 10000ms (10 secs) using RTC to wake up.
  digitalWrite(AWAKE_LED_PIN, HIGH);   // Turn on LED to show we're awake.
  delay(3000);                         // Wait a little while for this to be acknowledged.
}

// Sleep in System ON RTC mode after having prepared the RTC to wake us up.
void sleep(uint32_t ms) {
  hwSleepPrepare(ms);
  while (nrf52_rtc_event_triggered == false) {
    hwSleep();
  }
  hwSleepEnd(ms);
}

void hwSleepPrepare(uint32_t ms) {

  // Reset RTC wake flag set by ISR.
  nrf52_rtc_event_triggered = false;

  // Enable low power sleep mode.
  NRF_POWER->TASKS_LOWPWR = 1;

  if (ms > 0) {
    // Reset RTC, just in case.
    MY_HW_RTC->TASKS_CLEAR = 1;

    // Calculate sleep time and prescaler.
    if (ms < 512000) {

      // With a prescaler of 0, the counter resolution will be 30.517μs (1/LFCLK = 1/32.768kHz).
      // Therefore it will overflow after only 512 seconds (approx. 8.5 minutes) sleep.
      MY_HW_RTC->PRESCALER = 0;

      // Set compare register to 1/30.517µs to guarantee event triggering.
      // A minimum of 2 ticks must be guaranteed to avoid jitter delays.
      // (1000/32768)<<12 == 125
      MY_HW_RTC->CC[0] = max(((ms << 12) / 125), 2);

    } else { // If we need a longer sleep time...

      // With a prescaler of 4095, the counter resolution will be 125ms.
      // (1/125ms = 8Hz) and (32.768kHz / 8Hz) - 1 = 4095.
      // Therefore it will overflow after 582.542 hours (approx. 24 days) sleep.
      MY_HW_RTC->PRESCALER = 4095;

      // Set compare register to 1/125ms to guarantee event triggering.
      // A minimum of 2 ticks must be guaranteed to avoid jitter delays.
      MY_HW_RTC->CC[0] = max((ms / 125), 2);
    }

    // Set up RTC and it's associated interrupt so that it can wake us from sleep.
    MY_HW_RTC->INTENSET = RTC_INTENSET_COMPARE0_Msk;
    MY_HW_RTC->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
    MY_HW_RTC->EVENTS_COMPARE[0] = 0;
    MY_HW_RTC->TASKS_START = 1;
    NVIC_SetPriority(MY_HW_RTC_IRQN, 15);
    NVIC_ClearPendingIRQ(MY_HW_RTC_IRQN);
    NVIC_EnableIRQ(MY_HW_RTC_IRQN);

  } else {
    NRF_RTC1->TASKS_STOP = 1;
  }

  // Idle serial device.
  if (Serial) {
    Serial.end();
    NRF_UART0->TASKS_STOPRX  = 1;
    NRF_UART0->TASKS_STOPTX  = 1;
    NRF_UART0->TASKS_SUSPEND = 1;
    while (NRF_UART0->ENABLE == 0x4); // Do nothing until the serial device disabled.
  }
}

// Sleep in System ON (with RTC enabled) mode.
inline void hwSleep(void) {
  __WFE();
  __SEV();
  __WFE();
}

// After waking up, be sure to stop the RTC, disable it's interrupt, etc.
void hwSleepEnd(uint32_t ms) {

  if (ms > 0) {
    // Stop RTC.
    MY_HW_RTC->INTENCLR = RTC_INTENSET_COMPARE0_Msk;
    MY_HW_RTC->EVTENCLR = RTC_EVTENSET_COMPARE0_Msk;
    MY_HW_RTC->TASKS_STOP = 1;
    NVIC_DisableIRQ(MY_HW_RTC_IRQN);

  } else {
    // Start Arduino RTC for millis().
    NRF_RTC1->TASKS_START = 1;
  }

  // Start serial device.
  NRF_UART0->TASKS_STARTRX = 1;
  NRF_UART0->TASKS_STARTTX = 1;
  Serial.begin(BAUD_RATE);
  while (!Serial);
  if (nrf52_rtc_event_triggered) {
    Serial.println("Woke up from sleeping via RTC! ");
  } else {
    Serial.println("Woke up for some other reason! ");
  }
}

/**
 * Reset events and read back on nRF52.
 * http://infocenter.nordicsemi.com/pdf/nRF52_Series_Migration_v1.0.pdf
 */
#if __CORTEX_M == 0x04
#define NRF_RESET_EVENT(event)                                                 \
  event = 0;                                                                   \
  (void)event
#else
#define NRF_RESET_EVENT(event) event = 0
#endif

extern "C" {
  // RTC interrupt handler
  void MY_HW_RTC_IRQ_HANDLER(void) {
    if (MY_HW_RTC->EVENTS_COMPARE[0] > 0) {
      nrf52_rtc_event_triggered = true;
      NRF_RESET_EVENT(MY_HW_RTC->EVENTS_COMPARE[0]);
    }
  }
}

// Keep the miminal amount of System RAM powered up.
void powerDownRAM() {
  #define SYSON_S0PWRON_S1PWRON     POWER_RAM_POWER_S0POWER_On  << POWER_RAM_POWER_S0POWER_Pos \
                                  | POWER_RAM_POWER_S1POWER_On  << POWER_RAM_POWER_S1POWER_Pos;

  #define SYSON_S0PWRON_S1PWROFF    POWER_RAM_POWER_S0POWER_On  << POWER_RAM_POWER_S0POWER_Pos \
                                  | POWER_RAM_POWER_S1POWER_Off << POWER_RAM_POWER_S1POWER_Pos;

  #define SYSON_S0PWROFF_S1PWRON    POWER_RAM_POWER_S0POWER_Off << POWER_RAM_POWER_S0POWER_Pos \
                                  | POWER_RAM_POWER_S1POWER_On  << POWER_RAM_POWER_S1POWER_Pos;

  #define SYSON_S0PWROFF_S1PWROFF   POWER_RAM_POWER_S0POWER_Off << POWER_RAM_POWER_S0POWER_Pos \
                                  | POWER_RAM_POWER_S1POWER_Off << POWER_RAM_POWER_S1POWER_Pos;

  NRF_POWER->RAM[0].POWER = SYSON_S0PWRON_S1PWROFF;
  NRF_POWER->RAM[1].POWER = SYSON_S0PWROFF_S1PWROFF;
  NRF_POWER->RAM[2].POWER = SYSON_S0PWROFF_S1PWROFF;
  NRF_POWER->RAM[3].POWER = SYSON_S0PWROFF_S1PWROFF;
  NRF_POWER->RAM[4].POWER = SYSON_S0PWROFF_S1PWROFF;
  NRF_POWER->RAM[5].POWER = SYSON_S0PWROFF_S1PWROFF;
  NRF_POWER->RAM[6].POWER = SYSON_S0PWROFF_S1PWROFF;
  NRF_POWER->RAM[7].POWER = SYSON_S0PWROFF_S1PWRON;
}
