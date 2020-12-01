
#define AWAKE_LED_PIN     13      // LED to show when Pro Mini nRF52 is awake.
#define BAUD_RATE         115200  // Serial baud rate.

#define SYSTEM_SLEEP      ON      // Sleep in System ON mode.
#define WAKE_FROM         GPIO    // Can be one of RESET, GPIO, NFC, LPCOMP or RTC (see below).
#define GPIO_WAKE_PIN     15      // The user button on the Pro Mini nRF52.
#define GPIO_WAKE_ON      PORT    // Can be one of PORT or PIN. PORT is lower power but poorer accuracy.
#define GOOD_WAKE_LED_PIN 8       // LED to show whether we woke up for the right reason.

// Helper defines.
#define OFF 0
#define ON  1

// Options for waking from a PORT change or PIN change - see above GPIO_WAKE_ON define.
#define PORT 0
#define PIN  1

// Options for what to wake up from - see above WAKE_FROM define - not fully coded yet!
#define RESET  0
#define GPIO   1
#define NFC    2
#define LPCOMP 3
#define RTC    4

// This flag will change from within the ISR, hence static volatile.
static volatile bool nrf5_irq_event_triggered = false;

// Simple flag-setting ISR.
void irqHandler(void) {
  nrf5_irq_event_triggered = true;
}

void setup() {
  pinMode(AWAKE_LED_PIN, OUTPUT);     // Set LED pin to output.
  pinMode(GOOD_WAKE_LED_PIN, OUTPUT); // Set LED pin to output.
  digitalWrite(AWAKE_LED_PIN, HIGH);  // Turn on LED to show we're awake.

  // Display some helpful debug info - note: UART will be "nicely" shutdown later.
  Serial.begin(BAUD_RATE);
  while (!Serial);
  Serial.println("\nPro Mini nRF52 - Example System ON Sleep");

  delay(8000);           // First time executing, allow an 8 second pause for uploading new code.

  NRF_POWER->DCDCEN = 1; // Enable DCDC mode for lowest possible power consumption.

  powerDownRAM();        // Power down as much RAM as possible for the lowest possible power consumption.

  pinMode(GPIO_WAKE_PIN, INPUT_PULLUP); // Set interrupt pin to input with internal pullup resistor.

  // Waking from a PORT vs. individual PIN is lower power, but poorer accuracy - choose from the defines above.
  if (GPIO_WAKE_ON == PORT) {
    attachInterruptLowAccuracy(digitalPinToInterrupt(GPIO_WAKE_PIN), irqHandler, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(GPIO_WAKE_PIN), irqHandler, FALLING);
  }
}

void loop() {
  Serial.println("Going to sleep...");            // Just so we know...
  digitalWrite(AWAKE_LED_PIN, LOW);               // Turn off LEDs to show we're going to sleep.
  digitalWrite(GOOD_WAKE_LED_PIN, LOW);

  sleep(SYSTEM_SLEEP, WAKE_FROM, GPIO_WAKE_PIN);  // System ON sleep until woken up.
                                                  // Note: Unlike System OFF, code execution continues from
                                                  //       this point after the device wakes up from sleep.
                                                  //       No need to decode the Reset Reason register.

  // ISR above should have set this flag for us after we woke up from sleep.
  if (nrf5_irq_event_triggered) {
    digitalWrite(AWAKE_LED_PIN, HIGH);
    digitalWrite(GOOD_WAKE_LED_PIN, HIGH);
    Serial.println("Wake up from sleeping via IRQ!");
    delay(3000);
  }
}

// Sleep in System ON mode.
void sleep(bool system_on, uint8_t wake_from, uint16_t wake_pin) {
  hwSleepPrepare(system_on, wake_from, wake_pin);
  hwSleep(system_on);
  hwSleepEnd();
}

void hwSleepPrepare(bool system_on, uint8_t wake_from, uint16_t wake_pin) {

  // Reset wake flag set by ISR.
  nrf5_irq_event_triggered = false;

  // Enable low power sleep mode.
  NRF_POWER->TASKS_LOWPWR = 1;

  // Idle serial device.
  if (Serial) {
    Serial.end();
    NRF_UART0->TASKS_STOPRX  = 1;
    NRF_UART0->TASKS_STOPTX  = 1;
    NRF_UART0->TASKS_SUSPEND = 1;
    while (NRF_UART0->ENABLE == 0x4); // Do nothing until the serial device disabled.
  }
}

// Sleep in System ON or OFF mode.
inline void hwSleep(bool system_on) {
  if (!system_on) {
    NRF_POWER->SYSTEMOFF = 1; // Turn system off.

  } else {                    // Keep system on, but power-down everything we're not using.
    __SEV();
    __WFE();
    __WFE();
  }
}

// After waking up, be sure to re-enable the UART so we can see what happened.
void hwSleepEnd(void) {
  if (nrf5_irq_event_triggered) {
    NRF_UART0->TASKS_STARTRX = 1;
    NRF_UART0->TASKS_STARTTX = 1;
    Serial.begin(BAUD_RATE);
    while (!Serial);
  }
}

// Keep the miminal amount of System RAM powered up.
void powerDownRAM(void) {
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
