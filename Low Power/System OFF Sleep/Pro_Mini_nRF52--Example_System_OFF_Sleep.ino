
#define AWAKE_LED_PIN     13      // LED to show when Pro Mini nRF52 is awake.
#define BAUD_RATE         115200  // Serial baud rate.

#define SYSTEM_SLEEP      OFF     // Sleep in System OFF mode.
#define WAKE_FROM         GPIO    // Can be one of RESET, GPIO, NFC or LPCOMP (see below).
#define GPIO_WAKE_PIN     15      // The user button on the Pro Mini nRF52.
#define GOOD_WAKE_LED_PIN 8       // LED to show whether we woke up for the right reason.

// Helper defines.
#define OFF 0
#define ON  1

// Options for what to wake up from - see above WAKE_FROM define - not fully coded yet!
#define RESET  0
#define GPIO   1
#define NFC    2
#define LPCOMP 3

// Bitfields of the Reset Reason register.
typedef union {
  struct {
    uint8_t resetpin:1;
    uint8_t dog:1;
    uint8_t sreq:1;
    uint8_t lockup:1;
    uint8_t :4;
    uint8_t :8;
    uint8_t off:1;
    uint8_t lpcomp:1;
    uint8_t dif:1;
    uint8_t nfc:1;
    uint8_t :4;
    uint8_t :8;
  } bit;
  uint32_t reg;
} reset_reason_t;


void setup() {
  pinMode(AWAKE_LED_PIN, OUTPUT);     // Set LED pin to output.
  pinMode(GOOD_WAKE_LED_PIN, OUTPUT); // Set LED pin to output.
  digitalWrite(AWAKE_LED_PIN, HIGH);  // Turn on LED to show we're awake.

  // Display some helpful debug info - note: UART will be "nicely" shutdown later.
  Serial.begin(BAUD_RATE);
  while (!Serial);
  Serial.println("\nPro Mini nRF52 - Example System OFF Sleep");

  decodeResetReason();   // Determine the reset reason...
  clearResetReason();    // ...then clear it.

  NRF_POWER->DCDCEN = 1; // Enable DCDC mode for lowest possible power consumption.

  powerDownRAM();        // Power down as much RAM as possible for the lowest possible power consumption.
}

void loop() {
  Serial.println("Going to sleep..."); // Just so we know...
  digitalWrite(AWAKE_LED_PIN, LOW);    // Turn off LED to show we're going to sleep.

  sleep(SYSTEM_SLEEP, WAKE_FROM, GPIO_WAKE_PIN);  // System OFF sleep until woken up.
                                                  // Note: No further code is executed after a System OFF sleep since
                                                  //       the device wakes up from sleep in a reset state.  We can
                                                  //       decode the Reset Reason register to determine what woke us.
}

void sleep(bool system_on, uint8_t wake_from, uint16_t wake_pin) {
  hwSleepPrepare(system_on, wake_from, wake_pin);
  hwSleep(system_on);
}

void hwSleepPrepare(bool system_on, uint8_t wake_from, uint16_t wake_pin) {

  // Configure defined GPIO pin to wake from System OFF Sleep.
  if (!system_on && wake_from == GPIO) {
    pinMode(wake_pin, INPUT_PULLUP); // Set interrupt pin to input with internal pullup resistor.
    NRF_GPIO->PIN_CNF[wake_pin] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
    NRF_GPIO->PIN_CNF[wake_pin] |=  ((uint32_t)GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
  }

  // TODO: Code other examples of LPCOMP, NFC, etc.

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

// Sleep in System OFF mode.
inline void hwSleep(bool system_on) {
  if (!system_on) {
    NRF_POWER->SYSTEMOFF = 1; // Turn system off.
  }
}

void decodeResetReason(void) {
  reset_reason_t rst;             // Create a data element to examine bitfields of reset register.
  rst.reg = NRF_POWER->RESETREAS; // Get the current reset reason(s).

  Serial.println("");
  if (rst.bit.resetpin) { Serial.println("Reset from pin-reset detected."); }
  if (rst.bit.dog)      { Serial.println("Reset from watchdog detected."); }
  if (rst.bit.sreq)     { Serial.println("Reset from soft reset detected."); }
  if (rst.bit.lockup)   { Serial.println("Reset from CPU lock-up detected."); }
  if (rst.bit.off)      { Serial.println("Reset due to wake up from System OFF mode when wakeup is triggered from DETECT signal from GPIO."); }
  if (rst.bit.lpcomp)   { Serial.println("Reset due to wake up from System OFF mode when wakeup is triggered from ANADETECT signal from LPCOMP."); }
  if (rst.bit.dif)      { Serial.println("Reset due to wake up from System OFF mode when wakeup is triggered from entering into debug interface mode."); }
  if (rst.bit.nfc)      { Serial.println("Reset due to wake up from System OFF mode by NFC field detect."); }

  // Turn on the LED to show we woke up from the correct source.
  if (WAKE_FROM == GPIO && rst.bit.off) {
    digitalWrite(GOOD_WAKE_LED_PIN, HIGH);
    delay(5000);
    digitalWrite(GOOD_WAKE_LED_PIN, LOW);
  } else {
    delay(8000); // Wait 8 seconds, to give enough time for another upload.
  }
}

void clearResetReason(void) {
  reset_reason_t rst;             // Create a data element to examine bitfields of reset register.
  rst.reg = NRF_POWER->RESETREAS; // Get the current reset reason(s).
  NRF_POWER->RESETREAS = rst.reg; // Clear all the currently set bitfields of the reset register.
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
