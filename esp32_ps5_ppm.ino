// PS5 DualSense -> ESP32 (Bluepad32) -> JR-style PPM for an external TX module.
//
// Board: original ESP32 (ESP32-D0WD), core: esp32-bluepad32:esp32:esp32
//
// Channel order is Futaba AETR (matches the receiver in use):
//   CH1 Aileron   <- right stick X  (3-ch receiver: rudder servo sits on CH1)
//   CH2 Elevator  <- right stick Y
//   CH3 Throttle  <- left stick Y   (centre = idle, push up = more throttle)
//   CH4 Rudder    <- right stick X  (same as CH1, for 4-ch receivers)
//   CH5-CH8       <- centred
//
// Square toggles rudder reverse, Triangle toggles elevator reverse (saved across reboots).
// Light bar: green = normal, blue = rudder rev, yellow = elevator rev, red = both rev.
//
// Built-in LED (GPIO2) blinks while either stick is off-centre.
// If the controller disconnects, throttle drops to idle and surfaces centre.

#include <Bluepad32.h>
#include <Preferences.h>
#include <driver/rmt.h>
#include <esp_timer.h>

// ---------------- Configuration ----------------
constexpr gpio_num_t PPM_PIN = GPIO_NUM_25;  // PPM out -> module bay signal pin
constexpr int LED_PIN = 2;                   // DevKit built-in LED
constexpr int PPM_IN_PIN = 26;               // PPM decoder input (loopback or receiver output)

constexpr int NUM_CHANNELS = 8;
constexpr uint32_t FRAME_US = 22500;  // standard 8-ch PPM frame
constexpr uint16_t PPM_MIN = 1000;
constexpr uint16_t PPM_MID = 1500;
constexpr uint16_t PPM_MAX = 2000;

// Defaults; both can be changed live over serial (see printHelp()).
// Inverted: line idles HIGH, separator pulses go LOW (Futaba/trainer-port style).
constexpr bool DEFAULT_PPM_INVERTED = true;
constexpr uint16_t DEFAULT_SEPARATOR_US = 300;

// true: left stick centre = idle, push up = full (safe for a spring-centred stick).
// false: full stick travel, bottom = idle, centre = half throttle.
constexpr bool THROTTLE_FROM_CENTRE = true;

constexpr int STICK_DEADZONE = 30;   // out of +/-512
constexpr uint32_t BLINK_MS = 100;   // LED toggle interval while sticks move

constexpr bool REVERSE_THROTTLE = false;

enum { CH_AIL = 0, CH_ELE = 1, CH_THR = 2, CH_RUD = 3 };

// ---------------- State ----------------
static volatile uint16_t channels[NUM_CHANNELS];
static ControllerPtr controller = nullptr;
static constexpr rmt_channel_t RMT_CH = RMT_CHANNEL_0;
static volatile bool ppmInverted = DEFAULT_PPM_INVERTED;
static volatile uint16_t separatorUs = DEFAULT_SEPARATOR_US;
enum TestMode { TEST_OFF, TEST_SWEEP, TEST_PATTERN };
static TestMode testMode = TEST_OFF;  // drive channels without a controller
static bool reverseElevator = false;  // toggled with Triangle
static bool reverseRudder = false;    // toggled with Square
static Preferences prefs;

static void setFailsafe() {
  for (int i = 0; i < NUM_CHANNELS; i++) channels[i] = PPM_MID;
  channels[CH_THR] = PPM_MIN;
}

// ---------------- PPM output (RMT hardware, jitter-free pulse widths) ----------------
static void sendPpmFrame(void*) {
  const uint32_t active = ppmInverted ? 0 : 1;
  const uint32_t idle = ppmInverted ? 1 : 0;
  const uint16_t sep = separatorUs;
  rmt_item32_t items[NUM_CHANNELS + 1];
  for (int i = 0; i < NUM_CHANNELS; i++) {
    items[i].level0 = active;
    items[i].duration0 = sep;
    items[i].level1 = idle;
    items[i].duration1 = channels[i] - sep;
  }
  // Final separator pulse; the line then idles until the next frame (sync gap).
  items[NUM_CHANNELS].level0 = active;
  items[NUM_CHANNELS].duration0 = sep;
  items[NUM_CHANNELS].level1 = idle;
  items[NUM_CHANNELS].duration1 = 0;
  rmt_write_items(RMT_CH, items, NUM_CHANNELS + 1, false);
}

static void setupPpm() {
  rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX(PPM_PIN, RMT_CH);
  cfg.clk_div = 80;  // 80 MHz APB / 80 = 1 us per tick
  cfg.tx_config.idle_output_en = true;
  cfg.tx_config.idle_level = ppmInverted ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW;
  ESP_ERROR_CHECK(rmt_config(&cfg));
  ESP_ERROR_CHECK(rmt_driver_install(RMT_CH, 0, 0));

  const esp_timer_create_args_t timerArgs = {
      .callback = &sendPpmFrame, .arg = nullptr, .dispatch_method = ESP_TIMER_TASK, .name = "ppm"};
  esp_timer_handle_t timer;
  ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer, FRAME_US));
}

static void setPolarity(bool inverted) {
  ppmInverted = inverted;
  rmt_set_idle_level(RMT_CH, true, inverted ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW);
}

// ---------------- Serial commands ----------------
static void printSettings() {
  Serial.printf("PPM: %s, separator %u us, %d ch, frame %lu us, test sweep %s\n",
                ppmInverted ? "INVERTED (idle high, low pulses)" : "NORMAL (idle low, high pulses)", separatorUs,
                NUM_CHANNELS, (unsigned long)FRAME_US,
                testMode == TEST_SWEEP ? "SWEEP" : testMode == TEST_PATTERN ? "PATTERN" : "off");
}

static void printHelp() {
  Serial.println(
      "Commands: p = flip polarity, s = separator 300/400 us, t = test sweep on/off, "
      "k = fixed pattern (CH1..8 = 1100,1300,..,1900,1500) on/off, ? = show settings");
}

static void handleSerial() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'p': setPolarity(!ppmInverted); printSettings(); break;
      case 's': separatorUs = separatorUs == 300 ? 400 : 300; printSettings(); break;
      case 't':
        testMode = testMode == TEST_SWEEP ? TEST_OFF : TEST_SWEEP;
        if (testMode == TEST_OFF) setFailsafe();
        printSettings();
        break;
      case 'k':
        testMode = testMode == TEST_PATTERN ? TEST_OFF : TEST_PATTERN;
        if (testMode == TEST_OFF) setFailsafe();
        printSettings();
        break;
      case '?': printSettings(); printHelp(); break;
    }
  }
}

// Slow triangle sweep on CH1-4 so servos visibly move end to end.
static void runTestSweep() {
  uint32_t phase = millis() % 4000;
  uint16_t v = phase < 2000 ? PPM_MIN + phase / 2 : PPM_MAX - (phase - 2000) / 2;
  for (int i = 0; i < 4; i++) channels[i] = v;
}

// Distinct value per channel so order mistakes are obvious.
static void runTestPattern() {
  static const uint16_t pattern[NUM_CHANNELS] = {1100, 1300, 1500, 1700, 1900, 1500, 1500, 1500};
  for (int i = 0; i < NUM_CHANNELS; i++) channels[i] = pattern[i];
}

// ---------------- PPM decoder (diagnostics) ----------------
// Times rising edges on PPM_IN_PIN. Rising-to-rising spacing equals the channel
// width for either polarity; a gap > 3 ms marks the frame sync. A plain servo
// signal shows up as 0 channels with its high-pulse width.
constexpr int MAX_IN_CHANNELS = 12;
static volatile uint16_t inWork[MAX_IN_CHANNELS];
static volatile uint16_t inChannels[MAX_IN_CHANNELS];
static volatile uint8_t inWorkCount = 0, inCount = 0;
static volatile uint32_t inFrames = 0, inEdges = 0;
static volatile uint32_t inLastRise = 0, inLastFall = 0, inFrameLen = 0, inSyncStart = 0;
static volatile uint16_t inHighUs = 0, inLowUs = 0;

static void IRAM_ATTR onPpmEdge() {
  uint32_t now = micros();
  inEdges++;
  if ((REG_READ(GPIO_IN_REG) >> PPM_IN_PIN) & 1) {
    inLowUs = now - inLastFall;
    uint32_t dt = now - inLastRise;
    inLastRise = now;
    if (dt > 3000) {
      if (inWorkCount > 0) {
        for (int i = 0; i < inWorkCount; i++) inChannels[i] = inWork[i];
        inCount = inWorkCount;
      }
      inFrameLen = now - inSyncStart;
      inSyncStart = now;
      inFrames++;
      inWorkCount = 0;
    } else if (inWorkCount < MAX_IN_CHANNELS) {
      inWork[inWorkCount++] = dt;
    }
  } else {
    inHighUs = now - inLastRise;
    inLastFall = now;
  }
}

static void setupPpmDecoder() {
  pinMode(PPM_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_IN_PIN), onPpmEdge, CHANGE);
}

static void printDecoded() {
  static uint32_t lastFrames = 0, lastEdges = 0;
  uint32_t frames = inFrames, edges = inEdges;
  if (edges == lastEdges) {
    Serial.printf("IN GPIO%d: no signal (line %s)\n", PPM_IN_PIN, digitalRead(PPM_IN_PIN) ? "HIGH" : "LOW");
  } else if (inCount == 0) {
    Serial.printf("IN GPIO%d: %lu frames, no channels; high %u us, low %u us\n", PPM_IN_PIN,
                  (unsigned long)(frames - lastFrames), inHighUs, inLowUs);
  } else {
    Serial.printf("IN GPIO%d: %2lu fr/0.5s, frame %5lu us, %d ch:", PPM_IN_PIN, (unsigned long)(frames - lastFrames),
                  (unsigned long)inFrameLen, inCount);
    for (int i = 0; i < inCount; i++) Serial.printf(" %4u", inChannels[i]);
    Serial.printf("  (high %u us, low %u us)\n", inHighUs, inLowUs);
  }
  lastFrames = frames;
  lastEdges = edges;
}

// ---------------- Stick mapping ----------------
static int applyDeadzone(int v) {
  if (abs(v) <= STICK_DEADZONE) return 0;
  // Rescale so output still reaches full range after the deadzone.
  int sign = v > 0 ? 1 : -1;
  return sign * map(abs(v), STICK_DEADZONE, 512, 0, 512);
}

// v in -512..512 -> 1000..2000
static uint16_t toPpm(int v, bool reverse) {
  if (reverse) v = -v;
  v = constrain(v, -512, 512);
  return PPM_MID + (v * (PPM_MAX - PPM_MID)) / 512;
}

static void updateChannels(ControllerPtr ctl) {
  // Bluepad32 axes are -511..512 with "up" negative; flip Y so up = positive.
  int ly = applyDeadzone(-ctl->axisY());
  int rx = applyDeadzone(ctl->axisRX());
  int ry = applyDeadzone(-ctl->axisRY());

  uint16_t thr;
  if (THROTTLE_FROM_CENTRE) {
    int up = max(ly, 0);  // only upward travel adds throttle
    thr = PPM_MIN + (constrain(up, 0, 512) * (PPM_MAX - PPM_MIN)) / 512;
    if (REVERSE_THROTTLE) thr = PPM_MAX + PPM_MIN - thr;
  } else {
    thr = toPpm(ly, REVERSE_THROTTLE);
  }

  channels[CH_THR] = thr;
  channels[CH_ELE] = toPpm(ry, reverseElevator);
  channels[CH_RUD] = toPpm(rx, reverseRudder);
  channels[CH_AIL] = channels[CH_RUD];
}

static bool sticksMoved(ControllerPtr ctl) {
  return abs(ctl->axisX()) > STICK_DEADZONE || abs(ctl->axisY()) > STICK_DEADZONE ||
         abs(ctl->axisRX()) > STICK_DEADZONE || abs(ctl->axisRY()) > STICK_DEADZONE;
}

// ---------------- Reverse toggles ----------------
static void showReverseState(ControllerPtr ctl) {
  if (reverseRudder && reverseElevator) ctl->setColorLED(255, 0, 0);
  else if (reverseRudder) ctl->setColorLED(0, 0, 255);
  else if (reverseElevator) ctl->setColorLED(255, 160, 0);
  else ctl->setColorLED(0, 255, 0);
}

static void handleReverseButtons(ControllerPtr ctl) {
  static bool lastSquare = false, lastTriangle = false;
  bool square = ctl->x(), triangle = ctl->y();
  bool changed = false;
  if (square && !lastSquare) {
    reverseRudder = !reverseRudder;
    changed = true;
  }
  if (triangle && !lastTriangle) {
    reverseElevator = !reverseElevator;
    changed = true;
  }
  lastSquare = square;
  lastTriangle = triangle;
  if (!changed) return;

  prefs.putBool("revRud", reverseRudder);
  prefs.putBool("revEle", reverseElevator);
  showReverseState(ctl);
  ctl->playDualRumble(0, 120, 0x80, 0x80);
  Serial.printf("Reverse: rudder %s, elevator %s\n", reverseRudder ? "REV" : "normal",
                reverseElevator ? "REV" : "normal");
}

// ---------------- Bluepad32 callbacks ----------------
void onConnectedController(ControllerPtr ctl) {
  if (controller != nullptr) {
    Serial.println("Second controller ignored");
    return;
  }
  controller = ctl;
  Serial.printf("Controller connected: %s\n", ctl->getModelName().c_str());
  showReverseState(ctl);
  ctl->setPlayerLEDs(0x01);
}

void onDisconnectedController(ControllerPtr ctl) {
  if (ctl != controller) return;
  controller = nullptr;
  setFailsafe();
  Serial.println("Controller disconnected -> failsafe");
}

// ---------------- Main ----------------
void setup() {
  Serial.begin(115200);
  prefs.begin("ps5ppm", false);
  reverseRudder = prefs.getBool("revRud", false);
  reverseElevator = prefs.getBool("revEle", false);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setFailsafe();
  setupPpm();
  setupPpmDecoder();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  const uint8_t* a = BP32.localBdAddress();
  Serial.printf("PS5 -> PPM receiver ready. BT addr %02X:%02X:%02X:%02X:%02X:%02X, PPM on GPIO%d\n", a[0], a[1],
                a[2], a[3], a[4], a[5], PPM_PIN);
  Serial.println("Pair: hold PS + Create on the controller until the light bar flashes.");
  printSettings();
  printHelp();
  Serial.printf("Reverse: rudder %s, elevator %s (Square / Triangle to toggle)\n", reverseRudder ? "REV" : "normal",
                reverseElevator ? "REV" : "normal");
}

void loop() {
  BP32.update();
  handleSerial();

  bool moving = false;
  if (testMode == TEST_SWEEP) {
    runTestSweep();
  } else if (testMode == TEST_PATTERN) {
    runTestPattern();
  } else if (controller && controller->isConnected() && controller->hasData() && controller->isGamepad()) {
    handleReverseButtons(controller);
    updateChannels(controller);
    moving = sticksMoved(controller);
  }

  static uint32_t lastToggle = 0;
  static bool ledOn = false;
  if (moving) {
    if (millis() - lastToggle >= BLINK_MS) {
      lastToggle = millis();
      ledOn = !ledOn;
      digitalWrite(LED_PIN, ledOn);
    }
  } else if (ledOn) {
    ledOn = false;
    digitalWrite(LED_PIN, LOW);
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.printf("%s  CH1 RUD %4u  CH2 ELE %4u  CH3 THR %4u  CH4 RUD %4u\n", controller ? "PS5 linked" : "no PS5   ",
                  channels[CH_AIL], channels[CH_ELE], channels[CH_THR], channels[CH_RUD]);
    printDecoded();
  }

  delay(5);
}
