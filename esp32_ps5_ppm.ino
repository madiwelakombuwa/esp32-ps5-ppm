// PS5 DualSense -> ESP32 (Bluepad32) -> PPM, on two outputs:
//   GPIO25: external TX module (e.g. 900 MHz). Failsafe on PS5 loss: throttle idle, surfaces centre.
//   GPIO26: radio trainer port (e.g. RadioMaster TX16S, Master/Jack). Stops on PS5 loss so the
//           radio reports "trainer signal lost" and the master sticks take over.
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
// Circle toggles rudder + elevator travel 100% <-> 125% (player LEDs: 1 = 100%, 5 = 125%).
// L1 = launch hold (full up elevator, same as right stick fully back), L2 = release.
//   Moving the right stick up/down past LAUNCH_CANCEL also releases it.
// R1 / R2 = elevator trim down / up (hold to repeat, both together = reset). Saved across reboots.
//
// Built-in LED (GPIO2) blinks while either stick is off-centre.

#include <Bluepad32.h>
#include <Preferences.h>
#include <driver/rmt.h>
#include <esp_timer.h>

// ---------------- Configuration ----------------
constexpr gpio_num_t PPM_PIN = GPIO_NUM_25;      // PPM out -> TX module signal pin
constexpr gpio_num_t TRAINER_PIN = GPIO_NUM_26;  // PPM out -> radio trainer jack tip
constexpr int LED_PIN = 2;                       // DevKit built-in LED
constexpr int PPM_IN_PIN = 27;                   // PPM decoder input (loopback or receiver output)

constexpr int NUM_CHANNELS = 8;
constexpr uint32_t FRAME_US = 22500;  // standard 8-ch PPM frame
constexpr uint16_t PPM_MIN = 1000;
constexpr uint16_t PPM_MID = 1500;
constexpr uint16_t PPM_MAX = 2000;

// Defaults; both can be changed live over serial (see printHelp()).
// Inverted: line idles HIGH, separator pulses go LOW (Futaba/trainer-port style).
constexpr bool DEFAULT_PPM_INVERTED = true;
constexpr bool DEFAULT_TRAINER_INVERTED = true;
constexpr uint16_t DEFAULT_SEPARATOR_US = 300;

// true: left stick centre = idle, push up = full (safe for a spring-centred stick).
// false: full stick travel, bottom = idle, centre = half throttle.
constexpr bool THROTTLE_FROM_CENTRE = true;

constexpr int STICK_DEADZONE = 30;   // out of +/-512
constexpr uint32_t BLINK_MS = 100;   // LED toggle interval while sticks move

constexpr bool REVERSE_THROTTLE = false;

// Extended travel for rudder/elevator: 125% = 1500 +/- 625 us (875..2125).
constexpr int HIGH_TRAVEL_PERCENT = 125;

// Launch hold: up elevator as % of full stick throw; stick deflection (of 512) that cancels it.
constexpr int LAUNCH_UP_PERCENT = 100;
constexpr int LAUNCH_CANCEL = 200;

// Elevator trim in stick units (512 = full throw, ~1 us each at 100% travel).
constexpr int TRIM_STEP = 8;
constexpr int TRIM_LIMIT = 128;  // +/-25% of throw
constexpr uint32_t TRIM_REPEAT_DELAY_MS = 400;
constexpr uint32_t TRIM_REPEAT_MS = 120;

// L2/R2 are analog (0..1023); use their position with hysteresis, since the
// digital trigger bit can flicker mid-pull and register as extra presses.
constexpr int TRIGGER_ON = 300;
constexpr int TRIGGER_OFF = 150;

enum { CH_AIL = 0, CH_ELE = 1, CH_THR = 2, CH_RUD = 3 };

// ---------------- State ----------------
static volatile uint16_t channels[NUM_CHANNELS];
static ControllerPtr controller = nullptr;
struct PpmOutput {
  const char* name;
  gpio_num_t pin;
  rmt_channel_t rmt;
  volatile bool inverted;
  volatile bool enabled;
};
static PpmOutput moduleOut = {"TX module", PPM_PIN, RMT_CHANNEL_0, DEFAULT_PPM_INVERTED, true};
static PpmOutput trainerOut = {"Trainer", TRAINER_PIN, RMT_CHANNEL_1, DEFAULT_TRAINER_INVERTED, false};
static volatile uint16_t separatorUs = DEFAULT_SEPARATOR_US;
enum TestMode { TEST_OFF, TEST_SWEEP, TEST_PATTERN };
static TestMode testMode = TEST_OFF;  // drive channels without a controller
static bool reverseElevator = false;  // toggled with Triangle
static bool reverseRudder = false;    // toggled with Square
static bool highTravel = false;       // toggled with Circle
static bool launchHold = false;       // L1 on, L2 / stick off
static int elevatorTrim = 0;          // R1 / R2, + = toward stick-up (down elevator)
static Preferences prefs;

static void setFailsafe() {
  for (int i = 0; i < NUM_CHANNELS; i++) channels[i] = PPM_MID;
  channels[CH_THR] = PPM_MIN;
}

// ---------------- PPM output (RMT hardware, jitter-free pulse widths) ----------------
static void writeFrame(const PpmOutput& out) {
  if (!out.enabled) return;  // line rests at idle level -> receiver sees signal loss
  const uint32_t active = out.inverted ? 0 : 1;
  const uint32_t idle = out.inverted ? 1 : 0;
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
  rmt_write_items(out.rmt, items, NUM_CHANNELS + 1, false);
}

static void sendPpmFrames(void*) {
  writeFrame(moduleOut);
  writeFrame(trainerOut);
}

static void setupOutput(const PpmOutput& out) {
  rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX(out.pin, out.rmt);
  cfg.clk_div = 80;  // 80 MHz APB / 80 = 1 us per tick
  cfg.tx_config.idle_output_en = true;
  cfg.tx_config.idle_level = out.inverted ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW;
  ESP_ERROR_CHECK(rmt_config(&cfg));
  ESP_ERROR_CHECK(rmt_driver_install(out.rmt, 0, 0));
}

static void setupPpm() {
  setupOutput(moduleOut);
  setupOutput(trainerOut);
  const esp_timer_create_args_t timerArgs = {
      .callback = &sendPpmFrames, .arg = nullptr, .dispatch_method = ESP_TIMER_TASK, .name = "ppm"};
  esp_timer_handle_t timer;
  ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer, FRAME_US));
}

static void setPolarity(PpmOutput& out, bool inverted) {
  out.inverted = inverted;
  rmt_set_idle_level(out.rmt, true, inverted ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW);
}

// ---------------- Serial commands ----------------
static void printSettings() {
  for (const PpmOutput* out : {&moduleOut, &trainerOut}) {
    Serial.printf("%-9s GPIO%d: %s, %s\n", out->name, out->pin,
                  out->inverted ? "INVERTED (idle high, low pulses)" : "NORMAL (idle low, high pulses)",
                  out->enabled ? "sending" : "stopped");
  }
  Serial.printf("Separator %u us, %d ch, frame %lu us, test %s\n", separatorUs, NUM_CHANNELS,
                (unsigned long)FRAME_US,
                testMode == TEST_SWEEP ? "SWEEP" : testMode == TEST_PATTERN ? "PATTERN" : "off");
}

static void printHelp() {
  Serial.println(
      "Commands: p = flip TX module polarity, P = flip trainer polarity, s = separator 300/400 us, t = test sweep on/off, "
      "k = fixed pattern (CH1..8 = 1100,1300,..,1900,1500) on/off, ? = show settings");
}

static void handleSerial() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'p': setPolarity(moduleOut, !moduleOut.inverted); printSettings(); break;
      case 'P': setPolarity(trainerOut, !trainerOut.inverted); printSettings(); break;
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
// travelPercent scales the throw: 100 -> 1000..2000, 125 -> 875..2125.
static uint16_t toPpm(int v, bool reverse, int travelPercent = 100) {
  if (reverse) v = -v;
  v = constrain(v, -512, 512);
  return PPM_MID + (v * (PPM_MAX - PPM_MID) * travelPercent) / (512 * 100);
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
  int travel = highTravel ? HIGH_TRAVEL_PERCENT : 100;
  int ele = ry + elevatorTrim;
  if (launchHold) {
    if (abs(ry) > LAUNCH_CANCEL) {
      launchHold = false;
      Serial.println("Launch hold released by stick");
    } else {
      ele = -512 * LAUNCH_UP_PERCENT / 100;  // stick fully back = up elevator
    }
  }
  channels[CH_ELE] = toPpm(ele, reverseElevator, travel);
  channels[CH_RUD] = toPpm(rx, reverseRudder, travel);
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

static void showTravel(ControllerPtr ctl) { ctl->setPlayerLEDs(highTravel ? 0x1F : 0x01); }

static bool triggerPressed(int value, bool& state) {
  if (!state && value >= TRIGGER_ON) state = true;
  else if (state && value <= TRIGGER_OFF) state = false;
  return state;
}

static void handleLaunchButtons(ControllerPtr ctl) {
  static bool lastL1 = false, lastL2 = false, l2State = false;
  bool l1 = ctl->l1(), l2 = triggerPressed(ctl->brake(), l2State);
  if (l1 && !lastL1 && !launchHold) {
    launchHold = true;
    ctl->playDualRumble(0, 300, 0xFF, 0xFF);
    Serial.println("Launch hold: full up elevator");
  }
  if (l2 && !lastL2 && launchHold) {
    launchHold = false;
    ctl->playDualRumble(0, 80, 0x60, 0x60);
    Serial.println("Launch hold released");
  }
  lastL1 = l1;
  lastL2 = l2;
}

static void handleTrimButtons(ControllerPtr ctl) {
  static int lastDir = 0;
  static uint32_t pressedAt = 0, lastStep = 0;
  static bool resetDone = false, r2State = false;
  bool down = ctl->r1(), up = triggerPressed(ctl->throttle(), r2State);

  if (down && up) {  // both = reset to centre (once per press)
    if (!resetDone && elevatorTrim != 0) {
      elevatorTrim = 0;
      prefs.putShort("eleTrim", elevatorTrim);
      ctl->playDualRumble(0, 200, 0xFF, 0xFF);
      Serial.println("Elevator trim reset");
    }
    resetDone = true;
    lastDir = 0;
    return;
  }
  if (!down && !up) resetDone = false;
  if (resetDone) return;  // wait until both are released

  int dir = down ? 1 : up ? -1 : 0;
  uint32_t now = millis();
  bool step = false;
  if (dir != 0 && dir != lastDir) {
    step = true;
    pressedAt = now;
  } else if (dir != 0 && now - pressedAt >= TRIM_REPEAT_DELAY_MS && now - lastStep >= TRIM_REPEAT_MS) {
    step = true;
  }
  lastDir = dir;
  if (!step) return;
  lastStep = now;

  int next = constrain(elevatorTrim + dir * TRIM_STEP, -TRIM_LIMIT, TRIM_LIMIT);
  if (next == elevatorTrim) {
    ctl->playDualRumble(0, 150, 0xFF, 0x00);  // at the limit
    return;
  }
  elevatorTrim = next;
  prefs.putShort("eleTrim", elevatorTrim);
  if (elevatorTrim == 0) ctl->playDualRumble(0, 150, 0xC0, 0xC0);  // passed through centre
  else ctl->playDualRumble(0, 30, 0x40, 0x40);
  Serial.printf("Elevator trim %+d\n", elevatorTrim);
}

static void handleTravelButton(ControllerPtr ctl) {
  static bool lastCircle = false;
  bool circle = ctl->b();
  if (circle && !lastCircle) {
    highTravel = !highTravel;
    prefs.putBool("hiTravel", highTravel);
    showTravel(ctl);
    ctl->playDualRumble(0, highTravel ? 250 : 80, 0xC0, 0xC0);
    Serial.printf("Travel: %d%%\n", highTravel ? HIGH_TRAVEL_PERCENT : 100);
  }
  lastCircle = circle;
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
  showTravel(ctl);
}

void onDisconnectedController(ControllerPtr ctl) {
  if (ctl != controller) return;
  controller = nullptr;
  launchHold = false;  // never reconnect straight into full up elevator
  setFailsafe();
  Serial.println("Controller disconnected -> failsafe");
}

// ---------------- Main ----------------
void setup() {
  Serial.begin(115200);
  prefs.begin("ps5ppm", false);
  reverseRudder = prefs.getBool("revRud", false);
  reverseElevator = prefs.getBool("revEle", false);
  highTravel = prefs.getBool("hiTravel", false);
  elevatorTrim = constrain(prefs.getShort("eleTrim", 0), -TRIM_LIMIT, TRIM_LIMIT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setFailsafe();
  setupPpm();
  setupPpmDecoder();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  const uint8_t* a = BP32.localBdAddress();
  Serial.printf("PS5 -> PPM ready. BT addr %02X:%02X:%02X:%02X:%02X:%02X, TX module GPIO%d, trainer GPIO%d\n",
                a[0], a[1], a[2], a[3], a[4], a[5], PPM_PIN, TRAINER_PIN);
  Serial.println("Pair: hold PS + Create on the controller until the light bar flashes.");
  printSettings();
  printHelp();
  Serial.printf("Reverse: rudder %s, elevator %s (Square / Triangle to toggle)\n", reverseRudder ? "REV" : "normal",
                reverseElevator ? "REV" : "normal");
  Serial.printf("Travel: %d%% (Circle to toggle)\n", highTravel ? HIGH_TRAVEL_PERCENT : 100);
  Serial.printf("Elevator trim: %+d (R1 down / R2 up, both = reset)\n", elevatorTrim);
}

void loop() {
  BP32.update();
  handleSerial();

  bool moving = false;
  bool live = false;  // channels are driven by the PS5 or a test mode
  if (testMode == TEST_SWEEP) {
    live = true;
    runTestSweep();
  } else if (testMode == TEST_PATTERN) {
    live = true;
    runTestPattern();
  } else if (controller && controller->isConnected() && controller->hasData() && controller->isGamepad()) {
    handleReverseButtons(controller);
    handleTravelButton(controller);
    handleLaunchButtons(controller);
    handleTrimButtons(controller);
    updateChannels(controller);
    moving = sticksMoved(controller);
    live = true;
  }
  trainerOut.enabled = live;

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
    Serial.printf("%s  CH1 RUD %4u  CH2 ELE %4u  CH3 THR %4u  CH4 RUD %4u  trainer %s\n",
                  controller ? "PS5 linked" : "no PS5   ", channels[CH_AIL], channels[CH_ELE], channels[CH_THR],
                  channels[CH_RUD], trainerOut.enabled ? "ON" : "off");
    printDecoded();
  }

  delay(5);
}
