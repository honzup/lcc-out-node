/** \copyright
 * Copyright (c) 2019, Mike Dunston — BSD-3 (see OpenMRNLite ESP32/IOBoard example).
 *
 * OpenMRNLite "13-output effects" LCC node — fleet node #2.
 * Cloned from the sibling lcc-8out-40action-node OpenMRNLite port (proven robust
 * over the ESP32-P4 GridConnect-TCP bridge). Carried over VERBATIM:
 *   - the CAN setup (Esp32HardwareCan, GPIO4=RX / GPIO5=TX -> SN65HVD230, 125 kbit/s),
 *   - SPIFFS config storage,
 *   - the fsync-on-write persistence fix (SyncedFileMemorySpace + debounced fsync),
 *   - the WiFi-OTA-gated-by-LCC-events feature + SNIP.
 *
 * DIFFERENT from node #1:
 *   - Node ID 03.04.00.1E.48.02.
 *   - OUTPUTS ONLY: 13 outputs, no inputs/producers.
 *   - Each output is FIXED 1:1 to a GPIO (no pin dropdown).
 *   - Per-output EFFECT: 0=Off, 1=Steady, 2=Random, 3=Flicker(fire).
 *   - A non-blocking millis()-based effect engine drives the pins; the on/off
 *     events set a commanded state rather than writing the GPIO directly.
 *
 * OpenMRNLite v2.0.0 API (Esp32HardwareCan). CAN fixed at 125 kbit/s.
 */

#include <Arduino.h>
#include <SPIFFS.h>
#include <unistd.h>        // fsync — config persistence to flash
#include <WiFiManager.h>   // tzapu — WiFi provisioning for OTA (not for LCC)
#include <ArduinoOTA.h>

#include <OpenMRNLite.h>
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/CallbackEventHandler.hxx"

#define USE_CAN   // CAN-only node for LCC (WiFi is OTA-only, event-gated)

#include "config.h"

/// Node ID — MERG member 7752 (0x001E48), node #2: 03.04.00.1E.48.02.
static constexpr uint64_t NODE_ID = UINT64_C(0x0304001E4802);

/// CAN pins -> SN65HVD230:  GPIO4 = RX (from R),  GPIO5 = TX (to D).
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_4;
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_5;

OpenMRN openmrn(NODE_ID);

// Workaround for a GCC/ESP32 empty-string static-init bug.
string dummystring("abcdef");

static constexpr openlcb::ConfigDef cfg(0);

// ---- Fixed output GPIOs -------------------------------------------------------
// Channel N is hard-wired to OUTPUT_PINS[N] — the 13 usable GPIOs on a
// WROVER-class ESP32. GPIO2 is the WiFi-status LED (not an LCC output); GPIO4/5
// are the CAN transceiver. Order = channel order shown in JMRI (Output 1..13).
static constexpr int OUTPUT_PINS[] = {
    13, 14, 15, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
};
static_assert(ARRAYSIZE(OUTPUT_PINS) == openlcb::NUM_OUTPUTS,
              "OUTPUT_PINS must have one GPIO per output channel");

// ---- Effect ids (KEEP IN SYNC with EFFECT_MAP in config.h) --------------------
enum : uint8_t { EFF_OFF = 0, EFF_STEADY = 1, EFF_RANDOM = 2, EFF_FLICKER = 3 };

// ---- Effect engine tuning -----------------------------------------------------
static constexpr uint32_t TICK_MS          = 30;     // effect updater period
// Random (building lights): independent per output.
static constexpr long     RAND_ON_MIN_MS   = 20000;  // ~20 s .. ~4 min lit
static constexpr long     RAND_ON_MAX_MS   = 240000;
static constexpr long     RAND_OFF_MIN_MS  = 10000;  // ~10 s .. ~2 min dark
static constexpr long     RAND_OFF_MAX_MS  = 120000;
// Flicker (fire): LEDC PWM duty jitter.
static constexpr int      LEDC_FREQ        = 5000;   // Hz — above visible flicker
static constexpr int      LEDC_RES_BITS    = 8;      // 8-bit duty (0..255)
static constexpr int      FLICKER_MIN      = 40;     // warm floor (never fully dark)
static constexpr int      FLICKER_START    = 180;
static constexpr int      FLICKER_STEP     = 40;     // +/- duty per update
static constexpr long     FLICKER_MIN_MS   = 30;     // update every 30..60 ms
static constexpr long     FLICKER_MAX_MS   = 60;

// ---- Lazy LEDC channel allocator ----------------------------------------------
// The classic ESP32 has 16 LEDC channels; we have 13 outputs, so worst case (all
// Flicker) fits. Allocate a channel to a pin ONLY while that output is Flicker,
// and free it when the effect changes away — so churn never leaks channels.
static bool s_ledc_used[16] = {false};
static int alloc_ledc()
{
    for (int i = 0; i < 16; ++i)
        if (!s_ledc_used[i]) { s_ledc_used[i] = true; return i; }
    return -1;   // exhausted (can't happen with <=13 flicker outputs)
}
static void free_ledc(int ch)
{
    if (ch >= 0 && ch < 16) s_ledc_used[ch] = false;
}

/// One output channel: a config-driven consumer whose on/off events set a
/// commanded state; a non-blocking effect engine (update()) drives the GPIO.
/// Mirrors node #1's GPIOBit+BitEventConsumer pattern, but the Gpio it feeds is
/// a CmdGpio that toggles commanded_ instead of writing the pin.
class OutputChannel : public DefaultConfigUpdateListener
{
public:
    OutputChannel(openlcb::Node *node, const openlcb::OutputConfig &cfg, int pin)
        : node_(node)
        , pin_(pin)
        , gpio_(this)
        , impl_(node, 0, 0, &gpio_)
        , consumer_(&impl_)
        , cfg_(cfg)
    {
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) override
    {
        AutoNotify n(done);
        set_effect(cfg_.effect().read(fd));   // (re)configure pin / LEDC channel

        openlcb::EventId on = cfg_.event_on().read(fd);
        openlcb::EventId off = cfg_.event_off().read(fd);
        if (on == impl_.event_on() && off == impl_.event_off())
            return UPDATED;   // only the effect (or nothing) changed
        // (Re)register the consumer for the new events, keeping the same gpio_.
        consumer_.~BitEventConsumer();
        impl_.openlcb::GPIOBit::~GPIOBit();
        new (&impl_) openlcb::GPIOBit(node_, on, off, &gpio_);
        new (&consumer_) openlcb::BitEventConsumer(&impl_);
        return REINIT_NEEDED;
    }

    void factory_reset(int fd) override
    {
        char d[16];
        snprintf(d, sizeof d, "GPIO%d", pin_);   // fresh tab reads "Output N (GPIOxx)"
        cfg_.description().write(fd, d);
        CDI_FACTORY_RESET(cfg_.effect);   // Off
    }

    /// Non-blocking effect tick (called from loop() every ~TICK_MS). All hardware
    /// writes for the effects happen HERE (loop thread), never from the executor.
    void update(uint32_t now)
    {
        if (!commanded_)
        {
            if (active_) { force_off(); active_ = false; }
            return;
        }
        if (restart_)   // ON just commanded — (re)start this effect cleanly
        {
            restart_ = false;
            active_ = true;
            on_ = false;
            duty_ = FLICKER_START;
            next_toggle_ = now;
            next_flicker_ = now;
        }
        switch (effect_)
        {
        case EFF_STEADY:
            if (!on_) { digitalWrite(pin_, HIGH); on_ = true; }
            break;
        case EFF_RANDOM:
            if ((int32_t)(now - next_toggle_) >= 0)
            {
                on_ = !on_;
                digitalWrite(pin_, on_ ? HIGH : LOW);
                next_toggle_ = now + (uint32_t)(on_
                    ? random(RAND_ON_MIN_MS, RAND_ON_MAX_MS)
                    : random(RAND_OFF_MIN_MS, RAND_OFF_MAX_MS));
            }
            break;
        case EFF_FLICKER:
            if (ledc_ch_ >= 0 && (int32_t)(now - next_flicker_) >= 0)
            {
                int d = duty_ + (int)random(-FLICKER_STEP, FLICKER_STEP + 1);
                if (d < FLICKER_MIN) d = FLICKER_MIN;
                if (d > 255) d = 255;
                duty_ = d;
                ledcWrite(ledc_ch_, duty_);
                next_flicker_ = now + (uint32_t)random(FLICKER_MIN_MS, FLICKER_MAX_MS);
            }
            break;
        default:   // EFF_OFF — disabled; hold LOW
            break;
        }
    }

private:
    /// A Gpio that routes writes into the owning channel's commanded state
    /// (instead of a physical pin). SET = commanded ON, CLR = commanded OFF.
    class CmdGpio : public Gpio
    {
    public:
        explicit CmdGpio(OutputChannel *o) : owner_(o) {}
        void write(Value v) const override
        {
            if (v == SET) { owner_->commanded_ = true; owner_->restart_ = true; }
            else          { owner_->commanded_ = false; }
        }
        Value read() const override { return owner_->commanded_ ? SET : CLR; }
        void set() const override { write(SET); }
        void clr() const override { write(CLR); }
        void set_direction(Direction) const override {}
        Direction direction() const override { return Direction::DOUTPUT; }

    private:
        OutputChannel *owner_;
    };

    /// Apply an effect selection: allocate/tear down the LEDC channel and set the
    /// pin mode. Called from apply_configuration() (config-save is rare).
    void set_effect(uint8_t eff)
    {
        if (eff == EFF_FLICKER && ledc_ch_ < 0)
        {
            ledc_ch_ = alloc_ledc();
            if (ledc_ch_ >= 0)
            {
                ledcSetup(ledc_ch_, LEDC_FREQ, LEDC_RES_BITS);
                ledcAttachPin(pin_, ledc_ch_);
                ledcWrite(ledc_ch_, 0);
            }
        }
        else if (eff != EFF_FLICKER && ledc_ch_ >= 0)
        {
            ledcDetachPin(pin_);
            free_ledc(ledc_ch_);
            ledc_ch_ = -1;
        }
        if (eff != EFF_FLICKER)
        {
            pinMode(pin_, OUTPUT);
            digitalWrite(pin_, LOW);
        }
        effect_ = eff;
        on_ = false;
        active_ = false;
        // Re-arm from scratch on the next tick if still commanded ON.
        if (commanded_) restart_ = true;
    }

    void force_off()
    {
        on_ = false;
        if (ledc_ch_ >= 0) ledcWrite(ledc_ch_, 0);
        else               digitalWrite(pin_, LOW);
    }

    openlcb::Node *node_;
    const int pin_;
    CmdGpio gpio_;
    openlcb::GPIOBit impl_;
    openlcb::BitEventConsumer consumer_;
    const openlcb::OutputConfig cfg_;

    // Effect runtime state. commanded_/restart_ are written from the executor
    // (event handler) and read on loop(); everything else is loop-only. The
    // effects are LED timing, so the single-word cross-thread flags need no lock.
    // ponytail: benign cross-thread flags, no mutex; add one only if an effect
    // ever gates something safety-critical (it won't — these are lamps).
    volatile bool commanded_ = false;   // event_on seen, not yet event_off
    volatile bool restart_   = false;   // (re)initialise effect on next tick
    bool     active_   = false;         // effect currently running (loop view)
    bool     on_       = false;         // digital effects: pin currently HIGH
    uint8_t  effect_   = EFF_OFF;
    int      ledc_ch_  = -1;            // LEDC channel while Flicker, else -1
    int      duty_     = FLICKER_START; // Flicker current duty
    uint32_t next_toggle_  = 0;         // Random next flip time (ms)
    uint32_t next_flicker_ = 0;         // Flicker next update time (ms)
};

#define OUT_CH(i) \
    OutputChannel(openmrn.stack()->node(), cfg.seg().outputs().entry<i>(), OUTPUT_PINS[i])
OutputChannel output_channels[] = {
    OUT_CH(0),  OUT_CH(1),  OUT_CH(2),  OUT_CH(3),  OUT_CH(4),
    OUT_CH(5),  OUT_CH(6),  OUT_CH(7),  OUT_CH(8),  OUT_CH(9),
    OUT_CH(10), OUT_CH(11), OUT_CH(12)
};
#undef OUT_CH

class FactoryResetHelper : public DefaultConfigUpdateListener {
public:
    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE {
        AutoNotify n(done);
        return UPDATED;
    }
    void factory_reset(int fd) override
    {
        // Per-channel fields (description, effect) are reset by each
        // OutputChannel's own factory_reset(); here we only seed the node info.
        cfg.userinfo().name().write(fd, openlcb::SNIP_STATIC_DATA.model_name);
        cfg.userinfo().description().write(
            fd, openlcb::SNIP_STATIC_DATA.model_name);
    }
} factory_reset_helper;

namespace openlcb
{
    const char CDI_FILENAME[] = "/spiffs/cdi.xml";
    extern const char CDI_DATA[] = "";
    extern const char *const CONFIG_FILENAME = "/spiffs/openlcb_config";
    extern const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();
    extern const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
}

// ============ DURABLE config persistence: fsync-after-write (debounced) =======
// Root cause of "config writes lost on power-cycle": ESP32 SPIFFS keeps file
// writes in cache and only commits to flash on fsync/close; nothing here ever
// fsync'd the config file, so writes survived only by incidental page eviction.
//
// Fix: fsync ONLY after a config WRITE, never during a read. We override the
// SPACE_CONFIG memory space's write() to ARM a debounce; the actual fsync runs
// on the Arduino loop() thread once writes have been idle for a short window.
//   - Reads (incl. the long config-open datagram stream) never arm it -> a read
//     is never stalled by a flush.
//   - Multi-datagram writes coalesce: each fragment re-arms the timer, so we
//     fsync once, ~CFG_SYNC_DEBOUNCE_MS after the burst settles.
//   - fsync runs on loop(), not the executor, so it can't block datagram
//     servicing mid-write.
static constexpr uint32_t CFG_SYNC_DEBOUNCE_MS = 350;
static int                s_cfg_fd       = -1;
static volatile bool      s_cfg_dirty    = false;
static volatile uint32_t  s_cfg_dirty_ms = 0;

class SyncedFileMemorySpace : public openlcb::FileMemorySpace
{
public:
    SyncedFileMemorySpace(int fd, address_t len)
        : openlcb::FileMemorySpace(fd, len) {}

    size_t write(address_t dest, const uint8_t *data, size_t len,
                 errorcode_t *error, Notifiable *again) override
    {
        size_t w = openlcb::FileMemorySpace::write(dest, data, len, error, again);
        s_cfg_dirty_ms = millis();   // arm/re-arm the debounce (write path only)
        s_cfg_dirty    = true;
        return w;
    }
};

// ===================== WiFi-over-LCC control (OTA only) =====================
// Ported from node #1. WiFi is OFF at boot. JMRI switches it on/off by PRODUCING
// these events (bind a sensor/turnout/button); the node CONSUMES them. Flip ON to
// flash over WiFi, flip OFF when done — so LCC/CAN never depends on WiFi.
//   WiFi ON  : 03.04.00.1E.48.02.FE.00
//   WiFi OFF : 03.04.00.1E.48.02.FE.01
static constexpr uint64_t WIFI_ON_EVENT  = (NODE_ID << 16) | 0xFE00;
static constexpr uint64_t WIFI_OFF_EVENT = (NODE_ID << 16) | 0xFE01;

// WiFi-status LED (local indicator, NOT an LCC object): onboard GPIO2.
static constexpr int WIFI_LED_PIN = 2;

static volatile bool s_wifi_desired     = false;  // set from the LCC event (executor ctx)
static bool          s_wifi_up          = false;  // we've acted on the desired state
static bool          s_ota_ready        = false;
static volatile bool s_ota_in_progress  = false;  // OTA-pause hook

// Consumer: a command arrived from JMRI.
static void wifi_event_report(const openlcb::EventRegistryEntry &entry,
                              openlcb::EventReport *report, BarrierNotifiable *done)
{
    if (entry.event == WIFI_ON_EVENT)       s_wifi_desired = true;
    else if (entry.event == WIFI_OFF_EVENT) s_wifi_desired = false;
}

// Producer state: answers JMRI's "identify" queries so the sensor shows the TRUE
// WiFi state (e.g. after an OTA reboot, or when JMRI (re)starts).
static openlcb::EventState wifi_event_state(const openlcb::EventRegistryEntry &entry,
                                            openlcb::EventReport *report)
{
    const bool on = s_ota_ready;   // WiFi up + OTA ready
    if (entry.event == WIFI_ON_EVENT)
        return on ? openlcb::EventState::VALID : openlcb::EventState::INVALID;
    if (entry.event == WIFI_OFF_EVENT)
        return on ? openlcb::EventState::INVALID : openlcb::EventState::VALID;
    return openlcb::EventState::UNKNOWN;
}

openlcb::CallbackEventHandler wifi_ctrl(
    openmrn.stack()->node(), &wifi_event_report, &wifi_event_state);

static void enable_wifi()
{
    digitalWrite(WIFI_LED_PIN, HIGH);               // lit while connecting/up
    WiFi.mode(WIFI_STA);
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);      // 3 UniFi APs on one SSID:
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);  // pick the strongest, not the first
    WiFi.setSleep(false);
    // autoConnect: saved creds if provisioned, else raises AP "LCC-Out-Setup".
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    if (wm.autoConnect("LCC-Out-Setup"))
    {
        ArduinoOTA.setHostname("lcc-out-node");
        ArduinoOTA.onStart([]() { s_ota_in_progress = true;  });
        ArduinoOTA.onEnd  ([]() { s_ota_in_progress = false; });
        ArduinoOTA.begin();
        s_ota_ready = true;
        openmrn.stack()->send_event(WIFI_ON_EVENT);   // report true state -> JMRI sensor Active
        LOG(INFO, "[OTA] ready at %s", WiFi.localIP().toString().c_str());
    }
    else
    {
        digitalWrite(WIFI_LED_PIN, LOW);            // failed -> LED off
        LOG(INFO, "[WiFi] connect failed/cancelled - fire WiFi-ON again to retry");
    }
}

static void disable_wifi()
{
    if (s_ota_ready) { ArduinoOTA.end(); s_ota_ready = false; }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    digitalWrite(WIFI_LED_PIN, LOW);               // LED off
    openmrn.stack()->send_event(WIFI_OFF_EVENT);   // report true state -> JMRI sensor Inactive
    LOG(INFO, "[WiFi] off");
}

void setup()
{
    Serial.begin(115200L);
    uint8_t reset_reason = Esp32SocInfo::print_soc_info();
    LOG(INFO, "[Node] ID: %s", uint64_to_string_hex(NODE_ID).c_str());

    randomSeed(esp_random());   // per-boot randomised effect timing

    if (!SPIFFS.begin())
    {
        printf("SPIFFS mount failed, formatting...\n");
        if (!SPIFFS.begin(true))
        {
            printf("SPIFFS mount failed even with format, giving up!\n");
            while (1) { }
        }
    }

    // Output pin modes / LEDC channels are set per channel in apply_configuration()
    // (set_effect) once the config is loaded — no static hw_init().

    openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);
    s_cfg_fd = openmrn.stack()->create_config_file_if_needed(
        cfg.seg().internal_config(),
        openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

    // OpenMRNLite v2.0.0: (name, RX pin, TX pin). Bitrate fixed at 125 kbit/s.
    openmrn.add_can_port(
        new Esp32HardwareCan("esp32can", CAN_RX_PIN, CAN_TX_PIN));

    openmrn.begin();
    // Override the default SPACE_CONFIG memory space with one that fsyncs after
    // writes (see SyncedFileMemorySpace above). registry insert is a map assign,
    // so this replaces the FileMemorySpace the stack registered in begin().
    if (s_cfg_fd >= 0)
    {
        static SyncedFileMemorySpace synced_cfg(
            s_cfg_fd, openlcb::CONFIG_FILE_SIZE);
        openmrn.stack()->memory_config_handler()->registry()->insert(
            openmrn.stack()->node(),
            openlcb::MemoryConfigDefs::SPACE_CONFIG, &synced_cfg);
    }
    if (reset_reason == RTCWDT_BROWN_OUT_RESET)
    {
        openmrn.stack()->executor()->add(new CallbackExecutable([]()
        {
            openmrn.stack()->send_event(openlcb::Defs::NODE_POWER_BROWNOUT_EVENT);
        }));
    }
    // WiFi is OFF at boot and only enabled on demand via the LCC events above.
    WiFi.mode(WIFI_OFF);
    pinMode(WIFI_LED_PIN, OUTPUT);
    digitalWrite(WIFI_LED_PIN, LOW);   // WiFi off at boot -> LED off
    // Consumer (accept command) + Producer (report state back to JMRI).
    const uint32_t pc = openlcb::CallbackEventHandler::IS_CONSUMER
                      | openlcb::CallbackEventHandler::IS_PRODUCER;
    wifi_ctrl.add_entry(WIFI_ON_EVENT,  pc);
    wifi_ctrl.add_entry(WIFI_OFF_EVENT, pc);

    openmrn.start_executor_thread();

    // Announce the boot state (WiFi off) so the JMRI sensor is correct after a
    // reboot/OTA; JMRI's identify query is also answered by wifi_event_state().
    openmrn.stack()->send_event(WIFI_OFF_EVENT);
}

static uint32_t s_last_tick = 0;

void loop()
{
    if (s_wifi_desired && !s_wifi_up)       { s_wifi_up = true;  enable_wifi(); }
    else if (!s_wifi_desired && s_wifi_up)  { s_wifi_up = false; disable_wifi(); }
    if (s_ota_ready) ArduinoOTA.handle();
    openmrn.loop();

    // Non-blocking effect updater: tick every TICK_MS. Each channel advances its
    // own effect (steady/random/flicker) from its runtime state — no blocking,
    // so the executor/datagram servicing is never stalled.
    uint32_t now = millis();
    if ((uint32_t)(now - s_last_tick) >= TICK_MS)
    {
        s_last_tick = now;
        for (auto &ch : output_channels) ch.update(now);
    }

    // Debounced config fsync: flush to flash once writes have been idle for
    // CFG_SYNC_DEBOUNCE_MS. Armed only by SyncedFileMemorySpace::write(), so a
    // read never triggers it. Runs here (loop thread), not on the executor.
    if (s_cfg_dirty &&
        (uint32_t)(millis() - s_cfg_dirty_ms) >= CFG_SYNC_DEBOUNCE_MS)
    {
        uint32_t armed = s_cfg_dirty_ms;
        if (s_cfg_fd >= 0) fsync(s_cfg_fd);
        // Clear only if no write landed during the fsync; else re-sync next pass.
        if (s_cfg_dirty_ms == armed) s_cfg_dirty = false;
    }
}
