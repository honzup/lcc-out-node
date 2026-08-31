#ifndef _LCC_OUT_OMRN_CONFIG_H_
#define _LCC_OUT_OMRN_CONFIG_H_

// OpenMRNLite CDI for the 13-output "effects" LCC node (fleet node #2).
// Derived from the sibling lcc-8out-40action-node (BSD-3, (c) 2019 Mike Dunston
// upstream). This node is OUTPUTS ONLY — no inputs/producers.
//
// Each output channel carries:
//   - a PIN dropdown (which board GPIO this channel drives; None = disabled,
//     defaulted at factory reset to the channel's natural GPIO),
//   - a 16-char user Description,
//   - an EFFECT dropdown (0=Off, 1=Steady, 2=Random, 3=Flicker),
//   - an on/off event pair.
// The 13 channels are a RepeatedGroup so JMRI shows the per-output tabs; the pin
// is a selectable field inside each tab (like node #1's lcc-8out-40action-node).
// The firmware reads the selected pin at config-load, retargets its DynamicGpio-
// style effect engine to that GPIO, and routes the on/off events into the
// channel's commanded state (see OutputChannel in main.cpp) rather than a direct
// GPIO write.

#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/MemoryConfig.hxx"

#if !defined(USE_CAN) && !defined(USE_WIFI)
#error "Invalid configuration detected, USE_CAN or USE_WIFI must be defined."
#endif

namespace openlcb
{

extern const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4,
    "OpenLCB",
    "13-Output Effects (OpenMRNLite)",
    ARDUINO_VARIANT,
    "1.00"};

constexpr uint8_t NUM_OUTPUTS = 13;   // consumers, fixed 1:1 to GPIOs

/// Bump this whenever the config layout changes, to force a clean reformat.
/// 0x1820 = 13-output effects layout (48-char desc, RepeatedGroup).
/// 0x1821 = 16-char desc + individually-named per-GPIO groups.
/// 0x1822 = 16-char desc, back to RepeatedGroup (JMRI tabbed display).
/// 0x1823 = added per-output pin dropdown (selectable GPIO inside each tab).
static constexpr uint16_t CANONICAL_VERSION = 0x1823;

// ---- Pin-selector dropdown (stored value = <property> index; label = GPIO) --
// The firmware maps these property indices to GPIOs via OUTPUT_PINS[] in
// main.cpp — KEEP THE TWO IN SYNC. Index 0 = None (channel disabled).
#define OUTPUT_PIN_MAP \
    "<relation><property>0</property><value>None</value></relation>" \
    "<relation><property>1</property><value>13</value></relation>" \
    "<relation><property>2</property><value>14</value></relation>" \
    "<relation><property>3</property><value>15</value></relation>" \
    "<relation><property>4</property><value>18</value></relation>" \
    "<relation><property>5</property><value>19</value></relation>" \
    "<relation><property>6</property><value>21</value></relation>" \
    "<relation><property>7</property><value>22</value></relation>" \
    "<relation><property>8</property><value>23</value></relation>" \
    "<relation><property>9</property><value>25</value></relation>" \
    "<relation><property>10</property><value>26</value></relation>" \
    "<relation><property>11</property><value>27</value></relation>" \
    "<relation><property>12</property><value>32</value></relation>" \
    "<relation><property>13</property><value>33</value></relation>"

// ---- Effect selector dropdown (stored value = effect id) --------------------
// KEEP IN SYNC with the EFF_* constants in main.cpp.
#define EFFECT_MAP \
    "<relation><property>0</property><value>Off (disabled)</value></relation>" \
    "<relation><property>1</property><value>Steady</value></relation>" \
    "<relation><property>2</property><value>Random (building lights)</value></relation>" \
    "<relation><property>3</property><value>Flicker (fire)</value></relation>"

/// One output channel: pin dropdown, 16-char description, effect dropdown, on/off
/// event pair. The pin is selectable at runtime (see OutputChannel in main.cpp).
CDI_GROUP(OutputConfig);
CDI_GROUP_ENTRY(pin, Uint8ConfigEntry, Default(0), //
    Name("Output GPIO"),
    Description("Which board GPIO this channel drives. None = disabled. "
                "Defaults to this channel's natural GPIO."),
    MapValues(OUTPUT_PIN_MAP));
CDI_GROUP_ENTRY(description, StringConfigEntry<16>, //
    Name("Description"),
    Description("User name of this output."));
CDI_GROUP_ENTRY(effect, Uint8ConfigEntry, Default(0), //
    Name("Effect"),
    Description("What 'On' does. Off = channel disabled (pin stays LOW). "
                "Steady = GPIO HIGH. Random = building-occupancy blinking. "
                "Flicker = warm PWM fire flicker (allocates an LEDC channel)."),
    MapValues(EFFECT_MAP));
CDI_GROUP_ENTRY(event_on, EventConfigEntry, //
    Name("Event On"), Description("Commands this output ON (per its effect)."));
CDI_GROUP_ENTRY(event_off, EventConfigEntry, //
    Name("Event Off"), Description("Commands this output OFF (forces pin LOW)."));
CDI_GROUP_END();

using AllOutputs = RepeatedGroup<OutputConfig, NUM_OUTPUTS>;

CDI_GROUP(IoBoardSegment, Segment(MemoryConfigDefs::SPACE_CONFIG), Offset(128));
CDI_GROUP_ENTRY(internal_config, InternalConfigData);
CDI_GROUP_ENTRY(outputs, AllOutputs, Name("Outputs"), RepName("Output"));
CDI_GROUP_END();

CDI_GROUP(ConfigDef, MainCdi());
CDI_GROUP_ENTRY(ident, Identification);
CDI_GROUP_ENTRY(acdi, Acdi);
CDI_GROUP_ENTRY(userinfo, UserInfoSegment, Name("User Info"));
CDI_GROUP_ENTRY(seg, IoBoardSegment, Name("Settings"));
CDI_GROUP_END();

} // namespace openlcb

#endif // _LCC_OUT_OMRN_CONFIG_H_
