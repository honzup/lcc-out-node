#ifndef _LCC_OUT_OMRN_CONFIG_H_
#define _LCC_OUT_OMRN_CONFIG_H_

// OpenMRNLite CDI for the 13-output "effects" LCC node (fleet node #2).
// Derived from the sibling lcc-8out-40action-node (BSD-3, (c) 2019 Mike Dunston
// upstream). This node is OUTPUTS ONLY — no inputs/producers.
//
// Each output channel is FIXED 1:1 to a board GPIO (no pin dropdown). It carries:
//   - a 48-char user Description (defaulted to "Output GPIOxx" at factory reset),
//   - an EFFECT dropdown (0=Off, 1=Steady, 2=Random, 3=Flicker),
//   - an on/off event pair.
// The firmware routes the on/off events into the channel's commanded state and a
// non-blocking effect engine (see OutputChannel in main.cpp) rather than a direct
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
/// 0x1820 = 13-output effects layout (desc + effect + on/off events).
static constexpr uint16_t CANONICAL_VERSION = 0x1820;

// ---- Effect selector dropdown (stored value = effect id) --------------------
// KEEP IN SYNC with the EFF_* constants in main.cpp.
#define EFFECT_MAP \
    "<relation><property>0</property><value>Off (disabled)</value></relation>" \
    "<relation><property>1</property><value>Steady</value></relation>" \
    "<relation><property>2</property><value>Random (building lights)</value></relation>" \
    "<relation><property>3</property><value>Flicker (fire)</value></relation>"

/// One output channel: 48-char description, effect dropdown, on/off event pair.
/// The GPIO is fixed by the channel's position (see OUTPUT_PINS in main.cpp), so
/// there is no pin field here.
CDI_GROUP(OutputConfig);
CDI_GROUP_ENTRY(description, StringConfigEntry<48>, //
    Name("Description"),
    Description("User name of this output (defaults to its fixed GPIO)."));
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
