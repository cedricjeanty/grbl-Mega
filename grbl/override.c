
#include "grbl.h"

const uint8_t feed_analog_pin = 0;
const uint8_t rapid_analog_pin = 1;
const uint8_t spindle_analog_pin = 2;

uint8_t get_feed_override(void)
{
	uint16_t val = analog_read(feed_analog_pin);
	uint8_t feed_override =  map(val, 0, 1023, MIN_FEED_RATE_OVERRIDE, MAX_FEED_RATE_OVERRIDE);
	return feed_override;
}

uint8_t get_rapid_override(void)
{
	uint16_t val = analog_read(rapid_analog_pin);
	uint8_t rapid_override =  map(val, 0, 1023, MIN_RAPID_OVERRIDE, MAX_RAPID_OVERRIDE);
	return rapid_override;
}

uint8_t get_spindle_override(void)
{
	uint16_t val = analog_read(spindle_analog_pin);
	uint8_t spindle_override =  map(val, 0, 1023, MIN_SPINDLE_SPEED_OVERRIDE, MAX_SPINDLE_SPEED_OVERRIDE);
	return spindle_override;
}