#ifndef hmi_h
#define hmi_h


#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 255
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 255
#endif

#define HMI_NO_DATA 0xff


void hmi_init();

// Writes one byte to the TX serial buffer. Called by main program.
void hmi_write(uint8_t data);

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t hmi_read();

// Reset and empty data in read buffer. Used by e-stop and reset.
void hmi_reset_read_buffer();

// Returns the number of bytes available in the RX serial buffer.
uint8_t hmi_get_rx_buffer_available();

// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t hmi_get_rx_buffer_count();

// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t hmi_get_tx_buffer_count();

void hmi_set_state_flag(uint8_t mask);

void hmi_clear_state_flag(uint8_t mask);

void hmi_printPgmString(const char *s);

void hmi_report_realtime_status();

#endif
