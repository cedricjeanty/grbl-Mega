
#include "grbl.h"

#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1)

uint8_t hmi_rx_buffer[RX_RING_BUFFER];
uint8_t hmi_rx_buffer_head = 0;
volatile uint8_t hmi_rx_buffer_tail = 0;

uint8_t hmi_tx_buffer[TX_RING_BUFFER];
uint8_t hmi_tx_buffer_head = 0;
volatile uint8_t hmi_tx_buffer_tail = 0;


// Returns the number of bytes available in the RX serial buffer.
uint8_t hmi_get_rx_buffer_available()
{
  uint8_t rtail = hmi_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (hmi_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (hmi_rx_buffer_head-rtail)); }
  return((rtail-hmi_rx_buffer_head-1));
}


// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t hmi_get_rx_buffer_count()
{
  uint8_t rtail = hmi_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (hmi_rx_buffer_head >= rtail) { return(hmi_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-hmi_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t hmi_get_tx_buffer_count()
{
  uint8_t ttail = hmi_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (hmi_tx_buffer_head >= ttail) { return(hmi_tx_buffer_head-ttail); }
  return (TX_RING_BUFFER - (ttail-hmi_tx_buffer_head));
}


void hmi_init()
{
  // Set baud rate
  #if HMI_BAUD_RATE < 57600
    uint16_t UBRR1_value = ((F_CPU / (8L * HMI_BAUD_RATE)) - 1)/2 ;
    UCSR1A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR1_value = ((F_CPU / (4L * HMI_BAUD_RATE)) - 1)/2;
    UCSR1A |= (1 << U2X1);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR1H = UBRR1_value >> 8;
  UBRR1L = UBRR1_value;

  // enable rx, tx, and interrupt on complete reception of a byte
  UCSR1B |= (1<<RXEN0 | 1<<TXEN1 | 1<<RXCIE1);

  // defaults to 8-bit, no parity, 1 stop bit
}


// Writes one byte to the TX serial buffer. Called by main program.
void hmi_write(uint8_t data) {
  // Calculate next head
  uint8_t next_head = hmi_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == hmi_tx_buffer_tail) {
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  hmi_tx_buffer[hmi_tx_buffer_head] = data;
  hmi_tx_buffer_head = next_head;

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR1B |=  (1 << UDRIE1);
}


// Data Register Empty Interrupt handler
ISR(HMI_UDRE)
{
  uint8_t tail = hmi_tx_buffer_tail; // Temporary hmi_tx_buffer_tail (to optimize for volatile)

  // Send a byte from the buffer
  UDR1 = hmi_tx_buffer[tail];

  // Update tail position
  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  hmi_tx_buffer_tail = tail;

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == hmi_tx_buffer_head) { UCSR1B &= ~(1 << UDRIE1); }
}


// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t hmi_read()
{
  uint8_t tail = hmi_rx_buffer_tail; // Temporary hmi_rx_buffer_tail (to optimize for volatile)
  if (hmi_rx_buffer_head == tail) {
    return HMI_NO_DATA;
  } else {
    uint8_t data = hmi_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { tail = 0; }
    hmi_rx_buffer_tail = tail;

    return data;
  }
}


ISR(HMI_RX)
{
  uint8_t data = UDR1;
  uint8_t next_head;

  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the main buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    case CMD_STATUS_REPORT: hmi_set_state_flag(EXEC_STATUS_REPORT); break; // Set as true//system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
    default :
      if (data > 0x7F) { // Real-time control characters are extended ACSII only.
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
          #endif
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
        }
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      } else { // Write character to buffer
        next_head = hmi_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // Write data to buffer unless it is full.
        if (next_head != hmi_rx_buffer_tail) {
          hmi_rx_buffer[hmi_rx_buffer_head] = data;
          hmi_rx_buffer_head = next_head;
        }
      }
  }
}


void hmi_reset_read_buffer()
{
  hmi_rx_buffer_tail = hmi_rx_buffer_head;
}

void hmi_set_state_flag(uint8_t mask) {
  uint8_t sreg = SREG;
  cli();
  hmi_rt_exec_state |= (mask);
  SREG = sreg;
}

void hmi_clear_state_flag(uint8_t mask) {
  uint8_t sreg = SREG;
  cli();
  hmi_rt_exec_state &= ~(mask);
  SREG = sreg;
}

// Print a string stored in PGM-memory
void hmi_printPgmString(const char *s)
{
  char c;
  while ((c = pgm_read_byte_near(s++)))
    hmi_write(c);
}

// Prints an uint8 variable in base 10.
void hmi_print_uint8_base10(uint8_t n)
{
  uint8_t digit_a = 0;
  uint8_t digit_b = 0;
  if (n >= 100) { // 100-255
    digit_a = '0' + n % 10;
    n /= 10;
  }
  if (n >= 10) { // 10-99
    digit_b = '0' + n % 10;
    n /= 10;
  }
  hmi_write('0' + n);
  if (digit_b) { hmi_write(digit_b); }
  if (digit_a) { hmi_write(digit_a); }
}

 // Prints real-time data. This function grabs a real-time snapshot of the stepper subprogram
 // and the actual location of the CNC machine. Users may change the following function to their
 // specific needs, but the desired real-time data report must be as short as possible. This is
 // requires as it minimizes the computational overhead and allows grbl to keep running smoothly,
 // especially during g-code programs with fast, short line segments and high frequency reports (5-20Hz).
void hmi_report_realtime_status()
{

  // Report current machine state and sub-states
  hmi_write('<');
  switch (sys.state) {
    case STATE_IDLE: hmi_printPgmString(PSTR("Idle")); break;
    case STATE_CYCLE: hmi_printPgmString(PSTR("Run")); break;
    case STATE_HOLD:
      if (!(sys.suspend & SUSPEND_JOG_CANCEL)) {
        hmi_printPgmString(PSTR("Hold:"));
        if (sys.suspend & SUSPEND_HOLD_COMPLETE) { hmi_write('0'); } // Ready to resume
        else { hmi_write('1'); } // Actively holding
        break;
      } // Continues to print jog state during jog cancel.
    case STATE_JOG: hmi_printPgmString(PSTR("Jog")); break;
    case STATE_HOMING: hmi_printPgmString(PSTR("Home")); break;
    case STATE_ALARM: hmi_printPgmString(PSTR("Alarm")); break;
    case STATE_CHECK_MODE: hmi_printPgmString(PSTR("Check")); break;
    case STATE_SAFETY_DOOR:
      hmi_printPgmString(PSTR("Door:"));
      if (sys.suspend & SUSPEND_INITIATE_RESTORE) {
        hmi_write('3'); // Restoring
      } else {
        if (sys.suspend & SUSPEND_RETRACT_COMPLETE) {
          if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR) {
            hmi_write('1'); // Door ajar
          } else {
            hmi_write('0');
          } // Door closed and ready to resume
        } else {
          hmi_write('2'); // Retracting
        }
      }
      break;
    case STATE_SLEEP: hmi_printPgmString(PSTR("Sleep")); break;
  }

  hmi_printPgmString(PSTR("|Ov:"));
  hmi_print_uint8_base10(sys.f_override);
  hmi_write(',');
  hmi_print_uint8_base10(sys.r_override);
  hmi_write(',');
  hmi_print_uint8_base10(sys.spindle_speed_ovr);

  hmi_write('>');
  hmi_write('\n');
}