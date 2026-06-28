#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>

// MSX CONNECTOR

#define MSX_RIGHT PC2
#define MSX_RIGHT_DDR DDRC
#define MSX_RIGHT_PORT PORTC

#define MSX_LEFT PC3
#define MSX_LEFT_DDR DDRC
#define MSX_LEFT_PORT PORTC

#define MSX_DOWN PC4
#define MSX_DOWN_DDR DDRC
#define MSX_DOWN_PORT PORTC

#define MSX_UP PC5
#define MSX_UP_DDR DDRC
#define MSX_UP_PORT PORTC

#define MSX_TRG_1 PD1
#define MSX_TRG_1_PORT PORTD
#define MSX_TRG_1_DDR DDRD

#define MSX_TRG_2 PD2
#define MSX_TRG_2_PORT PORTD
#define MSX_TRG_2_DDR DDRD

#define MSX_OUT PD0
#define MSX_OUT_PORT PORTD
#define MSX_OUT_DDR DDRD
#define MSX_OUT_PIN PIND

#define MSX_C_MASK ((1 << MSX_RIGHT) | (1 << MSX_LEFT) | (1 << MSX_DOWN) | (1 << MSX_UP))
#define MSX_D_MASK ((1 << MSX_TRG_1) | (1 << MSX_TRG_2))

// JOYMEGA CONNECTOR

#define JOY_LEFT PB0
#define JOY_LEFT_DDR DDRB
#define JOY_LEFT_PORT PORTB
#define JOY_LEFT_PIN PINB

#define JOY_RIGHT PB1
#define JOY_RIGHT_DDR DDRB
#define JOY_RIGHT_PORT PORTB
#define JOY_RIGHT_PIN PINB

#define JOY_UP PD6
#define JOY_UP_DDR DDRD
#define JOY_UP_PORT PORTD
#define JOY_UP_PIN PIND

#define JOY_DOWN PD7
#define JOY_DOWN_DDR DDRD
#define JOY_DOWN_PORT PORTD
#define JOY_DOWN_PIN PIND

#define JOY_TRG1AB PB7
#define JOY_TRG1AB_DDR DDRB
#define JOY_TRG1AB_PORT PORTB
#define JOY_TRG1AB_PIN PINB

#define JOY_TRG2CSTART PB2
#define JOY_TRG2CSTART_DDR DDRB
#define JOY_TRG2CSTART_PORT PORTB
#define JOY_TRG2CSTART_PIN PINB

#define JOY_THSEL PD5
#define JOY_THSEL_DDR DDRD
#define JOY_THSEL_PORT PORTD

#define JOY_READ_PHASE_LENGTH_US 5
#define JOY_READ_SEQUENCE_INTERVAL_MS 4

// LEDS

#define LED_RED PC1
#define LED_RED_PORT PORTC
#define LED_RED_DDR DDRC

#define LED_BLUE PD3
#define LED_BLUE_PORT PORTD
#define LED_BLUE_DDR DDRD

// GLOBAL VARIABLES

#define SEQ_MSX_PHASES 8
#define SEQ_PHASE_DEFAULT 8
#define MSX_OUT_SEQUENCE_TIMEOUT_CNTS 45        // 45 counts of 32 us = 1440 us

volatile uint8_t seq_msx_phase_counter = 1;

// JOYMEGA state variables, true = not pressed, false = pressed (because of pull-ups)

bool joy_up_state = true;
bool joy_down_state = true;
bool joy_left_state = true;
bool joy_right_state = true;
bool joy_trg_a_state = true;
bool joy_trg_b_state = true;
bool joy_trg_c_state = true;
bool joy_trg_x_state = true;
bool joy_trg_y_state = true;
bool joy_trg_z_state = true;
bool joy_trg_start_state = true;
bool joy_trg_mode_state = true;

bool joy_is_3_keys = false;
bool joy_is_6_keys = false;

volatile uint8_t msx_portc_phase[SEQ_MSX_PHASES];
volatile uint8_t msx_portd_phase[SEQ_MSX_PHASES];

volatile uint8_t portc_phase_full[SEQ_MSX_PHASES];

volatile bool led_red_state = false;
volatile bool led_blue_state = false;

#define DEFAULT_STATE_TIMEOUT_CYCLES 700     // number of cycles after which default state is re-entered if no MSX OUT change occurs (700 cycles = 700 * 1440 us = 1008000 us)

volatile bool default_state = true;
volatile uint16_t default_state_counter = 0;

#define PHASED_STATE_MODE_OFF 0
#define PHASED_STATE_MODE_MEGADRIVE 1
#define PHASED_STATE_MODE_MOUSE 2

volatile uint8_t phased_state_mode = PHASED_STATE_MODE_MOUSE;
volatile bool phased_mode_changed = true;

#define MAX_MOUSE_XY_SPEED 10
#define MAX_MOUSE_SLOW_XY_SPEED 2
#define MAX_MOUSE_XY_MIN_SPEED 2

#define MOUSE_XY_STEP 1.8
#define MOUSE_SLOW_XY_STEP 1

#define MOUSE_Z_STEP 1

#define MAX_MOUSE_Z_SPEED 1

#define MAX_MOUSE_Z_LONGPRESS_SPEED 3

#define MOUSE_DZ_LONGPRESS_THRESHOLD_CYCLES 120     // number of cycles after which long-press is registered for Z axis in mouse mode (120 cycles = 120 * 1440 us = 172800 us)
volatile uint8_t mouse_dz_counter_longpress = 0;

volatile float mouse_dx = 0;
volatile float mouse_dy = 0;
volatile float mouse_dz = 0;

volatile uint8_t led_blink_counter = 0;

#define AUTOFIRE_SPEED_DISABLED 0
#define AUTOFIRE_SPEED_SLOW 1           // 3.75 clicks per second (1 click per 512 cycles)
#define AUTOFIRE_SPEED_MEDIUM 2         // 7.5 clicks per second (1 click per 256 cycles)
#define AUTOFIRE_SPEED_FAST 3           // 15 clicks per second (1 click per 128 cycles)

volatile uint8_t default_mode_autofire_counter = 0;
volatile uint8_t default_mode_trg_1_autofire = AUTOFIRE_SPEED_DISABLED;
volatile uint8_t default_mode_trg_2_autofire = AUTOFIRE_SPEED_DISABLED;
volatile bool default_mode_trg_1_autofire_fire = false;
volatile bool default_mode_trg_2_autofire_fire = false;
volatile bool default_mode_trg_1_autofire_mode_changed = false;
volatile bool default_mode_trg_2_autofire_mode_changed = false;

// Allocate variable in EEPROM
uint8_t EEMEM settings_phased_state_mode = 0x00; 
uint16_t phased_state_mode_store_counter = 0x0000;
#define PHASED_STATE_MODE_STORE_CYCLES 480          // about 2 seconds with 4 ms JOY read sequence interval

void setup(void);
void timer0_init(void);
void timer1_init(void);
void read_joy_states(uint8_t);
void set_th_high(void);
void set_th_low(void);
void build_msx_phase_buffers_default_mode(void);
void build_msx_phase_buffers_megadrive_mode(void);
void build_msx_phase_buffers_mouse_mode(void);
void build_msx_phase_buffers_off_mode(void);
void msx_out_interrupt_init(void);
void disable_and_reset_autofire(void);
static inline int8_t fast_jitter_soft(void);

ISR(TIMER0_COMPA_vect)
{

    if (MSX_OUT_PIN & (1 << MSX_OUT)) {
        PORTD = (PORTD & ~MSX_D_MASK) | msx_portd_phase[1];
        PORTC = portc_phase_full[1];
        seq_msx_phase_counter = 0;
    } else {
        PORTD = (PORTD & ~MSX_D_MASK) | msx_portd_phase[0];
        PORTC = portc_phase_full[0];
        seq_msx_phase_counter = 1;
    }

    if (!default_state) {

        if (default_state_counter == 0) {                       // read sequence just ended (not default state and counter is 0)
            mouse_dx = 0;
            mouse_dy = 0;
            mouse_dz = 0;
        }

        if (++default_state_counter >= DEFAULT_STATE_TIMEOUT_CYCLES) {
            default_state = true;
            default_state_counter = 0;
        }

    }

    PCIFR = (1 << PCIF2);
}

ISR(PCINT2_vect, ISR_NAKED)
{
    asm volatile(
        // save used regs + SREG
        "push r24              \n\t"
        "push r25              \n\t"
        "push r30              \n\t"
        "push r31              \n\t"
        "in   r24, __SREG__    \n\t"
        "push r24              \n\t"

        // r24 = seq
        "lds  r24, seq_msx_phase_counter \n\t"

        // ----- PORTD = (PORTD & ~MSX_D_MASK) | msx_portd_phase[seq]

        "ldi  r30, lo8(msx_portd_phase)  \n\t"
        "ldi  r31, hi8(msx_portd_phase)  \n\t"
        "add  r30, r24                   \n\t"
        "adc  r31, __zero_reg__          \n\t"
        "ld   r25, Z                     \n\t" // r25 = msx_portd_phase[seq]

        "in   r30, %[portd]              \n\t"
        "andi r30, 0xF9                  \n\t" // WARNING: hardcoded for MSX_TRG_1=PD1 and MSX_TRG_2=PD2; 0xF9 = ~(PD1 | PD2)
        "or   r30, r25                   \n\t"
        "out  %[portd], r30              \n\t"

        // ----- PORTC = portc_phase_full[seq]

        "ldi  r30, lo8(portc_phase_full) \n\t"
        "ldi  r31, hi8(portc_phase_full) \n\t"
        "add  r30, r24                   \n\t"
        "adc  r31, __zero_reg__          \n\t"
        "ld   r25, Z                     \n\t"
        "out  %[portc], r25              \n\t"

        // ----- seq = (seq + 1) & 7

        "inc  r24                        \n\t"
        "andi r24, 7                     \n\t"
        "sts  seq_msx_phase_counter, r24 \n\t"

        // default_state = false
        "clr r30                         \n\t"
        "sts default_state, r30          \n\t"

        // default_state_counter = 0
        "clr r30                           \n\t"
        "sts default_state_counter, r30    \n\t"
        "sts default_state_counter+1, r30  \n\t"

        // ----- TCNT0 = 0

        "out  %[tcnt0], __zero_reg__     \n\t"

        // ----- clear PCIF2 pending flag

        "ldi  r24, %[pcif2_mask]         \n\t"
        "out  %[pcifr], r24              \n\t"

        // restore SREG + regs
        "pop  r24              \n\t"
        "out  __SREG__, r24    \n\t"
        "pop  r31              \n\t"
        "pop  r30              \n\t"
        "pop  r25              \n\t"
        "pop  r24              \n\t"
        "reti                  \n\t"
        :
        : [portd] "I" (_SFR_IO_ADDR(PORTD)),
          [portc] "I" (_SFR_IO_ADDR(PORTC)),
          [tcnt0] "I" (_SFR_IO_ADDR(TCNT0)),
          [pcifr] "I" (_SFR_IO_ADDR(PCIFR)),
          [pcif2_mask] "M" (1 << PCIF2)
    );
}

int main(void)
{
    setup();
    timer0_init();
    timer1_init();
    msx_out_interrupt_init();

    sei();

    while (1) {

        set_th_high();
        _delay_us(JOY_READ_PHASE_LENGTH_US);
        read_joy_states(0);     // Phase 0: read UP, DOWN, LEFT, RIGHT, TRG_B, TRG_C

        set_th_low();
        _delay_us(JOY_READ_PHASE_LENGTH_US);
        read_joy_states(1);     // Phase 1: read TRG_A, TRG_START

        set_th_high();
        _delay_us(JOY_READ_PHASE_LENGTH_US);

        set_th_low();
        _delay_us(JOY_READ_PHASE_LENGTH_US);

        set_th_high();
        _delay_us(JOY_READ_PHASE_LENGTH_US);

        set_th_low();
        _delay_us(JOY_READ_PHASE_LENGTH_US);
        read_joy_states(5);     // Phase 5: 3/6 keys detection

        set_th_high();
        _delay_us(JOY_READ_PHASE_LENGTH_US);
        read_joy_states(6);     // Phase 6: if 6 keys, read TRG_X, TRG_Y, TRG_Z, TRG_MODE

        set_th_low();
        _delay_us(JOY_READ_PHASE_LENGTH_US);

        set_th_high();

        if (default_state) {

            default_mode_autofire_counter++;

            if (phased_state_mode == PHASED_STATE_MODE_MOUSE) {
                led_red_state = true;
                led_blue_state = false;
            } else if (phased_state_mode == PHASED_STATE_MODE_MEGADRIVE) {
                led_red_state = false;
                led_blue_state = true;
            } else {
                led_red_state = true;
                led_blue_state = true;
            }
            
            led_blink_counter = 0;

        } else if (phased_state_mode == PHASED_STATE_MODE_MOUSE) {

            led_blink_counter++;
            if (led_blink_counter & (1 << 7)) {
                led_red_state = true;
            } else {
                led_red_state = false;
            }
            led_blue_state = false;

        } else if (phased_state_mode == PHASED_STATE_MODE_MEGADRIVE) {

            led_blink_counter++;
            led_red_state = false;
            if (led_blink_counter & (1 << 7)) {
                led_blue_state = true;
            } else {
                led_blue_state = false;
            }

        } else {

            led_red_state = false;
            led_blue_state = false;
            
        }

        if (led_red_state) {
            LED_RED_PORT |= (1 << LED_RED);
        } else {
            LED_RED_PORT &= ~(1 << LED_RED);
        }

        if (led_blue_state) {
            LED_BLUE_PORT |= (1 << LED_BLUE);
        } else {
            LED_BLUE_PORT &= ~(1 << LED_BLUE);
        }

        if (default_state) {

            if (!joy_trg_x_state && !default_mode_trg_1_autofire_mode_changed) {
                switch (default_mode_trg_1_autofire){
                    case AUTOFIRE_SPEED_SLOW:
                        default_mode_trg_1_autofire = AUTOFIRE_SPEED_MEDIUM;
                        default_mode_trg_1_autofire_fire = false;
                        break;
                    case AUTOFIRE_SPEED_MEDIUM:
                        default_mode_trg_1_autofire = AUTOFIRE_SPEED_FAST;
                        default_mode_trg_1_autofire_fire = false;
                        break;
                    case AUTOFIRE_SPEED_FAST:
                        default_mode_trg_1_autofire = AUTOFIRE_SPEED_DISABLED;
                        default_mode_trg_1_autofire_fire = false;
                        break;
                    case AUTOFIRE_SPEED_DISABLED:
                        default_mode_trg_1_autofire = AUTOFIRE_SPEED_SLOW;
                        default_mode_trg_1_autofire_fire = false;
                        break;
                    default:
                        default_mode_trg_1_autofire = AUTOFIRE_SPEED_DISABLED;
                        default_mode_trg_1_autofire_fire = false;
                        break;
                }
                default_mode_trg_1_autofire_mode_changed = true;
            } else if (joy_trg_x_state) {
                default_mode_trg_1_autofire_mode_changed = false;
            }

            if (!joy_trg_y_state && !default_mode_trg_2_autofire_mode_changed) {
                switch (default_mode_trg_2_autofire){
                    case AUTOFIRE_SPEED_SLOW:
                        default_mode_trg_2_autofire = AUTOFIRE_SPEED_MEDIUM;
                        default_mode_trg_2_autofire_fire = false;
                        break;
                    case AUTOFIRE_SPEED_MEDIUM:
                        default_mode_trg_2_autofire = AUTOFIRE_SPEED_FAST;
                        default_mode_trg_2_autofire_fire = false;
                        break;
                    case AUTOFIRE_SPEED_FAST:
                        default_mode_trg_2_autofire = AUTOFIRE_SPEED_DISABLED;
                        default_mode_trg_2_autofire_fire = false;
                        break;
                    case AUTOFIRE_SPEED_DISABLED:
                        default_mode_trg_2_autofire = AUTOFIRE_SPEED_SLOW;
                        default_mode_trg_2_autofire_fire = false;
                        break;
                    default:
                        default_mode_trg_2_autofire = AUTOFIRE_SPEED_DISABLED;
                        default_mode_trg_2_autofire_fire = false;
                        break;
                }
                default_mode_trg_2_autofire_mode_changed = true;
            } else if (joy_trg_y_state) {
                default_mode_trg_2_autofire_mode_changed = false;
            }

            if (!joy_trg_z_state) {
                disable_and_reset_autofire();
            }

            if (!joy_trg_a_state) {
                default_mode_trg_1_autofire = AUTOFIRE_SPEED_DISABLED;
                default_mode_trg_1_autofire_fire = false;
            }

            if (!(joy_trg_b_state && joy_trg_c_state)) {
                default_mode_trg_2_autofire = AUTOFIRE_SPEED_DISABLED;
                default_mode_trg_2_autofire_fire = false;
            }

            switch (default_mode_trg_1_autofire){
                case AUTOFIRE_SPEED_SLOW:
                    if ((default_mode_autofire_counter & (1 << 5)) == 0) {
                        default_mode_trg_1_autofire_fire = true;
                    } else default_mode_trg_1_autofire_fire = false;
                    break;
                case AUTOFIRE_SPEED_MEDIUM:
                    if ((default_mode_autofire_counter & (1 << 4)) == 0) {
                        default_mode_trg_1_autofire_fire = true;
                    } else default_mode_trg_1_autofire_fire = false;
                    break;
                case AUTOFIRE_SPEED_FAST:
                    if ((default_mode_autofire_counter & (1 << 3)) == 0) {
                        default_mode_trg_1_autofire_fire = true;
                    } else default_mode_trg_1_autofire_fire = false;
                    break;
                default:
                    default_mode_trg_1_autofire_fire = false;
                    break;
            }

            switch (default_mode_trg_2_autofire){
                case AUTOFIRE_SPEED_SLOW:
                    if ((default_mode_autofire_counter & (1 << 5)) == 0) {
                        default_mode_trg_2_autofire_fire = true;
                    } else default_mode_trg_2_autofire_fire = false;
                    break;
                case AUTOFIRE_SPEED_MEDIUM:
                    if ((default_mode_autofire_counter & (1 << 4)) == 0) {
                        default_mode_trg_2_autofire_fire = true;
                    } else default_mode_trg_2_autofire_fire = false;
                    break;
                case AUTOFIRE_SPEED_FAST:
                    if ((default_mode_autofire_counter & (1 << 3)) == 0) {
                        default_mode_trg_2_autofire_fire = true;
                    } else default_mode_trg_2_autofire_fire = false;
                    break;
                default:
                    default_mode_trg_2_autofire_fire = false;
                    break;
            }
        }

        if (default_state) {
            if ((!joy_trg_mode_state) && (!phased_mode_changed)) {

                if (phased_state_mode == PHASED_STATE_MODE_MOUSE) {
                    phased_state_mode = PHASED_STATE_MODE_MEGADRIVE;
                } else if (phased_state_mode == PHASED_STATE_MODE_MEGADRIVE) {
                    phased_state_mode = PHASED_STATE_MODE_OFF;
                } else {
                    phased_state_mode = PHASED_STATE_MODE_MOUSE;
                }

                phased_mode_changed = true;

            } else if (joy_trg_mode_state) {
                if (phased_mode_changed) {
                    phased_state_mode_store_counter = PHASED_STATE_MODE_STORE_CYCLES;
                }
                phased_mode_changed = false;
            }
            build_msx_phase_buffers_default_mode();
        } else {
            disable_and_reset_autofire();

            if (phased_state_mode == PHASED_STATE_MODE_MOUSE) {

                if (!joy_trg_c_state) {
                    if (!joy_left_state && (mouse_dx < MAX_MOUSE_SLOW_XY_SPEED)) mouse_dx+=MOUSE_SLOW_XY_STEP;
                    if (!joy_right_state && (mouse_dx > -MAX_MOUSE_SLOW_XY_SPEED)) mouse_dx-=MOUSE_SLOW_XY_STEP;
                    if (!joy_up_state && (mouse_dy < MAX_MOUSE_SLOW_XY_SPEED)) mouse_dy+=MOUSE_SLOW_XY_STEP;
                    if (!joy_down_state && (mouse_dy > -MAX_MOUSE_SLOW_XY_SPEED)) mouse_dy-=MOUSE_SLOW_XY_STEP;
                } else {
                    if (!joy_left_state && (mouse_dx < MAX_MOUSE_XY_SPEED)) mouse_dx+=MOUSE_XY_STEP;
                    if (!joy_right_state && (mouse_dx > -MAX_MOUSE_XY_SPEED)) mouse_dx-=MOUSE_XY_STEP;
                    if (!joy_up_state && (mouse_dy < MAX_MOUSE_XY_SPEED)) mouse_dy+=MOUSE_XY_STEP;
                    if (!joy_down_state && (mouse_dy > -MAX_MOUSE_XY_SPEED)) mouse_dy-=MOUSE_XY_STEP;
                }

                if (mouse_dz_counter_longpress > 0 && mouse_dz_counter_longpress < MOUSE_DZ_LONGPRESS_THRESHOLD_CYCLES) mouse_dz_counter_longpress++;

                if (!(joy_trg_z_state && joy_trg_y_state)) {
                    if (mouse_dz_counter_longpress == 0) {
                        if (!joy_trg_z_state && (mouse_dz < MAX_MOUSE_Z_SPEED)) mouse_dz += MOUSE_Z_STEP;
                        if (!joy_trg_y_state && (mouse_dz > -MAX_MOUSE_Z_SPEED)) mouse_dz -= MOUSE_Z_STEP;
                        mouse_dz_counter_longpress = 1;
                    } else if (mouse_dz_counter_longpress >= MOUSE_DZ_LONGPRESS_THRESHOLD_CYCLES) {
                        if (!joy_trg_z_state && (mouse_dz < MAX_MOUSE_Z_LONGPRESS_SPEED)) mouse_dz += MOUSE_Z_STEP;
                        if (!joy_trg_y_state && (mouse_dz > -MAX_MOUSE_Z_LONGPRESS_SPEED)) mouse_dz -= MOUSE_Z_STEP;
                    }
                } else if (joy_trg_z_state && joy_trg_y_state) {
                    mouse_dz_counter_longpress = 0;
                }

                build_msx_phase_buffers_mouse_mode();

            } else if (phased_state_mode == PHASED_STATE_MODE_MEGADRIVE) {
                build_msx_phase_buffers_megadrive_mode();
            } else {
                build_msx_phase_buffers_off_mode();
            }
        }

        if (phased_state_mode_store_counter > 0) {
            phased_state_mode_store_counter--;
            if (phased_state_mode_store_counter == 0) {
                eeprom_update_byte(&settings_phased_state_mode, phased_state_mode);
            }
        }

        _delay_ms(JOY_READ_SEQUENCE_INTERVAL_MS);
    }
}

void timer0_init(void)
{
    TCCR0A = 0;
    TCNT0 = 0;

    OCR0A = MSX_OUT_SEQUENCE_TIMEOUT_CNTS - 1;

    TCCR0A |= (1 << WGM01); // CTC
    TCCR0B |= (1 << CS02);  // prescaler 256, 1 tick = 32 us

    TIMSK0 |= (1 << OCIE0A);
}

void timer1_init(void)
{
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    // Normal mode, prescaler /8
    // 8 MHz / 8 = 1 MHz, czyli 1 tick = 1 us
    TCCR1B |= (1 << CS11);
}

void msx_out_interrupt_init(void)
{
    // wczyść ewentualną flagę
    PCIFR |= (1 << PCIF2);

    // włącz grupę PCINT[23:16]
    PCICR |= (1 << PCIE2);

    // włącz konkretny pin PCINT16
    PCMSK2 |= (1 << PCINT16);
}

void setup(void) {

    // CLOCK setup

    CLKPR = (1 << CLKPCE); // odblokowanie zmiany preskalera
    CLKPR = 0;             // dzielnik = 1, taktowanie 8 MHz

    // LED setup

    LED_RED_DDR |= (1 << LED_RED);   // LED_RED jako wyjście (czerwony)
    LED_RED_PORT |= (1 << LED_RED); // LED_RED jako WYSOKI (czerwony włączony)

    LED_BLUE_DDR |= (1 << LED_BLUE);   // LED_BLUE jako wyjście (niebieski)
    LED_BLUE_PORT |= (1 << LED_BLUE); // LED_BLUE jako WYSOKI (niebieski włączony)

    // MSX connector setup

    MSX_UP_DDR |= (1 << MSX_UP); // MSX UP jako wyjście
    MSX_UP_PORT |= (1 << MSX_UP); // MSX UP OUT jako WYSOKI

    MSX_DOWN_DDR |= (1 << MSX_DOWN); // MSX DOWN jako wyjście
    MSX_DOWN_PORT |= (1 << MSX_DOWN); // MSX DOWN OUT jako WYSOKI

    MSX_LEFT_DDR |= (1 << MSX_LEFT); // MSX LEFT jako wyjście
    MSX_LEFT_PORT |= (1 << MSX_LEFT); // MSX LEFT OUT jako WYSOKI

    MSX_RIGHT_DDR |= (1 << MSX_RIGHT); // MSX RIGHT jako wyjście
    MSX_RIGHT_PORT |= (1 << MSX_RIGHT); // MSX RIGHT OUT jako WYSOKI

    MSX_TRG_1_DDR |= (1 << MSX_TRG_1); // MSX TRG_1 jako wyjście
    MSX_TRG_1_PORT |= (1 << MSX_TRG_1); // MSX TRG_1 OUT jako WYSOKI

    MSX_TRG_2_DDR |= (1 << MSX_TRG_2); // MSX TRG_2 jako wyjście
    MSX_TRG_2_PORT |= (1 << MSX_TRG_2); // MSX TRG_2 OUT jako WYSOKI

    MSX_OUT_DDR &= ~(1 << MSX_OUT); // MSX OUT jako wejście
    MSX_OUT_PORT &= ~(1 << MSX_OUT); // MSX OUT bez podciągania

    // JOYMEGA connector setup

    JOY_LEFT_DDR &= ~(1 << JOY_LEFT); // JOY_LEFT jako wejście
    JOY_LEFT_PORT |= (1 << JOY_LEFT); // JOY_LEFT z podciąganiem
    
    JOY_RIGHT_DDR &= ~(1 << JOY_RIGHT); // JOY_RIGHT jako wejście
    JOY_RIGHT_PORT |= (1 << JOY_RIGHT); // JOY_RIGHT z podciąganiem
    
    JOY_UP_DDR &= ~(1 << JOY_UP); // JOY_UP jako wejście
    JOY_UP_PORT |= (1 << JOY_UP); // JOY_UP z podciąganiem
    
    JOY_DOWN_DDR &= ~(1 << JOY_DOWN); // JOY_DOWN jako wejście
    JOY_DOWN_PORT |= (1 << JOY_DOWN); // JOY_DOWN z podciąganiem
    
    JOY_TRG1AB_DDR &= ~(1 << JOY_TRG1AB); // JOY_TRG1AB jako wejście
    JOY_TRG1AB_PORT |= (1 << JOY_TRG1AB); // JOY_TRG1AB z podciąganiem
    
    JOY_TRG2CSTART_DDR &= ~(1 << JOY_TRG2CSTART); // JOY_TRG2CSTART jako wejście
    JOY_TRG2CSTART_PORT |= (1 << JOY_TRG2CSTART); // JOY_TRG2CSTART z podciąganiem
    
    JOY_THSEL_DDR |= (1 << JOY_THSEL); // JOY_THSEL jako wyjście
    JOY_THSEL_PORT |= (1 << JOY_THSEL); // JOY_THSEL jako WYSOKI

    // Load settings from EEPROM

    uint8_t phased_state_mode_loaded = eeprom_read_byte(&settings_phased_state_mode);

    if (phased_state_mode_loaded != PHASED_STATE_MODE_OFF && phased_state_mode_loaded != PHASED_STATE_MODE_MEGADRIVE && phased_state_mode_loaded != PHASED_STATE_MODE_MOUSE) {
        phased_state_mode = PHASED_STATE_MODE_MOUSE;
    } else {
        phased_state_mode = phased_state_mode_loaded;
    }
}

void set_th_high(void) {
    JOY_THSEL_PORT |= (1 << JOY_THSEL); // JOY_THSEL jako WYSOKI
}

void set_th_low(void) {
    JOY_THSEL_PORT &= ~(1 << JOY_THSEL); // JOY_THSEL jako NISKI
}

void read_joy_states(uint8_t joy_phase) {
    // JOYMEGA state reading phases

    switch (joy_phase)
    {
        case 0: // Phase 0: read UP, DOWN, LEFT, RIGHT, TRG_B, TRG_C

            if (JOY_UP_PIN & (1 << JOY_UP)) {
                joy_up_state = true;
            } else {
                joy_up_state = false;
            }

            if (JOY_DOWN_PIN & (1 << JOY_DOWN)) {
                joy_down_state = true;
            } else {
                joy_down_state = false;
            }

            if (JOY_LEFT_PIN & (1 << JOY_LEFT)) {
                joy_left_state = true;
            } else {
                joy_left_state = false;
            }

            if (JOY_RIGHT_PIN & (1 << JOY_RIGHT)) {
                joy_right_state = true;
            } else {
                joy_right_state = false;
            }

            if (JOY_TRG1AB_PIN & (1 << JOY_TRG1AB)) {
                joy_trg_b_state = true;
            } else {
                joy_trg_b_state = false;
            }

            if (JOY_TRG2CSTART_PIN & (1 << JOY_TRG2CSTART)) {
                joy_trg_c_state = true;
            } else {
                joy_trg_c_state = false;
            }

            break;

        case 1: // Phase 1: read TRG_A, TRG_START

            if (JOY_TRG1AB_PIN & (1 << JOY_TRG1AB)) {
                joy_trg_a_state = true;
            } else {
                joy_trg_a_state = false;
            }

            if (JOY_TRG2CSTART_PIN & (1 << JOY_TRG2CSTART)) {
                joy_trg_start_state = true;
            } else {
                joy_trg_start_state = false;
            }

            break;

        case 5: // Phase 5: 3/6 keys detection

            if ((JOY_UP_PIN & (1 << JOY_UP)) || (JOY_DOWN_PIN & (1 << JOY_DOWN))) {
                joy_is_6_keys = false;
            } else {
                joy_is_6_keys = true;
            }

            if ((JOY_LEFT_PIN & (1 << JOY_LEFT)) || (JOY_RIGHT_PIN & (1 << JOY_RIGHT))) {
                joy_is_3_keys = false;
            } else {
                joy_is_3_keys = true;
            }

            break;

        case 6:

            if (joy_is_6_keys) {
                if (JOY_UP_PIN & (1 << JOY_UP)) {
                    joy_trg_z_state = true;
                } else {
                    joy_trg_z_state = false;
                }

                if (JOY_DOWN_PIN & (1 << JOY_DOWN)) {
                    joy_trg_y_state = true;
                } else {
                    joy_trg_y_state = false;
                }

                if (JOY_LEFT_PIN & (1 << JOY_LEFT)) {
                    joy_trg_x_state = true;
                } else {
                    joy_trg_x_state = false;
                }

                if (JOY_RIGHT_PIN & (1 << JOY_RIGHT)) {
                    joy_trg_mode_state = true;
                } else {
                    joy_trg_mode_state = false;
                }
            } else {
                joy_trg_z_state = true;
                joy_trg_y_state = true;
                joy_trg_x_state = true;
                joy_trg_mode_state = true;
            }

            break;
    
        default:
            break;
    }
}

void build_msx_phase_buffers_default_mode(void)
{
    uint8_t c;
    uint8_t d;

    // Phase 0 / 1 = DEFAULT
    c = 0;
    d = 0;

    if (joy_up_state)    c |= (1 << MSX_UP);
    if (joy_down_state)  c |= (1 << MSX_DOWN);
    if (joy_left_state)  c |= (1 << MSX_LEFT);
    if (joy_right_state) c |= (1 << MSX_RIGHT);

    if (joy_trg_a_state && joy_trg_start_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_b_state && joy_trg_c_state) d |= (1 << MSX_TRG_2);

    if (default_mode_trg_1_autofire_fire) d &= ~(1 << MSX_TRG_1);
    if (default_mode_trg_2_autofire_fire) d &= ~(1 << MSX_TRG_2);

    msx_portc_phase[0] = c;
    msx_portd_phase[0] = d;
    msx_portc_phase[1] = c;
    msx_portd_phase[1] = d;

    portc_phase_full[0] = (PORTC & ~MSX_C_MASK) | msx_portc_phase[0];
    portc_phase_full[1] = (PORTC & ~MSX_C_MASK) | msx_portc_phase[1];
}

void build_msx_phase_buffers_megadrive_mode(void)
{
    uint8_t c;
    uint8_t d;

    // Phase 0 / 2 / 4
    c = 0;
    d = 0;

    if (joy_up_state)    c |= (1 << MSX_UP);
    if (joy_down_state)  c |= (1 << MSX_DOWN);
    if (joy_left_state)  c |= (1 << MSX_LEFT);
    if (joy_right_state) c |= (1 << MSX_RIGHT);

    if (joy_trg_b_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_c_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[0] = c;
    msx_portd_phase[0] = d;
    msx_portc_phase[2] = c;
    msx_portd_phase[2] = d;
    msx_portc_phase[4] = c;
    msx_portd_phase[4] = d;
    
    // Phase 1 / 3 / 7

    c = 0;
    d = 0;
    
    if (joy_up_state)    c |= (1 << MSX_UP);
    if (joy_down_state)  c |= (1 << MSX_DOWN);
    c &= ~(1 << MSX_LEFT);
    c &= ~(1 << MSX_RIGHT);

    if (joy_trg_a_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_start_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[1] = c;
    msx_portd_phase[1] = d;
    msx_portc_phase[3] = c;
    msx_portd_phase[3] = d;
    msx_portc_phase[7] = c;
    msx_portd_phase[7] = d;

    // Phase 5

    c = 0;
    d = 0;

    c &= ~(1 << MSX_UP);
    c &= ~(1 << MSX_DOWN);
    c &= ~(1 << MSX_LEFT);
    c &= ~(1 << MSX_RIGHT);

    if (joy_trg_a_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_start_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[5] = c;
    msx_portd_phase[5] = d;

    // Phase 6

    c = 0;
    d = 0;

    if (joy_trg_z_state)  c |= (1 << MSX_UP);
    if (joy_trg_y_state)  c |= (1 << MSX_DOWN);
    if (joy_trg_x_state)  c |= (1 << MSX_LEFT);
    if (joy_trg_mode_state)  c |= (1 << MSX_RIGHT);

    if (joy_trg_b_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_c_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[6] = c;
    msx_portd_phase[6] = d;

    for (uint8_t phase = 0; phase < SEQ_MSX_PHASES; phase++) {
        portc_phase_full[phase] = (PORTC & ~MSX_C_MASK) | msx_portc_phase[phase];
    }
}

void build_msx_phase_buffers_mouse_mode(void)
{
    uint8_t c;
    uint8_t d;

    uint8_t mouse_dx_uint = (mouse_dx < MAX_MOUSE_XY_MIN_SPEED && mouse_dx > -MAX_MOUSE_XY_MIN_SPEED) ? 0 : (uint8_t)(mouse_dx + fast_jitter_soft());
    uint8_t mouse_dy_uint = (mouse_dy < MAX_MOUSE_XY_MIN_SPEED && mouse_dy > -MAX_MOUSE_XY_MIN_SPEED) ? 0 : (uint8_t)(mouse_dy + fast_jitter_soft());
    uint8_t mouse_dz_uint = (mouse_dz <= MAX_MOUSE_Z_SPEED) ? (uint8_t)(mouse_dz) : (uint8_t)(mouse_dz + fast_jitter_soft());

    bool x0 = (mouse_dx_uint >> 0) & 1;
    bool x1 = (mouse_dx_uint >> 1) & 1;
    bool x2 = (mouse_dx_uint >> 2) & 1;
    bool x3 = (mouse_dx_uint >> 3) & 1;
    bool x4 = (mouse_dx_uint >> 4) & 1;
    bool x5 = (mouse_dx_uint >> 5) & 1;
    bool x6 = (mouse_dx_uint >> 6) & 1;
    bool x7 = (mouse_dx_uint >> 7) & 1;

    bool y0 = (mouse_dy_uint >> 0) & 1;
    bool y1 = (mouse_dy_uint >> 1) & 1;
    bool y2 = (mouse_dy_uint >> 2) & 1;
    bool y3 = (mouse_dy_uint >> 3) & 1;
    bool y4 = (mouse_dy_uint >> 4) & 1;
    bool y5 = (mouse_dy_uint >> 5) & 1;
    bool y6 = (mouse_dy_uint >> 6) & 1;
    bool y7 = (mouse_dy_uint >> 7) & 1;

    bool z0 = (mouse_dz_uint >> 0) & 1;
    bool z1 = (mouse_dz_uint >> 1) & 1;
    bool z2 = (mouse_dz_uint >> 2) & 1;
    bool z3 = (mouse_dz_uint >> 3) & 1;
    bool z4 = (mouse_dz_uint >> 4) & 1;
    bool z5 = (mouse_dz_uint >> 5) & 1;
    bool z6 = (mouse_dz_uint >> 6) & 1;
    bool z7 = (mouse_dz_uint >> 7) & 1;

    bool b3_0 = joy_trg_x_state?0:1;
    bool b3_1 = joy_trg_start_state?0:1;
    bool b3_2 = joy_trg_mode_state?0:1;
    bool b3_3 = 0;

    bool b3_4 = 1;
    bool b3_5 = 0;
    bool b3_6 = 0;
    bool b3_7 = 0;

    // Phase 1
    c = 0;
    d = 0;

    if (x4)  c |= (1 << MSX_UP);
    if (x5)  c |= (1 << MSX_DOWN);
    if (x6)  c |= (1 << MSX_LEFT);
    if (x7)  c |= (1 << MSX_RIGHT);

    if (joy_trg_a_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_b_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[1] = c;
    msx_portd_phase[1] = d;
    
    // Phase 2

    c = 0;
    d = 0;
    
    if (x0)  c |= (1 << MSX_UP);
    if (x1)  c |= (1 << MSX_DOWN);
    if (x2)  c |= (1 << MSX_LEFT);
    if (x3)  c |= (1 << MSX_RIGHT);

    if (joy_trg_a_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_b_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[2] = c;
    msx_portd_phase[2] = d;

    // Phase 3
    c = 0;
    d = 0;

    if (y4)  c |= (1 << MSX_UP);
    if (y5)  c |= (1 << MSX_DOWN);
    if (y6)  c |= (1 << MSX_LEFT);
    if (y7)  c |= (1 << MSX_RIGHT);

    if (joy_trg_a_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_b_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[3] = c;
    msx_portd_phase[3] = d;

    // Phase 4

    c = 0;
    d = 0;
    
    if (y0)  c |= (1 << MSX_UP);
    if (y1)  c |= (1 << MSX_DOWN);
    if (y2)  c |= (1 << MSX_LEFT);
    if (y3)  c |= (1 << MSX_RIGHT);

    if (joy_trg_a_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_b_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[4] = c;
    msx_portd_phase[4] = d;

    // Phase 5
    c = 0;
    d = 0;

    if (b3_4)  c |= (1 << MSX_UP);
    if (b3_5)  c |= (1 << MSX_DOWN);
    if (b3_6)  c |= (1 << MSX_LEFT);
    if (b3_7)  c |= (1 << MSX_RIGHT);

    if (joy_trg_a_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_b_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[5] = c;
    msx_portd_phase[5] = d;

    // Phase 6
    c = 0;
    d = 0;

    if (b3_0)  c |= (1 << MSX_UP);
    if (b3_1)  c |= (1 << MSX_DOWN);
    if (b3_2)  c |= (1 << MSX_LEFT);
    if (b3_3)  c |= (1 << MSX_RIGHT);

    if (joy_trg_a_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_b_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[6] = c;
    msx_portd_phase[6] = d;

    // Phase 7
    c = 0;
    d = 0;

    if (z4)  c |= (1 << MSX_UP);
    if (z5)  c |= (1 << MSX_DOWN);
    if (z6)  c |= (1 << MSX_LEFT);
    if (z7)  c |= (1 << MSX_RIGHT);

    if (joy_trg_a_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_b_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[7] = c;
    msx_portd_phase[7] = d;

    // Phase 0
    c = 0;
    d = 0;

    if (z0)  c |= (1 << MSX_UP);
    if (z1)  c |= (1 << MSX_DOWN);
    if (z2)  c |= (1 << MSX_LEFT);
    if (z3)  c |= (1 << MSX_RIGHT);

    if (joy_trg_a_state) d |= (1 << MSX_TRG_1);
    if (joy_trg_b_state) d |= (1 << MSX_TRG_2);

    msx_portc_phase[0] = c;
    msx_portd_phase[0] = d;

    for (uint8_t phase = 0; phase < SEQ_MSX_PHASES; phase++) {
        portc_phase_full[phase] = (PORTC & ~MSX_C_MASK) | msx_portc_phase[phase];
    }
}

void build_msx_phase_buffers_off_mode(void)
{
    uint8_t c;
    uint8_t d;

    // Phases 0 - 7
    c = 0;
    d = 0;

    c |= (1 << MSX_UP);
    c |= (1 << MSX_DOWN);
    c |= (1 << MSX_LEFT);
    c |= (1 << MSX_RIGHT);

    d |= (1 << MSX_TRG_1);
    d |= (1 << MSX_TRG_2);

    for (uint8_t phase = 0; phase < SEQ_MSX_PHASES; phase++) {
        msx_portc_phase[phase] = c;
        msx_portd_phase[phase] = d;
    }

    for (uint8_t phase = 0; phase < SEQ_MSX_PHASES; phase++) {
        portc_phase_full[phase] = (PORTC & ~MSX_C_MASK) | msx_portc_phase[phase];
    }
}


void disable_and_reset_autofire(void) {
    default_mode_trg_1_autofire = AUTOFIRE_SPEED_DISABLED;
    default_mode_trg_2_autofire = AUTOFIRE_SPEED_DISABLED;
    default_mode_trg_1_autofire_mode_changed = false;
    default_mode_trg_2_autofire_mode_changed = false;
    default_mode_trg_1_autofire_fire = false;
    default_mode_trg_2_autofire_fire = false;
}

static inline int8_t fast_jitter_soft(void)
{
    uint8_t t = TCNT1L & 0x07;   // 0..7

    if (t < 6) return 0;         // 6/8 = 75%
    if (t == 6) return -1;       // 1/8 = 12.5%
    return 1;                    // 1/8 = 12.5%
}
