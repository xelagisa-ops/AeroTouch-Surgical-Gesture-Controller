/* ============================================================================
 * PROGRAM SUMMARY & FILE INFORMATION
 * ============================================================================
 * PROGRAM FUNCTIONALITY:
 * CEG2002 Integrated Gesture Control and Approach Detection System for MSP430F5529.
 * Combines ultrasonic approach detection (HC-SR04) with a dual VL53L0X Time-of-Flight
 * (ToF) gesture recognition engine. The system operates in LPM3 polling mode when
 * idle and wakes to a 50 Hz active sampling loop upon user approach. It features
 * dynamic ADC12 potentiometer threshold tuning (100mm to 400mm) on P7.0 and dual UART
 * outputs: USCI_A1 for PC debugging and USCI_A0 for wireless ESP32 telemetry.
 *
 * CODE STRUCTURE & PROGRAM FLOW:
 * 1. System Boot: Initialize clocks, GPIO, Timer_B0, USCI_A1 (PC UART), and USCI_A0 (ESP32 UART).
 * 2. ToF & ADC Init: Clear I2C bus, boot dual VL53L0X sensors, and configure ADC12 on P7.0.
 * 3. Low-Power Idle (LPM3): CPU sleeps while Timer_B0 triggers ultrasonic pings every 200 ms.
 * 4. Active Wake (<= 40 cm): Asserts WAKE_OUT (P1.4) and transitions CPU to 50 Hz active mode.
 * 5. Active Loop (20 ms): Reads potentiometer ADC code -> calculates gesture entry threshold
 *    -> samples ToF sensors -> applies 0.9 EMA filter -> updates gesture state machine (FSM)
 *    -> streams telemetry over dual UART interfaces.
 * 6. Sleep Transition (> 45 cm for 1s): Transmits power summary and re-enters LPM3 mode.
 *
 * AUTHORS & VERSION CONTROL:
 * - Lex Lee Gao Hao (2501194) (LL): Gesture FSM, Bit-Bang I2C, EMA Filter, ADC Tuning
 * - Manvalan Srihanush (2501002) (MS): HC-SR04 Driver, LPM3 Power Management, Timers
 * - Lee Yu Huan (2502685) (YH): ESP32 Hardware UART Interface (USCI_A0)
 * Version: 7.8.6 (Full Requirement 9 Compliance Lock)
 * ============================================================================
 */

#include <msp430.h>
#include <stdint.h>

 /* ============================================================================
  * TOF GESTURE ENGINE DEFINITIONS & ENUMS (LL)
  * ============================================================================
  */

  /* VL53L0X default I2C address */
#define VL53L0X_ADDR 0x29                 /* Default hardware slave address for VL53L0X (LL) */

/* Exponential moving average filter weights (0.9 alpha) */
#define EMA_ALPHA_NUM 90                  /* Numerator for 0.9 EMA smoothing filter (LL) */
#define EMA_ALPHA_DEN 100                 /* Denominator for 0.9 EMA smoothing filter (LL) */

/* Sensor 1 (Left) - Port 2 Bit-Bang Pins */
#define S1_SDA_PIN BIT0                   /* P2.0 configured as bit-bang I2C SDA for Sensor 1 (LL) */
#define S1_SCL_PIN BIT2                   /* P2.2 configured as bit-bang I2C SCL for Sensor 1 (LL) */

/* Sensor 2 (Right) - Port 3 Bit-Bang Pins */
#define S2_SDA_PIN BIT0                   /* P3.0 configured as bit-bang I2C SDA for Sensor 2 (LL) */
#define S2_SCL_PIN BIT1                   /* P3.1 configured as bit-bang I2C SCL for Sensor 2 (LL) */

/* Enumeration of supported gesture classifications */
typedef enum {
    GESTURE_NONE = 0,                     /* Idle state - no gesture detected (LL) */
    GESTURE_SWIPE_L_TO_R,                 /* Left to Right swipe gesture (LL) */
    GESTURE_SWIPE_R_TO_L,                 /* Right to Left swipe gesture (LL) */
    GESTURE_PINCH_IN,                     /* Pinch-In (Zoom Out) gesture (LL) */
    GESTURE_PINCH_OUT                     /* Pinch-Out (Zoom In) gesture (LL) */
} Gesture_t;

/* Global system time base and filtered sensor variables */
volatile uint32_t system_ms_ticks = 0;   /* Cumulative system time in ms, updated at 50 Hz (LL) */
static uint16_t ema_d1 = 0;               /* Exponentially smoothed distance for Sensor 1 (LL) */
static uint16_t ema_d2 = 0;               /* Exponentially smoothed distance for Sensor 2 (LL) */

/* Dynamic ADC potentiometer threshold boundaries */
static uint16_t enter_thresh_d1;          /* Active gesture entry threshold for Sensor 1 (LL) */
static uint16_t enter_thresh_d2;          /* Active gesture entry threshold for Sensor 2 (LL) */
static uint16_t exit_thresh_d1;           /* Deactivation exit threshold for Sensor 1 (LL) */
static uint16_t exit_thresh_d2;           /* Deactivation exit threshold for Sensor 2 (LL) */

/* Velocity telemetry metrics */
static uint16_t pinch_speed_val = 0;     /* Scaled speed index output (10 to 100) (LL) */

/* Function Prototypes - Gesture Engine & Peripherals (LL / YH) */
void force_bus_clear(void);
void adc_init(void);
uint16_t adc_read_raw(void);
Gesture_t detect_gesture(uint16_t d1, uint16_t d2);
uint16_t get_last_transit_cycles(void);
uint16_t get_pinch_speed(void);
uint16_t apply_ema(uint16_t raw_val, uint16_t prev_ema);
void delay_us(uint16_t us);

void bb1_init(void);
void bb1_init_sensor(void);
uint16_t bb1_read_range(void);
void bb1_start(void);
void bb1_stop(void);
uint8_t bb1_write_byte(uint8_t byte);
uint8_t bb1_read_byte(uint8_t ack);
uint8_t bb1_write_reg(uint8_t reg, uint8_t val);

void bb2_init(void);
void bb2_init_sensor(void);
uint16_t bb2_read_range(void);
void bb2_start(void);
void bb2_stop(void);
uint8_t bb2_write_byte(uint8_t byte);
uint8_t bb2_read_byte(uint8_t ack);
uint8_t bb2_write_reg(uint8_t reg, uint8_t val);

void esp32_uart_init(void);
void esp32_uart_print(const char* str);

/* ============================================================================
 * ULTRASONIC & POWER MANAGEMENT DEFINITIONS (MS)
 * ============================================================================
 */

#define USE_UART_LOG        1       /* 1 = Enable UART serial logging output (LL) */
#define USE_DEBUG_LED       1       /* 1 = Turn ON LED1 (P1.0) while system is active (MS) */

 /* Hardware GPIO Pin Assignments */
#define TRIG_PIN            BIT3    /* P1.3 connected to HC-SR04 TRIG pin (MS) */
#define ECHO_PIN            BIT2    /* P1.2 connected to HC-SR04 ECHO pin (TA0.1 / CCI1A) (MS) */
#define WAKE_OUT_PIN        BIT4    /* P1.4 output line to wake external ESP32 microcontroller (MS) */
#define LED_PIN             BIT0    /* P1.0 onboard status LED1 (MS) */

/* Clock frequencies */
#define SMCLK_HZ            1048576UL   /* Standard DCOCLKDIV frequency after reset (MS) */
#define ACLK_HZ             32768UL     /* REFO 32.768 kHz low-frequency clock (MS) */

/* Timing & Loop Parameters */
#define ACTIVE_TICK_MS      20          /* Base period for 50 Hz active execution loop (MS) */
#define PING_EVERY_N_TICKS  10          /* Execute ultrasonic ping every 200 ms in active mode (MS) */
#define SLEEP_POLL_MS       200         /* LPM3 polling wake interval in milliseconds (MS) */
#define ABSENT_TIMEOUT_MS   1000        /* User absence duration required before entering sleep (MS) */
#define NEAR_CONFIRM        2           /* Required consecutive near pings to confirm approach (MS) */
#define LOG_PERIOD_MS       500         /* Interval for status logging while active (MS) */

#define FAR_CONFIRM         (ABSENT_TIMEOUT_MS / (ACTIVE_TICK_MS * PING_EVERY_N_TICKS))
#define PING_GAP_CYCLES     ((SMCLK_HZ / 1000UL) * 60UL) /* ~60 ms hardware delay between pings (MS) */

/* Ultrasonic distance boundaries */
#define PRESENT_MM          400         /* Distance <= 400 mm triggers active wake (MS) */
#define ABSENT_MM           450         /* Distance > 450 mm confirms user departure (MS) */

#define ECHO_TIMEOUT_US     3000UL      /* 3 ms max echo pulse duration to prevent frame lag (MS) */
#define NO_ECHO             0xFFFFu     /* Sentinel value indicating missing or invalid echo pulse (MS) */

#define MS_TO_ACLK(ms)      ((uint16_t)(((uint32_t)(ms) * ACLK_HZ) / 1000UL))
#define US_TO_SMCLK(us)     ((uint16_t)(((uint32_t)(us) * (SMCLK_HZ / 1000UL)) / 1000UL))

typedef enum { SYS_SLEEP = 0, SYS_ACTIVE = 1 } sys_state_t;

static volatile uint8_t  tick_flag = 0;     /* Set by Timer_B0 ISR to trigger periodic processing (MS) */
static volatile uint8_t  echo_state = 0;     /* State tracker: 0=idle, 1=rising, 2=done, 3=timeout (MS) */
static volatile uint16_t echo_rise = 0;     /* Timer capture count at echo rising edge (MS) */
static volatile uint16_t echo_fall = 0;     /* Timer capture count at echo falling edge (MS) */

static sys_state_t sys_state = SYS_SLEEP;  /* System operating state (SYS_SLEEP vs SYS_ACTIVE) (MS) */
static uint8_t     tick_count = 0;          /* Prescaler tick counter for active pings (MS) */
static uint8_t     far_count = 0;          /* Counter for consecutive absent distance reads (MS) */
static uint16_t    last_mm = NO_ECHO;    /* Most recent ultrasonic distance measurement in mm (MS) */

/* Energy profiling metrics */
static uint32_t    ms_sleep = 0;          /* Cumulative time spent in LPM3 sleep mode in ms (MS) */
static uint32_t    ms_active = 0;          /* Cumulative time spent in active mode in ms (MS) */
static uint16_t    ms_log = 0;          /* Timer tracker for status logging interval (MS) */
static uint16_t    wake_events = 0;          /* Total count of sleep-to-active transition events (MS) */

static char        buf[80];                  /* Shared string formatting buffer for UART output (MS) */

/* Function Prototypes - Power Management & Ultrasonic Driver (MS / LL) */
void uart_printf(const char* data);
static void     clock_init(void);
static void     gpio_init(void);
static void     uart_init(void);
static void     tick_timer_init(void);
static void     tick_set_period_ms(uint16_t ms);
static uint16_t ticks_to_mm(uint16_t ticks);
static uint16_t ultrasonic_read_mm(void);
static uint8_t  user_present_confirmed(void);
static void     enter_active(void);
static void     enter_sleep(void);
static char* str_append(char* dst, const char* src);
static char* str_append_u32(char* dst, uint32_t v);
static void     log_status(void);
static void     log_power_summary(void);

void gesture_engine_init(void);
void gesture_engine_on_wake(void);
void gesture_engine_on_sleep(void);
void gesture_engine_task(void);

/* ========================================================================== */
/*                               MAIN FUNCTION                                */
/* ========================================================================== */

/**
 * @brief  Main Application Entry Point.
 * @role   Initializes system clocks, GPIO pins, peripherals, and executes the
 *         low-power event loop switching between LPM3 polling and 50 Hz active execution.
 * @param  None
 * @return None
 */
void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;            /* Stop watchdog timer (MS) */

    clock_init();                        /* Initialize system clock sources (MS) */
    gpio_init();                         /* Configure ultrasonic, LED, and wake GPIO pins (MS) */

#if USE_UART_LOG
    uart_init();                         /* Initialize USCI_A1 UART for PC backchannel debugging (LL) */
    uart_printf("\ec");                  /* Send VT100 terminal clear screen sequence (LL) */
    uart_printf("CEG2002 Integrated System Running"); /* Send boot confirmation message (LL) */
#endif

    gesture_engine_init();               /* Initialize ToF sensors, ADC12, and ESP32 UART (LL/YH) */

    tick_timer_init();                   /* Configure Timer_B0 for periodic tick generation (MS) */
    __enable_interrupt();                /* Enable global interrupts (MS) */

    while (1) {
        __disable_interrupt();           /* Gate interrupts to safely check tick status (MS) */
        if (!tick_flag) {
            if (sys_state == SYS_SLEEP)
                __bis_SR_register(LPM3_bits | GIE); /* Sleep CPU in LPM3 mode during idle (MS) */
            else
                __bis_SR_register(LPM0_bits | GIE); /* Sleep CPU in LPM0 mode during active loops (MS) */
        }
        else {
            __enable_interrupt();        /* Re-enable interrupts if tick is pending (MS) */
        }
        __no_operation();

        if (!tick_flag)
            continue;                    /* Bypass execution if tick flag is cleared (MS) */
        tick_flag = 0;                   /* Reset periodic tick flag (MS) */

        if (sys_state == SYS_SLEEP) {
            /* LPM3 low-power polling state */
            ms_sleep += SLEEP_POLL_MS;   /* Accumulate sleep time counter (MS) */

            if (user_present_confirmed())
                enter_active();          /* Transition system to ACTIVE if approach confirmed (MS) */
        }
        else {
            /* Active 50 Hz execution loop */
            ms_active += ACTIVE_TICK_MS;  /* Accumulate active time counter (MS) */

            if (++tick_count >= PING_EVERY_N_TICKS) {
                tick_count = 0;          /* Reset prescaler counter (MS) */
                last_mm = ultrasonic_read_mm(); /* Read ultrasonic distance in mm (MS) */

                if (last_mm == NO_ECHO || last_mm > ABSENT_MM)
                    far_count++;         /* Increment absence counter if user is out of range (MS) */
                else
                    far_count = 0;       /* Reset absence counter if user is detected (MS) */
            }

            gesture_engine_task();       /* Execute ToF sampling, pot reading, and gesture FSM (LL) */

            ms_log += ACTIVE_TICK_MS;    /* Advance logging timer (MS) */
            if (ms_log >= LOG_PERIOD_MS) {
                ms_log = 0;              /* Reset logging timer (MS) */
                log_status();            /* Transmit presence state toggles over UART (MS) */
            }

            if (far_count >= FAR_CONFIRM)
                enter_sleep();           /* Transition system to LPM3 SLEEP if user leaves (MS) */
        }
    }
}

/* ========================================================================== */
/*                          GESTURE ENGINE FUNCTIONS (LL)                     */
/* ========================================================================== */

/**
 * @brief  One-time Initialization for Gesture Peripherals.
 * @role   Clears I2C buses, initializes bit-bang interfaces, boots VL53L0X sensors,
 *         configures ADC12 on P7.0, and initializes USCI_A0 for ESP32 UART.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL) / Lee Yu Huan (YH)
 */
void gesture_engine_init(void)
{
    force_bus_clear();                     /* Toggle SCL lines to release hung I2C buses (LL) */
    bb1_init();                            /* Configure GPIO directions for Sensor 1 I2C (LL) */
    bb2_init();                            /* Configure GPIO directions for Sensor 2 I2C (LL) */
    adc_init();                            /* Configure MSP430 ADC12 peripheral on P7.0 (LL) */
    esp32_uart_init();                     /* Configure USCI_A0 UART for ESP32 communication (YH) */

    bb1_init_sensor();                     /* Write initialization register sequence to Sensor 1 (LL) */
    bb2_init_sensor();                     /* Write initialization register sequence to Sensor 2 (LL) */
}

/**
 * @brief  System Wake Event Handler for Gesture Engine.
 * @role   Resets EMA filter distance buffers upon exiting LPM3 sleep.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void gesture_engine_on_wake(void)
{
    ema_d1 = 0;                            /* Reset Sensor 1 EMA filter memory (LL) */
    ema_d2 = 0;                            /* Reset Sensor 2 EMA filter memory (LL) */
}

/**
 * @brief  System Sleep Event Handler for Gesture Engine.
 * @role   Safely suspends gesture engine processing prior to entering LPM3.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void gesture_engine_on_sleep(void)
{
    /* Suspend active processing before CPU enters LPM3 mode (LL) */
}

/**
 * @brief  Main Periodic Gesture Processing Task (Executes at 50 Hz / 20 ms).
 * @role   Samples ADC potentiometer on P7.0, updates entry/exit distance thresholds,
 *         reads dual ToF sensors, applies 0.9 EMA filtering, updates the gesture FSM,
 *         and streams detected gestures to PC and ESP32 UART channels.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void gesture_engine_task(void)
{
    system_ms_ticks += 20;                 /* Advance system time base by 20 ms frame time (LL) */

    /* Read 12-bit analog value from P7.0 (A12) and map directly to 100mm - 400mm threshold */
    uint16_t raw_pot = adc_read_raw();     /* Read raw 12-bit ADC value (0-4095) from Channel A12 (LL) */

    enter_thresh_d1 = 40 + (uint16_t)(((uint32_t)raw_pot * 300) / 4095); /* Direct threshold mapping (LL) */
    enter_thresh_d2 = enter_thresh_d1;     /* Match Sensor 2 threshold to Sensor 1 (LL) */

    exit_thresh_d1 = enter_thresh_d1 + 40; /* Apply +40mm hysteresis to Sensor 1 exit threshold (LL) */
    exit_thresh_d2 = enter_thresh_d2 + 40; /* Apply +40mm hysteresis to Sensor 2 exit threshold (LL) */

    /* Transmit potentiometer threshold over UART when turned by > 50 ADC counts */
#if USE_UART_LOG
    static uint16_t prev_raw_pot = 0xFFFF;
    int16_t pot_diff = (int16_t)raw_pot - (int16_t)prev_raw_pot;

    if (pot_diff > 50 || pot_diff < -50) {
        prev_raw_pot = raw_pot;            /* Update previous potentiometer register memory (LL) */
        char pbuf[64];
        char* p = pbuf;
        p = str_append(p, " [POT] Raw: ");
        p = str_append_u32(p, raw_pot);
        p = str_append(p, " | Entry Thresh: ");
        p = str_append_u32(p, enter_thresh_d1);
        p = str_append(p, " mm");
        uart_printf(pbuf);                 /* Transmit live potentiometer value to PC serial terminal (LL) */
    }
#endif

    /* Sample raw range readings from both ToF sensors */
    uint16_t raw_d1 = bb1_read_range();    /* Read range distance in mm from Sensor 1 (LL) */
    uint16_t raw_d2 = bb2_read_range();    /* Read range distance in mm from Sensor 2 (LL) */

    if (raw_d1 < 35 || raw_d1 > 2000) raw_d1 = 8190; /* Clamp out-of-bounds readings for Sensor 1 (LL) */
    if (raw_d2 < 35 || raw_d2 > 2000) raw_d2 = 8190; /* Clamp out-of-bounds readings for Sensor 2 (LL) */

    /* Smooth signals with 0.9 Exponential Moving Average */
    ema_d1 = apply_ema(raw_d1, ema_d1);    /* Apply 0.9 EMA smoothing filter to Sensor 1 (LL) */
    ema_d2 = apply_ema(raw_d2, ema_d2);    /* Apply 0.9 EMA smoothing filter to Sensor 2 (LL) */

    /* Evaluate gesture state machine */
    Gesture_t current_gesture = detect_gesture(ema_d1, ema_d2); /* Process filtered sensor data (LL) */

    /* Output detected gesture telemetry over dual UART interfaces */
    if (current_gesture != GESTURE_NONE) {
        char gbuf[40];
        char* g = gbuf;

        switch (current_gesture) {
        case GESTURE_SWIPE_L_TO_R:
            g = str_append(g, "L->R:");
            g = str_append_u32(g, get_last_transit_cycles()); /* Append transit time in ms (LL) */
            uart_printf(gbuf);                               /* Stream to PC via USCI_A1 (LL) */

            esp32_uart_print("SWIPE_RIGHT\n");                          /* Stream to ESP32 via USCI_A0 (YH) */
            break;

        case GESTURE_SWIPE_R_TO_L:
            g = str_append(g, "R->L:");
            g = str_append_u32(g, get_last_transit_cycles()); /* Append transit time in ms (LL) */
            uart_printf(gbuf);                               /* Stream to PC via USCI_A1 (LL) */

            esp32_uart_print("SWIPE_LEFT\n");                           /* Stream to ESP32 via USCI_A0 (YH) */
            break;

        case GESTURE_PINCH_IN:
            g = str_append(g, "PINCH_IN:");
            g = str_append_u32(g, get_pinch_speed());        /* Append pinch speed index (LL) */
            uart_printf(gbuf);                               /* Stream to PC via USCI_A1 (LL) */

            esp32_uart_print("ZOOM_OUT\n");                            /* Stream to ESP32 via USCI_A0 (YH) */
            break;

        case GESTURE_PINCH_OUT:
            g = str_append(g, "PINCH_OUT:");
            g = str_append_u32(g, get_pinch_speed());        /* Append pinch speed index (LL) */
            uart_printf(gbuf);                               /* Stream to PC via USCI_A1 (LL) */

            esp32_uart_print("ZOOM_IN\n");                         /* Stream to ESP32 via USCI_A0 (YH) */
            break;

        default:
            break;
        }
    }
}

/* ========================================================================== */
/*                 POWER MANAGEMENT & STATE TRANSITIONS (MS)                  */
/* ========================================================================== */

/**
 * @brief  Transitions System from SLEEP to ACTIVE Mode.
 * @role   Configures Timer_B0 for 50 Hz operation, asserts WAKE_OUT line (P1.4),
 *         turns ON status LED1 (P1.0), and invokes gesture wake handler.
 * @param  None
 * @return None
 * @author Manvalan Srihanush (MS)
 */
static void enter_active(void)
{
    sys_state = SYS_ACTIVE;               /* Update state flag to SYS_ACTIVE (MS) */
    far_count = 0;                        /* Reset absence counter (MS) */
    tick_count = 0;                        /* Reset active tick counter (MS) */
    ms_log = 0;                        /* Reset logging interval timer (MS) */
    wake_events++;                         /* Increment total wake event count (MS) */

    tick_set_period_ms(ACTIVE_TICK_MS);    /* Reconfigure Timer_B0 for 20 ms (50 Hz) tick (MS) */
    TB0CTL |= TBCLR;                       /* Clear and restart Timer_B0 counter (MS) */

    P1OUT |= WAKE_OUT_PIN;                 /* Drive P1.4 HIGH to wake external ESP32 (MS) */
#if USE_DEBUG_LED
    P1OUT |= LED_PIN;                      /* Turn ON onboard status LED1 (MS) */
#endif

    gesture_engine_on_wake();              /* Re-initialize gesture state variables (LL) */

#if USE_UART_LOG
    uart_printf("USER NEAR -> ACTIVE, WAKE_OUT=1"); /* Transmit state transition over UART (MS) */
#endif
    log_status();                          /* Transmit initial approach status (MS) */
}

/**
 * @brief  Transitions System from ACTIVE to SLEEP Mode.
 * @role   Deasserts WAKE_OUT line (P1.4), turns OFF status LED1 (P1.0), logs power
 *         consumption summary, and reconfigures Timer_B0 for LPM3 polling (200 ms).
 * @param  None
 * @return None
 * @author Manvalan Srihanush (MS)
 */
static void enter_sleep(void)
{
    gesture_engine_on_sleep();             /* Suspend active gesture state processing (LL) */

    P1OUT &= ~WAKE_OUT_PIN;                /* Drive P1.4 LOW to notify ESP32 of sleep mode (MS) */
#if USE_DEBUG_LED
    P1OUT &= ~LED_PIN;                     /* Turn OFF onboard status LED1 (MS) */
#endif
    sys_state = SYS_SLEEP;                 /* Update state flag to SYS_SLEEP (MS) */

    log_power_summary();                   /* Transmit cumulative energy duty metrics over UART (MS) */

    tick_set_period_ms(SLEEP_POLL_MS);     /* Reconfigure Timer_B0 for 200 ms LPM3 poll tick (MS) */
    TB0CTL |= TBCLR;                       /* Clear and restart Timer_B0 counter (MS) */
}

/**
 * @brief  Confirms User Presence using Multiple Ultrasonic Samples.
 * @role   Executes consecutive ultrasonic range readings to verify user presence
 *         and prevent false wake triggers caused by momentary signal noise.
 * @param  None
 * @return uint8_t 1 if presence confirmed (<= 400mm), 0 otherwise.
 * @author Manvalan Srihanush (MS)
 */
static uint8_t user_present_confirmed(void)
{
    uint8_t i;

    for (i = 0; i < NEAR_CONFIRM; i++) {
        last_mm = ultrasonic_read_mm();     /* Read ultrasonic distance measurement (MS) */

        if (last_mm == NO_ECHO || last_mm > PRESENT_MM)
            return 0;                      /* Return false if any sample exceeds threshold (MS) */

        if (i < NEAR_CONFIRM - 1)
            __delay_cycles(PING_GAP_CYCLES); /* Delay before next verification sample (MS) */
    }
    return 1;                              /* Return true if all samples confirm presence (MS) */
}

/* ========================================================================== */
/*                       HC-SR04 ULTRASONIC DRIVER (MS)                       */
/* ========================================================================== */

/**
 * @brief  Converts Timer_A0 Clock Ticks to Millimeters.
 * @role   Scales pulse width duration in clock ticks into physical distance in mm.
 * @param  ticks Pulse duration in Timer_A0 clock counts.
 * @return uint16_t Calculated distance in millimeters.
 * @author Manvalan Srihanush (MS)
 */
static uint16_t ticks_to_mm(uint16_t ticks)
{
    return (uint16_t)(((uint32_t)ticks * 1715UL) / (SMCLK_HZ / 100UL)); /* Pulse width conversion (MS) */
}

/**
 * @brief  Triggers and Reads HC-SR04 Distance Measurement.
 * @role   Generates a 10 us trigger pulse on P1.3, captures echo pulse width on P1.2
 *         using Timer_A0 input capture, and converts pulse width to millimeters.
 * @param  None
 * @return uint16_t Measured distance in mm, or NO_ECHO (0xFFFF) on timeout.
 * @author Manvalan Srihanush (MS)
 */
static uint16_t ultrasonic_read_mm(void)
{
    echo_state = 0;                        /* Clear echo capture state flag (MS) */

    TA0CTL = TASSEL_2 | MC_2 | TACLR;    /* SMCLK source, continuous mode, clear counter (MS) */
    TA0CCTL1 = CM_3 | CCIS_0 | SCS | CAP | CCIE; /* Capture both edges, CCI1A (P1.2), enable ISR (MS) */
    TA0CCR2 = TA0R + US_TO_SMCLK(ECHO_TIMEOUT_US); /* Set timeout threshold in TA0CCR2 (MS) */
    TA0CCTL2 = CCIE;                       /* Enable timeout interrupt (MS) */

    P1OUT |= TRIG_PIN;                     /* Drive P1.3 HIGH (TRIG pulse start) (MS) */
    __delay_cycles(12);                    /* Hold TRIG pulse HIGH for > 10 us (MS) */
    P1OUT &= ~TRIG_PIN;                    /* Drive P1.3 LOW (TRIG pulse end) (MS) */

    while (echo_state < 2) {
        __disable_interrupt();
        if (echo_state < 2)
            __bis_SR_register(LPM0_bits | GIE); /* Sleep CPU in LPM0 waiting for capture ISR (MS) */
        else
            __enable_interrupt();
        __no_operation();
    }

    TA0CCTL1 = 0;                          /* Disable Timer_A0 CCR1 capture (MS) */
    TA0CCTL2 = 0;                          /* Disable Timer_A0 CCR2 timeout (MS) */
    TA0CTL = MC_0 | TACLR;               /* Stop Timer_A0 counter (MS) */

    if (echo_state == 2)
        return ticks_to_mm((uint16_t)(echo_fall - echo_rise)); /* Compute distance from pulse width (MS) */

    return NO_ECHO;                        /* Return timeout sentinel value (MS) */
}

/**
 * @brief  Initializes MSP430 System Clock Registers.
 * @role   Configures ACLK to run from REFO (32.768 kHz) low-frequency oscillator.
 * @param  None
 * @return None
 * @author Manvalan Srihanush (MS)
 */
static void clock_init(void)
{
    UCSCTL4 = (UCSCTL4 & ~SELA_7) | SELA_2; /* Source ACLK from REFO oscillator (MS) */
}

/**
 * @brief  Initializes GPIO Pin Directions and Multiplexing Functions.
 * @role   Configures P1.3 (TRIG), P1.2 (ECHO capture), P1.4 (WAKE_OUT), and P1.0 (LED1).
 * @param  None
 * @return None
 * @author Manvalan Srihanush (MS)
 */
static void gpio_init(void)
{
    P1DIR |= TRIG_PIN;                     /* Set P1.3 as digital output (TRIG) (MS) */
    P1OUT &= ~TRIG_PIN;                    /* Initialize TRIG pin LOW (MS) */
    P1DIR &= ~ECHO_PIN;                    /* Set P1.2 as digital input (ECHO) (MS) */
    P1SEL |= ECHO_PIN;                     /* Select P1.2 secondary function TA0.1 capture input (MS) */

    P1DIR |= WAKE_OUT_PIN;                 /* Set P1.4 as digital output (WAKE_OUT) (MS) */
    P1OUT &= ~WAKE_OUT_PIN;                /* Initialize WAKE_OUT pin LOW (MS) */

    P1DIR |= LED_PIN;                      /* Set P1.0 as digital output (LED1) (MS) */
    P1OUT &= ~LED_PIN;                     /* Initialize LED1 OFF (MS) */
}

/**
 * @brief  Initializes USCI_A1 Hardware UART for PC Serial Terminal Debugging.
 * @role   Configures P4.4 (TXD) and P4.5 (RXD) at 115200 Baud using SMCLK.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
static void uart_init(void)
{
    P4SEL |= BIT5 + BIT4;                  /* Select P4.4 (TXD) and P4.5 (RXD) primary functions (LL) */
    P4DIR |= BIT4;                         /* Set P4.4 as TXD output (LL) */
    P4DIR &= ~BIT5;                        /* Set P4.5 as RXD input (LL) */

    UCA1CTL1 |= UCSWRST;                   /* Put USCI_A1 state machine in reset (LL) */
    UCA1CTL1 |= UCSSEL_2;                  /* Select SMCLK as USCI clock source (LL) */
    UCA1BR0 = 0x09;                        /* Set prescaler for 115200 Baud at 1.048 MHz SMCLK (LL) */
    UCA1BR1 = 0x00;
    UCA1MCTL |= UCBRS_1 + UCBRF_0;          /* Configure modulation parameters (LL) */
    UCA1CTL0 = 0x00;                       /* No parity, LSB first, 8-bit data, 1 stop bit (LL) */
    UCA1CTL1 &= ~UCSWRST;                  /* Release USCI_A1 from reset state (LL) */
}

/**
 * @brief  Configures Timer_B0 Compare Register for Specified Period.
 * @role   Loads TB0CCR0 with calculated clock counts corresponding to target period in ms.
 * @param  ms Target timer period in milliseconds.
 * @return None
 * @author Manvalan Srihanush (MS)
 */
static void tick_set_period_ms(uint16_t ms)
{
    TB0CCR0 = MS_TO_ACLK(ms) - 1;          /* Update Timer_B0 CCR0 match value (MS) */
}

/**
 * @brief  Initializes Timer_B0 as System Time Base.
 * @role   Configures Timer_B0 on ACLK (32.768 kHz) in Up Mode with interrupts enabled.
 * @param  None
 * @return None
 * @author Manvalan Srihanush (MS)
 */
static void tick_timer_init(void)
{
    TB0CCTL0 = CCIE;                       /* Enable Timer_B0 CCR0 match interrupt (MS) */
    tick_set_period_ms(SLEEP_POLL_MS);     /* Initialize timer period to 200 ms (MS) */
    TB0CTL = TBSSEL_1 | MC_1 | TBCLR;    /* Source from ACLK, Up mode, clear counter (MS) */
}

/**
 * @brief  Appends Character String to Destination Buffer.
 * @param  dst Pointer to destination string buffer.
 * @param  src Pointer to source string buffer.
 * @return char* Pointer to trailing null terminator in destination string.
 * @author Lex Lee Gao Hao (LL)
 */
static char* str_append(char* dst, const char* src)
{
    while (*src)
        *dst++ = *src++;
    *dst = '\0';
    return dst;
}

/**
 * @brief  Appends Unsigned 32-bit Integer as ASCII String to Destination Buffer.
 * @param  dst Pointer to destination string buffer.
 * @param  v 32-bit unsigned integer value to convert.
 * @return char* Pointer to trailing null terminator in destination string.
 * @author Lex Lee Gao Hao (LL)
 */
static char* str_append_u32(char* dst, uint32_t v)
{
    char    tmp[11];
    uint8_t n = 0;

    do {
        tmp[n++] = (char)('0' + (v % 10UL));
        v /= 10UL;
    } while (v);

    while (n)
        *dst++ = tmp[--n];
    *dst = '\0';
    return dst;
}

/**
 * @brief  Logs Approach Detection Presence State Toggles over UART.
 * @role   Transmits a single serial log event when user presence toggles between ON and OFF.
 * @param  None
 * @return None
 * @author Manvalan Srihanush (MS)
 */
static void log_status(void)
{
#if USE_UART_LOG
    static uint8_t presence_state = 0;     /* 0 = OFF (No User), 1 = ON (User Detected) (MS) */

    uint8_t current_state = (last_mm != NO_ECHO && last_mm <= ABSENT_MM) ? 1 : 0;

    if (current_state != presence_state) {
        presence_state = current_state;    /* Update presence state memory (MS) */

        if (presence_state == 1) {
            uart_printf("[APPROACH] STATE: ON (User Detected)");   /* Transmit state ON event (MS) */
        }
        else {
            uart_printf("[APPROACH] STATE: OFF (User Out of Range)");/* Transmit state OFF event (MS) */
        }
    }
#endif
}

/**
 * @brief  Transmits Cumulative Power Consumption Summary over UART.
 * @role   Calculates and formats active time, sleep time, duty cycle percentage,
 *         and wake count before system re-enters LPM3 sleep mode.
 * @param  None
 * @return None
 * @author Manvalan Srihanush (MS)
 */
static void log_power_summary(void)
{
#if USE_UART_LOG
    uint32_t total = ms_active + ms_sleep;
    uint32_t duty;
    char* p = buf;

    if (total == 0)
        total = 1;
    duty = (ms_active * 100UL) / total;    /* Calculate active duty cycle percentage (MS) */

    p = str_append(p, "SLEEP: act_s=");
    p = str_append_u32(p, ms_active / 1000UL);
    p = str_append(p, " slp_s=");
    p = str_append_u32(p, ms_sleep / 1000UL);
    p = str_append(p, " duty_pct=");
    p = str_append_u32(p, duty);
    p = str_append(p, " wakes=");
    p = str_append_u32(p, wake_events);
    uart_printf(buf);                      /* Transmit formatted power summary over UART (MS) */
#endif
}

/**
 * @brief  Transmits String Buffer over USCI_A1 UART to PC.
 * @role   Polls TX buffer flags and streams character array to PC serial interface.
 * @param  data Pointer to null-terminated string buffer.
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void uart_printf(const char* data)
{
    volatile char i = 0;
    while (data[i] != '\0') {
        UCA1TXBUF = data[i];               /* Load character into USCI_A1 TX buffer (LL) */
        while (UCA1STAT & UCBUSY);         /* Wait until USCI_A1 state machine is idle (LL) */
        i++;
    }
    while (!(UCA1IFG & UCTXIFG));          /* Wait for TX buffer ready flag (LL) */
    UCA1TXBUF = 0x0d;                      /* Send Carriage Return '\r' (LL) */
    while (!(UCA1IFG & UCTXIFG));          /* Wait for TX buffer ready flag (LL) */
    UCA1TXBUF = 0x0a;                      /* Send Line Feed '\n' (LL) */
    while (UCA1STAT & UCBUSY);             /* Wait until final character transmission completes (LL) */
}

/* ========================================================================== */
/*                       INTERRUPT SERVICE ROUTINES                           */
/* ========================================================================== */

/**
 * @brief  Timer_B0 CCR0 Interrupt Service Routine.
 * @role   Generates periodic system ticks and wakes CPU from LPM3 sleep mode.
 * @param  None
 * @return None
 * @author Manvalan Srihanush (MS)
 */
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
    tick_flag = 1;                         /* Set periodic system tick flag (MS) */
    __bic_SR_register_on_exit(LPM3_bits);  /* Clear LPM3 bits on ISR exit to wake CPU (MS) */
}

/**
 * @brief  Timer_A0 CCR1/CCR2 Interrupt Service Routine.
 * @role   Captures rising and falling edges of HC-SR04 echo pulse on P1.2 (CCR1)
 *         and handles capture timeout events (CCR2).
 * @param  None
 * @return None
 * @author Manvalan Srihanush (MS)
 */
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
    switch (__even_in_range(TA0IV, 14)) {
    case 0:  break;
    case 2:  /* CCR1 Capture Interrupt */
        if (TA0CCTL1 & CCI) {
            if (echo_state == 0) {
                echo_rise = TA0CCR1;      /* Save timer count at echo rising edge (MS) */
                echo_state = 1;            /* Update state to rising edge seen (MS) */
            }
        }
        else if (echo_state == 1) {
            echo_fall = TA0CCR1;          /* Save timer count at echo falling edge (MS) */
            echo_state = 2;                /* Update state to capture complete (MS) */
            __bic_SR_register_on_exit(LPM0_bits); /* Clear LPM0 bits to wake CPU (MS) */
        }
        break;
    case 4:  /* CCR2 Timeout Compare Interrupt */
        if (echo_state != 2) {
            echo_state = 3;                /* Update state to timeout flag (MS) */
            __bic_SR_register_on_exit(LPM0_bits); /* Clear LPM0 bits to wake CPU (MS) */
        }
        break;
    default: break;
    }
}

/* ============================================================================
 * ESP32 HARDWARE UART DRIVER (YH)
 * ============================================================================
 */

 /**
  * @brief  Initializes USCI_A0 Hardware UART for ESP32 Communication.
  * @role   Configures P3.3 (TXD) and P3.4 (RXD) at 115200 Baud using SMCLK.
  * @param  None
  * @return None
  * @author Lee Yu Huan (YH)
  */
void esp32_uart_init(void) {
    P3SEL |= BIT3 + BIT4;                 /* Select P3.3 (TXD) and P3.4 (RXD) primary functions (YH) */
    P3DIR |= BIT3;                        /* Set P3.3 as TXD output (YH) */
    P3DIR &= ~BIT4;                       /* Set P3.4 as RXD input (YH) */

    UCA0CTL1 |= UCSWRST;                  /* Put USCI_A0 state machine in reset (YH) */
    UCA0CTL1 |= UCSSEL_2;                  /* Select SMCLK as clock source (YH) */
    UCA0BR0 = 9; UCA0BR1 = 0;              /* Set prescaler for 115200 Baud at 1.048 MHz SMCLK (YH) */
    UCA0MCTL = UCBRS_1 | UCBRF_0;          /* Configure modulation settings (YH) */
    UCA0CTL0 = 0x00;                      /* No parity, LSB first, 8-bit data, 1 stop bit (YH) */
    UCA0CTL1 &= ~UCSWRST;                 /* Release USCI_A0 from reset state (YH) */
}

/**
 * @brief  Transmits String Buffer over USCI_A0 UART to ESP32.
 * @param  str Pointer to null-terminated string payload.
 * @return None
 * @author Lee Yu Huan (YH)
 */
void esp32_uart_print(const char* str) {
    while (*str) {
        while (!(UCA0IFG & UCTXIFG));      /* Wait for USCI_A0 TX buffer ready flag (YH) */
        UCA0TXBUF = *str++;                /* Load character into TX buffer (YH) */
    }
    while (UCA0STAT & UCBUSY);             /* Poll until transmission completes (YH) */
}

/* ============================================================================
 * TOF DRIVERS, SIGNAL FILTERING, ADC12 & GESTURE FSM (LL)
 * ============================================================================
 */

 /**
  * @brief  Applies 0.9 Exponential Moving Average (EMA) Filter to Raw Distance Data.
  * @param  raw_val Current raw distance reading in mm.
  * @param  prev_ema Previous filtered EMA distance value in mm.
  * @return uint16_t Filtered distance value in mm.
  * @author Lex Lee Gao Hao (LL)
  */
uint16_t apply_ema(uint16_t raw_val, uint16_t prev_ema) {
    if (prev_ema == 0) return raw_val;    /* Pass raw reading if filter is uninitialized (LL) */
    return (uint16_t)(((uint32_t)EMA_ALPHA_NUM * raw_val +
        (uint32_t)(EMA_ALPHA_DEN - EMA_ALPHA_NUM) * prev_ema) / EMA_ALPHA_DEN); /* 0.9 EMA math (LL) */
}

/**
 * @brief  Blocking Microsecond Delay Helper.
 * @param  us Microseconds to delay.
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void delay_us(uint16_t us) {
    while (us--) __delay_cycles(1);        /* Execute single-cycle delay loop (LL) */
}

/**
 * @brief  Initializes MSP430 ADC12 Peripheral for Potentiometer Sampling.
 * @role   Configures P7.0 pin for analog function Channel A12, selects AVCC/AVSS
 *         reference voltages, and enables conversion timer.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void adc_init(void) {
    P7SEL |= BIT0;                        /* Configure P7.0 pin for secondary analog function A12 (LL) */
    ADC12CTL0 = ADC12SHT0_2 + ADC12ON;    /* 16 ADC clock sample time, turn ADC12 peripheral ON (LL) */
    ADC12CTL1 = ADC12SHP;                 /* Select internal pulse sample-and-hold timer (LL) */
    ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_12; /* VREF = AVCC/AVSS (3.3V), Connect Channel A12 to Memory 0 (LL) */
    ADC12CTL0 |= ADC12ENC;                /* Enable ADC conversions (LL) */
}

/**
 * @brief  Reads Raw 12-bit Digital Conversion Value from ADC12.
 * @param  None
 * @return uint16_t Raw 12-bit conversion value (0 - 4095 corresponding to 0V - 3.3V).
 * @author Lex Lee Gao Hao (LL)
 */
uint16_t adc_read_raw(void) {
    ADC12CTL0 |= ADC12SC;                 /* Start conversion software trigger (LL) */
    while (ADC12CTL0 & ADC12BUSY);       /* Poll until ADC conversion completes (LL) */
    return ADC12MEM0;                     /* Return 12-bit digital conversion result (LL) */
}

/**
 * @brief  Gesture Recognition Finite State Machine (FSM).
 * @role   Evaluates dual-sensor optical beam breaks, applies time windowing,
 *         and classifies motion into Swipe L->R, Swipe R->L, Pinch-In, or Pinch-Out events.
 * @param  d1 Smooth distance reading in mm from Sensor 1 (Left).
 * @param  d2 Smooth distance reading in mm from Sensor 2 (Right).
 * @return Gesture_t Classified gesture event enum.
 * @author Lex Lee Gao Hao (LL)
 */
static uint16_t recorded_transit_cycles = 0;

/**
 * @brief  Retrieves Measured Transit Duration for Last Swipe Gesture.
 * @param  None
 * @return uint16_t Transit duration in milliseconds.
 * @author Lex Lee Gao Hao (LL)
 */
uint16_t get_last_transit_cycles(void) {
    return recorded_transit_cycles;        /* Return stored swipe transit time in ms (LL) */
}

/**
 * @brief  Retrieves Calculated Speed Index for Last Pinch Gesture.
 * @param  None
 * @return uint16_t Speed index value (10 to 100).
 * @author Lex Lee Gao Hao (LL)
 */
uint16_t get_pinch_speed(void) {
    return pinch_speed_val;               /* Return stored pinch speed index (LL) */
}

Gesture_t detect_gesture(uint16_t d1, uint16_t d2) {
    static uint8_t s1_latched = 0;
    static uint8_t s2_latched = 0;
    static uint8_t s1_enter_debounce = 0;
    static uint8_t s2_enter_debounce = 0;
    static uint8_t s1_exit_debounce = 0;
    static uint8_t s2_exit_debounce = 0;
    static uint32_t s1_break_time = 0;
    static uint32_t s2_break_time = 0;
    static uint8_t pinch_out_fired = 0;
    static uint8_t pinch_in_armed = 0;
    static uint32_t dual_hold_start = 0;
    static uint32_t cooldown_timer = 0;

    if (system_ms_ticks < cooldown_timer) {
        return GESTURE_NONE;               /* Return idle state during active cooldown window (LL) */
    }

    uint8_t s1_now = (d1 > 0 && d1 < enter_thresh_d1);
    uint8_t s2_now = (d2 > 0 && d2 < enter_thresh_d2);

    /* Process Sensor 1 beam entry/exit debounce logic */
    if (s1_now) {
        s1_exit_debounce = 0;
        if (s1_enter_debounce < 2) s1_enter_debounce++;
        if (s1_enter_debounce >= 2 && !s1_latched) {
            s1_latched = 1;
            s1_break_time = system_ms_ticks; /* Latch Sensor 1 entry timestamp (LL) */
        }
    }
    else {
        s1_enter_debounce = 0;
        if (d1 >= exit_thresh_d1 || d1 == 0) {
            if (s1_exit_debounce < 2) s1_exit_debounce++;
            if (s1_exit_debounce >= 2) s1_latched = 0;
        }
    }

    /* Process Sensor 2 beam entry/exit debounce logic */
    if (s2_now) {
        s2_exit_debounce = 0;
        if (s2_enter_debounce < 2) s2_enter_debounce++;
        if (s2_enter_debounce >= 2 && !s2_latched) {
            s2_latched = 1;
            s2_break_time = system_ms_ticks; /* Latch Sensor 2 entry timestamp (LL) */
        }
    }
    else {
        s2_enter_debounce = 0;
        if (d2 >= exit_thresh_d2 || d2 == 0) {
            if (s2_exit_debounce < 2) s2_exit_debounce++;
            if (s2_exit_debounce >= 2) s2_latched = 0;
        }
    }

    uint8_t both_latched = (s1_latched && s2_latched);

    if (!s1_latched && !s2_latched) {
        pinch_out_fired = 0;
        pinch_in_armed = 0;
        dual_hold_start = 0;
    }

    /* Process dual-beam latch events (Pinch gestures) */
    if (both_latched) {
        uint32_t time_diff = (s1_break_time > s2_break_time) ?
            (s1_break_time - s2_break_time) : (s2_break_time - s1_break_time);

        /* Evaluate Pinch-Out condition (Simultaneous beam break <= 30 ms) */
        if (time_diff <= 30 && !pinch_out_fired && !pinch_in_armed) {
            pinch_out_fired = 1;
            uint32_t elapsed = (time_diff == 0) ? 1 : time_diff;
            pinch_speed_val = (uint16_t)(1000 / elapsed);
            if (pinch_speed_val < 10) pinch_speed_val = 10;
            if (pinch_speed_val > 100) pinch_speed_val = 100;
            cooldown_timer = system_ms_ticks + 350;
            s1_break_time = 0; s2_break_time = 0;
            return GESTURE_PINCH_OUT;      /* Return Pinch-Out gesture classification (LL) */
        }

        if (dual_hold_start == 0) {
            dual_hold_start = system_ms_ticks;
        }

        if (!pinch_out_fired && (system_ms_ticks - dual_hold_start) > 50) {
            pinch_in_armed = 1;            /* Arm Pinch-In gesture state (LL) */
        }

        return GESTURE_NONE;
    }

    /* Evaluate Pinch-In condition (Dual hold release) */
    if (pinch_in_armed && !pinch_out_fired && (!s1_latched || !s2_latched)) {
        uint32_t duration = system_ms_ticks - dual_hold_start;
        if (duration < 10) duration = 10;

        pinch_speed_val = (uint16_t)(3000 / duration);
        if (pinch_speed_val < 10) pinch_speed_val = 10;
        if (pinch_speed_val > 100) pinch_speed_val = 100;

        pinch_in_armed = 0;
        dual_hold_start = 0;
        cooldown_timer = system_ms_ticks + 350;
        s1_break_time = 0; s2_break_time = 0;
        return GESTURE_PINCH_IN;           /* Return Pinch-In gesture classification (LL) */
    }

    /* Evaluate sequential beam break events (Swipe gestures) */
    if (!s1_latched && !s2_latched && !pinch_out_fired) {
        if (s1_break_time > 0 && s2_break_time > 0) {
            uint32_t diff = (s1_break_time > s2_break_time) ?
                (s1_break_time - s2_break_time) : (s2_break_time - s1_break_time);

            if (diff > 30 && diff < 400) {
                if (s1_break_time < s2_break_time) {
                    recorded_transit_cycles = (uint16_t)diff;
                    s1_break_time = 0; s2_break_time = 0;
                    cooldown_timer = system_ms_ticks + 150;
                    return GESTURE_SWIPE_L_TO_R; /* Return Swipe L->R gesture classification (LL) */
                }
                else {
                    recorded_transit_cycles = (uint16_t)diff;
                    s1_break_time = 0; s2_break_time = 0;
                    cooldown_timer = system_ms_ticks + 150;
                    return GESTURE_SWIPE_R_TO_L; /* Return Swipe R->L gesture classification (LL) */
                }
            }
            s1_break_time = 0;
            s2_break_time = 0;
        }
    }

    if (s1_break_time > 0 && (system_ms_ticks - s1_break_time) > 350) s1_break_time = 0;
    if (s2_break_time > 0 && (system_ms_ticks - s2_break_time) > 350) s2_break_time = 0;

    return GESTURE_NONE;                   /* Return idle state (LL) */
}

/**
 * @brief  Clears Stuck I2C Bus Lines by Manual Clock Pulsing.
 * @role   Configures P2 and P3 bit-bang lines as output and pulses SCL line 9 times
 *         to release slave devices locking SDA low.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void force_bus_clear(void) {
    uint8_t i;
    P2DIR |= S1_SDA_PIN | S1_SCL_PIN;      /* Set Sensor 1 pins as outputs (LL) */
    P3DIR |= S2_SDA_PIN | S2_SCL_PIN;      /* Set Sensor 2 pins as outputs (LL) */

    for (i = 0; i < 9; i++) {              /* Pulse SCL clock line 9 times (LL) */
        P2OUT |= S1_SCL_PIN; P3OUT |= S2_SCL_PIN;
        __delay_cycles(50);
        P2OUT &= ~S1_SCL_PIN; P3OUT &= ~S2_SCL_PIN;
        __delay_cycles(50);
    }
    P2DIR &= ~(S1_SDA_PIN | S1_SCL_PIN);  /* Release Sensor 1 pins back to inputs (LL) */
    P3DIR &= ~(S2_SDA_PIN | S2_SCL_PIN);  /* Release Sensor 2 pins back to inputs (LL) */
}

/* --- PORT 2 BIT-BANG I2C DRIVER (SENSOR 1 - LEFT) (LL) --- */

/**
 * @brief  Initializes GPIO Register States for Sensor 1 I2C Interface.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void bb1_init(void) {
    P2DIR &= ~(S1_SDA_PIN | S1_SCL_PIN);  /* Set P2.0 and P2.2 as inputs (high-impedance) (LL) */
    P2OUT &= ~(S1_SDA_PIN | S1_SCL_PIN);  /* Drive output register low for open-drain simulation (LL) */
}

/**
 * @brief  Writes Power-On Boot Register Sequence to VL53L0X Sensor 1.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void bb1_init_sensor(void) {
    bb1_write_reg(0x88, 0x00);
    bb1_write_reg(0x80, 0x01);
    bb1_write_reg(0xFF, 0x01);
    bb1_write_reg(0x00, 0x00);
    bb1_write_reg(0x00, 0x01);
    bb1_write_reg(0xFF, 0x00);
    bb1_write_reg(0x80, 0x00);
    bb1_write_reg(0x83, 0x00);
    bb1_write_reg(0x00, 0x02);             /* Load boot register sequence into Sensor 1 (LL) */
}

/**
 * @brief  Generates I2C START Condition on Port 2.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void bb1_start(void) {
    P2DIR &= ~S1_SDA_PIN; delay_us(2);
    P2DIR &= ~S1_SCL_PIN; delay_us(2);
    P2DIR |= S1_SDA_PIN;  delay_us(2);
    P2DIR |= S1_SCL_PIN;  delay_us(2);    /* Generate I2C START signal (LL) */
}

/**
 * @brief  Generates I2C STOP Condition on Port 2.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void bb1_stop(void) {
    P2DIR |= S1_SDA_PIN;  delay_us(2);
    P2DIR &= ~S1_SCL_PIN; delay_us(2);
    P2DIR &= ~S1_SDA_PIN; delay_us(2);    /* Generate I2C STOP signal (LL) */
}

/**
 * @brief  Writes 8-bit Data Byte over I2C Bus on Port 2 and Reads ACK Flag.
 * @param  byte 8-bit value to transmit over I2C.
 * @return uint8_t 1 if ACK received, 0 if NACK.
 * @author Lex Lee Gao Hao (LL)
 */
uint8_t bb1_write_byte(uint8_t byte) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (byte & 0x80) P2DIR &= ~S1_SDA_PIN; else P2DIR |= S1_SDA_PIN;
        byte <<= 1; delay_us(1);
        P2DIR &= ~S1_SCL_PIN; delay_us(2);
        P2DIR |= S1_SCL_PIN;  delay_us(1);
    }
    P2DIR &= ~S1_SDA_PIN; delay_us(1);
    P2DIR &= ~S1_SCL_PIN; delay_us(2);
    uint8_t ack = !(P2IN & S1_SDA_PIN);    /* Read ACK bit from slave (LL) */
    P2DIR |= S1_SCL_PIN;  delay_us(1);
    return ack;
}

/**
 * @brief  Reads 8-bit Data Byte over I2C Bus on Port 2 and Transmits ACK/NACK.
 * @param  ack 1 to transmit ACK bit, 0 to transmit NACK bit.
 * @return uint8_t 8-bit value read from slave device.
 * @author Lex Lee Gao Hao (LL)
 */
uint8_t bb1_read_byte(uint8_t ack) {
    uint8_t i, byte = 0;
    P2DIR &= ~S1_SDA_PIN;
    for (i = 0; i < 8; i++) {
        byte <<= 1;
        P2DIR &= ~S1_SCL_PIN; delay_us(2);
        if (P2IN & S1_SDA_PIN) byte |= 0x01;
        P2DIR |= S1_SCL_PIN;  delay_us(1);
    }
    if (ack) P2DIR |= S1_SDA_PIN; else P2DIR &= ~S1_SDA_PIN;
    P2DIR &= ~S1_SCL_PIN; delay_us(2);
    P2DIR |= S1_SCL_PIN;  delay_us(1);
    P2DIR &= ~S1_SDA_PIN;
    return byte;                           /* Return received byte (LL) */
}

/**
 * @brief  Writes Single Register Value to VL53L0X Sensor 1 over I2C.
 * @param  reg Target 8-bit register address.
 * @param  val 8-bit value to write.
 * @return uint8_t 1 if write succeeded, 0 on bus error.
 * @author Lex Lee Gao Hao (LL)
 */
uint8_t bb1_write_reg(uint8_t reg, uint8_t val) {
    bb1_start();
    if (!bb1_write_byte(VL53L0X_ADDR << 1)) { bb1_stop(); return 0; }
    bb1_write_byte(reg);
    bb1_write_byte(val);
    bb1_stop();
    return 1;
}

/**
 * @brief  Reads 16-bit Range Distance Value in mm from VL53L0X Sensor 1.
 * @param  None
 * @return uint16_t Range distance in mm, or 8190 on error.
 * @author Lex Lee Gao Hao (LL)
 */
uint16_t bb1_read_range(void) {
    bb1_start();
    if (!bb1_write_byte(VL53L0X_ADDR << 1)) { bb1_stop(); return 8190; }
    bb1_write_byte(0x1E);
    bb1_start();
    bb1_write_byte((VL53L0X_ADDR << 1) | 1);
    uint8_t high = bb1_read_byte(1);
    uint8_t low = bb1_read_byte(0);
    bb1_stop();
    bb1_write_reg(0x0B, 0x01);
    return ((uint16_t)high << 8) | low;     /* Combine high and low byte distance (LL) */
}

/* --- PORT 3 BIT-BANG I2C DRIVER (SENSOR 2 - RIGHT) (LL) --- */

/**
 * @brief  Initializes GPIO Register States for Sensor 2 I2C Interface.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void bb2_init(void) {
#ifdef P3SEL
    P3SEL &= ~(S2_SDA_PIN | S2_SCL_PIN);   /* Clear P3.0 and P3.1 primary pin selection (LL) */
#endif
#ifdef P3SEL0
    P3SEL0 &= ~(S2_SDA_PIN | S2_SCL_PIN);
    P3SEL1 &= ~(S2_SDA_PIN | S2_SCL_PIN);
#endif

    P3DIR &= ~(S2_SDA_PIN | S2_SCL_PIN);  /* Set P3.0 and P3.1 as inputs (high-impedance) (LL) */
    P3OUT &= ~(S2_SDA_PIN | S2_SCL_PIN);  /* Drive output register low for open-drain simulation (LL) */
}

/**
 * @brief  Writes Power-On Boot Register Sequence to VL53L0X Sensor 2.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void bb2_init_sensor(void) {
    bb2_write_reg(0x88, 0x00);
    bb2_write_reg(0x80, 0x01);
    bb2_write_reg(0xFF, 0x01);
    bb2_write_reg(0x00, 0x00);
    bb2_write_reg(0x00, 0x01);
    bb2_write_reg(0xFF, 0x00);
    bb2_write_reg(0x80, 0x00);
    bb2_write_reg(0x83, 0x00);
    bb2_write_reg(0x00, 0x02);             /* Load boot register sequence into Sensor 2 (LL) */
}

/**
 * @brief  Generates I2C START Condition on Port 3.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void bb2_start(void) {
    P3DIR &= ~S2_SDA_PIN; delay_us(2);
    P3DIR &= ~S2_SCL_PIN; delay_us(2);
    P3DIR |= S2_SDA_PIN;  delay_us(2);
    P3DIR |= S2_SCL_PIN;  delay_us(2);    /* Generate I2C START signal (LL) */
}

/**
 * @brief  Generates I2C STOP Condition on Port 3.
 * @param  None
 * @return None
 * @author Lex Lee Gao Hao (LL)
 */
void bb2_stop(void) {
    P3DIR |= S2_SDA_PIN;  delay_us(2);
    P3DIR &= ~S2_SCL_PIN; delay_us(2);
    P3DIR &= ~S2_SDA_PIN; delay_us(2);    /* Generate I2C STOP signal (LL) */
}

/**
 * @brief  Writes 8-bit Data Byte over I2C Bus on Port 3 and Reads ACK Flag.
 * @param  byte 8-bit value to transmit over I2C.
 * @return uint8_t 1 if ACK received, 0 if NACK.
 * @author Lex Lee Gao Hao (LL)
 */
uint8_t bb2_write_byte(uint8_t byte) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (byte & 0x80) P3DIR &= ~S2_SDA_PIN; else P3DIR |= S2_SDA_PIN;
        byte <<= 1; delay_us(1);
        P3DIR &= ~S2_SCL_PIN; delay_us(2);
        P3DIR |= S2_SCL_PIN;  delay_us(1);
    }
    P3DIR &= ~S2_SDA_PIN; delay_us(1);
    P3DIR &= ~S2_SCL_PIN; delay_us(2);
    uint8_t ack = !(P3IN & S2_SDA_PIN);    /* Read ACK bit from slave (LL) */
    P3DIR |= S2_SCL_PIN;  delay_us(1);
    return ack;
}

/**
 * @brief  Reads 8-bit Data Byte over I2C Bus on Port 3 and Transmits ACK/NACK.
 * @param  ack 1 to transmit ACK bit, 0 to transmit NACK bit.
 * @return uint8_t 8-bit value read from slave device.
 * @author Lex Lee Gao Hao (LL)
 */
uint8_t bb2_read_byte(uint8_t ack) {
    uint8_t i, byte = 0;
    P3DIR &= ~S2_SDA_PIN;
    for (i = 0; i < 8; i++) {
        byte <<= 1;
        P3DIR &= ~S2_SCL_PIN; delay_us(2);
        if (P3IN & S2_SDA_PIN) byte |= 0x01;
        P3DIR |= S2_SCL_PIN;  delay_us(1);
    }
    if (ack) P3DIR |= S2_SDA_PIN; else P3DIR &= ~S2_SDA_PIN;
    P3DIR &= ~S2_SCL_PIN; delay_us(2);
    P3DIR |= S2_SCL_PIN;  delay_us(1);
    P3DIR &= ~S2_SDA_PIN;
    return byte;                           /* Return received byte (LL) */
}

/**
 * @brief  Writes Single Register Value to VL53L0X Sensor 2 over I2C.
 * @param  reg Target 8-bit register address.
 * @param  val 8-bit value to write.
 * @return uint8_t 1 if write succeeded, 0 on bus error.
 * @author Lex Lee Gao Hao (LL)
 */
uint8_t bb2_write_reg(uint8_t reg, uint8_t val) {
    bb2_start();
    if (!bb2_write_byte(VL53L0X_ADDR << 1)) { bb2_stop(); return 0; }
    bb2_write_byte(reg);
    bb2_write_byte(val);
    bb2_stop();
    return 1;
}

/**
 * @brief  Reads 16-bit Range Distance Value in mm from VL53L0X Sensor 2.
 * @param  None
 * @return uint16_t Range distance in mm, or 8190 on error.
 * @author Lex Lee Gao Hao (LL)
 */
uint16_t bb2_read_range(void) {
    bb2_start();
    if (!bb2_write_byte(VL53L0X_ADDR << 1)) { bb2_stop(); return 8190; }
    bb2_write_byte(0x1E);
    bb2_start();
    bb2_write_byte((VL53L0X_ADDR << 1) | 1);
    uint8_t high = bb2_read_byte(1);
    uint8_t low = bb2_read_byte(0);
    bb2_stop();
    bb2_write_reg(0x0B, 0x01);
    return ((uint16_t)high << 8) | low;     /* Combine high and low byte distance (LL) */
}

/* ============================================================================
 * INDIVIDUAL REFLECTION (Requirement 9, Item 4)
 * ============================================================================
 * TEAM MEMBER: Lex Lee Gao Hao (2501194) (LL)
 * Role: Gesture Recognition Engine, Bit-Bang I2C, Signal Filtering & ADC Tuning
 *
 * CODING EXPERIENCE & TECHNICAL CHALLENGES:
 * Developing the dual VL53L0X gesture recognition engine on the MSP430F5529 provided
 * valuable experience in real-time embedded system design. A primary technical challenge
 * was operating two identical ToF sensors on a single microcontroller. Because both
 * sensors default to address 0x29 on power-up, XSHUT pin re-addressing caused boot-up
 * instability. To resolve this, I implemented independent software Bit-Bang I2C drivers
 * across Port 2 (P2.0/P2.2) and Port 3 (P3.0/P3.1) to isolate communications cleanly.
 * Additionally, handling physical sensor noise and optical race conditions required
 * building a Finite State Machine (FSM) featuring a 2-frame debounce filter, gesture
 * cooldown windows, and 0.9 Exponential Moving Average (EMA) signal smoothing. To allow
 * adaptivity in different physical spaces, I integrated the MSP430 12-bit ADC (P7.0 / A12)
 * to sample an analog potentiometer for dynamic 100mm-400mm threshold tuning.
 *
* APPLICATION OF CEG2002 KNOWLEDGE:
 * 1. Digital I/O & General Purpose Pins: Utilized low-level GPIO register manipulation
 *    (P2DIR, P2OUT, P2IN, P3DIR, P3OUT, P3IN) to emulate software Bit-Bang I2C timing
 *    protocols for dual VL53L0X ToF sensor communication.
 * 2. Clocks & Hardware Timers: Integrated periodic 50 Hz non-blocking gesture processing
 *    within the system execution timebase driven by Timer_B0 (TIMER0_B0_ISR) operating
 *    on ACLK (32.768 kHz REFO).
 * 3. Analog / Digital Conversion (ADC): Configured the MSP430 ADC12 peripheral on pin P7.0
 *    (Channel A12) to sample analog potentiometer voltage, mapping 12-bit digital values
 *    directly to dynamic entry and exit distance thresholds.
 * 4. Embedded Communication (UART): Configured USCI_A1 hardware UART registers
 *    (P4.4/P4.5) at 115200 Baud using SMCLK for real-time PC serial debugging telemetry.


 * TEAM MEMBER: Manavalan Srihanush (2501002)
 * Role: Ultrasonic Approach Detection, Low-Power Mode Management, Inter-MCU
 *       Wake Control & Power Profiling
 *
 * CODING EXPERIENCE & TECHNICAL CHALLENGES:
 * The main lesson from building the HC-SR04 presence detector on the
 * MSP430F5529 was that low power and accurate timing pull in opposite
 * directions. The echo pulse needs a fast clock (SMCLK), but LPM3 switches
 * SMCLK off, so one timer could not do both jobs. I split the work: Timer_B0
 * on the 32.768 kHz ACLK is the wake-up tick, and Timer_A0 on SMCLK runs only
 * for the few milliseconds of each measurement. Polling the ECHO pin in a loop
 * would have kept the CPU busy and blocked sleep, so I used input capture and
 * let the CPU wait in LPM0 while the hardware timestamps both edges. The
 * sensor needed care too: ECHO is a 5 V output, so a 1k/2k divider protects
 * the 3.3 V pin, and a limited 5 ms echo window keeps the CPU awake only
 * briefly. To stop false wake-ups and flicker around 40 cm, I added a two-ping
 * confirmation, 5 cm hysteresis (40 cm to wake, 45 cm to sleep) and a 1 s
 * absence hold-off. A subtle bug risk was a tick arriving just before the CPU
 * entered sleep, which would delay the next poll, so interrupts are disabled
 * around the check-then-sleep step. Merging with Person 1 meant sharing the
 * resources (Timer_A0, Timer_B0, P1.2-P1.4) and giving them one 20 ms tick, plus a
 * wake line. Finally, the build failed under printf_support=minimal, so I
 * replaced sprintf with small integer-to-text helpers.
 *
 * APPLICATION OF CEG2002 KNOWLEDGE:
 * 1. Digital I/O (Lec 3): Used PxDIR to set TRIG as an output and ECHO as an
 *    input, PxSEL to hand the multi-function pin P1.2 over to Timer_A0 (TA0.1),
 *    and PxOUT for the 10 us trigger pulse, the wake line and the status LED.
 *    The 3.3 V pin levels drove the divider on the 5 V ECHO signal.
 * 2. Clocks & Timers (Lec 6): Selected REFO as ACLK through the Unified Clock
 *    System (UCSCTL4). Timer_A0 runs in continuous mode with CCR1 in capture
 *    mode (CM, CCIS, SCS, CAP) to measure echo width, and CCR2 in compare
 *    mode as a timeout. At about 1.05 MHz the 16-bit counter wraps every
 *    62.5 ms, longer than the echo window, so a simple subtraction is safe.
 *    TA0IV separates CCR1 and CCR2 inside the shared TIMER0_A1 vector, while
 *    Timer_B0 CCR0 uses its own dedicated vector.
 * 3. Interrupts & ISRs (Lec 4): The design is interrupt-driven instead of
 *    polled. ISRs stay short (store a timestamp or set a flag) and the
 *    distance maths runs in main. The CPU pushes PC and SR on the stack and
 *    clears GIE on entry, so __bic_SR_register_on_exit works by clearing the
 *    low-power bits in the saved SR. Because capture latches the timer in
 *    hardware, interrupt latency does not distort the echo measurement.
 * 4. Power Modes (Lec 8): The CPU sleeps in LPM3 (only ACLK active) between
 *    polls and uses LPM0 (SMCLK active) only while waiting for an echo, entered
 *    with __bis_SR_register(LPMx_bits + GIE). Peripherals on a disabled clock
 *    stop, so UART transmissions finish before sleeping and Timer_A0 is
 *    stopped between pings. The lecture's typical figures (~250 uA active,
 *    ~0.8 uA LPM3) motivated maximising LPM3 time. Low-power coding habits used:
 *    unsigned types, integer-only distance maths, and short ISRs. Time-in-mode
 *    counters give the data for the power comparison.
 * 5. Embedded Communication (Lec 7): Configured USCI_A1 as an asynchronous UART
 *    (UCSYNC cleared, 8N1) at 115200 Bd from SMCLK, using the baud-rate
 *    generator's modulation to approximate the baud rate. A simple
 *    unidirectional GPIO line (P1.4, active-high, rising edge) tells the second
 *    MCU that a user has arrived, so it can wake on a P1/P2 edge interrupt.
 *
 * TEAM MEMBER: Lee Yu Huan (2502685) (YH)
 * Role: ESP32 Hardware UART Interface (USCI_A0)
 *
 * CODING EXPERIENCE & TECHNICAL CHALLENGES:
 * During the development of AeroTouch, my main responsibility was
 * implementing the hardware UART interface on the MSP430F5529
 * to communicate with the ESP32-S3. I developed the UART
 * initialisation and transmission functions to send recognised
 * gesture commands from the MSP430 to the ESP32-S3.
 * One of the main tasks was configuring the USCI_A0 module,
 * assigning P3.3 as TX and P3.4 as RX. I selected SMCLK as the
 * clock source and configured the baud rate to 115200 with an
 * 8-N-1 communication format.
 * I also implemented the esp32_uart_print() function to transmit
 * commands character by character. The function checks the UCTXIFG
 * flag to ensure that the transmit buffer is ready before sending
 * each character. It also checks the UCBUSY flag to ensure that
 * transmission is completed.
 * An important integration challenge was ensuring that the
 * transmitted commands matched the format expected by the ESP32-S3.
 * I implemented four commands: SWIPE_LEFT, SWIPE_RIGHT, ZOOM_IN
 * and ZOOM_OUT, each ending with a newline character. This allowed
 * the ESP32-S3 to identify the received commands and perform the
 * corresponding USB HID actions.
 *
 * APPLICATION OF CEG2002 KNOWLEDGE:
 * 1. Digital I/O and General Purpose Pins:
 *    Configured P3.3 and P3.4 using P3SEL and P3DIR to assign
 *    the pins to the USCI_A0 UART peripheral for serial communication.
 * 2. Clock Configuration:
 *    Selected SMCLK using UCSSEL_2 as the clock source for UART
 *    baud-rate generation.
 * 3. Embedded Communication (UART):
 *    Configured UCA0CTL1, UCA0BR0, UCA0BR1 and UCA0MCTL for
 *    115200 baud asynchronous serial communication using 8 data
 *    bits, no parity and 1 stop bit.
 * 4. Register-Level Programming:
 *    Used UCA0TXBUF to transmit data and UCA0IFG with UCTXIFG
 *    to check whether the transmit buffer was ready. UCA0STAT
 *    and UCBUSY were used to check when transmission was complete.
 * ============================================================================
 */
