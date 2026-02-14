#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <esp_idf_lib_helpers.h>
#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "stdio.h"
#include "esp_adc/adc_cali.h"
#include "freertos/timers.h"
#include "driver/ledc.h"

#define PSEAT_PIN         GPIO_NUM_4       // passenger seat button pin 4
#define DSEAT_PIN         GPIO_NUM_5       // driver seat button pin 5
#define PBELT_PIN         GPIO_NUM_6       // passenger belt switch pin 6
#define DBELT_PIN         GPIO_NUM_7       // driver belt switch pin 7
#define IGNITION_BUTTON   GPIO_NUM_17      // ignition button pin 17
#define READY_LED         GPIO_NUM_8       // ready LED pin 8
#define SUCCESS_LED       GPIO_NUM_10      // success LED pin 10
#define ALARM_PIN         GPIO_NUM_12      // alarm pin 12
#define ADC_ATTEN         ADC_ATTEN_DB_12  // set ADC attenuation
#define BITWIDTH          ADC_BITWIDTH_12  // set ADC bitwidth
#define DELAY_MS          250              // delay in milliseconds
#define DELAY2_MS         2000             // delay in milliseconds
//LEDC config vars
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (9)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
//PWM signal frequency required by servo motor
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. 50 Hz for a 20ms period.
//minimum and maximum servo pulse widths
#define LEDC_DUTY_MIN           (220) // Set duty to 2.7% (0 deg angle position)
#define LEDC_DUTY_MAX           (590) // Set duty to 7.2% to achieve an angle of 90% (max)
//step sized to change how fast the servo motor rotates
#define STEP_HIGH_SPEED      (12.2) //speed fast -- 90 deg in 0.6 sec
#define STEP_LOW_SPEED       (4.92) //speed slow -- 90 deg in 1.5 sec
//ADC config vars
#define MODE_SELECTOR     ADC_CHANNEL_1 //MUST BE ADC CHANNEL
#define DELAY_TIME_SELECTOR     ADC_CHANNEL_0 //MUST BE ADC CHANNEL

bool dseat = false;     // Detects when the driver is seated 
bool pseat = false;     // Detects when the passenger is seated
bool dbelt = false;     // Detects when the driver seatbelt is on
bool pbelt = false;     // Detects when the passenger seatbelt is on
bool ignition = false;  // Detects when the ignition is turned on
int executed = 0;       // Keep track of print statements
int ready_led = 0;      // Keep track of whether ready_led should be on or off
int ignition_off = 0;   // Keep track of whether the ignition can be turned off
int task = 0;           // Keep track of which LCD message to print
int error = 0;          // Keep track of error state
int OFF = 1024;         // WindowWiper Subsystem mode OFF threshold
int INT = 2048;         // WindowWiper Subsystem mode INT threshold
int LOW = 3072;         // WindowWiper Subsystem mode LOW threshold
bool off_selected=false;        // Detects mode of WindowWiper Subsystem selected by driver - OFF
bool int_selected=false;        // Detects mode of WindowWiper Subsystem selected by driver - INT
bool low_selected=false;        // Detects mode of WindowWiper Subsystem selected by driver - LOW
bool high_selected=false;        // Detects mode of WindowWiper Subsystem selected by driver - HIGH
int timeDelaySel1 = 1365;        // Time Delay for WindowWiper Subsystem threshold 1
int timeDelaySel2 = 2730;        // Time Delay for WindowWiper Subsystem threshold 2
int INTtimeDelay;       // delay time for INT mode (ms)
int mode = 0;           // Keep track of which mode is selected
int wipers = 0;          // Keep track of whether wipers are on or off

static void ledc_init(void);

// Wiper system globals
int OFF = 1024;
int INT = 2048;
int LOW = 3072;
bool off_selected = false;
bool int_selected = false;
bool low_selected = false;
bool high_selected = false;
int timeDelaySel1 = 1365;
int timeDelaySel2 = 2730;
int INTtimeDelay = 1000;
int mode = 0;           // Current mode: 0=OFF, 1=INT, 2=LOW, 3=HIGH
int state = 0;          // Different wiper states: 0=PARKED, 1=WAIT, 2=UP, 3=DOWN
int timeInterval = 0;
float duty = LEDC_DUTY_MIN;
float current_step = STEP_LOW_SPEED;
float requested_step = STEP_LOW_SPEED;

// LCD global handling
hd44780_t lcd_display;

// Stop the servo motor at 0 degrees
void stop_servo_motor(void) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// LCD Task
void lcd_task(void *pvParameters)
{
    // Initialize LCD
    lcd_display.write_cb = NULL;
    lcd_display.font = HD44780_FONT_5X8;
    lcd_display.lines = 2;
    lcd_display.pins.rs = GPIO_NUM_38;
    lcd_display.pins.e  = GPIO_NUM_37;
    lcd_display.pins.d4 = GPIO_NUM_36;
    lcd_display.pins.d5 = GPIO_NUM_35;
    lcd_display.pins.d6 = GPIO_NUM_48;
    lcd_display.pins.d7 = GPIO_NUM_47;
    lcd_display.pins.bl = HD44780_NOT_USED;

    ESP_ERROR_CHECK(hd44780_init(&lcd_display));

    if (wipers == 0){
        hd44780_gotoxy(&lcd, 0, 0);
        hd44780_puts(&lcd, "Wiper mode:");
        if (mode == 0) {          //0-1023
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "OFF");
        }
        if (mode == 1) {   //1024-2047
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "INT");
        } 
        if (mode == 2) {   //2048-3071
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "LOW");
        } 
        if (mode == 3) {                  //3072-4095
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "HIGH");
        }
    }

    while (1)
    {
        // Clear the display
        hd44780_clear(&lcd_display);
        
        if (engine_running) {
            // Show "Wiper mode:" on first line
            hd44780_gotoxy(&lcd_display, 0, 0);
            hd44780_puts(&lcd_display, "Wiper mode:");
            
            // Show the current mode on second line
            hd44780_gotoxy(&lcd_display, 0, 1);
            if (mode == 0) {
                hd44780_puts(&lcd_display, "OFF");
            }
            else if (mode == 1) {
                // For INT mode, show the delay time too
                if (INTtimeDelay == 1000) {
                    hd44780_puts(&lcd_display, "INT-SHORT");
                }
                else if (INTtimeDelay == 3000) {
                    hd44780_puts(&lcd_display, "INT-MEDIUM");
                }
                else {
                    hd44780_puts(&lcd_display, "INT-LONG");
                }
            }
            else if (mode == 2) {
                hd44780_puts(&lcd_display, "LOW");
            }
            else if (mode == 3) {
                hd44780_puts(&lcd_display, "HIGH");
            }
        } else {
            // Engine is off
            hd44780_gotoxy(&lcd_display, 0, 0);
            hd44780_puts(&lcd_display, "Engine Off");
        }
        // Add delay
        vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
    }
}

//Initialize servo motor
static void ledc_init(void)
{
    // Timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void app_main(void)
{
    // Configure GPIO pins
    // set driver seat pin config to input and internal pullup
    gpio_reset_pin(DSEAT_PIN);
    gpio_set_direction(DSEAT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DSEAT_PIN);

    // set passenger seat pin config to input and internal pullup
    gpio_reset_pin(PSEAT_PIN);
    gpio_set_direction(PSEAT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(PSEAT_PIN);

    // set driver belt pin config to input and internal pullup
    gpio_reset_pin(DBELT_PIN);
    gpio_set_direction(DBELT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DBELT_PIN);

    // set passenger belt pin config to input and internal pullup
    gpio_reset_pin(PBELT_PIN);
    gpio_set_direction(PBELT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(PBELT_PIN);

    // set ignition button config to input and internal pullup
    gpio_reset_pin(IGNITION_BUTTON);
    gpio_set_direction(IGNITION_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(IGNITION_BUTTON);

    // set ready led pin config to output, level 0
    gpio_reset_pin(READY_LED);
    gpio_set_direction(READY_LED, GPIO_MODE_OUTPUT);

    // set success led pin config to output, level 0
    gpio_reset_pin(SUCCESS_LED);
    gpio_set_direction(SUCCESS_LED, GPIO_MODE_OUTPUT);

    // set alarm pin config to output, level 0
    gpio_reset_pin(ALARM_PIN);
    gpio_set_direction(ALARM_PIN, GPIO_MODE_OUTPUT);
    
    // Initialize servo motor
    ledc_init();
    // Start with motor stopped
    stop_servo_motor();
    
    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };                                                                  // Unit configuration
    adc_oneshot_unit_handle_t adc1_handle;                              // Unit handle
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle)); // Populate unit handle

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                  // Channel configuration

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MODE_SELECTOR, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, DELAY_TIME_SELECTOR, &chan_config));
    
    // Create LCD task
    xTaskCreate(lcd_task, "lcd_task", 4096, NULL, 5, NULL);
    
    int modeSel_adc_bits;
    int delayTimeSel_adc_bits;
    int loop_count = 0;
    
    while (1)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        loop_count++;

        // Read ADC values
        adc_oneshot_read(adc1_handle, MODE_SELECTOR, &modeSel_adc_bits);            // Read ADC bits (mode potentiometer)
        adc_oneshot_read(adc1_handle, DELAY_TIME_SELECTOR, &delayTimeSel_adc_bits); // Read ADC bits (delay potentiometer)

        // Read GPIO inputs
        dseat = gpio_get_level(DSEAT_PIN) == 0;
        pseat = gpio_get_level(PSEAT_PIN) == 0;
        dbelt = gpio_get_level(DBELT_PIN) == 0;
        pbelt = gpio_get_level(PBELT_PIN) == 0;
        ignition = gpio_get_level(IGNITION_BUTTON) == 0;
        
        // Determine wiper mode selection
        off_selected = (modeSel_adc_bits < OFF);
        int_selected = (modeSel_adc_bits >= OFF && modeSel_adc_bits < INT);
        low_selected = (modeSel_adc_bits >= INT && modeSel_adc_bits < LOW);
        high_selected = (modeSel_adc_bits >= LOW);

        //** Ignition Subsystem **//

        /* ------ Car Alarm Subsystem ------- */
        // if the driver seat button is pressed, print the welcome message once
        if (dseat){
            if (executed == 0){     // if executed equals 0, print welcome message
                printf("Welcome to enhanced alarm system model 218-W25\n"); 
                executed = 1;       // set executed = 1 so welcome message only prints once
            }
        }

        // Check if ignition is enabled
        bool ignition_enabled = dseat && pseat && dbelt && pbelt;
        
        // if all of the conditions are met
        if (ignition_enabled){
            //set ready led to ON
            if (executed == 1 && ready_led == 0){
                gpio_set_level(READY_LED, 1);
                ready_led = 1;
            }
            // if ignition button is pressed while all conditions are met
            if (ignition == true && executed == 1){
                // turn on ignition LED and turn off ready LED
                gpio_set_level(SUCCESS_LED, 1);
                gpio_set_level(READY_LED, 0);
                gpio_set_level(ALARM_PIN, 0);
                engine_running = true;
                // print engine started message once
                printf("Engine started!\n");
                executed = 2;       // set executed = 2 so engine started message only prints once
            }
        }

        // otherwise (at least one condition is not satisfied)
        else{
            // set ready LED to OFF and set variable ready_led to 0
            gpio_set_level(READY_LED,0);
            ready_led = 0;
            // if ignition button is pressed while conditions are not satisfied
            if (ignition==true && executed < 2){
                    // turn on alarm buzzer
                    gpio_set_level(ALARM_PIN, 1);
                    printf("Ignition inhibited.\n");
                    // check which conditions are not met, print corresponding message
                    if (!pseat){
                        printf("Passenger seat not occupied.\n");
                    }
                    if (!dseat){
                        printf("Driver seat not occupied.\n");
                    }
                    if (!pbelt){
                        printf("Passenger seatbelt not fastened.\n");
                    }
                    if (!dbelt){
                        printf("Drivers seatbelt not fastened.\n");
                    }
                    executed = 4;   
            }
        }

        // if executed = 4 (failed ignition) and ignition button is released
        if (ignition == false && executed == 4){
            // reset to state after welcome message, testing for conditions
            executed = 1;
        }

        // if ignition is successfully started and then ignition is released, set ignition_off = 1
        if (executed == 2 && ignition == false){
            ignition_off = 1;
        }

        // if ignition_off = 1 and inition is pressed, turn off all LEDs
        if (ignition_off==1 && ignition == true){
            engine_running = false;
            gpio_set_level(SUCCESS_LED,0);          // turn off ignition
            state = 0; // Reset wiper to park
            duty = LEDC_DUTY_MIN;
            stop_servo_motor();
            executed = 3;                           // set executed = 3 to keep LEDs off
        } 
        last_ignition_button = ignition;

        //** WINDSHIELD WIPER SUBSYSTEM **//
        
        if (engine_running) {
            // Determine delay time
            if (delayTimeSel_adc_bits < timeDelaySel1) {
                INTtimeDelay = 1000; // SHORT
            } else if (delayTimeSel_adc_bits < timeDelaySel2) {
                INTtimeDelay = 3000; // MEDIUM
            } else {
                INTtimeDelay = 5000; // LONG
            }

            // Determine mode (for display)
            int old_mode = mode;
            if (off_selected) {
                mode = 0;
            }
            else if (int_selected) {
                mode = 1;
            }
            else if (low_selected) {
                mode = 2;
            }
            else {
                mode = 3;
            }

            // Determine selected speed
            if (high_selected) {
                requested_step = STEP_HIGH_SPEED;
            } else {
                requested_step = STEP_LOW_SPEED;
            }

            // States for wiper control
            int old_state = state;
            bool entering_new_state = false;
            
            // Wipers parked at 0 degrees
            if (state == 0) {
                // Only set position when first entering this state
                if (old_state != 0) {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    entering_new_state = true;
                }
                
                // Start moving if mode selected
                if (int_selected) {
                    state = 1; // wait at 0 degrees
                    timeInterval = 0;
                    current_step = requested_step; // set requested speed
                } else if (low_selected || high_selected) {
                    state = 2; // start moving up immediately
                    current_step = requested_step; // set speed
                }
            }
            
            // INT mode (waiting at 0 degrees for the selected delay time)
            else if (state == 1) {
                timeInterval += 20;
                
                // Only set position when first entering wait state
                if (old_state != 1) {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    entering_new_state = true;
                }
                
                // Check if we should exit wait
                if (off_selected) {
                    state = 0; // Stay parked
                } else if (timeInterval >= INTtimeDelay) {
                    state = 2; // Start moving up
                }
            }
            
            // Wipers moving up from 0 to 90 degrees
            else if (state == 2) {
                duty += current_step;
                if (duty >= LEDC_DUTY_MAX) {
                    duty = LEDC_DUTY_MAX;
                    state = 3; // Start moving down
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (int)duty);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            }
            
            // Wipers moving down from 90 to 0 degrees
            else if (state == 3) {
                duty -= current_step;
                if (duty <= LEDC_DUTY_MIN) {
                    duty = LEDC_DUTY_MIN;
                    // Update speed for next cycle
                    current_step = requested_step;
                    // Determine next state based on mode
                    if (off_selected) {
                        state = 0; // reset wipers to park
                    } else if (int_selected) {
                        state = 1; // have wipers wait
                        timeInterval = 0;
                    } else {
                        state = 2; // Continuous (LOW/HIGH)
                    }
                } else {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (int)duty);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                }
            }
            
        } else {
            // Engine off, make sure wipers are parked
            if (state != 0) {
                state = 0;
                duty = LEDC_DUTY_MIN;
                stop_servo_motor();
            }
        }

        /* ------- Window Wiper Subsystem -------- */
        //determine the delay time selected by driver
        //0-1364 - SHORT
        if (delayTimeSel_adc_bits<timeDelaySel1) { INTtimeDelay=1000;} 
        //1365-2729 - MEDIUM
        else if (delayTimeSel_adc_bits<timeDelaySel2) {INTtimeDelay=3000;} 
        //2730-4095 -- LONG
        else { INTtimeDelay=5000;}

        // read from Mode Selector potentiometer & determine the selected mode
        // OFF or engine off behavior
        if (off_selected) {
            if (state == 0) {
                state = 0;   // already parked
            } else {
                state = 4;   // ALWAYS finish cycle and park
            }
        }
        // INT mode
        else if (int_selected) {
            if (state == 0) {
                state = 1;
                timeInterval = 0;
            }
        }
        // LOW speed mode
        else if (low_selected) {
            if (state == 0) {
                state = 2;
            }
        }
        // HIGH speed mode
        else if (high_selected) {
            if (state == 0) {
                state = 2;
            }
        }

        //determine step that we'll use to move wiper up/down
        if (high_selected) {requested_step = STEP_HIGH_SPEED;} 
        else {requested_step = STEP_LOW_SPEED; }

        //change states logic
        switch (state) {
            case 1:     //wait
                timeInterval += 20;

                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

                if (timeInterval >= INTtimeDelay && !off_selected) {
                    state = 2;
                }
                break;

            case 2://MOVE FROM 0 to 90
                duty += current_step;
                if (duty >= LEDC_DUTY_MAX) {
                    duty = LEDC_DUTY_MAX;
                    state = 3;
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                break;

            case 3: //MOVE FROM 90 to 0
                duty -= current_step;
                if (duty <= LEDC_DUTY_MIN) {
                    duty = LEDC_DUTY_MIN;

                    //switch speeds only after the previous cycle is complete
                    current_step = requested_step;

                    if (off_selected) {
                    // if (off_selected || state == 4) {
                        state = 0;       // park and stop
                    }
                    else if (int_selected) {
                        state = 1;
                        timeInterval = 0;
                    }
                    else {
                        state = 2;    // continuous LOW/HIGH
                    }
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                break;

            case 4: //FINISH CYCLE
                // always move toward 0°
                duty -= current_step;
                if (duty <= LEDC_DUTY_MIN) {
                    duty = LEDC_DUTY_MIN;

                    //switch speeds only after the previous cycle is complete
                    current_step = requested_step;

                    state = 0;   // parked
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                break;

            case 0: //STOP
            default:
                // parked and stationary
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                break;
        }
    
    /* //OLD VERSION WITH TASKS YOU ADDED
            //determine the delay time selected by driver -- NEEDS IMPROVEMENT!
        if (delayTimeSel_adc_bits<1365) { //0-1364
            //1 sec
            INTtimeDelay=1000;
        } else if (delayTimeSel_adc_bits<2730) {    //1365-2729
            //3sec
            INTtimeDelay=3000;
        } else {    //2730-4095
            //5sec
            INTtimeDelay=5000;
        }

        // read from Mode Selector potentiometer & determine the selected mode
        if (modeSel_adc_bits<OFF) {     //0-1023
            // MODE SELECTED: OFF
            // printf("OFF\n");
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        } else if (modeSel_adc_bits<INT) {      //1024-2047
            // MODE SELECTED: INT
            // printf("INT\n");
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(INTtimeDelay));
            //go from 0 to 90 in LOW SPEED
            for (float i=LEDC_DUTY_MIN; i<= LEDC_DUTY_MAX; i+=STEP_LOW_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);    
            }
            // go from 90 to 0 in LOW SPEED
            for (float i=LEDC_DUTY_MAX; i>=LEDC_DUTY_MIN; i-=STEP_LOW_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);
            }
        } else if (modeSel_adc_bits<LOW) {      //2048-3071
            // MODE SELECTED: LOW
            //go from 0 to 90 in LOW SPEED
            for (float i=LEDC_DUTY_MIN; i<= LEDC_DUTY_MAX; i+=STEP_LOW_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);    
            }
            // go from 90 to 0 in LOW SPEED
            for (float i=LEDC_DUTY_MAX; i>=LEDC_DUTY_MIN; i-=STEP_LOW_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);
            }
        } else {                        //3072-4095
            // MODE SELECTED: HIGH
            // printf("HIGH\n");
            //go from 0 to 90 in HIGH SPEED
            for (float i=LEDC_DUTY_MIN; i<= LEDC_DUTY_MAX; i+=STEP_HIGH_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);    
            }
            // go from 90 to 0 in HIGH SPEED
            for (float i=LEDC_DUTY_MAX; i>=LEDC_DUTY_MIN; i-=STEP_HIGH_SPEED) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);
            }
        }
    */

    }
}
