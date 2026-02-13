#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <esp_idf_lib_helpers.h>
#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "stdio.h"
#include "esp_adc/adc_cali.h"
#include "freertos/timers.h"


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
#define DELAY_MS          1000             // delay in milliseconds
#define DELAY2_MS         2000             // delay in milliseconds
#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO    9
#define LEDC_CHANNEL      LEDC_CHANNEL_0
#define LEDC_DUTY_RES     LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
//PWM signal frequency required by servo motor
#define LEDC_FREQUENCY    50 // Frequency in Hertz. 50 Hz for a 20ms period.
//minimum and maximum servo pulse widths
#define LEDC_DUTY_MIN     220 // Set duty to 2.7% (0 deg angle position)
#define LEDC_DUTY_MAX     590 // Set duty to 7.2% to achieve an angle of 90% (max)
//step sized to change how fast the servo motor rotates
#define STEP_HIGH_SPEED   6.1 //speed fast -- 90 deg in 0.6 sec
#define STEP_LOW_SPEED    2.46 //speed slow -- 90 deg in 1.5 sec
#define MODE_SELECTOR     ADC_CHANNEL_4 //MUST BE ADC CHANNEL -- need to CHANGE!!!
#define DELAY_TIME_SELECTOR     ADC_CHANNEL_3 //MUST BE ADC CHANNEL -- need to CHANGE!!!
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define BITWIDTH        ADC_BITWIDTH_12

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

static void ledc_init(void);

void lcd_task(void *pvParameters)
{
    hd44780_t lcd =
    {
        .write_cb = NULL,
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .rs = GPIO_NUM_38,
            .e  = GPIO_NUM_37,
            .d4 = GPIO_NUM_36,
            .d5 = GPIO_NUM_35,
            .d6 = GPIO_NUM_48,
            .d7 = GPIO_NUM_47,
            .bl = HD44780_NOT_USED
        }
    };

    ESP_ERROR_CHECK(hd44780_init(&lcd));

    if (task == 0){
        hd44780_clear(&lcd);
    }

    if (task == 1){
        hd44780_gotoxy(&lcd, 0, 0);
        hd44780_puts(&lcd, "Welcome to car");
        hd44780_gotoxy(&lcd, 0, 1);
        hd44780_puts(&lcd, "alarm system!");
    }

    if (task == 2){
        hd44780_gotoxy(&lcd, 0, 0);
        hd44780_puts(&lcd, "Engine started!");
    }

    if (task == 3){
        hd44780_gotoxy(&lcd, 0, 0);
        hd44780_puts(&lcd, "Ignition");
        hd44780_gotoxy(&lcd, 0, 1);
        hd44780_puts(&lcd, "Inhibited!");
        vTaskDelay(DELAY2_MS / portTICK_PERIOD_MS);

        if (!pseat){
            hd44780_gotoxy(&lcd, 0, 0);
            hd44780_puts(&lcd, "Passenger seat");
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "not occupied.");
            vTaskDelay(DELAY2_MS / portTICK_PERIOD_MS);
        }
        if (!dseat){
            hd44780_gotoxy(&lcd, 0, 0);
            hd44780_puts(&lcd, "Driver seat");
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "not occupied.");
            vTaskDelay(DELAY2_MS / portTICK_PERIOD_MS);
        }
        if (!pbelt){
            hd44780_gotoxy(&lcd, 0, 0);
            hd44780_puts(&lcd, "Pass. seatbelt");
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "not fastened.");
            vTaskDelay(DELAY2_MS / portTICK_PERIOD_MS);
        }
        if (!dbelt){
            hd44780_gotoxy(&lcd, 0, 0);
            hd44780_puts(&lcd, "Driver seatbelt");
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "not fastened.");
            vTaskDelay(DELAY2_MS / portTICK_PERIOD_MS);
        }
    }

    if (task == 4){
        hd44780_gotoxy(&lcd, 0, 0);
        hd44780_puts(&lcd, "Wiper mode:");
        if (modeSel_adc_bits<OFF) {          //0-1023
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "OFF");
        } else if (modeSel_adc_bits<INT) {   //1024-2047
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "INT");
        } else if (modeSel_adc_bits<LOW) {   //2048-3071
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "LOW");
        } else {                             //3072-4095
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "HIGH");
        }
        vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
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

    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 3.75% (0 degrees) & Update duty
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // Initalize ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };                                                  // Unit configuration
    adc_oneshot_unit_handle_t adc1_handle;              // Unit handle
    adc_oneshot_new_unit(&init_config1, &adc1_handle);  // Populate unit handle

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
    adc_oneshot_config_channel(adc1_handle, MODE_SELECTOR, &chan_config);     // Configure the chan
    adc_oneshot_config_channel(adc1_handle, DELAY_TIME_SELECTOR, &chan_config);     // Configure the chan

    int modeSel_adc_bits;                                   // ADC reading (bits)
    int delayTimeSel_adc_bits;                               // ADC reading (bits)

    while (1){

        // Task Delay to prevent watchdog
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // initialize variables in relation to GPIO pin inputs
        dseat = gpio_get_level(DSEAT_PIN)==0;
        pseat = gpio_get_level(PSEAT_PIN)==0;
        dbelt = gpio_get_level(DBELT_PIN)==0;
        pbelt = gpio_get_level(PBELT_PIN)==0;
        ignition = gpio_get_level(IGNITION_BUTTON)==0;

        // if the driver seat button is pressed, print the welcome message once
        if (dseat){
            if (executed == 0){     // if executed equals 0, print welcome message
                printf("Welcome to enhanced alarm system model 218-W25 \n"); 
                task = 1;
                xTaskCreate(lcd_task, "lcd_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
                executed = 1;       // set executed = 1 so welcome message only prints once
            }
        }

        // if all of the conditions are met
        if (dseat && pseat && dbelt && pbelt){
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
                // print engine started message once
                printf("Engine started!\n");
                task = 2;
                xTaskCreate(lcd_task, "lcd_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
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
                    task = 3;
                    xTaskCreate(lcd_task, "lcd_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
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
            gpio_set_level(SUCCESS_LED,0);          // turn off ignition LED
            executed = 3;                           // set executed = 3 to keep LEDs off
            task = 0;                               // set task = 0 to clear LCD
            xTaskCreate(lcd_task, "lcd_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
        }

        //WINDOW WIPER CONTROL
        adc_oneshot_read(adc1_handle, MODE_SELECTOR, &modeSel_adc_bits);    // Read ADC bits
        adc_oneshot_read(adc1_handle, DELAY_TIME_SELECTOR, &delayTimeSel_adc_bits);    // Read ADC bits
        int INTtimeDelay;

        //determine the delay time selected by driver -- NEEDS IMPROVEMENT!
        if (delayTimeSel_adc_bits<1365) {           //0-1364
            //1 sec
            INTtimeDelay=1000;
        } else if (delayTimeSel_adc_bits<2730) {    //1365-2729
            //3sec
            INTtimeDelay=3000;
        } else {                                    //2730-4095
            //5sec
            INTtimeDelay=5000;
        task = 4;
        xTaskCreate(lcd_task, "lcd_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
        }

        // read from Mode Selector potentiometer & determine the selected mode
        if (modeSel_adc_bits<OFF) {                     //0-1023
            // MODE SELECTED: OFF
            // printf("OFF\n");
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        } else if (modeSel_adc_bits<INT) {              //1024-2047
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
        } else if (modeSel_adc_bits<LOW) {              //2048-3071
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
        } else {                                        //3072-4095
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
    }
}

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set initial duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}