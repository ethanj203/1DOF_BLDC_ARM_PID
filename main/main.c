#include <stdio.h>
#include <stdint.h> 
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "mpu6050.h"


/*
Todo:
1. Setup I2C (FOR MPU) [Done]
2. Setup MPU6050 [Done]
3. Setup PWM Functionality [Done]
4. Setup Button Activation [Done]
5. Setup PID Control Loop [Done]
6. Success? (After Tesiting OFC) 
*/

/*
Original Issue:
- Error: Didn't work as a PID controller would
- Fix: Sign errors + forgot parathensis + etc in PID equation

- Error: Odd behavior
- Fix: Go back to more basic PID formula from Wikiapedia (instead of the "actual one above it" - likely due to how I was handeling error...)

- Error: Tuning it didn't help at all, felt like no matter how I set it (as in increase ki to it's limit) there would be SSE or just too unstable to safely use near my PSU)
- Fix: Changed how I mapped control to us, instead of trying to make "m" insto something fancy where in theory it would be semi-normalized then multiply 
       it by the max value I'd want, and then add "b", the motor spinning threshold. I chose to instead to throw that out the window. I made b a value that could hold it
       nearly steady at the horizontal point (max theoretical required thrust for application), and then made m = 1 so it was just us = control + motorbias. 
       
*/

//MPU - I2C Params
#define TAG             "BLDC_PID"
#define I2C_MASTER_NUM  I2C_NUM_0
#define I2C_MODE        I2C_MODE_MASTER
#define SDA_PIN         GPIO_NUM_21
#define SCL_PIN         GPIO_NUM_22
#define I2C_CLK_SPEED   400000

//LEDC Params
#define ESC_PIN         GPIO_NUM_19
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_RES        LEDC_TIMER_16_BIT
#define LEDC_TIMER      LEDC_TIMER_0
#define ESC_FREQ        50
#define LEDC_CLK        LEDC_AUTO_CLK
#define ESC_MIN_US      1000
#define ESC_IDLE_US     1225
#define ESC_MAX_US      1500

//Button Params
#define LED_PIN         GPIO_NUM_2
#define BUTTON_PIN      GPIO_NUM_18

//PID Controller Params
#define KP              0.5 //Stability limit @ kp = 0.675 (if touching side of mount)    Good Val: kp = 0.335
#define KI              2.00 //Limit of stability (kp = 0.XX) prev 0.25                     Good Val: ki = 0.3
#define KD              0.25 // Increasing it past this point makes derivative spike       Gool Val: kd = 0.15
#define MOTORBIAS       1300 //PWM -> us
#define MAXERRORSUM     500

#define RUNTIME         20 //seconds
#define DTMS            50  //~1ms interval for PID loop calcs (not including calc times)

//I2C Setup 
static void i2cConfig(void) {

    i2c_config_t i2cConf = {
        .mode =             I2C_MODE,
        .sda_io_num =       SDA_PIN,
        .sda_pullup_en =    GPIO_PULLUP_ENABLE,
        .scl_io_num =       SCL_PIN,
        .scl_pullup_en =    GPIO_PULLUP_ENABLE,
        .clk_flags =        0,
        .master.clk_speed = I2C_CLK_SPEED
    };

    i2c_param_config(I2C_MASTER_NUM, &i2cConf);
    i2c_driver_install(I2C_MASTER_NUM, i2cConf.mode, 0, 0, 0);

}

//LEDC Config
static void ledcSetup(void) {

    ledc_timer_config_t ledTimer = {

        .speed_mode =      LEDC_MODE,
        .duty_resolution = LEDC_RES,
        .timer_num =       LEDC_TIMER,
        .freq_hz =         ESC_FREQ,
        .clk_cfg =         LEDC_CLK
    };

    ledc_channel_config_t ledChannel = {

        .gpio_num =            ESC_PIN,
        .speed_mode =          LEDC_MODE,
        .channel =             LEDC_CHANNEL,
        .timer_sel =           LEDC_TIMER,
        .duty =                0,
        .hpoint =              0,
        .flags.output_invert = 0
    };

    ledc_timer_config(&ledTimer);
    ledc_channel_config(&ledChannel);

}

//Sets up button
static void buttonConfig(void) {

    esp_rom_gpio_pad_select_gpio(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_PIN);
    gpio_pulldown_dis(BUTTON_PIN);
    ESP_LOGI(TAG, "Button Configured!");

}

//Converts from time (us) to PWM Duty
static inline uint32_t us_to_duty(uint32_t us) {

    const uint32_t max_duty = (1u << LEDC_RES) - 1u;
    const uint32_t period_us = 1000000u / ESC_FREQ;
    uint64_t duty = (uint64_t)us * max_duty / period_us;
    
    return (duty > max_duty) ? max_duty : (uint32_t)duty;

}

//Writes to ESC
static inline void writeToESC(uint32_t us) {

    if (us < ESC_MIN_US) us = ESC_MIN_US;
    if (us > ESC_MAX_US) us = ESC_MAX_US;

    //ESP_LOGI(TAG, "us=%i", us);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, us_to_duty(us)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

}

//Checks for ButtonPress 
static inline uint32_t buttonPress(void) {

    bool pressState = false;

    while(1) {
        int buttonState = gpio_get_level(BUTTON_PIN);

        if (pressState == false && buttonState == 1) {     
            ESP_LOGI(TAG, "Button Pressed!");
            pressState = true;
            return 0;
        }
        else
        {
            if (pressState == true && buttonState == 0) {
                pressState = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

struct PIDVAL {
    double error;
    double errorSum;
    double control;
};


static inline struct PIDVAL PID(struct PIDVAL PIDVALS, float angleRead, float setPoint, float priorError, float kp, float ki, float kd, uint32_t dtMS) {

    PIDVALS.error = angleRead - setPoint; 

    double dt = dtMS / 1000.0; //converts from MS to S

    PIDVALS.errorSum += PIDVALS.error;

    if (PIDVALS.errorSum > MAXERRORSUM) {
        PIDVALS.errorSum = MAXERRORSUM;
    }
    if (PIDVALS.errorSum < -MAXERRORSUM) {
        PIDVALS.errorSum = -MAXERRORSUM;
    }

    double proportional = kp * PIDVALS.error;
    double integral = ki * PIDVALS.errorSum * dt;
    double derivative = kd * (PIDVALS.error - (double)priorError) / dt;

    //if (PIDVALS.error > 5 || PIDVALS.error < -5) {
    //    integral = 0.5 * integral;
    //}

    //PIDVALS.control = kp * PIDVALS.error + ki * (PIDVALS.error - priorError) * dt + kd * (PIDVALS.error - priorError) / dt;
    PIDVALS.control = proportional + integral + derivative;

    ESP_LOGI(TAG, "Angle=%f Error=%0.2f ErrorSum=%0.2f Proportional=%0.4f Integral=%0.4f Derivative=%0.4f Control=%0.4f", angleRead, PIDVALS.error, PIDVALS.errorSum, proportional, integral, derivative, PIDVALS.control);

    return PIDVALS;
}

//Mapping control signal to MS value between 1000us and 2000us - Linear interpolation
static inline int32_t controlToMs(double control) {

    int32_t us = (int32_t)control + MOTORBIAS;
    //ESP_LOGI(TAG, "us=%u, control=%0.3f", (unsigned)us, (double)control);

    return us;
}

static inline void motorRampFunct() {
    ESP_LOGI(TAG, "Ramping!");
    for (uint32_t i = ESC_IDLE_US; i < 1300; i += 10) {
        writeToESC(i);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "Ramped!");

}

void app_main(void)
{

    i2cConfig();
    ledcSetup();
    buttonConfig();

    //Define start vars

    uint32_t dtMS = DTMS;

    double kp = KP;
    double ki = KI;
    double kd = KD;

    double priorError = 0;
    //uint32_t PIDVALS.control = 0;

    double setPoint = 45.0; //Degrees

    uint32_t runTime = RUNTIME * 1000; //converts s to MS

    writeToESC(ESC_MIN_US); // Sets "throttle" to zero for ESC Init
    ESP_LOGI(TAG, "3s Delay for ESC Startup");
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Waiting for buttonpress to proceed!");

    buttonPress();

    ESP_LOGI(TAG, "MOVED PASS BUTTONPRESS");

    mpu6050_handle_t mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS); //Remember, have to actually assign the handle to something!

    if(!mpu6050) {
        ESP_LOGE(TAG, "MPU Handle Coulden't Be Created!");
    }

    mpu6050_wake_up(mpu6050);

    mpu6050_config(mpu6050, ACCE_FS_8G, GYRO_FS_1000DPS);

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    complimentary_angle_t angles;

    struct PIDVAL PIDVALS = {
        .error = 0.0,
        .errorSum = 0.0,
        .control = 0.0
    };

    motorRampFunct();

    long start = xTaskGetTickCount();

    /*
        Run loop with PID, Controll to Duty, Write to Duty to ESC functionality
        Make break condition for loop to be after x amount of seconds!
    */
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(runTime)) {

        mpu6050_get_acce(mpu6050, &acce);
        mpu6050_get_gyro(mpu6050, &gyro);

        mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &angles);

        //ESP_LOGI(TAG, "pitch=%0.2f, roll=%0.2f", angles.pitch, angles.roll);

        float angleRead = angles.roll; //Need to change depending on how the sensor is positioned final!

        PIDVALS = PID(PIDVALS, angleRead, setPoint, priorError, kp, ki, kd, dtMS);

        priorError = PIDVALS.error;

        uint32_t us = controlToMs(PIDVALS.control);

        //ESP_LOGI(TAG, "us=%u, error=%0.3f, priorError=%0.3f, control=%0.3f", (unsigned)us, (double)PIDVALS.error, (double)priorError, (double)PIDVALS.control);

        writeToESC(us);

        vTaskDelay(pdMS_TO_TICKS(dtMS));

    }

    ESP_LOGI(TAG, "Control Demo Complete");

    writeToESC(ESC_MIN_US);

    vTaskDelay(pdMS_TO_TICKS(500));

}


