// Luke Peter Spring 2026
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TaskHandle_t monitor_task_handle;
TaskHandle_t control_task_handle;

// LEDC Registers
#define SYSTEM_PERIP_CLK_EN0_REG 0x600C0018 
#define SYSTEM_PERIP_RST_EN0_REG 0x600C0020
#define LEDC_TIMER0_CONF_REG 0x600190A0  // set reference clock, divider, and duty cycle width
#define LEDC_CONF_REG 0x600190D0  // Enables and selects LEDC clock
#define LEDC_CH0_CONF0_REG 0x60019000  // idle value, enable, select timer
#define LEDC_CH0_CONF1_REG 0x6001900C  // commits configuration changes
#define LEDC_CH0_DUTY_REG 0x60019008  // duty cycle
#define LEDC_INT_RAW_REG 0x600190C0  // interrupt status
#define LEDC_INT_CLR_REG 0x600190CC  // clear interrupt
#define GPIO_FUNC5_OUT_SEL_CFG_REG 0x60004568

#define TASK_PERIOD_IN_MS 30
#define GOAL_RPM 1000
#define DIR 4
#define BRAKE 21

pcnt_unit_handle_t mypcnt = 0;
pcnt_channel_handle_t mychannel = 0;

// speed variables
float rpm = 0.0;
int val = 0;

// timing variables
uint64_t smTotal = 0;
uint64_t mcTotal = 0;
uint64_t start_time = 0;


// pid variables
float integral_error = 0;
float last_error = 0;
float Kp = 0.5, Ki = 0.1, Kd = 0.01;

// monitor the speed of the motor via PCNT
void speed_monitor (void *args) {
  uint64_t start_task;
  TickType_t xLastWakeTime = xTaskGetTickCount();  // current tick count of system
  TickType_t xTimeIncrement = pdMS_TO_TICKS(TASK_PERIOD_IN_MS);  // converts period into system ticks
 
  for (;;) {
    start_task = esp_timer_get_time();  // exact microsecond the task begins
  
    pcnt_unit_get_count(mypcnt, &val);  // read the counter
    // rotation per period = val/100, periods per minute = 60000/task period
    rpm = (val/200.0)*(60000/TASK_PERIOD_IN_MS); //val*(60000.0/TASK_PERIOD_IN_MS)/100.0;
    pcnt_unit_clear_count(mypcnt);  // clear after reading
  
    smTotal += esp_timer_get_time() - start_task;  // execution time on cpu
    vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);  // waits until xTimeIncrement has passed since xLastWakeTime, keeping timing right
  }
}

//set the LEDC PWM output value to control the motor
void motor_control (void *args) {
  uint64_t start_task;
  TickType_t xLastWakeTime = xTaskGetTickCount();  // current tick count of system
  TickType_t xTimeIncrement = pdMS_TO_TICKS(TASK_PERIOD_IN_MS);  // converts period into system ticks
  static uint32_t initial = 1;
  for (;;) {
    start_task = esp_timer_get_time();  // exact microsecond the task begins
  
    float error = GOAL_RPM - rpm;
    float derivative_error = error - last_error;
    integral_error = error + integral_error;
    //Serial.println(error);
    //Serial.println(derivative_error);
    //Serial.println(integral_error);

    float pid_output = Kp*error + Kd*derivative_error + Ki*integral_error; 
    if (pid_output < 0) { pid_output = 0; }
    if (pid_output > 4095) { pid_output = 4095; }  // cap to 0-4095
    last_error = error;

    update_PWM(initial, 4095 - (uint32_t)pid_output);
    initial = 0;
    //update_PWM(1, 4095);  
    mcTotal += esp_timer_get_time() - start_task;  // execution time on cpu
    vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);  // waits until xTimeIncrement has passed since xLastWakeTime, keeping timing right
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  start_time = esp_timer_get_time();

  Serial.println("Setting up LEDC... ");
  setup_LEDC();
  REG_CLR_BIT(GPIO_FUNC5_OUT_SEL_CFG_REG, 1 << 11); 
  REG_CLR_BIT(GPIO_FUNC5_OUT_SEL_CFG_REG, 1 << 10);
  //REG_WRITE(GPIO_FUNC5_OUT_SEL_CFG_REG, 73);
  REGISTER_SET_BITS(GPIO_FUNC5_OUT_SEL_CFG_REG, 0x1FF, 0, 73);
  Serial.println("LEDC Setup Complete");

  Serial.println("Setting up PCNT... ");
  setup_pcnt();
  Serial.println("PCNT Setup Complete");

  pinMode(DIR, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(DIR, LOW);
  digitalWrite(BRAKE, HIGH);
  xTaskCreate(speed_monitor,"monitor the speed of the motor via PCNT",1<<16,0,configMAX_PRIORITIES - 1, &monitor_task_handle);
  xTaskCreate(motor_control,"set the LEDC PWM output value to control the motor",1<<16,0,configMAX_PRIORITIES - 1, &control_task_handle);
}

void loop() {  // monitor cpu utilization of both tasks, report every second

  float cpu_util_sm = 100.0 * smTotal / (esp_timer_get_time() - start_time);  // speed monitor cpu utilization
  float cpu_util_mc = 100.0 * mcTotal / (esp_timer_get_time() - start_time);  // motor control cpu utilization

  Serial.print("Read rpm: ");
  Serial.println(rpm);

  Serial.print("Speed Monitor CPU Utilization: ");
  Serial.print(cpu_util_sm);
  Serial.println("%");
  Serial.print("Motor Control CPU Utilization: ");
  Serial.print(cpu_util_mc);
  Serial.println("%");

  delay(1000); // delay 1 second
}

void setup_pcnt() {
  pcnt_unit_config_t myunitconfig = {.low_limit = -1, .high_limit = (1<<14), .intr_priority = 0, .flags = { .accum_count=0 }};
  pcnt_chan_config_t mychannelconfig = {.edge_gpio_num = 6, .level_gpio_num = -1, .flags = 
    { .invert_edge_input = 0, .invert_level_input = 0, .virt_edge_io_level= 0, .io_loop_back = 0 } };
  pcnt_new_unit(&myunitconfig, &mypcnt);
  pcnt_new_channel(mypcnt,&mychannelconfig, &mychannel);
  pcnt_channel_set_edge_action(mychannel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_unit_clear_count(mypcnt);
  pcnt_unit_enable(mypcnt);
  pcnt_unit_start(mypcnt);
}

void setup_LEDC() {  // initializes LEDC 
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, 1 << 11);  // Turn on LEDC: set bit 11 in SYSTEM_PERIP_CLK_EN0_REG
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, 1 << 11);  // Take LEDC out of reset: clear bit 11 in SYSTEM_PERIP_RST_EN0_REG 
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, 1 << 11);  // (set and clear to ensure reset)
  REG_SET_BIT(LEDC_CONF_REG, 1 << 31);  // Set bit 31 to 1 to enable the clock to the LEDC’s registers
  REG_SET_BIT(LEDC_CONF_REG, 1 << 0);  // Set bit 0 to 1 to select the APB clock.
  // calculate Sample Rate
  float R = 8000.0;
  uint32_t whole =  floor(80e6 / (R*256));
  uint32_t frac =  (80e6/R - floor(80e6/R))*256;
  REGISTER_SET_BITS(LEDC_TIMER0_CONF_REG, 0xF, 0, 12);  // Set LEDC_DUTY_RES to 12
  REG_CLR_BIT(LEDC_TIMER0_CONF_REG, 1 << 23);  // Set LEDC_TIMER0_RST to 0 (bit 23).
  REGISTER_SET_BITS(LEDC_TIMER0_CONF_REG, 0x3FF << 12, 12, whole);  // Set LEDC_CLK_DIV_TIMERx to (10.8 bit divider value calculated above)
  REGISTER_SET_BITS(LEDC_TIMER0_CONF_REG, 0xFF << 4, 4, frac);  // factional portion
  REG_SET_BIT(LEDC_TIMER0_CONF_REG, 1 << 25);  // Set LEDC_TIMER0_PARA_UP (bit 25) to commit changes
  REG_SET_BIT(LEDC_CH0_CONF0_REG, 1 << 2); // Set bit 2 to enable LEDC.
  REGISTER_SET_BITS(LEDC_CH0_CONF0_REG, 0x3 << 0, 0, 0); // Set LEDC_TIMER_SEL_CH0 to 0 (selects timer).
  REG_SET_BIT(LEDC_CH0_CONF0_REG, 1 << 4);// Set LEDC_PARA_UP_CH0 (bit 4) to 1.
}

uint32_t sample_index = 0;
void update_PWM(uint32_t initial, uint32_t sample) {  
// If intitial isn't set, only update when the PWM timer overflows.
  if (!initial) {
    if ((REG_READ(LEDC_INT_RAW_REG) & (1 << 0)) == 0) {
        return;  // not time to update yet
    }
    // if the  LEDC_TIMER0_OVF_INT_RAW is set, clear it in LEDC_INT_CLR_REG
    REG_SET_BIT(LEDC_INT_CLR_REG, 1 << 0);
  }
  sample_index++;
  // Write the duty cycle into the whole bits of the LEDC_CHn_DUTY_REG
  REG_WRITE(LEDC_CH0_DUTY_REG, sample << 4);  // shift by 4 so fract bits are 0

  // In the LEDC_CH0_CONF1_REG, set the LEDC_DUTY_START_CH0 bit.
  REG_SET_BIT(LEDC_CH0_CONF1_REG, 1 << 31);

  // In the LEDC_CH0_CONF0_REG, set the LEDC_PARA_UP_CH0 bit.
  REG_SET_BIT(LEDC_CH0_CONF0_REG, 1 << 4);
}

void REGISTER_SET_BITS(uint32_t reg, uint32_t mask, uint32_t shift, uint32_t value) {
  uint32_t temp = REG_READ(reg);  // read current register
  temp &= ~mask;  // clear the bits
  temp |= (value << shift); // set the new bit values
  REG_WRITE(reg, temp);  // write to that register
}
