// Luke Peter CSCE491 Spring 2026
#include <math.h>
#include "array.h" // audio sample array
#define SAMPLE_LENGTH (sizeof(sampleArray) / sizeof(sampleArray[0]))

// RMT Registers
#define SYSTEM_PERIP_CLK_EN0_REG 0x600C0018  // turns on RMT
#define SYSTEM_PERIP_RST_EN0_REG 0x600C0020  // disable reset of RMT
#define RMT_SYS_CONF_REG 0x600160C0  // enable RMT RAM
#define RMT_CH0_CONF0_REG 0x60016020  // set clock divider, disable carrier
#define RMT_INT_RAW_REG 0x60016070  // check if RMT is finished transmitting
#define RMT_INT_CLR_REG 0x6001607C  // clear the interrupt indicating completion
#define RMT_RAM 0x60016800  // array of 25 values to define output pattern
// LEDC Registers
#define LEDC_TIMER0_CONF_REG 0x600190A0  // set reference clock, divider, and duty cycle width
#define LEDC_CONF_REG 0x600190D0  // Enables and selects LEDC clock
#define LEDC_CH0_CONF0_REG 0x60019000  // idle value, enable, select timer
#define LEDC_CH0_CONF1_REG 0x6001900C  // commits configuration changes
#define LEDC_CH0_DUTY_REG 0x60019008  // duty cycle
#define LEDC_INT_RAW_REG 0x600190C0  // interrupt status
#define LEDC_INT_CLR_REG 0x600190CC  // clear interrupt

#define GPIO_FUNC14_OUT_SEL_CFG_REG 0x6000458C//0x58C  // pin 14, LEDC, GPIO_FUNC_OUT_SEL 73
#define GPIO_FUNC33_OUT_SEL_CFG_REG 0x600045D8 //0x5D8  // pin 33, RMT, GPIO_FUNC_OUT_SEL 81

#define NUM_LEDS 100
// RMT timing 80 MHz is 12.5 ns per clk cycle
#define T1H 64   // 800 ns
#define T1L 36   // 450 ns
#define T0H 32   // 400 ns
#define T0L 68   // 850 ns

// Pin 14 IO_MUX_GPIO14_REG MCU_SEL 2
// Pin 33 IO_MUX_GPIO33_REG MCU_SEL 2


void REGISTER_SET_BITS(uint32_t reg, uint32_t mask, uint32_t shift, uint32_t value) {
  uint32_t temp = REG_READ(reg);  // read current register
  temp &= ~mask;  // clear the bits
  temp |= (value << shift); // set the new bit values
  REG_WRITE(reg, temp);  // write to that register
}

void setup_RMT() {  // initializes RMT 
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, 1 << 9);
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, 1 << 9);  // set to 1 first
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, 1 << 9);  // then 0 to ensure reset
  REG_CLR_BIT(RMT_SYS_CONF_REG, 1 << 0);  // Set RMT_APB_FIFO_MASK to 0.
  REG_SET_BIT(RMT_SYS_CONF_REG, 1 << 31);  // Set RMT_CLK_EN (bit 31) to 1.
  REG_SET_BIT(RMT_SYS_CONF_REG, 1 << 1);  // Set RMT_MEM_CLK_FORCE_ON (bit 1) to 1.
  REGISTER_SET_BITS(RMT_SYS_CONF_REG, 0xFF << 4, 4, 0); // Set RMT_SCLK_DIV_NUM to 0 (sets clock divider to 1; NUM+1)
  REG_CLR_BIT(RMT_CH0_CONF0_REG, 1 << 21);  // Set RMT_CARRIER_EN_CH0 to 0 to disable RMT carrier frequency.
  REGISTER_SET_BITS(RMT_CH0_CONF0_REG, 0xFF << 8, 8, 1);  // Set RMT_DIV_CNT_CH0 to 1.
  REG_SET_BIT(RMT_CH0_CONF0_REG, 1 << 6);  // Set RMT_IDLE_OUT_EN_CH0 to 1 to drive the output to logic-0 when the RMT is not transmitting (this enables the WS2812 reset functionality).
  REG_CLR_BIT(RMT_CH0_CONF0_REG, 1 << 5);  // Set the RMT_IDLE_OUT_LV_CH0 to 0 to set the output to 0 when the RMT is idle.
  REG_SET_BIT(RMT_CH0_CONF0_REG, 1 << 24);  // Set RMT_CONF_UPDATE_CH0 (bit 24) to 1.
  REG_SET_BIT(RMT_INT_CLR_REG, (1 << 0));  // suggested by instructions
}

void setup_LEDC() {  // initializes LEDC 
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, 1 << 11);  // Turn on LEDC: set bit 11 in SYSTEM_PERIP_CLK_EN0_REG
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, 1 << 11);  // Take LEDC out of reset: clear bit 11 in SYSTEM_PERIP_RST_EN0_REG 
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, 1 << 11);  // (set and clear to ensure reset)
  REG_SET_BIT(LEDC_CONF_REG, 1 << 31);  // Set bit 31 to 1 to enable the clock to the LEDC’s registers
  REG_SET_BIT(LEDC_CONF_REG, 1 << 0);  // Set bit 0 to 1 to select the APB clock.
  // calculate Sample Rate
  float R = sampleRate;  // sandstorm rate from conversion file
  uint32_t whole =  floor(80e6 / (R*256));
  uint32_t frac =  (80e6/R - floor(80e6/R))*256;
  REGISTER_SET_BITS(LEDC_TIMER0_CONF_REG, 0xF, 0, 8);  // Set LEDC_DUTY_RES to 8 (as given by the sample size in your audio file).
  REG_CLR_BIT(LEDC_TIMER0_CONF_REG, 1 << 23);  // Set LEDC_TIMER0_RST to 0 (bit 23).
  REGISTER_SET_BITS(LEDC_TIMER0_CONF_REG, 0x3FF << 12, 12, whole);  // Set LEDC_CLK_DIV_TIMERx to (10.8 bit divider value calculated above)
  REGISTER_SET_BITS(LEDC_TIMER0_CONF_REG, 0xFF << 4, 4, frac);  // factional portion
  REG_SET_BIT(LEDC_TIMER0_CONF_REG, 1 << 25);  // Set LEDC_TIMER0_PARA_UP (bit 25) to commit changes
  REG_SET_BIT(LEDC_CH0_CONF0_REG, 1 << 2); // Set bit 2 to enable LEDC.
  REGISTER_SET_BITS(LEDC_CH0_CONF0_REG, 0x3 << 0, 0, 0); // Set LEDC_TIMER_SEL_CH0 to 0 (selects timer).
  REG_SET_BIT(LEDC_CH0_CONF0_REG, 1 << 4);// Set LEDC_PARA_UP_CH0 (bit 4) to 1.
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  //Serial.println("Booting...");

  //Serial.println("Calling setup_RMT()");
  setup_RMT();
  //Serial.println("setup_RMT done");

  //Serial.println("Calling setup_LEDC()");
  setup_LEDC();
  //Serial.println("setup_LEDC done");

  //Serial.println("Configuring GPIO");
  REG_CLR_BIT(GPIO_FUNC33_OUT_SEL_CFG_REG, 1 << 11);  // GPIO_FUNCx_OEN_INV_SEL controls if the output is inverted (set to 0)
  REG_CLR_BIT(GPIO_FUNC33_OUT_SEL_CFG_REG, 1 << 10);  // GPIO_FUNCx_OEN_SEL determines if the output enable is controlled by a separate output enable
  REG_CLR_BIT(GPIO_FUNC14_OUT_SEL_CFG_REG, 1 << 11);  //register or by the peripheral (set to 0)
  REG_CLR_BIT(GPIO_FUNC14_OUT_SEL_CFG_REG, 1 << 10); 
  

  REGISTER_SET_BITS(GPIO_FUNC14_OUT_SEL_CFG_REG, 0x1FF, 0, 81);
  REGISTER_SET_BITS(GPIO_FUNC33_OUT_SEL_CFG_REG, 0x1FF, 0, 73);
  //Serial.println("GPIO configured");
}

// updates the PWM output. The parameter initial is a flag that specifies if this is the first time
// the PWM is being output, and sample is the sample (duty cycle value) to output. The function
// should check the LEDC_INT_RAW_REG to determine if it is time to update the PWM value. If not,
// the function should return without writing to any peripherals unless the initial flag is set.
uint32_t sample_index = 0;
void update_PWM(uint32_t initial,uint32_t sample) {  
// If intitial isn't set, only update when the PWM timer overflows.
  // Serial.println("initial update PWm =");
  // Serial.print(initial);
  // Serial.println(" sample=");
  // Serial.print(sample);
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


// Sending a 1-bit requires a high period, or T1H, of 800 ns and a low period, or T1L of 450 ns, while a
//0-bit requires a high period, or T0H, of 400 ns and a low period, or T0L, of 850 ns.
// update all five LEDs to colors as given by the array colors
void transmit_led_signal(uint32_t *colors) {
  Serial.println("transmit_led_signal start");
  for (int led = 0; led < NUM_LEDS; led++) {  // loop through each of the 100 leds
    uint32_t color = colors[led];

    // extract GRB colors
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t b = (color >> 0) & 0xFF;

    uint8_t grb[3] = { g, r, b };

    int i=0;
    uint32_t item = 0;
    for (int byte = 0; byte < 3; byte++) {
      for (int bit = 7; bit >= 0; bit--) {
        uint8_t bitval = (grb[byte] >> bit) & 1;  // extract each bit from each rgb byte
        if (bitval) {
          item = ((uint32_t)(T1H & 0x7FFF)) | (1 << 15) | ((uint32_t)(T1L & 0x7FFF) << 16);  //rmt_data(T1H, T1L);
        } else {
          item = ((uint32_t)(T0H & 0x7FFF)) | (1 << 15) | ((uint32_t)(T0L & 0x7FFF) << 16);  // rmt_item(T0H, T0L);
        }
         
        REG_WRITE(RMT_RAM + (i * 4), item);  // write to RMT_RAM
        i++;
      }
    }
    REG_WRITE(RMT_RAM + (24 * 4), 0); // write 0 on 25th to stop
    
    REG_WRITE(RMT_INT_CLR_REG, 1 << 0);  // set to 0 
    // start transmit
    REG_SET_BIT(RMT_CH0_CONF0_REG, 1 << 24);  // set CONF_UPDATE (bit 24) to 1 in the RMT_CH0_CONF0_REG register,
    REG_SET_BIT(RMT_CH0_CONF0_REG, 1 << 0);   // set RMT_TX_START_CH0 (bit 0) to 1 in the RMT_CH0_CONF0_REG register.
    //Serial.println("writing RMT items done");


    // read the RMT_INT_RAW_REG register, check if the RMT_CH0_TX_END_INT_RAW bit is set
    while ((REG_READ(RMT_INT_RAW_REG) & (1 << 0)) == 0) {
      //Serial.println("waiting for RMT_INT_RAW_REG");
      delayMicroseconds(50); 
    }
    //Serial.println("done waiting for RMT");
    // if yes, set the RMT_TX_END_INT bit in the RMT_INT_CLR_REG register (suggest you also do this when initializing the RMT)
    REG_SET_BIT(RMT_INT_CLR_REG, (1 << 0));
    //REG_WRITE(RMT_INT_CLR_REG, 1 << 0);
  }
 
  delayMicroseconds(50);  // delay 50,000 nanoseconds
}



void loop() {
  static uint32_t initial = 1;
  uint32_t colors[NUM_LEDS];

  uint8_t sample = sampleArray[sample_index];


  // send audio sample to PWM
 // Serial.println("before update_PWM");
  update_PWM(initial, sample);
//  Serial.println("after update_PWM");
  initial = 0;  // set initial to 0 after first sample is sent


  // uint8_t mag = (sample > 127) ? (sample - 127) : (127 - sample);
  // double log_val = log10((double)mag + 1.0) / log10(128.0);
  // int num_LEDs_to_light = (int)round(log_val * 100.0);
  // if (num_LEDs_to_light < 0) num_LEDs_to_light = 0;
  // if (num_LEDs_to_light > NUM_LEDS) num_LEDs_to_light = NUM_LEDS;

  // for (int led = 0; led < NUM_LEDS; led++) {
  //     if ((uint32_t)led < num_LEDs_to_light) {
  //         // Color based on magnitude 
  //         uint8_t r, g, b;
  //         if (mag <= 51) {  // blue to cyan
  //             r = 0;
  //             g = mag * 5;
  //             b = 255;
  //         } else if (mag <= 102) {  // cyan to green
  //             r = 0;
  //             g = 255;
  //             b = 255 - (mag - 51) * 5;
  //         } else if (mag <= 153) {  // green to yellow
  //             r = (mag - 102) * 5;
  //             g = 255;
  //             b = 0;
  //         } else if (mag <= 204) {  // yellow to orange/red
  //             r = 255;
  //             g = 255 - (mag - 153) * 5;
  //             b = 0;
  //         } else {  // bright red
  //             r = 255;
  //             g = 0;
  //             b = 0;
  //         }

  //         colors[led] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  //     } else {
  //         colors[led] = 0x000000;  // if we've lit all the leds we should, set the rest to 0
  //     }
  // }

  // // Send LED colors
  // Serial.println("before transmit_led_signal");
  // transmit_led_signal(colors);
  // Serial.println("after transmit_led_signal");

 
  if (sample_index >= SAMPLE_LENGTH) {
    sample_index = 0;  // set sample_index back to 0 to repeat the song
  }
  //delayMicroseconds(50);
}
