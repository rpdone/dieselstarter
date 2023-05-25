/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* --- PRINTF_BYTE_TO_BINARY macro's --- */
#define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8(i)    \
    (((i) & 0x80ll) ? '1' : '0'), \
    (((i) & 0x40ll) ? '1' : '0'), \
    (((i) & 0x20ll) ? '1' : '0'), \
    (((i) & 0x10ll) ? '1' : '0'), \
    (((i) & 0x08ll) ? '1' : '0'), \
    (((i) & 0x04ll) ? '1' : '0'), \
    (((i) & 0x02ll) ? '1' : '0'), \
    (((i) & 0x01ll) ? '1' : '0')

#define PRINTF_BINARY_PATTERN_INT16 \
    PRINTF_BINARY_PATTERN_INT8              PRINTF_BINARY_PATTERN_INT8
#define PRINTF_BYTE_TO_BINARY_INT16(i) \
    PRINTF_BYTE_TO_BINARY_INT8((i) >> 8),   PRINTF_BYTE_TO_BINARY_INT8(i)
#define PRINTF_BINARY_PATTERN_INT32 \
    PRINTF_BINARY_PATTERN_INT16             PRINTF_BINARY_PATTERN_INT16
#define PRINTF_BYTE_TO_BINARY_INT32(i) \
    PRINTF_BYTE_TO_BINARY_INT16((i) >> 16), PRINTF_BYTE_TO_BINARY_INT16(i)
#define PRINTF_BINARY_PATTERN_INT64    \
    PRINTF_BINARY_PATTERN_INT32             PRINTF_BINARY_PATTERN_INT32
#define PRINTF_BYTE_TO_BINARY_INT64(i) \
    PRINTF_BYTE_TO_BINARY_INT32((i) >> 32), PRINTF_BYTE_TO_BINARY_INT32(i)
/* --- end macros --- */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "sdkconfig.h"
#include "driver/ledc.h"                                                  // Biblioteca ESP32 LEDC
#include "driver/pcnt.h"                                                  // Biblioteca ESP32 PCNT
#include "soc/pcnt_struct.h"
#include  <math.h>
#include <string.h>
//#include "OLED.c"
#include "cal_adc.c"
#include "demo.c"





//Configuracion Frecuencimetro
#define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 // Set Pulse Counter Unit - 0 
#define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                              // Set Pulse Counter channel - 0 

#define PCNT_INPUT_SIG_IO     GPIO_NUM_34                                 // Set Pulse Counter input - Freq Meter Input GPIO 34
#define LEDC_HS_CH0_GPIO      GPIO_NUM_33                                 // Saida do LEDC - gerador de pulsos - GPIO_33
#define PCNT_INPUT_CTRL_IO    GPIO_NUM_35                                 // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down  
#define OUTPUT_CONTROL_GPIO   GPIO_NUM_32                                 // Timer output control port - GPIO_32
#define PCNT_H_LIM_VAL        overflow                                    // Overflow of Pulse Counter 

#define IN_BOARD_LED          GPIO_NUM_2                                  // ESP32 native LED - GPIO 2

bool            flag          = true;                                     // Flag to enable print frequency reading
uint32_t        overflow      = 20000;                                    // Max Pulse Counter value
int16_t         pulses        = 0;                                        // Pulse Counter value
uint32_t        multPulses    = 0;                                        // Quantidade de overflows do contador PCNT
uint32_t        sample_time   = 100000;                                  // sample time of 1 second to count pulses
uint32_t        osc_freq      = 12543;                                    // Oscillator frequency - initial 12543 Hz (may be 1 Hz to 40 MHz)
uint32_t        mDuty         = 0;                                        // Duty value
uint32_t        resolution    = 0;                                        // Resolution value
float           frequency     = 0;                                        // frequency value
char            buf[32];                                                  // Buffer
char            * informe     = "Arrancador Diesel";
esp_timer_create_args_t create_args;                                      // Create an esp_timer instance
esp_timer_handle_t timer_handle;                                          // Create an single timer

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;                     // portMUX_TYPE to do synchronism

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/

#define STARTER_PIN           GPIO_NUM_12
#define IG_PIN                GPIO_NUM_27
#define JOCKEY_PIN            GPIO_NUM_14
#define PRESSURE_PIN          ADC2_CHANNEL_9_GPIO_NUM
#define PRESSURE_CH           ADC2_CHANNEL_9
#define HISTERESIS_PIN        ADC2_CHANNEL_8_GPIO_NUM
#define HISTERESIS_CH         ADC2_CHANNEL_8
#define JOCKEY_PIN_ADC        ADC1_CHANNEL_3_GPIO_NUM
#define JOCKEY_CH             ADC1_CHANNEL_3
#define DIESEL_PIN            ADC1_CHANNEL_0_GPIO_NUM
#define DIESEL_CH             ADC1_CHANNEL_0
#define TRANSDUCER_MAX_PSI    580
#define EXTERNAL_PIN          GPIO_NUM_4
int encendido =0 ; 
int pressure_jockey = 0;
int pressure_diesel = 0;
int deviation = 0;
uint32_t outputstate = 0;
uint32_t pinstat = 0;
int crank_tries = 0;
  SSD1306_t dev;
	int center, top, bottom;
	char lineChar[20];

void init_hw();
int act_pressure(float psp);
float check_rpm();
void init_frequencyMeter ();
void read_PCNT(void *p);
int des_pressure(float psp);
void print_data(char *texto,int page_oled);
void set_pressure_points();


void app_main()
{  

    init_hw();

        
    while(1) {
      //oled_printf("Hola");
      printf("Inicio \n");
      
      printf("Voltaje: %d \n",read_adc());

        //print_data((((read_adc()-744) * TRANSDUCER_MAX_PSI)/3351),0);

      if (act_pressure(45))
      {
        gpio_set_level(JOCKEY_PIN,1);
        print_data("Activar Jockey",0);
              

      } 

      if(des_pressure(65)){
                gpio_set_level(JOCKEY_PIN,0);
                print_data("No Jockey",0);
      }
      
     if (encendido==0) {    
        if (act_pressure(30)){
           
            while(des_pressure(3) == 0 && crank_tries<5){

                 gpio_set_level(STARTER_PIN,1);
                 gpio_set_level(IG_PIN,1);
                 for (size_t i = 0; i < 7 ; i++)
                 {
                  vTaskDelay(800/portTICK_PERIOD_MS);
                 }
                                
                 
                 crank_tries++;

                  gpio_set_level(STARTER_PIN,0);
                   for (size_t i = 0; i < 3 ; i++)
                 {
                  vTaskDelay(800/portTICK_PERIOD_MS);
                 }
                
            }
            if (crank_tries==5){
               gpio_set_level(STARTER_PIN,0);
               gpio_set_level(IG_PIN,0);

               while (gpio_get_level(EXTERNAL_PIN)==1){
                print_data("falla motor",0);
                               }
               if (des_pressure(3)==1)
               {
                                  gpio_set_level(STARTER_PIN,0);
                                  gpio_set_level(IG_PIN,1);
                                  print_data("Motor Encendido",0);
                                  printf("encendio");
                                  encendido=1;

               }
               
               crank_tries=0;

            }
        }
        
         vTaskDelay(900/portTICK_PERIOD_MS);
      }}
}

void init_hw() {
    gpio_config_t pines ;
      pines.pin_bit_mask = GPIO_SEL_14 | GPIO_SEL_12 | GPIO_SEL_27;
      pines.mode = GPIO_MODE_OUTPUT;
      pines.pull_up_en = GPIO_PULLUP_DISABLE;
      pines.pull_down_en = GPIO_PULLDOWN_DISABLE;
      pines.intr_type = GPIO_PIN_INTR_DISABLE;
      gpio_config(&pines);
  gpio_config_t entradas ;     
 entradas.pin_bit_mask = GPIO_SEL_34;
 entradas.mode = GPIO_MODE_INPUT;
 entradas.pull_down_en = GPIO_PULLDOWN_ENABLE;
 entradas.pull_up_en = GPIO_PULLUP_DISABLE;
 entradas.intr_type = GPIO_PIN_INTR_DISABLE;
 gpio_config(&entradas);
 PIN_FUNC_SELECT(IO_MUX_GPIO14_REG,PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(IO_MUX_GPIO12_REG,PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(IO_MUX_GPIO27_REG,PIN_FUNC_GPIO); 
  gpio_pad_select_gpio(IG_PIN);
  gpio_set_direction(IG_PIN, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(STARTER_PIN);
  gpio_set_direction(STARTER_PIN, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(JOCKEY_PIN);
  gpio_set_direction(JOCKEY_PIN, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(EXTERNAL_PIN);
  gpio_set_direction(EXTERNAL_PIN,GPIO_MODE_INPUT);
  
  gpio_set_level(IG_PIN,1);
  gpio_set_level(STARTER_PIN,0);
  gpio_set_level(JOCKEY_PIN,0);


  i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
  ssd1306_init(&dev, 128, 32);
  ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);

  ssd1306_display_text(&dev, 0, informe, strlen(informe), false);
  vTaskDelay(3000 / portTICK_PERIOD_MS);


    //adc2_config_channel_atten(PRESSURE_CH,ADC_ATTEN_11db);
    //init_frequencyMeter ();
   // test_oled();
    //olednit();
   // calibrate_adc();
  //demo();

}

int act_pressure(float psp)
{
    int result = 0;
    float pressureCalc = 0; 
    int conversionVal = 0 ;



      pressureCalc = pressureCalc + (((read_adc()-935) * TRANSDUCER_MAX_PSI)/(4096-935)) ;
     //vTaskDelay(200/portTICK_PERIOD_MS);
      
    
    
        printf("presion: %f \n",pressureCalc);
      if (pressureCalc<= psp ) {

        result = 1;


     }else { 
        result= 0;
     }
    
     


     
     
   return result ;  
     
}

int des_pressure(float psp)
{
    int result = 0;
    float pressureCalc = 0; 
    int conversionVal = 0 ;
        pressureCalc = pressureCalc + (((read_adc()-935) * TRANSDUCER_MAX_PSI)/(4096-935)) ;
    // vTaskDelay(200/portTICK_PERIOD_MS);
    printf("presion: %f \n",pressureCalc);

      if (pressureCalc>= psp ) {

        result = 1;


     }else { 
        result= 0;
     } 
   return result ;  
     
}
void init_osc_freq ()                                                     // Initialize Oscillator to test Freq Meter
{
  resolution = (log (80000000 / osc_freq)  / log(2)) / 2 ;                // Calc of resolution of Oscillator
  if (resolution < 1) resolution = 1;                                     // set min resolution 
  // Serial.println(resolution);                                          // Print
  mDuty = (pow(2, resolution)) / 2;                                       // Calc of Duty Cycle 50% of the pulse
  // Serial.println(mDuty);                                               // Print

  ledc_timer_config_t ledc_timer = {};                                    // LEDC timer config instance

  ledc_timer.duty_resolution = resolution;            // Set resolution
  ledc_timer.freq_hz    = osc_freq;                                       // Set Oscillator frequency
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                           // Set high speed mode
  ledc_timer.timer_num = LEDC_TIMER_0;                                    // Set LEDC timer index - 0
  ledc_timer_config(&ledc_timer);                                         // Set LEDC Timer config

  ledc_channel_config_t ledc_channel = {};                                // LEDC Channel config instance

  ledc_channel.channel    = LEDC_CHANNEL_0;                               // Set HS Channel - 0
  ledc_channel.duty       = mDuty;                                        // Set Duty Cycle 50%
  ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;                             // LEDC Oscillator output GPIO 33
  ledc_channel.intr_type  = LEDC_INTR_DISABLE;                            // LEDC Fade interrupt disable
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;                         // Set LEDC high speed mode
  ledc_channel.timer_sel  = LEDC_TIMER_0;                                 // Set timer source of channel - 0
  ledc_channel_config(&ledc_channel);                                     // Config LEDC channel
}

//----------------------------------------------------------------------------------
static void IRAM_ATTR pcnt_intr_handler(void *arg)                        // Counting overflow pulses
{
  portENTER_CRITICAL_ISR(&timerMux);                                      // disabling the interrupts
  multPulses++;                                                           // increment Overflow counter
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT);                                // Clear Pulse Counter interrupt bit
  portEXIT_CRITICAL_ISR(&timerMux);                                       // enabling the interrupts
}

//----------------------------------------------------------------------------------
void init_PCNT(void)                                                      // Initialize and run PCNT unit
{
  pcnt_config_t pcnt_config = { };                                        // PCNT unit instance

  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;                         // Pulse input GPIO 34 - Freq Meter Input
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;                         // Control signal input GPIO 35
  pcnt_config.unit = PCNT_COUNT_UNIT;                                     // Unidade de contagem PCNT - 0
  pcnt_config.channel = PCNT_COUNT_CHANNEL;                               // PCNT unit number - 0
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;                             // Maximum counter value - 20000
  pcnt_config.pos_mode = PCNT_COUNT_INC;                                  // PCNT positive edge count mode - inc
  pcnt_config.neg_mode = PCNT_COUNT_INC;                                  // PCNT negative edge count mode - inc
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;                             // PCNT low control mode - disable
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                                // PCNT high control mode - won't change counter mode
  pcnt_unit_config(&pcnt_config);                                         // Initialize PCNT unit

  pcnt_counter_pause(PCNT_COUNT_UNIT);                                    // Pause PCNT unit
  pcnt_counter_clear(PCNT_COUNT_UNIT);                                    // Clear PCNT unit

  pcnt_event_enable(PCNT_COUNT_UNIT, PCNT_EVT_H_LIM);                     // Enable event to watch - max count
  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);                    // Setup Register ISR handler
  pcnt_intr_enable(PCNT_COUNT_UNIT);                                      // Enable interrupts for PCNT unit

  pcnt_counter_resume(PCNT_COUNT_UNIT);                                   // Resume PCNT unit - starts count
}

//----------------------------------------------------------------------------------
void read_PCNT(void *p)                                                   // Read Pulse Counter
{
  gpio_set_level(OUTPUT_CONTROL_GPIO, 0);                                 // Stop counter - output control LOW
  pcnt_get_counter_value(PCNT_COUNT_UNIT, &pulses);                       // Read Pulse Counter value
  flag = true;                                                            // Change flag to enable print
}

//---------------------------------------------------------------------------------
void init_frequencyMeter ()
{
  init_osc_freq();                                                        // Initialize Oscillator
  init_PCNT();                                                            // Initialize and run PCNT unit

  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                              // Set GPIO pad
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT);              // Set GPIO 32 as output

  create_args.callback = read_PCNT;                                       // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);                          // Create esp-timer instance

  gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT);                     // Set LED inboard as output

  gpio_matrix_in(PCNT_INPUT_SIG_IO, SIG_IN_FUNC226_IDX, false);           // Set GPIO matrin IN - Freq Meter input
  gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false);        // Set GPIO matrix OUT - to inboard LED
}

//----------------------------------------------------------------------------------------
char *ultos_recursive(unsigned long val, char *s, unsigned radix, int pos) // Format an unsigned long (32 bits) into a string
{
  int c;
  if (val >= radix)
    s = ultos_recursive(val / radix, s, radix, pos + 1);
  c = val % radix;
  c += (c < 10 ? '0' : 'a' - 10);
  *s++ = c;
  if (pos % 3 == 0) *s++ = ',';
  return s;
}
//----------------------------------------------------------------------------------------
char *ltos(long val, char *s, int radix)                                  // Format an long (32 bits) into a string
{
  if (radix < 2 || radix > 36) {
    s[0] = 0;
  } else {
    char *p = s;
    if (radix == 10 && val < 0) {
      val = -val;
      *p++ = '-';
    }
    p = ultos_recursive(val, p, radix, 0) - 1;
    *p = 0;
  }
  return s;
}



float check_rpm(){

    /* code */
    if (flag == true)                                                     // If count has ended
  {
    flag = false;                                                       // change flag to disable print
    frequency = (pulses + (multPulses * overflow)) / 2  ;
                // Calculation of frequency
   

    multPulses = 0;                                                     // Clear overflow counter
    // Put your function here, if you want
                                                     // Delay 100 ms
    // Put your function here, if you want

    pcnt_counter_clear(PCNT_COUNT_UNIT);                                // Clear Pulse Counter
    esp_timer_start_once(timer_handle, sample_time);                    // Initialize High resolution timer (1 sec)
    gpio_set_level(OUTPUT_CONTROL_GPIO, 1);                             // Set enable PCNT count
  }
  
  
   printf("Pulsos : %s", ltos(frequency, buf, 10));               // Print frequency with commas
    printf(" Hz \n");                                                   // Print unity Hz


  return frequency ;
}


void print_data(char *texto,int page_oled) {
   ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);

  ssd1306_display_text(&dev, page_oled, texto, strlen(texto), false);


} 

void set_pressure_points(){

     if (pressure_jockey)
     {
      /* code */
     }
     
      



}