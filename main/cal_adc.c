#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        // Use 1100 mV as the default reference voltage
#define NO_OF_SAMPLES   300          // Number of samples to average
#define TRANSDUCER_MAX_PSI 232

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC2_CHANNEL_9;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_2;


int read_adc1(adc1_channel_t channel_to_use,adc_atten_t atten_to_use,adc_bits_width_t width_of_conversion);
void calibrate_adc()
{
    // Configure ADC
    //adc2_config_width(ADC_WIDTH_BIT_12);
    
    adc2_config_channel_atten(channel, atten);

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    // Check characterization results
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("ADC characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("ADC characterized using eFuse Vref\n");
    } else {
        printf("ADC characterized using Default Vref\n");
    }
}

int read_adc()
{
            
     adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_11db);
    // Read ADC value
    uint32_t acumulador = 0; 
    int adc_reading = 0;
    int resultado = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {

         esp_err_t r =  adc2_get_raw(ADC2_CHANNEL_9,ADC_WIDTH_12Bit,&adc_reading);
    if ( r == ESP_OK ) {
        //printf("muestra %d de  valor %d \n",i,adc_reading);
       acumulador = adc_reading + acumulador;
    } else if ( r == ESP_ERR_TIMEOUT ) {
        printf("ADC2 used by Wi-Fi.\n");
       
    }}
    resultado = acumulador / NO_OF_SAMPLES;

   return resultado;
}

int read_adc1(adc1_channel_t channel_to_use,adc_atten_t atten_to_use,adc_bits_width_t width_of_conversion)

{
    adc1_config_width(width_of_conversion);
     adc1_config_channel_atten(channel_to_use,atten_to_use);       
     
    // Read ADC value
    uint32_t acumulador = 0; 
    int adc_reading = 0;
    int resultado = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {

         adc_reading =  adc1_get_raw(channel_to_use);

       acumulador = adc_reading + acumulador;
   
    
       
    }
    resultado = acumulador / NO_OF_SAMPLES;

   return resultado;
}


int conversion_to_psi(adc1_channel_t channel_pot) {


    int conversion =  (((read_adc1(channel_pot,ADC_ATTEN_DB_11,ADC_WIDTH_12Bit)) * TRANSDUCER_MAX_PSI)/4095) ;
    return conversion;

}