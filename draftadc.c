#define FSR0_GPIO_NUM 36
#define FSR1_GPIO_NUM 39
#define FSR2_GPIO_NUM 34
#define FSR3_GPIO_NUM 35
#define FSR4_GPIO_NUM 32
#define FSR5_GPIO_NUM 33
#define NUM_FSRS 6

// todo calculate fsr adc
#define FSR0_ADC_Vmax 1
#define FSR1_ADC_Vmax 1
#define FSR2_ADC_Vmax 1
#define FSR3_ADC_Vmax 1
#define FSR4_ADC_Vmax 1
#define FSR5_ADC_Vmax 1

// todo set this from adc vmax
#define FSR0_ADC_attenuation 1
#define FSR1_ADC_attenuation 1
#define FSR2_ADC_attenuation 1
#define FSR3_ADC_attenuation 1
#define FSR4_ADC_attenuation 1
#define FSR5_ADC_attenuation 1

#define FSR0_ADC_bitwidth 1
#define FSR1_ADC_bitwidth 1
#define FSR2_ADC_bitwidth 1
#define FSR3_ADC_bitwidth 1
#define FSR4_ADC_bitwidth 1
#define FSR5_ADC_bitwidth 1

#define FSR0_ADC_Dmax (1 << FSR0_ADC_bitwidth)
#define FSR1_ADC_Dmax (1 << FSR1_ADC_bitwidth)
#define FSR2_ADC_Dmax (1 << FSR2_ADC_bitwidth)
#define FSR3_ADC_Dmax (1 << FSR3_ADC_bitwidth)
#define FSR4_ADC_Dmax (1 << FSR4_ADC_bitwidth)
#define FSR5_ADC_Dmax (1 << FSR5_ADC_bitwidth)

#define FSRS_sample_frequency 100

adc_continous_handle_cfg_t adc_config = {
    .max_store_buf_size = 1024,
    .conv_frame_size = 100,
    .flags = flush_pool,
};

adc_continuous_handle_t adc_handle;

//initialize ADC continous mode driver
ESP_ERROR_CHECK(adc_continous_new_handle(adc_handle, &adc_config));

adc_unit_t FSR0_unit;
adc_unit_t FSR1_unit;
adc_unit_t FSR2_unit;
adc_unit_t FSR3_unit;
adc_unit_t FSR4_unit;
adc_unit_t FSR5_unit;

adc_channel_t FSR0_channel;
adc_channel_t FSR1_channel;
adc_channel_t FSR2_channel;
adc_channel_t FSR3_channel;
adc_channel_t FSR4_channel;
adc_channel_t FSR5_channel;

adc_continuous_io_to_channel((int)FSR0_GPIO_NUM, &FSR0_unit, &FSR0_channel);
adc_continuous_io_to_channel((int)FSR1_GPIO_NUM, &FSR1_unit, &FSR1_channel);
adc_continuous_io_to_channel((int)FSR2_GPIO_NUM, &FSR2_unit, &FSR2_channel);
adc_continuous_io_to_channel((int)FSR3_GPIO_NUM, &FSR3_unit, &FSR3_channel);
adc_continuous_io_to_channel((int)FSR4_GPIO_NUM, &FSR4_unit, &FSR4_channel);
adc_continuous_io_to_channel((int)FSR5_GPIO_NUM, &FSR5_unit, &FSR5_channel);

adc_digi_pattern_config_t *FSR_adc_patterns[NUM_FSRS];
FSR_adc_patterns[0] = {
    .atten = (uint8_t)FSR0_ADC_attenuation,
    .channel = (uint8_t)FSR0_channel,
    .unit = (uint8_t)FSR0_unit,
    .bit_width = (uint8_t)FSR0_ADC_bitwidth,
};

FSR_adc_patterns[1] = {
    .atten = (uint8_t)FSR1_ADC_attenuation,
    .channel = (uint8_t)FSR1_channel,
    .unit = (uint8_t)FSR1_unit,
    .bit_width = (uint8_t)FSR1_ADC_bitwidth,
};

FSR_adc_patterns[2] = {
    .atten = (uint8_t)FSR2_ADC_attenuation,
    .channel = (uint8_t)FSR2_channel,
    .unit = (uint8_t)FSR2_unit,
    .bit_width = (uint8_t)FSR2_ADC_bitwidth,
};

FSR_adc_patterns[3] = {
    .atten = (uint8_t)FSR3_ADC_attenuation,
    .channel = (uint8_t)FSR3_channel,
    .unit = (uint8_t)FSR3_unit,
    .bit_width = (uint8_t)FSR3_ADC_bitwidth,
};

FSR_adc_patterns[4] = {
    .atten = (uint8_t)FSR4_ADC_attenuation,
    .channel = (uint8_t)FSR4_channel,
    .unit = (uint8_t)FSR4_unit,
    .bit_width = (uint8_t)FSR4_ADC_bitwidth,
};

FSR_adc_patterns[5] = {
    .atten = (uint8_t)FSR5_ADC_attenuation,
    .channel = (uint8_t)FSR5_channel,
    .unit = (uint8_t)FSR5_unit,
    .bit_width = (uint8_t)FSR5_ADC_bitwidth,
};

adc_continuous_config_t adc_cont_config = {
    .pattern_num = (uint32_t)NUM_FSRS,
    .adc_pattern = FSR_adc_patterns,
    .sample_freq_hz = (uint32_t)FSRS_sample_frequency,
    //only actively using ADC1 anyway so use this exclusively for conversion
    .conv_mode = ADC_CONV_SINGLE_UNIT 1,
    //CHECK THIS
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
};

ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_cont_config));

ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));

//deinit and recycle ADC continous mode driver
ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));

uint8_t *buf;
uint32_t length_max;
uint32_t *out_length;
uint32_t timeout_ms;
ESP_ERROR_CHECK(adc_continuous_read(adc_handle, 
                                    buf, 
                                    length_max, 
                                    out_length, 
                                    timeout_ms));

ESP_ERROR_CHECK(adc_continuous_flush_pool(adc_handle));

int *FSRs_Dout[NUM_FSRS];
for (int i = 0; i < NUM_FSRS; i++) {
    //this will not be quite correct check later
    FSRs_Dout[i] = buf
}
int FSR0_Dout
int FSR1_Dout
int FSR2_Dout
int FSR3_Dout
int FSR4_Dout
int FSR5_Dout

int FSR0_Vout = 
int FSR1_Vout
int FSR2_Vout
int FSR3_Vout
int FSR4_Vout
int FSR5_Vout

/****************************************
 * EVENT CALLBACKS
 * *****************************************
 */

adc_continuous_evt_cbs_t cbs = {
    .on_conv_done = ,
    .on_pool_ovf = ,
};
void *user_data;
//DO NOT CALL WHEN ADC CONT MODE DRIVER STARTED
ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, 
                                                        &cbs,
                                                        user_data));

/**
 * EVENT DATA STRUCT
 * struct adc_continous_evt_data_t {
 * uint8_t *conv_frame_buffer;
 * uint32_t size;
 * }
 */
