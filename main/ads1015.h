/*
 * ads1015.h
 *
 * Created on: 25 дек. 2022 г.
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#ifndef MAIN_ADS1015_H_
#define MAIN_ADS1015_H_

#define ADS111X_ADDR_GND 0x48 //!< I2C device address with ADDR pin connected to ground
#define ADS111X_ADDR_VCC 0x49 //!< I2C device address with ADDR pin connected to VCC
#define ADS111X_ADDR_SDA 0x4a //!< I2C device address with ADDR pin connected to SDA
#define ADS111X_ADDR_SCL 0x4b //!< I2C device address with ADDR pin connected to SCL

#define ADS_I2CADDR     ADS111X_ADDR_GND //!< I2C device address with ADDR pin connected to ground
//#define ADS_I2CADDR     ADS111X_ADDR_VCC //!< I2C device address with ADDR pin connected to VCC
//#define ADS_I2CADDR     ADS111X_ADDR_SDA //!< I2C device address with ADDR pin connected to SDA
//#define ADS_I2CADDR     ADS111X_ADDR_SCL  //!< I2C device address with ADDR pin connected to SCL

#define ADC_CHANNEL_COUNT	4
#define ADC_HIST_LENGTH	32

typedef struct {
	int16_t 		last_val;
	esp_err_t 	err;
	int16_t diff;
	TickType_t last_change_time;
	int16_t val[ADC_HIST_LENGTH];
} adc_t;

extern adc_t adc[ADC_CHANNEL_COUNT];

#define IS_ADS1015_PRESENT (I2C_detect[ADS_I2CADDR])
#define ADC_READ_PERIOD_MS	2000

esp_err_t readADS(uint16_t channel, uint16_t cnt, int16_t* value);
void start_adc_task(void);

#endif /* MAIN_ADS1015_H_ */
