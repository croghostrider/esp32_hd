/*
 * ads1015.c
 *
 *  Created on: 25 дек. 2022 г.
 *      Author: Vladimir Loskutnikov
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "config.h"
#include "debug.h"
#include "hd_spi_i2c.h"
#include "ads1015.h"

#define ADS1115_REG_CONFIG_OS     					(0x8000)

#define ADS1115_REG_CONFIG_MUX_SINGLE_0   (0x4000)
#define ADS1115_REG_CONFIG_MUX_SINGLE_1   (0x5000)
#define ADS1115_REG_CONFIG_MUX_SINGLE_2   (0x6000)
#define ADS1115_REG_CONFIG_MUX_SINGLE_3   (0x7000)

#define ADS1115_REG_CONFIG_PGA_3mV			((uint16_t)0<<9)
#define ADS1115_REG_CONFIG_PGA_2mV			((uint16_t)1<<9)
#define ADS1115_REG_CONFIG_PGA_1mV			((uint16_t)2<<9) //default
#define ADS1115_REG_CONFIG_PGA_500uV			((uint16_t)3<<9)
#define ADS1115_REG_CONFIG_PGA_250uV			((uint16_t)4<<9)
#define ADS1115_REG_CONFIG_PGA_125uV			((uint16_t)5<<9)

#define ADS1115_REG_CONFIG_SINGE_MODE		(0x0100)

#define ADS1115_REG_CONFIG_RATE_128			((uint16_t)0<<5)
#define ADS1115_REG_CONFIG_RATE_250			((uint16_t)1<<5)
#define ADS1115_REG_CONFIG_RATE_490			((uint16_t)2<<5)
#define ADS1115_REG_CONFIG_RATE_920			((uint16_t)3<<5)
#define ADS1115_REG_CONFIG_RATE_1600			((uint16_t)4<<5)
#define ADS1115_REG_CONFIG_RATE_2400			((uint16_t)5<<5)
#define ADS1115_REG_CONFIG_RATE_3300			((uint16_t)6<<5)

#define ADS1115_REG_POINTER_CONVERT         (0x00)
#define ADS1115_REG_POINTER_CONFIG          	(0x01)


adc_t adc[ADC_CHANNEL_COUNT];

esp_err_t read_reg(uint8_t i2c_address, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    esp_err_t res;
    if ((res = i2c_read(i2c_address, &reg, 1, buf, 2)) != ESP_OK)
    {
        ESP_LOGE(__func__, "Could not read from register 0x%02x, err:%d", reg, res);
        return res;
    }
    *val = (buf[0] << 8) | buf[1];
    return ESP_OK;
}

esp_err_t write_reg(uint8_t i2c_address, uint8_t reg, uint16_t val){
    uint8_t buf[2] = { val >> 8, val };
    esp_err_t res;
    if ((res = i2c_write(i2c_address, &reg, 1, buf, 2)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not write 0x%04x to register 0x%02x", val, reg);
        return res;
    }
    return ESP_OK;
}

esp_err_t readADS(uint16_t channel, uint16_t cnt, int16_t* value) {

  if (channel > 3 || cnt == 0 || value==NULL) return 0;

  esp_err_t  ret=0;
  uint16_t config =	ADS1115_REG_CONFIG_OS |
		  	  	  	  	  	ADS1115_REG_CONFIG_MUX_SINGLE_0 | (channel<<12) |
							ADS1115_REG_CONFIG_PGA_1mV |
							ADS1115_REG_CONFIG_SINGE_MODE |
							ADS1115_REG_CONFIG_RATE_2400 |
							0x03; // Disable comparator
  int32_t summ = 0;
  uint16_t val=0;
  for (uint8_t i=0; i<cnt; i++) {
	  ret = write_reg(ADS_I2CADDR, ADS1115_REG_POINTER_CONFIG, config); // Write config register to the ADC
	  if (ret != ESP_OK) return ret;
	  vTaskDelay( 2 / portTICK_PERIOD_MS);
	  ret = read_reg(ADS_I2CADDR, ADS1115_REG_POINTER_CONVERT,&val);
	  if (ret != ESP_OK) return ret;
	  /*
	  if (IT_IS_ADS1015) {
		  val = val >> 4;
		  if (val > 0x07FF)  {
			  // negative number - extend the sign to 16th bit
			  val |= 0xF000;
		  }
	  }
	  */
	  summ +=(int16_t)val;
  }

  *value = (int16_t)((summ + cnt/2)/cnt);
  return ret;
}

#define ADC_HIST_PERIOD_TICKS  (10000UL / portTICK_PERIOD_MS)

void put_adc(int channel, int16_t adc_value){
	adc_t*  ch_ptr = &adc[channel];
	ch_ptr->last_val=adc_value;

	DBG("ch:%d v:%d",channel,	adc_value);
	// не пришло ли время поместить в историю значений
	if ((xTaskGetTickCount ()-ch_ptr->last_change_time) < ADC_HIST_PERIOD_TICKS) {
		return;
	}
	ch_ptr->val[0] = ch_ptr->last_val;
	// сдвиг массива отсчетов
	for(int n=ADC_HIST_LENGTH; n>0; n--) {
		ch_ptr->val[n] = ch_ptr->val[n-1];
		//ch_ptr->filtered[n] = ch_ptr->filtered[n-1];
	}
	ch_ptr->val[0]=adc_value; //запись последнего отсчета в начало массива
	ch_ptr->diff = ch_ptr->val[0] - ch_ptr->val[ADC_HIST_LENGTH-1];
	//ch_ptr->filtered[0] =get_filtered(ch_ptr);

	if (ch_ptr->diff){
		ch_ptr->last_change_time = xTaskGetTickCount ();// системное время в тиках
	}
}

void adc_task(void *arg)
{
	int channel;
	adc_t*  ch_ptr;
	int16_t adc_value;
	while(1) {
		for (channel=0;channel<ADC_CHANNEL_COUNT;channel++){
			ch_ptr = &adc[channel];
			ch_ptr->err = readADS(channel, 8, &adc_value);
			if (ch_ptr->err)
				ESP_LOGE(__func__,"ch:%d err:%d",channel, ch_ptr->err);
			else {
				put_adc(channel, adc_value);
			}
		} // channels
		vTaskDelay(ADC_READ_PERIOD_MS/portTICK_PERIOD_MS);
	}
}

void start_adc_task(void){
	if (!I2C_detect[ADS_I2CADDR]) return;
	/* опрос каналов ADC */
	xTaskCreate(&adc_task, "adc_task", 4096, NULL, 1, NULL);
}
