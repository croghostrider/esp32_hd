/*
esp32_hd
Copyright (c) 2018 ys1797 (yuri@rus.net)

License (MIT license):
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include "freertos/xtensa_api.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp32/rom/rtc.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include "esp_event.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "driver/ledc.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "soc/ledc_struct.h"
#include "math.h"
#include "time.h"
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>
#include <cJSON.h>
#include "lwip/apps/sntp.h"
#include "ds.h"
#include "hd_bmp180.h"
#include "esp_platform.h"
#include "config.h"
#include "hd_spi_i2c.h"
#include "hd_wifi.h"
#include "hd_main.h"
#include "cgiupdate.h"
#include "cgiwebsocket.h"
#include "esp_request.h"
#include "debug.h"


//#define GPIO_INPUT_PIN_SEL  (1<<GPIO_DETECT_ZERO) 
#define GPIO_OUTPUT_BEEPER  (1<<GPIO_BEEP)

volatile int32_t Hpoint = HMAX;
volatile int zero_imp_shift;


#define TIMER_DIVIDER   80		/*!< Hardware timer clock divider */
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (0*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */ 
#define TIMER_INTERVAL_SEC   (0.001)   /*!< test interval for timer */ 

char *Hostname;		// Имя хоста
char *httpUser;		// Имя пользователя для http
char *httpPassword;	// Пароль для http
int httpSecure;		// Спрашивать пароль
int wsPeriod=5;		// Период обновления данных через websocket

unsigned char ds1820_devices;                  // Количество датчиков ds18b20

// клапана
unsigned char klp_gpio[4] =  {26, 27, 32, 33};
klp_list Klp[MAX_KLP];		// Список клапанов.
xQueueHandle valve_cmd_queue; // очередь команд клапанов
void valveCMDtask(void *arg);
void cmd2valve (int valve_num, valve_cmd_t cmd);
void restoreProcess(void);

/* Время */
volatile uint32_t tic_counter;
volatile uint32_t uptime_counter;
volatile int gpio_counter = 0;
volatile uint32_t setDelay = 5;

/* Данные режима работы */
main_mode MainMode = MODE_IDLE;	// Текущий режим работы
int16_t MainStatus=START_WAIT;		// Текущее состояние (в зависимости от режима)
bool broken_proc = false;					// признак старта после прерванного процесса

alarm_mode AlarmMode = NO_ALARM;	// Состояние аварии
int16_t CurPower;			// Текущая измерянная мощность
int16_t SetPower;			// Установленная мощность
int16_t CurVolts;			// Текущее измеренное напряжение
volatile int16_t CurFreq;		// Измерянное число периодов в секунду питающего напряжения
int16_t WaterOn =-1;			// Флаг включения контура охлаждения
float TempWaterIn = -1;			// Температура воды на входе в контур
float TempWaterOut = -1; 		// Температура воды на выходе из контура
int16_t WaterFlow=-1;			// Значения датчика потока воды.
int16_t fAlarmSoundOff=0;

// Динамические параметры
double tempTube20Prev;		// Запомненое значение температуры в колонне
uint32_t secTempPrev;		// Отметка времени измеренной температуры
double  tempStabSR;		// Температура, относительно которой стабилизируется отбор СР
int16_t ProcChimSR = 0;		// Текущий процент отбора СР
double  startPressure = -1;	// Атмосферное давление в начале процесса
#define COUNT_PWM	15	// Размер таблицы автоподбора шим

typedef struct {
	double temp;
	int16_t pwm;
} autopwm;
autopwm autoPWM[COUNT_PWM] = {
	{0, -1},
	{88.0, 68},
	{96.0, 25},
	{100, 0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0}
};

#ifdef DEBUG
TickType_t xOpenTime=0,xCloseTime=0;
#endif


int16_t rect_timer1=0;		// Таймер для отсчета секунд 1
int16_t timer_sec2=0;		// Таймер для отсчета секунд 2
int16_t timer_sec3=0;		// Таймер для отсчета секунд 3
int32_t SecondsEnd;		// Время окончания процесса
RESET_REASON resetReason;	// Причина перезагрузки
nvs_handle nvsHandle;

static volatile int beeperTime=0;
static volatile bool beepActive = false;

// Включаем бипер
void IRAM_ATTR myBeep(bool lng)
{
	if (beepActive) return;

	if (lng) beeperTime = 1000;	
	else beeperTime = 500;
	beepActive = true;
	GPIO_ON(GPIO_BEEP);
}

void shortBeep(void)
{
	if (beepActive) return;
	beeperTime = 150;
	beepActive = true;
	GPIO_ON(GPIO_BEEP);
}

double roundX (double x, int precision)
{
   int mul = 10;
   
   for (int i = 0; i < precision; i++)
      mul *= mul;
   if (x > 0)
      return floor(x * mul + .5) / mul;
   else
      return ceil(x * mul - .5) / mul;
}

extern uint8_t PZEM_Version;	// Device version 3.0 in use ?
bool is_diffOffCondition(void);

void set_proc_power(char* param_name){
	int16_t pw;
	if (broken_proc){
		nvs_get_i16(nvsHandle, "SetPower", &pw); //мощность процесса на момент прерывания
		broken_proc=false;
	}
	else {
		pw = getIntParam(DEFL_PARAMS, param_name);  //  мощность процесса из настроек
	}
	setPower(pw); //мощность до выключения
}

void diffOffTask(void *arg){
	openKlp(klp_diff);
	vTaskDelay(5000/portTICK_PERIOD_MS);
	closeKlp(klp_diff);
	vTaskDelete(NULL);
}

void alarmControlTask(void *arg){
	int vDIFFoffDelaySec;
	bool offByoverPwr,offByT;

	while(1) {
		if (AlarmMode){	//EXISTS_ALARM(ALARM_FREQ | ALARM_NOLOAD | ALARM_PZEM_ERR | ALARM_TEMP | ALARM_SENSOR_ERR)
			if (!fAlarmSoundOff)	myBeep(false);
		}
		offByoverPwr = getIntParam(DEFL_PARAMS, "alarmDIFFoffP") ;
		offByT= getIntParam(DEFL_PARAMS, "alarmDIFFoffT");
		vDIFFoffDelaySec = getIntParam(DEFL_PARAMS, "DIFFoffDelay");

		if ((EXISTS_ALARM(ALARM_OVER_POWER) && offByoverPwr) ||  (EXISTS_ALARM(ALARM_TEMP) &&  offByT))
		{
			ERR_MSG("start DIFF-OFF proc. Delay (%d sec). Alarm:%d",vDIFFoffDelaySec,AlarmMode);
			for (int i=0;i<vDIFFoffDelaySec;i++){// задержка выключения диф-автомата
				shortBeep();
				vTaskDelay(SEC_TO_TICKS(1));
			}
			if (((EXISTS_ALARM(ALARM_OVER_POWER) && offByoverPwr) ||  (EXISTS_ALARM(ALARM_TEMP) &&  offByT)))
			{ // если ситуация не исправилась
				ERR_MSG("DIFF turned off");
				openKlp(klp_diff); 											// подаем напряжение (клапан 4)
				vTaskDelay(SEC_TO_TICKS(5));
				closeKlp(klp_diff);											// через 5 сек - снимаем напряжение с клапана
			}
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

error_t readPZEM(int16_t* currV,int16_t* currP){
	// --------reading PZEM-----------
	if (PZEM_Version)  {
		if (!PZEMv30_updateValues()){
			*currV = -1;
			*currP = -1;
			return ESP_FAIL;
		}
	}
	*currV = PZEM_voltage();
	*currP = PZEM_power();
	if ((*currV<0) || (*currP<0)) {
		*currV = -1;
		*currP = -1;
		return ESP_FAIL;
	}
	return ESP_OK;
}

typedef struct {
	int16_t ticks;
	int16_t powerPrc;
} powerTable_t;

const powerTable_t pwrAngle[] = {
	{0,100},
	{150,98},
	{207,95},
	{265,90},
	{345,80},
	{461,60},
	{564,40},
	{680,20},
	{760,10},
	{817,5},
	{874, 2},
	{1024, 0}
};

#define ARR_SIZE (sizeof(pwrAngle)/sizeof(powerTable_t))

int powerPrcByHpoint(int hp){
	if (hp>=HMAX) return 0;
	if (hp<=0) return 100;
	int i;
	for (i=0;i<ARR_SIZE;i++)	if (hp<=pwrAngle[i].ticks) break;
	if (hp==pwrAngle[i].ticks) return pwrAngle[i].powerPrc;

	int tmp = (pwrAngle[i-1].powerPrc - pwrAngle[i].powerPrc);
	tmp = ((pwrAngle[i].ticks - pwrAngle[i-1].ticks)+tmp/2)/tmp;
	return (pwrAngle[i-1].powerPrc - ((hp - pwrAngle[i-1].ticks)+tmp/2)/tmp);
}

int hpointByPowerPrc(int prc){
	if (prc>=100) return 0;
	if (prc<=0) return HMAX;
	int i;
	for (i=0;i<ARR_SIZE;i++)	if (prc >= pwrAngle[i].powerPrc) break;
	if (prc==pwrAngle[i].powerPrc) return pwrAngle[i].ticks;

	int tmp = (pwrAngle[i-1].powerPrc - pwrAngle[i].powerPrc);
	tmp = ((pwrAngle[i].ticks - pwrAngle[i-1].ticks)+tmp/2)/tmp;
	return (pwrAngle[i].ticks - tmp*(prc - pwrAngle[i].powerPrc));
}

void pzem_task(void *arg)
{
	TickType_t overPowerAlarmTime;
	bool  flag_overPower=0;
	bool  pzem_ok;

	PZEM_init();

	zero_imp_shift = getIntParam(NET_PARAMS, "z_shift");

	vTaskDelay(1000/portTICK_PERIOD_MS);

	while(1) {
		vTaskDelay(1200/portTICK_PERIOD_MS);
		// --------reading PZEM-----------
		pzem_ok = (readPZEM(&CurVolts,&CurPower) == ESP_OK);
		//=========ALARMS CHECKS========
		//---- PZEM connection--------
		if (!pzem_ok) { // no connection to PZEM, alarm!
			SET_ALARM(ALARM_PZEM_ERR);
		}
		else
		{
			CLEAR_ALARM(ALARM_PZEM_ERR);
			//-------------- low voltage 220V----------
			if (CurVolts<10)
				SET_ALARM(ALARM_FREQ);
			else
				CLEAR_ALARM(ALARM_FREQ);
			//-------------- no payload ----------
			if ((SetPower > 0) && (CurPower<5)  &&	 (Hpoint<=TRIAC_GATE_MAX_CYCLES) ) {
				SET_ALARM(ALARM_NOLOAD);
			} else {
				CLEAR_ALARM(ALARM_NOLOAD);
			}

			//-------------- triac breakdown to a short circuit ----------
			if (((CurPower- SetPower)*100L/getIntParam(DEFL_PARAMS, "maxPower"))>DELTA_TRIAK_ALARM_PRC){
				ERR_MSG("triac breakdown times:%d",flag_overPower);
				//myBeep(true);
				if (!flag_overPower){  // first detection
					overPowerAlarmTime = xTaskGetTickCount () + SEC_TO_TICKS(TRIAK_ALARM_DELAY_SEC);// memorize event time (by ticks)
					flag_overPower = true;
				}
				else {
					if (xTaskGetTickCount () > overPowerAlarmTime){
						SET_ALARM(ALARM_OVER_POWER);
					}
				}
			}
			else {
				flag_overPower = false;
				CLEAR_ALARM(ALARM_OVER_POWER);
			}
		}

		//======
		if (SetPower) {
			if (MainMode == MODE_IDLE) setPower(0);	// turn off if Monitor Mode
			if (PROC_END == MainStatus) setPower(0); // turn off on EndProc stage
		}

		//======
		if ((SetPower <= 0) || EXISTS_ALARM(ALARM_FREQ | ALARM_NOLOAD | ALARM_PZEM_ERR | ALARM_TEMP | ALARM_SENSOR_ERR)) {
			DBG("SetPower:%d  Alarm:%d",SetPower, AlarmMode);
			Hpoint = HMAX;
			continue;
		}

		// =============Power control=============
		int errP = CurPower-SetPower;
		int errPercent = abs((errP*100 + SetPower/2)/SetPower);

		if ((errP==0)||(errPercent==0)) { // precision is 1%
			DBG("STABLE cur:%d(%d) errPrc:%d Hpoint:%d",CurPower,SetPower,errPercent,Hpoint);
			continue;
		}
		int voltagePower = powerPrcByHpoint(Hpoint);
		int increment=0,calcPowerPrc=0;
		if ((CurPower==0) || (voltagePower==0))
		{
			Hpoint = hpointByPowerPrc(powerPrcByHpoint(Hpoint)+5);
		}
		else	{ // CurPower>0
			//--- number real watts on one percent of the voltage power
			double WattsOn1PrcVoltagePwr =(double)(CurPower)/(double)(voltagePower);
			DBG("WattsOn1PrcVoltagePwr:%4.1f",WattsOn1PrcVoltagePwr);
			int calcHpoint;

			if (errPercent>50){
				calcPowerPrc=(int)(0.5 + SetPower/WattsOn1PrcVoltagePwr);
			}
			else {
				calcPowerPrc = voltagePower + (int)(0.5 + errP/WattsOn1PrcVoltagePwr);
			}
			calcHpoint=hpointByPowerPrc(calcPowerPrc);
			increment = abs(Hpoint-calcHpoint);
			if (increment>1) 	increment = (increment +1)/2;
			if (!increment) increment=1;
			if (errP<0) increment=-abs(increment);

			Hpoint += increment;
			//zero_imp_shift = getIntParam(NET_PARAMS, "z_shift");
			DBG("===zshift:%d max_hpoint:%d",zero_imp_shift, HMAX-zero_imp_shift);
			if (Hpoint >= (HMAX-zero_imp_shift) ) {
				Hpoint = (HMAX-zero_imp_shift);
			}
			if (Hpoint<zero_imp_shift) {
				Hpoint=zero_imp_shift;
			}
			DBG("===set:%d cur:%d hp:%d inc:%d",SetPower, CurPower, Hpoint, increment);
		}// triac angle correction, as CurPower>0
	}//while(1)
}


const char *getMainModeStr(void)
{
	switch (MainMode) {
        case MODE_IDLE:	return "Монитор";
        case MODE_POWEERREG: return "Регулятор мощности";
	case MODE_DISTIL: return "Дистилляция";
	case MODE_RECTIFICATION: return "Ректификация";
	case MODE_TESTKLP: return "Тестирование клапанов";
	default: return "Неизвестно";
	}
}

const char *getMainStatusStr(void)
{
	switch (MainMode) {
    case MODE_IDLE:
	case MODE_TESTKLP:
		return "Ожидание команды";
	case MODE_POWEERREG:
		switch (MainStatus) {
		case PROC_START: return "Стабилизация мощности";
		}
	case MODE_DISTIL:
		switch (MainStatus) {
		case PROC_DISTILL: return "Дистилляция";
		}
	case MODE_RECTIFICATION:
		switch (MainStatus) {
		case START_WAIT: return "Ожидание запуска процесса";
		case PROC_START: return "Начало процесса";
		case PROC_RAZGON: return "Разгон";
		case PROC_STAB: return "Стабилизация колонны";
		case PROC_GLV: return "Отбор головных фракций";
		case PROC_T_WAIT: return "Рестабилизация по стопу";
		case PROC_SR: return "Отбор СР";
		case PROC_HV: return "Отбор хвостовых фракций";
		case PROC_WAITEND: return "Охлаждение куба";
		default: return "Завершение работы";
		}
		break;
	default:
		return "Неизвестно";
	}

}

const char *getAlarmModeStr(void)
{
	static char str[256];
	int cnt;

	if (!AlarmMode) return "<b class=\"green\">Не зафиксированo</b>";
	strcpy(str, "<b class=\"red\">");

	if (EXISTS_ALARM(ALARM_TEMP)) {
		cnt = sizeof(str) - strlen(str);
		strncat(str, "Превышение температуры", cnt);
	}
	if (EXISTS_ALARM(ALARM_WATER)) {
		cnt = sizeof(str) - strlen(str);
		strncat(str, " Нет охлаждения", cnt);
	}
	if (EXISTS_ALARM(ALARM_FREQ)) {
		cnt = sizeof(str) - strlen(str);
		strncat(str, "Нет сети", cnt);
	}
	if (EXISTS_ALARM(ALARM_NOLOAD)) {
		cnt = sizeof(str) - strlen(str);
		strncat(str,"  Нет нагрузки", cnt);
	}
	if (EXISTS_ALARM(ALARM_EXT)) {
		cnt = sizeof(str) - strlen(str);
		strncat(str,"  аварийный датчик", cnt);
	}
	if (EXISTS_ALARM(ALARM_OVER_POWER)) {
		cnt = sizeof(str) - strlen(str);
		strncat(str,"  ВЫСОКАЯ МОЩНОСТЬ !", cnt);
	}
	if (EXISTS_ALARM(ALARM_PZEM_ERR)) {
		cnt = sizeof(str) - strlen(str);
		strncat(str,"  Ошибка PZEM!", cnt);
	}
	if (EXISTS_ALARM(ALARM_SENSOR_ERR)) {
		cnt = sizeof(str) - strlen(str);
		strncat(str,"  нет датчика температуры!", cnt);
	}
	strcat(str,"</b>");
	return str;
}


// Получение строки о причине перезагрузки
const char *getResetReasonStr(void)
{

	switch (resetReason) {
        case POWERON_RESET: return "Vbat power on reset";
        case SW_RESET: return "Software reset digital core";
        case OWDT_RESET: return "Legacy watch dog reset digital core";
        case DEEPSLEEP_RESET: return "Deep Sleep reset digital core";
        case SDIO_RESET: return "Reset by SLC module, reset digital core";
        case TG0WDT_SYS_RESET: return "Timer Group0 Watch dog reset digital core";
        case TG1WDT_SYS_RESET: return "Timer Group1 Watch dog reset digital core";
        case RTCWDT_SYS_RESET: return "RTC Watch dog Reset digital core";
        case INTRUSION_RESET: return "Instrusion tested to reset CPU";
        case TGWDT_CPU_RESET: return "Time Group reset CPU";
        case SW_CPU_RESET: return "Software reset CPU";
        case RTCWDT_CPU_RESET: return "RTC Watch dog Reset CPU";
        case EXT_CPU_RESET: return "For APP CPU, reseted by PRO CPU";
        case RTCWDT_BROWN_OUT_RESET: return "Reset when the vdd voltage is not stable";
        case RTCWDT_RTC_RESET: return "RTC Watch dog reset digital core and rtc module";
	default: return "Uncnown reason";
	}
}

/* valve program PWM task  (one task for each valve)
 * @*arg is (int valve number)
 */
void valvePWMtask(void *arg){
	int num=(int)arg;
	TickType_t xLastWakeTime=xTaskGetTickCount ();
	DBGV("======started v:%d",num);
	while(1) {
		if (Klp[num].is_pwm) {
			DBGV("pwmON |%04.1f sec|",Klp[num].open_time);
			if (Klp[num].open_time>0.2) { //if time less 0.2 sec do nothing
				cmd2valve (num, cmd_open);				//turn-on valve
				vTaskDelayUntil( &xLastWakeTime, SEC_TO_TICKS(Klp[num].open_time));
			}

			if (Klp[num].is_pwm){
				DBGV("pwmOFF|%04.1f sec|",Klp[num].close_time);
				if (Klp[num].close_time>0.2) { //if time less 0.2 sec do nothing
					cmd2valve (num, cmd_close); //turn-off valve
					vTaskDelayUntil( &xLastWakeTime, SEC_TO_TICKS(Klp[num].close_time));
				}
			}
		}

		if (!Klp[num].is_pwm)	{  // if no pwm
			DBGV("suspend v:%d",num);
			vTaskSuspend(NULL);	// stop the task (it will be resumed when pwm is turn-on in func startKlpPwm() )
			DBGV("resume  v:%d",num);
			xLastWakeTime = xTaskGetTickCount ();
		}
	}
}


void IRAM_ATTR timer0_group0_isr(void *para)
{
	int timer_idx = (int) para;
	uint32_t intr_status = TIMERG0.int_st_timers.val;

	if (intr_status & BIT(timer_idx)) {

	        /*Timer will reload counter value*/
	        TIMERG0.hw_timer[timer_idx].update = 1;
		/*We don't call a API here because they are not declared with IRAM_ATTR*/
		TIMERG0.int_clr_timers.t1 = 1;
		/* For a auto-reload timer, we still need to set alarm_en bit if we want to enable alarm again.*/
		TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;

		if (beepActive) {
			// Обработка пищалки
			beeperTime--;
			if (beeperTime <= 0) {
				beeperTime = 0;
				beepActive = false;
				GPIO_OFF(GPIO_BEEP);
				//GPIO.out_w1tc = (1 << GPIO_BEEP);
			}
		}	

		tic_counter++;
		if (tic_counter >= 1000) {
			tic_counter=0;
			uptime_counter++;
			if (rect_timer1>0) rect_timer1--; // Таймер для отсчета секунд 1
			else rect_timer1=0;
			if (timer_sec2>0) timer_sec2--;	// Таймер для отсчета секунд 2
			else timer_sec2=0;
			if (timer_sec3>0) timer_sec3--;	// Таймер для отсчета секунд 3
			else timer_sec3=0;
			CurFreq = gpio_counter;
			gpio_counter=0;
			//xQueueSendFromISR(timer_queue, &intr_status, NULL);
		}

	}
}

/**
 * Настройка аппаратного таймера из group0
 */
static void tg0_timer0_init()
{
	int timer_group = TIMER_GROUP_0;
	int timer_idx = TIMER_1;
	timer_config_t config;

	//timer_queue = xQueueCreate(10, sizeof(uint32_t));

	config.alarm_en = true;
	config.auto_reload = 1;
	config.counter_dir = TIMER_COUNT_UP;
	config.divider = TIMER_DIVIDER;
	config.intr_type = TIMER_INTR_LEVEL;
	config.counter_en = TIMER_PAUSE;

    /* Конфигурация таймера */
    timer_init(timer_group, timer_idx, &config);

    /* Ставим таймер на паузу */
    timer_pause(timer_group, timer_idx);

    /* Загружаем значение таймера */
    timer_set_counter_value(timer_group, timer_idx, 0);

    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, (TIMER_INTERVAL_SEC * TIMER_SCALE) - TIMER_FINE_ADJ);
    /* Разрешаем прерывания таймера */
    timer_enable_intr(timer_group, timer_idx);

    /* Устанавливаем обработчик прерывания */
    timer_isr_register(timer_group, timer_idx, timer0_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    /* Запускаем отсчет таймера */
    timer_start(timer_group, timer_idx);

    xTaskCreate(valvePWMtask, "valvePWMtask", 8192, NULL, 5, NULL);
}

// ISR triggered by GPIO edge at the end of each Alternating Current half-cycle.
// Used to reset the PWM timer, which synchronize the TRIAC operation with
// the mains frequency.  Also used to perform fading of PWM phase.
void IRAM_ATTR gpio_isr_handler(void* arg) 
{ 
	uint32_t intr_st = GPIO.status;
	if (intr_st & (1 << GPIO_DETECT_ZERO)) {

		if (!(GPIO.in & (1 << GPIO_DETECT_ZERO))) {
			// Zero the PWM timer at the zero crossing.
			LEDC.timer_group[0].timer[0].conf.rst = 1;
			LEDC.timer_group[0].timer[0].conf.rst = 0;
		
			if (Hpoint >= HMAX - TRIAC_GATE_MAX_CYCLES) {
				// If hpoint if very close to the maximum value, ie mostly off, simply turn off 
				// the output to avoid glitch where hpoint exceeds duty. 
				LEDC.channel_group[0].channel[0].conf0.sig_out_en = 0; 
			} else { 
				LEDC.channel_group[0].channel[0].hpoint.hpoint = Hpoint; 
				LEDC.channel_group[0].channel[0].conf0.sig_out_en = 1; 
				LEDC.channel_group[0].channel[0].conf1.duty_start = 1; 
			} 
			gpio_counter++;
		}
	} else if (intr_st & (1 << GPIO_ALARM)) {
		// Авария от внешнего источника
		AlarmMode |= ALARM_EXT;
	}
	GPIO.status_w1tc = intr_st;
} 

/* Настройка и установка состояния GPIO для работы */
void setProcessGpio(int on)
{
	static char is_configured = 0;
	uint32_t pin_sel = 0;
	uint32_t gpio = getIntParam(DEFL_PARAMS, "processGpio");
	if (gpio <= 0 && gpio > 64)   return;

	pin_sel = 1<<gpio;

	if (!is_configured) {
		// Configure output gpio
		gpio_config_t io_conf;
		io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
		io_conf.mode = GPIO_MODE_OUTPUT;
		io_conf.pin_bit_mask = pin_sel;
		io_conf.pull_down_en = 0;
		io_conf.pull_up_en = 0;
		gpio_config(&io_conf);
		is_configured++;
	}
	gpio_set_level(gpio, on);	

}

/* Загрузка и установка параметров работы */
int paramSetup(void)
{
	// Загрузка параметров
	if (param_load(DEFL_PARAMS, RECT_CONFIGURATION) < 0) {
		// Файл не найден - заполняем значениями по умолчанию
		return param_default(DEFL_PARAMS, RECT_CONFIGURATION);
	}
	return ESP_OK;
}

cJSON* getInformation(void)
{
	char data[80];
	const char *wo;
	cJSON *ja, *j, *jt;
	time_t CurrentTime;
	struct tm CurrentTm;

	CurrentTime = time(NULL);
	localtime_r(&CurrentTime, &CurrentTm);
	ja = cJSON_CreateObject();
	cJSON_AddItemToObject(ja, "cmd", cJSON_CreateString("info"));
	snprintf(data, sizeof(data)-1, "%02d:%02d", CurrentTm.tm_hour, CurrentTm.tm_min);
	cJSON_AddItemToObject(ja, "time", cJSON_CreateString(data));
	snprintf(data, sizeof(data)-1, "%02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
	cJSON_AddItemToObject(ja, "MainMode", cJSON_CreateNumber(MainMode));
	cJSON_AddItemToObject(ja, "MainModeStr", cJSON_CreateString(getMainModeStr()));
	cJSON_AddItemToObject(ja, "MainStatus", cJSON_CreateNumber(MainStatus));
	cJSON_AddItemToObject(ja, "MainStatusStr", cJSON_CreateString(getMainStatusStr()));
	cJSON_AddItemToObject(ja, "uptime", cJSON_CreateString(data));
	cJSON_AddItemToObject(ja, "CurVolts", cJSON_CreateNumber(CurVolts));
	cJSON_AddItemToObject(ja, "CurPower", cJSON_CreateNumber(CurPower));
	cJSON_AddItemToObject(ja, "SetPower", cJSON_CreateNumber(SetPower));
	cJSON_AddItemToObject(ja, "CurFreq", cJSON_CreateNumber(CurFreq/2));
	cJSON_AddItemToObject(ja, "Alarm", cJSON_CreateNumber(AlarmMode));
	cJSON_AddItemToObject(ja, "AlarmMode", cJSON_CreateString(getAlarmModeStr()));
	if (WaterOn<0) wo = "No data";
	else if (0 == WaterOn) wo = "Off";
	else wo = "On";
	cJSON_AddItemToObject(ja, "WaterOn", cJSON_CreateString(wo));
	cJSON_AddItemToObject(ja, "TempWaterIn", cJSON_CreateNumber(TempWaterIn));
	cJSON_AddItemToObject(ja, "TempWaterOut", cJSON_CreateNumber(TempWaterOut));
	cJSON_AddItemToObject(ja, "WaterFlow", cJSON_CreateNumber(WaterFlow));
	cJSON_AddItemToObject(ja, "heap", cJSON_CreateNumber(esp_get_free_heap_size()));
	if (bmpTemperature > 0 && bmpTruePressure > 0) {
		cJSON_AddItemToObject(ja, "bmpTemperature", cJSON_CreateNumber(bmpTemperature));
		snprintf(data, sizeof(data)-1, "%0.2f", bmpTruePressure/133.332);
		cJSON_AddItemToObject(ja, "bmpTruePressure", cJSON_CreateString(data));
		cJSON_AddItemToObject(ja, "bmpPressurePa", cJSON_CreateNumber(bmpTruePressure));
	}

	j = getDSjson();
	cJSON_AddItemToObject(ja, "sensors", j);

	j = cJSON_CreateArray();
	cJSON_AddItemToObject(ja, "klapans", j);
	for (int i=0; i<MAX_KLP; i++) {
		int pwm = Klp[i].open_time+Klp[i].close_time;
		float pwm_percent = 0;
		if (Klp[i].open_time>0) {
			float p = pwm/Klp[i].open_time;
			if (p) pwm_percent = roundX(100/p,2);
		}

		jt = cJSON_CreateObject();
		cJSON_AddItemToArray(j, jt);
		cJSON_AddItemToObject(jt, "id", cJSON_CreateNumber(i));
		cJSON_AddItemToObject(jt, "is_pwm", cJSON_CreateNumber(Klp[i].is_pwm));
		cJSON_AddItemToObject(jt, "is_open", cJSON_CreateNumber(Klp[i].is_open));
		cJSON_AddItemToObject(jt, "pwm_time", cJSON_CreateNumber((int)pwm));
		cJSON_AddItemToObject(jt, "pwm_percent", cJSON_CreateNumber((int)(pwm_percent+0.5)));
	}

	if (MODE_RECTIFICATION == MainMode) {
		float timeStabKolonna= fabs(getFloatParam(DEFL_PARAMS, "timeStabKolonna"));
		// Режим ректификации
		switch (MainStatus) {
		  case PROC_STAB:
			  // TimeStab
				snprintf(data, sizeof(data)-1, "%02d/%02.0f sec", uptime_counter-secTempPrev, timeStabKolonna);
				cJSON_AddItemToObject(ja, "rect_time_stab", cJSON_CreateString(data));
				/* no break */
		  case PROC_GLV:
				// TemperatureStab
				//snprintf(data, sizeof(data)-1, "%02.1f C", tempStabSR);
				//cJSON_AddItemToObject(ja, "rect_t_stab", cJSON_CreateString(data));
				break;
		  case PROC_T_WAIT:
				snprintf(data, sizeof(data)-1, "%02d sec", rect_timer1);
				cJSON_AddItemToObject(ja, "rect_timer1", cJSON_CreateString(data));
				// delta TemperatureStab
				snprintf(data, sizeof(data)-1, "%02.1f <-- %02.1f C", tempStabSR, getTube20Temp());
				cJSON_AddItemToObject(ja, "rect_t_stab", cJSON_CreateString(data));
				break;
		  case PROC_SR:
				// TemperatureStab
				snprintf(data, sizeof(data)-1, "%02.1f C", tempStabSR);
				cJSON_AddItemToObject(ja, "rect_t_stab", cJSON_CreateString(data));
				// PWMsr
				cJSON_AddItemToObject(ja, "rect_p_shim", cJSON_CreateNumber(ProcChimSR));
				break;
		  default:
			  break;
		}
	}
	return ja;
}

// Отправка SMS
void sendSMS(char *text)
{
	request_t *req;
	int ret;
	char *post, *user, *hash, *phones;
	int s;
	if (!getIntParam(NET_PARAMS, "useSmsc")) return;

	user = getStringParam(NET_PARAMS, "smscUser");
	hash = getStringParam(NET_PARAMS, "smscHash");
	phones = getStringParam(NET_PARAMS, "smscPhones");


	if (!user || !hash || !phones) return;
	if (strlen(user)<=0 || strlen(hash)<=30 || strlen(phones)<=0) return;
	s = strlen(user) + strlen(hash) + strlen(phones) + strlen(text);
	post = malloc(s+30);
	if (!post) return;
	sprintf(post, "login=%s&psw=%s&phones=%s&mes=%s", user, hash, phones, text);

	DBG(">> SMS start");
	req = req_new("https://smsc.ru/sys/send.php"); 
	req_setopt(req, REQ_SET_METHOD, "POST");
	req_setopt(req, REQ_SET_POSTFIELDS, post);
	ret = req_perform(req);
	if (ret/100 > 2) {
		DBG("sms failed, error code: %d", ret);
	}
	req_clean(req);
	free(post);
	DBG("<< Sms Done");
}


// Установка рабочей мощности
void setPower(int16_t pw)
{
	int16_t mp;
	mp = getIntParam(DEFL_PARAMS, "maxPower");
	if (pw > mp) SetPower = mp;
	else SetPower = pw;

	if (pw > mp/2) {
		LEDC.channel_group[0].channel[0].duty.duty = (TRIAC_GATE_IMPULSE_CYCLES*3) << 4;
	} else {
		LEDC.channel_group[0].channel[0].duty.duty = TRIAC_GATE_IMPULSE_CYCLES << 4;
	}

	if (pw>0) setProcessGpio(1);
	else setProcessGpio(0);

	if (nvsHandle) {
		nvs_set_i16(nvsHandle, "SetPower", SetPower);
	}
}                                                   

void setNewProcChimSR(int16_t newValue)
{
	LOG("PWM %d->%d",ProcChimSR,newValue);
	if (ProcChimSR != newValue) {
		ProcChimSR = newValue;
		if (nvsHandle) {
			nvs_set_i16(nvsHandle, "ProcChimSR", ProcChimSR);
		}
	}
}

void save_double_NVS(double newValue, const char* name){
	union {
		uint64_t i;
		double d;
	} var;
	var.d = newValue;
	if (nvsHandle) {
		ESP_ERROR_CHECK(nvs_set_u64(nvsHandle, name, var.i));
	}
}


void setTempStabSR(double newValue)
{
	if (tempStabSR != newValue){
		tempStabSR = newValue;
		save_double_NVS(newValue,"tempStabSR");
	}
}


static void setTempTube20Prev(double newValue)
{
	if (tempTube20Prev != newValue){
		tempTube20Prev = newValue;
		save_double_NVS(newValue,"tempTube20Prev");
	}
}

void set_status(int16_t newStatus)
{
	DBG("%d->%d",MainStatus,newStatus);
	if ((newStatus==PROC_END) &&
		((MainMode==MODE_DISTIL) || (MainMode==MODE_RECTIFICATION)) &&
		((MainStatus>PROC_RAZGON)&&(MainStatus<PROC_WAITEND))
		){
		MainStatus = PROC_WAITEND;
	}
	else {
		MainStatus = newStatus;
	}
	if (nvsHandle) {
		nvs_set_i16(nvsHandle, "MainStatus", MainStatus);
	}
	if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
}


// Установка нового режима работы
void setMainMode(int nm)
{
	main_mode new_mode = (main_mode) nm;
	if (new_mode == MainMode) return; // Не изменился
	MainMode = new_mode;
	if (nvsHandle) {
		nvs_set_i16(nvsHandle, "MainMode", nm);
	}

	if (SetPower) {
		setPower(0);
	}

	if (MainMode != MODE_POWEERREG) {
		set_status(START_WAIT);
	}

	switch (MainMode) {
	case MODE_IDLE:
		// Режим мониторинга
		LOG("Main mode: Idle.");
		break;
	case MODE_POWEERREG:
		// Режим регулятора мощности
		LOG("Main mode: Power reg.");
		if (MainStatus !=PROC_START)	setPower(getIntParam(DEFL_PARAMS, "ustPowerReg"));
		set_status(PROC_START);
		break;
	case MODE_DISTIL:
		// Режим дистилляции
		LOG("Main mode: Distillation.");
		break;
	case MODE_RECTIFICATION:
		// Режим ректификации
		LOG("Main mode: Rectification.");
		break;
	case MODE_TESTKLP:
		// Режим тестирования клапанов
		LOG("Main mode: Test klp.");
		break;
	}
	myBeep(false);
}

const state_vector_t rect_states[]={
	 {START_WAIT  /* START_WAIT	*/,PROC_START},
	 {START_WAIT  /* PROC_START	*/,PROC_RAZGON},
	 {START_WAIT  /* PROC_RAZGON	*/,PROC_STAB},
	 {PROC_RAZGON /* PROC_STAB	*/,PROC_GLV},
	 {PROC_STAB	/* PROC_GLV		*/,PROC_T_WAIT},
	 {PROC_GLV	/* PROC_T_WAIT	*/,PROC_SR},
	 {PROC_GLV	/* PROC_SR		*/,PROC_HV},
	 {PROC_SR		/* PROC_HV		*/,PROC_WAITEND},
	 {PROC_HV		/* PROC_WAITEND	*/,PROC_END},
	 {PROC_WAITEND	/* PROC_END		*/,START_WAIT}
};

const state_vector_t dist_states[]={
	 {START_WAIT  	/* START_WAIT*/,	PROC_START},
	 {START_WAIT  	/* PROC_START*/,	PROC_RAZGON},
	 {START_WAIT  	/* PROC_RAZGON*/,	PROC_GLV},
	 {PROC_RAZGON	/* PROC_STAB	*/,	PROC_GLV}, 			//inapplicable
	 {PROC_RAZGON	/* PROC_GLV		*/,	PROC_DISTILL},
	 {PROC_GLV			/* PROC_T_WAIT*/,	PROC_DISTILL},	//inapplicable
	 {PROC_GLV			/* PROC_DISTILL*/,	PROC_HV},
	 {PROC_DISTILL	/* PROC_HV		*/,	PROC_WAITEND},
	 {PROC_HV			/* PROC_WAITEND*/,PROC_END},
	 {PROC_WAITEND	/* PROC_END_	*/,	START_WAIT}
};


// Ручная установка состояния конечного автомата
void moveStatus(int next)
{
	if ((next) && (MainStatus>=PROC_END)) return;
	if ((!next) && (MainStatus<= START_WAIT)) return;

	int state=MainStatus;
	if (MainStatus==PROC_END) state=PROC_END_;

	const state_vector_t* vectors=NULL;

	switch (MainMode) {
		case MODE_DISTIL:
			vectors =  dist_states+state;
			break;
		case MODE_RECTIFICATION:
			vectors =  rect_states+state;
			break;
		default:
			vectors =  NULL;
	}

	if (vectors){
		if (next) { //шаг вперед
			state = vectors->nextState;
		}
		else { // на предыдущий
			state = vectors->prevState;
		}
	}
	else {
		if (next) { //шаг вперед
			state = PROC_END;
		}
		else { // на предыдущий
			state = START_WAIT;
		}
	}
	set_status(state);
}

/*
 * Если настройка "Количество значений в таблице обучения" больше 2
 * 		- вернет ШИМ по таблице-"шпоре" от температуры низа колонны
 * иначе
 * 		- вернет текущее ProcChimSR
 */
int16_t GetSrPWM(void)
{
	int16_t table_size=getIntParam(DEFL_PARAMS, "cntCHIM");
	if (table_size<2) return ProcChimSR;

	int16_t found = ProcChimSR;
	double t = getTube20Temp();

	DBG("first ProcChimSR:%d t:%0.2f table_size:%d",ProcChimSR,t,table_size);
	for (int i=1; i<table_size; i++) {
		if (autoPWM[i-1].temp <= t  && autoPWM[i].temp > t) {
			DBG("[%d]temp:%0.2f pwm:%d		[%d]temp:%0.2f pwm:%d",i-1,autoPWM[i-1].temp,autoPWM[i-1].pwm,i,autoPWM[i].temp,autoPWM[i].pwm);
			if (autoPWM[i-1].pwm > 0) {
				found = (t - autoPWM[i-1].temp) * 
					(autoPWM[i].pwm - autoPWM[i-1].pwm) / 
					(autoPWM[i].temp - autoPWM[i-1].temp) + autoPWM[i-1].pwm;
				DBG("pwm-1>0,  return %d",found);
				return found;
			}
		}	
	}
	DBG("after FOR, return %d",found);
	return found;
}

bool end_condition_SR(void){
	float tempEndRectOtbSR = getFloatParam(DEFL_PARAMS, "tempEndRectOtbSR");
	if (tempEndRectOtbSR>0)					// контроль по Т куба
		return (getCubeTemp() >= tempEndRectOtbSR);
	else							// контроль по Т20
		return (getTube20Temp() >= fabs(tempEndRectOtbSR) );
}

esp_err_t sensor_err(double t){
	if (-1 == t) { // сбой датчика
		DBG("error DS of '20%% tube'");
		SET_ALARM(ALARM_SENSOR_ERR);
		return ESP_FAIL;
	}
	CLEAR_ALARM(ALARM_SENSOR_ERR);
	return ESP_OK;
}

// Обработка состояний в режиме ректификации
void Rectification(void)
{
	double t;
	float tempEndRectRazgon;
	char b[80];
	static int16_t prev_status=START_WAIT;

	switch (MainStatus) {
	case START_WAIT:		// Ожидание запуска процесса
		if (prev_status!=MainStatus){
			prev_status=MainStatus;
			setPower(0);		// Снятие мощности с тэна
			closeAllKlp();		// Закрытие всех клапанов.
		}
		break;
	case PROC_START:
		// Начало процесса
		LOG("START");
		set_status(PROC_RAZGON); // @suppress("No break at end of case")
		 /* fall through */

	case PROC_RAZGON: 		// Разгон до рабочей температуры

		tempEndRectRazgon = getFloatParam(DEFL_PARAMS, "tempEndRectRazgon");
		if (tempEndRectRazgon > 0) t = getCubeTemp();
		else t = getTube20Temp();

		// старт
		if (prev_status!=MainStatus){
			prev_status=MainStatus;
			set_proc_power("maxPower");
			startPressure = bmpTruePressure; // Фиксация атм. давления.
			closeAllKlp();		// Закрытие всех клапанов.
			LOG("RAZGON");
		}
		// контроль

		if (sensor_err(t)) break;

		if (t < fabs(tempEndRectRazgon)) break;

		// завершение
		set_status(PROC_STAB);
		DBG("Switch state to stabilization."); // @suppress("No break at end of case")
		 /* fall through */

	case PROC_STAB:		//Стабилизация колонны
		// старт
		t = getTube20Temp();
		if (prev_status!=MainStatus){
			LOG("STABILIZATION");
			openKlp(klp_water);	// Открытие клапана воды
			closeKlp(klp_glwhq);
			closeKlp(klp_sr);
			set_proc_power("powerRect");
			// Запоминаем температуру и время
	        setTempStabSR(t);
			setTempTube20Prev(t);
			secTempPrev = uptime_counter;
			prev_status=MainStatus;
		}

		// контроль
		if ( sensor_err(t) || (t < 70))			//сбой датчика или колонна еще не прогрелась, ждем
		{
			DBG("Cube or 20%% tube temperature less 70 dg.");
			secTempPrev = uptime_counter;
			break;
		}

		float timeStabKolonna = getFloatParam(DEFL_PARAMS, "timeStabKolonna");

		if (timeStabKolonna > 0) {			// Время считаем относительно последнего изменения температуры
			if (fabs(t - tempTube20Prev) < 0.2) {	// температура колонны в пределах погрешности в 0.2 градуса C
				DBG( "Stabillization %d of %02.0f sec.", uptime_counter-secTempPrev, fabs(timeStabKolonna));
				if (uptime_counter-secTempPrev<timeStabKolonna) {// время стабилизации истекло?
					break; // нет, продолжаем стабилизацию
				}
				// время стабилизации истекло, заканчиваем
			} else { // температура колонны изменилась более чем на 0.2 градуса
				DBG("Stab. temp. changed from %0.2f to %0.2f. Reseting timer.", tempTube20Prev, t);
		        setTempStabSR(t);
				setTempTube20Prev(t); // фиксируем новую температуру
				secTempPrev = uptime_counter;//начинаем отсчет времени с нуля
				break;
			}
		} else {	// время считаем от начала стабилизации
			DBG("Stabillization %d of %0.0f.", uptime_counter-secTempPrev, fabs(timeStabKolonna));
			if (uptime_counter-secTempPrev < fabs(timeStabKolonna)) {		// время истекло?
				break; // нет - продолжаем стабилизацию
			}
			// время стабилизации истекло, заканчиваем
		}

		// завершение
		set_status(PROC_GLV);	//переходим к отбору голов.
		DBG("end razgon, switch to `glv` stage"); // @suppress("No break at end of case")
		 /* fall through */

	case PROC_GLV:			// Отбор головных фракций
		// старт
		if (prev_status!=MainStatus){
			LOG("PROC_GLV");
			prev_status=MainStatus;
			openKlp(klp_water);		// Открытие клапана воды
			closeKlp(klp_sr);				// Отключение клапана отбора товарного продукта
			secTempPrev = uptime_counter;
			setTempStabSR(getTube20Temp());	// температура стабилизации отбора
			set_proc_power("powerRect");
			start_valve_PWMpercent(
					klp_glwhq, // медленный ШИМ клапан голов
					getFloatParam(DEFL_PARAMS, "timeChimRectOtbGlv"),
					getFloatParam(DEFL_PARAMS, "procChimOtbGlv"));
		}

		// контроль
		float tEndRectOtbGlv =getFloatParam(DEFL_PARAMS, "tEndRectOtbGlv");

		if (tEndRectOtbGlv>0){ 						// tEndRectOtbGlv положительный -> это температура куба завершения отбора голов
			t = getCubeTemp();
			if (sensor_err(t)) break; 					// ошибка датчика, ждем исправления

			if (t < tEndRectOtbGlv) 	// Т куба не достигла заданной
				break;											// продолжаем
		}
		else {												// tEndRectOtbGlv отрицательное -> это кол-во минут отбора голов
			if ((uptime_counter-secTempPrev) < (-tEndRectOtbGlv*60) )	// время не истекло
				break;												// продолжаем
		}

		// Окончание отбора голов
		set_status(PROC_T_WAIT);		// Переходим к стабилизации температуры
		DBG("end glv, switch to `T wait` stage"); // @suppress("No break at end of case")
		 /* fall through */

	case PROC_T_WAIT: // ре-стабилизация  после стопа
		// старт
		t = getTube20Temp();

		if (prev_status!=MainStatus){
			LOG("PROC_RESTAB");
			closeKlp(klp_sr);				// Отключение клапана отбора товарного продукта
			closeKlp(klp_glwhq); 		// Отключение клапана отбора голов/хвостов
			openKlp(klp_water);		// Открытие клапана воды
			if ((prev_status == PROC_SR)){ //попали сюда по "стоп" отбора продукта
				rect_timer1 = getIntParam(DEFL_PARAMS, "timeRestabKolonna");
			}
			else {//попали сюда или с отбора голов или после [ре]старта контроллера
				if (!broken_proc){
					setTempStabSR(t); // начальная температура стабилизации
					setNewProcChimSR(getIntParam(DEFL_PARAMS, "beginProcChimOtbSR")); // стартовый % отбора тела
				}
				set_proc_power("powerRect");  //задаем мощность ректификации
			}
			prev_status=MainStatus;
		}
		if (sensor_err(t)) break; // сбой датчика, ждем
		if (tempStabSR <= 0) setTempStabSR(28.5);

		//-----------контроль
		// настройка времени рестабилизации не ноль и истек таймер: принимаем текущую температуру за Тстаб
		if ( (0 == rect_timer1) &&  // таймер досчитал
			 (getIntParam(DEFL_PARAMS, "timeRestabKolonna") > 0) // настройка времени стабилизации не ноль
			) {
			DBG("restab. newTstab:%0.2f  timer:%d setting:%d", tempStabSR,rect_timer1,getIntParam(DEFL_PARAMS, "timeRestabKolonna"));
			setTempStabSR(getTube20Temp());		// принимаем текущую температуру за Тстаб
			if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		}

		//  пока рестабилизировались - не пришла ли пора завершить тело и перейти к хвостам?
		if (end_condition_SR())	{
       		set_status(PROC_HV); // Уходим на стадию отбора хвостов
			break;
		}
		
		//  проверим: Т низа колонные снизилась до Тстаб?
		if (getTube20Temp() > tempStabSR) {
			break; //если нет, остаемся в рестабилизации
		}
		//рестабилизация завершена

		//------------- завершение---------------
		secTempPrev = uptime_counter;	// время, когда стабилизировалась температура
		DBG("T restored; go to SR. T:%0.2f", tempStabSR);
		set_status(PROC_SR);	// @suppress("No break at end of case") Переход к отбору продукта
        /* fall through */

	case PROC_SR:	// Отбор СР
		// старт
		if (prev_status!=MainStatus){
			if (prev_status!=PROC_T_WAIT) { // если не после рестабилизации
				openKlp(klp_water);		// Открытие клапана воды
				closeKlp(klp_glwhq); 		// Отключение клапана отбора голов/хвостов
				set_proc_power("powerRect");
			}
			setNewProcChimSR(GetSrPWM());// PWM отбора (по-шпоре)
			int period=getIntParam(DEFL_PARAMS, "timeChimRectOtbSR");
			start_valve_PWMpercent	// включаем медленный ШИМ клапана продукта
			  ( klp_sr,
				period,// период ШИМ в сек
				ProcChimSR //%
			  );

			if (tempStabSR <= 0) setTempStabSR(28.5);
			prev_status=MainStatus;
			LOG("PROC_SR  Tstab:%5.1f PWM:%d%%(%d sec)",tempStabSR,ProcChimSR,period);
		}

		//-----------контроль
		t = getTube20Temp(); // T низа колонны
		if (sensor_err(t)) break; // сбой датчика, ждем

		int minProcChimOtbSR = getIntParam(DEFL_PARAMS, "minProcChimOtbSR"); // минимальный % отбора

		if ((t>0)&&(t < tempStabSR)) {	//если температура стала ниже Т стабилизации,
			setTempStabSR(t);	// принимаем ее за новую Т стабилизации (ПБ)
		}
		float delta_limit = getFloatParam(DEFL_PARAMS, "tempDeltaRect");
		if ( t >= (tempStabSR + delta_limit) ) {	// Температура превысила Тстаб+дельта, переход на рестабилизацию
			LOG("STOP by delta:%5.2f(limit:%05.1f Tube20:%5.1f Tstb:%05.1f",(t-tempStabSR),delta_limit,t,tempStabSR);
			//-- вносим температуру стопа в таблицу самообучения
			int cntCHIM = getIntParam(DEFL_PARAMS, "cntCHIM");
			if (cntCHIM < 0) {
				// Запоминаем температуру, когда произошел стоп за вычетом 0.1 градуса.
				autoPWM[-cntCHIM].temp = t - 0.1;
				autoPWM[-cntCHIM].pwm = ProcChimSR;
				if (-cntCHIM < COUNT_PWM-1) cntCHIM--;
			}

			//-- уменьшаем % отбора по заданному декременту
			int decrementCHIM = getIntParam(DEFL_PARAMS, "decrementCHIM");
			int pwm_sr=ProcChimSR;
			if (decrementCHIM>=0) { // положительное значение: декремент в абсолютном значении процентов
				pwm_sr -= decrementCHIM; // вычтем из текущего %отбора
			} else {// отрицательное значение: декремент относительный, в процентах от текущего %та
				uint16_t v = (ProcChimSR * (-decrementCHIM + 50))/100; // рассчитали величину декремента
				if (v<=0) v=1; 	// если декремент получился ноль, зададим 1
				pwm_sr -= v; 	// уменьшим % отбора на расчитанную величину
			}
			if (pwm_sr < minProcChimOtbSR)	pwm_sr = minProcChimOtbSR;
			LOG("dec PWM %d->%d",ProcChimSR,pwm_sr);
			if (ProcChimSR != pwm_sr)	 setNewProcChimSR(pwm_sr);

			//-- переключение на рестабилизацию
			set_status(PROC_T_WAIT);
			if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
			DBG("Switch state to temperature re-stabilization.");
			break;
		}

		// -- автоувеличение % отбора - если меньше 100%
		if (ProcChimSR<100){
			int timeAutoIncCHIM = getIntParam(DEFL_PARAMS, "timeAutoIncCHIM");
			if ((timeAutoIncCHIM>0) && ((uptime_counter - secTempPrev) > timeAutoIncCHIM) ) {
				if (ProcChimSR > minProcChimOtbSR) {	// Шим прибавляем только если не минимальный
					int incrementCHIM = getIntParam(DEFL_PARAMS, "incrementCHIM");
					int pwm=ProcChimSR;
					if (incrementCHIM>=0) {	// инкремент - абсолютное значение
						pwm += incrementCHIM;
					} else {								// инкремент - процент от текущих % отбора
						pwm +=  ProcChimSR*(-incrementCHIM + 50)/100;
						if (pwm == ProcChimSR) pwm++;
					}
					if (pwm>100) pwm=100;
					LOG("inc PWM %d->%d",ProcChimSR,pwm);
					if (ProcChimSR != pwm)	 setNewProcChimSR(pwm);
					//--- перезапускаем медленный ШИМ с новым % отбора
					start_valve_PWMpercent
					  ( klp_sr, // клапан продукта
						getFloatParam(DEFL_PARAMS, "timeChimRectOtbSR"),// период ШИМ в сек
						ProcChimSR //%
					  );
				}
				secTempPrev = uptime_counter;
			} //auto-increment block
		}

		// проверим - можно ли продолжать отбор тела?
		if (! end_condition_SR())	{
			break;  // продолжаем отбор тела
		}
		// условия завершения отбора продукта выполнены

		//------------завершение тела------
		set_status(PROC_HV);			// Переход к отбору хвостов
		 /* fall through */

	case PROC_HV:	// Отбор хвостовых фракций
		// старт
		if (prev_status!=MainStatus){
			LOG("PROC_TAIL (till T cube:%5.1f)",getFloatParam(DEFL_PARAMS, "tempEndRect"));
			if (prev_status!=PROC_SR) { // если не после отбора тела
				openKlp(klp_water);		// Открытие клапана воды
				set_proc_power("powerRect");
			}
			closeKlp(klp_sr); 			// Отключение клапана продукта
			start_valve_PWMpercent(klp_glwhq,
						getFloatParam(DEFL_PARAMS, "timeChimRectOtbGlv"),
						100);
			prev_status=MainStatus;
		}

		//-- контроль
		t = getCubeTemp();
		if (t < getFloatParam(DEFL_PARAMS, "tempEndRect")) {
			break;
		}
		// Температура достигла отметки окончания ректификации

		//-- завершение
		set_status(PROC_WAITEND); // на охлаждение
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		DBG("Switch state to wait End of Rectification."); // @suppress("No break at end of case")
		 /* fall through */

	case PROC_WAITEND:	// Отключение нагрева, подача воды для охлаждения

		#define END_OF_COOLDOWN_DELTA_T 6.0
		// старт
		if (prev_status!=MainStatus){
			setPower(0);		// Снятие мощности с тэна
			closeKlp(klp_glwhq); 	// Отключение клапана отбора голов/хвостов
			if (prev_status!=PROC_HV) { // если попали сюда не штатно, после отбора хвоста, а после рестарта то
				openKlp(klp_water);		// Открытие клапана воды
				closeKlp(klp_sr); 			// Отключение клапана продукта
			}
			tempTube20Prev = getTube20Temp(); // фиксируем текущую T низа колонны
			secTempPrev = uptime_counter; // и текущее время
			prev_status=MainStatus;
			if (tempTube20Prev>0){
				LOG("PROC_COOLDOWN (till Ttube20:%5.1f)",tempTube20Prev-END_OF_COOLDOWN_DELTA_T);
			}
			else {
				LOG("PROC_COOLDOWN (for 180 sec)");
			}
		}

		//  контроль завершения
		if (tempTube20Prev>0) { // датчик низа колонны исправен, контролируем завершение охлаждения по нему
			//ждем когда Тниза колонны снизится на заданное число градусов
			if (getTube20Temp() > (tempTube20Prev - END_OF_COOLDOWN_DELTA_T)) break;
		}
		else { // иначе контроль завершения по времени
			if ((uptime_counter - secTempPrev) < 180) break; //ждем 180 секунд
		}

		//  завершение
       	set_status(PROC_END);
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		break;

	case PROC_END:		// Окончание работы
		// старт
		if (prev_status!=MainStatus){
			LOG("END");
			prev_status=MainStatus;
			setPower(0);		// Снятие мощности с тэна
			closeAllKlp();		// Закрытие всех клапанов.
			SecondsEnd = uptime_counter;
			sprintf(b, "Rectification complete, time: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
			sendSMS(b);
			DBG("SMS:'%s'", b);
			if (getIntParam(DEFL_PARAMS, "DIFFoffOnStop")) {
				xTaskCreate(&diffOffTask, "diff Off task", 4096, NULL, 1, NULL); // выключаем дифф
			}
		}
		//  контроль
		break;  // ничего не делаем

	default:
		break;		// Окончание работы
	}
}


// Обработка состояний в режиме дистилляции
void Distillation(void)
{
	double t;
	char b[80];
	static int16_t prev_status=START_WAIT;

	switch (MainStatus) {
	case START_WAIT:
		if (prev_status!=MainStatus){
			prev_status=MainStatus;
			closeAllKlp();
			if (SetPower) setPower(0);
			startPressure = bmpTruePressure; // Фиксация атм. давления.
		}
		// Ожидание запуска процесса
		break;

	case PROC_START:		// Начало процесса
		set_status(PROC_RAZGON); // @suppress("No break at end of case")
		/* fall through */

	case PROC_RAZGON:		// Разгон до рабочей температуры
		if (prev_status!=MainStatus){
			prev_status=MainStatus;
			closeAllKlp();
			set_proc_power("maxPower");
		}

		t = getCubeTemp();
		if (-1 == t) { // сбой датчика
			ESP_LOGE(TAG, "error DS of CUB'");
			SET_ALARM(ALARM_SENSOR_ERR);
			break;
		}
		else {
			CLEAR_ALARM(ALARM_SENSOR_ERR);
		}

		if (t < getFloatParam(DEFL_PARAMS, "tempEndRectRazgon")) break;
		// go to next
		set_status(PROC_DISTILL); // @suppress("No break at end of case")
		 /* fall through */

	case PROC_DISTILL:		// Процесс дистилляции
		if (prev_status!=MainStatus){
			prev_status=MainStatus;
			closeKlp(klp_glwhq);
			openKlp(klp_water);
			openKlp(klp_sr);
			set_proc_power("powerDistil"); // Мощность дистилляции
		}

		t = getCubeTemp();
		if (t < getFloatParam(DEFL_PARAMS, "tempEndDistil")) {
			break;
		}

		//Завершение
		set_status(PROC_WAITEND);			// @suppress("No break at end of case")
		 /* fall through */

	case PROC_WAITEND:
		// Отключение нагрева, подача воды для охлаждения
		if (prev_status!=MainStatus){
			prev_status=MainStatus;
			closeKlp(klp_sr);
			openKlp(klp_water);
			setPower(0);		// Снятие мощности с тэна
			secTempPrev = uptime_counter;
		}

		if ((uptime_counter - secTempPrev) < 180) break;

		set_status(PROC_END); // @suppress("No break at end of case")

	case PROC_END:
		// Окончание работы
		if (prev_status!=MainStatus){
			SecondsEnd = uptime_counter;
			prev_status=MainStatus;
			setPower(0);		// Снятие мощности с тэна
			closeAllKlp();		// Закрытие всех клапанов.
			sprintf(b, "Distillation complete, time: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
			sendSMS(b);
			if (getIntParam(DEFL_PARAMS, "DIFFoffOnStop")) {
				xTaskCreate(&diffOffTask, "diff Off task", 4096, NULL, 1, NULL); // выключаем дифф
			}
		}
		break;
	}
}

/*
 * send command (ON/OFF) to valvePWMtask
 */
void cmd2valve (int valve_num, valve_cmd_t cmd){
	static valveCMDmessage_t cmd_message;
	DBGV("v:%d cmd:%d",valve_num, cmd);
	if (valve_num>=MAX_KLP) {
		ERR_MSG("incorrect valve num %d", valve_num);
		return;
	}
	if (valve_cmd_queue){
		cmd_message.cmd = cmd;
		cmd_message.valve_num=valve_num;
		if (xQueueSend( valve_cmd_queue, ( void * ) &cmd_message, ( TickType_t ) 10 )!= pdPASS){
			ERR_MSG("timeout of cmd sending");
		}
	}
	else {
		ERR_MSG("CMD queue doesn't exist");
	}
}

/*
 * Закрытие всех клапанов.
 */
void closeAllKlp(void)
{
	for (int i=0; i<MAX_KLP; i++)	closeKlp(i);
}

/*
 * Открытие клапана с выключением программного ШИМ
 */
void openKlp(int i)
{
	cmd2valve(i, cmd_open);
	Klp[i].is_pwm = false;
}

/*
 * Закрытие клапана с выключением программного ШИМ
 */
inline void closeKlp(int i)
{
	cmd2valve (i, cmd_close);
	Klp[i].is_pwm = false;
}

/*
 * Запуск шима клапана
 */
void startKlpPwm(int i, float topen, float tclose)
{
	if (i>=MAX_KLP) return;
	DBGV( "PWM klp %d %04.1f/%5.1f", i, topen, tclose);
	Klp[i].open_time = topen;	// Время в течении которого клапан открыт
	Klp[i].close_time = tclose;	// Время в течении которого клапан закрыт
	Klp[i].is_pwm = true;	// Запускаем медленный Шим режим

	if (! Klp[i].pwm_task_Handle){
		xTaskCreate(valvePWMtask, "valvePWMtask", 8192, (void *)i, 5, &Klp[i].pwm_task_Handle);	//запускаем задачу программного ШИМ клапана
		vTaskDelay( 100/portTICK_PERIOD_MS );
	}
	else
		vTaskResume( Klp[i].pwm_task_Handle);
}

/* Запуск программного ШИМ с параметрами
* @клапан
* @период в сек
* @процент времени открытия
*/
void start_valve_PWMpercent(int valve_num, int period_sec, int percent_open){
	float topened= (period_sec*percent_open+50)/100l;
	float tclosed= period_sec-topened;
	if ((topened < 0)||(topened<0)||(period_sec==0)){
		DBGV("incorrect param,period %d open %05.2f close %05.2f", period_sec, topened, tclosed);
		return;
	}
	startKlpPwm(valve_num, topened, tclosed);
}

static struct {
    struct arg_str *value;
    struct arg_end *end;
} set_args;

static int set_ct(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &set_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, set_args.end, argv[0]);
		return 1;
	}
	const char *values = set_args.value->sval[0];
	testCubeTemp = atof(values);
	emulate_devices = 1;
	DBG("New Cube temp: %f\n", testCubeTemp);
	return 0;
}

const esp_console_cmd_t set_ct_cmd = {
        .command = "t",
        .help = "Emulate cube temp",
        .hint = NULL,
        .func = &set_ct,
        .argtable = &set_args
};

/* 'version' command */
static int get_version(int argc, char **argv)
{
	esp_chip_info_t info;
	esp_chip_info(&info);
	printf("IDF Version:%s\r\n", esp_get_idf_version());
	printf("Chip info:\r\n");
	printf("\tmodel:%s\r\n", info.model == CHIP_ESP32 ? "ESP32" : "Unknow");
	printf("\tcores:%d\r\n", info.cores);
	printf("\tfeature:%s%s%s%s%d%s\r\n",
           info.features & CHIP_FEATURE_WIFI_BGN ? "/802.11bgn" : "",
           info.features & CHIP_FEATURE_BLE ? "/BLE" : "",
           info.features & CHIP_FEATURE_BT ? "/BT" : "",
           info.features & CHIP_FEATURE_EMB_FLASH ? "/Embedded-Flash:" : "/External-Flash:",
           spi_flash_get_chip_size() / (1024 * 1024), " MB");
	printf("\trevision number:%d\r\n", info.revision);
	return 0;
}

static void register_version()
{
    const esp_console_cmd_t cmd = {
        .command = "version",
        .help = "Get version of chip and SDK",
        .hint = NULL,
        .func = &get_version,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/** 'restart' command restarts the program */

static int restart(int argc, char **argv)
{
	LOG("Restarting");
	esp_restart();
}

static void register_restart()
{
	const esp_console_cmd_t cmd = {
		.command = "restart",
		.help = "Software reset of the chip",
		.hint = NULL,
		.func = &restart,
	};
	ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}



void console_task(void *arg)
{
	const char* prompt = LOG_COLOR_I "hd> " LOG_RESET_COLOR;



	/* Выключаем буферизацию stdin и stdout */
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	// Настройка консоли
	esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
	esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
	ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
	// Tell VFS to use UART driver
	esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
	// Инициализация консоли
	esp_console_config_t console_config = {
		.max_cmdline_args = 8,
		.max_cmdline_length = 256,
		.hint_color = atoi(LOG_COLOR_CYAN)
	};
	ESP_ERROR_CHECK(esp_console_init(&console_config));

	linenoiseSetMultiLine(1);
	linenoiseSetCompletionCallback(&esp_console_get_completion);
	linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);
	linenoiseHistorySetMaxLen(100);
	linenoiseHistoryLoad(HISTORY_PATH);
	/* Register commands */
	esp_console_register_help_command();

	int probe_status = linenoiseProbe();
	if (probe_status) {
	        linenoiseSetDumbMode(1);
	}

	set_args.value = arg_str1("v", "value", "<value>", "value to be stored");
	set_args.end = arg_end(2);
	ESP_ERROR_CHECK( esp_console_cmd_register(&set_ct_cmd) );
	register_version();
	register_restart();

	while (true) {
		char* line;
		int ret;
		line = linenoise(prompt);
		if (line) {
			int res;
			linenoiseHistoryAdd(line); // add to history
			linenoiseHistorySave(HISTORY_PATH); // save history
			ret = esp_console_run(line, &res);
			if (ret == ESP_ERR_NOT_FOUND) {
				printf("Unrecognized command\n");
			} else if (ret == ESP_OK && res != ESP_OK) {
				printf("Command returned non-zero error code: 0x%x\n", res);
			} else if (ret != ESP_OK) {
				printf("Internal error: 0x%x\n", ret);
			}
			linenoiseFree(line); // free line
		}
	}

}

// Основная точка входа в программу
void app_main(void)
{
	gpio_config_t io_conf;
	esp_err_t ret;

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	DBG("RAM left %d", esp_get_free_heap_size());


	ESP_ERROR_CHECK(ret = nvs_open("storage", NVS_READWRITE, &nvsHandle));
	if (ret != ESP_OK) nvsHandle = 0;

	// I2C init
	I2C_Init(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
	task_i2cscanner();

	DBG("Initializing SPIFFS");
	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/s",
		.partition_label = NULL,
		.max_files = 5,
		.format_if_mount_failed = true
	};
	ret = esp_vfs_spiffs_register(&conf);
	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ERR_MSG("Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ERR_MSG("Failed to find SPIFFS partition");
		} else {
			ERR_MSG("Failed to initialize SPIFFS (%d)", ret);
		}
		return;
	}

	// Получение причины (пере)загрузки
	resetReason = rtc_get_reset_reason(0);
	DBG("Reset reason: %s\n", getResetReasonStr());


	TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
	TIMERG0.wdt_feed=1;
	TIMERG0.wdt_wprotect=0;

	/* Запуск консоли */
	xTaskCreate(&console_task, "console_task", 8192, NULL, 1, NULL);

	/* Получаем конфигурацию сетевого доступа */
	param_load(NET_PARAMS, NET_CONFIGURATION);
	Hostname = getStringParam(NET_PARAMS, "host");
	httpUser = getStringParam(NET_PARAMS, "user");
	httpPassword = getStringParam(NET_PARAMS, "pass");
	httpSecure = getIntParam(NET_PARAMS, "secure");
	wsPeriod = getIntParam(NET_PARAMS, "wsPeriod");

	setTimezone(getIntParam(NET_PARAMS, "timezone"));

	/* Настройка wifi */
	wifiSetup();

	/* Чтение настроек */
	paramSetup();

	/* Поиск и настройка датиков температуры и периодический опрос их */
	xTaskCreate(&ds_task, "ds_task", 4096, NULL, 1, NULL);

	/* Инициализация датчика давления bmp 180 */
	initBMP085();


	if (! getIntParam(DEFL_PARAMS, "no_power")){ //--- если используем регулятор-стабилизатор
		/* Настройка gpio детектора нуля сетевого напряжения */
		gpio_set_direction(GPIO_DETECT_ZERO, GPIO_MODE_INPUT);
		gpio_set_intr_type(GPIO_DETECT_ZERO, GPIO_INTR_NEGEDGE);
		gpio_set_pull_mode(GPIO_DETECT_ZERO, GPIO_PULLUP_ONLY);
	}

	if (getIntParam(DEFL_PARAMS, "useExernalAlarm")) {
		/* Настройка gpio внешнего аварийного детектора */
		gpio_set_direction(GPIO_ALARM, GPIO_MODE_INPUT);
		gpio_set_intr_type(GPIO_ALARM, GPIO_INTR_NEGEDGE);
		gpio_set_pull_mode(GPIO_ALARM, GPIO_PULLUP_ONLY);
	}

	/* Настройка прерываний */
	ESP_ERROR_CHECK(gpio_isr_register(gpio_isr_handler, NULL, ESP_INTR_FLAG_LEVEL2, NULL));

	// Configure output gpio
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_OUTPUT_BEEPER;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	/* Запуск http сервера */
	hd_httpd_init();	

	/* Настройка таймера */
	tg0_timer0_init();

	/* Запуск отображения на дисплее */
	hd_display_init();

	//--- регулятор-стабилизатор
	if (! getIntParam(DEFL_PARAMS, "no_power")){ //--если его используем
		/* PZEM */
		xTaskCreate(&pzem_task, "pzem_task", 2048*4, NULL, 1, NULL);
		// ШИМ управления симистором
		ledc_timer_config_t ledc_timer = {
	#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
			.duty_resolution = LEDC_TIMER_10_BIT,
	#else
			.bit_num = LEDC_TIMER_10_BIT,
	#endif
			.freq_hz = TRIAC_CONTROL_LED_FREQ_HZ * 2,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.timer_num = LEDC_TIMER_0
		};
		ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
	    ledc_channel_config_t ledc_channel = {
	            .gpio_num = GPIO_TRIAC,
	            .speed_mode = LEDC_HIGH_SPEED_MODE,
	            .channel = 0,
	            .timer_sel = LEDC_TIMER_0,
	            .duty = (1 << LEDC_TIMER_10_BIT) - 1,
	            .intr_type = LEDC_INTR_DISABLE
		};
		ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

		LEDC.channel_group[0].channel[0].duty.duty = TRIAC_GATE_IMPULSE_CYCLES << 4;
	        // Initial brightness of 0, meaning turn TRIAC on at very end:
	    LEDC.channel_group[0].channel[0].conf0.sig_out_en = 0;
		LEDC.channel_group[0].channel[0].conf1.duty_start = 0;
	}

	// valves control
	valve_cmd_queue = xQueueCreate(10, sizeof(valveCMDmessage_t));			//---очередь команд открытия/закрытия клапанов
	if (! valve_cmd_queue){
		ERR_MSG("error of QUEUE creating!");
	}
	else {
		xTaskCreate(valveCMDtask, "valveCMDtask", 8192, NULL, 5, NULL);	//---задача включения/выключения клапанов по командам
	}

	/* задача контроля флагов тревоги, звук при аварии, выключения дифф-автомата*/
	xTaskCreate(&alarmControlTask, "alarmControl", 4096, NULL, 1, NULL);

	if (getIntParam(DEFL_PARAMS, "DIFFoffOnStart")) {// при настройке "выключать дифф при старте"
		xTaskCreate(&diffOffTask, "diff Off task", 4096, NULL, 1, NULL); // выключаем дифф
	}

	ESP_ERROR_CHECK(gpio_intr_enable(GPIO_DETECT_ZERO));
	DBG("Enabled zero crossing interrupt");

	if (getIntParam(DEFL_PARAMS, "useExernalAlarm")) {
		ESP_ERROR_CHECK(gpio_intr_enable(GPIO_ALARM));
	}

	restoreProcess();

	if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(false);

#ifdef DEBUG1
	emulateT(DS_CUB, 80.0);
	emulateT(DS_TUBE20, 45.0);
#endif

	while (true) {
		cJSON *ja = getInformation();
     		char *r=cJSON_Print(ja);
		cgiWebsockBroadcast("/ws", r, strlen(r), WEBSOCK_FLAG_NONE);
		cJSON_Delete(ja);
		if (r) free(r);
		if (MODE_RECTIFICATION == MainMode) Rectification();
		else if (MODE_DISTIL == MainMode) Distillation();
		vTaskDelay(wsPeriod*1000/portTICK_PERIOD_MS);
	}
}

void valveCMDtask(void *arg){
	valveCMDmessage_t qcmd;
	ledc_channel_t ch;
	TickType_t xLastWakeTime=0;
#ifdef DEBUGV
	TickType_t prevValveSwitch=0;
#endif

	// LEDC
	ledc_timer_config_t ledc_timer1 = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
		.duty_resolution = LEDC_TIMER_10_BIT,
#else
		.bit_num = LEDC_TIMER_10_BIT,
#endif
		.freq_hz = VALVES_CONTROL_LED_FREQ_HZ,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_num = LEDC_TIMER_1
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));
	// Настройка клапанов управления
	for (int i=0; i<MAX_KLP; i++) {
		ledc_channel_config_t ledc_channel = {
			.gpio_num = klp_gpio[i],
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.channel = i+1,
			.timer_sel = LEDC_TIMER_1,
			.duty = 0,
			.intr_type = LEDC_INTR_DISABLE,
		};
		Klp[i].is_open = false;
		Klp[i].channel = i+1; // +1 т.к. нулевой канал зарезервирован под регулятор мощности
		Klp[i].pwm_task_Handle=NULL;
		ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
		LEDC.channel_group[0].channel[i+1].conf0.sig_out_en = 0;
	}
	ledc_fade_func_install(0);

	while (1){
		if (xQueueReceive(valve_cmd_queue, &qcmd, portMAX_DELAY)!=pdTRUE) // ждем события на открытие/закрытие клапана
			continue;																									// если таймаут - повторим
		ch = Klp[qcmd.valve_num].channel;															// ledc-канал  клапана
		LEDC.channel_group[0].channel[ch].conf0.sig_out_en = 1;
		DBGV("v:%d(ch:%d) cmd:%d",qcmd.valve_num,ch,qcmd.cmd);
		switch (qcmd.cmd) {
			case cmd_open:
				if (! Klp[qcmd.valve_num].is_open) { // если клапан закрыт то открываем
					// -------логика "тихого" включения
					if (getIntParam(DEFL_PARAMS,"klpSilentNode")) {
						ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, ch, VALVE_DUTY, VALVE_ON_FADE_TIME_MS);
						ledc_fade_start(LEDC_HIGH_SPEED_MODE, ch, LEDC_FADE_NO_WAIT);
						vTaskDelay(VALVE_ON_FADE_TIME_MS/portTICK_PERIOD_MS);
					} else {
						ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, VALVE_DUTY);
						ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
					}
					xLastWakeTime = xTaskGetTickCount ();// системное время включения, в тиках
	#ifdef DEBUGV
					DBGV(" ON:%d(%d ms)",qcmd.valve_num, (xLastWakeTime-prevValveSwitch)*portTICK_PERIOD_MS );
					prevValveSwitch=xLastWakeTime;
	#endif
					Klp[qcmd.valve_num].is_open = true;
					// ---------логика снижения ШИМ клапана после его включения---------
					if (((qcmd.valve_num == klp_water)&&(getIntParam(DEFL_PARAMS,"klp1_isPWM"))) //если не клапан а насос, то не снижаем
						||
						 (qcmd.valve_num == klp_diff) // если это ключ выхода на дифф-автомат то не снижаем
						) break;

					if ((KEEP_KLP_PWM==0)||(KEEP_KLP_PWM==100)) break;								// если настройка ШИМ удержания 0 или 100 - не снижаем
					if (	(Klp[qcmd.valve_num].is_pwm)																//если клапан в ШИМ
							&&																										//и время открытого его состояния
							(KEEP_KLP_DELAY_MS >= (Klp[qcmd.valve_num].open_time*1000))			//меньше времени задержки до перехода на удержание
						)	{																											// то не снижаем
						break;
					}

					vTaskDelayUntil( &xLastWakeTime, KEEP_KLP_DELAY_MS/portTICK_PERIOD_MS );//ждем включения механики клапана
					ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, ((VALVE_DUTY*KEEP_KLP_PWM)/100ul));
					ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
				}
				else {
					DBGV("ON ignored");
				}
				break;

			case cmd_close:
				if (Klp[qcmd.valve_num].is_open){ // закрываем если открыт
#ifdef DEBUGV
					xLastWakeTime = xTaskGetTickCount ();
					DBGV("OFF:%d(%d ms)", qcmd.valve_num, (xLastWakeTime-prevValveSwitch)*portTICK_PERIOD_MS);
					prevValveSwitch =xLastWakeTime;
#endif
					ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, 0);
					ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
					LEDC.channel_group[0].channel[ch].conf0.sig_out_en = 0;
					Klp[qcmd.valve_num].is_open = false;
				}
				else {
					DBGV("cmd close ignored");
				}
				break;
			default:
				break;
		}
	}
}

void setTimezone(int gmt_offset){
	char tz[10];
	snprintf(tz, 10, "CST-%d", gmt_offset);
	setenv("TZ", tz, 1);
	tzset();
}

void restoreProcess(void) {
	int16_t pw;
	if (!nvsHandle) {
		ERR_MSG("no NVS handle, process isn't restored");
		return;
	}
	nvs_get_i16(nvsHandle, "MainMode", (int16_t*)&MainMode);
	nvs_get_i16(nvsHandle, "SetPower", &pw);
	nvs_get_i16(nvsHandle, "MainStatus", &MainStatus);
	DBG("MMode:%d State:%d Power:%d",MainMode,MainStatus,pw);
	switch (MainMode) {
		case  MODE_RECTIFICATION:
		case  MODE_DISTIL:
			if ( (MainStatus>PROC_START) && (MainStatus<PROC_END)){
				broken_proc = true;
				ESP_ERROR_CHECK(nvs_get_u64(nvsHandle, "tempStabSR", (uint64_t*)&tempStabSR));
				nvs_get_u64(nvsHandle, "tempTube20Prev", (uint64_t*)&tempTube20Prev);
				nvs_get_i16(nvsHandle, "ProcChimSR", &ProcChimSR);
				DBG("broken proc=true  Tstab:%02.1f Tprev:%02.1f ProcPWM:%d",tempStabSR,tempTube20Prev,ProcChimSR);
			} // @suppress("No break at end of case")
		case  MODE_POWEERREG:
			  setPower(pw);
			  break;
		default:
			break;
	}
}

void write2log(const char* s){
	cJSON *ja;
	char data[80];

	ja = cJSON_CreateObject();
	cJSON_AddItemToObject(ja, "cmd", cJSON_CreateString("logline"));
	cJSON_AddItemToObject(ja, "ch", cJSON_CreateString(""));
	cJSON_AddItemToObject(ja, "level", cJSON_CreateString(""));

	time_t CurrentTime=time(NULL);
	struct tm CurrentTm;
	localtime_r(&CurrentTime, &CurrentTm);
	snprintf(data, sizeof(data)-1, "%02d-%02d-%d %02d:%02d:%02d", CurrentTm.tm_mday,CurrentTm.tm_mon+1,CurrentTm.tm_year+1900, CurrentTm.tm_hour, CurrentTm.tm_min,CurrentTm.tm_sec);
	cJSON_AddItemToObject(ja, "date", cJSON_CreateString(data));
	cJSON_AddItemToObject(ja, "message", cJSON_CreateString(s));

 	char *r=cJSON_Print(ja);
	cgiWebsockBroadcast("/ws", r, strlen(r), WEBSOCK_FLAG_NONE);
	cJSON_Delete(ja);
	if (r) free(r);
}
