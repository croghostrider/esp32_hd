/*
 * sms.c
 *
 *  Created on: 13 мая 2022 г.
 *      Author: Vladimir Loskutnikov
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "esp_tls.h"
#include "esp_http_client.h"

#include "debug.h"
#include "config.h"
#include "sms.h"

typedef union {
struct {
	int inited:1;
	int useTLG:1;
	int useSMS:1;
};
int flag_int;
}  msg_flags_t;

static  msg_flags_t flags;
static  char* tokenTlg=NULL;
static  char* chatidTlg=NULL;

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            //DBG("HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            //DBG("HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
        	//DBG("HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
        	//DBG("HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
        	//DBG("HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                // printf("%.*s", evt->data_len, (char*)evt->data);
            }
            break;
        case HTTP_EVENT_ON_FINISH:
        	//DBG( "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
        	;//DBG("HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            break;
    }
    return ESP_OK;
}

esp_err_t send_message2bot(const char *token,const char *chat_id,const char *text) {

	if ((!chat_id)||(!token)) return ESP_ERR_INVALID_ARG;

	esp_http_client_handle_t client;
    esp_err_t err;
    int len;

    esp_http_client_config_t config = {
        .url = "https://api.telegram.org",
        .event_handler = _http_event_handler,
		.method = HTTP_METHOD_POST,
        //.is_async = true,
        .timeout_ms = 5000,
    };
    client = esp_http_client_init(&config);
    if (! client) 	return ESP_FAIL;

    char url[strlen(token) + 50];
	sprintf(url, "https://api.telegram.org/bot%s/sendmessage", token);
    if (esp_http_client_set_url(client, url)!=ESP_OK) return ESP_FAIL;
    DBG("URL: %s",url);

	char* post = malloc(strlen(chat_id) + strlen(text) +20);
	if (!post) return ESP_ERR_NO_MEM;
	sprintf(post, "chat_id=%s&text=%s", chat_id, text);
    esp_http_client_set_post_field(client, post, strlen(post));
    DBG(" post:%s",post);

    err = esp_http_client_perform(client);
    if (err)  	{
    	DBG("error:%d",err);
    }
    else {
    	//esp_http_client_get_status_code(esp_http_client_handle_t client);
    	len = esp_http_client_get_content_length(client);
    	char* buf = malloc(len+1);
    	len = esp_http_client_read(client, buf, len);
    	DBG("resp: %s",buf);
    	free(buf);
    }

    free(post);
    esp_http_client_cleanup(client);
    return err;
}

esp_err_t send2Telegram(const char *text){
	return send_message2bot(
			getStringParam(NET_PARAMS, "tokenTlg"),
			getStringParam(NET_PARAMS, "chatidTlg"),
			text);
}

void send_message(const char *text){
	if (! flags.inited) {
		DBG("init");
		flags.inited=1;
		flags.useTLG = getIntParam(NET_PARAMS, "useTlg");
		flags.useSMS = getIntParam(NET_PARAMS, "useSmsc");
		if (flags.useTLG) {
			tokenTlg=getStringParam(NET_PARAMS, "tokenTlg");
			chatidTlg=getStringParam(NET_PARAMS, "chatidTlg");
		}
	}
	if (flags.useTLG) send_message2bot(tokenTlg, chatidTlg, text);
	if (flags.useSMS) sendSMS(text);
}

// Отправка SMS
void sendSMS(const char *text){
	char *post, *user, *hash, *phones;

	if (!getIntParam(NET_PARAMS, "useSmsc")) return;

	user = getStringParam(NET_PARAMS, "smscUser");
	hash = getStringParam(NET_PARAMS, "smscHash");
	phones = getStringParam(NET_PARAMS, "smscPhones");


	if (!user || !hash || !phones) return;
	if (strlen(user)<=0 || strlen(hash)<=30 || strlen(phones)<=0) return;

	esp_http_client_handle_t client;
    esp_err_t err;

    esp_http_client_config_t config = {
        .url = "https://smsc.ru/sys/send.php",
        .event_handler = _http_event_handler,
		.method = HTTP_METHOD_POST,
        //.is_async = true,
        .timeout_ms = 5000,
    };
    client = esp_http_client_init(&config);
    if (! client) 	return;

	int s = strlen(user) + strlen(hash) + strlen(phones) + strlen(text);
	post = malloc(s+30);
	if (!post) return;
	DBG(">>SMS start:%s",post);
	sprintf(post, "login=%s&psw=%s&phones=%s&mes=%s", user, hash, phones, text);
    esp_http_client_set_post_field(client, post, strlen(post));
    err = esp_http_client_perform(client);
	if (err) {
		DBG("sms failed, error code: %d", err);
	}
    free(post);
    esp_http_client_cleanup(client);
	DBG("<< Sms Done");
}
