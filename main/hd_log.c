/*
 * hd_log.c
 *
 *  Created on: 16 θών. 2022 γ.
 *      Author: Vladimir Loskutnikov
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "freertos/FreeRTOS.h"
#include <cJSON.h>
#include "esp_platform.h"
#include "cgiwebsocket.h"
#include "hd_log.h"

void ws_log2WEB(const char* s){
	cJSON *ja;
	ja = cJSON_CreateObject();
	cJSON_AddItemToObject(ja, "cmd", cJSON_CreateString("logline"));
	cJSON_AddItemToObject(ja, "ch", cJSON_CreateString(""));
	cJSON_AddItemToObject(ja, "level", cJSON_CreateString(""));
	cJSON_AddItemToObject(ja, "message", cJSON_CreateString(s));

 	char *r=cJSON_Print(ja);
	cgiWebsockBroadcast("/ws", r, strlen(r), WEBSOCK_FLAG_NONE);
	cJSON_Delete(ja);
	if (r) free(r);
}

