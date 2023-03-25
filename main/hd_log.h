/*
 * hd_log.h
 *
 *  Created on: 16 θών. 2022 γ.
 *      Author: Vladimir Loskutnikov
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#ifndef HD_LOG_H_
#define HD_LOG_H_

//#define NO_LOG

#ifndef NO_LOG
#define LOG( format, ... ) do { \
		ESP_LOGI(__func__, format, ##__VA_ARGS__); \
		char data[80]; \
		snprintf(data, sizeof(data)-1, format, ##__VA_ARGS__); \
		ws_log2WEB(data);\
	} while(0)
#else
#define LOG( format, ... ) do { \
		ESP_LOGI(__func__, format, ##__VA_ARGS__); \
	} while(0)
#endif

#define ERR_MSG( format, ... ) ESP_LOGE(__func__, format, ##__VA_ARGS__)
#define LOG_MSG( format, ... ) ESP_LOGI(__func__, format, ##__VA_ARGS__)

void ws_log2WEB(const char* s);

#endif /* HD_LOG_H_ */
