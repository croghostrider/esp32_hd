/*
 * sms.h
 *
 *  Created on: 13 мая 2022 г.
 *  Author: Vladimir Loskutnikov
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#ifndef MAIN_SMS_H_
#define MAIN_SMS_H_

void send_message(const char *text);

void sendSMS(const char *text);
esp_err_t send2Telegram(const char *text);
esp_err_t send_message2bot(const char *token,const char *chat_id,const char *text);


#endif /* MAIN_SMS_H_ */
