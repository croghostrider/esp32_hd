/*
 * debug.h
 *
 *  Created on: 9 мар. 2022 г.
 *      Author: Vladimir Loskutnikov
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#ifndef MAIN_DEBUG_H_
#define MAIN_DEBUG_H_

#ifdef DEBUG
#define DBGT( tag, format, ... ) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define DBG( format, ... ) ESP_LOGI(__func__, format, ##__VA_ARGS__)
#else
#define DBGT( tag, format, ... )
#define DBG( format, ... )
#endif


#endif /* MAIN_DEBUG_H_ */
