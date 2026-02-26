/*
 * ds3231.h
 *
 *  Created on: Jan 21, 2026
 *      Author: John
 */

#ifndef DS3231_H_
#define DS3231_H_

#include "stm32f103xx.h"

//Configuration Items
#define DS3231_GPIO_PORT		GPIOB
#define DS3231_GPIO_SDA_PIN		PIN_NUM_7
#define DS3231_GPIO_SCL_PIN		PIN_NUM_6
#define DS3231_I2C				I2C1
#define DS3231_I2C_SCL_SPEED	I2C_SCL_SPEED_SM

//Register Addresses

#define DS3231_ADDR_SEC 		0x00
#define DS3231_ADDR_MIN 		0x01
#define DS3231_ADDR_HOUR 		0x02
#define DS3231_ADDR_DAY 		0x03
#define DS3231_ADDR_DATE 		0x04
#define DS3231_ADDR_MONTH 		0x05
#define DS3231_ADDR_YEAR 		0X06

#define TIME_FORMAT_12HR_AM		0
#define TIME_FORMAT_12HR_PM		1
#define TIME_FORMAT_24HR		2

#define DS3231_SLAVE_ADDR		0x68U

#define SUNDAY					1
#define MONDAY					2
#define TUESDAY					3
#define WEDNESDAY				4
#define THURSDAY				5
#define FRIDAY					6
#define SATURDAY				7

typedef struct
{
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
}RTC_date_t;


typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
}RTC_time_t;

//Function prototypes

void ds3231_init(void);

void ds3231_set_current_time(RTC_time_t *);
void ds3231_get_current_time(RTC_time_t *);

void ds3231_set_current_date(RTC_date_t *);
void ds3231_get_current_date(RTC_date_t *);



#endif /* DS3231_H_ */
