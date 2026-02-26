/*
 * ds3231.c
 *
 *  Created on: Jan 21, 2026
 *      Author: John
 */

#include "ds3231.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_i2c.h"
#include <stdint.h>
#include <string.h>

#define MY_ADDR 0x52

static void ds3231_gpio_config(void);
static void ds3231_i2c_config(void);
static void ds3231_write(uint8_t regaddr, uint8_t value);
static uint8_t ds3231_read(uint8_t regaddr);
static uint8_t bcd_to_binary(uint8_t bcd_data);
static uint8_t binary_to_bcd(uint8_t binary_data);

I2C_Handle_t pi2c;

void ds3231_init(void)
{
	// Enable I2C peripheral clock
	I2C_PeriClockControl(I2C1, ENABLE);

	//1. gpio pin config
	ds3231_gpio_config();

	//2. i2c configuration
	ds3231_i2c_config();

}

void ds3231_set_current_time(RTC_time_t *rtctime)
{
	uint8_t second = binary_to_bcd(rtctime->seconds);
	uint8_t minute = binary_to_bcd(rtctime->minutes);
	uint8_t hour = binary_to_bcd(rtctime->hours);

	ds3231_write(DS3231_ADDR_SEC, second);
	ds3231_write(DS3231_ADDR_MIN, minute);
	if (rtctime->time_format == TIME_FORMAT_24HR)
	{
		hour &= ~(1<<6);
	}else
	{
		hour |= (1<<6);
		hour = (rtctime->time_format  == TIME_FORMAT_12HR_PM) ? hour | ( 1 << 5) :  hour & ~( 1 << 5) ;
	}
	ds3231_write(DS3231_ADDR_HOUR, hour);
}
void ds3231_get_current_time(RTC_time_t *rtctime)
{
	uint8_t hour;
	uint8_t tf_bit = ( 1 << 6 );
	uint8_t am_pm_bit = (1 << 5);


	rtctime->seconds = bcd_to_binary(ds3231_read(DS3231_ADDR_SEC));
	rtctime->minutes = bcd_to_binary(ds3231_read(DS3231_ADDR_MIN));

	hour = ds3231_read(DS3231_ADDR_HOUR);
	if( hour & tf_bit)
	{
		// 12-hour time format
		if( hour & am_pm_bit)
		{
			// PM
			rtctime->time_format = TIME_FORMAT_12HR_PM;

		}else
		{
			// AM
			rtctime->time_format = TIME_FORMAT_12HR_AM;

		}
		rtctime->hours = bcd_to_binary(hour & 0x1F);
	}else
	{
		// 24hour time-format
		rtctime->time_format = TIME_FORMAT_24HR;
		rtctime->hours = bcd_to_binary(hour & 0x3F);
	}

}

void ds3231_set_current_date(RTC_date_t *rtcdate)
{
	uint8_t day = binary_to_bcd(rtcdate->day);
	uint8_t date = binary_to_bcd(rtcdate->date);
	uint8_t month = binary_to_bcd(rtcdate->month);
	uint8_t year = binary_to_bcd(rtcdate->year);

	ds3231_write(DS3231_ADDR_DAY, day);
	ds3231_write(DS3231_ADDR_DATE, date);
	ds3231_write(DS3231_ADDR_MONTH, month);
	ds3231_write(DS3231_ADDR_YEAR, year);
}
void ds3231_get_current_date(RTC_date_t *rtcdate)
{
	rtcdate->day = bcd_to_binary(ds3231_read(DS3231_ADDR_DAY));
	rtcdate->date = bcd_to_binary(ds3231_read(DS3231_ADDR_DATE));
	rtcdate->month = bcd_to_binary(ds3231_read(DS3231_ADDR_MONTH) & 0x1F);
	rtcdate->year = bcd_to_binary(ds3231_read(DS3231_ADDR_YEAR));

}

static void ds3231_gpio_config(void)
{
	/*
	 * SCL -> PB6
	 * SDA -> PB7
	 */

	GPIO_Handle_t gpio_sda, gpio_scl;
	memset(&gpio_sda,0,sizeof(gpio_sda));
	memset(&gpio_scl,0,sizeof(gpio_scl));

	// SCL pin GPIO config.
	gpio_scl.pGPIOx = DS3231_GPIO_PORT;
	gpio_scl.GPIO_PinConfig.GPIO_PinNumber = DS3231_GPIO_SCL_PIN;
	gpio_scl.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	gpio_scl.GPIO_PinConfig.GPIO_OPMode = OP_AF_OPDR;
	GPIO_Init(&gpio_scl);

	// SDA pin GPIO config.
	gpio_sda.pGPIOx = DS3231_GPIO_PORT;
	gpio_sda.GPIO_PinConfig.GPIO_PinNumber = DS3231_GPIO_SDA_PIN;
	gpio_sda.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	gpio_sda.GPIO_PinConfig.GPIO_OPMode = OP_AF_OPDR;
	GPIO_Init(&gpio_sda);



}

static void ds3231_i2c_config(void)
{
	// SDA pin I2C config.

	memset(&pi2c, 0, sizeof(pi2c));

	pi2c.pI2Cx = DS3231_I2C;
	pi2c.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	pi2c.I2C_Config.I2C_SCLSpeed = DS3231_I2C_SCL_SPEED;
	pi2c.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	pi2c.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C_Init(&pi2c);

}

static void ds3231_write(uint8_t regaddr, uint8_t value)
{
	uint8_t tx[2];
	tx[0] = regaddr;
	tx[1] = value;

	I2C_MasterSendData(&pi2c, tx, 2, DS3231_SLAVE_ADDR, 0);
}

static uint8_t ds3231_read(uint8_t regaddr)
{
	uint8_t rx=0;

	I2C_MasterSendData(&pi2c, &regaddr, 1, DS3231_SLAVE_ADDR, 0);
	I2C_MasterReceiveData(&pi2c, &rx, 1, DS3231_SLAVE_ADDR, 0);

	return rx;
}

static uint8_t bcd_to_binary(uint8_t bcd_data) {
    return ((bcd_data >> 4) * 10) + (bcd_data & 0x0F);
}

static uint8_t binary_to_bcd(uint8_t binary_data)
{
	uint8_t bcd_data = 0;

	bcd_data |= binary_data % 10;
	bcd_data |= ((binary_data / 10) << 4);

	return bcd_data;
}






