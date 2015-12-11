/* Author: Domen Puncer <domen@cba.si>.  License: WTFPL, see file LICENSE */
#include <stdio.h>
#include <stdint.h>
#include "lcd.h"
#include "board.h"
//#include "retarget.h"
#include "gpio_17xx_40xx.h"
#include "cmsis.h"

void _delay_ms (unsigned short ms)
{
 uint16_t delay;
 volatile uint32_t i;
 for (delay = ms; delay >0 ; delay--)
//1ms loop with -Os optimisation
  {
  for (i=35; i >0;i--){};
  }
}

/* there are actually read commands/data too, but we won't use that.. for now... maybe BF is important */

static void lcd_put(struct hd44780_data *lcd, int rs, int data,uint8_t GpioNum)
{
	//gpio_set(lcd->pins.rs, rs);
	//Chip_GPIO_SetPinOutHigh();
	Chip_GPIO_SetValue(LPC_GPIO,GpioNum,lcd->pins.rs);
	Chip_GPIO_SetPortValue(LPC_GPIO,GpioNum,rs);

	//gpio_set(lcd->pins.rw, 0);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO,GpioNum,lcd->pins.rw);
	Chip_GPIO_SetPortValue(LPC_GPIO,GpioNum,0);
	asm("nop");
	_delay_ms(lcd->Te);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO,GpioNum,lcd->pins.e);
	Chip_GPIO_SetPortValue(LPC_GPIO,GpioNum,1);

	//gpio_set(lcd->pins.e, 1);
	//udelay(lcd->Te);
	_delay_ms(lcd->Te);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO,GpioNum,lcd->pins.d4);
	Chip_GPIO_SetPortValue(LPC_GPIO,GpioNum,data & 1);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO,GpioNum,lcd->pins.d6);
	Chip_GPIO_SetPortValue(LPC_GPIO,GpioNum,data>>2 & 1);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO,GpioNum,lcd->pins.d7);
	Chip_GPIO_SetPortValue(LPC_GPIO,GpioNum,data>>3 & 1);
	Chip_GPIO_SetPinOutLow(LPC_GPIO,GpioNum,lcd->pins.e);
#if 0
	gpio_set(lcd->pins.d5, data>>1 & 1);
	gpio_set(lcd->pins.d6, data>>2 & 1);
	gpio_set(lcd->pins.d7, data>>3 & 1);
	gpio_set(lcd->pins.e, 0);
#endif
}

int lcd_read(struct hd44780_data *lcd, int rs,uint8_t Num)
{
	int tmp;
	Chip_GPIO_SetPinDIRInput(LPC_GPIO,Num,lcd->pins.d4);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO,Num,lcd->pins.d5);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO,Num,lcd->pins.d6);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO,Num,lcd->pins.d7);
#if 0
	gpio_init(lcd->pins.d4, GPIO_INPUT, 0);
	gpio_init(lcd->pins.d5, GPIO_INPUT, 0);
	gpio_init(lcd->pins.d6, GPIO_INPUT, 0);
	gpio_init(lcd->pins.d7, GPIO_INPUT, 0);
#endif
	Chip_GPIO_SetPinOutHigh(LPC_GPIO,Num,lcd->pins.rs);
	Chip_GPIO_SetPortValue(LPC_GPIO,Num,rs);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO,Num,lcd->pins.rw);
	Chip_GPIO_SetPortValue(LPC_GPIO,Num,1);
#if 0
	gpio_set(lcd->pins.rs, rs);
	gpio_set(lcd->pins.rw, 1);
#endif
	//udelay(lcd->Te);
	_delay_ms(lcd->Te);
	//gpio_set(lcd->pins.e, 1);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO,3,lcd->pins.e);
	Chip_GPIO_SetPortValue(LPC_GPIO,3,1);
	//GPIO_SetValue(3,1);
	//udelay(lcd->Te);
	_delay_ms(lcd->Te);
	tmp = 0;
	tmp |=Chip_GPIO_GetPinState(LPC_GPIO,Num,lcd->pins.d4)<<4;
	tmp |=Chip_GPIO_GetPinState(LPC_GPIO,Num,lcd->pins.d5)<<5;
	tmp |=Chip_GPIO_GetPinState(LPC_GPIO,Num,lcd->pins.d6)<<6;
	tmp |=Chip_GPIO_GetPinState(LPC_GPIO,Num,lcd->pins.d7)<<7;
#if 0
	tmp |= gpio_get(lcd->pins.d4) << 4;
	tmp |= gpio_get(lcd->pins.d5) << 5;
	tmp |= gpio_get(lcd->pins.d6) << 6;
	tmp |= gpio_get(lcd->pins.d7) << 7;
#endif
	//gpio_set(lcd->pins.e, 0);
    Chip_GPIO_SetPinState(LPC_GPIO,Num,lcd->pins.e,false);
    //Chip_GPIO_SetMaskedPortValue()
	//udelay(lcd->Te);
    _delay_ms(lcd->Te);
    Chip_GPIO_SetPinState(LPC_GPIO,Num,lcd->pins.e,true);
	//gpio_set(lcd->pins.e, 1);

	_delay_ms(lcd->Te);
	tmp |= Chip_GPIO_GetPinState(LPC_GPIO,Num,lcd->pins.d4)<<0;
	tmp |= Chip_GPIO_GetPinState(LPC_GPIO,Num,lcd->pins.d5)<<1;
	tmp |= Chip_GPIO_GetPinState(LPC_GPIO,Num,lcd->pins.d6)<<2;
	tmp |= Chip_GPIO_GetPinState(LPC_GPIO,Num,lcd->pins.d7)<<3;
	Chip_GPIO_SetPinState(LPC_GPIO,Num,lcd->pins.e,false);
#if 0
	tmp |= gpio_get(lcd->pins.d4) << 0;
	tmp |= gpio_get(lcd->pins.d5) << 1;
	tmp |= gpio_get(lcd->pins.d6) << 2;
	tmp |= gpio_get(lcd->pins.d7) << 3;

	gpio_set(lcd->pins.e, 0);
#endif
	Chip_GPIO_SetDir(LPC_GPIO,Num,lcd->pins.d4,GPIO_OUTPUT);
	Chip_GPIO_SetDir(LPC_GPIO,Num,lcd->pins.d5,GPIO_OUTPUT);
	Chip_GPIO_SetDir(LPC_GPIO,Num,lcd->pins.d6,GPIO_OUTPUT);
	Chip_GPIO_SetDir(LPC_GPIO,Num,lcd->pins.d7,GPIO_OUTPUT);
	return tmp;
}

static void lcd_cmd(struct hd44780_data *lcd, int cmd,uint8_t GpioNum)
{
	int timeout = 1000;
	while (--timeout) {
		/* bits 0-6 are address, that might be useful too */
		if ((lcd_read(lcd, 0,GpioNum) & 0x80) == 0)
			break;
	}
	if (timeout == 0)
		printf("%s, timeouted at cmd %x\n", __func__, cmd);

	lcd_put(lcd, 0, cmd>>4,GpioNum);
	lcd_put(lcd, 0, cmd&0xf,GpioNum);
}

static void lcd_data(struct hd44780_data *lcd, int cmd,uint8_t GpioNum)
{
	int timeout = 1000;
	while (--timeout) {
		/* bits 0-6 are address, that might be useful too */
		if ((lcd_read(lcd, 0,GpioNum) & 0x80) == 0)
			break;
	}
	if (timeout == 0)
		printf("%s, timeouted at cmd %x\n", __func__, cmd);

	lcd_put(lcd, 1, cmd>>4,GpioNum);
	lcd_put(lcd, 1, cmd&0xf,GpioNum);
}

void lcd_init(struct hd44780_data *lcd,uint8_t GpioNum)
{
	//gpio_init(, GPIO_OUTPUT, 0);
	Chip_GPIO_WriteDirBit(LPC_GPIO,GpioNum,lcd->pins.rs,true);
	Chip_GPIO_WriteDirBit(LPC_GPIO,GpioNum,lcd->pins.rw,true);
	Chip_GPIO_WriteDirBit(LPC_GPIO,GpioNum,lcd->pins.e,true);
	Chip_GPIO_WriteDirBit(LPC_GPIO,GpioNum,lcd->pins.d4,true);
	Chip_GPIO_WriteDirBit(LPC_GPIO,GpioNum,lcd->pins.d5,true);
	Chip_GPIO_WriteDirBit(LPC_GPIO,GpioNum,lcd->pins.d6,true);
	Chip_GPIO_WriteDirBit(LPC_GPIO,GpioNum,lcd->pins.d7,true);

	/* reset sequence */
	lcd_put(lcd, 0, 3,GpioNum);

	_delay_ms(4100);
	lcd_put(lcd, 0, 3,GpioNum);
	_delay_ms(100);

	lcd_put(lcd, 0, 3,GpioNum);
	_delay_ms(37);
	lcd_put(lcd, 0, 2,GpioNum);
	_delay_ms(37);

	/* ok, in 4-bit mode now */
	int tmp = 0;
	if (lcd->caps & HD44780_CAPS_2LINES)
		tmp |= 1<<3;
	if (lcd->caps & HD44780_CAPS_5X10)
		tmp |= 1<<2;
	lcd_cmd(lcd, CMD_FUNCTION_SET | tmp,GpioNum);
	lcd_cmd(lcd, CMD_DISPLAY_ON_OFF,GpioNum); /* display, cursor and blink off */
	lcd_cmd(lcd, CMD_CLEAR,GpioNum);

	lcd_cmd(lcd, CMD_ENTRY_MODE | HD44780_ENTRY_INC,GpioNum);
}


static void lcd_clear(struct hd44780_data *lcd,uint8_t GpioNum)
{
	lcd_cmd(lcd, CMD_CLEAR,GpioNum);
}

static void lcd_home(struct hd44780_data *lcd,uint8_t GpioNum)
{
	lcd_cmd(lcd, CMD_HOME,GpioNum);
}

static void lcd_entry_mode(struct hd44780_data *lcd, int mode,uint8_t GpioNum)
{
	lcd_cmd(lcd, CMD_ENTRY_MODE | (mode&0x3),GpioNum);
}

static void lcd_onoff(struct hd44780_data *lcd, int features,uint8_t GpioNum)
{
	lcd_cmd(lcd, CMD_DISPLAY_ON_OFF | (features&0x7),GpioNum);
}

static void lcd_shift(struct hd44780_data *lcd, int shift,uint8_t GpioNum)
{
	lcd_cmd(lcd, CMD_SHIFT | (shift&0xc),GpioNum);
}

static void lcd_set_position(struct hd44780_data *lcd, int pos,uint8_t GpioNum)
{
	lcd_cmd(lcd, CMD_DDRAM_ADDR | (pos&0x7f),GpioNum);
}

void lcd_print(struct hd44780_data *lcd, const char *str,uint8_t GpioNum)
{
	while (*str)
		lcd_data(lcd, *str++,GpioNum);
}
const struct hd44780_driver hd44780_driver = {
	.init= lcd_init,
	.clear = lcd_clear,
	.home = lcd_home,
	.entry_mode = lcd_entry_mode,
	.onoff = lcd_onoff,
	.shift = lcd_shift,
	.set_position = lcd_set_position,
	.write = lcd_data,
	.print = lcd_print,
	.read = lcd_read,
	.write_cmd = lcd_cmd,
};
