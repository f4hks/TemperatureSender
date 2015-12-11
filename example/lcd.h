/* Author: Domen Puncer <domen@cba.si>.  License: WTFPL, see file LICENSE */
/*!
 * @file : lcd.h
 * @author: Domen Puncer
 * @license: WTFPL
 *
 * */
#ifndef _LCD_H_
#define _LCD_H_

void _delay_ms (unsigned short ms);

struct hd44780_data {
	struct {
		int rs, rw, e,gpioNum;
		int d4, d5, d6, d7;
	} pins;
	int Te; /* TcycE/2 in us, must be >= 1 */
	int caps;
};

#define HD44780_CAPS_2LINES     (1<<0)
#define HD44780_CAPS_5X10       (1<<1)

#define HD44780_ENTRY_INC       (1<<1)
#define HD44780_ENTRY_SHIFT     (1<<0)

#define HD44780_ONOFF_DISPLAY_ON      (1<<2)
#define HD44780_ONOFF_CURSOR_ON       (1<<1)
#define HD44780_ONOFF_BLINKING_ON     (1<<0)

#define HD44780_SHIFT_DISPLAY   (1<<3)
#define HD44780_SHIFT_CURSOR    (0<<3)
#define HD44780_SHIFT_RIGHT     (1<<2)
#define HD44780_SHIFT_LEFT      (0<<2)

#define HD44780_LINE_OFFSET	0x40
#define GPIO_OUTPUT       !0
#define GPIO_INPUT        0

/* local defines */
enum hd44780_commands {
	CMD_CLEAR =             0x01,
	CMD_HOME =              0x02,
	CMD_ENTRY_MODE =        0x04, /* args: I/D, S */
	CMD_DISPLAY_ON_OFF =    0x08, /* args: D, C, B */
	CMD_SHIFT =             0x10, /* args: S/C, R/L, -, - */
	CMD_FUNCTION_SET =      0x20, /* args: DL, N, F, -, -; only valid at reset */
	CMD_CGRAM_ADDR =        0x40,
	CMD_DDRAM_ADDR =        0x80,
};
void lcd_init(struct hd44780_data *lcd,uint8_t GpioNum);
void lcd_print(struct hd44780_data *lcd, const char *str,uint8_t GpioNum)




struct hd44780_driver {
	void (*init)(struct hd44780_data *lcd,uint8_t GpioNum);
	void (*clear)(struct hd44780_data *lcd,uint8_t GpioNum);
	void (*home)(struct hd44780_data *lcd,uint8_t GpioNum);
	void (*entry_mode)(struct hd44780_data *lcd, int mode,uint8_t GpioNum);
	void (*onoff)(struct hd44780_data *lcd, int features,uint8_t GpioNum);
	void (*shift)(struct hd44780_data *lcd, int shift,uint8_t GpioNum);
	void (*set_position)(struct hd44780_data *lcd, int pos,uint8_t GpioNum);
	void (*write)(struct hd44780_data *lcd, int data,uint8_t GpioNum);
	void (*print)(struct hd44780_data *lcd, const char *str,uint8_t GpioNum);
	/* low level, not to be used */
	int (*read)(struct hd44780_data *lcd, int rs,uint8_t Num);
	void (*write_cmd)(struct hd44780_data *lcd, int cmd,uint8_t GpioNum);
};
extern const struct hd44780_driver hd44780_driver;


#endif
