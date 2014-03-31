/*
 * ILI9320 Framebuffer
 *
 * ToDo: Fix this text vv
 *
 * Original: Copyright (c) 2009 Jean-Christian de Rivaz
 *
 * Console support, 320x240 instead of 240x320:
 * Copyright (c) 2012 Jeroen Domburg <jeroen@spritesmods.com>
 *
 * Bits and pieces borrowed from the fsl-ili9325.c:
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Author: Alison Wang <b18965@freescale.com>
 *         Jason Jin <Jason.jin@freescale.com>
 *
 * Supports ILI9320 chip by Shawn Chain <shawn.chain@gmail.com>
 * 
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * The Solomon Systech ILI9325 chip drive TFT screen up to 240x320. 
 *
 * For direct I/O-mode:
 *
 * This driver expect the SSD1286 to be connected to a 16 bits local bus
 * and to be set in the 16 bits parallel interface mode. To use it you must
 * define in your board file a struct platform_device with a name set to
 * "ili9325" and a struct resource array with two IORESOURCE_MEM: the first
 * for the control register; the second for the data register.
 *
 *
 * LCDs in their own, native SPI mode aren't supported yet, mostly because I 
 * can't get my hands on a cheap one.
 */

//#define DEBUG

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/fb.h>
#include <asm/io.h>
#include <linux/delay.h>

#define LCD_NCS 4       // BRCM_GPIO_04 <--> P1_07 <--> RPI_GPIO_7
// for PCB rev2
#define LCD_NRD 27      // BRCM_GPIO_27 <--> p1_13 <--> RPI_GPIO_? (rev2)
#define LCD_NWR 17      // BRCM_GPIO_17 <--> p1_11 <--> RPI_GPIO_0


#define LCD_RS 18       // BRCM_GPIO_18 <--> p1_12 <--> RPI_GPIO_1
#define LCD_NRST 7      // BRCM_GPIO_07 <--> p1_26 <--> RPI_SPI_CE1_N

#define LCD_D10 22      // BRCM_GPIO_22 <--> p1_15 <--> RPI_GPIO_3
#define LCD_D11 23      // BRCM_GPIO_23 <--> p1_16 <--> RPI_GPIO_4
#define LCD_D12 24      // BRCM_GPIO_24 <--> p1_18 <--> RPI_GPIO_5
#define LCD_D13 10      // BRCM_GPIO_10 <--> p1_19 <--> RPI_SPI_MOSI
#define LCD_D14 25      // BRCM_GPIO_25 <--> p1_22 <--> RPI_GPIO_6
#define LCD_D15 9       // BRCM_GPIO_09 <--> p1_21 <--> RPI_SPI_MISO
#define LCD_D16 11      // BRCM_GPIO_11 <--> p1_23 <--> RPI_SPI_SCLK
#define LCD_D17 8       // BRCM_GPIO_08 <--> p1_24 <--> RPI_CE0_N

#define UPSIDEDOWN

struct ili9325_page {
	unsigned short x;
	unsigned short y;
	unsigned short *buffer;
	unsigned short *oldbuffer;
	unsigned short len;
	int must_update;
};

struct ili9325 {
	struct device *dev;
	struct fb_info *info;
	unsigned int pages_count;
	struct ili9325_page *pages;
	unsigned long pseudo_palette[17];
	int backlight;
};

// ==== GPIO HI/LO operations ====
// GPIO_BASE+0x28 = GPCLR0(GPIO Pin Output Clear 0)
// GPIO_BASE+0x1C = GPSET0(GPIO Pin Output Set 0)
// Always use GPSET0 because pin number larger than 32 is not used here.
// - Shawn
#define BRCM_GPSET0     (GPIO_BASE + 0x1C)
#define BRCM_GPCLR0     (GPIO_BASE + 0x28)
#define BRCM_GPLEV0     (GPIO_BASE + 0x34)
#define BRCM_GPPUD      (GPIO_BASE + 0x94)
#define BRCM_GPPUDCLK0  (GPIO_BASE + 0x98)

#define GPIO_HI(port)   writel((1<<port),  __io_address(BRCM_GPSET0));
#define GPIO_LO(port)   writel((1<<port),  __io_address(BRCM_GPCLR0));
#define GPIO_NOOP       writel(0,  __io_address(BRCM_GPCLR0));

// ==== GPIO Pull up/down setup ====
static inline void gpio_setpud(unsigned int port, unsigned int val){
    writel(val,__io_address(BRCM_GPPUD));
    GPIO_NOOP;
    writel((1<<port),__io_address(BRCM_GPPUDCLK0)); // the pin 01 pull down
    GPIO_NOOP;
}

// ==== GPIO Pin Mode Setups ====
// See BCM2835 GPIO Function Select Registers datasheet.
// The selector register uses 3bits for each GPIO port function definition
// For a 32bit register, it contains up to 10 GPIO port function definitions.
// Eg: set GPFEL0->bit[12,13,14] = 001 means Use GPIO4 as output
// - Shawn
#define PIN_MODE_INPUT  0
#define PIN_MODE_OUTPUT 1
#define GPIO_DATA_READ      ili932x_gpio_set_databus_mode(PIN_MODE_INPUT);
#define GPIO_DATA_WRITE     ili932x_gpio_set_databus_mode(PIN_MODE_OUTPUT);
#define GPIO_ALT_OFFSET(g)  ((((g)/10))*4)
#define GPIO_ALT_VAL(a, g)  ((a)<<(((g)%10)*3))
static inline void ili932x_gpio_set_pin_mode(int gpio,int mode){
    unsigned int v;
	v=readl(__io_address(GPIO_BASE+GPIO_ALT_OFFSET(gpio)));
	v&=~GPIO_ALT_VAL(0x7, gpio); //clear existing bits 0111
	v|=GPIO_ALT_VAL(mode, gpio); //pin mode, currently output or input only
	writel(v, __io_address(GPIO_BASE+GPIO_ALT_OFFSET(gpio)));
}
static inline void ili932x_gpio_set_databus_mode(int mode){
    //TODO use one-shot changes
    ili932x_gpio_set_pin_mode(LCD_D10,mode);
	ili932x_gpio_set_pin_mode(LCD_D11,mode);
	ili932x_gpio_set_pin_mode(LCD_D12,mode);
	ili932x_gpio_set_pin_mode(LCD_D13,mode);
	ili932x_gpio_set_pin_mode(LCD_D14,mode);
	ili932x_gpio_set_pin_mode(LCD_D15,mode);
	ili932x_gpio_set_pin_mode(LCD_D16,mode);
	ili932x_gpio_set_pin_mode(LCD_D17,mode);
}


// ==== Data Read/Write ====
/*
Use direct GPIO reg access instead of the gpiolib framework: because we want to write
multiple bits at once to the (presumably connected over a slower bus) GPIO block,
this involves less writes and so will be faster.
*/
/* macros to get at IO space when running virtually */
#define GPIOSET(no, ishigh){ if (ishigh) set|=(1<<no); else reset|=(1<<no); } while(0)
static inline void ili932x_gpio_set_databus_level(unsigned char aByte){
    unsigned int set=0,reset=0;
    GPIOSET(LCD_D10, (aByte&0x01)); // 8bit data
	GPIOSET(LCD_D11, (aByte&0x02));
	GPIOSET(LCD_D12, (aByte&0x04));
	GPIOSET(LCD_D13, (aByte&0x08));
	GPIOSET(LCD_D14, (aByte&0x10));
	GPIOSET(LCD_D15, (aByte&0x20));
	GPIOSET(LCD_D16, (aByte&0x40));
	GPIOSET(LCD_D17, (aByte&0x80));
	writel(set, __io_address(BRCM_GPSET0));
	writel(reset, __io_address(BRCM_GPCLR0));
}

static inline void ili932x_write_begin(void){
    GPIO_LO(LCD_NCS);
    // GPIO_DATA_WRITE; - Databus set for write
    ili932x_gpio_set_databus_mode(PIN_MODE_OUTPUT);
}

static inline void ili932x_read_begin(void){
    GPIO_LO(LCD_NCS);
    // GPIO_DATA_READ; - Databus set for write
    ili932x_gpio_set_databus_mode(PIN_MODE_INPUT);
}


#define LCD_CMD  0
#define LCD_DATA 1

/**
 * Write a word, assume nCS is LOW
 */
static inline void ili932x_write_unsafe(unsigned int data, int cmdOrData){
    unsigned int set=0;
	unsigned int reset=0;
    unsigned char aByte = 0;

    if(cmdOrData == LCD_CMD){
        GPIO_LO(LCD_RS);
    }
    
    // write high 8bits
    aByte = (data >> 8);
    // We don't use GPIOSET macro here for performance reason
    set |= (((aByte >> 0)&0x01) << LCD_D10);
    set |= (((aByte >> 1)&0x01) << LCD_D11);
    set |= (((aByte >> 2)&0x01) << LCD_D12);
    set |= (((aByte >> 3)&0x01) << LCD_D13);
    set |= (((aByte >> 4)&0x01) << LCD_D14);
    set |= (((aByte >> 5)&0x01) << LCD_D15);
    set |= (((aByte >> 6)&0x01) << LCD_D16);
    set |= (((aByte >> 7)&0x01) << LCD_D17);
    
    reset |= ((1 & ~((aByte >> 0)&0x01)) << LCD_D10);
    reset |= ((1 & ~((aByte >> 1)&0x01)) << LCD_D11);
    reset |= ((1 & ~((aByte >> 2)&0x01)) << LCD_D12);
    reset |= ((1 & ~((aByte >> 3)&0x01)) << LCD_D13);
    reset |= ((1 & ~((aByte >> 4)&0x01)) << LCD_D14);
    reset |= ((1 & ~((aByte >> 5)&0x01)) << LCD_D15);
    reset |= ((1 & ~((aByte >> 6)&0x01)) << LCD_D16);
    reset |= ((1 & ~((aByte >> 7)&0x01)) << LCD_D17);

	writel(set, __io_address(BRCM_GPSET0));
	writel(reset, __io_address(BRCM_GPCLR0));
    
    GPIO_LO(LCD_NWR);
    GPIO_NOOP;
    GPIO_HI(LCD_NWR);

    // write low 8bits
    set = 0;
    reset = 0;
    aByte = (data & 0xff);
    
    set |= (((aByte >> 0)&0x01) << LCD_D10);
    set |= (((aByte >> 1)&0x01) << LCD_D11);
    set |= (((aByte >> 2)&0x01) << LCD_D12);
    set |= (((aByte >> 3)&0x01) << LCD_D13);
    set |= (((aByte >> 4)&0x01) << LCD_D14);
    set |= (((aByte >> 5)&0x01) << LCD_D15);
    set |= (((aByte >> 6)&0x01) << LCD_D16);
    set |= (((aByte >> 7)&0x01) << LCD_D17);
    
    reset |= ((1 & ~((aByte >> 0)&0x01)) << LCD_D10);
    reset |= ((1 & ~((aByte >> 1)&0x01)) << LCD_D11);
    reset |= ((1 & ~((aByte >> 2)&0x01)) << LCD_D12);
    reset |= ((1 & ~((aByte >> 3)&0x01)) << LCD_D13);
    reset |= ((1 & ~((aByte >> 4)&0x01)) << LCD_D14);
    reset |= ((1 & ~((aByte >> 5)&0x01)) << LCD_D15);
    reset |= ((1 & ~((aByte >> 6)&0x01)) << LCD_D16);
    reset |= ((1 & ~((aByte >> 7)&0x01)) << LCD_D17);
    
	writel(set, __io_address(BRCM_GPSET0));
	writel(reset, __io_address(BRCM_GPCLR0));
    
    GPIO_LO(LCD_NWR);
    GPIO_NOOP;//msleep(10);
    GPIO_HI(LCD_NWR);
    if(cmdOrData == LCD_CMD){
        // restore the RS state
        GPIO_HI(LCD_RS);
    }
#if 0
    printk(KERN_INFO "ili932x_fb  ili932x_write_unsafe: %#06x\n",data);
#endif
}

static inline void ili932x_write_end(void){
    GPIO_HI(LCD_NCS);
}

/**
 * Write a word to LCD controller
 */
static inline void ili932x_write(unsigned int data, int cmdOrData){
    ili932x_write_begin();
    ili932x_write_unsafe(data,cmdOrData);
    ili932x_write_end();
}

#define GPIOGET(n,v) ((v & (1<<n)) >> n)
static inline unsigned int ili932x_read_unsafe(void){
    unsigned int val = 0;
    unsigned int pinLevels = 0;
    unsigned char aByte = 0;

    // GPIO_DATA_READ; - Databus set for read
    ili932x_gpio_set_databus_mode(PIN_MODE_INPUT);
    ili932x_gpio_set_databus_level(0xFF);
    
    // Read the high part
	GPIO_LO(LCD_NRD);//writel((1<<LCD_NRD),  __io_address(GPIO_BASE+0x28));
    msleep(10);
    pinLevels = readl(__io_address(BRCM_GPLEV0)); // a 32bit mask value
    aByte |= (GPIOGET(LCD_D10,pinLevels) << 0); // bit 0 - 7
    aByte |= (GPIOGET(LCD_D11,pinLevels) << 1);
    aByte |= (GPIOGET(LCD_D12,pinLevels) << 2);
    aByte |= (GPIOGET(LCD_D13,pinLevels) << 3);
    aByte |= (GPIOGET(LCD_D14,pinLevels) << 4);
    aByte |= (GPIOGET(LCD_D15,pinLevels) << 5);
    aByte |= (GPIOGET(LCD_D16,pinLevels) << 6);
    aByte |= (GPIOGET(LCD_D17,pinLevels) << 7);
    val = ((aByte<<8) & 0xFF00);
    GPIO_HI(LCD_NRD);//writel((1<<LCD_NRD),  __io_address(GPIO_BASE+0x1C));
    
    // Read the low part
    aByte = 0;
	GPIO_LO(LCD_NRD);//writel((1<<LCD_NRD),  __io_address(GPIO_BASE+0x28));
    msleep(10);
    pinLevels = readl(__io_address(BRCM_GPLEV0)); // a 32bit mask value
    aByte |= (GPIOGET(LCD_D10,pinLevels) << 0); // bit 0 - 7
    aByte |= (GPIOGET(LCD_D11,pinLevels) << 1);
    aByte |= (GPIOGET(LCD_D12,pinLevels) << 2);
    aByte |= (GPIOGET(LCD_D13,pinLevels) << 3);
    aByte |= (GPIOGET(LCD_D14,pinLevels) << 4);
    aByte |= (GPIOGET(LCD_D15,pinLevels) << 5);
    aByte |= (GPIOGET(LCD_D16,pinLevels) << 6);
    aByte |= (GPIOGET(LCD_D17,pinLevels) << 7);
    val |= aByte;
    GPIO_HI(LCD_NRD);//writel((1<<LCD_NRD),  __io_address(GPIO_BASE+0x1C));
    
    printk(KERN_INFO "ili932x_fb  ili932x_read_unsafe: %#06x\n",val);
    return val;
}

static inline unsigned int ili932x_read(void){
    unsigned int value = 0;
    ili932x_read_begin();
    value = ili932x_read_unsafe();
    ili932x_write_end();
    return value;
}

// ---- LCD Register Operator ----
#define LCD_SET_REG(x,y) lcd_write_register(x,y)

static inline unsigned int lcd_read_register(unsigned int reg){
    unsigned int value = 0;
    // write command
    ili932x_write(reg,LCD_CMD);
    // read data
    value = ili932x_read();
    return value;
}

static inline void lcd_write_register(unsigned int reg, unsigned int val){
    // write command
    ili932x_write(reg,LCD_CMD);
    // write data
    ili932x_write(val,LCD_DATA);
}

// ---- Initialize the BCM board GPIO pins ----//
static inline void ili9325_gpio_init(void)
{
	ili932x_gpio_set_pin_mode(LCD_NCS,PIN_MODE_OUTPUT);
	ili932x_gpio_set_pin_mode(LCD_NWR,PIN_MODE_OUTPUT); //write
	ili932x_gpio_set_pin_mode(LCD_NRD,PIN_MODE_OUTPUT); //read
	ili932x_gpio_set_pin_mode(LCD_RS,PIN_MODE_OUTPUT);  //cmd or data
	ili932x_gpio_set_pin_mode(LCD_NRST,PIN_MODE_OUTPUT);//reset

    // Use BCM_GPUPD/GPUPDCLK0 to setup the pin pull up.
    gpio_setpud(LCD_NCS,1);
    gpio_setpud(LCD_RS,1);
    gpio_setpud(LCD_NRD,1);
    gpio_setpud(LCD_NWR,1);
    gpio_setpud(LCD_NRST,1);
    
    // setup data bus for write
    ili932x_gpio_set_databus_mode(PIN_MODE_OUTPUT);
    ili932x_gpio_set_databus_level(0xff);    
}

// ---- Hardware reset the LCD controller ----
static inline int ili932x_init_controller(void) {
    int driverCode = 0;

    // reset LCD controller
    printk(KERN_INFO "ili932x_fb:  Resetting TFT controller\n");
	GPIO_LO(LCD_NRST);
	msleep(20);
	GPIO_HI(LCD_NRST);
    msleep(20);

    printk(KERN_INFO "ili932x_fb:  Starting TFT controller Oscillation\n");
    
    // Start oscillation
    LCD_SET_REG(0x0000,0x0001); // Start Oscillation
    msleep(50);
    
    // read the driver version code
    driverCode = lcd_read_register(0x0000);
    printk(KERN_INFO "ili932x_fb:  Read TFT controller type code: %#08x\n",driverCode);
    if(driverCode == 0){
        printk(KERN_INFO "ili932x_fb:  No TFT controller found.\n");
        return 0;
    }
    
    return driverCode;
}


/*
 * Set LCD position pointer for display
 */
static void ili9325_setptr(struct ili9325 *item, int x, int y) {
#ifdef UPSIDEDOWN
	LCD_SET_REG(0x0020,y); // Horizontal GRAM Start Address
	LCD_SET_REG(0x0021,(item->info->var.xres - 1)-x); // Vertical GRAM Start Address
#else
	LCD_SET_REG(0x0020,(item->info->var.yres - 1)-y); // Horizontal GRAM Start Address
	LCD_SET_REG(0x0021,x); // Vertical GRAM Start Address
#endif
	ili932x_write(0x0022,LCD_CMD); // Prepare for data
}

static void ili9325_setptr_unsafe(struct ili9325 *item, int x, int y) {
    unsigned int r20, r21;
#ifdef UPSIDEDOWN
    r20 = y; // Horizontal
    r21 = (item->info->var.xres - 1)-x;// Vertical
#else
    r20 = (item->info->var.yres - 1)-y; // Horizontal
    r20 = x; // Vertical
#endif
    ili932x_write_unsafe(0x0020,LCD_CMD);
    ili932x_write_unsafe(r20,LCD_DATA);// Horizontal GRAM Start Address
    
    ili932x_write_unsafe(0x0021,LCD_CMD);
    ili932x_write_unsafe(r21,LCD_DATA);// Vertical GRAM Start Address
    
    ili932x_write_unsafe(0x0022,LCD_CMD); // Prepare for data
}


/*
 * write fb buffer data to TFT
 */
static void ili9325_copy(struct ili9325 *item, unsigned int index)
{
	unsigned short x;
	unsigned short y;
	unsigned short *buffer;
	unsigned short *oldbuffer;
	unsigned int len;
	unsigned int count;
	int sendNewPos=1;
	x = item->pages[index].x;
	y = item->pages[index].y;
	buffer = item->pages[index].buffer;
	oldbuffer = item->pages[index].oldbuffer;
	len = item->pages[index].len;
	dev_dbg(item->dev,
		"%s: page[%u]: x=%3hu y=%3hu buffer=0x%p len=%3hu\n",
		__func__, index, x, y, buffer, len);

	//Only update changed pixels in the page.
    ili932x_write_begin();
	for (count = 0; count < len; count++) {
		if (buffer[count]==oldbuffer[count]) {
			sendNewPos=1;
		} else {
			if (sendNewPos) {
				ili9325_setptr_unsafe(item, x, y);
				sendNewPos=0;
			}
			ili932x_write_unsafe(buffer[count],LCD_DATA);//ili932x_write_word(buffer[count], 1);
			oldbuffer[count]=buffer[count];
		}
		x++;
		if (x>=item->info->var.xres) {
			y++;
			x=0;
		}
	}
    ili932x_write_end();
}

static void ili9325_update_all(struct ili9325 *item)
{
	unsigned short i;
	struct fb_deferred_io *fbdefio = item->info->fbdefio;
	for (i = 0; i < item->pages_count; i++) {
		item->pages[i].must_update=1;
	}
	schedule_delayed_work(&item->info->deferred_work, fbdefio->delay);
}

static void ili9325_update(struct fb_info *info, struct list_head *pagelist)
{
	struct ili9325 *item = (struct ili9325 *)info->par;
	struct page *page;
	int i;

	//We can be called because of pagefaults (mmap'ed framebuffer, pages
	//returned in *pagelist) or because of kernel activity 
	//(pages[i]/must_update!=0). Add the former to the list of the latter.
	list_for_each_entry(page, pagelist, lru) {
		item->pages[page->index].must_update=1;
	}

	//Copy changed pages.
	for (i=0; i<item->pages_count; i++) {
		//ToDo: Small race here between checking and setting must_update, 
		//maybe lock?
		if (item->pages[i].must_update) {
			item->pages[i].must_update=0;
			ili9325_copy(item, i);
		}
	}

}

#define ILI9320_REG_OSCILLATION      0x00
#define ILI9320_REG_DRIVER_OUT_CTRL  0x01
#define ILI9320_REG_LCD_DRIVE_AC     0x02
#define ILI9320_REG_POWER_CTRL_1     0x03
#define ILI9320_REG_DISPLAY_CTRL     0x07
#define ILI9320_REG_FRAME_CYCLE      0x0b
#define ILI9320_REG_POWER_CTRL_2     0x0c
#define ILI9320_REG_POWER_CTRL_3     0x0d
#define ILI9320_REG_POWER_CTRL_4     0x0e
#define ILI9320_REG_GATE_SCAN_START  0x0f
#define ILI9320_REG_SLEEP_MODE       0x10
#define ILI9320_REG_ENTRY_MODE       0x11
#define ILI9320_REG_POWER_CTRL_5     0x1e
#define ILI9320_REG_GDDRAM_DATA      0x22
#define ILI9320_REG_WR_DATA_MASK_1   0x23
#define ILI9320_REG_WR_DATA_MASK_2   0x24
#define ILI9320_REG_FRAME_FREQUENCY  0x25
#define ILI9320_REG_GAMMA_CTRL_1     0x30
#define ILI9320_REG_GAMME_CTRL_2     0x31
#define ILI9320_REG_GAMMA_CTRL_3     0x32
#define ILI9320_REG_GAMMA_CTRL_4     0x33
#define ILI9320_REG_GAMMA_CTRL_5     0x34
#define ILI9320_REG_GAMMA_CTRL_6     0x35
#define ILI9320_REG_GAMMA_CTRL_7     0x36
#define ILI9320_REG_GAMMA_CTRL_8     0x37
#define ILI9320_REG_GAMMA_CTRL_9     0x3a
#define ILI9320_REG_GAMMA_CTRL_10    0x3b
#define ILI9320_REG_V_SCROLL_CTRL_1  0x41
#define ILI9320_REG_V_SCROLL_CTRL_2  0x42
#define ILI9320_REG_H_RAM_ADR_POS    0x44
#define ILI9320_REG_V_RAM_ADR_START  0x45
#define ILI9320_REG_V_RAM_ADR_END    0x46
#define ILI9320_REG_FIRST_WIN_START  0x48
#define ILI9320_REG_FIRST_WIN_END    0x49
#define ILI9320_REG_SECND_WIN_START  0x4a
#define ILI9320_REG_SECND_WIN_END    0x4b
#define ILI9320_REG_GDDRAM_X_ADDR    0x4e
#define ILI9320_REG_GDDRAM_Y_ADDR    0x4f


#define LCD_ALIGNMENT_VERTICAL 1
static void inline ili9320_setup_1(struct ili9325 *item){
    /*** Display Setting ***/
    lcd_write_register(0x01, 0x0100);   //Driver output control (1)
    lcd_write_register(0x02, 0x0700);   //LCD driving control
    
    // Check page 55 in the datasheet for more information about alignment
#if LCD_ALIGNMENT_VERTICAL
    lcd_write_register(0x03, 0x1018);         //Entry mode
#else // align horizontal
    lcd_write_register(0x03, 0x1028);         //Entry mode
#endif
    
    lcd_write_register(0x04, 0x0000);   //Resize control
    lcd_write_register(0x08, 0x0202);   //Display control (2)
    lcd_write_register(0x09, 0x0000);   //Display control (3)
    
    /*** Power Control ***/
    lcd_write_register(0x07, 0x0101);   //power control 1 BT, AP
    lcd_write_register(0x17, 0x0001);
    lcd_write_register(0x10, 0x0000);
    lcd_write_register(0x11, 0x0007);   //power control 2 DC, VC
    lcd_write_register(0x12, 0x0000);   //power control 3 VRH
    lcd_write_register(0x12, 0x0000);   //power control 3 VRH
    lcd_write_register(0x13, 0x0000);   //power control 4 VCOM amplitude
    msleep(20);
    
    lcd_write_register(0x10, 0x16B0);             //power control 1 BT,AP
    lcd_write_register(0x11, 0x0037);             //power control 2 DC,VC
    msleep(50);
    lcd_write_register(0x12, 0x013E);             //power control 3 VRH
    msleep(50);
    lcd_write_register(0x13, 0x1A00);             //power control 4 vcom amplitude
    lcd_write_register(0x29, 0x000F);             //power control 7 VCOMH
    msleep(50);
    lcd_write_register(0x20, 0x0000);              //Horizontal GRAM Address Set
    lcd_write_register(0x21, 0x0000);              //Vertical GRAM Address Set
    
    lcd_write_register(0x50, 0x0000);              //Horizontal Address Start Position
    lcd_write_register(0x51, 0x00EF);              //Horizontal Address end Position (239)
    lcd_write_register(0x52, 0x0000);              //Vertical Address Start Position
    lcd_write_register(0x53, 0x013F);              //Vertical Address end Position (319)
    
    
    lcd_write_register(0x60, 0x2700);              //Driver Output Control 2
    //  lcd_write_register(0x61, 0x0001);              //Base Image Display Control
    lcd_write_register(0x61, 0x0003);              //Base Image Display Control
    lcd_write_register(0x6a, 0x0000);              //Base Image Display Control
    
    lcd_write_register(0x90, 0x0010);              //Panel Interface Control 1
    lcd_write_register(0x92, 0x0000);              //Panel Interface Control 2
    lcd_write_register(0x93, 0x0000);              //Panel Interface Control 3
    /*** GAMMA Control ***/
    lcd_write_register(0x30, 0x0007);
    lcd_write_register(0x31, 0x0403);
    lcd_write_register(0x32, 0x0404);
    lcd_write_register(0x35, 0x0002);
    lcd_write_register(0x36, 0x0707);
    lcd_write_register(0x37, 0x0606);
    lcd_write_register(0x38, 0x0106);
    lcd_write_register(0x39, 0x0007);
    lcd_write_register(0x3c, 0x0700);
    lcd_write_register(0x3d, 0x0707);
    
    lcd_write_register(0x07, 0x0173);
    
    GPIO_HI(LCD_NCS);
    msleep(30);
    GPIO_LO(LCD_NCS);
}

/*
ili9320_setup_2(struct ili9325 *item){
     //DCT=b1010=fosc/4 BT=b001=VGH:+6,VGL:-4
     //DC=b1010=fosc/4 AP=b010=small to medium
     LCD_SET_REG(ILI9320_REG_POWER_CTRL_1, 0xa2a4);
     //VRC=b100:5.5V
     LCD_SET_REG(ILI9320_REG_POWER_CTRL_2, 0x0004);
     //VRH=b1000:Vref*2.165
     LCD_SET_REG(ILI9320_REG_POWER_CTRL_3, 0x0308);
     //VCOMG=1 VDV=b1000:VLCD63*1.05
     LCD_SET_REG(ILI9320_REG_POWER_CTRL_4, 0x3000);
     //nOTP=1 VCM=0x2a:VLCD63*0.77
     LCD_SET_REG(ILI9320_REG_POWER_CTRL_5, 0x00ea);
     //RL=0 REV=1 CAD=0 BGR=1 SM=0 TB=1 MUX=0x13f=319
     LCD_SET_REG(ILI9320_REG_DRIVER_OUT_CTRL, 0x2b3f);
     //FLD=0 ENWS=0 D/C=1 EOR=1 WSMD=0 NW=0x00=0
     LCD_SET_REG(ILI9320_REG_LCD_DRIVE_AC, 0x0600);
     //SLP=0
     LCD_SET_REG(ILI9320_REG_SLEEP_MODE, 0x0000);
     //VSMode=0 DFM=3:65k TRAMS=0 OEDef=0 WMode=0 Dmode=0
     //TY=0 ID=2 AM=1 LG=0 (orig: ID=3, AM=0)
     //	ssd1289_reg_set(item, SSD1289_REG_ENTRY_MODE, 0x6030);
     LCD_SET_REG(ILI9320_REG_ENTRY_MODE, 0x6028);
     //PT=0 VLE=1 SPT=0 GON=1 DTE=1 CM=0 D=3
     LCD_SET_REG(ILI9320_REG_DISPLAY_CTRL, 0x0233);
     //NO=0 SDT=0 EQ=0 DIV=0 SDIV=1 SRTN=1 RTN=9:25 clock
     LCD_SET_REG(ILI9320_REG_FRAME_CYCLE, 0x0039);
     //SCN=0
     LCD_SET_REG(ILI9320_REG_GATE_SCAN_START, 0x0000);
     
     //PKP1=7 PKP0=7
     LCD_SET_REG(ILI9320_REG_GAMMA_CTRL_1, 0x0707);
     //PKP3=2 PKP2=4
     LCD_SET_REG(ILI9320_REG_GAMME_CTRL_2, 0x0204);
     //PKP5=2 PKP4=2
     LCD_SET_REG(ILI9320_REG_GAMMA_CTRL_3, 0x0204);
     //PRP1=5 PRP0=2
     LCD_SET_REG(ILI9320_REG_GAMMA_CTRL_4, 0x0502);
     //PKN1=5 PKN0=7
     LCD_SET_REG(ILI9320_REG_GAMMA_CTRL_5, 0x0507);
     //PKN3=2 PNN2=4
     LCD_SET_REG(ILI9320_REG_GAMMA_CTRL_6, 0x0204);
     //PKN5=2 PKN4=4
     LCD_SET_REG(ILI9320_REG_GAMMA_CTRL_7, 0x0204);
     //PRN1=5 PRN0=2
     LCD_SET_REG(ILI9320_REG_GAMMA_CTRL_8, 0x0502);
     //VRP1=3 VRP0=2
     LCD_SET_REG(ILI9320_REG_GAMMA_CTRL_9, 0x0302);
     //VRN1=3 VRN0=2
     LCD_SET_REG(ILI9320_REG_GAMMA_CTRL_10, 0x0302);
     
     //WMR=0 WMG=0
     LCD_SET_REG(ILI9320_REG_WR_DATA_MASK_1, 0x0000);
     //WMB=0
     LCD_SET_REG(ILI9320_REG_WR_DATA_MASK_2, 0x0000);
     //OSC=b1010:548k
     LCD_SET_REG(ILI9320_REG_FRAME_FREQUENCY, 0xa000);
     //SS1=0
     LCD_SET_REG(ILI9320_REG_FIRST_WIN_START, 0x0000);
     //SE1=319
     LCD_SET_REG(ILI9320_REG_FIRST_WIN_END, (item->info->var.xres - 1));
     //SS2=0
     LCD_SET_REG(ILI9320_REG_SECND_WIN_START, 0x0000);
     //SE2=0
     LCD_SET_REG(ILI9320_REG_SECND_WIN_END, 0x0000);
     //VL1=0
     LCD_SET_REG(ILI9320_REG_V_SCROLL_CTRL_1, 0x0000);
     //VL2=0
     LCD_SET_REG(ILI9320_REG_V_SCROLL_CTRL_2, 0x0000);
     //HEA=0xef=239 HSA=0
     LCD_SET_REG(ILI9320_REG_H_RAM_ADR_POS, (item->info->var.yres - 1) << 8);
     //VSA=0
     LCD_SET_REG(ILI9320_REG_V_RAM_ADR_START, 0x0000);
     //VEA=0x13f=319
     LCD_SET_REG(ILI9320_REG_V_RAM_ADR_END, (item->info->var.xres - 1));
}
*/

static void inline ili9325_setup(struct ili9325 *item){
    /*** Display Setting ***/
    lcd_write_register(0x01, 0x0100);   //Driver output control (1)
    lcd_write_register(0x02, 0x0700);   //LCD driving control
    
    // Check page 55 in the datasheet for more information about alignment
#ifdef UPSIDEDOWN
    lcd_write_register(0x03, 0x1018);         //Entry mode
#else
    lcd_write_register(0x03, 0x1028);         //Entry mode
#endif
    
    lcd_write_register(0x04, 0x0000);   //Resize control
    lcd_write_register(0x08, 0x0207);   //Display control (2)
    lcd_write_register(0x09, 0x0000);   //Display control (3)
    
    lcd_write_register(0x0A, 0x0000);
    lcd_write_register(0x0C, 0x0000);
    lcd_write_register(0x0D, 0x0000);
    lcd_write_register(0x0F, 0x0000);
    msleep(50);
    
    /*** Power Control ***/
    lcd_write_register(0x10, 0x0000);
    lcd_write_register(0x11, 0x0007);   //power control 2 DC, VC
    lcd_write_register(0x12, 0x0000);   //power control 3 VRH
    lcd_write_register(0x13, 0x0000);   //power control 4 VCOM amplitude
    msleep(100);
    
//    lcd_write_register(0x07, 0x0101);   //power control 1 BT, AP
//    lcd_write_register(0x17, 0x0001);
//    msleep(20);
    
    lcd_write_register(0x10, 0x1490);             //power control 1 BT,AP
    lcd_write_register(0x11, 0x0227);             //power control 2 DC,VC
    msleep(50);
    lcd_write_register(0x12, 0x001c);             //power control 3 VRH
    msleep(50);
    lcd_write_register(0x13, 0x0A00);             //power control 4 vcom amplitude
    lcd_write_register(0x29, 0x000F);             //power control 7 VCOMH
    msleep(50);
    lcd_write_register(0x20, 0x0000);              //Horizontal GRAM Address Set
    lcd_write_register(0x21, 0x0000);              //Vertical GRAM Address Set

    /*** GAMMA Control ***/
    lcd_write_register(0x30, 0x0000);
    lcd_write_register(0x31, 0x0203);
    lcd_write_register(0x32, 0x0001);
    lcd_write_register(0x35, 0x0205);
    lcd_write_register(0x36, 0x030c);
    lcd_write_register(0x37, 0x0607);
    lcd_write_register(0x38, 0x0405);
    lcd_write_register(0x39, 0x0707);
    lcd_write_register(0x3c, 0x0502);
    lcd_write_register(0x3d, 0x1008);
    msleep(50);    
    
    lcd_write_register(0x50, 0x0000);              //Horizontal Address Start Position
    lcd_write_register(0x51, 0x00EF);              //Horizontal Address end Position (239)
    lcd_write_register(0x52, 0x0000);              //Vertical Address Start Position
    lcd_write_register(0x53, 0x013F);              //Vertical Address end Position (319)
    
    lcd_write_register(0x60, 0xA700);              //Driver Output Control 2
    lcd_write_register(0x61, 0x0001);              //Base Image Display Control
    lcd_write_register(0x6a, 0x0000);              //Base Image Display Control
    msleep(50);
    
    //-------------- Partial Display Control ---------//
    lcd_write_register(0x0080,0);
    lcd_write_register(0x0081,0);
    lcd_write_register(0x0082,0);
    lcd_write_register(0x0083,0);
    lcd_write_register(0x0084,0);
    lcd_write_register(0x0085,0);
    
    //-------------- Panel Control -------------------//
    lcd_write_register(0x90, 0x0010);              //Panel Interface Control 1
    lcd_write_register(0x92, 0x0600);              //Panel Interface Control 2
    lcd_write_register(0x93, 0x0003);              //Panel Interface Control 3
    lcd_write_register(0x95, 0x0110);
    lcd_write_register(0x97, 0x0000);
    lcd_write_register(0x98, 0x0000);
    msleep(50);
    
    // 262K color and display ON
    lcd_write_register(0x07, 0x0133);
    msleep(100);
}


/*
 * Setup 9320/9325 TFT controller
 */
static int __init ili932x_setup(struct ili9325 *item){
    int i = 0;
    int code = 0;

    printk(KERN_INFO "ili932x_fb:  Setup GPIO ports\n");
	ili9325_gpio_init();

    while((code = ili932x_init_controller()) == 0){
        if(i++ < 100){
            printk(KERN_INFO "ili932x_fb:  Initializing TFT controller failed, will retry in 3s later\n");
            msleep(3000);
        }else{
            printk(KERN_INFO "ili932x_fb:  Initialization aborted\n");
            return -1;
        }
    }
    
    switch(code){
        case 0x9320:
            ili9320_setup_1(item);
            //    ili9320_setup_2(item);
            //    ili9320_setup_3(item);
            break;
            
        case 0x9325:
            ili9325_setup(item);
            break;
        default:
            // unknow controller code
            printk(KERN_INFO "ili932x_fb:  Unsupported TFT controller, type code: %#08x\n",code);
            return -1;
    }
    return 0;
}

static inline void ili932x_clearscreen(struct ili9325 *item){
    int x;
    dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);
    //Clear screen
	ili9325_setptr(item, 0, 0); // LCD_CMD(0x0022) is called already
    ili932x_write_begin();
	for (x=0; x<320*240; x++){
        ili932x_write_unsafe(0,LCD_DATA);
    }
    ili932x_write_end();
    
	ili9325_setptr(item, 0, 0);
    ili932x_write_begin();
	for (x=0; x<320*240; x++){
        ili932x_write_unsafe(item->pages[0].buffer[x],LCD_DATA);
    }
    ili932x_write_end();
}

//This routine will allocate the buffer for the complete framebuffer. This
//is one continuous chunk of 16-bit pixel values; userspace programs
//will write here.
static int __init ili9325_video_alloc(struct ili9325 *item)
{
	unsigned int frame_size;

	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	frame_size = item->info->fix.line_length * item->info->var.yres;
	dev_dbg(item->dev, "%s: item=0x%p frame_size=%u\n",
		__func__, (void *)item, frame_size);

	item->pages_count = frame_size / PAGE_SIZE;
	if ((item->pages_count * PAGE_SIZE) < frame_size) {
		item->pages_count++;
	}
	dev_dbg(item->dev, "%s: item=0x%p pages_count=%u\n",
		__func__, (void *)item, item->pages_count);

	item->info->fix.smem_len = item->pages_count * PAGE_SIZE;
	item->info->fix.smem_start =
	    (unsigned long)vmalloc(item->info->fix.smem_len);
	if (!item->info->fix.smem_start) {
		dev_err(item->dev, "%s: unable to vmalloc\n", __func__);
		return -ENOMEM;
	}
	memset((void *)item->info->fix.smem_start, 0, item->info->fix.smem_len);

	return 0;
}

static void ili9325_video_free(struct ili9325 *item)
{
	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	kfree((void *)item->info->fix.smem_start);
}

//This routine will allocate a ili9325_page struct for each vm page in the
//main framebuffer memory. Each struct will contain a pointer to the page
//start, an x- and y-offset, and the length of the pagebuffer which is in the framebuffer.
static int __init ili9325_pages_alloc(struct ili9325 *item)
{
	unsigned short pixels_per_page;
	unsigned short yoffset_per_page;
	unsigned short xoffset_per_page;
	unsigned int index;
	unsigned short x = 0;
	unsigned short y = 0;
	unsigned short *buffer;
	unsigned short *oldbuffer;
	unsigned int len;

	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	item->pages = kmalloc(item->pages_count * sizeof(struct ili9325_page),
			      GFP_KERNEL);
	if (!item->pages) {
		dev_err(item->dev, "%s: unable to kmalloc for ili9325_page\n",
			__func__);
		return -ENOMEM;
	}

	pixels_per_page = PAGE_SIZE / (item->info->var.bits_per_pixel / 8);
	yoffset_per_page = pixels_per_page / item->info->var.xres;
	xoffset_per_page = pixels_per_page -
	    (yoffset_per_page * item->info->var.xres);
	dev_dbg(item->dev, "%s: item=0x%p pixels_per_page=%hu "
		"yoffset_per_page=%hu xoffset_per_page=%hu\n",
		__func__, (void *)item, pixels_per_page,
		yoffset_per_page, xoffset_per_page);

	oldbuffer = kmalloc(item->pages_count * pixels_per_page * 2,
			      GFP_KERNEL);
	if (!oldbuffer) {
		dev_err(item->dev, "%s: unable to kmalloc for ili9325_page oldbuffer\n",
			__func__);
		return -ENOMEM;
	}

	buffer = (unsigned short *)item->info->fix.smem_start;
	for (index = 0; index < item->pages_count; index++) {
		len = (item->info->var.xres * item->info->var.yres) -
		    (index * pixels_per_page);
		if (len > pixels_per_page) {
			len = pixels_per_page;
		}
		dev_dbg(item->dev,
			"%s: page[%d]: x=%3hu y=%3hu buffer=0x%p len=%3hu\n",
			__func__, index, x, y, buffer, len);
		item->pages[index].x = x;
		item->pages[index].y = y;
		item->pages[index].buffer = buffer;
		item->pages[index].oldbuffer = oldbuffer;
		item->pages[index].len = len;

		x += xoffset_per_page;
		if (x >= item->info->var.xres) {
			y++;
			x -= item->info->var.xres;
		}
		y += yoffset_per_page;
		buffer += pixels_per_page;
		oldbuffer += pixels_per_page;
	}

	return 0;
}

static void ili9325_pages_free(struct ili9325 *item)
{
	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	kfree(item->pages);
}

static inline __u32 CNVT_TOHW(__u32 val, __u32 width)
{
	return ((val<<width) + 0x7FFF - val)>>16;
}

//This routine is needed because the console driver won't work without it.
static int ili9325_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;
			u32 value;

			red = CNVT_TOHW(red, info->var.red.length);
			green = CNVT_TOHW(green, info->var.green.length);
			blue = CNVT_TOHW(blue, info->var.blue.length);
			transp = CNVT_TOHW(transp, info->var.transp.length);

			value = (red << info->var.red.offset) |
				(green << info->var.green.offset) |
				(blue << info->var.blue.offset) |
				(transp << info->var.transp.offset);

			pal[regno] = value;
			ret = 0;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}
	return ret;
}

static int ili9325_blank(int blank_mode, struct fb_info *info)
{
	struct ili9325 *item = (struct ili9325 *)info->par;
	if (blank_mode == FB_BLANK_UNBLANK)
		item->backlight=1;
	else
		item->backlight=0;
	//Item->backlight won't take effect until the LCD is written to. Force that
	//by dirty'ing a page.
	item->pages[0].must_update=1;
	schedule_delayed_work(&info->deferred_work, 0);
	return 0;
}

static void ili9325_touch(struct fb_info *info, int x, int y, int w, int h) 
{
	struct fb_deferred_io *fbdefio = info->fbdefio;
	struct ili9325 *item = (struct ili9325 *)info->par;
	int i, ystart, yend;
	if (fbdefio) {
		//Touch the pages the y-range hits, so the deferred io will update them.
		for (i=0; i<item->pages_count; i++) {
			ystart=item->pages[i].y;
			yend=item->pages[i].y+(item->pages[i].len/info->fix.line_length)+1;
			if (!((y+h)<ystart || y>yend)) {
				item->pages[i].must_update=1;
			}
		}
		//Schedule the deferred IO to kick in after a delay.
		schedule_delayed_work(&info->deferred_work, fbdefio->delay);
	}
}

static void ili9325_fillrect(struct fb_info *p, const struct fb_fillrect *rect) 
{
	sys_fillrect(p, rect);
	ili9325_touch(p, rect->dx, rect->dy, rect->width, rect->height);
}

static void ili9325_imageblit(struct fb_info *p, const struct fb_image *image) 
{
	sys_imageblit(p, image);
	ili9325_touch(p, image->dx, image->dy, image->width, image->height);
}

static void ili9325_copyarea(struct fb_info *p, const struct fb_copyarea *area) 
{
	sys_copyarea(p, area);
	ili9325_touch(p, area->dx, area->dy, area->width, area->height);
}

static ssize_t ili9325_write(struct fb_info *p, const char __user *buf, 
				size_t count, loff_t *ppos) 
{
	ssize_t res;
	res = fb_sys_write(p, buf, count, ppos);
	ili9325_touch(p, 0, 0, p->var.xres, p->var.yres);
	return res;
}

static struct fb_ops ili9325_fbops = {
	.owner        = THIS_MODULE,
	.fb_read      = fb_sys_read,
	.fb_write     = ili9325_write,
	.fb_fillrect  = ili9325_fillrect,
	.fb_copyarea  = ili9325_copyarea,
	.fb_imageblit = ili9325_imageblit,
	.fb_setcolreg	= ili9325_setcolreg,
	.fb_blank	= ili9325_blank,
};

static struct fb_fix_screeninfo ili9325_fix __initdata = {
	.id          = "ILI9325",
	.type        = FB_TYPE_PACKED_PIXELS,
	.visual      = FB_VISUAL_TRUECOLOR,
	.accel       = FB_ACCEL_NONE,
	.line_length = 320 * 2,
};

static struct fb_var_screeninfo ili9325_var __initdata = {
	.xres		= 320,
	.yres		= 240,
	.xres_virtual	= 320,
	.yres_virtual	= 240,
	.width		= 320,
	.height		= 240,
	.bits_per_pixel	= 16,
	.red		= {11, 5, 0},
	.green		= {5, 6, 0},
	.blue		= {0, 5, 0},
	.activate	= FB_ACTIVATE_NOW,
	.vmode		= FB_VMODE_NONINTERLACED,
};

static struct fb_deferred_io ili9325_defio = {
        .delay          = HZ / 25,
        .deferred_io    = &ili9325_update,
};

static int __init ili9325_probe(struct platform_device *dev)
{
	int ret = 0;
	struct ili9325 *item;
	struct fb_info *info;

	dev_dbg(&dev->dev, "%s\n", __func__);

	printk(KERN_INFO "ili932x_fb: Driver start up\n");
    
	item = kzalloc(sizeof(struct ili9325), GFP_KERNEL);
	if (!item) {
		dev_err(&dev->dev,
			"%s: unable to kzalloc for ili9325\n", __func__);
		ret = -ENOMEM;
		goto out;
	}
	item->dev = &dev->dev;
	dev_set_drvdata(&dev->dev, item);
	item->backlight=1;

    // Prepare framebuffer
	info = framebuffer_alloc(sizeof(struct ili9325), &dev->dev);
	if (!info) {
		ret = -ENOMEM;
		dev_err(&dev->dev,
			"%s: unable to framebuffer_alloc\n", __func__);
		goto out_item;
	}
	info->pseudo_palette = &item->pseudo_palette;
	item->info = info;
	info->par = item;
	info->dev = &dev->dev;
	info->fbops = &ili9325_fbops;
	info->flags = FBINFO_FLAG_DEFAULT|FBINFO_VIRTFB;
	info->fix = ili9325_fix;
	info->var = ili9325_var;

	ret = ili9325_video_alloc(item);
	if (ret) {
		dev_err(&dev->dev,
			"%s: unable to ili9325_video_alloc\n", __func__);
		goto out_info;
	}
	info->screen_base = (char __iomem *)item->info->fix.smem_start;

	ret = ili9325_pages_alloc(item);
	if (ret < 0) {
		dev_err(&dev->dev,
			"%s: unable to ili9325_pages_init\n", __func__);
		goto out_video;
	}

	info->fbdefio = &ili9325_defio;
	fb_deferred_io_init(info);

    // setup the controller
    if(0 != ili932x_setup(item)){
        printk(KERN_WARNING "ili932x_fb:  No ili932x TFT/LCD controller found\n");
        goto out_pages;
    }
    
    // driver initialized
    printk(KERN_INFO "ili932x_fb: Clear screen\n");
    ili932x_clearscreen(item);
    ili9325_update_all(item);

    // now we can register framebuffer
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&dev->dev,
			"%s: unable to register_frambuffer\n", __func__);
		goto out_pages;
	}

	return ret;

out_pages:
	ili9325_pages_free(item);
out_video:
	ili9325_video_free(item);
out_info:
	framebuffer_release(info);
out_item:
	kfree(item);
out:
	return ret;
}


static int ili9325_remove(struct platform_device *dev)
{
	struct fb_info *info = dev_get_drvdata(&dev->dev);
	struct ili9325 *item = (struct ili9325 *)info->par;
    if(item){
        unregister_framebuffer(info);
        ili9325_pages_free(item);
        ili9325_video_free(item);
        framebuffer_release(info);
        kfree(item);
    }
	return 0;
}

#ifdef CONFIG_PM
static int ili9325_suspend(struct platform_device *dev, pm_message_t state)
{
//	struct fb_info *info = dev_get_drvdata(&spi->dev);
//	struct ili9325 *item = (struct ili9325 *)info->par;
	/* enter into sleep mode */
//	ili9325_reg_set(item, ILI9325_REG_SLEEP_MODE, 0x0001);
	return 0;
}

static int ili9325_resume(struct platform_device *dev)
{
//	struct fb_info *info = dev_get_drvdata(&spi->dev);
//	struct ili9325 *item = (struct ili9325 *)info->par;
	/* leave sleep mode */
//	ili9325_reg_set(item, ILI9325_REG_SLEEP_MODE, 0x0000);
	return 0;
}
#else
#define ili9325_suspend NULL
#define ili9325_resume NULL
#endif

static struct platform_driver ili9325_driver = {
	.probe = ili9325_probe,
	.driver = {
		   .name = "ili9325",
		   },
};

static int __init ili9325_init(void)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	ret = platform_driver_register(&ili9325_driver);
	if (ret) {
		pr_err("%s: unable to platform_driver_register\n", __func__);
	}

	return ret;
}

module_init(ili9325_init);
module_remove(ili9325_remove);


MODULE_DESCRIPTION("ILI932X LCD Driver");
MODULE_AUTHOR("Jeroen Domburg <jeroen@spritesmods.com>, Shawn Chain <shawn.chain@gmail.com>");
MODULE_LICENSE("GPL");
