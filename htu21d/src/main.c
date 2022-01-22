/* Temperature and Humidity sensor reading */

#include <drivers/gpio.h>
#include <devicetree.h>
#include <sys/util.h>
#include <inttypes.h>
#include <stdio.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#define SLEEP_TIME_MS 2000
#define HTU21D_I2C_ADDR	0x40

#define SW0_NODE        DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
		                                                              {0});
static struct gpio_callback button_cb_data;

static struct k_timer my_timer;


void sensor_reading(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	uint8_t buf[3];
	int i, ret;
	uint16_t sensor_data;
	double sensor_tmp;
	double result;
	if (!device_is_ready(i2c_dev)) 
	{
		printk("I2C: Device is not ready.\n");
		return;
	}

	buf[0] = 0xE3;
	ret = i2c_write(i2c_dev,&buf[0],1,HTU21D_I2C_ADDR);
	if (ret) 
	{
		printk("Error writing to HTU21D! error code (%d)\n", ret);
		return;
	} 
	else 
	{
		printk("Wrote 0xE3 to address 0x00.\n");
	}



	buf[3] = 0;
	
	/* for read the values in sensor */

	ret = i2c_read(i2c_dev,&buf,3,HTU21D_I2C_ADDR);
	if (ret) 
	{
	printk("Error reading from HTU21D! error code (%d)\n", ret);
		return;
	} 
	else 
	{
		printk("Read 0x%X from address 0x00.\n", buf[0]);
	}

// device response, 14-bit ADC value:
//  first 8 bit part ACK  second 8 bit part        CRC
// [0 1 2 3 4 5 6 7] [8] [9 10 11 12 13 14 15 16] [17 18 19 20 21 22 23 24]
// bit 15 - measurement type (‘0’: temperature, ‘1’: humidity)
// bit 16 - currently not assigned
	 
	sensor_data = (buf [0] << 8 | buf [1]) & 0xFFFC;



	/* conversion */

	 sensor_tmp = sensor_data / 65536.0;
	 result = -46.85 + (175.72 * sensor_tmp);

	printk("Temperature: %.2f C\n", result);
	result = -6.0 + (125.0 * sensor_tmp);

	printf("Humidity: %.2f %%\n", result);
}


K_WORK_DEFINE(my_work, sensor_reading);

static void my_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&my_work);
}


void button_pressed(const struct device *dev, struct gpio_callback *cb,
		                    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());

      k_timer_init(&my_timer,my_timer_handler,NULL);
      k_timer_start(&my_timer,K_SECONDS(2),K_MSEC(1000));
}


void func(void)
{
	       int ret;
     if (!device_is_ready(button.port))
                {
		printk("Error: button device %s is not ready\n",button.port->name);
		return;
		}
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
        if (ret != 0)
                {
                printk("Error %d: failed to configure %s pin %d\n",ret, button.port->name, button.pin);
                return;
                }
        ret = gpio_pin_interrupt_configure_dt(&button,GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0)
                {
                printk("Error %d: failed to configure interrupt on %s pin %d\n",ret, button.port->name, button.pin);
                return;
                }

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
        gpio_add_callback(button.port, &button_cb_data);
	        printk("Set up button at %s pin %d\n", button.port->name, button.pin);
		
		
					
}

SYS_INIT(func, APPLICATION, 2);

