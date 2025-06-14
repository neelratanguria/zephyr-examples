
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>


#define I2C0_NODE DT_NODELABEL(mysensor)

#define ADXL345_I2C_ADDR 0x53
#define ADXL345_REG_DEVID     0x00
#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_REG_DATA_X0   0x32

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

// Pins
// SDA: P0.26
// SCL: P0.27

/*
 * 📌 Hardware Connections: ADXL345 to nRF52833 DK (I2C Mode)
 *
 * ADXL345 Pin    Function       Connected to nRF52833 DK Pin
 * ----------------------------------------------------------
 * VCC            Power          VDD (3.3V)
 * GND            Ground         GND
 * SDA            I2C Data       P0.26  // or any available GPIO
 * SCL            I2C Clock      P0.27  // or any available GPIO
 * CS             Mode Select    VDD    // Tie HIGH to enable I2C mode
 * SDO            Addr Select    GND    // GND = I2C address 0x53, VDD = 0x1D
 * INT1, INT2     Interrupts     (Optional) Connect to any GPIO if used
 */


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* STEP 8 - Define the I2C slave device address and the addresses of relevant registers */

#define CHIP_ID 0x60
#define SENSOR_CONFIG_VALUE 0x93

/* STEP 6 - Get the node identifier of the sensor */

// I2C write to ADXL345
int adxl345_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_write(dev_i2c.bus, buf, 2, ADXL345_I2C_ADDR);
}

// I2C read from ADXL345
int adxl345_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_write_read(dev_i2c.bus, dev_i2c.addr, &reg, 1, val, 1);
}

// I2C read multiple
int adxl345_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t reg = ADXL345_REG_DATA_X0;
    uint8_t buf[6];
    int ret = i2c_write_read(dev_i2c.bus, ADXL345_I2C_ADDR, &reg, 1, buf, 6);
    if (ret != 0) return ret;

    *x = (int16_t)((buf[1] << 8) | buf[0]);
    *y = (int16_t)((buf[3] << 8) | buf[2]);
    *z = (int16_t)((buf[5] << 8) | buf[4]);
    return 0;
}

int main(void)
{
	if (!device_is_ready(dev_i2c.bus))
	{
		printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
		return 0;
	}

	printk("I2C bus %s ready!\n\r", dev_i2c.bus->name);

	int ret;

	printk("Read writing!\n\r");

	uint8_t devid = 0;
    if (adxl345_read_reg(ADXL345_REG_DEVID, &devid) == 0) {
        printk("Device ID: 0x%X\n\r", devid);
    } else {
        printk("Failed to read Device ID");
        
    }

	// Set measurement mode
    adxl345_write_reg(ADXL345_REG_POWER_CTL, 0x08);
	

	while (1)
	{
		int16_t x, y, z;
        if (adxl345_read_xyz(&x, &y, &z) == 0) {
            printk("X: %d, Y: %d, Z: %d \n\r", x, y, z);
        } else {
            printk("Failed to read acceleration");
        }

        k_msleep(1000);
	}
}
