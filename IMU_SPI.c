#include <linux/spi/spi.h>
#include "IMU_SPI.h"
#include <linux/kernel.h>

extern struct spi_device *spi_device;

int whoAmI(void)
{
	printk("Mpu9250::whoAmI()=%x\n",read(WHO_AM_I));
	if(read(WHO_AM_I) == WHO_AM_I_VALUE )
		return 0;
	return -1;
}

unsigned char read(unsigned char addr)
{
	unsigned char tx,rx;
	tx = (addr|0x80);//the first bit of the tranfered data is 1
	rx = spi_w8r8(spi_device,0x80|WHO_AM_I);
	return rx;
}

void write(unsigned char addr,unsigned char value)
{
	unsigned char tx[2];
	tx[0] = addr&0x7F; tx[1] = value;//the first bit of the tranfered data is 0
	spi_write(spi_device, tx, sizeof(tx));
}

void readMultiple(unsigned char addr,int len,unsigned char *data)
{
	unsigned char  tx[2];
    tx[0] = addr|0x80;
	spi_write_then_read(spi_device, tx, 1, data, len);
}

void InitIMU(int mode)
{  
	unsigned char c;
	// wake up device
	write(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
	mdelay(10); // Wait for all registers to reset 

	// get stable time source
	write(PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	mdelay(20); 

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	write(CONFIG, 0x03);  
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	write(SMPLRT_DIV, 0x01);  		// Use a 200 Hz rate; a rate consistent with the filter update rate 
		                            // determined inset in CONFIG above
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3

	c = read(GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5] 
	c = c & ~0x02; // Clear Fchoice bits [1:0] 
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | (Gscale * 8); // Set full scale range for the gyro
	// c =| 0x00; // SeinitMagt Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	write(GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	c = read(ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5] 
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | (Ascale * 8) ; // Set full scale range for the accelerometer 
	write(ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = read(ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	write(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
				
	write(INT_PIN_CFG, 0x22);    
	write(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	mdelay(10);

	write(USER_CTRL, 0x30);    
#if MPU9250		
	write(I2C_MST_CTRL, 0x0D);    
	write(I2C_SLV0_ADDR, AK8963_ADDRESS);    

	if(mode == 0)
    {
	    write(I2C_SLV0_REG, AK8963_CNTL);    
	    write(I2C_SLV0_DO, 0x1F);    //0x16 mode 2;;0x1F mode fuse;;
	    write(I2C_SLV0_CTRL, 0x81); 
	}
	else if(mode ==1)
    {
    	write(I2C_SLV0_REG, AK8963_CNTL);    
    	write(I2C_SLV0_DO, 0x12);    //0x16 mode 2;;0x1F mode fuse;;
    	write(I2C_SLV0_CTRL, 0x81); 
    }
#endif
	mdelay(100);
}


#define MY_BUS_NUM 1

int spi_init(void)
{
    int ret;
    struct spi_master *master;
     
    //Register information about your slave device:
    struct spi_board_info spi_device_info = {
        .modalias = "my-device-driver-name",
        .max_speed_hz = 1000000, //speed your device (slave) can handle
        .bus_num = MY_BUS_NUM,
        .chip_select = 0,
        .mode = 3,//CPHA|CPOL
    };
     
    /*To send data we have to know what spi port/pins should be used. This information 
      can be found in the device-tree. */
    master = spi_busnum_to_master( spi_device_info.bus_num );
    if( !master ){
        printk("MASTER not found.\n");
            return -ENODEV;
    }
     
    // create a new slave device, given the master and device info
    spi_device = spi_new_device( master, &spi_device_info );
 
    if( !spi_device ) {
        printk("FAILED to create slave.\n");
        return -ENODEV;
    }
     
    spi_device->bits_per_word = 8;
 
    ret = spi_setup( spi_device );
     
    if( ret ){
        printk("FAILED to setup slave.\n");
        spi_unregister_device( spi_device );
        return -ENODEV;
    }
    return 0;
}

void spi_deinit(void)
{
    unsigned char ch = 0Xff;
 
    if( spi_device ){
        spi_write(spi_device, &ch, sizeof(ch));
        spi_unregister_device( spi_device );
    }
}