#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define I2C_SLAVE_RDWR	0x0712

#define BUFF_FULL		0xff00	/* for AST I2C slave */
#define I2C_S_EN			0x0002	/* Slave mode enable */
#define I2C_S_ALT		0x0004	/* Slave issue alert */

#define I2C_S_BUF_SIZE 	256
#define ADC_DIR "/sys/devices/platform/ast-adc.0"
#define ADC_VALUE "adc%d_value"
#define TACHO_DIR "/sys/devices/platform/ast_pwm_tacho.0"
#define TACHO_VALUE "tacho%d_rpm"

#define MAX_DVNAME_LEN	128
int i2c_write(int i2c_dev, int addr, char value)
{
        unsigned char val[2] = {0};

        val[0] = addr;
        val[1] = value;

        write(i2c_dev, val, sizeof(val));
        return 0;
}

int i2c_read(int i2c_dev, int addr)
{
        unsigned char val[2] = {0};

        val[0] = addr;

        write(i2c_dev, &val[0], 1);
        read(i2c_dev, &val[1], 1);

        return val[1];
}
int i2c_readw(int i2c_dev, int addr)
{
        unsigned char val[2] = {0};
	unsigned char rval[2]={0};
	int ret;

        val[0] = addr;      
                            
        write(i2c_dev, &val[0], 1);
        read(i2c_dev, &rval[0], 2);

	ret = rval[0]*256+rval[1];
        return ret;
}
static int
read_device_float(const char *device, float *value) {
  FILE *fp;
  int rc;
  char tmp[10];

  fp = fopen(device, "r");
  if (!fp) {
    int err = errno;
    printf("Device %s open Error!\n", device);
    return err;
  }

  rc = fscanf(fp, "%s", tmp);
  fclose(fp);

  if (rc != 1) {
    printf("Device %s read Error!\n", device);

    return ENOENT;
  }

//  printf("ADC Read Data =%s\n", tmp);
  *value = atof(tmp);

  return 0;
}

static int
read_adc_value(const int pin, float *value) {
  char device_name[MAX_DVNAME_LEN]={0};
  char full_name[MAX_DVNAME_LEN]={0};


  snprintf(device_name, MAX_DVNAME_LEN, ADC_VALUE, pin);
  //printf("device name : %s\n", device_name);
  snprintf(full_name, MAX_DVNAME_LEN, "%s/%s", ADC_DIR, device_name);
  //printf("full name : %s\n", full_name);

  return read_device_float(full_name, value);
}

static int
read_tacho_value(const int pin, float *value) {
  char device_name[MAX_DVNAME_LEN]={0};
  char full_name[MAX_DVNAME_LEN]={0};


  snprintf(device_name, MAX_DVNAME_LEN, TACHO_VALUE, pin);
  //printf("device name : %s\n", device_name);
  snprintf(full_name, MAX_DVNAME_LEN, "%s/%s", TACHO_DIR, device_name);
  //printf("full name : %s\n", full_name);

  return read_device_float(full_name, value);
}

int set_adc_enable(void)
{
        char buf[200];
        int i;

        fprintf(stdout,"adc enable \n");

    for (i = 0; i < 15; i++)
    {
                sprintf(buf,"echo \"1 : Enable\" > /sys/devices/platform/ast-adc.0/adc%d_en", i);
                system(buf);
                //fprintf(stdout,"%s \n", buf);         
    }

        return 1;
}

int set_tacho_enable(void)
{
        char buf[200];
        int i;

        fprintf(stdout,"tacho enable \n");

    for (i = 0; i < 16; i++)
    {
                sprintf(buf,"echo \"1 : Enable\" > /sys/devices/platform/ast_pwm_tacho.0/tacho%d_en", i);
                system(buf);
                //fprintf(stdout,"%s \n", buf);         
    }

        return 1;
}


int set_pwm_enable(void)
{
        char buf[200];
        int i;

        fprintf(stdout,"pwm enable \n");

    for (i = 0; i < 8; i++)
    {
                sprintf(buf,"echo \"1 : Enable\" > /sys/devices/platform/ast_pwm_tacho.0/pwm%d_en", i);
                system(buf);
                //fprintf(stdout,"%s \n", buf);         
    }

        return 1;
}

int set_pwm_disable(void)
{
        char buf[200];
        int i;

        fprintf(stdout,"pwm disable \n");

    for (i = 0; i < 8; i++)
    {
                sprintf(buf,"echo \"0 : Disable\" > /sys/devices/platform/ast_pwm_tacho.0/pwm%d_en", i);
                system(buf);
                //fprintf(stdout,"%s \n", buf); 
    }
        return 1;
}

int set_fan_speed(int fanlevel)
{
        char buf[200];
        int i;

    //if (fanlevel < 0) fanlevel = 0;
        //else if (fanlevel > 9) fanlevel = 9;
        //fprintf(stdout,"fan speed level(0~10): %d \n", 9-fanlevel);

    for (i = 0; i < 8; i++)
    {
                sprintf(buf,"echo \"%d : unit limit\" > /sys/devices/platform/ast_pwm_tacho.0/pwm%d_rising", fanlevel, i);
                system(buf);
                //fprintf(stdout,"%s \n", buf);
    }
}


void main (void) {
	int file, i, len, ret;
	char filename[40];
	unsigned char rdbuf[I2C_S_BUF_SIZE], wrbuf[1];
	struct i2c_msg msg[2];
	struct i2c_rdwr_ioctl_data rdwr_msgs;
	char ch;
	float adc_val;


        set_fan_speed(9);

        set_pwm_enable();

	set_adc_enable();
	
	set_tacho_enable();

	sprintf(filename,"/dev/i2c-6");
	if ((file = open(filename,O_RDWR)) < 0) {
		printf("Failed to open the bus.");
		/* ERROR HANDLING; you can check errno to see what went wrong */
		exit(1);
	}
	while(1){
                if (ioctl(file,I2C_SLAVE_FORCE, 0x48) < 0) {
                        /* ERROR HANDLING; you can check errno to see what went wrong */
                        //fprintf(stdout,"Can not set slave address 0x%02X\n", I2C_ADDR);
                }

                ret = i2c_readw(file, 0);
		ret >>= 4;
                ret = ret*128/0x7ff;
		printf("bus = 6, addr = 0x48, regs = 0x00, Temp1=%d C\n", ret);
		sleep(1);
                if (ioctl(file,I2C_SLAVE_FORCE, 0x49) < 0) {
                        /* ERROR HANDLING; you can check errno to see what went wrong */
                        //fprintf(stdout,"Can not set slave address 0x%02X\n", I2C_ADDR);
                }
                ret = i2c_readw(file, 0);
                ret >>= 4;
                ret = ret*128/0x7ff;
                printf("bus = 6, addr = 0x49, regs = 0x00, Temp2=%d C\n", ret);
		sleep(1);
                if (ioctl(file,I2C_SLAVE_FORCE, 0x4a) < 0) {
                        /* ERROR HANDLING; you can check errno to see what went wrong */
                        //fprintf(stdout,"Can not set slave address 0x%02X\n", I2C_ADDR);
                }
                ret = i2c_readw(file, 0);
                ret >>= 4;
                ret = ret*128/0x7ff;
                printf("bus = 6, addr = 0x4a, regs = 0x00, Temp3=%d C\n", ret);
		sleep(1);
                if (ioctl(file,I2C_SLAVE_FORCE, 0x4c) < 0) {
                        /* ERROR HANDLING; you can check errno to see what went wrong */
                        //fprintf(stdout,"Can not set slave address 0x%02X\n", I2C_ADDR);
                }
                ret = i2c_readw(file, 0);
                ret >>= 4;
                ret = ret*128/0x7ff;
                printf("bus = 6, addr = 0x4c, regs = 0x00, Temp4=%d C\n", ret);
		sleep(1);
                if (ioctl(file,I2C_SLAVE_FORCE, 0x4f) < 0) {
                        /* ERROR HANDLING; you can check errno to see what went wrong */
                        //fprintf(stdout,"Can not set slave address 0x%02X\n", I2C_ADDR);
                }
                ret = i2c_readw(file, 0);
                ret >>= 4;
                ret = ret*128/0x7ff;
                printf("bus = 6, addr = 0x4f, regs = 0x00, Temp5=%d C\n", ret);
		sleep(1);
		for (i=0; i<16; i++){
			read_adc_value(i, (float*)&adc_val);
			printf("ADC%d data=%f\n",i, adc_val);
			usleep(1000000);
		}
                for (i=0; i<16; i++){
                        read_tacho_value(i, (float*)&adc_val);
                        printf("TACHO%d RPM data=%f\n",i, adc_val);
                        usleep(1000000);
                }

        }
	close (file);
	/* master read, slave write */
	
	/* slave alt */
}


