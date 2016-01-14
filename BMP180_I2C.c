/**
 * BMP180 pressure & temperature sensor on Beaglebone black Rev C using I2C bus
 *
 * OS: Linux beaglebone 3.8.13-bone70 #1 SMP Fri Jan 23 02:15:42 UTC 2015 armv7l GNU/Linux
 *
 * The i2c bus is on pins P9_19 as SCL, P9_20 as SDA
 *
 * The code is partly converted from https://arduinodiy.wordpress.com/2015/05/26/reading-the-bmp180-pressure-sensor-with-an-attiny85/
 *
 * Author: Yingfeng Shen, yfshen@gmail.com
 * Date: 1.2016
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define I2C_BUS   	1
#define DEV_ADDR   	0b1110111	// 0x77
#define RET(msg,r)	{fprintf(stderr,msg); return r;}

// define calibration data for temperature:
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;

long x1, x2, b5;

// Oversampling Setting
const unsigned char OSS = 0;

int i2c_open(int bus, uint8_t dev_addr, int timeout){
	char path[64];
	int	fd;

	sprintf(path, "/dev/i2c-%d", bus);
	fd = open(path, O_SYNC | O_RDWR);
	if (fd<0) RET("failed to open i2c device",-1);
	ioctl(fd,I2C_SLAVE, dev_addr);
	ioctl(fd,I2C_TIMEOUT,timeout);	// unit 10 ms

	return fd;
}

void i2c_close(int fd){
	if (fd>0) close(fd);

	fd=0;
}

int bmp180ReadInt(int fd, unsigned char address) {
	uint8_t data[2];

	data[0] = address;
	write(fd, data, 1); // write address

	if (read(fd, data, 2)!=2) RET("error in bmp180ReadInt",0);

	return (int) (data[0] << 8 | data[1]);
}

void readBmp180Calibration(int fd) {
	// Read calibration data from EEPROM
	ac1 = bmp180ReadInt(fd,0xAA);
	ac2 = bmp180ReadInt(fd,0xAC);
	ac3 = bmp180ReadInt(fd,0xAE);
	ac4 = bmp180ReadInt(fd,0xB0);
	ac5 = bmp180ReadInt(fd,0xB2);
	ac6 = bmp180ReadInt(fd,0xB4);
	b1 = bmp180ReadInt(fd,0xB6);
	b2 = bmp180ReadInt(fd,0xB8);
	mb = bmp180ReadInt(fd,0xBA);
	mc = bmp180ReadInt(fd,0xBC);
	md = bmp180ReadInt(fd,0xBE);
}

unsigned int bmp180ReadUT(int fd) {
	uint8_t data[8];
	int ret;

	data[0] = 0xf4;
	data[1] = 0x2e;
	ret = write(fd, data, 2);
	usleep(5000);

	data[0] = 0xf6; // read MSB first
	ret = write(fd, data, 1);

	// read temperature
	ret = read(fd, data, 2);

	if (ret != 2) {
		RET("Read temperature failed\n",0);
	}

	return (int) (data[0] << 8 | data[1]);
}

double bmp180CorrectTemperature(unsigned int ut) {
	x1 = (((long) ut - (long) ac6) * (long) ac5) >> 15;
	x2 = ((long) mc << 11) / (x1 + md);
	b5 = x1 + x2;

	return (((b5 + 8) >> 4)) / 10.0;
}

unsigned long bmp180ReadUP(int fd) {
	unsigned char msb, lsb, xlsb;
	unsigned long up = 0;
	uint8_t data[8];
	int ret;

	// Write 0x34+(OSS<<6) into register 0xF4
	// Request a pressure reading w/ oversampling setting
	data[0] = 0xf4;
	data[1] = 0x34 + (OSS << 6);
	ret = write(fd, data, 2); // write address
	usleep(5000 * (1 + OSS));

	// Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
	data[0] = 0xf6;
	ret = write(fd, data, 1);

	ret = read(fd, data, 3);
	if (ret != 3) {
		RET("Read pressure failed\n",0);
	}
	msb = data[0];
	lsb = data[1];
	xlsb = data[2];

	up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8)
			| (unsigned long) xlsb) >> (8 - OSS);

	return up;
}

double bmp180CorrectPressure(unsigned long up) {
	long x3, b3, b6, p;
	unsigned long b4, b7;

	b6 = b5 - 4000;
	// Calculate B3
	x1 = (b2 * (b6 * b6) >> 12) >> 11;
	x2 = (ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((long) ac1) * 4 + x3) << OSS) + 2) >> 2;

	// Calculate B4
	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;

	b7 = ((unsigned long) (up - b3) * (50000 >> OSS));
	if (b7 < 0x80000000)
		p = (b7 << 1) / b4;
	else
		p = (b7 / b4) << 1;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;

	return p;
}

int main(int argc, char* argv[]) {
	int bmp180=1;			// device file descriptor
	time_t rawtime;
	struct tm * timeinfo;
	struct timeval tv;
	double t0, t;
	double temperature, pressure, altitude, p0=101325.0,interval=0.5;
	int i,n = 0;
	FILE *fp=NULL;

	if (argc > 2){
		interval = atof(argv[1]);
		n = (int)(atof(argv[2])/interval+0.5);
		if (argc>3) fp=fopen(argv[3],"w");
	}

	time(&rawtime);
	timeinfo = localtime(&rawtime);
    gettimeofday(&tv, NULL);

	// open i2c device, timeout 10ms
	bmp180 = i2c_open(I2C_BUS, DEV_ADDR,1);

	if (bmp180 < 0) {
		RET("Failed to get I2C device BMP180!\n",0);
	}

	readBmp180Calibration(bmp180);

	t0 = (timeinfo->tm_hour * 60 + timeinfo->tm_min) * 60 + timeinfo->tm_sec+tv.tv_usec/1.0e6; // seconds

	if (n == 0) {
		temperature = bmp180CorrectTemperature(bmp180ReadUT(bmp180));
		pressure = bmp180CorrectPressure(bmp180ReadUP(bmp180));
		altitude = 44330.0*(1-pow(pressure/p0,1/5.255));
		printf("%d/%d/%d %d:%d:%d %.1lfC %.1lfPa %.1lfm\n", timeinfo->tm_year + 1900,
				timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour,
				timeinfo->tm_min, timeinfo->tm_sec, temperature, pressure, altitude);
	}

	t = t0;
	for (i = 0; i < n; i++) {
		temperature = bmp180CorrectTemperature(bmp180ReadUT(bmp180));
		pressure = bmp180CorrectPressure(bmp180ReadUP(bmp180));
		altitude = 44330.0*(1-pow(pressure/p0,1/5.255));
		fprintf(stdout, "%.1lf %.1lf %.1lf\n", t - t0, temperature, pressure);
		if (fp) fprintf(fp, "%.1lf %.1lf %.1lf\n", t - t0, temperature, pressure);
		while((t-t0)<(i+1)*interval){
			usleep(10);
			time(&rawtime);
			timeinfo = localtime(&rawtime);
			gettimeofday(&tv, NULL);
			t = (timeinfo->tm_hour * 60 + timeinfo->tm_min) * 60 + timeinfo->tm_sec+tv.tv_usec/1e6; // second
		}
	}

	i2c_close(bmp180);	// close device
	if (fp) fclose(fp);	// close log file

	return 1;
}
