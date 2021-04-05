#
# Makefile for the sensor drivers.
#

# Each configuration option enables a list of files.
obj-$(CONFIG_SENSORS_SSP)		+= sensors_core.o ssp_dev.o ssp_comm.o ssp_data.o ssp_sysfs.o \
						ssp_iio.o ssp_firmware.o ssp_debug.o ssp_sensorhub.o \
						ssp_sensor_dump.o \
						factory/
#grip_sensor
obj-$(CONFIG_SENSORS_A96T3X6)		+= a96t3x6.o