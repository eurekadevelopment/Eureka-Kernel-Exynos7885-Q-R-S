#
# Makefile for the sensors drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_SENSORS_CORE)          += sensors_core.o

# accelerometer_sensor
obj-$(CONFIG_SENSORS_LSM6DSL)            += lsm6dsl_core.o
obj-$(CONFIG_SENSORS_LSM6DSL_I2C)        += lsm6dsl_i2c.o
obj-$(CONFIG_SENSORS_LSM6DSL_SPI)        += lsm6dsl_spi.o
obj-$(CONFIG_SENSORS_LIS2DS)            += lis2ds_core.o
obj-$(CONFIG_SENSORS_LIS2DS_I2C)            += lis2ds_i2c.o
obj-$(CONFIG_SENSORS_LIS2DS_SPI)            += lis2ds_spi.o
obj-$(CONFIG_SENSORS_K2HH)          += k2hh.o
# magnetic_sensor
obj-$(CONFIG_SENSORS_YAS539)		+= yas_mag_drv-yas539.o yas_mag_kernel.o
obj-$(CONFIG_SENSORS_AK09918C)	    += ak09916c.o
# optical_sensor
obj-$(CONFIG_SENSORS_GP2AP070S)	    += gp2ap070s.o
obj-$(CONFIG_SENSORS_GP2AP110S)	    += gp2ap110s.o
obj-$(CONFIG_SENSORS_STK3013)       += stk3013.o
obj-$(CONFIG_SENSORS_STK3X3X)       += stk3x3x.o
obj-$(CONFIG_SENSORS_CM36658)	    += cm36658.o
obj-$(CONFIG_SENSORS_CM36686)       += cm36686.o
obj-$(CONFIG_SENSORS_CM36672P)	    += cm36672p.o
obj-$(CONFIG_SENSORS_TCS3701)       += ams_dax.o ams_i2c.o
obj-$(CONFIG_SENSORS_VEML3328)      += veml3328.o
obj-$(CONFIG_SENSORS_CM3323)        += cm3323.o
obj-$(CONFIG_SENSORS_TMD3725)       += tmd3725.o
#barometer_sensor
obj-$(CONFIG_SENSORS_LPS22HB)       += lps22hb_core.o lps22hb_i2c.o
#grip_sensor
obj-$(CONFIG_SENSORS_A96T3X6)		+= a96t3x6.o
obj-$(CONFIG_SENSORS_A96T3X6_SUB)	+= a96t3x6_sub.o
obj-$(CONFIG_SENSORS_TC3XXK)  		+= tc3xxk.o
obj-$(CONFIG_SENSORS_SX9330)		+= sx9330.o
obj-$(CONFIG_SENSORS_SX9330_WIFI)	+= sx9330_wifi.o
obj-$(CONFIG_SENSORS_SX9360)		+= sx9360.o
obj-$(CONFIG_SENSORS_SX9360_WIFI)	+= sx9360_wifi.o