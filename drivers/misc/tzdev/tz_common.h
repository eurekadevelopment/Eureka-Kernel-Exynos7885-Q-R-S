/*
 * Copyright (C) 2016 Samsung Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __TZ_COMMON_H__
#define __TZ_COMMON_H__

#include <linux/types.h>
#include <linux/limits.h>

/*
 * CA_ID is a full basename of CA executable.
 * It can not be longer than max supported filename length defined in Linux.
 */
#define CA_ID_LEN			(256)

#define TZ_IOC_MAGIC			'c'

#define TZIO_SMC			_IOWR(TZ_IOC_MAGIC, 101, struct tzio_smc_data)
#define TZIO_WAIT_EVT			_IO(TZ_IOC_MAGIC, 112)
#define TZIO_GET_PIPE			_IOR(TZ_IOC_MAGIC, 113, int)
#define TZIO_UPDATE_REE_TIME		_IO(TZ_IOC_MAGIC, 114)
#define TZIO_GET_ACCESS_INFO		_IOWR(TZ_IOC_MAGIC, 119, struct tzio_access_info)
#define TZIO_MEM_REGISTER		_IOWR(TZ_IOC_MAGIC, 120, struct tzio_mem_register)
#define TZIO_MEM_RELEASE		_IOW(TZ_IOC_MAGIC, 121, int)
#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG)
#define TZIO_SET_QC_CLK			_IOW(TZ_IOC_MAGIC, 123, int)
#endif
#define TZIO_GET_SYSCONF		_IOW(TZ_IOC_MAGIC, 124, struct tzio_sysconf)
#define TZIO_BOOST			_IO(TZ_IOC_MAGIC, 125)
#define TZIO_RELAX			_IO(TZ_IOC_MAGIC, 126)
#define TZIO_GET_CPU_MASK		_IO(TZ_IOC_MAGIC, 127)
#define TZIO_ACCEPT_NEW_HOLE		_IO(TZ_IOC_MAGIC, 128)

struct tzio_access_info {
	int32_t pid;			/* CA PID */
	int32_t gid;			/* CA GID - to be received from TZDEV */
	char ca_name[CA_ID_LEN];	/* CA identity - to be received from TZDEV */
};

/* NB: Sysconf related definitions should match with those in SWd */
#define SYSCONF_VERSION_LEN			(256)
#define SYSCONF_CRYPTO_CLOCK_MANAGEMENT		(1 << 0)

struct tzio_sysconf {
	uint32_t os_version;			/* SWd OS version */
	uint32_t cpu_num;			/* SWd number of cpu supported */
	uint32_t big_cpus_mask;			/* SWd big cluster cpus mask */
	uint32_t flags;				/* SWd flags */
	char version[SYSCONF_VERSION_LEN];	/* SWd OS version string */
} __attribute__((__packed__));

#define NR_SMC_ARGS	(4)

struct tzio_smc_data {
	union {
		int32_t args[NR_SMC_ARGS]; /* SMC args (in/out) */
		struct {
			uint16_t pipe;
			uint8_t reserved1;
			uint8_t event_mask;
			uint32_t reserved2;
			uint16_t iwnotify_oem_flags;
			uint16_t reserved3;
			uint32_t reserved4;
		} __packed;
	};
};

struct tzio_mem_register {
	int32_t pid;			/* Memory region owner's PID (in) */
	const uint64_t ptr;		/* Memory region start (in) */
	uint64_t size;			/* Memory region size (in) */
	int32_t id;			/* Memory region ID (out) */
	int32_t write;			/* 1 - rw, 0 - ro */
};

struct tzio_service_channel {
	uint32_t mask;
};

#endif /*__TZ_COMMON_H__*/
