/****************************************************************************
 *
 * Copyright (c) 2014 - 2018 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#ifndef __SCSC_LOG_COLLECTOR_H__
#define __SCSC_LOG_COLLECTOR_H__

/* High nibble is Major, Low nibble is Minor */
#define SCSC_LOG_HEADER_VERSION_MAJOR	0x01
#define SCSC_LOG_HEADER_VERSION_MINOR	0x00
/* Magic string. 4 bytes "SCSC"*/
/* Header version. 1 byte */
/* Num chunks. 1 byte */
/* Offset first Chunk. 2 bytes  */
/* Collection reason. 1 byte */
/* Reserved. 1 byte */
/* Reason Code . 2 bytes */
#define SCSC_LOG_HEADER_SIZE		(12)
#define SCSC_LOG_FW_VERSION_SIZE	(64)
#define SCSC_LOG_HOST_VERSION_SIZE	(64)
#define SCSC_LOG_FAPI_VERSION_SIZE	(64)
/* Reserved 2 . 4 byte */
#define SCSC_LOG_RESERVED_2		4
/* Ideally header + versions should be 16 bytes aligne*/
#define SCSC_SUPPORTED_CHUNKS_HEADER    48

#define SCSC_LOG_CHUNK_ALIGN		1
/* First chunk should be aligned */
#define SCSC_LOG_OFFSET_FIRST_CHUNK	(((SCSC_LOG_HEADER_SIZE + SCSC_LOG_FW_VERSION_SIZE + \
					SCSC_LOG_HOST_VERSION_SIZE + SCSC_LOG_FAPI_VERSION_SIZE + \
					SCSC_LOG_RESERVED_2 + SCSC_SUPPORTED_CHUNKS_HEADER) + \
					(SCSC_LOG_CHUNK_ALIGN - 1)) & ~(SCSC_LOG_CHUNK_ALIGN - 1))
enum scsc_log_reason {
	SCSC_LOG_UNKNOWN = 0,
	SCSC_LOG_FW_PANIC,
	SCSC_LOG_USER,
	SCSC_LOG_FW,
	SCSC_LOG_DUMPSTATE,
	SCSC_LOG_HOST_WLAN,
	SCSC_LOG_HOST_BT,
	SCSC_LOG_HOST_COMMON,
	/* Add others */
};

extern const char *scsc_loc_reason_str[];

#define SCSC_CHUNK_DAT_LEN_SIZE		4
#define SCSC_CHUNK_TYP_LEN_SIZE		4
#define SCSC_CHUNK_HEADER_SIZE		(SCSC_CHUNK_DAT_LEN_SIZE + SCSC_CHUNK_TYP_LEN_SIZE)

/* CHUNKS WILL COLLECTED ON THIS ORDER -
 * SYNC SHOULD BE THE FIRST CHUNK
 * LOGRING SHOULD BE THE LAST ONE SO IT COULD CAPTURE COLLECTION ERRORS
 */
enum scsc_log_chunk_type {
	SCSC_LOG_CHUNK_SYNC, /* SYNC should be the first chunk to collect */
	SCSC_LOG_CHUNK_IMP,
	SCSC_LOG_CHUNK_MXL,
	SCSC_LOG_CHUNK_UDI,
	SCSC_LOG_CHUNK_BT_HCF,
	SCSC_LOG_CHUNK_WLAN_HCF,
	SCSC_LOG_CHUNK_HIP4_SAMPLER,
	SCSC_LOG_RESERVED_COMMON,
	SCSC_LOG_RESERVED_BT,
	SCSC_LOG_RESERVED_WLAN,
	SCSC_LOG_RESERVED_RADIO,
	SCSC_LOG_CHUNK_LOGRING = 254,
	/* Add other chunks */
	SCSC_LOG_CHUNK_INVALID = 255,
};

#define SCSC_LOG_COLLECT_MAX_SIZE	(16*1024*1024)

/* ADD Collection codes here for HOST triggers */
/* Consider moving the definitions to specific services if required */

/* Reason codes for SCSC_LOG_USER */
#define SCSC_LOG_USER_REASON_PROC			0x0000
/* Reason codes for SCSC_LOG_DUMPSTATE */
#define SCSC_LOG_DUMPSTATE_REASON			0x0000
#define SCSC_LOG_DUMPSTATE_REASON_DRIVERDEBUGDUMP	0x0001
/* Reason codes for SCSC_LOG_HOST_WLAN */
#define SCSC_LOG_HOST_WLAN_REASON_DISCONNECT		0x0000
#define SCSC_LOG_HOST_WLAN_REASON_DISCONNECT_IND	0x0001
#define SCSC_LOG_HOST_WLAN_REASON_DISCONNECTED_IND	0x0002
#define SCSC_LOG_HOST_WLAN_REASON_DRIVERDEBUGDUMP	0x0003
/* Reason codes for SCSC_LOG_HOST_BT */
#define SCSC_LOG_HOST_BT_REASON_HCI_ERROR		0x0000
/* Reason codes for SCSC_LOG_HOST_COMMON */
#define SCSC_LOG_HOST_COMMON_REASON_START		0x0000
#define SCSC_LOG_HOST_COMMON_REASON_STOP		0x0001
#define SCSC_LOG_HOST_COMMON_RECOVER_RST		0x0002

/* SBL HEADER v 0.0*/
struct scsc_log_sbl_header {
	char magic[4];
	u8   version_major;
	u8   version_minor;
	u8   num_chunks;
	u8   trigger;
	u16  offset_data;
	char fw_version[SCSC_LOG_FW_VERSION_SIZE];
	char host_version[SCSC_LOG_HOST_VERSION_SIZE];
	char fapi_version[SCSC_LOG_FAPI_VERSION_SIZE];
	u16  reason_code;
	u8   reserved2[SCSC_LOG_RESERVED_2];
	char supported_chunks[SCSC_SUPPORTED_CHUNKS_HEADER];
} __packed;

struct scsc_log_chunk_header {
	char magic[3];
	u8   type;
	u32  chunk_size;
} __packed;

struct scsc_log_collector_client {
	char *name;
	enum scsc_log_chunk_type type;
	int (*collect_init)(struct scsc_log_collector_client *collect_client);
	int (*collect)(struct scsc_log_collector_client *collect_client, size_t size);
	int (*collect_end)(struct scsc_log_collector_client *collect_client);
	void *prv;
};

int scsc_log_collector_register_client(struct scsc_log_collector_client *collect_client);
int scsc_log_collector_unregister_client(struct scsc_log_collector_client *collect_client);

/* Public method to get pointer of SBL RAM buffer. */
unsigned char *scsc_log_collector_get_buffer(void);

/* Public method to register FAPI version. */
void scsc_log_collector_write_fapi(char __user *buf, size_t len);

void scsc_log_collector_schedule_collection(enum scsc_log_reason reason, u16 reason_code);
int scsc_log_collector_write(char __user *buf, size_t count, u8 align);
#endif /* __SCSC_LOG_COLLECTOR_H__ */
