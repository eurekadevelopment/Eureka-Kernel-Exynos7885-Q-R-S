#include "himax_inspection.h"

extern int g_ts_dbg;

extern struct himax_core_fp g_core_fp;
extern struct himax_ts_data *private_ts;
extern struct himax_ic_data *ic_data;
extern struct fw_operation *pfw_op;

extern int g_zero_event_count;

static int g_gap_vertical_partial = 2;
static int *g_gap_vertical_part;
static int g_gap_horizontal_partial = 1;
static int *g_gap_horizontal_part;
static int g_dc_max;
static int g_1kind_raw_size;
void himax_inspection_init(void);

void (*fp_himax_self_test_init) (void) = himax_inspection_init;

#ifdef HX_ESD_RECOVERY
extern u8 HX_ESD_RESET_ACTIVATE;
#endif

static void himax_press_powerkey(void)
{
	I(" %s POWER KEY event %x press\n", __func__, KEY_POWER);
	input_report_key(private_ts->input_dev, KEY_POWER, 1);
	input_sync(private_ts->input_dev);

	I(" %s POWER KEY event %x release\n", __func__, KEY_POWER);
	input_report_key(private_ts->input_dev, KEY_POWER, 0);
	input_sync(private_ts->input_dev);
}

static uint8_t NOISEMAX;

static int hx_test_data_pop_out(char *rslt_buf, char *filepath)
{

	struct file *raw_file = NULL;
	struct filename *vts_name = NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	int ret_val = NO_ERR;
	I("%s: Entering!\n", __func__);
	I("data size=0x%04X\n", (uint32_t) strlen(rslt_buf));
	vts_name = getname_kernel(filepath);

	if (raw_file == NULL)
		raw_file =
			file_open_name(vts_name, O_TRUNC | O_CREAT | O_RDWR, 0660);

	if (IS_ERR(raw_file)) {
		E("%s open file failed = %ld\n", __func__, PTR_ERR(raw_file));
		ret_val = -EIO;
		goto SAVE_DATA_ERR;
	}

	fs = get_fs();
	set_fs(get_ds());
	vfs_write(raw_file, rslt_buf,
			g_1kind_raw_size * HX_CRITERIA_ITEM * sizeof(char), &pos);

	filp_close(raw_file, NULL);

	set_fs(fs);

SAVE_DATA_ERR:
	I("%s: End!\n", __func__);
	return ret_val;
}

static int hx_test_data_get(uint32_t RAW[], char *start_log, char *result,
				int now_item)
{
	uint32_t i;

	ssize_t len = 0;
	char *testdata = NULL;
	uint32_t SZ_SIZE = g_1kind_raw_size;

	I("%s: Entering, Now type=%s!\n", __func__,
		g_himax_inspection_mode[now_item]);

	testdata = kzalloc(sizeof(char) * SZ_SIZE, GFP_KERNEL);

	len += snprintf((testdata + len), SZ_SIZE - len, "%s", start_log);
	for (i = 0; i < ic_data->HX_TX_NUM * ic_data->HX_RX_NUM; i++) {
		if (i > 1 && ((i + 1) % ic_data->HX_RX_NUM) == 0) {
			len +=
				snprintf((testdata + len), SZ_SIZE - len, "%5d,\n",
					RAW[i]);
		} else {
			len +=
				snprintf((testdata + len), SZ_SIZE - len, "%5d,",
					RAW[i]);
		}
	}
	len += snprintf((testdata + len), SZ_SIZE - len, "\n%s", result);

	memcpy(&g_rslt_data[now_item * SZ_SIZE], testdata, SZ_SIZE);

	/* dbg */
	/*for(i = 0; i < SZ_SIZE; i++)
	{
		I("0x%04X, ", g_rslt_data[i + (now_item * SZ_SIZE)]);
		if(i > 0 && (i % 16 == 15))
			printk("\n");
	}*/

	kfree(testdata);
	I("%s: End!\n", __func__);
	return NO_ERR;

}

static int himax_switch_mode_inspection(int mode)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	I("%s: Entering\n", __func__);

/*Stop Handshaking*/
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x00;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);

/*Swtich Mode*/
	switch (mode) {
	case HIMAX_INSPECTION_SORTING:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_SORTING_START;
		tmp_data[0] = PWD_SORTING_START;
		break;
	case HIMAX_INSPECTION_OPEN:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_OPEN_START;
		tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_OPEN_START;
		tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_SHORT:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_SHORT_START;
		tmp_data[0] = PWD_SHORT_START;
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:

	case HIMAX_INSPECTION_RAWDATA:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_RAWDATA_START;
		tmp_data[0] = PWD_RAWDATA_START;
		break;
	case HIMAX_INSPECTION_BPN_RAWDATA:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_RAWDATA_START;
		tmp_data[0] = PWD_RAWDATA_START;
		break;
	case HIMAX_INSPECTION_NOISE:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_NOISE_START;
		tmp_data[0] = PWD_NOISE_START;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_ACT_IDLE_START;
		tmp_data[0] = PWD_ACT_IDLE_START;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_LPWUG_START;
		tmp_data[0] = PWD_LPWUG_START;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_LPWUG_IDLE_START;
		tmp_data[0] = PWD_LPWUG_IDLE_START;
		break;

	default:
		I("%s,Nothing to be done!\n", __func__);
		break;
	}

	if (g_core_fp.fp_assign_sorting_mode != NULL)
		g_core_fp.fp_assign_sorting_mode(tmp_data);
	I("%s: End of setting!\n", __func__);

	return 0;

}

static int himax_get_rawdata(int32_t RAW[], uint32_t datalen, uint8_t checktype)
{
	uint8_t *tmp_rawdata;
	bool get_raw_rlst = false;
	uint8_t retry = 0;
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t index = 0;
	int Min_DATA = 99999;
	int Max_DATA = -99999;

	tmp_rawdata = kzalloc(sizeof(uint8_t) * (datalen * 2), GFP_KERNEL);
	if (!tmp_rawdata)
		goto mem_alloc_fail;

	/*1 Set Data Ready PWD*/
	while (retry < 200) {
		get_raw_rlst = g_core_fp.fp_get_DSRAM_data(tmp_rawdata, false);
		if (get_raw_rlst)
	/*2 Read Data from SRAM*/
	/*I("last total_size_temp=%d\n", total_size_temp);*/
	/*3 Check Checksum*/
			break;
		retry++;
	}

	if (retry >= 200)
		goto DIRECT_END;

	/*4 Copy Data*/
	for (i = 0; i < ic_data->HX_TX_NUM * ic_data->HX_RX_NUM; i++) {
		if(checktype == HIMAX_INSPECTION_NOISE)
			RAW[i] = ((int8_t)tmp_rawdata[(i * 2) + 1] * 256 )+ tmp_rawdata[(i * 2)];
		else
			RAW[i] = tmp_rawdata[(i * 2) + 1] * 256 + tmp_rawdata[(i * 2)];
	}

	if (g_ts_dbg != 0) {
		for (j = 0; j < ic_data->HX_RX_NUM; j++) {
			if (j == 0) {
				printk("      RX%2d", j + 1);
			} else {
				printk("  RX%2d", j + 1);
			}
		}
		printk("\n");

		for (i = 0; i < ic_data->HX_TX_NUM; i++) {
			printk("TX%2d", i + 1);
			for (j = 0; j < ic_data->HX_RX_NUM; j++) {
				printk("%5d ", ((int32_t)RAW[index]));
				if (((int32_t)RAW[index]) > Max_DATA) {
					Max_DATA = RAW[index];
				}
				if (((int32_t)RAW[index]) < Min_DATA) {
					Min_DATA = RAW[index];
				}
				index++;
			}
			printk("\n");
		}
		I("Max = %5d, Min = %5d \n", Max_DATA, Min_DATA);
	}
DIRECT_END:
	kfree(tmp_rawdata);
mem_alloc_fail:
	if (get_raw_rlst) {
		return HX_INSPECT_OK;
	} else {
		E("Get SRAM data fail\n");
		return HX_INSPECT_EOTHER;
	}
}

static void himax_switch_data_type(uint8_t checktype)
{
	uint8_t datatype = 0x00;

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		datatype = DATA_SORTING;
		break;
	case HIMAX_INSPECTION_OPEN:
		datatype = DATA_OPEN;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		datatype = DATA_MICRO_OPEN;
		break;
	case HIMAX_INSPECTION_SHORT:
		datatype = DATA_SHORT;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		datatype = DATA_RAWDATA;
		break;
	case HIMAX_INSPECTION_BPN_RAWDATA:
		datatype = DATA_RAWDATA;
		break;
	case HIMAX_INSPECTION_NOISE:
		datatype = DATA_NOISE;
		break;
	case HIMAX_INSPECTION_BACK_NORMAL:
		datatype = DATA_BACK_NORMAL;
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:
		datatype = DATA_RAWDATA;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
		datatype = DATA_ACT_IDLE_RAWDATA;
		break;
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		datatype = DATA_ACT_IDLE_NOISE;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
		datatype = DATA_LPWUG_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWUG_NOISE:
		datatype = DATA_LPWUG_NOISE;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
		datatype = DATA_LPWUG_IDLE_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		datatype = DATA_LPWUG_IDLE_NOISE;
		break;

	default:
		E("Wrong type=%d\n", checktype);
		break;
	}
	g_core_fp.fp_diag_register_set(datatype, 0x00);
}

int hx_chk_sup_neg(void)
{
	int rslt = 0;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x7F; tmp_addr[0] = 0xD8;
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	if ((tmp_data[3] & 0x04) == 0x04)
		rslt = 1;
	else 
		rslt = 0;
	return rslt;
}

static void himax_set_N_frame(uint16_t Nframe, uint8_t checktype)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	input_info(true, &private_ts->client->dev, "%s %s: Entering\n", HIMAX_LOG_TAG, __func__);

	/*IIR MAX*/
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x72;
	tmp_addr[0] = 0x94;
	if (hx_chk_sup_neg()) {
		tmp_data[3] = 0x7F;
		tmp_data[2] = 0x0C;
	} else {
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
	}
	tmp_data[1] = (uint8_t) ((Nframe & 0xFF00) >> 8);
	tmp_data[0] = (uint8_t) (Nframe & 0x00FF);
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);

	/*skip frame*/
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70;
	tmp_addr[0] = 0xF4;
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	switch (checktype) {

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		tmp_data[0] = BS_ACT_IDLE;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		tmp_data[0] = BS_LPWUG;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		tmp_data[0] = BS_LPWUG_dile;
		break;

	case HIMAX_INSPECTION_RAWDATA:
	case HIMAX_INSPECTION_BPN_RAWDATA:
	case HIMAX_INSPECTION_NOISE:
		tmp_data[0] = BS_RAWDATANOISE;
		break;
	default:
		tmp_data[0] = BS_OPENSHORT;
		break;
	}
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
	input_info(true, &private_ts->client->dev, "%s %s: End\n", HIMAX_LOG_TAG, __func__);
}

static void himax_get_noise_base(void)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70;
	tmp_addr[0] = 0x8C;
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	g_inspection_criteria[IDX_NOISEMAX][0] = tmp_data[3];
	NOISEMAX = tmp_data[3];
	I("%s: g_inspection_criteria[IDX_NOISEMAX]=%d\n", __func__,
		g_inspection_criteria[IDX_NOISEMAX][0]);
}

static uint32_t himax_check_mode(uint8_t checktype)
{
	uint8_t tmp_data[4] = { 0 };
	uint8_t tmp_addr[4] = { 0 };
	uint8_t wait_pwd[2] = { 0 };

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		wait_pwd[0] = PWD_SORTING_END;
		wait_pwd[1] = PWD_SORTING_END;
		break;
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_BPN_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		wait_pwd[0] = PWD_ACT_IDLE_END;
		wait_pwd[1] = PWD_ACT_IDLE_END;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		wait_pwd[0] = PWD_LPWUG_END;
		wait_pwd[1] = PWD_LPWUG_END;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWUG_IDLE_END;
		wait_pwd[1] = PWD_LPWUG_IDLE_END;
		break;

	default:
		E("Wrong type=%d\n", checktype);
		break;
	}

	/* fw status */
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x7F; tmp_addr[0] = 0x40;
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	input_info(true, &private_ts->client->dev,
		"%s: FW status, [0]=0x%02X,[1]=0x%02X,[2]=0x%02X,[3]=0x%02X \n",
		__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

	if (g_core_fp.fp_check_sorting_mode != NULL)
		g_core_fp.fp_check_sorting_mode(tmp_data);

	if ((wait_pwd[0] == tmp_data[0]) && (wait_pwd[1] == tmp_data[1])) {
		I("Change to mode=%s\n", g_himax_inspection_mode[checktype]);
		return 0;
	} else {
		return 1;
	}
}

static uint32_t himax_wait_sorting_mode(uint8_t checktype)
{
	uint8_t tmp_addr[4] = { 0 };
	uint8_t tmp_data[4] = { 0 };
	uint8_t wait_pwd[2] = { 0 };
	int count = 0;

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		wait_pwd[0] = PWD_SORTING_END;
		wait_pwd[1] = PWD_SORTING_END;
		break;
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_BPN_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		wait_pwd[0] = PWD_ACT_IDLE_END;
		wait_pwd[1] = PWD_ACT_IDLE_END;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		wait_pwd[0] = PWD_LPWUG_END;
		wait_pwd[1] = PWD_LPWUG_END;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWUG_IDLE_END;
		wait_pwd[1] = PWD_LPWUG_IDLE_END;
		break;

	default:
		I("No Change Mode and now type=%d\n", checktype);
		break;
	}

	do {
		if (g_core_fp.fp_check_sorting_mode != NULL)
			g_core_fp.fp_check_sorting_mode(tmp_data);
		if ((wait_pwd[0] == tmp_data[0])
			&& (wait_pwd[1] == tmp_data[1])) {
			return 0;
		}

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I("%s: 0x900000A8, tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xE4;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I("%s: 0x900000E4, tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x40;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I("%s: 0x10007F40,tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		I("Now retry %d times!\n", count++);
		msleep(50);
	} while (count < 200);

	return 1;
}

static int himax_gap_test_vertical_setting(void)
{
	g_gap_vertical_part =
		kzalloc(sizeof(int) * g_gap_vertical_partial, GFP_KERNEL);
	g_gap_vertical_part[0] = 0;
	g_gap_vertical_part[1] = ic_data->HX_TX_NUM / 2;
	return NO_ERR;
}

static void himax_cal_gap_data_vertical(int start, int end_idx, int direct,
					uint32_t * org_raw, int *result_raw)
{
	int i = 0;
	int rx_num = ic_data->HX_RX_NUM;
	I("%s:start=%d\n", __func__, start);
	I("%s:end_idx=%d\n", __func__, end_idx);
	for (i = start; i < (start + rx_num * end_idx); i++) {
		if (direct == 0) {	/* up - down */
			if (i < start + rx_num) {
				result_raw[i] = 0;
			} else {
				result_raw[i] =
					(((int)org_raw[i - rx_num]) - ((int)org_raw[i]))
					* 100 / ((int)org_raw[i]);
			}
		} else {	/* down - up */
			if (i > (start + rx_num * (end_idx - 1) - 1)) {
				result_raw[i] = 0;
			} else {
				result_raw[i] =
					(((int)org_raw[i + rx_num]) - ((int)org_raw[i]))
					* 100 / ((int)org_raw[i]);
			}
		}
		result_raw[i] = HX_ABS(result_raw[i]);
	}
}

static int himax_gap_test_vertical_raw(int test_type, uint32_t * org_raw)
{
	int i_partial = 0;
	int tmp_start = 0;
	int tmp_end_idx = 0;
	uint32_t *result_raw;
	int i = 0;
	int ret_val = NO_ERR;
	int tx_num = ic_data->HX_TX_NUM;
	int rx_num = ic_data->HX_RX_NUM;
	himax_gap_test_vertical_setting();
	I("Print vertical ORG RAW\n");
	for (i = 0; i < tx_num * rx_num; i++) {
		I("%04d,", org_raw[i]);
		if (i > 0 && i % rx_num == (rx_num - 1))
			I("\n");
	}
	result_raw = kzalloc(sizeof(uint32_t) * tx_num * rx_num, GFP_KERNEL);
	for (i_partial = 0; i_partial < g_gap_vertical_partial; i_partial++) {
		tmp_start = g_gap_vertical_part[i_partial] * rx_num;
		if (i_partial + 1 == g_gap_vertical_partial) {
			tmp_end_idx = tx_num - g_gap_vertical_part[i_partial];
		} else {
			tmp_end_idx =
				g_gap_vertical_part[i_partial + 1] -
				g_gap_vertical_part[i_partial];
		}
		if (i_partial % 2 == 0) {
			himax_cal_gap_data_vertical(tmp_start, tmp_end_idx, 0,
							org_raw, result_raw);
		} else {
			himax_cal_gap_data_vertical(tmp_start, tmp_end_idx, 1,
							org_raw, result_raw);
		}
	}
	I("Print Vertical New RAW\n");
	for (i = 0; i < tx_num * rx_num; i++) {
		I("%04d,", result_raw[i]);
		if (i > 0 && i % rx_num == (rx_num - 1))
			I("\n");
	}
	for (i = 0; i < tx_num * rx_num; i++) {
		if (result_raw[i] < g_inspection_criteria[IDX_GAP_VER_RAWMIN][i]
			&& result_raw[i] >
			g_inspection_criteria[IDX_GAP_VER_RAWMAX][i]) {
			ret_val = NO_ERR - i;
			break;
		}
	}
	kfree(g_gap_vertical_part);
	kfree(result_raw);
	return ret_val;
}

static int himax_gap_test_horizontal_setting(void)
{
	g_gap_horizontal_part =
		kzalloc(sizeof(int) * g_gap_horizontal_partial, GFP_KERNEL);
	g_gap_horizontal_part[0] = 0;
	/*g_gap_horizontal_part[1] = ic_data->HX_RX_NUM / 2;*/

	return NO_ERR;
}

static void himax_cal_gap_data_horizontal(int start, int end_idx, int direct,
					uint32_t * org_raw, int *result_raw)
{
	int i = 0;
	int j = 0;
	int rx_num = ic_data->HX_RX_NUM;
	int tx_num = ic_data->HX_TX_NUM;
	I("start=%d\n", start);
	I("end_idx=%d\n", end_idx);
	for (j = 0; j < tx_num; j++) {
		for (i = (start + (j * rx_num)); i < (start + (j * rx_num) + end_idx); i++) {	/* left - right */
			if (direct == 0) {
				if (i == (start + (j * rx_num))) {
					result_raw[i] = 0;
				} else {
					result_raw[i] =
						(((int)org_raw[i - 1]) -	 ((int)org_raw[i]))
						* 100 / ((int)org_raw[i]);
				}
			} else {	/* right - left */
				if (i == ((start + (j * rx_num) + end_idx) - 1)) {
					result_raw[i] = 0;
				} else {
					result_raw[i] =
						(((int)org_raw[i + 1]) - ((int)org_raw[i]))
						* 100 / ((int)org_raw[i]);
				}
			}
			result_raw[i] = HX_ABS(result_raw[i]);
		}
	}
}

static int himax_gap_test_honrizontal_raw(int test_type, uint32_t * raw)
{
	int rx_num = ic_data->HX_RX_NUM;
	int tx_num = ic_data->HX_TX_NUM;
	int tmp_start = 0;
	int tmp_end_idx = 0;
	int i_partial = 0;
	int *result_raw;
	int i = 0;
	int ret_val = NO_ERR;
	himax_gap_test_horizontal_setting();
	result_raw =
		kzalloc(sizeof(int) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM,
			GFP_KERNEL);
	I("Print Horizontal ORG RAW\n");
	for (i = 0; i < tx_num * rx_num; i++) {
		I("%04d,", raw[i]);
		if (i > 0 && i % rx_num == (rx_num - 1))
			I("\n");
	}
	for (i_partial = 0; i_partial < g_gap_horizontal_partial; i_partial++) {
		tmp_start = g_gap_horizontal_part[i_partial];
		if (i_partial + 1 == g_gap_horizontal_partial) {
			tmp_end_idx = rx_num - g_gap_horizontal_part[i_partial];
		} else {
			tmp_end_idx =
				g_gap_horizontal_part[i_partial + 1] -
				g_gap_horizontal_part[i_partial];
		}
		if (i_partial % 2 == 0) {
			himax_cal_gap_data_horizontal(tmp_start, tmp_end_idx, 0,
							raw, result_raw);
		} else {
			himax_cal_gap_data_horizontal(tmp_start, tmp_end_idx, 1,
							raw, result_raw);
		}
	}
	I("Print Horizontal New RAW\n");
	for (i = 0; i < tx_num * rx_num; i++) {
		I("%04d,", result_raw[i]);
		if (i > 0 && i % rx_num == (rx_num - 1))
			I("\n");
	}
	for (i = 0; i < tx_num * rx_num; i++) {
		if (result_raw[i] < g_inspection_criteria[IDX_GAP_HOR_RAWMIN][i]
			&& result_raw[i] > g_inspection_criteria[IDX_GAP_HOR_RAWMAX][i]) {
			ret_val = NO_ERR - i;
			break;
		}
	}
	kfree(g_gap_horizontal_part);
	kfree(result_raw);
	return ret_val;
}

#if 0
static int himax_check_notch(int index)
{
	if (SKIP_NOTCH_START < 0 && SKIP_NOTCH_END < 0 && SKIP_DUMMY_START < 0
		&& SKIP_DUMMY_START < 0) {
		/* no support notch */
		return 0;
	}
	if ((index >= SKIP_NOTCH_START) && (index <= SKIP_NOTCH_END))
		return 1;
	else if ((index >= SKIP_DUMMY_START) && (index <= SKIP_DUMMY_END))
		return 1;
	else
		return 0;
}
#endif

static uint32_t mpTestFunc(uint8_t checktype, uint32_t datalen)
{
	uint32_t i /*, j */ , ret = 0;
	uint32_t *RAW;

	char *rslt_log;
	char *start_log;

	int ret_val = HX_INSPECT_EOTHER;

	/*uint16_t* pInspectGridData = &gInspectGridData[0];*/
	/*uint16_t* pInspectNoiseData = &gInspectNoiseData[0];*/
	I("Now Check type = %d\n", checktype);

	if (himax_check_mode(checktype)) {
		I("Need Change Mode ,target=%s\n",
			g_himax_inspection_mode[checktype]);

		g_core_fp.fp_sense_off(true);

#ifndef HX_ZERO_FLASH
		if (g_core_fp.fp_reload_disable != NULL)
			g_core_fp.fp_reload_disable(1);
#endif

		himax_switch_mode_inspection(checktype);

		if (checktype == HIMAX_INSPECTION_NOISE) {
			himax_set_N_frame(NOISEFRAME, checktype);
			himax_get_noise_base();

		} else if (checktype == HIMAX_INSPECTION_ACT_IDLE_RAWDATA
				|| checktype == HIMAX_INSPECTION_ACT_IDLE_NOISE) {
			I("N frame = %d\n", 10);
			himax_set_N_frame(10, checktype);

		} else if (checktype >= HIMAX_INSPECTION_LPWUG_RAWDATA) {
			I("N frame = %d\n", 1);
			himax_set_N_frame(1, checktype);

		} else {
			himax_set_N_frame(2, checktype);
		}

		g_core_fp.fp_sense_on(1);

		ret = himax_wait_sorting_mode(checktype);
		if (ret) {
			E("%s: himax_wait_sorting_mode FAIL\n", __func__);
			return ret;
		}
	}

	himax_switch_data_type(checktype);

	RAW = kzalloc(datalen * sizeof(uint32_t), GFP_KERNEL);
	if (!RAW)
		goto err_alloc_raw;
	ret = himax_get_rawdata(RAW, datalen, checktype);
	if (ret) {
		E("%s: himax_get_rawdata FAIL\n", __func__);
		kfree(RAW);
		return ret;
	}

	g_dc_max = g_core_fp.fp_get_max_dc();
	/* back to normal */
	himax_switch_data_type(HIMAX_INSPECTION_BACK_NORMAL);

	I("%s: Init OK, start to test!\n", __func__);
	rslt_log = kzalloc(256 * sizeof(char), GFP_KERNEL);
	if (!rslt_log)
		goto err_alloc_rslt;
	start_log = kzalloc(256 * sizeof(char), GFP_KERNEL);
	if (!start_log)
		goto err_alloc_start;

	snprintf(start_log, 256 * sizeof(char), "\n%s%s\n",
		g_himax_inspection_mode[checktype], ": data as follow!\n");

	/*Check Data*/
	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] < g_inspection_criteria[IDX_SORTMIN][i]) {
				E("%s: sorting mode open test FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_EOPEN;
				goto FAIL_END;
			}
		}
		I("%s: sorting mode open test PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_OPEN:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_OPENMAX][i]
				|| (int)RAW[i] <
				g_inspection_criteria[IDX_OPENMIN][i]) {
				E("%s: open test FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_EOPEN;
				goto FAIL_END;
			}
		}
		I("%s: open test PASS\n", __func__);

		break;

	case HIMAX_INSPECTION_MICRO_OPEN:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_M_OPENMAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_M_OPENMIN][i]) {
				E("%s: micro open test FAIL in index %d\n",
					__func__, i);
				ret_val = HX_INSPECT_EMOPEN;
				goto FAIL_END;
			}
		}
		I("%s: micro open test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_SHORT:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_SHORTMAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_SHORTMIN][i]) {
				E("%s: short test FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_ESHORT;
				goto FAIL_END;
			}
		}
		I("%s: short test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/* I("Now new compare, datalen=%d!\n",ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); */
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_RAWMAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_RAWMIN][i]) {
				E("%s: rawdata test FAIL:RAW[%d]=%d\n",
					__func__, i, RAW[i]);
				I("%s: Now Criteria max=%d,min=%d\n", __func__,
					g_inspection_criteria[IDX_RAWMAX][i],
					g_inspection_criteria[IDX_RAWMIN][i]);
				ret_val = HX_INSPECT_ERAW;
				goto FAIL_END;
			}
		}
		I("%s: rawdata test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_BPN_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			RAW[i] = (int)RAW[i] * 100 / g_dc_max;
			if ((int)RAW[i] >
				g_inspection_criteria[IDX_BPN_RAWMAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_BPN_RAWMIN][i]) {
				E("%s: rawdata test FAIL:BPN RAW[%d]=%d\n",
					__func__, i, RAW[i]);
				I("%s: Now Criteria max=%d,min=%d\n", __func__,
					g_inspection_criteria[IDX_BPN_RAWMAX][i],
					g_inspection_criteria[IDX_BPN_RAWMIN][i]);
				ret_val = HX_INSPECT_ERAW;
				goto FAIL_END;
			}
		}
		I("%s: BPN rawdata test PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_NOISE:
		I("NOISEMAX=%d\n", NOISEMAX);
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_NOISEMAX][0]) {
				E("%s: noise test FAIL\n", __func__);
				ret_val = HX_INSPECT_ENOISE;
				goto FAIL_END;
			}
		}
		I("%s: noise test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:
		if (himax_gap_test_vertical_raw
			(HIMAX_INSPECTION_GAPTEST_RAW, RAW) != NO_ERR) {
			E("%s: HIMAX_INSPECTION_GAPTEST_RAW FAIL\n", __func__);
			ret_val = HX_INSPECT_EGAP_RAW;
			goto FAIL_END;
		}
		if (himax_gap_test_honrizontal_raw
			(HIMAX_INSPECTION_GAPTEST_RAW, RAW) != NO_ERR) {
			E("%s: HIMAX_INSPECTION_GAPTEST_RAW FAIL\n", __func__);
			ret_val = HX_INSPECT_EGAP_RAW;
			goto FAIL_END;
		}
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_ACT_IDLE_RAWDATA_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_ACT_IDLE_RAWDATA_MIN][i])
			{
				E("%s: HIMAX_INSPECTION_ACT_IDLE_RAWDATA FAIL  in index %d\n", __func__, i);
				ret_val = HX_INSPECT_EACT_IDLE_RAW;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_ACT_IDLE_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
			continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_ACT_IDLE_NOISE_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_ACT_IDLE_NOISE_MIN][i]) {
				E("%s: HIMAX_INSPECTION_ACT_IDLE_NOISE FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_EACT_IDLE_NOISE;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_ACT_IDLE_NOISE PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_RAWDATA_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_RAWDATA_MIN][i]) {
				E("%s: HIMAX_INSPECTION_LPWUG_RAWDATA FAIL  in index %d\n", __func__, i);
				ret_val = HX_INSPECT_ELPWUG_RAW;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_NOISE_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_NOISE_MIN][i]) {
				E("%s: HIMAX_INSPECTION_LPWUG_NOISE FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_ELPWUG_NOISE;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_NOISE PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
			continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_IDLE_RAWDATA_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_IDLE_RAWDATA_MIN][i]) {
				E("%s: HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_ELPWUG_IDLE_RAW;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
			continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_IDLE_NOISE_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_IDLE_NOISE_MIN][i])
			{
				E("%s: HIMAX_INSPECTION_LPWUG_IDLE_NOISE FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_ELPWUG_IDLE_NOISE;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_IDLE_NOISE PASS\n", __func__);
		break;

	default:
		E("Wrong type=%d\n", checktype);
		break;
	}

	ret_val = HX_INSPECT_OK;
	snprintf(rslt_log, 256 * sizeof(char), "\n%s%s\n",
		g_himax_inspection_mode[checktype], " Test Pass!\n");
	I("pass write log\n");
	goto END_FUNC;

FAIL_END:
	snprintf(rslt_log, 256 * sizeof(char), "\n%s%s\n",
		g_himax_inspection_mode[checktype], " Test Fail!\n");
	I("fail write log\n");
END_FUNC:
	hx_test_data_get(RAW, start_log, rslt_log, checktype);
	kfree(start_log);
err_alloc_start:
	kfree(rslt_log);
err_alloc_rslt:
	kfree(RAW);
err_alloc_raw:
	return ret_val;

/* parsing Criteria start */

}

/* claculate 10's power function */
static int himax_power_cal(int pow, int number)
{
	int i = 0;
	int result = 1;

	for (i = 0; i < pow; i++)
		result *= 10;
	result = result * number;

	return result;

}

/* String to int */
static int hiamx_parse_str2int(char *str)
{
	int i = 0;
	int temp_cal = 0;
	int result = 0;
	int str_len = strlen(str);
	int negtive_flag = 0;
	for (i = 0; i < strlen(str); i++) {
		if (str[i] != '-' && str[i] > '9' && str[i] < '0') {
			E("%s: Parsing fail!\n", __func__);
			result = -9487;
			negtive_flag = 0;
			break;
		}
		if (str[i] == '-') {
			negtive_flag = 1;
			continue;
		}
		temp_cal = str[i] - '0';
		result += himax_power_cal(str_len - i - 1, temp_cal);
		/* str's the lowest char is the number's the highest number
		So we should reverse this number before using the power function
		-1: starting number is from 0 ex:10^0 = 1,10^1=10 */
	}

	if (negtive_flag == 1) {
		result = 0 - result;
	}

	return result;
}

/* Get sub-string from original string by using some charaters return size of result*/
static int himax_saperate_comma(const struct firmware *file_entry,
				char **result, int str_size)
{
	int count = 0;
	int str_count = 0;	/* now string */
	int char_count = 0;	/* now char count in string */

	do {
		switch (file_entry->data[count]) {
		case ASCII_COMMA:
		case ACSII_SPACE:
		case ASCII_CR:
		case ASCII_LF:
			count++;
			/* If end of line as above condifiton, differencing the count of char.
			If char_count != 0 it's meaning this string is parsing over .
			The Next char is belong to next string */
			if (char_count != 0) {
				char_count = 0;
				str_count++;
			}
			break;
		default:
			result[str_count][char_count++] =
				file_entry->data[count];
			count++;
			break;
		}
	} while (count < file_entry->size && str_count < str_size);

	return str_count;
}

static int hx_diff_str(char *str1, char *str2)
{
	int i = 0;
	int result = 0;		/* zero is all same, non-zero is not same index */
	int str1_len = strlen(str1);
	int str2_len = strlen(str2);

	if (str1_len != str2_len) {
		if (private_ts->debug_log_level & BIT(4))
			I("%s:Size different!\n", __func__);
		return LENGTH_FAIL;
	}

	for (i = 0; i < str1_len; i++) {
		if (str1[i] != str2[i]) {
			result = i + 1;
			I("%s: different in %d!\n", __func__, result);
			return result;
		}
	}

	return result;
}

/* get idx of criteria whe parsing file */
int hx_find_crtra_id(char *input)
{
	int i = 0;
	int result = 0;

	for (i = 0; i < HX_CRITERIA_SIZE; i++) {
		if (hx_diff_str(g_hx_inspt_crtra_name[i], input) == 0) {
			result = i;
			I("find the str=%s,idx=%d\n", g_hx_inspt_crtra_name[i], i);
			break;
		}
	}
	if (i > (HX_CRITERIA_SIZE - 1)) {
		E("%s: find Fail!\n", __func__);
		return LENGTH_FAIL;
	}

	return result;
}

int hx_print_crtra_after_parsing(void)
{
	int i = 0, j = 0;
	int all_mut_len = ic_data->HX_TX_NUM * ic_data->HX_RX_NUM;

	for (i = 0; i < HX_CRITERIA_SIZE; i++) {
		I("Now is %s\n", g_hx_inspt_crtra_name[i]);
		if (g_inspt_crtra_flag[i] == 1) {
			for (j = 0; j < all_mut_len; j++) {
				I("%d, ", g_inspection_criteria[i][j]);
				if (j % 16 == 15)
					printk("\n");
			}
		} else {
			I("No this Item in this criteria file!\n");
		}
		printk("\n");
	}

	return 0;
}

static int hx_get_crtra_by_name(char **result, int size_of_result_str)
{
	int i = 0;
	/* count of criteria type */
	int count_type = 0;
	/* count of criteria data */
	int count_data = 0;
	int err = HX_INSPECT_OK;
	int all_mut_len = ic_data->HX_TX_NUM * ic_data->HX_RX_NUM;
	int temp = 0;

	/* get criteria and assign to a global array(2-Dimensional/int) */
	/* basiclly the size of criteria will be (crtra_count * (all_mut_len) + crtra_count)
	but we use file size to be the end of counter*/
	for (i = 0; i < size_of_result_str && result[i] != NULL; i++) {
		/* It have get one page(all mutual) criteria data!
		And we should skip the string of criteria name!
		*/
		if (i == 0 || i == ((i / (all_mut_len)) +
			(i / (all_mut_len) * (all_mut_len)))) {
			count_data = 0;

			if (private_ts->debug_log_level & BIT(4)) {
				I("Now find str=%s ,idx=%d\n", result[i], i);
			}
			/* check the item of criteria is in criteria file or not*/
			count_type = hx_find_crtra_id(result[i]);
			if (count_type < 0) {
				E("1. %s:Name Not match!\n", __func__);
				/* E("can recognize[%d]=%s\n", count_type, g_hx_inspt_crtra_name[count_type]); */
				E("get from file[%d]=%s\n", i, result[i]);
				E("Please check criteria file again!\n");
				err = HX_INSPECT_EFILE;
				return err;
			} else {
				I("Now str=%s, idx=%d\n",
					g_hx_inspt_crtra_name[count_type],
					count_type);
				g_inspt_crtra_flag[count_type] = 1;
			}
			continue;
		}
		/* change string to int*/
		temp = hiamx_parse_str2int(result[i]);
		if (temp != -9487)
			g_inspection_criteria[count_type][count_data] = temp;
		else {
			E("%s: Parsing Fail in %d\n", __func__, i);
			E("in range:[%d]=%s\n", count_type,
				g_hx_inspt_crtra_name[count_type]);
			E("btw, get from file[%d]=%s\n", i, result[i]);
			break;
		}
		/* dbg
		I("[%d]g_inspection_criteria[%d][%d]=%d\n", i, count_type, count_data, g_inspection_criteria[count_type][count_data]);
		*/
		count_data++;

	}

	if (private_ts->debug_log_level & BIT(4)) {
	/* dbg:print all of criteria from parsing file */
		hx_print_crtra_after_parsing();
	}

	I("Total loop=%d\n", i);

	return err;
}

static int himax_parse_criteria_file(void)
{
	int err = HX_INSPECT_OK;
	const struct firmware *file_entry = NULL;
	char *file_name = "hx_criteria.csv";
	char **result;
	int i = 0;

	int crtra_count = HX_CRITERIA_SIZE;
	int data_size = 0;	/* The maximum of number Data */
	int all_mut_len = ic_data->HX_TX_NUM * ic_data->HX_RX_NUM;
	int str_max_len = 128;
	int result_all_len = 0;
	int file_size = 0;
	int size_of_result_str = 0;

	I("%s,Entering \n", __func__);
	I("file name = %s\n", file_name);

	/* default path is /system/etc/firmware */
	err = request_firmware(&file_entry, file_name, private_ts->dev);
	if (err < 0) {
		E("%s,fail in line%d error code=%d\n", __func__, __LINE__, err);
		err = HX_INSPECT_EFILE;
		goto END_FUNC_REQ_FAIL;
	}

	/* size of criteria include name string */
	data_size = ((all_mut_len) * crtra_count) + crtra_count;

	/* init the array which store original criteria and include name string*/
	result = kzalloc(data_size * sizeof(char *), GFP_KERNEL);
	for (i = 0; i < data_size; i++)
		result[i] = kzalloc(str_max_len * sizeof(char), GFP_KERNEL);

	result_all_len = data_size;
	file_size = file_entry->size;
	I("Now result_all_len=%d\n", result_all_len);
	I("Now file_size=%d\n", file_size);

	/* dbg */
	if (private_ts->debug_log_level & BIT(4)) {
		I("first 4 bytes 0x%2X,0x%2X,0x%2X,0x%2X !\n",
			file_entry->data[0], file_entry->data[1], file_entry->data[2],
			file_entry->data[3]);
	}

	/* parse value in to result array(1-Dimensional/String) */
	size_of_result_str =
		himax_saperate_comma(file_entry, result, data_size);

	I("%s: now size_of_result_str=%d\n", __func__, size_of_result_str);

	err = hx_get_crtra_by_name(result, size_of_result_str);
	if (err != HX_INSPECT_OK) {
		E("%s:Load criteria from file fail, go end!\n", __func__);
		goto END_FUNC;
	}

END_FUNC:
	for (i = 0; i < data_size; i++)
		kfree(result[i]);
	kfree(result);
	release_firmware(file_entry);
END_FUNC_REQ_FAIL:
	I("%s,END \n", __func__);
	return err;
	/* parsing Criteria end */
}

int hx_get_size_str_arr(char **input)
{
	int i = 0;
	int result = 0;

	while (input[i] != NULL) {
		i++;
	}
	result = i;
	if (private_ts->debug_log_level & BIT(4)) {
		I("There is %d in [0]=%s\n", result, input[0]);
	}
	return result;
}

static int himax_self_test_data_init(void)
{
	int ret = HX_INSPECT_OK;
	int i = 0;

	g_1kind_raw_size = 5 * ic_data->HX_RX_NUM * ic_data->HX_RX_NUM * 2;
	/* get test item and its items of criteria*/
	HX_CRITERIA_ITEM = hx_get_size_str_arr(g_himax_inspection_mode);
	HX_CRITERIA_SIZE = hx_get_size_str_arr(g_hx_inspt_crtra_name);
	I("There is %d HX_CRITERIA_ITEM and %d HX_CRITERIA_SIZE\n",
		HX_CRITERIA_ITEM, HX_CRITERIA_SIZE);

	/* init criteria data*/
	g_inspt_crtra_flag =
		kzalloc(HX_CRITERIA_SIZE * sizeof(int), GFP_KERNEL);
	if (!g_inspt_crtra_flag) {
		I("g_inspt_crtra_flag allocate fail\n");
		goto ALLC_FAIL_INSPT_CRTRA_FLAG;
	}
	g_inspection_criteria =
		kzalloc(sizeof(int *) * HX_CRITERIA_SIZE, GFP_KERNEL);
	if (!g_inspection_criteria) {
		I("g_inspection_criteria allocate fail\n");
		goto ALLC_FAIL_INSPECTION_CRITERIA;
	}
	for (i = 0; i < HX_CRITERIA_SIZE; i++) {
		g_inspection_criteria[i] =
			kzalloc(sizeof(int) *
				(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM),
				GFP_KERNEL);
		if (!g_inspection_criteria[i]) {
			I("g_inspection_criteria[%d] allocate fail\n", i);
			goto ALLC_FAIL_INSPECTION_CRITERIA_N;
		}
	}

	/* parsing criteria from file*/
	ret = himax_parse_criteria_file();

	if (private_ts->debug_log_level & BIT(4)) {
	/* print get criteria string */
		for (i = 0; i < HX_CRITERIA_SIZE; i++) {
			if (g_inspt_crtra_flag[i] != 0)
				I("%s: [%d]There is String=%s\n", __func__, i,
					g_hx_inspt_crtra_name[i]);
		}
	}

	/* init result output data*/
	g_file_path = kzalloc(256 * sizeof(char), GFP_KERNEL);
	if (!g_file_path) {
		I("g_file_path allocate fail\n");
		goto ALLC_FAIL_FILE_PATH;
	}
	g_rslt_data =
		kzalloc(g_1kind_raw_size * HX_CRITERIA_ITEM * sizeof(char),
			GFP_KERNEL);
	if (!g_rslt_data) {
		I("g_rslt_data allocate fail\n");
		goto ALLC_FAIL_RSLT_DATA;
	}

	snprintf(g_file_path,
		(int)(strlen(HX_RSLT_OUT_PATH) + strlen(HX_RSLT_OUT_FILE)),
		"%s%s", HX_RSLT_OUT_PATH, HX_RSLT_OUT_FILE);

	return ret;
	kfree(g_rslt_data);
ALLC_FAIL_RSLT_DATA:
	kfree(g_file_path);
ALLC_FAIL_FILE_PATH:
	for (i = 0; i < HX_CRITERIA_SIZE; i++) {
		if (g_inspection_criteria[i]) {
			I("free g_inspection_criteria[%d]\n", i);
			kfree(g_inspection_criteria[i]);
		}
	}
ALLC_FAIL_INSPECTION_CRITERIA_N:
	kfree(g_inspection_criteria);
ALLC_FAIL_INSPECTION_CRITERIA:
	kfree(g_inspt_crtra_flag);
ALLC_FAIL_INSPT_CRTRA_FLAG:
	ret = HX_INSPECT_EOTHER;
	return ret;
}

static void himax_self_test_data_deinit(void)
{
	int i = 0;

	/*dbg*/
	/*for (i = 0; i < HX_CRITERIA_ITEM; i++)
	I("%s:[%d]%d\n", __func__, i, g_inspection_criteria[i]);*/
	if (g_inspection_criteria) {
		I("Start deinit g_inspection_criteria\n");
		for (i = 0; i < HX_CRITERIA_SIZE; i++) {
			if (g_inspection_criteria[i]) {
				I("Start deinit g_inspection_criteria[%d]\n", i);
				kfree(g_inspection_criteria[i]);
			} else {
				I("No need to deinit g_inspection_criteria[%d]\n", i);
			}
		}
		kfree(g_inspection_criteria);
		I("Now it have free the g_inspection_criteria!\n");
	} else {
		I("No Need to free g_inspection_criteria!\n");
	}

	if (g_inspt_crtra_flag) {
		I("Start to free g_inspt_crtra_flag\n");
		kfree(g_inspt_crtra_flag);
	} else {
		I("No need to free g_inspt_crtra_flag\n");
	}
	if (g_file_path) {
		I("Start to free g_file_path\n");
		kfree(g_file_path);
	} else {
		I("No need to free g_file_path\n");
	}
	if (g_rslt_data) {
		I("Start to free g_rslt_data\n");
		kfree(g_rslt_data);
	} else {
		I("No need to free g_rslt_data\n");
	}

}

static int himax_chip_self_test(void)
{
	uint32_t ret = HX_INSPECT_OK;

	I("%s:IN\n", __func__);

	ret = himax_self_test_data_init();
	if (ret != HX_INSPECT_OK) {
		E("%s: himax_self_test_data_init fail!\n", __func__);
		goto END_FUNC;
	}

	if (g_inspt_crtra_flag[IDX_OPENMIN] == 1
		&& g_inspt_crtra_flag[IDX_OPENMAX] == 1) {
		/*1. Open Test*/
		I("[MP_OPEN_TEST_RAW]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_OPEN,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("1. Open Test: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_OPENMIN],
			g_inspt_crtra_flag[IDX_OPENMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_OPENMAX],
			g_inspt_crtra_flag[IDX_OPENMAX]);
	}

	if (g_inspt_crtra_flag[IDX_M_OPENMIN] == 1
		&& g_inspt_crtra_flag[IDX_M_OPENMAX] == 1) {
		/*2. Micro-Open Test*/
		I("[MP_MICRO_OPEN_TEST_RAW]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_MICRO_OPEN,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("2. Micro Open Test: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_M_OPENMIN],
			g_inspt_crtra_flag[IDX_M_OPENMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_M_OPENMAX],
			g_inspt_crtra_flag[IDX_M_OPENMAX]);
	}

	if (g_inspt_crtra_flag[IDX_SHORTMIN] == 1
		&& g_inspt_crtra_flag[IDX_SHORTMAX] == 1) {
		/*3. Short Test*/
		I("[MP_SHORT_TEST_RAW]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_SHORT,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("3. Short Test: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_SHORTMIN],
			g_inspt_crtra_flag[IDX_SHORTMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_SHORTMAX],
			g_inspt_crtra_flag[IDX_SHORTMAX]);
	}

	if (g_inspt_crtra_flag[IDX_RAWMIN] == 1
		&& g_inspt_crtra_flag[IDX_RAWMAX] == 1) {
		/*4. RawData Test*/
		I("==========================================\n");
		I("[MP_RAW_TEST_RAW]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_RAWDATA,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("%d. %s: End %d\n\n\n", HIMAX_INSPECTION_RAWDATA,
			g_himax_inspection_mode[HIMAX_INSPECTION_RAWDATA], ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_RAWMIN],
			g_inspt_crtra_flag[IDX_RAWMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_RAWMAX],
			g_inspt_crtra_flag[IDX_RAWMAX]);
	}

	if (g_inspt_crtra_flag[IDX_BPN_RAWMIN] == 1
		&& g_inspt_crtra_flag[IDX_BPN_RAWMAX] == 1) {
		I("==========================================\n");
		I("[MP_BPN_RAW_TEST_RAW]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_BPN_RAWDATA,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("%d. %s: End %d\n\n\n", HIMAX_INSPECTION_BPN_RAWDATA,
			g_himax_inspection_mode[HIMAX_INSPECTION_BPN_RAWDATA], ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_BPN_RAWMIN],
			g_inspt_crtra_flag[IDX_BPN_RAWMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_BPN_RAWMAX],
			g_inspt_crtra_flag[IDX_BPN_RAWMAX]);
	}
	if (g_inspt_crtra_flag[IDX_NOISEMAX] == 1) {
		/*5. Noise Test*/
		I("[MP_NOISE_TEST_RAW]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_NOISE,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("5. Noise Test: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_NOISEMAX],
			g_inspt_crtra_flag[IDX_NOISEMAX]);
	}

	if (g_inspt_crtra_flag[IDX_SORTMIN] == 1
		&& g_inspt_crtra_flag[IDX_SORTMAX] == 1) {
		/*6. Sorting Test*/
		I("[SORTING TEST]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_SORTING,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("6. SORTING TEST: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_SORTMIN],
			g_inspt_crtra_flag[IDX_SORTMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_SORTMAX],
			g_inspt_crtra_flag[IDX_SORTMAX]);
	}

	if ((g_inspt_crtra_flag[IDX_GAP_HOR_RAWMAX] == 1
		&& g_inspt_crtra_flag[IDX_GAP_HOR_RAWMIN] == 1)
		&& (g_inspt_crtra_flag[IDX_GAP_VER_RAWMAX] == 1
		&& g_inspt_crtra_flag[IDX_GAP_VER_RAWMIN] == 1)) {
		/*6. GAP Test*/
		I("[MP_GAP_TEST_RAW]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_GAPTEST_RAW,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("6. MP_GAP_TEST_RAW: End %d\n\n\n", ret);
	} else {
		I("Now %s : HOR flag=%d\n",
			g_hx_inspt_crtra_name[IDX_GAP_HOR_RAWMAX],
			g_inspt_crtra_flag[IDX_GAP_HOR_RAWMAX]);
		I("Now %s : HOR flag=%d\n",
			g_hx_inspt_crtra_name[IDX_GAP_HOR_RAWMIN],
			g_inspt_crtra_flag[IDX_GAP_HOR_RAWMIN]);
		I("Now %s : VERTICAL flag=%d\n",
			g_hx_inspt_crtra_name[IDX_GAP_VER_RAWMAX],
			g_inspt_crtra_flag[IDX_GAP_VER_RAWMAX]);
		I("Now %s : VERTICAL flag=%d\n",
			g_hx_inspt_crtra_name[IDX_GAP_VER_RAWMIN],
			g_inspt_crtra_flag[IDX_GAP_VER_RAWMIN]);
	}

	if (g_inspt_crtra_flag[IDX_ACT_IDLE_RAWDATA_MIN] == 1
		&& g_inspt_crtra_flag[IDX_ACT_IDLE_RAWDATA_MAX] == 1) {
		/*7. ACT_IDLE RAWDATA*/
		I("[MP_ACT_IDLE_TEST_RAW]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_ACT_IDLE_RAWDATA,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("7. MP_ACT_IDLE_TEST_RAW: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_ACT_IDLE_RAWDATA_MAX],
			g_inspt_crtra_flag[IDX_ACT_IDLE_RAWDATA_MAX]);
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_ACT_IDLE_RAWDATA_MIN],
			g_inspt_crtra_flag[IDX_ACT_IDLE_RAWDATA_MIN]);
	}

	if (g_inspt_crtra_flag[IDX_ACT_IDLE_NOISE_MIN] == 1
		&& g_inspt_crtra_flag[IDX_ACT_IDLE_NOISE_MAX] == 1) {
		/*8. ACT_IDLE NOISE*/
		I("[MP_ACT_IDLE_TEST_NOISE]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_ACT_IDLE_NOISE,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("8. MP_ACT_IDLE_TEST_NOISE: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_ACT_IDLE_NOISE_MAX],
			g_inspt_crtra_flag[IDX_ACT_IDLE_NOISE_MAX]);
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_ACT_IDLE_NOISE_MIN],
			g_inspt_crtra_flag[IDX_ACT_IDLE_NOISE_MIN]);
	}

	/* check press power key or not for LPWUG test item*/
	if ((g_inspt_crtra_flag[IDX_LPWUG_NOISE_MAX] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_NOISE_MIN] == 1)
		|| (g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MAX] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MIN] == 1)
		|| (g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MAX] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MIN] == 1)
		|| (g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MAX] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MIN] == 1)) {
		himax_press_powerkey();
	}

	if (g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MIN] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MAX] == 1) {
		/*9. LPWUG RAWDATA*/
		I("[MP_LPWUG_TEST_RAW]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_LPWUG_RAWDATA,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("9. MP_LPWUG_TEST_RAW: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_LPWUG_RAWDATA_MIN],
			g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MIN]);
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_LPWUG_RAWDATA_MAX],
			g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MAX]);
	}

	if (g_inspt_crtra_flag[IDX_LPWUG_NOISE_MAX] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_NOISE_MIN] == 1) {

		/*10. LPWUG NOISE*/
		I("[MP_LPWUG_TEST_NOISE]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_LPWUG_NOISE,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("10. MP_LPWUG_TEST_NOISE: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_LPWUG_NOISE_MAX],
			g_inspt_crtra_flag[IDX_LPWUG_NOISE_MAX]);
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_LPWUG_NOISE_MIN],
			g_inspt_crtra_flag[IDX_LPWUG_NOISE_MIN]);
	}

	if (g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MIN] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MAX] == 1) {
		/*11. LPWUG IDLE RAWDATA*/
		I("[MP_LPWUG_IDLE_TEST_RAW]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("11. MP_LPWUG_IDLE_TEST_RAW: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_LPWUG_IDLE_RAWDATA_MIN],
			g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MIN]);
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_LPWUG_IDLE_RAWDATA_MAX],
			g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MAX]);
	}

	if (g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MIN] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MAX] == 1) {
		/*12. LPWUG IDLE RAWDATA*/
		I("[MP_LPWUG_IDLE_TEST_NOISE]\n");
		ret +=
			mpTestFunc(HIMAX_INSPECTION_LPWUG_IDLE_NOISE,
					(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) +
					ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
		I("12. MP_LPWUG_IDLE_TEST_NOISE: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_LPWUG_IDLE_NOISE_MIN],
			g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MIN]);
		I("Now %s :flag=%d\n",
			g_hx_inspt_crtra_name[IDX_LPWUG_IDLE_NOISE_MAX],
			g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MAX]);
	}

/* check press power key or not for LPWUG test item*/
	if ((g_inspt_crtra_flag[IDX_LPWUG_NOISE_MAX] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_NOISE_MIN] == 1)
		|| (g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MAX] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MIN] == 1)
		|| (g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MAX] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MIN] == 1)
		|| (g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MAX] == 1
		&& g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MIN] == 1)) {
		himax_press_powerkey();
	}

	hx_test_data_pop_out(g_rslt_data, g_file_path);
	g_core_fp.fp_sense_off(true);
	himax_set_N_frame(1, HIMAX_INSPECTION_NOISE);
#ifndef HX_ZERO_FLASH
	if (g_core_fp.fp_reload_disable != NULL)
		g_core_fp.fp_reload_disable(0);
#endif
	g_core_fp.fp_sense_on(0);

END_FUNC:
	himax_self_test_data_deinit();

	I("running status = %d \n", ret);

	if (ret != 0)
		ret = 1;

	I("%s:OUT\n", __func__);
	return ret;
}

static void hx_findout_limit(uint32_t * RAW, int *limit_val, int sz_mutual)
{
	int i = 0;
	int tmp = 0;

	for (i = 0; i < sz_mutual; i++) {
		tmp = (int32_t)RAW[i];
		if (tmp > limit_val[0]) {
			limit_val[0] = tmp;
		}
		if (tmp < limit_val[1]) {
			limit_val[1] = tmp;
		}
		tmp = 0;
	}
	I("%s: max=%d, min=%d\n", __func__, limit_val[0], limit_val[1]);
}

static void himax_osr_ctrl(bool enable)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint8_t w_byte = 0;
	uint8_t back_byte = 0;
	uint8_t retry = 10;

	input_info(true, &private_ts->client->dev, "%s %s: Entering\n", HIMAX_LOG_TAG, __func__);

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x70; tmp_addr[0] = 0x88;

	do {
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

		if (enable == 0) {
			tmp_data[0] &= ~(1 << 4);
			tmp_data[0] &= ~(1 << 5);
		} else {
			tmp_data[0] |= (1 << 4);
			tmp_data[0] |= (1 << 5);
		}

		w_byte = tmp_data[0];

		g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		back_byte = tmp_data[0];
		if (w_byte != back_byte)
			input_info(true, &private_ts->client->dev, "%s %s: Write osr_ctrl failed, w_byte = %02X, back_byte = %02X\n",
				HIMAX_LOG_TAG, __func__, w_byte, back_byte);
		else {
			input_info(true, &private_ts->client->dev, "%s %s: Write osr_ctrl correctly, w_byte = %02X, back_byte = %02X\n",
				HIMAX_LOG_TAG, __func__, w_byte, back_byte);
			break;
		}
	} while (--retry > 0);
}

static int hx_turn_on_mp_func(int on)
{
	int rslt = 0;
	int retry = 3;
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint8_t tmp_read[4] = {0};
	/* char *tmp_chipname = private_ts->chip_name; */
	tmp_addr[3] = 0x10;tmp_addr[2] = 0x00;tmp_addr[1] = 0x73;tmp_addr[0] = 0xEC;
	if (on) {
		I("%s : Turn on!\n", __func__);
		if (strcmp(HX_83102D_SERIES_PWON,private_ts->chip_name) == 0) {
			I("%s: need to enter Mp mode!\n",__func__);
			tmp_data[3] = 0x00;tmp_data[2] = 0x10;tmp_data[1] = 0x73;tmp_data[0] = 0x80;
			do {
				g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
				msleep(10);
				g_core_fp.fp_register_read(tmp_addr, 4, tmp_read, false);
				input_info(true, &private_ts->client->dev, "%s %s: now read[2]=0x%02X, read[1]=0x%02X, read[0]=0x%02X!\n",
					HIMAX_LOG_TAG ,__func__, tmp_read[2], tmp_read[1], tmp_read[0]);
				retry--;
			} while ((retry > 0) && (tmp_read[2] != tmp_data[2] && tmp_read[1] != tmp_data[1] && tmp_read[0] != tmp_data[0]));
		} else {
			input_info(true, &private_ts->client->dev, "%s %s:Nothing to be done!\n", HIMAX_LOG_TAG ,__func__);
		}
	} else {
		I("%s : Turn off!\n", __func__);
		if (strcmp(HX_83102D_SERIES_PWON,private_ts->chip_name) == 0) {
			I("%s: need to enter Mp mode!\n",__func__);
			tmp_data[3] = 0x00;tmp_data[2] = 0x00;tmp_data[1] = 0x00;tmp_data[0] = 0x00;
			do {
				g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
				msleep(10);
				g_core_fp.fp_register_read(tmp_addr, 4, tmp_read, false);
				input_info(true, &private_ts->client->dev, "%s %s: now read[2]=0x%02X, read[1]=0x%02X, read[0]=0x%02X!\n",
					HIMAX_LOG_TAG ,__func__, tmp_read[2], tmp_read[1], tmp_read[0]);
				retry--;
			} while( (retry > 0) && (tmp_read[2] != tmp_data[2] && tmp_read[1] != tmp_data[1] && tmp_read[0] != tmp_data[0]));
		} else {
			input_info(true, &private_ts->client->dev, "%s %s:Nothing to be done!\n", HIMAX_LOG_TAG ,__func__);
		}
	}
	return rslt;
}

static int hx_get_one_raw(int32_t * RAW, uint8_t checktype, uint32_t datalen)
{
	int ret = 0;
#ifdef HX_ESD_RECOVERY
	/* skip zero counter*/
	g_zero_event_count = 0;
#endif

		input_info(true, &private_ts->client->dev, "%s Need Change Mode ,target=%s\n",
			HIMAX_LOG_TAG, g_himax_inspection_mode[checktype]);
		g_core_fp.fp_sense_off(true);
		hx_turn_on_mp_func(1);
#ifndef HX_ZERO_FLASH
		if (g_core_fp.fp_reload_disable != NULL)
			g_core_fp.fp_reload_disable(1);
#endif
		/* force to active status */
		g_core_fp.fp_idle_mode(1);

		himax_switch_mode_inspection(checktype);
		if (checktype == HIMAX_INSPECTION_NOISE) {
			himax_osr_ctrl(0); /*disable OSR_HOP_EN*/
			himax_set_N_frame(NOISEFRAME, checktype);
			/* himax_get_noise_base(); */
		} else if (checktype == HIMAX_INSPECTION_ACT_IDLE_RAWDATA
				|| checktype == HIMAX_INSPECTION_ACT_IDLE_NOISE) {
			I("N frame = %d\n", 10);
			himax_set_N_frame(10, checktype);
		} else if (checktype >= HIMAX_INSPECTION_LPWUG_RAWDATA) {
			I("N frame = %d\n", 1);
			himax_set_N_frame(1, checktype);
		} else {
			himax_set_N_frame(2, checktype);
		}
		g_core_fp.fp_sense_on(1);
		ret = himax_wait_sorting_mode(checktype);
		if (ret) {
			input_err(true, &private_ts->client->dev, "%s %s: himax_wait_sorting_mode FAIL\n",
				HIMAX_LOG_TAG, __func__);
			goto END_FUNC;
		}

	himax_switch_data_type(checktype);
	ret = himax_get_rawdata(RAW, datalen, checktype);
	if (ret) {
		input_err(true, &private_ts->client->dev, "%s %s: himax_get_rawdata FAIL\n",
			HIMAX_LOG_TAG, __func__);
		goto END_FUNC;
	}
	g_dc_max = g_core_fp.fp_get_max_dc();

END_FUNC:
	input_info(true, &private_ts->client->dev, "%s %s:return normal status!\n",
		HIMAX_LOG_TAG, __func__);
	g_core_fp.fp_sense_off(true);

	himax_switch_data_type(HIMAX_INSPECTION_BACK_NORMAL);
	himax_switch_mode_inspection(HIMAX_INSPECTION_RAWDATA);
	hx_turn_on_mp_func(0);
	/* change to auto status */
	g_core_fp.fp_idle_mode(0);
	if (checktype == HIMAX_INSPECTION_NOISE)
		himax_osr_ctrl(1); /*enable OSR_HOP_EN*/
	himax_set_N_frame(1, HIMAX_INSPECTION_NOISE);
#ifndef HX_ZERO_FLASH
	if (g_core_fp.fp_reload_disable != NULL)
		g_core_fp.fp_reload_disable(0);
#endif

	msleep(20);
	g_core_fp.fp_sense_on(1);
	msleep(20);

	return ret;
}

void himax_inspection_init(void)
{
	I("%s: enter, %d \n", __func__, __LINE__);

	g_core_fp.fp_chip_self_test = himax_chip_self_test;

	return;
}

/* SEC shipping */
/*
static int hx_get_one_iir(uint32_t	*RAW, uint8_t checktype, uint32_t datalen)
{
	int ret = 0;
	himax_switch_data_type(checktype);
	ret = himax_get_rawdata(RAW, datalen);
	if (ret) {
		E("%s: himax_get_rawdata FAIL\n", __func__);
		return ret;
	}
	himax_switch_data_type(HIMAX_INSPECTION_BACK_NORMAL);
	return ret;
}
*/
int hx_raw_2d_to_1d(int x, int y)
{
	if (y == 0)
		return x;
	return (x + (ic_data->HX_RX_NUM * (y - 1)));
}

int hx_get_3x3_noise(int *RAW, int mid_pos_x, int mid_pos_y)
{
	int ret = NO_ERR;
	int x = 0;
	int y = 0;
	int mid_idx = 0;
	int start_idx = 0;
	int now_idx = 0;
	int sum = 0;
	mid_idx = hx_raw_2d_to_1d(mid_pos_x, mid_pos_y);
	/* in the middle's postion of left and up*/
	start_idx = mid_idx - 1 - ic_data->HX_RX_NUM;

	for (y = 0; y < 3; y++) {
		for (x = 0; x < 3; x++) {
			now_idx = start_idx + (x * 1) + (y * ic_data->HX_RX_NUM);
			sum += RAW[now_idx];
			I("idx_%02d:%03d", now_idx, RAW[now_idx]);
		}
		printk("\n");
	}
	I("Now SUM=%d\n", sum);
	ret = sum;
	return ret;
}

extern int32_t *diag_mutual;
int hx_sensity_test(int *result)
{
	int ret = NO_ERR;
	int *RAW;
	int i = 0;
	int j = 0;
	int index = 0;
	int datalen = 0;
	int tx_num = ic_data->HX_TX_NUM;
	int rx_num = ic_data->HX_RX_NUM;
	int sens_pt_grp[5][2];

	I("%s:Entering!\n", __func__);
	/* 1: y, 0:x*/
	sens_pt_grp[0][1] = tx_num / 4;
	sens_pt_grp[0][0] = rx_num - 1 - rx_num / 4;	//rx_num / 4;
	sens_pt_grp[1][1] = tx_num - 1 - tx_num / 4;
	sens_pt_grp[1][0] = rx_num - 1 - rx_num / 4;	//rx_num / 4;
	sens_pt_grp[2][1] = tx_num / 2;
	sens_pt_grp[2][0] = rx_num / 2;
	sens_pt_grp[3][1] = tx_num / 4;
	sens_pt_grp[3][0] = rx_num / 4;	//rx_num-1 - rx_num / 4;
	sens_pt_grp[4][1] = tx_num - 1 - tx_num / 4;
	sens_pt_grp[4][0] = rx_num / 4;	//rx_num-1 - rx_num / 4;
	datalen = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM)
			+ ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	RAW = kzalloc(sizeof(int) * datalen, GFP_KERNEL);
	/* get one time, but now markup it*/
	/* ret = hx_get_one_iir(RAW, HIMAX_INSPECTION_RAWDATA, datalen); */

	memcpy(RAW, diag_mutual, sizeof(int) * datalen);
	if (g_ts_dbg != 0) {
		for (j = 0; j < ic_data->HX_RX_NUM; j++) {
			if (j == 0) {
				printk("      RX%2d", j + 1);
			} else {
				printk("  RX%2d", j + 1);
			}
		}
		printk("\n");

		for (i = 0; i < ic_data->HX_TX_NUM; i++) {
			printk("TX%2d", i + 1);
			for (j = 0; j < ic_data->HX_RX_NUM; j++) {
				index = i * j;
				printk("%5d ", RAW[index]);
			}
			printk("\n");
		}
	}
	for (i = 0; i < 5; i++) {
		I("now:%d,x=%d,y=%d\n", i, sens_pt_grp[i][0],
			sens_pt_grp[i][1]);
		result[i] =
			hx_get_3x3_noise(RAW, sens_pt_grp[i][0], sens_pt_grp[i][1]);
	}
	kfree(RAW);
	I("%s:End!\n", __func__);
	return ret;
}

#ifdef SEC_FACTORY_MODE
#define HIMAX_UMS_FW_PATH "/sdcard/Firmware/TSP/himax.fw"
#define LEN_RSLT	128
#define FACTORY_BUF_SIZE    PAGE_SIZE
#define BUILT_IN            (0)
#define UMS                 (1)

#define TSP_NODE_DEBUG      (0)
#define TSP_CM_DEBUG        (0)
#define TSP_CH_UNUSED       (0)
#define TSP_CH_SCREEN       (1)
#define TSP_CH_GTX          (2)
#define TSP_CH_KEY          (3)
#define TSP_CH_UNKNOWN      (-1)

#include <linux/uaccess.h>
#define MAX_FW_PATH 255
int g_f_edge_border = 0;
int g_f_cal_en = 0;

struct sec_rawdata_buffs *g_sec_raw_buff;
extern unsigned long FW_VER_MAJ_FLASH_ADDR;
extern unsigned long FW_VER_MIN_FLASH_ADDR;
extern unsigned long CFG_VER_MAJ_FLASH_ADDR;
extern unsigned long CFG_VER_MIN_FLASH_ADDR;
extern unsigned long CID_VER_MAJ_FLASH_ADDR;
extern unsigned long CID_VER_MIN_FLASH_ADDR;
extern unsigned long PANEL_VERSION_ADDR;
extern bool fw_update_complete;

static void not_support_cmd(void *dev_data)
{
	char buf[LEN_RSLT] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);
	snprintf(buf, sizeof(buf), "%s", "NA");

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &data->client->dev, "%s %s: \"%s(%d)\"\n",
			HIMAX_LOG_TAG, __func__, buf, (int)strnlen(buf,
								sizeof(buf)));
	return;
}

static void check_connection(void *dev_data)
{
	char buf[LEN_RSLT] = { 0 };
	uint8_t tmp_data[DATA_LEN_4] = { 0 };
	int ret_val = NO_ERR;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);
	ret_val =
		g_core_fp.fp_register_read(pfw_op->addr_chk_fw_status, DATA_LEN_4,
					tmp_data, 0);

	if (ret_val == NO_ERR) {
		I("R900000A8 : [3]=0x%02X,[2]=0x%02X,[1]=0x%02X,[0]=0x%02X\n",
			tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
		snprintf(buf, sizeof(buf), "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		snprintf(buf, sizeof(buf), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_chip_vendor(void *dev_data)
{
	char buf[LEN_RSLT] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);
	snprintf(buf, sizeof(buf), "%s", "HIMAX");

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "IC_VENDOR");
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_chip_name(void *dev_data)
{
	char buf[LEN_RSLT] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	snprintf(buf, sizeof(buf), "%s", private_ts->chip_name);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "IC_NAME");
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_chip_id(void *dev_data)
{
	char buf[LEN_RSLT] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	snprintf(buf, sizeof(buf), "%#02x", ic_data->vendor_sensor_id);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void fw_update(void *dev_data)
{
	int ret = 0;
	char buf[LEN_RSLT] = { 0 };
	char fw_path[MAX_FW_PATH + 1];
	unsigned char *upgrade_fw = NULL;
	struct file *filp = NULL;
	mm_segment_t oldfs;
	const struct firmware *firmware = NULL;
	long fsize = 0;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);
#if defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
	if (sec->cmd_param[0] == 1) {
		sec->cmd_state = SEC_CMD_STATUS_OK;
		snprintf(buf, sizeof(buf), "%s", "OK");
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		input_info(true, &data->client->dev, "%s: user_ship, success\n", __func__);
		return;
	}
#endif

	input_info(true, &data->client->dev, "%s %s(), %d\n", HIMAX_LOG_TAG,
			__func__, sec->cmd_param[0]);

	switch (sec->cmd_param[0]) {
	case BUILT_IN:
		sec->cmd_state = SEC_CMD_STATUS_OK;
#ifdef CONFIG_TOUCHSCREEN_HIMAX_DEBUG
		fw_update_complete = false;
#endif
		himax_int_enable(0);
		memset(fw_path, 0, MAX_FW_PATH);
		snprintf(fw_path, MAX_FW_PATH, "%s",
			data->pdata->i_CTPM_firmware_name);
		ret = request_firmware(&firmware, fw_path, data->dev);
		if (ret) {
			input_err(true, &data->client->dev,
					"%s %s: do not request firmware: %d name as=%s\n",
					HIMAX_LOG_TAG, __func__, ret, fw_path);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto FAIL_END;
		}
		switch (firmware->size) {
		case FW_SIZE_32k:
			ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_32k(
					(unsigned char *) firmware->data, firmware->size, false);
			break;
		case FW_SIZE_60k:
			ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_60k(
					(unsigned char *) firmware->data, firmware->size, false);
			break;
		case FW_SIZE_64k:
			ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_64k(
					(unsigned char *) firmware->data, firmware->size, false);
			break;
		case FW_SIZE_124k:
			ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_124k(
					(unsigned char *) firmware->data, firmware->size, false);
			break;
		case FW_SIZE_128k:
			ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_128k(
					(unsigned char *) firmware->data, firmware->size, false);
			break;
		default:
			input_err(true, &data->client->dev,
					"%s %s: does not support fw size %d\n",
					HIMAX_LOG_TAG, __func__, firmware->size);
		}

		if (ret == 0)
			sec->cmd_state = SEC_CMD_STATUS_FAIL;

		release_firmware(firmware);
		break;
	case UMS:
		sec->cmd_state = SEC_CMD_STATUS_OK;
		himax_int_enable(0);
		
		filp = filp_open(HIMAX_UMS_FW_PATH, O_RDONLY, S_IRUSR);
		if (IS_ERR(filp)) {
			input_err(true, &data->client->dev,
					"%s %s: open firmware file failed\n",
					HIMAX_LOG_TAG, __func__);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			break;
		}
		oldfs = get_fs();
		set_fs(KERNEL_DS);

		fsize = filp->f_path.dentry->d_inode->i_size;
		upgrade_fw =
			kzalloc(sizeof(unsigned char) * fsize, GFP_KERNEL);

		/* read the latest firmware binary file */
		ret =
			filp->f_op->read(filp, upgrade_fw,
					sizeof(unsigned char) * fsize,
					&filp->f_pos);
		if (ret < 0) {
			input_err(true, &data->client->dev,
					"%s %s: read firmware file failed\n",
					HIMAX_LOG_TAG, __func__);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			break;
		}
		filp_close(filp, NULL);
		set_fs(oldfs);

		switch (fsize) {
		case FW_SIZE_32k:
			ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_32k(
						upgrade_fw, fsize, false);
			break;
		case FW_SIZE_60k:
			ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_60k(
						upgrade_fw, fsize, false);
			break;
		case FW_SIZE_64k:
			ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_64k(
						upgrade_fw, fsize, false);
			break;
		case FW_SIZE_124k:
			ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_124k(
						upgrade_fw, fsize, false);
			break;
		case FW_SIZE_128k:
			ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_128k(
						upgrade_fw, fsize, false);
			break;
		default:
			input_err(true, &data->client->dev,
					"%s %s: does not support fw size %d\n",
					HIMAX_LOG_TAG, __func__, fsize);
		}

		if (ret == 0)
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
		break;
	default:
		input_err(true, &data->client->dev,
				"%s %s(), Invalid fw file type!\n", HIMAX_LOG_TAG,
				__func__);
		goto FAIL_END;
	}

	if (upgrade_fw)
		kfree(upgrade_fw);

	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_read_FW_ver();
	g_core_fp.fp_touch_information();
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(true, false);
#else
	g_core_fp.fp_sense_on(0x00);
#endif
FAIL_END:
	if (sec->cmd_state == SEC_CMD_STATUS_OK)
		snprintf(buf, sizeof(buf), "%s", "OK");
	else
		snprintf(buf, sizeof(buf), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	himax_int_enable(1);
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_fw_ver_bin(void *dev_data)
{
	int ret = 0;
	uint8_t bin_ver_ic_name = 0; /* self definition*/
	uint8_t bin_ver_proj = 0; /* CID Maj */
	uint8_t bin_ver_modul = 0; /* Panel Ver */
	uint8_t bin_ver_fw = 0; /* CID Min*/
	int bin_fw_ver = 0;
	int bin_cid_ver = 0;
	char buf[LEN_RSLT] = { 0 };
	char fw_path[MAX_FW_PATH + 1];
	const struct firmware *firmware = NULL;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	snprintf(fw_path, MAX_FW_PATH, "%s",
		private_ts->pdata->i_CTPM_firmware_name);
	input_info(true, &data->client->dev, "%s fw_path=%s\n", HIMAX_LOG_TAG,
			fw_path);

	ret = request_firmware(&firmware, fw_path, private_ts->dev);
	if (ret) {
		input_err(true, &data->client->dev,
				"%s %s: do not request firmware: %d name as=%s\n",
				HIMAX_LOG_TAG, __func__, ret, fw_path);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf),
			"HX: get bin fail please check bin file!\n");
	} else {
		if(strcmp(private_ts->pdata->proj_name ,"M20") == 0) {
			bin_fw_ver =
				(firmware->data[FW_VER_MAJ_FLASH_ADDR] << 8) |
				firmware->data[FW_VER_MIN_FLASH_ADDR];
			bin_cid_ver =
				(firmware->data[CID_VER_MAJ_FLASH_ADDR] << 8) |
				firmware->data[CID_VER_MIN_FLASH_ADDR];

			snprintf(buf, sizeof(buf), "HX%04X%04X", bin_fw_ver, bin_cid_ver);
		} else {
			I("Now project name = %s\n", private_ts->pdata->proj_name);
			if(strcmp(HX_83112A_SERIES_PWON,private_ts->chip_name) == 0)
				bin_ver_ic_name = 0x01;
			else if(strcmp(HX_83102D_SERIES_PWON,private_ts->chip_name) == 0)
				bin_ver_ic_name = 0x02;
			else
				bin_ver_ic_name = 0x00;
			bin_ver_proj = firmware->data[CID_VER_MAJ_FLASH_ADDR];
			bin_ver_modul = firmware->data[PANEL_VERSION_ADDR];
			bin_ver_fw = firmware->data[CID_VER_MIN_FLASH_ADDR];

			snprintf(buf, sizeof(buf), "HX%02X%02X%02X%02X", bin_ver_ic_name, bin_ver_proj, bin_ver_modul, bin_ver_fw);
		}
	}
	if (firmware) {
		release_firmware(firmware);
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "FW_VER_BIN");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_config_ver(void *dev_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	char buf[LEN_RSLT] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buf, sizeof(buf), "%s_%02X%02X", "HX",
		ic_data->vendor_touch_cfg_ver,
		ic_data->vendor_display_cfg_ver);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_checksum_data(void *dev_data)
{
	char buf[LEN_RSLT] = { 0 };
	u32 chksum = 0;
	u32 chksum_size = 0;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	if (strcmp(HX_83102D_SERIES_PWON,private_ts->chip_name) == 0) {
		chksum_size = FW_SIZE_128k;
	} else {
		chksum_size = FW_SIZE_64k;
	}
	I("Now Size = %d\n", chksum_size);

	himax_int_enable(0);
	msleep(10);
	g_core_fp.fp_sense_off(true);
	msleep(10);

	chksum =
		g_core_fp.fp_check_CRC(pfw_op->addr_program_reload_from, chksum_size);
	msleep(10);

	g_core_fp.fp_sense_on(0);
	msleep(10);
	himax_int_enable(1);
	msleep(10);

	/*
	chksum == 0 => checksum pass
	chksum != 0 => checksum fail
	*/

	snprintf(buf, sizeof(buf), "0x%06X", chksum);
	if (chksum == 0)
		sec->cmd_state = SEC_CMD_STATUS_OK;
	else
		sec->cmd_state = SEC_CMD_STATUS_FAIL;

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_crc_check(void *dev_data)
{
	char buf[LEN_RSLT] = { 0 };
	u32 chksum = 0;
	u32 chksum_size = 0;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	if (strcmp(HX_83102D_SERIES_PWON,private_ts->chip_name) == 0) {
		chksum_size = FW_SIZE_128k;
	} else {
		chksum_size = FW_SIZE_64k;
	}
	I("Now Size = %d\n", chksum_size);

	himax_int_enable(0);
	msleep(10);
	g_core_fp.fp_sense_off(true);
	msleep(10);

	chksum =
		g_core_fp.fp_check_CRC(pfw_op->addr_program_reload_from, chksum_size);
	msleep(10);

	g_core_fp.fp_sense_on(0);
	msleep(10);
	himax_int_enable(1);
	msleep(10);

	/*
	chksum == 0 => checksum pass
	chksum != 0 => checksum fail
	*/

	if (chksum == 0) {
		snprintf(buf, sizeof(buf), "%s", "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_fw_ver_ic(void *dev_data)
{
	uint8_t ic_ver_ic_name = 0; /* self definition*/
	uint8_t ic_ver_proj = 0; /* CID Maj */
	uint8_t ic_ver_modul = 0; /* Panel Ver */
	uint8_t ic_ver_fw = 0; /* CID Min*/
	
	char buf[LEN_RSLT] = { 0 };
	char model[16] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	g_core_fp.fp_sense_off(true);
	g_core_fp.fp_read_FW_ver();
	g_core_fp.fp_sense_on(0);

	I("Now project name = %s\n", private_ts->pdata->proj_name);

	if (strcmp(private_ts->pdata->proj_name ,"M20") == 0) {
		snprintf(buf, sizeof(buf), "HX%04X%04X", ic_data->vendor_fw_ver,
			(ic_data->vendor_cid_maj_ver << 8 | ic_data->vendor_cid_min_ver));
	} else {
		if(strcmp(HX_83112A_SERIES_PWON,private_ts->chip_name) == 0)
			ic_ver_ic_name = 0x01;
		else if(strcmp(HX_83102D_SERIES_PWON,private_ts->chip_name) == 0)
			ic_ver_ic_name = 0x02;
		else
			ic_ver_ic_name = 0x00;
		ic_ver_proj = ic_data->vendor_cid_maj_ver;
		ic_ver_modul = ic_data->vendor_panel_ver;
		ic_ver_fw = ic_data->vendor_cid_min_ver;

		snprintf(buf, sizeof(buf), "HX%02X%02X%02X%02X", ic_ver_ic_name, ic_ver_proj, ic_ver_modul, ic_ver_fw);
		snprintf(model, sizeof(model), "HX%02X%02X", ic_ver_ic_name, ic_ver_proj);
	}
	
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING) {
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "FW_VER_IC");

		if (data->pdata->item_version > 1)
			sec_cmd_set_cmd_result_all(sec, model, strnlen(model, sizeof(model)), "FW_MODEL");
	}
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void set_edge_mode(void *dev_data)
{
	int ret = 0;
	char buf[LEN_RSLT] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	msleep(200);

	if (data->suspended == true) {
		input_info(true, &data->client->dev,
				"%s %s: It had entered suspend, skip this command \n",
				HIMAX_LOG_TAG, __func__);
		return;
	}
	
	sec_cmd_set_default_result(sec);

	input_info(true, &data->client->dev, "%s %s(), %d\n", HIMAX_LOG_TAG,
			__func__, sec->cmd_param[0]);

	switch (sec->cmd_param[0]) {
	case 0:
		ret = g_core_fp.set_edge_border(FW_EDGE_BORDER_OFF);
		if (ret == 0) {
			g_f_edge_border = FW_EDGE_BORDER_OFF;
			input_info(true, &data->client->dev,
					"%s %s(), Unset Edge Mode\n", HIMAX_LOG_TAG,
					__func__);
		}
		break;
	case 1:
		ret = g_core_fp.set_edge_border(FW_EDGE_BORDER_ON);
		if (ret == 0) {
			g_f_edge_border = FW_EDGE_BORDER_ON;
			input_info(true, &data->client->dev,
					"%s %s(), Set Edge Mode\n", HIMAX_LOG_TAG,
					__func__);
		}
		break;
	default:
		input_err(true, &data->client->dev,
				"%s %s(), Invalid Argument\n", HIMAX_LOG_TAG,
				__func__);
		break;
	}

	if (ret) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf), "%s", "NG");
	} else {
		sec->cmd_state = SEC_CMD_STATUS_OK;
		snprintf(buf, sizeof(buf), "%s", "OK");
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

/* only support Letter box */
static void set_grip_data(void *dev_data)
{
	int ret = 0;
	int retry = 10;
	char buf[LEN_RSLT] = { 0 };
	uint8_t write_data[4];
	uint8_t read_data[4] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	if (data->suspended) {
		input_err(true, &data->client->dev,
			"%s %s: now IC status is OFF\n", HIMAX_LOG_TAG, __func__);
		goto err_grip_data;
	}

	g_core_fp.fp_register_read(pfw_op->addr_edge_border, DATA_LEN_4, write_data, false);

	if (sec->cmd_param[0] == 2) {
		if (sec->cmd_param[1] == 0) {
			write_data[3] = 0x00; write_data[2] = 0x00;
			write_data[1] = 0x00;
		} else if (sec->cmd_param[1] == 1) {
			write_data[3] = 0x11; write_data[2] = sec->cmd_param[4];
			write_data[1] = sec->cmd_param[5];
		} else {
			input_err(true, &private_ts->client->dev, "%s %s: cmd1 is abnormal, %d\n",
					HIMAX_LOG_TAG, __func__, sec->cmd_param[1]);
			goto err_grip_data;
		}
	} else {
		input_err(true, &private_ts->client->dev, "%s %s: cmd0 is abnormal, %d\n",
			HIMAX_LOG_TAG, __func__, sec->cmd_param[0]);
		goto err_grip_data;
	}

	do {
		g_core_fp.fp_register_write(pfw_op->addr_edge_border, DATA_LEN_4, write_data, false);
		msleep(10);
		g_core_fp.fp_register_read(pfw_op->addr_edge_border, DATA_LEN_4, read_data, false);

		if (read_data[3] == write_data[3] && read_data[2] == write_data[2]
			&& read_data[1] == write_data[1] && read_data[0] == write_data[0]) {
			ret = 1;
		} else {
			retry--;
			input_err(true, &private_ts->client->dev, "%s %s: write register retry(%d)\n",
					HIMAX_LOG_TAG, __func__, retry);
		}
	} while (retry > 0 && ret == 0);

err_grip_data:
	if (ret == 1) {
		sec->cmd_state = SEC_CMD_STATUS_OK;
		snprintf(buf, sizeof(buf), "%s", "OK");
	} else {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf), "%s", "NG");
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_threshold(void *dev_data)
{
	char buf[LEN_RSLT] = { 0 };
	int threshold = 0;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	threshold = g_core_fp.get_rport_thrsh();

	if (threshold > 0) {
		snprintf(buf, sizeof(buf), "0x%02X", threshold);
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		snprintf(buf, sizeof(buf), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_scr_x_num(void *dev_data)
{
	int val = -1;
	char buf[LEN_RSLT] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	val = ic_data->HX_X_RES;
	if (val >= 0) {
		snprintf(buf, sizeof(buf), "%u", val);
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_scr_y_num(void *dev_data)
{
	int val = -1;
	char buf[LEN_RSLT] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	val = ic_data->HX_Y_RES;
	if (val >= 0) {
		snprintf(buf, sizeof(buf), "%u", val);
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_all_x_num(void *dev_data)
{
	int val = -1;
	char buf[LEN_RSLT] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	val = ic_data->HX_TX_NUM;	/* device direction, long side is length */
	if (val >= 0) {
		snprintf(buf, sizeof(buf), "%u", val);
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_all_y_num(void *dev_data)
{
	int val = -1;
	char buf[LEN_RSLT] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	val = ic_data->HX_RX_NUM;	/* device direction, short side is width */
	if (val >= 0) {
		snprintf(buf, sizeof(buf), "%u", val);
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void hx_print_frame(struct himax_ts_data *ts, uint32_t *frame)
{
	int i = 0;
	char msg[15] = { 0 };
	int msg_len = 15;
	char *buf = NULL;

	buf = kzalloc(ic_data->HX_RX_NUM * 10, GFP_KERNEL);
	if (!buf)
		return;
	for (i = 0; i < ic_data->HX_RX_NUM; i++) {
		if (i == 0)
			snprintf(msg, msg_len, "        RX%2d", i + 1);
		else
			snprintf(msg, msg_len, "  RX%2d", i + 1);

		strncat(buf, msg, msg_len);
	}
	input_info(true, &ts->client->dev, "%s\n", buf);
	buf[0] = '\0';

	for (i = 0; i < ic_data->HX_TX_NUM * ic_data->HX_RX_NUM; i++) {
		snprintf(msg, msg_len, "%5d ", frame[i]);
		strncat(buf, msg, msg_len);
		if (i % ic_data->HX_RX_NUM == (ic_data->HX_RX_NUM - 1)) {
			input_info(true, &ts->client->dev, "TX%2d %s\n", (i / ic_data->HX_RX_NUM + 1), buf);
			buf[0] = '\0';
		}
	}
	kfree(buf);
}

extern int hx_write_4k_flash_flow(uint32_t start_addr, uint8_t * write_data,
				uint32_t write_len);
static void get_rawcap(void *dev_data)
{
	uint32_t *RAW = NULL;
	int datalen = 0;
	int val = 0;
	char buf[LEN_RSLT] = { 0 };
#ifdef HX_SAVE_RAW_TO_FLASH
	int i = 0;
	uint8_t *flash_data = NULL;
	uint32_t flash_size = HX_SZ_4K;	/* 4K */
	uint32_t rawdata_size;
#endif
	int limit_val[2] = { 0 };	/* 0: max, 1: min */
	int sz_mutual = ic_data->HX_TX_NUM * ic_data->HX_RX_NUM;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	datalen =
		(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM +
		ic_data->HX_RX_NUM;
	RAW = kzalloc(sizeof(uint32_t) * datalen, GFP_KERNEL);
	if (!RAW) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf), "%s:failed to allocate memory", __func__);
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		return;
	}
#ifdef HX_SAVE_RAW_TO_FLASH
	flash_data = kzalloc(sizeof(uint8_t) * flash_size, GFP_KERNEL);	
	if (!flash_data) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf), "%s:failed to allocate memory", __func__);
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		kfree(RAW);
		return;
	}
#endif
	himax_int_enable(0);
	val = hx_get_one_raw(RAW, HIMAX_INSPECTION_RAWDATA, datalen);

	if (val < 0) {
		input_err(true, &data->client->dev,
				"%s %s: get rawdata fail!\n", HIMAX_LOG_TAG,
				__func__);
		snprintf(buf, sizeof(buf), "%s:get fail", __func__);
		goto END_OUPUT;
	}

	limit_val[0] = -99999;	/* max */
	limit_val[1] = 99999;	/* min */
	hx_findout_limit(RAW, limit_val, sz_mutual);

	memcpy(&g_sec_raw_buff->_rawdata[0], &RAW[0],
			sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM);

	hx_print_frame(data, g_sec_raw_buff->_rawdata);
#ifdef HX_SAVE_RAW_TO_FLASH
	/* save to flash */
	/* 1. data size*/
	rawdata_size = sizeof(uint32_t) * datalen;
	I("Now rawdata size=%d\n", rawdata_size);
	flash_data[0] = rawdata_size % 0x100;
	flash_data[1] = (rawdata_size >> 8) % 0x100;
	flash_data[2] = (rawdata_size >> 16) % 0x100;
	flash_data[3] = (rawdata_size >> 24) % 0x100;
	I("Now flash_data[0]=0x%02X,[1]=0x%02X,[2]=0x%02X,[3]=0x%02X\n",
		flash_data[0], flash_data[1], flash_data[2], flash_data[3]);
	/*2. data type */
	flash_data[4] = 0x02;
	flash_data[5] = 0x02;
	flash_data[6] = 0x02;
	flash_data[7] = 0x02;
	/*3. save data to buffer*/
	if (flash_size - 8 > rawdata_size) {
		I("Now flash_size > rawdata_size!\n");
		for (i = 0; i < ic_data->HX_TX_NUM * ic_data->HX_RX_NUM; i++) {
			flash_data[8 + i * 2] = RAW[i] % 0x100;
			flash_data[8 + i * 2 + 1] = (RAW[i] >> 8) % 0x100;
		}
	} else {
		I("Now rawdata_size > flash_size!\n");
		for (i = 0; i * 2 < (flash_size - 8); i++) {
			flash_data[8 + i * 2] = RAW[i] % 0x100;
			flash_data[8 + i * 2 + 1] = (RAW[i] >> 8) % 0x100;
		}
	}
	/* 4. save to flash*/
	flash_data[0] = rawdata_size % 0x100;
	flash_data[1] = (rawdata_size >> 8) % 0x100;
	flash_data[2] = (rawdata_size >> 16) % 0x100;
	flash_data[3] = (rawdata_size >> 24) % 0x100;
	hx_write_4k_flash_flow(HX_ADR_RAW_FLASH, flash_data, flash_size);
#endif
END_OUPUT:
	if (val >= 0) {
		g_sec_raw_buff->f_ready_rawdata = HX_RAWDATA_READY;
		input_info(true, &data->client->dev, "%s %s: ret val is ok!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%d,%d", limit_val[1], limit_val[0]);	/*min,max */
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		g_sec_raw_buff->f_ready_rawdata = HX_RAWDATA_NOT_READY;
		input_err(true, &data->client->dev, "%s %s: ret val is fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s,error_code=%d", "NG", val);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	I("Min=%d, Max=%d\n", limit_val[1], limit_val[0]);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "RAWCAP");

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
	kfree(RAW);
#ifdef HX_SAVE_RAW_TO_FLASH
	kfree(flash_data);
#endif
	himax_int_enable(1);
}

static void run_rawcap_read_all(void *dev_data)
{
	char temp[SEC_CMD_STR_LEN] = { 0 };
	char *buf = NULL;
	int i = 0;
	char msg[SEC_CMD_STR_LEN] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	get_rawcap(sec);

	sec_cmd_set_default_result(sec);

	if (g_sec_raw_buff->f_ready_rawdata != HX_RAWDATA_READY) {
		input_err(true, &data->client->dev,
				"%s Need to get rawdata first!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}
	
	buf = kzalloc(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM * 10, GFP_KERNEL);
	if (!buf) {
		input_err(true, &data->client->dev,
				"%s failed to allocate memory!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}

	for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
		if (i != (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM - 1)) {
			snprintf(msg, SEC_CMD_STR_LEN, "%d,", (int)g_sec_raw_buff->_rawdata[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		} else {
			snprintf(msg, SEC_CMD_STR_LEN, "%d", (int)g_sec_raw_buff->_rawdata[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		}
		
	}

	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, ic_data->HX_TX_NUM * ic_data->HX_RX_NUM * 10));

	kfree(buf);
	return;
END_OUPUT:
	snprintf(temp, SEC_CMD_STR_LEN, "Test Fail");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, temp, strnlen(temp, SEC_CMD_STR_LEN));
}

static void get_open(void *dev_data)
{
	uint32_t *RAW = NULL;
	int datalen = 0;
	int val = 0;
	char buf[LEN_RSLT] = { 0 };
	int limit_val[2] = { 0 };	/* 0: max, 1: min */
	int sz_mutual = ic_data->HX_TX_NUM * ic_data->HX_RX_NUM;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	datalen =(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM)
			+ ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	RAW = kzalloc(sizeof(uint32_t) * datalen, GFP_KERNEL);
	if (!RAW) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf), "%s:failed to allocate memory", __func__);
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		return;
	}

	himax_int_enable(0);
	val = hx_get_one_raw(RAW, HIMAX_INSPECTION_OPEN, datalen);

	if (val < 0) {
		input_err(true, &data->client->dev, "%s %s: get open fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s:get fail\n", __func__);
		goto END_OUPUT;
	}

	limit_val[0] = -9999;	/* max */
	limit_val[1] = 9999;	/* min */
	hx_findout_limit(RAW, limit_val, sz_mutual);

	memcpy(&g_sec_raw_buff->_open[0], &RAW[0],
			sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM);

	hx_print_frame(data, g_sec_raw_buff->_open);

END_OUPUT:
	if (val >= 0) {
		g_sec_raw_buff->f_ready_open = HX_RAWDATA_READY;
		input_info(true, &data->client->dev, "%s %s: ret val is ok!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%d,%d", limit_val[1], limit_val[0]);	/*min,max */
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		g_sec_raw_buff->f_ready_open = HX_RAWDATA_NOT_READY;
		input_err(true, &data->client->dev, "%s %s: ret val is fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s,error_code=%d", "NG", val);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	I("Min=%d, Max=%d\n", limit_val[1], limit_val[0]);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "OPEN");
	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
	kfree(RAW);
	himax_int_enable(1);
}

static void run_open_read_all(void *dev_data)
{
	char temp[SEC_CMD_STR_LEN] = { 0 };
	char *buf = NULL;
	int i = 0;
	char msg[SEC_CMD_STR_LEN] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	get_open(sec);

	sec_cmd_set_default_result(sec);

	if (g_sec_raw_buff->f_ready_open != HX_RAWDATA_READY) {
		input_err(true, &data->client->dev,
				"%s Need to get rawdata first!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}
	
	buf = kzalloc(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM * 10, GFP_KERNEL);
	if (!buf) {
		input_err(true, &data->client->dev,
				"%s failed to allocate memory!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}

	for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
		if (i != (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM - 1)) {
			snprintf(msg, SEC_CMD_STR_LEN, "%d,", (int)g_sec_raw_buff->_open[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		} else {
			snprintf(msg, SEC_CMD_STR_LEN, "%d", (int)g_sec_raw_buff->_open[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		}
		
	}

	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, ic_data->HX_TX_NUM * ic_data->HX_RX_NUM * 10));

	kfree(buf);
	return;
END_OUPUT:
	snprintf(temp, SEC_CMD_STR_LEN, "Test Fail");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, temp, strnlen(temp, SEC_CMD_STR_LEN));
}

static void get_short(void *dev_data)
{
	uint32_t *RAW = NULL;
	int datalen = 0;
	int val = 0;
	char buf[LEN_RSLT] = { 0 };
	int limit_val[2] = { 0 };	/* 0: max, 1: min */
	int sz_mutual = ic_data->HX_TX_NUM * ic_data->HX_RX_NUM;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	datalen = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM)
			+ ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	RAW = kzalloc(sizeof(uint32_t) * datalen, GFP_KERNEL);
	if (!RAW) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf), "%s:failed to allocate memory", __func__);
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		return;
	}

	himax_int_enable(0);
	val = hx_get_one_raw(RAW, HIMAX_INSPECTION_SHORT, datalen);

	if (val < 0) {
		input_err(true, &data->client->dev, "%s %s: get short fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s:get fail\n", __func__);
		goto END_OUPUT;
	}

	limit_val[0] = -9999;	/* max */
	limit_val[1] = 9999;	/* min */
	hx_findout_limit(RAW, limit_val, sz_mutual);

	memcpy(&g_sec_raw_buff->_short[0], &RAW[0],
			sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM);

	hx_print_frame(data, g_sec_raw_buff->_short);

END_OUPUT:
	if (val >= 0) {
		g_sec_raw_buff->f_ready_short = HX_RAWDATA_READY;
		input_info(true, &data->client->dev, "%s %s: ret val is ok!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%d,%d", limit_val[1], limit_val[0]);	/*min,max */
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		g_sec_raw_buff->f_ready_short = HX_RAWDATA_NOT_READY;
		input_err(true, &data->client->dev, "%s %s: ret val is fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s,error_code=%d", "NG", val);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	I("Min=%d, Max=%d\n", limit_val[1], limit_val[0]);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "SHORT");

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
	kfree(RAW);
	himax_int_enable(1);
}

static void run_short_read_all(void *dev_data)
{
	char temp[SEC_CMD_STR_LEN] = { 0 };
	char *buf = NULL;
	int i = 0;
	char msg[SEC_CMD_STR_LEN] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	get_short(sec);

	sec_cmd_set_default_result(sec);

	if (g_sec_raw_buff->f_ready_short != HX_RAWDATA_READY) {
		input_err(true, &data->client->dev,
				"%s Need to get rawdata first!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}
	
	buf = kzalloc(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM * 10, GFP_KERNEL);
	if (!buf) {
		input_err(true, &data->client->dev,
				"%s failed to allocate memory!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}

	for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
		if (i != (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM - 1)) {
			snprintf(msg, SEC_CMD_STR_LEN, "%d,", (int)g_sec_raw_buff->_short[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		} else {
			snprintf(msg, SEC_CMD_STR_LEN, "%d", (int)g_sec_raw_buff->_short[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		}
		
	}

	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, ic_data->HX_TX_NUM * ic_data->HX_RX_NUM * 10));

	kfree(buf);
	return;
END_OUPUT:
	snprintf(temp, SEC_CMD_STR_LEN, "Test Fail");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, temp, strnlen(temp, SEC_CMD_STR_LEN));
}

static void get_mic_open(void *dev_data)
{
	uint32_t *RAW = NULL;
	int datalen = 0;
	int val = 0;
	char buf[LEN_RSLT] = { 0 };
	int limit_val[2] = { 0 };	/* 0: max, 1: min */
	int sz_mutual = ic_data->HX_TX_NUM * ic_data->HX_RX_NUM;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	datalen = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM)
			+ ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	RAW = kzalloc(sizeof(uint32_t) * datalen, GFP_KERNEL);
	if (!RAW) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf), "%s:failed to allocate memory", __func__);
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		return;
	}

	himax_int_enable(0);
	val = hx_get_one_raw(RAW, HIMAX_INSPECTION_MICRO_OPEN, datalen);

	if (val < 0) {
		input_err(true, &data->client->dev,
				"%s %s: get rawdata fail!\n", HIMAX_LOG_TAG,
				__func__);
		snprintf(buf, sizeof(buf), "%s:get fail\n", __func__);
		goto END_OUPUT;
	}

	limit_val[0] = -9999;	/* max */
	limit_val[1] = 9999;	/* min */
	hx_findout_limit(RAW, limit_val, sz_mutual);

	memcpy(&g_sec_raw_buff->_mopen[0], &RAW[0],
			sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM);

	hx_print_frame(data, g_sec_raw_buff->_mopen);

END_OUPUT:
	if (val >= 0) {
		g_sec_raw_buff->f_ready_mopen = HX_RAWDATA_READY;
		input_info(true, &data->client->dev, "%s %s: ret val is ok!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%d,%d", limit_val[1], limit_val[0]);	/*min,max */
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		g_sec_raw_buff->f_ready_mopen = HX_RAWDATA_NOT_READY;
		input_err(true, &data->client->dev, "%s %s: ret val is fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s,error_code=%d", "NG", val);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	I("Min=%d, Max=%d\n", limit_val[1], limit_val[0]);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "MICRO_OPEN");

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
	kfree(RAW);
	himax_int_enable(1);
}

static void run_mic_open_read_all(void *dev_data)
{
	char temp[SEC_CMD_STR_LEN] = { 0 };
	char *buf = NULL;
	int i = 0;
	char msg[SEC_CMD_STR_LEN] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	get_mic_open(sec);

	sec_cmd_set_default_result(sec);

	if (g_sec_raw_buff->f_ready_mopen!= HX_RAWDATA_READY) {
		input_err(true, &data->client->dev,
				"%s Need to get rawdata first!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}
	
	buf = kzalloc(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM * 10, GFP_KERNEL);
	if (!buf) {
		input_err(true, &data->client->dev,
				"%s failed to allocate memory!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}

	for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
		if (i != (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM - 1)) {
			snprintf(msg, SEC_CMD_STR_LEN, "%d,", (int)g_sec_raw_buff->_mopen[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		} else {
			snprintf(msg, SEC_CMD_STR_LEN, "%d", (int)g_sec_raw_buff->_mopen[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		}
		
	}

	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, ic_data->HX_TX_NUM * ic_data->HX_RX_NUM * 10));

	kfree(buf);
	return;
END_OUPUT:
	snprintf(temp, SEC_CMD_STR_LEN, "Test Fail");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, temp, strnlen(temp, SEC_CMD_STR_LEN));
}

static void get_noise(void *dev_data)
{
	int32_t *RAW = NULL;
	int datalen = 0;
	int val = 0;
	char buf[LEN_RSLT] = { 0 };
	int limit_val[2] = { 0 };	/* 0: max, 1: min */
	int sz_mutual = ic_data->HX_TX_NUM * ic_data->HX_RX_NUM;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	datalen = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM)
			+ ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	RAW = kzalloc(sizeof(int32_t) * datalen, GFP_KERNEL);
	if (!RAW) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf), "%s:failed to allocate memory", __func__);
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		return;
	}

	himax_int_enable(0);
	val = hx_get_one_raw(RAW, HIMAX_INSPECTION_NOISE, datalen);

	if (val < 0) {
		input_err(true, &data->client->dev,
				"%s %s: get rawdata fail!\n", HIMAX_LOG_TAG,
				__func__);
		snprintf(buf, sizeof(buf), "%s:get fail\n", __func__);
		goto END_OUPUT;
	}

	limit_val[0] = -9999;	/* max */
	limit_val[1] = 9999;	/* min */
	hx_findout_limit(RAW, limit_val, sz_mutual);

	memcpy(&g_sec_raw_buff->_noise[0], &RAW[0],
			sizeof(int32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM);

	hx_print_frame(data, g_sec_raw_buff->_noise);

END_OUPUT:
	if (val >= 0) {
		g_sec_raw_buff->f_ready_noise = HX_RAWDATA_READY;
		input_info(true, &data->client->dev, "%s %s: ret val is ok!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%d,%d", limit_val[1], limit_val[0]);	/*min,max */
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		g_sec_raw_buff->f_ready_noise = HX_RAWDATA_NOT_READY;
		input_err(true, &data->client->dev, "%s %s: ret val is fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s,error_code=%d", "NG", val);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	I("Min=%d, Max=%d\n", limit_val[1], limit_val[0]);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "NOISE");

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
	kfree(RAW);
	himax_int_enable(1);
}

static void run_noise_read_all(void *dev_data)
{
	char temp[SEC_CMD_STR_LEN] = { 0 };
	char *buf = NULL;
	int i = 0;
	char msg[SEC_CMD_STR_LEN] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	get_noise(sec);

	sec_cmd_set_default_result(sec);

	if (g_sec_raw_buff->f_ready_noise!= HX_RAWDATA_READY) {
		input_err(true, &data->client->dev,
				"%s Need to get rawdata first!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}
	
	buf = kzalloc(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM * 10, GFP_KERNEL);
	if (!buf) {
		input_err(true, &data->client->dev,
				"%s failed to allocate memory!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}

	for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
		if (i != (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM - 1)) {
			snprintf(msg, SEC_CMD_STR_LEN, "%d,", (int)g_sec_raw_buff->_noise[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		} else {
			snprintf(msg, SEC_CMD_STR_LEN, "%d", (int)g_sec_raw_buff->_noise[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		}
		
	}

	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, ic_data->HX_TX_NUM * ic_data->HX_RX_NUM * 10));

	kfree(buf);
	return;
END_OUPUT:
	snprintf(temp, SEC_CMD_STR_LEN, "Test Fail");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, temp, strnlen(temp, SEC_CMD_STR_LEN));
}

static void get_lp_rawcap(void *dev_data)
{
	uint32_t *RAW = NULL;
	int datalen = 0;
	int val = 0;
	char buf[LEN_RSLT] = { 0 };
	int limit_val[2] = { 0 };	/* 0: max, 1: min */
	int sz_mutual = ic_data->HX_TX_NUM * ic_data->HX_RX_NUM;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	datalen = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM)
			+ ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	RAW = kzalloc(sizeof(uint32_t) * datalen, GFP_KERNEL);
	if (!RAW) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf), "%s:failed to allocate memory", __func__);
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		return;
	}

	himax_int_enable(0);
	val = hx_get_one_raw(RAW, HIMAX_INSPECTION_LPWUG_RAWDATA, datalen);

	if (val < 0) {
		input_err(true, &data->client->dev,
				"%s %s: get rawdata fail!\n", HIMAX_LOG_TAG,
				__func__);
		snprintf(buf, sizeof(buf), "%s:get fail\n", __func__);
		goto END_OUPUT;
	}

	limit_val[0] = -9999;	/* max */
	limit_val[1] = 9999;	/* min */
	hx_findout_limit(RAW, limit_val, sz_mutual);

	memcpy(&g_sec_raw_buff->_lp_rawdata[0], &RAW[0],
			sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM);

	hx_print_frame(data, g_sec_raw_buff->_lp_rawdata);

END_OUPUT:
	if (val >= 0) {
		g_sec_raw_buff->f_ready_lp_rawdata = HX_RAWDATA_READY;
		input_info(true, &data->client->dev, "%s %s: ret val is ok!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%d,%d", limit_val[1], limit_val[0]);	/*min,max */
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		g_sec_raw_buff->f_ready_lp_rawdata = HX_RAWDATA_NOT_READY;
		input_err(true, &data->client->dev, "%s %s: ret val is fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s,error_code=%d", "NG", val);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	I("Min=%d, Max=%d\n", limit_val[1], limit_val[0]);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "LP_RAW");

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
	kfree(RAW);
	himax_int_enable(1);
}

static void get_lp_noise(void *dev_data)
{
	uint32_t *RAW = NULL;
	int datalen = 0;
	int val = 0;
	char buf[LEN_RSLT] = { 0 };
	int limit_val[2] = { 0 };	/* 0: max, 1: min */
	int sz_mutual = ic_data->HX_TX_NUM * ic_data->HX_RX_NUM;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	datalen = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM)
			+ ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	RAW = kzalloc(sizeof(uint32_t) * datalen, GFP_KERNEL);
	if (!RAW) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buf, sizeof(buf), "%s:failed to allocate memory", __func__);
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		return;
	}

	himax_int_enable(0);
	val = hx_get_one_raw(RAW, HIMAX_INSPECTION_LPWUG_NOISE, datalen);

	if (val < 0) {
		input_err(true, &data->client->dev,
				"%s %s: get rawdata fail!\n", HIMAX_LOG_TAG,
				__func__);
		snprintf(buf, sizeof(buf), "%s:get fail\n", __func__);
		goto END_OUPUT;
	}

	limit_val[0] = -9999;	/* max */
	limit_val[1] = 9999;	/* min */
	hx_findout_limit(RAW, limit_val, sz_mutual);

	memcpy(&g_sec_raw_buff->_lp_noise[0], &RAW[0],
			sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM);

	hx_print_frame(data, g_sec_raw_buff->_lp_noise);

END_OUPUT:
	if (val >= 0) {
		g_sec_raw_buff->f_ready_lp_noise = HX_RAWDATA_READY;
		input_info(true, &data->client->dev, "%s %s: ret val is ok!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%d,%d", limit_val[1], limit_val[0]);	/*min,max */
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		g_sec_raw_buff->f_ready_lp_noise = HX_RAWDATA_NOT_READY;
		input_err(true, &data->client->dev, "%s %s: ret val is fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s,error_code=%d", "NG", val);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	I("Min=%d, Max=%d\n", limit_val[1], limit_val[0]);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "LP_NOISE");

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
	kfree(RAW);
	himax_int_enable(1);
}

static int hx_gap_hor_raw(int test_type, uint32_t * raw, int *result_raw)
{
	int rx_num = ic_data->HX_RX_NUM;
	int tx_num = ic_data->HX_TX_NUM;
	int tmp_start = 0;
	int tmp_end_idx = 0;
	int i_partial = 0;
	int i = 0;
	int ret_val = NO_ERR;
	int *tmp_result_raw;
	int idx_assign = 0;

	himax_gap_test_horizontal_setting();
	if (g_ts_dbg != 0) {
		I("Print Horizontal ORG RAW\n");
		for (i = 0; i < tx_num * rx_num; i++) {
			printk("%04d,", raw[i]);
			if (i > 0 && i % rx_num == (rx_num - 1))
				I("\n");
		}
	}
	tmp_result_raw = kzalloc(sizeof(int) * rx_num * tx_num, GFP_KERNEL);
	if (!tmp_result_raw) {
		kfree(g_gap_horizontal_part);
		return -ENOMEM;
	}

	for (i_partial = 0; i_partial < g_gap_horizontal_partial; i_partial++) {
		tmp_start	= g_gap_horizontal_part[i_partial];
		if (i_partial+1 == g_gap_horizontal_partial) {
			tmp_end_idx = rx_num - g_gap_horizontal_part[i_partial];
		} else {
			tmp_end_idx = g_gap_horizontal_part[i_partial+1] - g_gap_horizontal_part[i_partial];
		}
		if (i_partial % 2 == 0) {
			himax_cal_gap_data_horizontal(tmp_start, tmp_end_idx, 0, raw, tmp_result_raw);
		} else {
			himax_cal_gap_data_horizontal(tmp_start, tmp_end_idx, 1, raw, tmp_result_raw);
		}
	}

	hx_print_frame(private_ts, tmp_result_raw);

	for (i = 0; i < tx_num*rx_num; i++) {
		if (i % rx_num == 0)    // ignore first colume RX = 0
			continue;

		result_raw[idx_assign++] = tmp_result_raw[i];
	}
	printk("idx_assign = %d\n", idx_assign);
	kfree(g_gap_horizontal_part);
	kfree(tmp_result_raw);
	return ret_val;
}

static void get_gap_data_y(void *dev_data)
{
	int val = 0;
	char buf[LEN_RSLT] = { 0 };
	int limit_val[2] = { 0 };	/* 0: max, 1: min */
	int sz_mutual = ic_data->HX_TX_NUM * (ic_data->HX_RX_NUM - g_gap_horizontal_partial);
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	himax_int_enable(0);
	if (g_sec_raw_buff->f_ready_rawdata != HX_RAWDATA_READY) {
		input_err(true, &data->client->dev,
				"%s %s: need get rawcap firstly\n", HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s:need get rawcap firstly", __func__);
		val = HX_RAW_NOT_READY;
		goto END_OUPUT;
	}

	val = hx_gap_hor_raw(HIMAX_INSPECTION_GAPTEST_RAW, g_sec_raw_buff->_rawdata,
				g_sec_raw_buff->_gap_hor);
	if (val < 0) {
		input_err(true, &private_ts->client->dev, "%s %s: failed to get gap Y\n",
			HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "failed to get gap Y");
		goto END_OUPUT;
	}

	limit_val[0] = -9999;	/* max */
	limit_val[1] = 9999;	/* min */
	hx_findout_limit(g_sec_raw_buff->_gap_hor, limit_val, sz_mutual);

END_OUPUT:
	if (val >= 0) {
		g_sec_raw_buff->f_ready_gap_hor = HX_RAWDATA_READY;
		input_info(true, &data->client->dev, "%s %s: ret val is ok!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%d,%d", limit_val[1], limit_val[0]);	/*min,max */
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		g_sec_raw_buff->f_ready_gap_hor = HX_RAWDATA_NOT_READY;
		input_err(true, &data->client->dev, "%s %s: ret val is fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s,error_code=%d", "NG", val);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	I("Min=%d, Max=%d\n", limit_val[1], limit_val[0]);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "RAW_GAP_Y");

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
	himax_int_enable(1);
}

static void run_raw_gap_y_read_all(void *dev_data)
{
	char temp[SEC_CMD_STR_LEN] = { 0 };
	char *buf = NULL;
	int i = 0;
	char msg[SEC_CMD_STR_LEN] = { 0 };
	int sz_mutual = ic_data->HX_TX_NUM * (ic_data->HX_RX_NUM - g_gap_horizontal_partial);
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	get_gap_data_y(sec);

	sec_cmd_set_default_result(sec);

	if (g_sec_raw_buff->f_ready_gap_hor!= HX_RAWDATA_READY) {
		input_err(true, &data->client->dev,
				"%s Need to get rawdata first!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}
	
	buf = kzalloc(sz_mutual * 10, GFP_KERNEL);
	if (!buf) {
		input_err(true, &data->client->dev,
				"%s failed to allocate memory!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}

	for (i = 0; i < sz_mutual; i++) {
		if (i != (sz_mutual - 1)) {
			snprintf(msg, SEC_CMD_STR_LEN, "%d,", (int)g_sec_raw_buff->_gap_hor[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		} else {
			snprintf(msg, SEC_CMD_STR_LEN, "%d", (int)g_sec_raw_buff->_gap_hor[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		}
		
	}

	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sz_mutual * 10));

	kfree(buf);
	return;
END_OUPUT:
	snprintf(temp, SEC_CMD_STR_LEN, "Test Fail");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, temp, strnlen(temp, SEC_CMD_STR_LEN));
}

static int hx_gap_ver_raw(int test_type, uint32_t * org_raw, int *result_raw)
{
	int i_partial = 0;
	int tmp_start = 0;
	int tmp_end_idx = 0;
	int i = 0;
	int ret_val = NO_ERR;
	int tx_num = ic_data->HX_TX_NUM;
	int rx_num = ic_data->HX_RX_NUM;
	int *tmp_result_raw;
	int idx_assign = 0;

	himax_gap_test_vertical_setting();
	if (g_ts_dbg != 0) {
		I("Print vertical ORG RAW\n");
		for (i = 0; i < tx_num * rx_num; i++) {
			printk("%04d,", org_raw[i]);
			if (i > 0 && i % rx_num == (rx_num - 1))
				I("\n");
		}
	}

	tmp_result_raw = kzalloc(sizeof(int) * rx_num * tx_num, GFP_KERNEL);
	if (!tmp_result_raw) {
		kfree(g_gap_vertical_part);
		return -ENOMEM;
	}

	for (i_partial = 0; i_partial < g_gap_vertical_partial; i_partial++) {
		tmp_start	= g_gap_vertical_part[i_partial] * rx_num;
		if (i_partial + 1 == g_gap_vertical_partial) {
			tmp_end_idx = tx_num - g_gap_vertical_part[i_partial];
		} else {
			tmp_end_idx = g_gap_vertical_part[i_partial + 1] - g_gap_vertical_part[i_partial];
		}
		if (i_partial % 2 == 0) {
			himax_cal_gap_data_vertical(tmp_start, tmp_end_idx, 0, org_raw, tmp_result_raw);
		}	else {
			himax_cal_gap_data_vertical(tmp_start, tmp_end_idx, 1, org_raw, tmp_result_raw);
		}
	}

	hx_print_frame(private_ts, tmp_result_raw);

	for (i = 0; i < tx_num * rx_num; i++) {
		if (i / rx_num == 0 || i / rx_num == tx_num - 1)   // ignore TX 0 and TX 15
			continue;

		result_raw[idx_assign++] = tmp_result_raw[i];
	}
	printk("idx_assign = %d\n", idx_assign);
	kfree(g_gap_vertical_part);
	kfree(tmp_result_raw);
	return ret_val;
}

static void get_gap_data_x(void *dev_data)
{
	int val = 0;
	char buf[LEN_RSLT] = { 0 };
	int limit_val[2] = { 0 };	/* 0: max, 1: min */
	int sz_mutual = (ic_data->HX_TX_NUM - g_gap_vertical_partial) * ic_data->HX_RX_NUM;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);

	himax_int_enable(0);
	if (g_sec_raw_buff->f_ready_rawdata != HX_RAWDATA_READY) {
		input_err(true, &data->client->dev,
				"%s %s: need get rawcap firstly\n", HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s:need get rawcap firstly", __func__);
		val = HX_RAW_NOT_READY;
		goto END_OUPUT;
	}

	val = hx_gap_ver_raw(HIMAX_INSPECTION_GAPTEST_RAW, g_sec_raw_buff->_rawdata,
				g_sec_raw_buff->_gap_ver);
	if (val < 0) {
		input_err(true, &private_ts->client->dev, "%s %s: failed to get gap X\n",
			HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "failed to get gap X");
		goto END_OUPUT;
	}

	limit_val[0] = -9999;	/* max */
	limit_val[1] = 9999;	/* min */
	hx_findout_limit(g_sec_raw_buff->_gap_ver, limit_val, sz_mutual);

END_OUPUT:
	if (val >= 0) {
		g_sec_raw_buff->f_ready_gap_ver = HX_RAWDATA_READY;
		input_info(true, &data->client->dev, "%s %s: ret val is ok!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%d,%d", limit_val[1], limit_val[0]);	/*min,max */
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		g_sec_raw_buff->f_ready_gap_ver = HX_RAWDATA_NOT_READY;
		input_err(true, &data->client->dev, "%s %s: ret val is fail!\n",
				HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s,error_code=%d", "NG", val);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	I("Min=%d, Max=%d\n", limit_val[1], limit_val[0]);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "RAW_GAP_X");

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
	himax_int_enable(1);
}

static void run_raw_gap_x_read_all(void *dev_data)
{
	char temp[SEC_CMD_STR_LEN] = { 0 };
	char *buf = NULL;
	int i = 0;
	char msg[SEC_CMD_STR_LEN] = { 0 };
	int sz_mutual = (ic_data->HX_TX_NUM - g_gap_vertical_partial) * ic_data->HX_RX_NUM;
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	get_gap_data_x(sec);

	sec_cmd_set_default_result(sec);

	if (g_sec_raw_buff->f_ready_gap_ver!= HX_RAWDATA_READY) {
		input_err(true, &data->client->dev,
				"%s Need to get rawdata first!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}
	
	buf = kzalloc(sz_mutual * 10, GFP_KERNEL);
	if (!buf) {
		input_err(true, &data->client->dev,
				"%s failed to allocate memory!\n", HIMAX_LOG_TAG);
		goto END_OUPUT;
	}

	for (i = 0; i < sz_mutual; i++) {
		if (i != (sz_mutual - 1)) {
			snprintf(msg, SEC_CMD_STR_LEN, "%d,", (int)g_sec_raw_buff->_gap_ver[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		} else {
			snprintf(msg, SEC_CMD_STR_LEN, "%d", (int)g_sec_raw_buff->_gap_ver[i]);
			strncat(buf, msg, SEC_CMD_STR_LEN);
		}
		
	}

	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sz_mutual * 10));

	kfree(buf);
	return;
END_OUPUT:
	snprintf(temp, SEC_CMD_STR_LEN, "Test Fail");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, temp, strnlen(temp, SEC_CMD_STR_LEN));
}

static void glove_mode(void *dev_data)
{
	char buf[LEN_RSLT] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec_cmd_set_default_result(sec);
	input_info(true, &data->client->dev, "%s %s,%d\n", HIMAX_LOG_TAG,
			__func__, sec->cmd_param[0]);

	data->glove_enabled = sec->cmd_param[0];

	if (data->suspended) {
		input_err(true, &data->client->dev,
			"%s %s: now IC status is not STATE_POWER_ON\n", HIMAX_LOG_TAG, __func__);
		snprintf(buf, sizeof(buf), "%s", "TSP_turned_off");
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		goto out;
	}

	switch (sec->cmd_param[0]) {
	case 0:
		sec->cmd_state = SEC_CMD_STATUS_OK;
		input_info(true, &data->client->dev,
				"%s %s(), Unset Glove Mode\n", HIMAX_LOG_TAG,
				__func__);
		g_core_fp.fp_set_HSEN_enable(0, false);
		break;
	case 1:
		sec->cmd_state = SEC_CMD_STATUS_OK;
		input_info(true, &data->client->dev,
				"%s %s(), Set Glove Mode\n", HIMAX_LOG_TAG,
				__func__);
		g_core_fp.fp_set_HSEN_enable(1, false);
		break;
	default:
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		input_info(true, &data->client->dev,
				"%s %s(), Invalid Argument\n", HIMAX_LOG_TAG,
				__func__);
		break;
	}

	if (sec->cmd_state == SEC_CMD_STATUS_OK)
		snprintf(buf, sizeof(buf), "%s", "OK");
	else
		snprintf(buf, sizeof(buf), "%s", "NG");
out:
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &data->client->dev, "%s %s: %s(%d)\n", HIMAX_LOG_TAG,
			__func__, buf, (int)strnlen(buf, sizeof(buf)));
}

static void factory_cmd_result_all(void *dev_data)
{
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct sec_cmd_data *sec = (struct sec_cmd_data *)dev_data;
	struct himax_ts_data *data =
		container_of(sec, struct himax_ts_data, sec);

	sec->item_count = 0;
	memset(sec->cmd_result_all, 0x00, SEC_CMD_RESULT_STR_LEN);

	sec->cmd_all_factory_state = SEC_CMD_STATUS_RUNNING;

	snprintf(buf, sizeof(buf), "%d", data->pdata->item_version);
	sec_cmd_set_cmd_result_all(sec, buf, sizeof(buf), "ITEM_VERSION");

	get_chip_vendor(sec);
	get_chip_name(sec);
	get_fw_ver_bin(sec);
	get_fw_ver_ic(sec);

	get_rawcap(sec);
	get_gap_data_x(sec);
	get_gap_data_y(sec);

	if (data->pdata->item_version > 1)
		get_open(sec);

	get_mic_open(sec);
	get_short(sec);
	get_noise(sec);
	/*get_lp_rawcap(sec);*/
	/*get_lp_noise(sec);*/

	sec->cmd_all_factory_state = SEC_CMD_STATUS_OK;
	input_info(true, &data->client->dev, "%s %s: %d%s\n", HIMAX_LOG_TAG,
			__func__, sec->item_count, sec->cmd_result_all);
}

static ssize_t show_close_tsp_test(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return snprintf(buf, FACTORY_BUF_SIZE, "%u\n", 0);
}

struct sec_cmd sec_cmds[] = {
	{SEC_CMD("check_connection", check_connection),},
	{SEC_CMD("fw_update", fw_update),},
	{SEC_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{SEC_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{SEC_CMD("get_config_ver", get_config_ver),},
	{SEC_CMD("get_checksum_data", get_checksum_data),},
	{SEC_CMD("get_threshold", get_threshold),},
	{SEC_CMD("get_chip_vendor", get_chip_vendor),},
	{SEC_CMD("get_chip_name", get_chip_name),},
	{SEC_CMD("get_chip_id", get_chip_id),},
	{SEC_CMD("get_scr_x_num", get_scr_x_num),},
	{SEC_CMD("get_scr_y_num", get_scr_y_num),},
	{SEC_CMD("get_x_num", get_all_x_num),},
	{SEC_CMD("get_y_num", get_all_y_num),},
	{SEC_CMD("get_rawcap", get_rawcap),},
	{SEC_CMD("run_rawcap_read_all", run_rawcap_read_all),},
	{SEC_CMD("get_open", get_open),},
	{SEC_CMD("run_open_read_all", run_open_read_all),},
	{SEC_CMD("get_mic_open", get_mic_open),},
	{SEC_CMD("run_mic_open_read_all", run_mic_open_read_all),},
	{SEC_CMD("get_short", get_short),},
	{SEC_CMD("run_short_read_all", run_short_read_all),},
	{SEC_CMD("get_noise", get_noise),},
	{SEC_CMD("run_noise_read_all", run_noise_read_all),},
	{SEC_CMD("get_lp_rawcap", get_lp_rawcap),},
	{SEC_CMD("get_lp_noise", get_lp_noise),},
	{SEC_CMD("run_jitter_test", not_support_cmd),},
	{SEC_CMD("get_gap_data_x", get_gap_data_x),},
	{SEC_CMD("run_raw_gap_x_read_all", run_raw_gap_x_read_all),},
	{SEC_CMD("get_gap_data_y", get_gap_data_y),},
	{SEC_CMD("run_raw_gap_y_read_all", run_raw_gap_y_read_all),},
	{SEC_CMD("set_tsp_test_result", not_support_cmd),},
	{SEC_CMD("get_tsp_test_result", not_support_cmd),},
	{SEC_CMD("clear_tsp_test_result", not_support_cmd),},
	{SEC_CMD("dead_zone_enable", set_edge_mode),},
	{SEC_CMD("set_grip_data", set_grip_data),},
	{SEC_CMD("factory_cmd_result_all", factory_cmd_result_all),},
	{SEC_CMD("glove_mode", glove_mode),},
	{SEC_CMD("not_support_cmd", not_support_cmd),},
	{SEC_CMD("get_crc_check", get_crc_check),},
};

/* sensitivity mode test */
extern int hx_set_sram_raw(int set_val);
static ssize_t sensitivity_mode_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int result[5] = { 0 };
	hx_sensity_test(result);
	return snprintf(buf, 256, "%d,%d,%d,%d,%d", result[0], result[1],
			result[2], result[3], result[4]);
}

static ssize_t sensitivity_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int mode;

	if (kstrtoint(buf, 10, &mode) < 0) {
		input_err(true, &private_ts->client->dev,
				"%s %s kstrtoint fail\n", HIMAX_LOG_TAG, __func__);
		return count;
	}

	if (unlikely((mode != 0) && (mode != 1)))
		return count;

	if (mode == 0) {
		hx_set_sram_raw(0);	// stop test
		input_info(true, &private_ts->client->dev,
				"%s %sTurn off Sensitivity Measurement\n",
				HIMAX_LOG_TAG, __func__);
	} else {
		hx_set_sram_raw(1);	// start test
		input_info(true, &private_ts->client->dev,
				"%s %sTurn on Sensitivity Measurement\n",
				HIMAX_LOG_TAG, __func__);
	}

	return count;
}

static ssize_t comm_err_count_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	input_info(true, &private_ts->client->dev, "%s %s %d\n",
			HIMAX_LOG_TAG, __func__, private_ts->comm_err_count);

	return snprintf(buf, 256, "%d", private_ts->comm_err_count);
}

static ssize_t comm_err_count_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	input_info(true, &private_ts->client->dev, "%s %s clear\n",
			HIMAX_LOG_TAG, __func__);

	private_ts->comm_err_count = 0;
	return count;

}

static ssize_t module_id_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 256, "HX00%04X%04X", ic_data->vendor_fw_ver,
			(ic_data->vendor_cid_maj_ver << 8 | ic_data->vendor_cid_min_ver));
}

static DEVICE_ATTR(sensitivity_mode, S_IRUGO | S_IWUSR | S_IWGRP,
			sensitivity_mode_show, sensitivity_mode_store);
static DEVICE_ATTR(close_tsp_test, S_IRUGO, show_close_tsp_test, NULL);
static DEVICE_ATTR(comm_err_count, S_IRUGO | S_IWUSR | S_IWGRP,
			comm_err_count_show, comm_err_count_store);
static DEVICE_ATTR(module_id, S_IRUGO, module_id_show, NULL);

static struct attribute *sec_touch_factory_attributes[] = {
	&dev_attr_sensitivity_mode.attr,
	&dev_attr_close_tsp_test.attr,
	&dev_attr_comm_err_count.attr,
	&dev_attr_module_id.attr,
	NULL,
};

static struct attribute_group sec_touch_factory_attr_group = {
	.attrs = sec_touch_factory_attributes,
};

int sec_touch_sysfs(struct himax_ts_data *data)
{
	int ret;

	/* /sys/class/sec/tsp */
	ret = sec_cmd_init(&data->sec, sec_cmds,
				ARRAY_SIZE(sec_cmds), SEC_CLASS_DEVT_TSP);
	if (ret < 0) {
		input_err(true, &data->client->dev,
				"%s %s Failed to create device (tsp)!\n",
				HIMAX_LOG_TAG, __func__);
		goto err_init_cmd;
	}
	ret = sysfs_create_link(&data->sec.fac_dev->kobj,
				&data->input_dev->dev.kobj, "input");
	if (ret < 0)
		input_err(true, &data->client->dev,
				"%s %s: Failed to create input symbolic link\n",
				HIMAX_LOG_TAG, __func__);

	/* /sys/class/sec/tsp/... */
	if (sysfs_create_group
		(&data->sec.fac_dev->kobj, &sec_touch_factory_attr_group)) {
		input_err(true, &data->client->dev,
				"%s %sFailed to create sysfs group(tsp)!\n",
				HIMAX_LOG_TAG, __func__);
		goto err_sec_fac_dev_attr;
	}

	I("Now init rawdata buffs\n");
	g_sec_raw_buff = kzalloc(sizeof(struct sec_rawdata_buffs), GFP_KERNEL);
	g_sec_raw_buff->_rawdata =
		kzalloc(sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM,
			GFP_KERNEL);
	g_sec_raw_buff->_open =
		kzalloc(sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM,
			GFP_KERNEL);
	g_sec_raw_buff->_mopen =
		kzalloc(sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM,
			GFP_KERNEL);
	g_sec_raw_buff->_short =
		kzalloc(sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM,
			GFP_KERNEL);
	g_sec_raw_buff->_noise =
		kzalloc(sizeof(int) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM,
			GFP_KERNEL);
	g_sec_raw_buff->_lp_rawdata =
		kzalloc(sizeof(uint32_t) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM,
			GFP_KERNEL);
	g_sec_raw_buff->_lp_noise =
		kzalloc(sizeof(int) * ic_data->HX_TX_NUM * ic_data->HX_RX_NUM,
			GFP_KERNEL);
	g_sec_raw_buff->_gap_ver =
		kzalloc(sizeof(int) * (ic_data->HX_TX_NUM - g_gap_vertical_partial) * ic_data->HX_RX_NUM, GFP_KERNEL);
	g_sec_raw_buff->_gap_hor =
		kzalloc(sizeof(int) * ic_data->HX_TX_NUM * (ic_data->HX_RX_NUM - g_gap_horizontal_partial), GFP_KERNEL);
	g_sec_raw_buff->f_crtra_ready = HX_THRESHOLD_NOT_READY;
	g_sec_raw_buff->f_ready_rawdata = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_open = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_mopen = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_short = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_noise = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_lp_rawdata = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_lp_noise = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_gap_ver = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_gap_hor = HX_RAWDATA_NOT_READY;

	return 0;

err_sec_fac_dev_attr:
err_init_cmd:
	return -ENODEV;
}

EXPORT_SYMBOL(sec_touch_sysfs);

void sec_touch_sysfs_remove(struct himax_ts_data *data)
{
	input_err(true, &data->client->dev, "%s %s: Entering!\n", HIMAX_LOG_TAG,
			__func__);
	sysfs_remove_link(&data->sec.fac_dev->kobj, "input");
	sysfs_remove_group(&data->sec.fac_dev->kobj,
				&sec_touch_factory_attr_group);
	I("Now remove raw data buffs \n");

	kfree(g_sec_raw_buff->_rawdata);
	kfree(g_sec_raw_buff->_open);
	kfree(g_sec_raw_buff->_mopen);
	kfree(g_sec_raw_buff->_short);
	kfree(g_sec_raw_buff->_noise);
	kfree(g_sec_raw_buff->_lp_rawdata);
	kfree(g_sec_raw_buff->_lp_noise);
	kfree(g_sec_raw_buff->_gap_ver);
	kfree(g_sec_raw_buff->_gap_hor);
	g_sec_raw_buff->f_crtra_ready = HX_THRESHOLD_NOT_READY;
	g_sec_raw_buff->f_ready_rawdata = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_open = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_mopen = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_short = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_noise = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_lp_rawdata = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_lp_noise = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_gap_ver = HX_RAWDATA_NOT_READY;
	g_sec_raw_buff->f_ready_gap_hor = HX_RAWDATA_NOT_READY;

	kfree(g_sec_raw_buff);
	input_err(true, &data->client->dev, "%s %s: End!\n", HIMAX_LOG_TAG,
			__func__);
}

EXPORT_SYMBOL(sec_touch_sysfs_remove);
#endif /* SEC_FACTORY_MODE */
