/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung SoC DisplayPort HDCP1.3 driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include "displayport.h"

HDCP13 HDCP13_DPCD;
struct hdcp13_info hdcp13_info;

void HDCP13_DPCD_BUFFER(void)
{
	u8 i = 0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_BKSV); i++)
		HDCP13_DPCD.HDCP13_BKSV[i] = 0x0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_R0); i++)
		HDCP13_DPCD.HDCP13_R0[i] = 0x0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_AKSV); i++)
		HDCP13_DPCD.HDCP13_AKSV[i] = 0x0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_AN); i++)
		HDCP13_DPCD.HDCP13_AN[i] = 0x0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_V_H0); i++)
		HDCP13_DPCD.HDCP13_V_H0[i] = 0x0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_V_H1); i++)
		HDCP13_DPCD.HDCP13_V_H1[i] = 0x0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_V_H2); i++)
		HDCP13_DPCD.HDCP13_V_H2[i] = 0x0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_V_H3); i++)
		HDCP13_DPCD.HDCP13_V_H3[i] = 0x0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_V_H4); i++)
		HDCP13_DPCD.HDCP13_V_H4[i] = 0x0;

	HDCP13_DPCD.HDCP13_BCAP[0] = 0x0;
	HDCP13_DPCD.HDCP13_BSTATUS[0] = 0x0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_BINFO); i++)
		HDCP13_DPCD.HDCP13_BINFO[i] = 0x0;

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_KSV_FIFO); i++)
		HDCP13_DPCD.HDCP13_KSV_FIFO[i] = 0x0;

	HDCP13_DPCD.HDCP13_AINFO[0] = 0x0;
}

void HDCP13_dump(char *str, u8 *buf, int size)
{
	int i;
	u8 *buffer = buf;

	displayport_dbg("[HDCP 1.3] %s = 0x", str);

	for (i = 0; i < size; i++)
		displayport_dbg("%02x", *(buffer+i));

	displayport_dbg("\n");
}

void HDCP13_AUTH_Select(void)
{
	hdcp13_info.is_repeater = 0;
	displayport_write_mask(HDCP_Control_Register_0, hdcp13_info.is_repeater, HW_AUTH_EN);

	if (hdcp13_info.is_repeater)
		displayport_dbg("[HDCP 1.3] HW Authentication Select\n");
	else
		displayport_dbg("[HDCP 1.3] SW Authentication Select\n");
}

void HDCP13_Func_En(u32 en)
{
	u32 val = en ? 0 : ~0; /* 0 is enable */

	displayport_write_mask(Function_En_1, val, HDCP_FUNC_EN_N);
}

u8 HDCP13_Read_Bcap(void)
{
	u8 return_val = 0;
	u8 hdcp_capa = 0;

	displayport_reg_dpcd_read(ADDR_HDCP13_BCAP, 1, HDCP13_DPCD.HDCP13_BCAP);

	hdcp13_info.is_repeater = (HDCP13_DPCD.HDCP13_BCAP[0] & 0x02) >> 1;

	hdcp_capa = HDCP13_DPCD.HDCP13_BCAP[0] & 0x01;

	if (hdcp_capa)
		return_val = 0;
	else
		return_val = -EINVAL;

	return return_val;
}

void HDCP13_Repeater_Set(void)
{
	displayport_write_mask(HDCP_Control_Register_0, hdcp13_info.is_repeater, SW_RX_REPEATER);
}

u8 HDCP13_Read_Bksv(void)
{
	u8 i = 0;
	u8 j = 0;
	u8 offset = 0;
	int one = 0;
	u8 ret;

	displayport_reg_dpcd_read_burst(ADDR_HDCP13_BKSV, sizeof(HDCP13_DPCD.HDCP13_BKSV), HDCP13_DPCD.HDCP13_BKSV);

	HDCP13_dump("BKSV", &(HDCP13_DPCD.HDCP13_BKSV[0]), 5);

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_BKSV); i++) {
		for (j = 0; j < 8; j++) {
			if (HDCP13_DPCD.HDCP13_BKSV[i] & (0x1 << j))
				one++;
		}
	}

	if (one == 20) {
		for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_BKSV); i++) {
			displayport_write(HDCP_BKSV_Register_0 + offset, (u32)HDCP13_DPCD.HDCP13_BKSV[i]);
			offset += 4;
		}

		displayport_dbg("[HDCP 1.3] Valid Bksv\n");
		ret = 0;
	} else {
		displayport_dbg("[HDCP 1.3] Invalid Bksv\n");
		ret = -EINVAL;
	}

	return ret;
}

void HDCP13_Set_An_val(void)
{
	displayport_write_mask(HDCP_Control_Register_0, 1, SW_STORE_AN);

	displayport_write_mask(HDCP_Control_Register_0, 0, SW_STORE_AN);
}

void HDCP13_Write_An_val(void)
{
	u8 i = 0;
	u8 offset = 0;

	HDCP13_Set_An_val();

	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_AN); i++) {
		HDCP13_DPCD.HDCP13_AN[i] = (u8)displayport_read(HDCP_AN_Register_0 + offset);
		offset += 4;
	}

	displayport_reg_dpcd_write_burst(ADDR_HDCP13_AN, 8, HDCP13_DPCD.HDCP13_AN);
}

u8 HDCP13_Write_Aksv(void)
{
	u8 i = 0;
	u8 offset = 0;
	u8 ret;

	if (displayport_read_mask(HDCP_Status_Register, AKSV_VALID)) {
		for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_AKSV); i++) {
			HDCP13_DPCD.HDCP13_AKSV[i] = (u8)displayport_read(HDCP_AKSV_Register_0 + offset);
			offset += 4;
		}

		displayport_reg_dpcd_write_burst(ADDR_HDCP13_AKSV, 5, HDCP13_DPCD.HDCP13_AKSV);

		displayport_dbg("[HDCP 1.3] Valid Aksv\n");

		ret = 0;
	} else {
		displayport_dbg("[HDCP 1.3] Invalid Aksv\n");
		ret = -EINVAL;
	}

	return ret;
}

u8 HDCP13_CMP_Ri(void)
{
	u8 i = 0;
	u8 offset = 0;
	u8 cnt = 0;
	u8 ri[2];

	/* BKSV Rewrite */
	for (i = 0; i < sizeof(HDCP13_DPCD.HDCP13_BKSV); i++) {
		displayport_write(HDCP_BKSV_Register_0 + offset, (u32)HDCP13_DPCD.HDCP13_BKSV[i]);
		offset += 4;
	}

	/* R0 Sink Available check */
	displayport_reg_dpcd_read(ADDR_HDCP13_BSTATUS, 1, HDCP13_DPCD.HDCP13_BSTATUS);

	while ((HDCP13_DPCD.HDCP13_BSTATUS[0] & BSTATUS_R0_AVAILABLE) == 0) {
		mdelay(RI_AVAILABLE_WAITING);

		cnt++;

		displayport_reg_dpcd_read(ADDR_HDCP13_BSTATUS, 1, HDCP13_DPCD.HDCP13_BSTATUS);

		if (cnt == 10) {
			displayport_dbg("[HDCP 1.3] R0 read not read in RX part\n");
			return -EFAULT;
		}
	}

	/* Read R0 from Sink */
	displayport_reg_dpcd_read_burst(ADDR_HDCP13_R0, sizeof(HDCP13_DPCD.HDCP13_R0), HDCP13_DPCD.HDCP13_R0);

	/* R0 Source Available check */
	cnt = 0;
	while (hdcp13_info.r0_read_flag != 1) {
		mdelay(RI_AVAILABLE_WAITING);
		cnt++;
		if (cnt == 10) {
			displayport_dbg("[HDCP 1.3] R0 read not read in TX part\n");
			return -EFAULT;
		}
	}

	hdcp13_info.r0_read_flag = 0;

	/* Read R0 from Source */
	ri[0] = (u8)displayport_read(HDCP_R0_Register_0);
	ri[1] = (u8)displayport_read(HDCP_R0_Register_1);

	if ((ri[0] == HDCP13_DPCD.HDCP13_R0[0]) && (ri[1] == HDCP13_DPCD.HDCP13_R0[1])) {
		displayport_dbg("[HDCP 1.3] Ri_Tx(0x%02x%02x) == Ri_Rx(0x%02x%02x)\n",
			ri[1], ri[0], HDCP13_DPCD.HDCP13_R0[1], HDCP13_DPCD.HDCP13_R0[0]);

		return 0;
	} else {
		displayport_dbg("[HDCP 1.3] Ri_Tx(0x%02x%02x) != Ri_Rx(0x%02x%02x)\n",
			ri[1], ri[0], HDCP13_DPCD.HDCP13_R0[1], HDCP13_DPCD.HDCP13_R0[0]);

		return -EFAULT;
	}
}

void HDCP13_Encryption_con(u8 enable)
{
	if (enable == 1) {
		displayport_write_mask(HDCP_Control_Register_0, ~(hdcp13_info.is_repeater), SW_AUTH_OK);
		displayport_write_mask(HDCP_Control_Register_0, 1, HDCP_ENC_EN);

		displayport_dbg("[HDCP 1.3] HDCP13 Encryption Enable\n");
	} else {
		displayport_write_mask(HDCP_Control_Register_0, 0, SW_AUTH_OK | HDCP_ENC_EN);

		displayport_dbg("[HDCP 1.3] HDCP13 Encryption Disable\n");
	}
}

void HDCP13_HW_ReAuth(void)
{
	displayport_write_mask(HDCP_Control_Register_0, 1, HW_RE_AUTHEN);

	displayport_write_mask(HDCP_Control_Register_0, 0, HW_RE_AUTHEN);
}

void HDCP13_Link_integrality_check(void)
{
	u8 dpcd_read_data[0];

	displayport_dbg("HDCP13_Link_integrality_check\n");

	if (hdcp13_info.link_check == LINK_CHECK_NEED) {
		displayport_reg_dpcd_read(DPCD_ADD_DEVICE_SERVICE_IRQ_VECTOR, 1, dpcd_read_data);

		displayport_reg_dpcd_read(ADDR_HDCP13_BSTATUS, 1, HDCP13_DPCD.HDCP13_BSTATUS);

		displayport_dbg("Reauth. req = %s\n",
			(HDCP13_DPCD.HDCP13_BSTATUS[0] & BSTATUS_REAUTH_REQ) == 1 ? "On" : "Off");
		displayport_dbg("Link Intergrity check Fail = %s\n",
			(HDCP13_DPCD.HDCP13_BSTATUS[0] & BSTATUS_LINK_INTEGRITY_FAIL) == 1 ? "On" : "Off");

		if (!(dpcd_read_data[0] & CP_IRQ)) {
			hdcp13_info.link_check = LINK_CHECK_PASS;
			displayport_dbg("HDCP1.3 Link check Pass\n");
			return;
		}

		if (!(HDCP13_DPCD.HDCP13_BSTATUS[0] & BSTATUS_LINK_INTEGRITY_FAIL)) {
			hdcp13_info.link_check = LINK_CHECK_PASS;
			displayport_dbg("HDCP1.3 Link check Pass\n");
			return;
		}

		HDCP13_Encryption_con(0);
		hdcp13_info.link_check = LINK_CHECK_FAIL;
	}

	if ((hdcp13_info.link_check == LINK_CHECK_FAIL) && (hdcp13_info.is_repeater)) {
		displayport_dbg("HDCP 1.3 HW ReAuth\n");
		HDCP13_HW_ReAuth();
	}
}

void HDCP13_HW_Revocation_set(u8 param)
{
	displayport_write_mask(HDCP_Debug_Control_Register, param, REVOCATION_CHK_DONE);
}

void HDCP13_HW_AUTH_set(u8 Auth_type)
{
	if (Auth_type == FIRST_AUTH)
		displayport_write_mask(HDCP_Control_Register_0, 1, HW_1ST_PART_ATHENTICATION_EN);
	else
		displayport_write_mask(HDCP_Control_Register_0, 1, HW_2ND_PART_ATHENTICATION_EN);
}

u8 HDCP13_HW_KSV_read(void)
{
	u32 KSV_bytes = 0;
	u32 read_count = 0;
	u32 i = 0;
	u8 ret;

	displayport_reg_dpcd_read_burst(ADDR_HDCP13_BINFO, sizeof(HDCP13_DPCD.HDCP13_BINFO), HDCP13_DPCD.HDCP13_BINFO);

	hdcp13_info.device_cnt = HDCP13_DPCD.HDCP13_BINFO[1] && 0x7F;

	KSV_bytes = hdcp13_info.device_cnt;

	displayport_dbg("[HDCP 1.3] Total KSV bytes = %d", KSV_bytes);

	read_count = (hdcp13_info.device_cnt - 1) / 3 + 1;

	for (i = 0; i < read_count; i++) {
		if (displayport_reg_dpcd_read_burst(ADDR_HDCP13_KSV_FIFO,
			sizeof(HDCP13_DPCD.HDCP13_KSV_FIFO), HDCP13_DPCD.HDCP13_KSV_FIFO) != 0) {
			ret =  -EFAULT;
			break;
		}

		HDCP13_dump("KSV List read", HDCP13_DPCD.HDCP13_KSV_FIFO, sizeof(HDCP13_DPCD.HDCP13_KSV_FIFO));
	}

	return ret;
}

u8 HDCP13_HW_KSV_check(void)
{
	if (displayport_read_mask(HDCP_Debug_Control_Register, CHECK_KSV))
		return 0;
	else
		return -EFAULT;
}

u8 HDCP13_HW_1st_Auth_check(void)
{
	if (displayport_read_mask(HDCP_Status_Register, HW_1ST_AUTHEN_PASS))
		return 0;
	else
		return -EFAULT;
}

u8 HDCP13_HW_Auth_pass_check(void)
{
	if (displayport_read_mask(HDCP_Status_Register, HW_AUTHEN_PASS)) {
		displayport_dbg("[HDCP 1.3] HDCP13 HW Authectication PASS in no revocation mode\n");
		return 0;
	} else
		return -EFAULT;
}

u8 HDCP13_HW_Auth_check(void)
{
	if (displayport_read_mask(HDCP_Status_Register, AUTH_FAIL)) {
		displayport_dbg("[HDCP 1.3] HDCP13 HW Authectication FAIL\n");
		return 0;
	} else
		return -EFAULT;
}

void HDCP3_IRQ_Mask(void)
{
	displayport_reg_set_interrupt_mask(HDCP_LINK_CHECK_INT_MASK, 1);
	displayport_reg_set_interrupt_mask(HDCP_LINK_FAIL_INT_MASK, 1);
}

void HDCP13_run(void)
{
	int retry_cnt = HDCP_RETRY_COUNT;
	u8 HW_AUTH_Fail = 0;

	while ((hdcp13_info.auth_state != HDCP13_STATE_AUTHENTICATED) && (retry_cnt != 0)) {
		retry_cnt--;

		HDCP13_DPCD_BUFFER();

		hdcp13_info.auth_state = HDCP13_STATE_NOT_AUTHENTICATED;
		hdcp13_info.is_repeater = 0;

		HDCP13_Encryption_con(0);
		HDCP13_Func_En(1);

		if (HDCP13_Read_Bcap() != 0)
			displayport_dbg("[HDCP 1.3] NONE HDCP CAPABLE\n");

		/* HDCP13_Repeater_Set(); */
		HDCP13_AUTH_Select();

		if (!hdcp13_info.is_repeater) {
			displayport_dbg("[HDCP 1.3] SW Auth.\n");
			displayport_reg_set_interrupt_mask(HDCP_R0_READY_INT_MASK, 1);

			if (HDCP13_Read_Bksv() != 0) {
				displayport_dbg("[HDCP 1.3] ReAuthentication Start!!!\n");
				continue;
			}

			HDCP13_Write_An_val();

			if (HDCP13_Write_Aksv() != 0) {
				displayport_dbg("[HDCP 1.3] ReAuthentication Start!!!\n");
				continue;
			}

			mdelay(RI_DELAY);

			if (HDCP13_CMP_Ri() != 0)
				continue;

			hdcp13_info.auth_state = HDCP13_STATE_AUTHENTICATED;
		} else {
			if (hdcp13_info.revocation_check) {
				displayport_dbg("[HDCP 1.3] HW Auth. Revocation check mode\n");

				displayport_reg_set_interrupt_mask(HW_AUTH_CHG_INT_MASK, 1);
				displayport_reg_set_interrupt_mask(HW_HDCP_DONE_INT_MASK, 1);

				HDCP13_HW_AUTH_set(FIRST_AUTH);
				HW_AUTH_Fail = 0;

				while (!HDCP13_HW_1st_Auth_check()) {
					displayport_dbg("[HDCP 1.3] Waiting First Part Auth Done...\n");

					mdelay(10);

					if (HDCP13_HW_Auth_check() != 0) {
						HW_AUTH_Fail = 1;
						break;
					}
				}

				if (HW_AUTH_Fail == 1) {
					HDCP13_Encryption_con(0);
					HDCP13_HW_ReAuth();
					continue;
				}

				HDCP13_Encryption_con(1);

				while (hdcp13_info.auth_state != HDCP13_STATE_SECOND_AUTH_DONE) {
					mdelay(1000);
					displayport_dbg("[HDCP 1.3] Waiting Second Part Auth Done...\n");
				}

				HW_AUTH_Fail = 0;

				while (HDCP13_HW_KSV_check() != 0) {
					if (HDCP13_HW_Auth_check() != 0) {
						HW_AUTH_Fail = 1;
						break;
					}
				}

				if (HW_AUTH_Fail == 1) {
					HDCP13_Encryption_con(0);
					HDCP13_HW_ReAuth();
					continue;
				}

				if (HDCP13_HW_KSV_read() == 0) {
					HDCP13_HW_AUTH_set(SECOND_AUTH);
					HDCP13_HW_Revocation_set(1);
				} else {
					HW_AUTH_Fail = 1;
					HDCP13_HW_Revocation_set(1);
				}

				if (HW_AUTH_Fail == 1) {
					HDCP13_Encryption_con(0);
					HDCP13_HW_ReAuth();
					continue;
				}

				hdcp13_info.auth_state = HDCP13_STATE_AUTHENTICATED;
			} else {
				displayport_dbg("[HDCP 1.3] HW Auth. Revocatin uncheck mode\n");

				displayport_reg_set_interrupt_mask(HW_AUTH_CHG_INT_MASK, 1);
				displayport_reg_set_interrupt_mask(HW_HDCP_DONE_INT_MASK, 1);

				HDCP13_HW_AUTH_set(FIRST_AUTH);
				HDCP13_HW_AUTH_set(SECOND_AUTH);
				HW_AUTH_Fail = 0;

				while (HDCP13_HW_1st_Auth_check() != 0) {
					displayport_dbg("[HDCP 1.3] Waiting First Part Auth Done...\n");
					mdelay(10);

					if (HDCP13_HW_Auth_check() != 0) {
						HW_AUTH_Fail = 1;
						break;
					}
				}

				if (HW_AUTH_Fail == 1) {
					HDCP13_Encryption_con(0);
					HDCP13_HW_ReAuth();
					continue;
				}

				HDCP13_Encryption_con(1);

				while (hdcp13_info.auth_state != HDCP13_STATE_SECOND_AUTH_DONE)

				while (HDCP13_HW_Auth_pass_check() != 0) {
					if (HDCP13_HW_Auth_check() != 0) {
						HW_AUTH_Fail = 1;
						break;
					}
				}

				if (HW_AUTH_Fail == 1) {
					HDCP13_Encryption_con(0);
					HDCP13_HW_ReAuth();
					continue;
				}

				hdcp13_info.auth_state = HDCP13_STATE_AUTHENTICATED;
			}
		}
	}

	if (hdcp13_info.auth_state == HDCP13_STATE_AUTHENTICATED)
		HDCP13_Encryption_con(1);
	else
		displayport_set_blue_screen_for_hdcp_fail();

	HDCP3_IRQ_Mask();
}
