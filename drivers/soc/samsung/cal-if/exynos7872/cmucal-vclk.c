#include "../cmucal.h"
#include "cmucal-node.h"
#include "cmucal-vclk.h"

#include "cmucal-vclklut.h"

/*=================CMUCAL version: S5E7872================================*/

/*=================CLK in each VCLK================================*/


/* DVFS List */
enum clk_id cmucal_vclk_vdd_cpucl0[] = {
	DIV_CLK_CPUCL0_ACLK,
	PLL_CPUCL0,
};
enum clk_id cmucal_vclk_vdd_cpucl1[] = {
	DIV_CLK_CPUCL1_ACLK,
	PLL_CPUCL1,
};
enum clk_id cmucal_vclk_vdd_g3d[] = {
	PLL_G3D,
};
enum clk_id cmucal_vclk_vdd_int[] = {
	DIV_CLK_IS_APB,
	PLL_MEM,
};
enum clk_id cmucal_vclk_dfs_abox[] = {
	DIV_CLK_AUD_BUS,
	DIV_CLK_AUD_PLL,
};

/* SPECIAL List */
enum clk_id cmucal_vclk_spl_clk_fsys_mmc_card_blk_cmu[] = {
	CLKCMU_FSYS_MMC_CARD,
	MUX_CLKCMU_FSYS_MMC_CARD,
};
enum clk_id cmucal_vclk_mux_clkcmu_is_isp_user_blk_cmu[] = {
	CLKCMU_IS_ISP,
	MUX_CLKCMU_IS_ISP,
};
enum clk_id cmucal_vclk_spl_clk_peri_spi_1_blk_cmu[] = {
	CLKCMU_PERI_SPI_1,
	MUX_CLKCMU_PERI_SPI_1,
};
enum clk_id cmucal_vclk_spl_clk_peri_uart_2_blk_cmu[] = {
	CLKCMU_PERI_UART_2,
	MUX_CLKCMU_PERI_UART_2,
};
enum clk_id cmucal_vclk_spl_clk_peri_usi2_blk_cmu[] = {
	CLKCMU_PERI_USI2,
	MUX_CLKCMU_PERI_USI2,
};
enum clk_id cmucal_vclk_clkcmu_cis_clk0_blk_cmu[] = {
	CLKCMU_CIS_CLK0,
	MUX_CLKCMU_CIS_CLK0,
};
enum clk_id cmucal_vclk_clkcmu_cis_clk2_blk_cmu[] = {
	CLKCMU_CIS_CLK2,
	MUX_CLKCMU_CIS_CLK2,
};
enum clk_id cmucal_vclk_spl_clk_fsys_mmc_embd_blk_cmu[] = {
	CLKCMU_FSYS_MMC_EMBD,
	MUX_CLKCMU_FSYS_MMC_EMBD,
};
enum clk_id cmucal_vclk_spl_clk_fsys_mmc_sdio_blk_cmu[] = {
	CLKCMU_FSYS_MMC_SDIO,
	MUX_CLKCMU_FSYS_MMC_SDIO,
};
enum clk_id cmucal_vclk_spl_clk_is_tpu_blk_cmu[] = {
	CLKCMU_IS_TPU,
	MUX_CLKCMU_IS_TPU,
};
enum clk_id cmucal_vclk_spl_clk_peri_spi_0_blk_cmu[] = {
	CLKCMU_PERI_SPI_0,
	MUX_CLKCMU_PERI_SPI_0,
};
enum clk_id cmucal_vclk_spl_clk_peri_uart_0_blk_cmu[] = {
	CLKCMU_PERI_UART_0,
	MUX_CLKCMU_PERI_UART_0,
};
enum clk_id cmucal_vclk_spl_clk_peri_usi1_blk_cmu[] = {
	CLKCMU_PERI_USI1,
	MUX_CLKCMU_PERI_USI1,
};
enum clk_id cmucal_vclk_occ_mif_cmuref_blk_cmu[] = {
	CLKCMU_MIF_BUS,
	MUX_CLKCMU_MIF_BUS,
};
enum clk_id cmucal_vclk_occ_cmu_cmuref_blk_cmu[] = {
	MUX_CMU_CMUREF,
	DIV_CLK_CMU_CMUREF,
	MUX_CLK_CMU_CMUREF,
};
enum clk_id cmucal_vclk_clkcmu_cis_clk1_blk_cmu[] = {
	CLKCMU_CIS_CLK1,
	MUX_CLKCMU_CIS_CLK1,
};
enum clk_id cmucal_vclk_spl_clk_peri_usi0_blk_cmu[] = {
	CLKCMU_PERI_USI0,
	MUX_CLKCMU_PERI_USI0,
};
enum clk_id cmucal_vclk_spl_clk_peri_uart_1_blk_cmu[] = {
	CLKCMU_PERI_UART_1,
	MUX_CLKCMU_PERI_UART_1,
};
enum clk_id cmucal_vclk_spl_clk_is_3aa_half_blk_cmu[] = {
	CLKCMU_IS_3AA,
	MUX_CLKCMU_IS_3AA,
};
enum clk_id cmucal_vclk_spl_clk_cpucl0_cntclk_blk_cpucl0[] = {
	DIV_CLK_CPUCL0_CNTCLK,
};
enum clk_id cmucal_vclk_spl_clk_cpucl0_atclk_blk_cpucl0[] = {
	DIV_CLK_CPUCL0_ATCLK,
};
enum clk_id cmucal_vclk_spl_clk_cpucl0_cmuref_blk_cpucl0[] = {
	DIV_CLK_CPUCL0_CMUREF,
};
enum clk_id cmucal_vclk_spl_clk_cpucl1_cntclk_blk_cpucl1[] = {
	DIV_CLK_CPUCL1_CNTCLK,
};
enum clk_id cmucal_vclk_div_clk_cpucl1_cmuref_blk_cpucl1[] = {
	DIV_CLK_CPUCL1_CMUREF,
};
enum clk_id cmucal_vclk_spl_clk_cpucl1_atclk_blk_cpucl1[] = {
	DIV_CLK_CPUCL1_ATCLK,
};
enum clk_id cmucal_vclk_spl_clk_aud_uaif2_blk_dispaud[] = {
	MUX_CLK_AUD_UAIF2,
	DIV_CLK_AUD_UAIF2,
};
enum clk_id cmucal_vclk_spl_clk_aud_cpu_aclk_blk_dispaud[] = {
	DIV_CLK_AUD_CPU_ACLK,
};
enum clk_id cmucal_vclk_spl_clk_aud_uaif0_blk_dispaud[] = {
	MUX_CLK_AUD_UAIF0,
	DIV_CLK_AUD_UAIF0,
};
enum clk_id cmucal_vclk_spl_clk_aud_uaif3_blk_dispaud[] = {
	MUX_CLK_AUD_UAIF3,
	DIV_CLK_AUD_UAIF3,
};
enum clk_id cmucal_vclk_spl_clk_aud_cpu_pclkdbg_blk_dispaud[] = {
	DIV_CLK_AUD_CPU_PCLKDBG,
};
enum clk_id cmucal_vclk_clk_aud_fm_blk_dispaud[] = {
	DIV_CLK_AUD_FM,
};
enum clk_id cmucal_vclk_spl_clk_is_3aa_half_blk_is[] = {
	DIV_CLK_IS_3AA_HALF,
};
enum clk_id cmucal_vclk_occ_mif_cmuref_blk_mif[] = {
	MUX_MIF_CMUREF,
};

/* COMMON List */
enum clk_id cmucal_vclk_blk_apm[] = {
	DIV_CLK_APM_I2C,
};
enum clk_id cmucal_vclk_blk_cmu[] = {
	CLKCMU_G3D_SWITCH,
	CLKCMU_IS_VRA,
	CLKCMU_DISPAUD_BUS,
	CLKCMU_FSYS_BUS,
	CLKCMU_MFCMSCL_MSCL,
	CLKCMU_MFCMSCL_MFC,
	AP2CP_SHARED0_PLL_CLK,
	CLKCMU_PERI_BUS,
	CLKCMU_APM_BUS,
	AP2CP_SHARED1_PLL_CLK,
	CLKCMU_MIF_CCI,
	PLL_SHARED0_DIV3,
	CLKCMU_CPUCL0_SECJTAG,
	MUX_CLKCMU_MFCMSCL_MSCL,
	MUX_CLKCMU_MFCMSCL_MFC,
	MUX_CLKCMU_IS_VRA,
	MUX_CLKCMU_DISPAUD_BUS,
	MUX_CLKCMU_FSYS_BUS,
	MUX_CLKCMU_PERI_BUS,
	MUX_CLKCMU_APM_BUS,
	MUX_CLKCMU_MIF_CCI,
	PLL_SHARED1_DIV2,
	PLL_SHARED0_DIV2,
	PLL_SHARED1,
	PLL_SHARED0,
	PLL_SHARED0_DIV4,
	PLL_SHARED1_DIV4,
};
enum clk_id cmucal_vclk_blk_cpucl0[] = {
	DIV_CLK_CPUCL0_PCLK,
	DIV_CLK_CPUCL0_PCLKDBG,
};
enum clk_id cmucal_vclk_blk_cpucl1[] = {
	DIV_CLK_CPUCL1_PCLK,
	DIV_CLK_CPUCL1_PCLKDBG,
};
enum clk_id cmucal_vclk_blk_dispaud[] = {
	DIV_CLK_DISPAUD_BUSP,
	DIV_CLK_AUD_AUDIF,
	PLL_AUD,
	MUX_CLK_AUD_CPU_HCH,
};
enum clk_id cmucal_vclk_blk_g3d[] = {
	DIV_CLK_G3D_BUSP,
	MUX_CLK_G3D_BUSD,
};
enum clk_id cmucal_vclk_blk_mfcmscl[] = {
	DIV_CLK_MFCMSCL_APB,
};
enum clk_id cmucal_vclk_blk_mif[] = {
	MUX_CLK_MIF_GIC,
	DIV_CLK_MIF_CCI,
	MUX_CLK_MIF_CCI,
	DIV_CLK_MIF_BUSP,
	DIV_CLK_MIF_BUS,
	MUX_CLK_MIF_BUS,
};

/* GATING List */
enum clk_id cmucal_vclk_apbif_gpio_alive[] = {
	GOUT_BLK_APM_UID_APBIF_GPIO_ALIVE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_apbif_pmu_alive[] = {
	GOUT_BLK_APM_UID_APBIF_PMU_ALIVE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_apm[] = {
	GOUT_BLK_APM_UID_APM_IPCLKPORT_ACLK_CPU,
	GOUT_BLK_APM_UID_APM_IPCLKPORT_ACLK_IntMEM,
	GOUT_BLK_APM_UID_APM_IPCLKPORT_ACLK_SYS,
};
enum clk_id cmucal_vclk_apm_cmu_apm[] = {
	CLK_BLK_APM_UID_APM_CMU_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_asyncapb_apm[] = {
	GOUT_BLK_APM_UID_ASYNCAPB_APM_IPCLKPORT_PCLKM,
	GOUT_BLK_APM_UID_ASYNCAPB_APM_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_i2c_apm[] = {
	GOUT_BLK_APM_UID_I2C_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ip_batcher_ap[] = {
	GOUT_BLK_APM_UID_IP_BATCHER_AP_IPCLKPORT_i_pclk,
};
enum clk_id cmucal_vclk_ip_batcher_cp[] = {
	GOUT_BLK_APM_UID_IP_BATCHER_CP_IPCLKPORT_i_pclk,
};
enum clk_id cmucal_vclk_lhm_axi_p_alive[] = {
	GOUT_BLK_APM_UID_LHM_AXI_P_ALIVE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_d_alive[] = {
	GOUT_BLK_APM_UID_LHS_AXI_D_ALIVE_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_mailbox_apm2ap[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM2AP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mailbox_apm2cp[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM2CP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mailbox_apm2gnss[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM2GNSS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mailbox_apm2wlbt[] = {
	GOUT_BLK_APM_UID_MAILBOX_APM2WLBT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mp_apbsema_hwacg_2ch[] = {
	GOUT_BLK_APM_UID_MP_APBSEMA_HWACG_2CH_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_speedy[] = {
	GOUT_BLK_APM_UID_SPEEDY_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_sysreg_apm[] = {
	GOUT_BLK_APM_UID_SYSREG_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_wdt_apm[] = {
	GOUT_BLK_APM_UID_WDT_APM_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_adm_apb_p_cssys_dbg[] = {
	GOUT_BLK_CPUCL0_UID_ADM_APB_P_CSSYS_DBG_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ads_ahb_g_sss[] = {
	GOUT_BLK_CPUCL0_UID_ADS_AHB_G_SSS_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_ads_apb_g_cssys_dbg[] = {
	GOUT_BLK_CPUCL0_UID_ADS_APB_G_CSSYS_DBG_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_p_dump_pc_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_AD_APB_P_DUMP_PC_CPUCL0_IPCLKPORT_PCLKM,
	GOUT_BLK_CPUCL0_UID_AD_APB_P_DUMP_PC_CPUCL0_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_p_secjtag_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_AD_APB_P_SECJTAG_CPUCL0_IPCLKPORT_PCLKM,
	GOUT_BLK_CPUCL0_UID_AD_APB_P_SECJTAG_CPUCL0_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_axi2apb_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_AXI2APB_CPUCL0_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_cpucl0_cmu_cpucl0[] = {
	CLK_BLK_CPUCL0_UID_CPUCL0_CMU_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_cssys_dbg[] = {
	GOUT_BLK_CPUCL0_UID_CSSYS_DBG_IPCLKPORT_PCLKDBG,
};
enum clk_id cmucal_vclk_dump_pc_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_DUMP_PC_CPUCL0_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_lhm_axi_p_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_LHM_AXI_P_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_ace_d_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_LHS_ACE_D_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_t_cssys_dbg[] = {
	GOUT_BLK_CPUCL0_UID_LHS_AXI_T_CSSYS_DBG_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_secjtag[] = {
	GOUT_BLK_CPUCL0_UID_SECJTAG_IPCLKPORT_i_clk,
};
enum clk_id cmucal_vclk_sysreg_cpucl0[] = {
	GOUT_BLK_CPUCL0_UID_SYSREG_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_adm_apb_g_cpucl1[] = {
	GOUT_BLK_CPUCL1_UID_ADM_APB_G_CPUCL1_IPCLKPORT_PCLKM,
};
enum clk_id cmucal_vclk_ad_apb_p_dump_pc_cpucl1[] = {
	GOUT_BLK_CPUCL1_UID_AD_APB_P_DUMP_PC_CPUCL1_IPCLKPORT_PCLKM,
	GOUT_BLK_CPUCL1_UID_AD_APB_P_DUMP_PC_CPUCL1_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_axi2apb_cpucl1[] = {
	GOUT_BLK_CPUCL1_UID_AXI2APB_CPUCL1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_cpucl1_cmu_cpucl1[] = {
	CLK_BLK_CPUCL1_UID_CPUCL1_CMU_CPUCL1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_dump_pc_cpucl1[] = {
	GOUT_BLK_CPUCL1_UID_DUMP_PC_CPUCL1_IPCLKPORT_I_PCLK,
};
enum clk_id cmucal_vclk_lhm_axi_p_cpucl1[] = {
	GOUT_BLK_CPUCL1_UID_LHM_AXI_P_CPUCL1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_ace_d_cpucl1[] = {
	GOUT_BLK_CPUCL1_UID_LHS_ACE_D_CPUCL1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_sysreg_cpucl1[] = {
	GOUT_BLK_CPUCL1_UID_SYSREG_CPUCL1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_abox[] = {
	GOUT_BLK_DISPAUD_UID_ABOX_IPCLKPORT_ACLK,
	GOUT_BLK_DISPAUD_UID_ABOX_IPCLKPORT_BCLK_SPDY,
	GOUT_BLK_DISPAUD_UID_ABOX_IPCLKPORT_BCLK_UAIF0,
	GOUT_BLK_DISPAUD_UID_ABOX_IPCLKPORT_BCLK_UAIF2,
	GOUT_BLK_DISPAUD_UID_ABOX_IPCLKPORT_BCLK_UAIF3,
	GOUT_BLK_DISPAUD_UID_ABOX_IPCLKPORT_CCLK_ASB,
	GOUT_BLK_DISPAUD_UID_ABOX_IPCLKPORT_CCLK_CA7,
};
enum clk_id cmucal_vclk_abox_dap[] = {
	GOUT_BLK_DISPAUD_UID_ABOX_DAP_IPCLKPORT_dapclk,
};
enum clk_id cmucal_vclk_ad_apb_decon0[] = {
	GOUT_BLK_DISPAUD_UID_AD_APB_DECON0_IPCLKPORT_PCLKM,
	GOUT_BLK_DISPAUD_UID_AD_APB_DECON0_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_decon0_secure[] = {
	GOUT_BLK_DISPAUD_UID_AD_APB_DECON0_SECURE_IPCLKPORT_PCLKM,
	GOUT_BLK_DISPAUD_UID_AD_APB_DECON0_SECURE_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_dpp[] = {
	GOUT_BLK_DISPAUD_UID_AD_APB_DPP_IPCLKPORT_PCLKM,
	GOUT_BLK_DISPAUD_UID_AD_APB_DPP_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_dpu_dma[] = {
	GOUT_BLK_DISPAUD_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM,
	GOUT_BLK_DISPAUD_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_dpu_dma_secure[] = {
	GOUT_BLK_DISPAUD_UID_AD_APB_DPU_DMA_SECURE_IPCLKPORT_PCLKM,
	GOUT_BLK_DISPAUD_UID_AD_APB_DPU_DMA_SECURE_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_dsim0[] = {
	GOUT_BLK_DISPAUD_UID_AD_APB_DSIM0_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_smmu_dpu[] = {
	GOUT_BLK_DISPAUD_UID_AD_APB_SMMU_DPU_IPCLKPORT_PCLKM,
	GOUT_BLK_DISPAUD_UID_AD_APB_SMMU_DPU_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_smmu_dpu_secure[] = {
	GOUT_BLK_DISPAUD_UID_AD_APB_SMMU_DPU_SECURE_IPCLKPORT_PCLKM,
	GOUT_BLK_DISPAUD_UID_AD_APB_SMMU_DPU_SECURE_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_axi2apb_dispaud[] = {
	GOUT_BLK_DISPAUD_UID_AXI2APB_DISPAUD_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_axi_us_32to128[] = {
	GOUT_BLK_DISPAUD_UID_AXI_US_32to128_IPCLKPORT_aclk,
};
enum clk_id cmucal_vclk_dftmux_dispaud[] = {
	GOUT_BLK_DISPAUD_UID_DFTMUX_DISPAUD_IPCLKPORT_AUD_CODEC_MCLK,
};
enum clk_id cmucal_vclk_dispaud_cmu_dispaud[] = {
	CLK_BLK_DISPAUD_UID_DISPAUD_CMU_DISPAUD_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_dpu[] = {
	GOUT_BLK_DISPAUD_UID_DPU_IPCLKPORT_ACLK_DECON0,
	GOUT_BLK_DISPAUD_UID_DPU_IPCLKPORT_ACLK_DMA,
	GOUT_BLK_DISPAUD_UID_DPU_IPCLKPORT_ACLK_DPP,
};
enum clk_id cmucal_vclk_gpio_dispaud[] = {
	GOUT_BLK_DISPAUD_UID_GPIO_DISPAUD_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_lhm_axi_p_dispaud[] = {
	GOUT_BLK_DISPAUD_UID_LHM_AXI_P_DISPAUD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_d_abox[] = {
	GOUT_BLK_DISPAUD_UID_LHS_AXI_D_ABOX_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_d_dpu[] = {
	GOUT_BLK_DISPAUD_UID_LHS_AXI_D_DPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_peri_axi_asb[] = {
	GOUT_BLK_DISPAUD_UID_PERI_AXI_ASB_IPCLKPORT_ACLKM,
	GOUT_BLK_DISPAUD_UID_PERI_AXI_ASB_IPCLKPORT_ACLKS,
	GOUT_BLK_DISPAUD_UID_PERI_AXI_ASB_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ppmu_abox[] = {
	GOUT_BLK_DISPAUD_UID_BCM_ABOX_IPCLKPORT_ACLK,
	GOUT_BLK_DISPAUD_UID_BCM_ABOX_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ppmu_dpu[] = {
	GOUT_BLK_DISPAUD_UID_BCM_DPU_IPCLKPORT_ACLK,
	GOUT_BLK_DISPAUD_UID_BCM_DPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_smmu_abox[] = {
	GOUT_BLK_DISPAUD_UID_SMMU_ABOX_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_smmu_dpu[] = {
	GOUT_BLK_DISPAUD_UID_SMMU_DPU_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_sysreg_dispaud[] = {
	GOUT_BLK_DISPAUD_UID_SYSREG_DISPAUD_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_wdt_aboxcpu[] = {
	GOUT_BLK_DISPAUD_UID_WDT_ABOXCPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_xiu_p_dispaud[] = {
	GOUT_BLK_DISPAUD_UID_XIU_P_DISPAUD_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_adm_ahb_sss[] = {
	GOUT_BLK_FSYS_UID_ADM_AHB_SSS_IPCLKPORT_HCLKM,
};
enum clk_id cmucal_vclk_ahb2apb_fsys[] = {
	GOUT_BLK_FSYS_UID_AHB2APB_FSYS_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_ahbbr_fsys[] = {
	GOUT_BLK_FSYS_UID_AHBBR_FSYS_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_ahbbr_fsys_1x4[] = {
	GOUT_BLK_FSYS_UID_AHBBR_FSYS_1x4_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_axi2ahb_fsys[] = {
	GOUT_BLK_FSYS_UID_AXI2AHB_FSYS_IPCLKPORT_aclk,
};
enum clk_id cmucal_vclk_fsys_cmu_fsys[] = {
	CLK_BLK_FSYS_UID_FSYS_CMU_FSYS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_gpio_fsys[] = {
	GOUT_BLK_FSYS_UID_GPIO_FSYS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_lhm_axi_p_fsys[] = {
	GOUT_BLK_FSYS_UID_LHM_AXI_P_FSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_d_fsys[] = {
	GOUT_BLK_FSYS_UID_LHS_AXI_D_FSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_mmc_card[] = {
	GOUT_BLK_FSYS_UID_MMC_CARD_IPCLKPORT_I_ACLK,
	GOUT_BLK_FSYS_UID_MMC_CARD_IPCLKPORT_SDCLKIN,
};
enum clk_id cmucal_vclk_mmc_embd[] = {
	GOUT_BLK_FSYS_UID_MMC_EMBD_IPCLKPORT_I_ACLK,
	GOUT_BLK_FSYS_UID_MMC_EMBD_IPCLKPORT_SDCLKIN,
};
enum clk_id cmucal_vclk_mmc_sdio[] = {
	GOUT_BLK_FSYS_UID_MMC_SDIO_IPCLKPORT_I_ACLK,
	GOUT_BLK_FSYS_UID_MMC_SDIO_IPCLKPORT_SDCLKIN,
};
enum clk_id cmucal_vclk_ppmu_fsys[] = {
	GOUT_BLK_FSYS_UID_BCM_FSYS_IPCLKPORT_ACLK,
	GOUT_BLK_FSYS_UID_BCM_FSYS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_rtic[] = {
	GOUT_BLK_FSYS_UID_RTIC_IPCLKPORT_i_ACLK,
	GOUT_BLK_FSYS_UID_RTIC_IPCLKPORT_i_PCLK,
};
enum clk_id cmucal_vclk_sss[] = {
	GOUT_BLK_FSYS_UID_SSS_IPCLKPORT_i_ACLK,
	GOUT_BLK_FSYS_UID_SSS_IPCLKPORT_i_PCLK,
};
enum clk_id cmucal_vclk_sysreg_fsys[] = {
	GOUT_BLK_FSYS_UID_SYSREG_FSYS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_usb20drd[] = {
	GOUT_BLK_FSYS_UID_USB20DRD_IPCLKPORT_ACLK_PHYCTRL,
	GOUT_BLK_FSYS_UID_USB20DRD_IPCLKPORT_bus_clk_early,
};
enum clk_id cmucal_vclk_xiu_d_fsys[] = {
	GOUT_BLK_FSYS_UID_XIU_D_FSYS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_axi2apb_g3d[] = {
	GOUT_BLK_G3D_UID_AXI2APB_G3D_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_g3d_cmu_g3d[] = {
	CLK_BLK_G3D_UID_G3D_CMU_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_lhm_axi_g3dsfr[] = {
	GOUT_BLK_G3D_UID_LHM_AXI_G3DSFR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_p_g3d[] = {
	GOUT_BLK_G3D_UID_LHM_AXI_P_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_d_g3d[] = {
	GOUT_BLK_G3D_UID_LHS_AXI_D_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_g3dsfr[] = {
	GOUT_BLK_G3D_UID_LHS_AXI_G3DSFR_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_sysreg_g3d[] = {
	GOUT_BLK_G3D_UID_SYSREG_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_xiu_p_g3d[] = {
	GOUT_BLK_G3D_UID_XIU_P_G3D_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_asyncm_p_is[] = {
	GOUT_BLK_IS_UID_ASYNCM_P_IS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_asyncs_d0_is[] = {
	GOUT_BLK_IS_UID_ASYNCS_D0_IS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_asyncs_d1_is[] = {
	GOUT_BLK_IS_UID_ASYNCS_D1_IS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_is_cmu_is[] = {
	CLK_BLK_IS_UID_IS_CMU_IS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ppmu_nrt[] = {
	GOUT_BLK_IS_UID_BCM_NRT_IPCLKPORT_ACLK,
	GOUT_BLK_IS_UID_BCM_NRT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ppmu_rt[] = {
	GOUT_BLK_IS_UID_BCM_RT_IPCLKPORT_ACLK,
	GOUT_BLK_IS_UID_BCM_RT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_smmu_is[] = {
	GOUT_BLK_IS_UID_SMMU_IS_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_sysreg_is[] = {
	GOUT_BLK_IS_UID_SYSREG_IS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_is5p15p0_is[] = {
	CLK_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS1_ISP,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_3AA,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_ASYNCM_3AA,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_ASYNCM_CSIS,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_ASYNCM_MCSC,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_ASYNCM_SMMU_NRT,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_ASYNCM_SMMU_RT,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_ASYNCM_TPU,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_ASYNCM_VRA,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_CSIS_0,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_CSIS_0_HALF,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_CSIS_1,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_CSIS_1_HALF,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_CSIS_DMA,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_MCSC,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_TPU,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_IS_VRA,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_CLK_XIU_IS_D,
	GOUT_BLK_IS_UID_is5p15p0_IS_IPCLKPORT_PCLK_IS,
};
enum clk_id cmucal_vclk_async_axi[] = {
	GOUT_BLK_MFCMSCL_UID_ASYNC_AXI_IPCLKPORT_ACLKM,
	GOUT_BLK_MFCMSCL_UID_ASYNC_AXI_IPCLKPORT_ACLKS,
};
enum clk_id cmucal_vclk_async_g2d_p[] = {
	GOUT_BLK_MFCMSCL_UID_ASYNC_G2D_P_IPCLKPORT_PCLKM,
	GOUT_BLK_MFCMSCL_UID_ASYNC_G2D_P_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_async_jpeg_p[] = {
	GOUT_BLK_MFCMSCL_UID_ASYNC_JPEG_P_IPCLKPORT_PCLKM,
	GOUT_BLK_MFCMSCL_UID_ASYNC_JPEG_P_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_async_mfc_p[] = {
	GOUT_BLK_MFCMSCL_UID_ASYNC_MFC_P_IPCLKPORT_PCLKM,
	GOUT_BLK_MFCMSCL_UID_ASYNC_MFC_P_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_async_mscl_p[] = {
	GOUT_BLK_MFCMSCL_UID_ASYNC_MSCL_P_IPCLKPORT_PCLKM,
	GOUT_BLK_MFCMSCL_UID_ASYNC_MSCL_P_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_async_smmu_ns_p[] = {
	GOUT_BLK_MFCMSCL_UID_ASYNC_SMMU_NS_P_IPCLKPORT_PCLKM,
	GOUT_BLK_MFCMSCL_UID_ASYNC_SMMU_NS_P_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_async_smmu_s_p[] = {
	GOUT_BLK_MFCMSCL_UID_ASYNC_SMMU_S_P_IPCLKPORT_PCLKM,
	GOUT_BLK_MFCMSCL_UID_ASYNC_SMMU_S_P_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_axi2apb_mfcmscl[] = {
	GOUT_BLK_MFCMSCL_UID_AXI2APB_MFCMSCL_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_g2d[] = {
	GOUT_BLK_MFCMSCL_UID_G2D_IPCLKPORT_Clk,
};
enum clk_id cmucal_vclk_jpeg[] = {
	GOUT_BLK_MFCMSCL_UID_JPEG_IPCLKPORT_I_FIMP_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_p_mfcmscl[] = {
	GOUT_BLK_MFCMSCL_UID_LHM_AXI_P_MFCMSCL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_d_mfcmscl[] = {
	GOUT_BLK_MFCMSCL_UID_LHS_AXI_D_MFCMSCL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_mfc[] = {
	GOUT_BLK_MFCMSCL_UID_MFC_IPCLKPORT_Clk,
};
enum clk_id cmucal_vclk_mfcmscl_cmu_mfcmscl[] = {
	CLK_BLK_MFCMSCL_UID_MFCMSCL_CMU_MFCMSCL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mscl[] = {
	GOUT_BLK_MFCMSCL_UID_MSCL_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ppmu_mfcmscl[] = {
	GOUT_BLK_MFCMSCL_UID_BCM_MFCMSCL_IPCLKPORT_ACLK,
	GOUT_BLK_MFCMSCL_UID_BCM_MFCMSCL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_smmu_mfcmscl[] = {
	GOUT_BLK_MFCMSCL_UID_SMMU_MFCMSCL_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_sysreg_mfcmscl[] = {
	GOUT_BLK_MFCMSCL_UID_SYSREG_MFCMSCL_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_xiu_d_mfcmscl[] = {
	GOUT_BLK_MFCMSCL_UID_XIU_D_MFCMSCL_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_ads_apb_g_cssys[] = {
	GOUT_BLK_MIF_UID_ADS_APB_G_CSSYS_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_pdma0[] = {
	GOUT_BLK_MIF_UID_AD_APB_PDMA0_IPCLKPORT_PCLKM,
	GOUT_BLK_MIF_UID_AD_APB_PDMA0_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_apb_spdma[] = {
	GOUT_BLK_MIF_UID_AD_APB_SPDMA_IPCLKPORT_PCLKM,
	GOUT_BLK_MIF_UID_AD_APB_SPDMA_IPCLKPORT_PCLKS,
};
enum clk_id cmucal_vclk_ad_axi_gic[] = {
	GOUT_BLK_MIF_UID_AD_AXI_GIC_IPCLKPORT_ACLKM,
	GOUT_BLK_MIF_UID_AD_AXI_GIC_IPCLKPORT_ACLKS,
	GOUT_BLK_MIF_UID_AD_AXI_GIC_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ahb2apb_corep0[] = {
	GOUT_BLK_MIF_UID_AHB2APB_COREP0_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_ahb2apb_cssys_dbg[] = {
	GOUT_BLK_MIF_UID_AHB2APB_CSSYS_DBG_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_ahb_bridge[] = {
	GOUT_BLK_MIF_UID_AHB_BRIDGE_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_axi2ahb_corep[] = {
	GOUT_BLK_MIF_UID_AXI2AHB_COREP_IPCLKPORT_aclk,
};
enum clk_id cmucal_vclk_axi2ahb_core_cssys[] = {
	GOUT_BLK_MIF_UID_AXI2AHB_CORE_CSSYS_IPCLKPORT_aclk,
};
enum clk_id cmucal_vclk_axi2apb_2mb_buscp[] = {
	GOUT_BLK_MIF_UID_AXI2APB_2MB_BUSCP_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_axi_async_dmc_cpu[] = {
	CLK_BLK_MIF_UID_AXI_ASYNC_DMC_CPU_IPCLKPORT_ACLKM,
	GOUT_BLK_MIF_UID_AXI_ASYNC_DMC_CPU_IPCLKPORT_ACLKS,
	GOUT_BLK_MIF_UID_AXI_ASYNC_DMC_CPU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_axi_us_a40_32to128_pdma[] = {
	GOUT_BLK_MIF_UID_AXI_US_A40_32to128_PDMA_IPCLKPORT_aclk,
};
enum clk_id cmucal_vclk_axi_us_a40_32to128_sdma[] = {
	GOUT_BLK_MIF_UID_AXI_US_A40_32to128_SDMA_IPCLKPORT_aclk,
};
enum clk_id cmucal_vclk_axi_us_a40_64to128_cssys[] = {
	GOUT_BLK_MIF_UID_AXI_US_A40_64to128_CSSYS_IPCLKPORT_aclk,
};
enum clk_id cmucal_vclk_cci_400[] = {
	GOUT_BLK_MIF_UID_CCI_400_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_gic400_aihwacg[] = {
	GOUT_BLK_MIF_UID_GIC400_AIHWACG_IPCLKPORT_CLK,
};
enum clk_id cmucal_vclk_lblk_mif[] = {
	CLK_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_ACLK_DMC0,
	GOUT_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_DDR_PHY0_PCLK,
	GOUT_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_HCLK_AHB2APB_BRIDGE_MIF,
	GOUT_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_I_PCLK_ASYNCSFR_WR_DMC_SECURE,
	GOUT_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_PCLK_APBBR_DMC_SECURE,
	GOUT_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_PCLK_DMC0,
	GOUT_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_PCLK_PF_DMC0,
	GOUT_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_PCLK_PPMPU_DMC0,
	GOUT_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_PCLK_SECURE_DMC0,
	CLK_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_BCM_CPU_ACLK,
	GOUT_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_BCM_CPU_PCLK,
	GOUT_BLK_MIF_UID_LBLK_MIF_IPCLKPORT_clk,
};
enum clk_id cmucal_vclk_lhm_ace_d_cpucl0[] = {
	GOUT_BLK_MIF_UID_LHM_ACE_D_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_ace_d_cpucl1[] = {
	GOUT_BLK_MIF_UID_LHM_ACE_D_CPUCL1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_abox[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_ABOX_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_apm[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_APM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_cp[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_CP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_cssys[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_CSSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_dpu[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_DPU_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_fsys[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_FSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_g3d[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_gnss[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_GNSS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_ispnrt[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_ISPNRT_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_isprt[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_ISPRT_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_mfcmscl[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_MFCMSCL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_d_wlbt[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_D_WLBT_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhm_axi_p_cp[] = {
	GOUT_BLK_MIF_UID_LHM_AXI_P_CP_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_p_apm[] = {
	GOUT_BLK_MIF_UID_LHS_AXI_P_APM_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_p_cpucl0[] = {
	GOUT_BLK_MIF_UID_LHS_AXI_P_CPUCL0_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_p_cpucl1[] = {
	GOUT_BLK_MIF_UID_LHS_AXI_P_CPUCL1_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_p_dispaud[] = {
	GOUT_BLK_MIF_UID_LHS_AXI_P_DISPAUD_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_p_fsys[] = {
	GOUT_BLK_MIF_UID_LHS_AXI_P_FSYS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_p_g3d[] = {
	GOUT_BLK_MIF_UID_LHS_AXI_P_G3D_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_p_is[] = {
	GOUT_BLK_MIF_UID_LHS_AXI_P_IS_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_p_mfcmscl[] = {
	GOUT_BLK_MIF_UID_LHS_AXI_P_MFCMSCL_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_lhs_axi_p_peri[] = {
	GOUT_BLK_MIF_UID_LHS_AXI_P_PERI_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_mailbox[] = {
	GOUT_BLK_MIF_UID_MAILBOX_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mailbox_abox[] = {
	GOUT_BLK_MIF_UID_MAILBOX_ABOX_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mailbox_gnsss[] = {
	GOUT_BLK_MIF_UID_MAILBOX_GNSSS_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mailbox_secure[] = {
	GOUT_BLK_MIF_UID_MAILBOX_SECURE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mailbox_wlbt0[] = {
	GOUT_BLK_MIF_UID_MAILBOX_WLBT0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mailbox_wlbt1[] = {
	GOUT_BLK_MIF_UID_MAILBOX_WLBT1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_mif_cmu_mif[] = {
	CLK_BLK_MIF_UID_MIF_CMU_MIF_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_pdma_core[] = {
	GOUT_BLK_MIF_UID_PDMA_CORE_IPCLKPORT_ACLK_PDMA0,
};
enum clk_id cmucal_vclk_ppcfw_g3d[] = {
	GOUT_BLK_MIF_UID_PPCFW_G3D_IPCLKPORT_ACLK,
	GOUT_BLK_MIF_UID_PPCFW_G3D_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ppmu_ace_cpucl0[] = {
	GOUT_BLK_MIF_UID_BCM_ACE_CPUCL0_IPCLKPORT_ACLK,
	GOUT_BLK_MIF_UID_BCM_ACE_CPUCL0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_ppmu_ace_cpucl1[] = {
	GOUT_BLK_MIF_UID_BCM_ACE_CPUCL1_IPCLKPORT_ACLK,
	GOUT_BLK_MIF_UID_BCM_ACE_CPUCL1_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_qe[] = {
	GOUT_BLK_MIF_UID_QE_IPCLKPORT_ACLK,
	GOUT_BLK_MIF_UID_QE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_sfr_apbif_cmu_cmu[] = {
	GOUT_BLK_MIF_UID_SFR_APBIF_CMU_CMU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_spdma_core[] = {
	GOUT_BLK_MIF_UID_SPDMA_CORE_IPCLKPORT_ACLK_PDMA1,
};
enum clk_id cmucal_vclk_sysreg_mif[] = {
	GOUT_BLK_MIF_UID_SYSREG_MIF_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_trex_d_core[] = {
	GOUT_BLK_MIF_UID_TREX_D_CORE_IPCLKPORT_ACLK,
	GOUT_BLK_MIF_UID_TREX_D_CORE_IPCLKPORT_MCLK,
	GOUT_BLK_MIF_UID_TREX_D_CORE_IPCLKPORT_pclk,
};
enum clk_id cmucal_vclk_trex_p_core[] = {
	GOUT_BLK_MIF_UID_TREX_P_CORE_IPCLKPORT_ACLK,
	GOUT_BLK_MIF_UID_TREX_P_CORE_IPCLKPORT_CCLK,
	GOUT_BLK_MIF_UID_TREX_P_CORE_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_wrap_adc_if[] = {
	GOUT_BLK_MIF_UID_WRAP_ADC_IF_IPCLKPORT_PCLK_S0,
	GOUT_BLK_MIF_UID_WRAP_ADC_IF_IPCLKPORT_PCLK_S1,
};
enum clk_id cmucal_vclk_xiu_d_pdma_3x1[] = {
	GOUT_BLK_MIF_UID_XIU_D_PDMA_3x1_IPCLKPORT_ACLK,
};
enum clk_id cmucal_vclk_modem_cmu[] = {
	CLK_BLK_MODEM_UID_MODEM_CMU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_axi2ahb_peri[] = {
	GOUT_BLK_PERI_UID_AXI2AHB_PERI_IPCLKPORT_aclk,
};
enum clk_id cmucal_vclk_busif_tmu[] = {
	GOUT_BLK_PERI_UID_BUSIF_TMU_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_busp1_peris0[] = {
	GOUT_BLK_PERI_UID_BUSP1_PERIS0_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_busp_br_peri[] = {
	GOUT_BLK_PERI_UID_BUSP_BR_PERI_IPCLKPORT_HCLK,
};
enum clk_id cmucal_vclk_lblk_peric[] = {
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_BUSP1_PERIC0_HCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_BUSP1_PERIC1_HCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_GPIO_TOP_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_HSI2C0_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_HSI2C1_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_HSI2C2_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_HSI2C3_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_I2C0_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_I2C1_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_I2C2_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_I2C3_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_I2C4_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_I2C5_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_I2C6_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_PWM_MOTOR_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_SPI0_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_SPI0_SPI_EXT_CLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_SPI1_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_SPI1_SPI_EXT_CLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_UART0_EXT_UCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_UART0_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_UART1_EXT_UCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_UART1_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_UART2_EXT_UCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_UART2_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_USI0_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_USI0_SCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_USI1_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_USI1_SCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_USI2_PCLK,
	GOUT_BLK_PERI_UID_LBLK_PERIC_IPCLKPORT_USI2_SCLK,
};
enum clk_id cmucal_vclk_lhm_axi_p_peri[] = {
	GOUT_BLK_PERI_UID_LHM_AXI_P_PERI_IPCLKPORT_I_CLK,
};
enum clk_id cmucal_vclk_mct[] = {
	GOUT_BLK_PERI_UID_MCT_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_otp_con_top[] = {
	GOUT_BLK_PERI_UID_OTP_CON_TOP_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_peri_cmu_peri[] = {
	CLK_BLK_PERI_UID_PERI_CMU_PERI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_secucon[] = {
	GOUT_BLK_PERI_UID_SECUCON_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_sysreg_peri[] = {
	GOUT_BLK_PERI_UID_SYSREG_PERI_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_wdt_cluster0[] = {
	GOUT_BLK_PERI_UID_WDT_CLUSTER0_IPCLKPORT_PCLK,
};
enum clk_id cmucal_vclk_wdt_cluster1[] = {
	GOUT_BLK_PERI_UID_WDT_CLUSTER1_IPCLKPORT_PCLK,
};

/* Switching LUT */
/* -1 is the Value of EMPTY_CAL_ID */
struct switch_lut spl_clk_cpucl0_cntclk_blk_cmu_lut[] = {
	{1600000, -1, 0},
	{400000, -1, 3},
};
struct switch_lut spl_clk_cpucl1_cntclk_blk_cmu_lut[] = {
	{800000, -1, 0},
	{400000, -1, 1},
	{200000, -1, 3},
};
struct switch_lut spl_clk_aud_cpu_aclk_blk_cmu_lut[] = {
	{800000, -1, 0},
	{400000, -1, 1},
	{266666, -1, 2},
};
struct switch_lut clk_mif_phy_clkm_blk_cmu_lut[] = {
	{1600000, 0, 0},
	{1332000, 1, 0},
	{666000, 1, 1},
};

/* DVFS LUT */
struct vclk_lut vdd_cpucl0_lut[] = {
	{1698666, vdd_cpucl0_od_lut_params},
	{1300000, vdd_cpucl0_nm_lut_params},
	{747500, vdd_cpucl0_ud_lut_params},
	{477000, vdd_cpucl0_sud_lut_params},
	{275166, vdd_cpucl0_sod_lut_params},
};
struct vclk_lut vdd_cpucl1_lut[] = {
	{1603333, vdd_cpucl1_sod_lut_params},
	{1352000, vdd_cpucl1_od_lut_params},
	{1005333, vdd_cpucl1_nm_lut_params},
	{600000, vdd_cpucl1_ud_lut_params},
	{385000, vdd_cpucl1_sud_lut_params},
};
struct vclk_lut vdd_g3d_lut[] = {
	{1200000, vdd_g3d_sod_lut_params},
	{949999, vdd_g3d_od_lut_params},
	{747500, vdd_g3d_nm_lut_params},
	{550000, vdd_g3d_ud_lut_params},
	{300000, vdd_g3d_sud_lut_params},
};
struct vclk_lut vdd_int_lut[] = {
	{1865500, vdd_int_nm_lut_params},
	{1332500, vdd_int_ud_lut_params},
	{667000, vdd_int_sud_lut_params},
};
struct vclk_lut dfs_abox_lut[] = {
	{1179648, dfs_abox_nm_lut_params},
	{589824, dfs_abox_ud_lut_params},
	{393216, dfs_abox_sud_lut_params},
	{98304, dfs_abox_ssud_lut_params},
};

/* SPECIAL LUT */
struct vclk_lut spl_clk_fsys_mmc_card_blk_cmu_lut[] = {
	{800000, spl_clk_fsys_mmc_card_blk_cmu_nm_lut_params},
	{400000, spl_clk_fsys_mmc_card_blk_cmu_l0_lut_params},
	{200000, spl_clk_fsys_mmc_card_blk_cmu_l1_lut_params},
	{100000, spl_clk_fsys_mmc_card_blk_cmu_l2_lut_params},
};
struct vclk_lut mux_clkcmu_is_isp_user_blk_cmu_lut[] = {
	{400000, mux_clkcmu_is_isp_user_blk_cmu_l0_lut_params},
	{320000, mux_clkcmu_is_isp_user_blk_cmu_ud_lut_params},
	{160000, mux_clkcmu_is_isp_user_blk_cmu_sud_lut_params},
};
struct vclk_lut spl_clk_peri_spi_1_blk_cmu_lut[] = {
	{100000, spl_clk_peri_spi_1_blk_cmu_nm_lut_params},
	{50000, spl_clk_peri_spi_1_blk_cmu_l0_lut_params},
	{26000, spl_clk_peri_spi_1_blk_cmu_l2_lut_params},
	{25000, spl_clk_peri_spi_1_blk_cmu_l1_lut_params},
	{13000, spl_clk_peri_spi_1_blk_cmu_l3_lut_params},
};
struct vclk_lut spl_clk_peri_uart_2_blk_cmu_lut[] = {
	{200000, spl_clk_peri_uart_2_blk_cmu_l0_lut_params},
	{133333, spl_clk_peri_uart_2_blk_cmu_sud_lut_params},
};
struct vclk_lut spl_clk_peri_usi2_blk_cmu_lut[] = {
	{200000, spl_clk_peri_usi2_blk_cmu_nm_lut_params},
	{100000, spl_clk_peri_usi2_blk_cmu_l0_lut_params},
	{50000, spl_clk_peri_usi2_blk_cmu_l1_lut_params},
	{26000, spl_clk_peri_usi2_blk_cmu_l2_lut_params},
	{25000, spl_clk_peri_usi2_blk_cmu_l3_lut_params},
	{13000, spl_clk_peri_usi2_blk_cmu_l4_lut_params},
};
struct vclk_lut clkcmu_cis_clk0_blk_cmu_lut[] = {
	{100000, clkcmu_cis_clk0_blk_cmu_l1_lut_params},
	{26000, clkcmu_cis_clk0_blk_cmu_l0_lut_params},
};
struct vclk_lut clkcmu_cis_clk2_blk_cmu_lut[] = {
	{100000, clkcmu_cis_clk2_blk_cmu_l1_lut_params},
	{26000, clkcmu_cis_clk2_blk_cmu_l0_lut_params},
};
struct vclk_lut spl_clk_fsys_mmc_embd_blk_cmu_lut[] = {
	{800000, spl_clk_fsys_mmc_embd_blk_cmu_nm_lut_params},
	{400000, spl_clk_fsys_mmc_embd_blk_cmu_l0_lut_params},
	{200000, spl_clk_fsys_mmc_embd_blk_cmu_l1_lut_params},
	{100000, spl_clk_fsys_mmc_embd_blk_cmu_l2_lut_params},
};
struct vclk_lut spl_clk_fsys_mmc_sdio_blk_cmu_lut[] = {
	{800000, spl_clk_fsys_mmc_sdio_blk_cmu_nm_lut_params},
	{400000, spl_clk_fsys_mmc_sdio_blk_cmu_l0_lut_params},
	{200000, spl_clk_fsys_mmc_sdio_blk_cmu_l1_lut_params},
	{100000, spl_clk_fsys_mmc_sdio_blk_cmu_l2_lut_params},
};
struct vclk_lut spl_clk_is_tpu_blk_cmu_lut[] = {
	{400000, spl_clk_is_tpu_blk_cmu_l0_lut_params},
	{320000, spl_clk_is_tpu_blk_cmu_ud_lut_params},
	{160000, spl_clk_is_tpu_blk_cmu_sud_lut_params},
};
struct vclk_lut spl_clk_peri_spi_0_blk_cmu_lut[] = {
	{100000, spl_clk_peri_spi_0_blk_cmu_nm_lut_params},
	{50000, spl_clk_peri_spi_0_blk_cmu_l0_lut_params},
	{26000, spl_clk_peri_spi_0_blk_cmu_l2_lut_params},
	{25000, spl_clk_peri_spi_0_blk_cmu_l1_lut_params},
	{13000, spl_clk_peri_spi_0_blk_cmu_l3_lut_params},
};
struct vclk_lut spl_clk_peri_uart_0_blk_cmu_lut[] = {
	{200000, spl_clk_peri_uart_0_blk_cmu_l0_lut_params},
	{133333, spl_clk_peri_uart_0_blk_cmu_sud_lut_params},
};
struct vclk_lut spl_clk_peri_usi1_blk_cmu_lut[] = {
	{200000, spl_clk_peri_usi1_blk_cmu_nm_lut_params},
	{133333, spl_clk_peri_usi1_blk_cmu_sud_lut_params},
	{100000, spl_clk_peri_usi1_blk_cmu_l0_lut_params},
	{50000, spl_clk_peri_usi1_blk_cmu_l1_lut_params},
	{26000, spl_clk_peri_usi1_blk_cmu_l2_lut_params},
	{25000, spl_clk_peri_usi1_blk_cmu_l3_lut_params},
	{13000, spl_clk_peri_usi1_blk_cmu_l4_lut_params},
};
struct vclk_lut occ_mif_cmuref_blk_cmu_lut[] = {
	{800000, occ_mif_cmuref_blk_cmu_l0_lut_params},
	{400000, occ_mif_cmuref_blk_cmu_ud_lut_params},
	{200000, occ_mif_cmuref_blk_cmu_sud_lut_params},
};
struct vclk_lut occ_cmu_cmuref_blk_cmu_lut[] = {
	{26000, occ_cmu_cmuref_blk_cmu_l0_lut_params},
};
struct vclk_lut clkcmu_cis_clk1_blk_cmu_lut[] = {
	{100000, clkcmu_cis_clk1_blk_cmu_l1_lut_params},
	{26000, clkcmu_cis_clk1_blk_cmu_l0_lut_params},
};
struct vclk_lut spl_clk_peri_usi0_blk_cmu_lut[] = {
	{200000, spl_clk_peri_usi0_blk_cmu_nm_lut_params},
	{133333, spl_clk_peri_usi0_blk_cmu_sud_lut_params},
	{100000, spl_clk_peri_usi0_blk_cmu_l0_lut_params},
	{50000, spl_clk_peri_usi0_blk_cmu_l1_lut_params},
	{26000, spl_clk_peri_usi0_blk_cmu_l2_lut_params},
	{25000, spl_clk_peri_usi0_blk_cmu_l3_lut_params},
	{13000, spl_clk_peri_usi0_blk_cmu_l4_lut_params},
};
struct vclk_lut spl_clk_peri_uart_1_blk_cmu_lut[] = {
	{200000, spl_clk_peri_uart_1_blk_cmu_l0_lut_params},
	{133333, spl_clk_peri_uart_1_blk_cmu_sud_lut_params},
};
struct vclk_lut spl_clk_is_3aa_half_blk_cmu_lut[] = {
	{400000, spl_clk_is_3aa_half_blk_cmu_l0_lut_params},
	{320000, spl_clk_is_3aa_half_blk_cmu_ud_lut_params},
	{160000, spl_clk_is_3aa_half_blk_cmu_sud_lut_params},
};
struct vclk_lut spl_clk_cpucl0_cntclk_blk_cpucl0_lut[] = {
	{424666, spl_clk_cpucl0_cntclk_blk_cpucl0_od_lut_params},
	{325000, spl_clk_cpucl0_cntclk_blk_cpucl0_nm_lut_params},
	{186875, spl_clk_cpucl0_cntclk_blk_cpucl0_ud_lut_params},
	{119250, spl_clk_cpucl0_cntclk_blk_cpucl0_sud_lut_params},
	{68791, spl_clk_cpucl0_cntclk_blk_cpucl0_sod_lut_params},
};
struct vclk_lut spl_clk_cpucl0_atclk_blk_cpucl0_lut[] = {
	{424666, spl_clk_cpucl0_atclk_blk_cpucl0_od_lut_params},
	{325000, spl_clk_cpucl0_atclk_blk_cpucl0_nm_lut_params},
	{186875, spl_clk_cpucl0_atclk_blk_cpucl0_ud_lut_params},
	{119250, spl_clk_cpucl0_atclk_blk_cpucl0_sud_lut_params},
	{68791, spl_clk_cpucl0_atclk_blk_cpucl0_sod_lut_params},
};
struct vclk_lut spl_clk_cpucl0_cmuref_blk_cpucl0_lut[] = {
	{849333, spl_clk_cpucl0_cmuref_blk_cpucl0_od_lut_params},
	{650000, spl_clk_cpucl0_cmuref_blk_cpucl0_nm_lut_params},
	{373750, spl_clk_cpucl0_cmuref_blk_cpucl0_ud_lut_params},
	{238500, spl_clk_cpucl0_cmuref_blk_cpucl0_sud_lut_params},
	{137583, spl_clk_cpucl0_cmuref_blk_cpucl0_sod_lut_params},
};
struct vclk_lut spl_clk_cpucl1_cntclk_blk_cpucl1_lut[] = {
	{400833, spl_clk_cpucl1_cntclk_blk_cpucl1_sod_lut_params},
	{338000, spl_clk_cpucl1_cntclk_blk_cpucl1_od_lut_params},
	{251333, spl_clk_cpucl1_cntclk_blk_cpucl1_nm_lut_params},
	{150000, spl_clk_cpucl1_cntclk_blk_cpucl1_ud_lut_params},
	{96250, spl_clk_cpucl1_cntclk_blk_cpucl1_sud_lut_params},
};
struct vclk_lut div_clk_cpucl1_cmuref_blk_cpucl1_lut[] = {
	{801666, div_clk_cpucl1_cmuref_blk_cpucl1_sod_lut_params},
	{676000, div_clk_cpucl1_cmuref_blk_cpucl1_od_lut_params},
	{502666, div_clk_cpucl1_cmuref_blk_cpucl1_nm_lut_params},
	{300000, div_clk_cpucl1_cmuref_blk_cpucl1_ud_lut_params},
	{192500, div_clk_cpucl1_cmuref_blk_cpucl1_sud_lut_params},
};
struct vclk_lut spl_clk_cpucl1_atclk_blk_cpucl1_lut[] = {
	{400833, spl_clk_cpucl1_atclk_blk_cpucl1_sod_lut_params},
	{338000, spl_clk_cpucl1_atclk_blk_cpucl1_od_lut_params},
	{251333, spl_clk_cpucl1_atclk_blk_cpucl1_nm_lut_params},
	{150000, spl_clk_cpucl1_atclk_blk_cpucl1_ud_lut_params},
	{96250, spl_clk_cpucl1_atclk_blk_cpucl1_sud_lut_params},
};
struct vclk_lut spl_clk_aud_uaif2_blk_dispaud_lut[] = {
	{26000, spl_clk_aud_uaif2_blk_dispaud_l0_lut_params},
};
struct vclk_lut spl_clk_aud_cpu_aclk_blk_dispaud_lut[] = {
	{589323, spl_clk_aud_cpu_aclk_blk_dispaud_nm_lut_params},
	{294912, spl_clk_aud_cpu_aclk_blk_dispaud_ud_lut_params},
	{294661, spl_clk_aud_cpu_aclk_blk_dispaud_l0_lut_params},
	{196441, spl_clk_aud_cpu_aclk_blk_dispaud_l1_lut_params},
	{190500, spl_clk_aud_cpu_aclk_blk_dispaud_sud_lut_params},
	{49110, spl_clk_aud_cpu_aclk_blk_dispaud_l2_lut_params},
};
struct vclk_lut spl_clk_aud_uaif0_blk_dispaud_lut[] = {
	{26000, spl_clk_aud_uaif0_blk_dispaud_l0_lut_params},
};
struct vclk_lut spl_clk_aud_uaif3_blk_dispaud_lut[] = {
	{26000, spl_clk_aud_uaif3_blk_dispaud_l0_lut_params},
};
struct vclk_lut spl_clk_aud_cpu_pclkdbg_blk_dispaud_lut[] = {
	{147330, spl_clk_aud_cpu_pclkdbg_blk_dispaud_nm_lut_params},
	{73728, spl_clk_aud_cpu_pclkdbg_blk_dispaud_ud_lut_params},
	{73665, spl_clk_aud_cpu_pclkdbg_blk_dispaud_l0_lut_params},
	{49110, spl_clk_aud_cpu_pclkdbg_blk_dispaud_l1_lut_params},
	{47625, spl_clk_aud_cpu_pclkdbg_blk_dispaud_sud_lut_params},
	{12277, spl_clk_aud_cpu_pclkdbg_blk_dispaud_l2_lut_params},
};
struct vclk_lut clk_aud_fm_blk_dispaud_lut[] = {
	{26000, clk_aud_fm_blk_dispaud_l0_lut_params},
};
struct vclk_lut spl_clk_is_3aa_half_blk_is_lut[] = {
	{200000, spl_clk_is_3aa_half_blk_is_nm_lut_params},
};
struct vclk_lut occ_mif_cmuref_blk_mif_lut[] = {
	{26000, occ_mif_cmuref_blk_mif_nm_lut_params},
};

/* COMMON LUT */
struct vclk_lut blk_apm_lut[] = {
	{26000, blk_apm_lut_params},
};
struct vclk_lut blk_cmu_lut[] = {
	{1600000, blk_cmu_lut_params},
};
struct vclk_lut blk_cpucl0_lut[] = {
	{162500, blk_cpucl0_lut_params},
};
struct vclk_lut blk_cpucl1_lut[] = {
	{125666, blk_cpucl1_lut_params},
};
struct vclk_lut blk_dispaud_lut[] = {
	{589323, blk_dispaud_lut_params},
};
struct vclk_lut blk_g3d_lut[] = {
	{747500, blk_g3d_lut_params},
};
struct vclk_lut blk_mfcmscl_lut[] = {
	{266666, blk_mfcmscl_lut_params},
};
struct vclk_lut blk_mif_lut[] = {
	{533333, blk_mif_lut_params},
};
/*=================VCLK Switch list================================*/

struct vclk_switch vclk_switch_blk_cpucl0[] = {
	{MUX_CLK_CPUCL0_PLL, EMPTY_CAL_ID, CLKCMU_CPUCL0_SWITCH, GATE_CLKCMU_CPUCL0_SWITCH, MUX_CLKCMU_CPUCL0_SWITCH_USER, spl_clk_cpucl0_cntclk_blk_cmu_lut, 2},
};
struct vclk_switch vclk_switch_blk_cpucl1[] = {
	{MUX_CLK_CPUCL1_PLL, EMPTY_CAL_ID, CLKCMU_CPUCL1_SWITCH, GATE_CLKCMU_CPUCL1_SWITCH, MUX_CLKCMU_CPUCL1_SWITCH_USER, spl_clk_cpucl1_cntclk_blk_cmu_lut, 3},
};
struct vclk_switch vclk_switch_blk_dispaud[] = {
	{MUX_CLK_AUD_CPU, EMPTY_CAL_ID, CLKCMU_DISPAUD_CPU, GATE_CLKCMU_DISPAUD_VCLK, MUX_CLKCMU_AUD_CPU_USER, spl_clk_aud_cpu_aclk_blk_cmu_lut, 3},
};
struct vclk_switch vclk_switch_blk_mif[] = {
	{MUX_CLK_MIF_DDRPHY_CLK2X, MUX_CLKCMU_MIF_SWITCH, CLKCMU_MIF_SWITCH, GATE_CLKCMU_MIF_SWITCH, MUX_CLKCMU_MIF_SWITCH_USER, clk_mif_phy_clkm_blk_cmu_lut, 3},
};

/*=================VCLK list================================*/

struct vclk cmucal_vclk_list[] = {

/* DVFS VCLK */
	CMUCAL_VCLK(VCLK_VDD_CPUCL0, vdd_cpucl0_lut, cmucal_vclk_vdd_cpucl0, NULL, vclk_switch_blk_cpucl0),
	CMUCAL_VCLK(VCLK_VDD_CPUCL1, vdd_cpucl1_lut, cmucal_vclk_vdd_cpucl1, NULL, vclk_switch_blk_cpucl1),
	CMUCAL_VCLK(VCLK_VDD_G3D, vdd_g3d_lut, cmucal_vclk_vdd_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_VDD_INT, vdd_int_lut, cmucal_vclk_vdd_int, NULL, vclk_switch_blk_mif),
	CMUCAL_VCLK(VCLK_DFS_ABOX, dfs_abox_lut, cmucal_vclk_dfs_abox, NULL, NULL),

/* SPECIAL VCLK */
	CMUCAL_VCLK(VCLK_SPL_CLK_FSYS_MMC_CARD_BLK_CMU, spl_clk_fsys_mmc_card_blk_cmu_lut, cmucal_vclk_spl_clk_fsys_mmc_card_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_MUX_CLKCMU_IS_ISP_USER_BLK_CMU, mux_clkcmu_is_isp_user_blk_cmu_lut, cmucal_vclk_mux_clkcmu_is_isp_user_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_PERI_SPI_1_BLK_CMU, spl_clk_peri_spi_1_blk_cmu_lut, cmucal_vclk_spl_clk_peri_spi_1_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_PERI_UART_2_BLK_CMU, spl_clk_peri_uart_2_blk_cmu_lut, cmucal_vclk_spl_clk_peri_uart_2_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_PERI_USI2_BLK_CMU, spl_clk_peri_usi2_blk_cmu_lut, cmucal_vclk_spl_clk_peri_usi2_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLKCMU_CIS_CLK0_BLK_CMU, clkcmu_cis_clk0_blk_cmu_lut, cmucal_vclk_clkcmu_cis_clk0_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLKCMU_CIS_CLK2_BLK_CMU, clkcmu_cis_clk2_blk_cmu_lut, cmucal_vclk_clkcmu_cis_clk2_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_FSYS_MMC_EMBD_BLK_CMU, spl_clk_fsys_mmc_embd_blk_cmu_lut, cmucal_vclk_spl_clk_fsys_mmc_embd_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_FSYS_MMC_SDIO_BLK_CMU, spl_clk_fsys_mmc_sdio_blk_cmu_lut, cmucal_vclk_spl_clk_fsys_mmc_sdio_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_IS_TPU_BLK_CMU, spl_clk_is_tpu_blk_cmu_lut, cmucal_vclk_spl_clk_is_tpu_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_PERI_SPI_0_BLK_CMU, spl_clk_peri_spi_0_blk_cmu_lut, cmucal_vclk_spl_clk_peri_spi_0_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_PERI_UART_0_BLK_CMU, spl_clk_peri_uart_0_blk_cmu_lut, cmucal_vclk_spl_clk_peri_uart_0_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_PERI_USI1_BLK_CMU, spl_clk_peri_usi1_blk_cmu_lut, cmucal_vclk_spl_clk_peri_usi1_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_OCC_MIF_CMUREF_BLK_CMU, occ_mif_cmuref_blk_cmu_lut, cmucal_vclk_occ_mif_cmuref_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_OCC_CMU_CMUREF_BLK_CMU, occ_cmu_cmuref_blk_cmu_lut, cmucal_vclk_occ_cmu_cmuref_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLKCMU_CIS_CLK1_BLK_CMU, clkcmu_cis_clk1_blk_cmu_lut, cmucal_vclk_clkcmu_cis_clk1_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_PERI_USI0_BLK_CMU, spl_clk_peri_usi0_blk_cmu_lut, cmucal_vclk_spl_clk_peri_usi0_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_PERI_UART_1_BLK_CMU, spl_clk_peri_uart_1_blk_cmu_lut, cmucal_vclk_spl_clk_peri_uart_1_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_IS_3AA_HALF_BLK_CMU, spl_clk_is_3aa_half_blk_cmu_lut, cmucal_vclk_spl_clk_is_3aa_half_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_CPUCL0_CNTCLK_BLK_CPUCL0, spl_clk_cpucl0_cntclk_blk_cpucl0_lut, cmucal_vclk_spl_clk_cpucl0_cntclk_blk_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_CPUCL0_ATCLK_BLK_CPUCL0, spl_clk_cpucl0_atclk_blk_cpucl0_lut, cmucal_vclk_spl_clk_cpucl0_atclk_blk_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_CPUCL0_CMUREF_BLK_CPUCL0, spl_clk_cpucl0_cmuref_blk_cpucl0_lut, cmucal_vclk_spl_clk_cpucl0_cmuref_blk_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_CPUCL1_CNTCLK_BLK_CPUCL1, spl_clk_cpucl1_cntclk_blk_cpucl1_lut, cmucal_vclk_spl_clk_cpucl1_cntclk_blk_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_DIV_CLK_CPUCL1_CMUREF_BLK_CPUCL1, div_clk_cpucl1_cmuref_blk_cpucl1_lut, cmucal_vclk_div_clk_cpucl1_cmuref_blk_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_CPUCL1_ATCLK_BLK_CPUCL1, spl_clk_cpucl1_atclk_blk_cpucl1_lut, cmucal_vclk_spl_clk_cpucl1_atclk_blk_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_AUD_UAIF2_BLK_DISPAUD, spl_clk_aud_uaif2_blk_dispaud_lut, cmucal_vclk_spl_clk_aud_uaif2_blk_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_AUD_CPU_ACLK_BLK_DISPAUD, spl_clk_aud_cpu_aclk_blk_dispaud_lut, cmucal_vclk_spl_clk_aud_cpu_aclk_blk_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_AUD_UAIF0_BLK_DISPAUD, spl_clk_aud_uaif0_blk_dispaud_lut, cmucal_vclk_spl_clk_aud_uaif0_blk_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_AUD_UAIF3_BLK_DISPAUD, spl_clk_aud_uaif3_blk_dispaud_lut, cmucal_vclk_spl_clk_aud_uaif3_blk_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_AUD_CPU_PCLKDBG_BLK_DISPAUD, spl_clk_aud_cpu_pclkdbg_blk_dispaud_lut, cmucal_vclk_spl_clk_aud_cpu_pclkdbg_blk_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_CLK_AUD_FM_BLK_DISPAUD, clk_aud_fm_blk_dispaud_lut, cmucal_vclk_clk_aud_fm_blk_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPL_CLK_IS_3AA_HALF_BLK_IS, spl_clk_is_3aa_half_blk_is_lut, cmucal_vclk_spl_clk_is_3aa_half_blk_is, NULL, NULL),
	CMUCAL_VCLK(VCLK_OCC_MIF_CMUREF_BLK_MIF, occ_mif_cmuref_blk_mif_lut, cmucal_vclk_occ_mif_cmuref_blk_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_WPLL_USBPLL, NULL, NULL, NULL, NULL),

/* COMMON VCLK */
	CMUCAL_VCLK(VCLK_BLK_APM, blk_apm_lut, cmucal_vclk_blk_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CMU, blk_cmu_lut, cmucal_vclk_blk_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CPUCL0, blk_cpucl0_lut, cmucal_vclk_blk_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_CPUCL1, blk_cpucl1_lut, cmucal_vclk_blk_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_DISPAUD, blk_dispaud_lut, cmucal_vclk_blk_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_G3D, blk_g3d_lut, cmucal_vclk_blk_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_MFCMSCL, blk_mfcmscl_lut, cmucal_vclk_blk_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_BLK_MIF, blk_mif_lut, cmucal_vclk_blk_mif, NULL, NULL),

/* GATING VCLK */
	CMUCAL_VCLK(VCLK_APBIF_GPIO_ALIVE, NULL, cmucal_vclk_apbif_gpio_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_APBIF_PMU_ALIVE, NULL, cmucal_vclk_apbif_pmu_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_APM, NULL, cmucal_vclk_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_APM_CMU_APM, NULL, cmucal_vclk_apm_cmu_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNCAPB_APM, NULL, cmucal_vclk_asyncapb_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_I2C_APM, NULL, cmucal_vclk_i2c_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BATCHER_AP, NULL, cmucal_vclk_ip_batcher_ap, NULL, NULL),
	CMUCAL_VCLK(VCLK_IP_BATCHER_CP, NULL, cmucal_vclk_ip_batcher_cp, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_P_ALIVE, NULL, cmucal_vclk_lhm_axi_p_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_D_ALIVE, NULL, cmucal_vclk_lhs_axi_d_alive, NULL, NULL),
	CMUCAL_VCLK(VCLK_MAILBOX_APM2AP, NULL, cmucal_vclk_mailbox_apm2ap, NULL, NULL),
	CMUCAL_VCLK(VCLK_MAILBOX_APM2CP, NULL, cmucal_vclk_mailbox_apm2cp, NULL, NULL),
	CMUCAL_VCLK(VCLK_MAILBOX_APM2GNSS, NULL, cmucal_vclk_mailbox_apm2gnss, NULL, NULL),
	CMUCAL_VCLK(VCLK_MAILBOX_APM2WLBT, NULL, cmucal_vclk_mailbox_apm2wlbt, NULL, NULL),
	CMUCAL_VCLK(VCLK_MP_APBSEMA_HWACG_2CH, NULL, cmucal_vclk_mp_apbsema_hwacg_2ch, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPEEDY, NULL, cmucal_vclk_speedy, NULL, NULL),
	CMUCAL_VCLK(VCLK_SYSREG_APM, NULL, cmucal_vclk_sysreg_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_WDT_APM, NULL, cmucal_vclk_wdt_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_ADM_APB_P_CSSYS_DBG, NULL, cmucal_vclk_adm_apb_p_cssys_dbg, NULL, NULL),
	CMUCAL_VCLK(VCLK_ADS_AHB_G_SSS, NULL, cmucal_vclk_ads_ahb_g_sss, NULL, NULL),
	CMUCAL_VCLK(VCLK_ADS_APB_G_CSSYS_DBG, NULL, cmucal_vclk_ads_apb_g_cssys_dbg, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_P_DUMP_PC_CPUCL0, NULL, cmucal_vclk_ad_apb_p_dump_pc_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_P_SECJTAG_CPUCL0, NULL, cmucal_vclk_ad_apb_p_secjtag_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI2APB_CPUCL0, NULL, cmucal_vclk_axi2apb_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_CPUCL0_CMU_CPUCL0, NULL, cmucal_vclk_cpucl0_cmu_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_CSSYS_DBG, NULL, cmucal_vclk_cssys_dbg, NULL, NULL),
	CMUCAL_VCLK(VCLK_DUMP_PC_CPUCL0, NULL, cmucal_vclk_dump_pc_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_P_CPUCL0, NULL, cmucal_vclk_lhm_axi_p_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_ACE_D_CPUCL0, NULL, cmucal_vclk_lhs_ace_d_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_T_CSSYS_DBG, NULL, cmucal_vclk_lhs_axi_t_cssys_dbg, NULL, NULL),
	CMUCAL_VCLK(VCLK_SECJTAG, NULL, cmucal_vclk_secjtag, NULL, NULL),
	CMUCAL_VCLK(VCLK_SYSREG_CPUCL0, NULL, cmucal_vclk_sysreg_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_ADM_APB_G_CPUCL1, NULL, cmucal_vclk_adm_apb_g_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_P_DUMP_PC_CPUCL1, NULL, cmucal_vclk_ad_apb_p_dump_pc_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI2APB_CPUCL1, NULL, cmucal_vclk_axi2apb_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_CPUCL1_CMU_CPUCL1, NULL, cmucal_vclk_cpucl1_cmu_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_DUMP_PC_CPUCL1, NULL, cmucal_vclk_dump_pc_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_P_CPUCL1, NULL, cmucal_vclk_lhm_axi_p_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_ACE_D_CPUCL1, NULL, cmucal_vclk_lhs_ace_d_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_SYSREG_CPUCL1, NULL, cmucal_vclk_sysreg_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_ABOX, NULL, cmucal_vclk_abox, NULL, NULL),
	CMUCAL_VCLK(VCLK_ABOX_DAP, NULL, cmucal_vclk_abox_dap, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_DECON0, NULL, cmucal_vclk_ad_apb_decon0, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_DECON0_SECURE, NULL, cmucal_vclk_ad_apb_decon0_secure, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_DPP, NULL, cmucal_vclk_ad_apb_dpp, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_DPU_DMA, NULL, cmucal_vclk_ad_apb_dpu_dma, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_DPU_DMA_SECURE, NULL, cmucal_vclk_ad_apb_dpu_dma_secure, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_DSIM0, NULL, cmucal_vclk_ad_apb_dsim0, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_SMMU_DPU, NULL, cmucal_vclk_ad_apb_smmu_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_SMMU_DPU_SECURE, NULL, cmucal_vclk_ad_apb_smmu_dpu_secure, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI2APB_DISPAUD, NULL, cmucal_vclk_axi2apb_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI_US_32to128, NULL, cmucal_vclk_axi_us_32to128, NULL, NULL),
	CMUCAL_VCLK(VCLK_DFTMUX_DISPAUD, NULL, cmucal_vclk_dftmux_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_DISPAUD_CMU_DISPAUD, NULL, cmucal_vclk_dispaud_cmu_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_DPU, NULL, cmucal_vclk_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_GPIO_DISPAUD, NULL, cmucal_vclk_gpio_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_P_DISPAUD, NULL, cmucal_vclk_lhm_axi_p_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_D_ABOX, NULL, cmucal_vclk_lhs_axi_d_abox, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_D_DPU, NULL, cmucal_vclk_lhs_axi_d_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_PERI_AXI_ASB, NULL, cmucal_vclk_peri_axi_asb, NULL, NULL),
	CMUCAL_VCLK(VCLK_BCM_ABOX, NULL, cmucal_vclk_ppmu_abox, NULL, NULL),
	CMUCAL_VCLK(VCLK_BCM_DPU, NULL, cmucal_vclk_ppmu_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SMMU_ABOX, NULL, cmucal_vclk_smmu_abox, NULL, NULL),
	CMUCAL_VCLK(VCLK_SMMU_DPU, NULL, cmucal_vclk_smmu_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SYSREG_DISPAUD, NULL, cmucal_vclk_sysreg_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_WDT_ABOXCPU, NULL, cmucal_vclk_wdt_aboxcpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_XIU_P_DISPAUD, NULL, cmucal_vclk_xiu_p_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_ADM_AHB_SSS, NULL, cmucal_vclk_adm_ahb_sss, NULL, NULL),
	CMUCAL_VCLK(VCLK_AHB2APB_FSYS, NULL, cmucal_vclk_ahb2apb_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_AHBBR_FSYS, NULL, cmucal_vclk_ahbbr_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_AHBBR_FSYS_1x4, NULL, cmucal_vclk_ahbbr_fsys_1x4, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI2AHB_FSYS, NULL, cmucal_vclk_axi2ahb_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_FSYS_CMU_FSYS, NULL, cmucal_vclk_fsys_cmu_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_GPIO_FSYS, NULL, cmucal_vclk_gpio_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_P_FSYS, NULL, cmucal_vclk_lhm_axi_p_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_D_FSYS, NULL, cmucal_vclk_lhs_axi_d_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_MMC_CARD, NULL, cmucal_vclk_mmc_card, NULL, NULL),
	CMUCAL_VCLK(VCLK_MMC_EMBD, NULL, cmucal_vclk_mmc_embd, NULL, NULL),
	CMUCAL_VCLK(VCLK_MMC_SDIO, NULL, cmucal_vclk_mmc_sdio, NULL, NULL),
	CMUCAL_VCLK(VCLK_BCM_FSYS, NULL, cmucal_vclk_ppmu_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_RTIC, NULL, cmucal_vclk_rtic, NULL, NULL),
	CMUCAL_VCLK(VCLK_SSS, NULL, cmucal_vclk_sss, NULL, NULL),
	CMUCAL_VCLK(VCLK_SYSREG_FSYS, NULL, cmucal_vclk_sysreg_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_USB20DRD, NULL, cmucal_vclk_usb20drd, NULL, NULL),
	CMUCAL_VCLK(VCLK_XIU_D_FSYS, NULL, cmucal_vclk_xiu_d_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI2APB_G3D, NULL, cmucal_vclk_axi2apb_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_G3D_CMU_G3D, NULL, cmucal_vclk_g3d_cmu_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_G3DSFR, NULL, cmucal_vclk_lhm_axi_g3dsfr, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_P_G3D, NULL, cmucal_vclk_lhm_axi_p_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_D_G3D, NULL, cmucal_vclk_lhs_axi_d_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_G3DSFR, NULL, cmucal_vclk_lhs_axi_g3dsfr, NULL, NULL),
	CMUCAL_VCLK(VCLK_SYSREG_G3D, NULL, cmucal_vclk_sysreg_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_XIU_P_G3D, NULL, cmucal_vclk_xiu_p_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNCM_P_IS, NULL, cmucal_vclk_asyncm_p_is, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNCS_D0_IS, NULL, cmucal_vclk_asyncs_d0_is, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNCS_D1_IS, NULL, cmucal_vclk_asyncs_d1_is, NULL, NULL),
	CMUCAL_VCLK(VCLK_IS_CMU_IS, NULL, cmucal_vclk_is_cmu_is, NULL, NULL),
	CMUCAL_VCLK(VCLK_BCM_NRT, NULL, cmucal_vclk_ppmu_nrt, NULL, NULL),
	CMUCAL_VCLK(VCLK_BCM_RT, NULL, cmucal_vclk_ppmu_rt, NULL, NULL),
	CMUCAL_VCLK(VCLK_SMMU_IS, NULL, cmucal_vclk_smmu_is, NULL, NULL),
	CMUCAL_VCLK(VCLK_SYSREG_IS, NULL, cmucal_vclk_sysreg_is, NULL, NULL),
	CMUCAL_VCLK(VCLK_is5p15p0_IS, NULL, cmucal_vclk_is5p15p0_is, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNC_AXI, NULL, cmucal_vclk_async_axi, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNC_G2D_P, NULL, cmucal_vclk_async_g2d_p, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNC_JPEG_P, NULL, cmucal_vclk_async_jpeg_p, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNC_MFC_P, NULL, cmucal_vclk_async_mfc_p, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNC_MSCL_P, NULL, cmucal_vclk_async_mscl_p, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNC_SMMU_NS_P, NULL, cmucal_vclk_async_smmu_ns_p, NULL, NULL),
	CMUCAL_VCLK(VCLK_ASYNC_SMMU_S_P, NULL, cmucal_vclk_async_smmu_s_p, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI2APB_MFCMSCL, NULL, cmucal_vclk_axi2apb_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_G2D, NULL, cmucal_vclk_g2d, NULL, NULL),
	CMUCAL_VCLK(VCLK_JPEG, NULL, cmucal_vclk_jpeg, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_P_MFCMSCL, NULL, cmucal_vclk_lhm_axi_p_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_D_MFCMSCL, NULL, cmucal_vclk_lhs_axi_d_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_MFC, NULL, cmucal_vclk_mfc, NULL, NULL),
	CMUCAL_VCLK(VCLK_MFCMSCL_CMU_MFCMSCL, NULL, cmucal_vclk_mfcmscl_cmu_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_MSCL, NULL, cmucal_vclk_mscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_BCM_MFCMSCL, NULL, cmucal_vclk_ppmu_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_SMMU_MFCMSCL, NULL, cmucal_vclk_smmu_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_SYSREG_MFCMSCL, NULL, cmucal_vclk_sysreg_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_XIU_D_MFCMSCL, NULL, cmucal_vclk_xiu_d_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_ADS_APB_G_CSSYS, NULL, cmucal_vclk_ads_apb_g_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_PDMA0, NULL, cmucal_vclk_ad_apb_pdma0, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_APB_SPDMA, NULL, cmucal_vclk_ad_apb_spdma, NULL, NULL),
	CMUCAL_VCLK(VCLK_AD_AXI_GIC, NULL, cmucal_vclk_ad_axi_gic, NULL, NULL),
	CMUCAL_VCLK(VCLK_AHB2APB_COREP0, NULL, cmucal_vclk_ahb2apb_corep0, NULL, NULL),
	CMUCAL_VCLK(VCLK_AHB2APB_CSSYS_DBG, NULL, cmucal_vclk_ahb2apb_cssys_dbg, NULL, NULL),
	CMUCAL_VCLK(VCLK_AHB_BRIDGE, NULL, cmucal_vclk_ahb_bridge, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI2AHB_COREP, NULL, cmucal_vclk_axi2ahb_corep, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI2AHB_CORE_CSSYS, NULL, cmucal_vclk_axi2ahb_core_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI2APB_2MB_BUSCP, NULL, cmucal_vclk_axi2apb_2mb_buscp, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI_ASYNC_DMC_CPU, NULL, cmucal_vclk_axi_async_dmc_cpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI_US_A40_32to128_PDMA, NULL, cmucal_vclk_axi_us_a40_32to128_pdma, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI_US_A40_32to128_SDMA, NULL, cmucal_vclk_axi_us_a40_32to128_sdma, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI_US_A40_64to128_CSSYS, NULL, cmucal_vclk_axi_us_a40_64to128_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_CCI_400, NULL, cmucal_vclk_cci_400, NULL, NULL),
	CMUCAL_VCLK(VCLK_GIC400_AIHWACG, NULL, cmucal_vclk_gic400_aihwacg, NULL, NULL),
	CMUCAL_VCLK(VCLK_LBLK_MIF, NULL, cmucal_vclk_lblk_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_ACE_D_CPUCL0, NULL, cmucal_vclk_lhm_ace_d_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_ACE_D_CPUCL1, NULL, cmucal_vclk_lhm_ace_d_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_ABOX, NULL, cmucal_vclk_lhm_axi_d_abox, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_APM, NULL, cmucal_vclk_lhm_axi_d_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_CP, NULL, cmucal_vclk_lhm_axi_d_cp, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_CSSYS, NULL, cmucal_vclk_lhm_axi_d_cssys, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_DPU, NULL, cmucal_vclk_lhm_axi_d_dpu, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_FSYS, NULL, cmucal_vclk_lhm_axi_d_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_G3D, NULL, cmucal_vclk_lhm_axi_d_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_GNSS, NULL, cmucal_vclk_lhm_axi_d_gnss, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_ISPNRT, NULL, cmucal_vclk_lhm_axi_d_ispnrt, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_ISPRT, NULL, cmucal_vclk_lhm_axi_d_isprt, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_MFCMSCL, NULL, cmucal_vclk_lhm_axi_d_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_D_WLBT, NULL, cmucal_vclk_lhm_axi_d_wlbt, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_P_CP, NULL, cmucal_vclk_lhm_axi_p_cp, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_P_APM, NULL, cmucal_vclk_lhs_axi_p_apm, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_P_CPUCL0, NULL, cmucal_vclk_lhs_axi_p_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_P_CPUCL1, NULL, cmucal_vclk_lhs_axi_p_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_P_DISPAUD, NULL, cmucal_vclk_lhs_axi_p_dispaud, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_P_FSYS, NULL, cmucal_vclk_lhs_axi_p_fsys, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_P_G3D, NULL, cmucal_vclk_lhs_axi_p_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_P_IS, NULL, cmucal_vclk_lhs_axi_p_is, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_P_MFCMSCL, NULL, cmucal_vclk_lhs_axi_p_mfcmscl, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHS_AXI_P_PERI, NULL, cmucal_vclk_lhs_axi_p_peri, NULL, NULL),
	CMUCAL_VCLK(VCLK_MAILBOX, NULL, cmucal_vclk_mailbox, NULL, NULL),
	CMUCAL_VCLK(VCLK_MAILBOX_ABOX, NULL, cmucal_vclk_mailbox_abox, NULL, NULL),
	CMUCAL_VCLK(VCLK_MAILBOX_GNSSS, NULL, cmucal_vclk_mailbox_gnsss, NULL, NULL),
	CMUCAL_VCLK(VCLK_MAILBOX_SECURE, NULL, cmucal_vclk_mailbox_secure, NULL, NULL),
	CMUCAL_VCLK(VCLK_MAILBOX_WLBT0, NULL, cmucal_vclk_mailbox_wlbt0, NULL, NULL),
	CMUCAL_VCLK(VCLK_MAILBOX_WLBT1, NULL, cmucal_vclk_mailbox_wlbt1, NULL, NULL),
	CMUCAL_VCLK(VCLK_MIF_CMU_MIF, NULL, cmucal_vclk_mif_cmu_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_PDMA_CORE, NULL, cmucal_vclk_pdma_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_PPCFW_G3D, NULL, cmucal_vclk_ppcfw_g3d, NULL, NULL),
	CMUCAL_VCLK(VCLK_BCM_ACE_CPUCL0, NULL, cmucal_vclk_ppmu_ace_cpucl0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BCM_ACE_CPUCL1, NULL, cmucal_vclk_ppmu_ace_cpucl1, NULL, NULL),
	CMUCAL_VCLK(VCLK_QE, NULL, cmucal_vclk_qe, NULL, NULL),
	CMUCAL_VCLK(VCLK_SFR_APBIF_CMU_CMU, NULL, cmucal_vclk_sfr_apbif_cmu_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_SPDMA_CORE, NULL, cmucal_vclk_spdma_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_SYSREG_MIF, NULL, cmucal_vclk_sysreg_mif, NULL, NULL),
	CMUCAL_VCLK(VCLK_TREX_D_CORE, NULL, cmucal_vclk_trex_d_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_TREX_P_CORE, NULL, cmucal_vclk_trex_p_core, NULL, NULL),
	CMUCAL_VCLK(VCLK_WRAP_ADC_IF, NULL, cmucal_vclk_wrap_adc_if, NULL, NULL),
	CMUCAL_VCLK(VCLK_XIU_D_PDMA_3x1, NULL, cmucal_vclk_xiu_d_pdma_3x1, NULL, NULL),
	CMUCAL_VCLK(VCLK_MODEM_CMU, NULL, cmucal_vclk_modem_cmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_AXI2AHB_PERI, NULL, cmucal_vclk_axi2ahb_peri, NULL, NULL),
	CMUCAL_VCLK(VCLK_BUSIF_TMU, NULL, cmucal_vclk_busif_tmu, NULL, NULL),
	CMUCAL_VCLK(VCLK_BUSP1_PERIS0, NULL, cmucal_vclk_busp1_peris0, NULL, NULL),
	CMUCAL_VCLK(VCLK_BUSP_BR_PERI, NULL, cmucal_vclk_busp_br_peri, NULL, NULL),
	CMUCAL_VCLK(VCLK_LBLK_PERIC, NULL, cmucal_vclk_lblk_peric, NULL, NULL),
	CMUCAL_VCLK(VCLK_LHM_AXI_P_PERI, NULL, cmucal_vclk_lhm_axi_p_peri, NULL, NULL),
	CMUCAL_VCLK(VCLK_MCT, NULL, cmucal_vclk_mct, NULL, NULL),
	CMUCAL_VCLK(VCLK_OTP_CON_TOP, NULL, cmucal_vclk_otp_con_top, NULL, NULL),
	CMUCAL_VCLK(VCLK_PERI_CMU_PERI, NULL, cmucal_vclk_peri_cmu_peri, NULL, NULL),
	CMUCAL_VCLK(VCLK_SECUCON, NULL, cmucal_vclk_secucon, NULL, NULL),
	CMUCAL_VCLK(VCLK_SYSREG_PERI, NULL, cmucal_vclk_sysreg_peri, NULL, NULL),
	CMUCAL_VCLK(VCLK_WDT_CLUSTER0, NULL, cmucal_vclk_wdt_cluster0, NULL, NULL),
	CMUCAL_VCLK(VCLK_WDT_CLUSTER1, NULL, cmucal_vclk_wdt_cluster1, NULL, NULL),
};
unsigned int cmucal_vclk_size = 242;

