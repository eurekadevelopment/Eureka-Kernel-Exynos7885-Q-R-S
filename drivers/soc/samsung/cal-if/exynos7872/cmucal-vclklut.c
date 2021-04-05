#include "../cmucal.h"

#include "cmucal-vclklut.h"

/*=================CMUCAL version: S5E7872================================*/

/*=================LUT in each VCLK================================*/
unsigned int vdd_cpucl0_nm_lut_params[] = {
	 1, 1300000,
};
unsigned int vdd_cpucl0_od_lut_params[] = {
	 1, 1698666,
};
unsigned int vdd_cpucl0_sod_lut_params[] = {
	 1, 275166,
};
unsigned int vdd_cpucl0_sud_lut_params[] = {
	 0, 477000,
};
unsigned int vdd_cpucl0_ud_lut_params[] = {
	 0, 747500,
};
unsigned int vdd_cpucl1_nm_lut_params[] = {
	 0, 1005333,
};
unsigned int vdd_cpucl1_od_lut_params[] = {
	 1, 1352000,
};
unsigned int vdd_cpucl1_sod_lut_params[] = {
	 1, 1603333,
};
unsigned int vdd_cpucl1_sud_lut_params[] = {
	 0, 385000,
};
unsigned int vdd_cpucl1_ud_lut_params[] = {
	 0, 600000,
};
unsigned int vdd_g3d_nm_lut_params[] = {
	 747500,
};
unsigned int vdd_g3d_od_lut_params[] = {
	 949999,
};
unsigned int vdd_g3d_sod_lut_params[] = {
	 1200000,
};
unsigned int vdd_g3d_sud_lut_params[] = {
	 300000,
};
unsigned int vdd_g3d_ud_lut_params[] = {
	 550000,
};
unsigned int vdd_int_nm_lut_params[] = {
	 2, 1865500,
};
unsigned int vdd_int_sud_lut_params[] = {
	 1, 667000,
};
unsigned int vdd_int_ud_lut_params[] = {
	 2, 1332500,
};
unsigned int dfs_abox_nm_lut_params[] = {
	 1, 0,
};
unsigned int dfs_abox_ssud_lut_params[] = {
	 5, 12,
};
unsigned int dfs_abox_sud_lut_params[] = {
	 5, 2,
};
unsigned int dfs_abox_ud_lut_params[] = {
	 5, 1,
};
unsigned int spl_clk_fsys_mmc_card_blk_cmu_l0_lut_params[] = {
	 1, 0,
};
unsigned int spl_clk_fsys_mmc_card_blk_cmu_l1_lut_params[] = {
	 3, 0,
};
unsigned int spl_clk_fsys_mmc_card_blk_cmu_l2_lut_params[] = {
	 7, 0,
};
unsigned int spl_clk_fsys_mmc_card_blk_cmu_nm_lut_params[] = {
	 0, 0,
};
unsigned int mux_clkcmu_is_isp_user_blk_cmu_l0_lut_params[] = {
	 3, 0,
};
unsigned int mux_clkcmu_is_isp_user_blk_cmu_sud_lut_params[] = {
	 9, 0,
};
unsigned int mux_clkcmu_is_isp_user_blk_cmu_ud_lut_params[] = {
	 4, 0,
};
unsigned int spl_clk_peri_spi_1_blk_cmu_l0_lut_params[] = {
	 7, 1,
};
unsigned int spl_clk_peri_spi_1_blk_cmu_l1_lut_params[] = {
	 15, 1,
};
unsigned int spl_clk_peri_spi_1_blk_cmu_l2_lut_params[] = {
	 0, 0,
};
unsigned int spl_clk_peri_spi_1_blk_cmu_l3_lut_params[] = {
	 1, 0,
};
unsigned int spl_clk_peri_spi_1_blk_cmu_nm_lut_params[] = {
	 3, 1,
};
unsigned int spl_clk_peri_uart_2_blk_cmu_l0_lut_params[] = {
	 1, 1,
};
unsigned int spl_clk_peri_uart_2_blk_cmu_sud_lut_params[] = {
	 2, 1,
};
unsigned int spl_clk_peri_usi2_blk_cmu_l0_lut_params[] = {
	 3, 1,
};
unsigned int spl_clk_peri_usi2_blk_cmu_l1_lut_params[] = {
	 7, 1,
};
unsigned int spl_clk_peri_usi2_blk_cmu_l2_lut_params[] = {
	 0, 0,
};
unsigned int spl_clk_peri_usi2_blk_cmu_l3_lut_params[] = {
	 15, 1,
};
unsigned int spl_clk_peri_usi2_blk_cmu_l4_lut_params[] = {
	 1, 0,
};
unsigned int spl_clk_peri_usi2_blk_cmu_nm_lut_params[] = {
	 1, 1,
};
unsigned int clkcmu_cis_clk0_blk_cmu_l0_lut_params[] = {
	 0, 0,
};
unsigned int clkcmu_cis_clk0_blk_cmu_l1_lut_params[] = {
	 3, 1,
};
unsigned int clkcmu_cis_clk2_blk_cmu_l0_lut_params[] = {
	 0, 0,
};
unsigned int clkcmu_cis_clk2_blk_cmu_l1_lut_params[] = {
	 3, 1,
};
unsigned int spl_clk_fsys_mmc_embd_blk_cmu_l0_lut_params[] = {
	 1, 0,
};
unsigned int spl_clk_fsys_mmc_embd_blk_cmu_l1_lut_params[] = {
	 3, 0,
};
unsigned int spl_clk_fsys_mmc_embd_blk_cmu_l2_lut_params[] = {
	 7, 0,
};
unsigned int spl_clk_fsys_mmc_embd_blk_cmu_nm_lut_params[] = {
	 0, 0,
};
unsigned int spl_clk_fsys_mmc_sdio_blk_cmu_l0_lut_params[] = {
	 1, 0,
};
unsigned int spl_clk_fsys_mmc_sdio_blk_cmu_l1_lut_params[] = {
	 3, 0,
};
unsigned int spl_clk_fsys_mmc_sdio_blk_cmu_l2_lut_params[] = {
	 7, 0,
};
unsigned int spl_clk_fsys_mmc_sdio_blk_cmu_nm_lut_params[] = {
	 0, 0,
};
unsigned int spl_clk_is_tpu_blk_cmu_l0_lut_params[] = {
	 3, 0,
};
unsigned int spl_clk_is_tpu_blk_cmu_sud_lut_params[] = {
	 9, 0,
};
unsigned int spl_clk_is_tpu_blk_cmu_ud_lut_params[] = {
	 4, 0,
};
unsigned int spl_clk_peri_spi_0_blk_cmu_l0_lut_params[] = {
	 7, 1,
};
unsigned int spl_clk_peri_spi_0_blk_cmu_l1_lut_params[] = {
	 15, 1,
};
unsigned int spl_clk_peri_spi_0_blk_cmu_l2_lut_params[] = {
	 0, 0,
};
unsigned int spl_clk_peri_spi_0_blk_cmu_l3_lut_params[] = {
	 1, 0,
};
unsigned int spl_clk_peri_spi_0_blk_cmu_nm_lut_params[] = {
	 3, 1,
};
unsigned int spl_clk_peri_uart_0_blk_cmu_l0_lut_params[] = {
	 1, 1,
};
unsigned int spl_clk_peri_uart_0_blk_cmu_sud_lut_params[] = {
	 2, 1,
};
unsigned int spl_clk_peri_usi1_blk_cmu_l0_lut_params[] = {
	 3, 1,
};
unsigned int spl_clk_peri_usi1_blk_cmu_l1_lut_params[] = {
	 7, 1,
};
unsigned int spl_clk_peri_usi1_blk_cmu_l2_lut_params[] = {
	 0, 0,
};
unsigned int spl_clk_peri_usi1_blk_cmu_l3_lut_params[] = {
	 15, 1,
};
unsigned int spl_clk_peri_usi1_blk_cmu_l4_lut_params[] = {
	 1, 0,
};
unsigned int spl_clk_peri_usi1_blk_cmu_nm_lut_params[] = {
	 1, 1,
};
unsigned int spl_clk_peri_usi1_blk_cmu_sud_lut_params[] = {
	 2, 1,
};
unsigned int occ_mif_cmuref_blk_cmu_l0_lut_params[] = {
	 0, 0,
};
unsigned int occ_mif_cmuref_blk_cmu_sud_lut_params[] = {
	 3, 0,
};
unsigned int occ_mif_cmuref_blk_cmu_ud_lut_params[] = {
	 1, 0,
};
unsigned int occ_cmu_cmuref_blk_cmu_l0_lut_params[] = {
	 0, 1, 0,
};
unsigned int clkcmu_cis_clk1_blk_cmu_l0_lut_params[] = {
	 0, 0,
};
unsigned int clkcmu_cis_clk1_blk_cmu_l1_lut_params[] = {
	 3, 1,
};
unsigned int spl_clk_peri_usi0_blk_cmu_l0_lut_params[] = {
	 3, 1,
};
unsigned int spl_clk_peri_usi0_blk_cmu_l1_lut_params[] = {
	 7, 1,
};
unsigned int spl_clk_peri_usi0_blk_cmu_l2_lut_params[] = {
	 0, 0,
};
unsigned int spl_clk_peri_usi0_blk_cmu_l3_lut_params[] = {
	 15, 1,
};
unsigned int spl_clk_peri_usi0_blk_cmu_l4_lut_params[] = {
	 1, 0,
};
unsigned int spl_clk_peri_usi0_blk_cmu_nm_lut_params[] = {
	 1, 1,
};
unsigned int spl_clk_peri_usi0_blk_cmu_sud_lut_params[] = {
	 2, 1,
};
unsigned int spl_clk_peri_uart_1_blk_cmu_l0_lut_params[] = {
	 1, 1,
};
unsigned int spl_clk_peri_uart_1_blk_cmu_sud_lut_params[] = {
	 2, 1,
};
unsigned int spl_clk_is_3aa_half_blk_cmu_l0_lut_params[] = {
	 3, 0,
};
unsigned int spl_clk_is_3aa_half_blk_cmu_sud_lut_params[] = {
	 9, 0,
};
unsigned int spl_clk_is_3aa_half_blk_cmu_ud_lut_params[] = {
	 4, 0,
};
unsigned int spl_clk_cpucl0_cntclk_blk_cpucl0_nm_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl0_cntclk_blk_cpucl0_od_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl0_cntclk_blk_cpucl0_sod_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl0_cntclk_blk_cpucl0_sud_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl0_cntclk_blk_cpucl0_ud_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl0_atclk_blk_cpucl0_nm_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl0_atclk_blk_cpucl0_od_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl0_atclk_blk_cpucl0_sod_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl0_atclk_blk_cpucl0_sud_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl0_atclk_blk_cpucl0_ud_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl0_cmuref_blk_cpucl0_nm_lut_params[] = {
	 1,
};
unsigned int spl_clk_cpucl0_cmuref_blk_cpucl0_od_lut_params[] = {
	 1,
};
unsigned int spl_clk_cpucl0_cmuref_blk_cpucl0_sod_lut_params[] = {
	 1,
};
unsigned int spl_clk_cpucl0_cmuref_blk_cpucl0_sud_lut_params[] = {
	 1,
};
unsigned int spl_clk_cpucl0_cmuref_blk_cpucl0_ud_lut_params[] = {
	 1,
};
unsigned int spl_clk_cpucl1_cntclk_blk_cpucl1_nm_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl1_cntclk_blk_cpucl1_od_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl1_cntclk_blk_cpucl1_sod_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl1_cntclk_blk_cpucl1_sud_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl1_cntclk_blk_cpucl1_ud_lut_params[] = {
	 3,
};
unsigned int div_clk_cpucl1_cmuref_blk_cpucl1_nm_lut_params[] = {
	 1,
};
unsigned int div_clk_cpucl1_cmuref_blk_cpucl1_od_lut_params[] = {
	 1,
};
unsigned int div_clk_cpucl1_cmuref_blk_cpucl1_sod_lut_params[] = {
	 1,
};
unsigned int div_clk_cpucl1_cmuref_blk_cpucl1_sud_lut_params[] = {
	 1,
};
unsigned int div_clk_cpucl1_cmuref_blk_cpucl1_ud_lut_params[] = {
	 1,
};
unsigned int spl_clk_cpucl1_atclk_blk_cpucl1_nm_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl1_atclk_blk_cpucl1_od_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl1_atclk_blk_cpucl1_sod_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl1_atclk_blk_cpucl1_sud_lut_params[] = {
	 3,
};
unsigned int spl_clk_cpucl1_atclk_blk_cpucl1_ud_lut_params[] = {
	 3,
};
unsigned int spl_clk_aud_uaif2_blk_dispaud_l0_lut_params[] = {
	 0, 1,
};
unsigned int spl_clk_aud_cpu_aclk_blk_dispaud_l0_lut_params[] = {
	 1,
};
unsigned int spl_clk_aud_cpu_aclk_blk_dispaud_l1_lut_params[] = {
	 1,
};
unsigned int spl_clk_aud_cpu_aclk_blk_dispaud_l2_lut_params[] = {
	 1,
};
unsigned int spl_clk_aud_cpu_aclk_blk_dispaud_nm_lut_params[] = {
	 1,
};
unsigned int spl_clk_aud_cpu_aclk_blk_dispaud_sud_lut_params[] = {
	 1,
};
unsigned int spl_clk_aud_cpu_aclk_blk_dispaud_ud_lut_params[] = {
	 1,
};
unsigned int spl_clk_aud_uaif0_blk_dispaud_l0_lut_params[] = {
	 0, 1,
};
unsigned int spl_clk_aud_uaif3_blk_dispaud_l0_lut_params[] = {
	 0, 1,
};
unsigned int spl_clk_aud_cpu_pclkdbg_blk_dispaud_l0_lut_params[] = {
	 7,
};
unsigned int spl_clk_aud_cpu_pclkdbg_blk_dispaud_l1_lut_params[] = {
	 7,
};
unsigned int spl_clk_aud_cpu_pclkdbg_blk_dispaud_l2_lut_params[] = {
	 7,
};
unsigned int spl_clk_aud_cpu_pclkdbg_blk_dispaud_nm_lut_params[] = {
	 7,
};
unsigned int spl_clk_aud_cpu_pclkdbg_blk_dispaud_sud_lut_params[] = {
	 7,
};
unsigned int spl_clk_aud_cpu_pclkdbg_blk_dispaud_ud_lut_params[] = {
	 7,
};
unsigned int clk_aud_fm_blk_dispaud_l0_lut_params[] = {
	 0,
};
unsigned int spl_clk_is_3aa_half_blk_is_nm_lut_params[] = {
	 1,
};
unsigned int occ_mif_cmuref_blk_mif_nm_lut_params[] = {
	 0,
};
unsigned int blk_apm_lut_params[] = {
	 0,
};
unsigned int blk_cmu_lut_params[] = {
	 1, 2, 2, 2, 0, 0, 0, 5, 0, 0, 0, 2, 7, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1332000, 1600000, 1, 1,
};
unsigned int blk_cpucl0_lut_params[] = {
	 7, 7,
};
unsigned int blk_cpucl1_lut_params[] = {
	 7, 7,
};
unsigned int blk_dispaud_lut_params[] = {
	 1, 23, 1178647, 0,
};
unsigned int blk_g3d_lut_params[] = {
	 3, 0,
};
unsigned int blk_mfcmscl_lut_params[] = {
	 1,
};
unsigned int blk_mif_lut_params[] = {
	 0, 0, 1, 1, 1, 0,
};
