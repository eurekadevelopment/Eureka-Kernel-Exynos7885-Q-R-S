#ifndef __ASV_EXYNOS7885_H__
#define __ASV_EXYNOS7885_H__

#include <linux/io.h>
#include <linux/printk.h>

#define ASV_TABLE_BASE	(0x10009000)
#define ID_TABLE_BASE	(0x10000000)

struct asv_tbl_info {
	unsigned big_asv_group:4;
	int big_modified_group:4;
	unsigned big_ssa1:4;
	unsigned big_ssa0:4;
	unsigned little_asv_group:4;
	int little_modified_group:4;
	unsigned little_ssa1:4;
	unsigned little_ssa0:4;

	unsigned g3d_asv_group:4;
	int g3d_modified_group:4;
	unsigned g3d_ssa1:4;
	unsigned g3d_ssa0:4;
	unsigned mif_asv_group:4;
	int mif_modified_group:4;
	unsigned mif_ssa1:4;
	unsigned mif_ssa0:4;

	unsigned int_asv_group:4;
	int int_modified_group:4;
	unsigned int_ssa1:4;
	unsigned int_ssa0:4;
	unsigned fsys_asv_group:4;
	int fsys_modified_group:4;
	unsigned fsys_ssa1:4;
	unsigned fsys_ssa0:4;

	unsigned asv_table_ver:7;
	unsigned fused_grp:1;
	unsigned reserved_0:8;
	unsigned cp_asv_group:4;
	int cp_modified_group:4;
	unsigned cp_ssa1:4;
	unsigned cp_ssa0:4;

	unsigned big_vthr:2;
	unsigned big_delta:2;
	unsigned little_vthr:2;
	unsigned little_delta:2;
	unsigned g3d_vthr1:2;
	unsigned g3d_vthr2:2;
	unsigned int_vthr:2;
	unsigned int_delta:2;
	unsigned mif_vthr:2;
	unsigned mif_delta:2;
};

struct id_tbl_info {
	unsigned reserved_0;
	unsigned reserved_1;
	unsigned reserved_2:10;
	unsigned char product_line:2;
	unsigned char reserved_3:4;
	unsigned char ids_cpucl0:8;
	unsigned char ids_g3d:8;
	unsigned char ids_int:8;
	unsigned char asb_version:8;
	unsigned char ids_mif:8;
	unsigned reserved_4:8;
	unsigned reserved_5:16;
	unsigned short sub_rev:4;
	unsigned short main_rev:4;
	unsigned reserved_6:8;
};

#define ASV_INFO_ADDR_CNT	(sizeof(struct asv_tbl_info) / 4)
#define ID_INFO_ADDR_CNT	(sizeof(struct id_tbl_info) / 4)

static struct asv_tbl_info asv_tbl;
static struct id_tbl_info id_tbl;

int asv_get_grp(unsigned int id)
{
	int grp = -1;

	switch (id) {
	case dvfs_mif:
		grp = asv_tbl.mif_asv_group + asv_tbl.mif_modified_group;
		break;
	case dvfs_int:
	case dvfs_cam:
	case dvfs_disp:
		grp = asv_tbl.int_asv_group + asv_tbl.int_modified_group;
		break;
	case dvfs_cpucl0:
		grp = asv_tbl.big_asv_group + asv_tbl.big_modified_group;
		break;
	case dvfs_cpucl1:
		grp = asv_tbl.little_asv_group + asv_tbl.little_modified_group;
		break;
	case dvfs_g3d:
		grp = asv_tbl.g3d_asv_group + asv_tbl.g3d_modified_group;
		break;
	case dvfs_fsys:
		grp = asv_tbl.fsys_asv_group + asv_tbl.fsys_modified_group;
		break;
	case dvs_cp:
		grp = asv_tbl.cp_asv_group + asv_tbl.cp_modified_group;
		break;
	default:
		pr_info("Un-support asv grp %d\n", id);
	}

	return grp;
}

int asv_get_ids_info(unsigned int id)
{
	int ids = 0;

	switch (id) {
	case dvfs_cpucl0:
		ids = id_tbl.ids_cpucl0;
		break;
	case dvfs_g3d:
		ids = id_tbl.ids_g3d;
		break;
	case dvfs_mif:
	case dvfs_cpucl1:
	case dvfs_int:
	case dvfs_cam:
	case dvfs_disp:
	case dvfs_aud:
	case dvs_cp:
	case dvfs_fsys:
		ids = id_tbl.ids_int;
		break;
	default:
		pr_info("Un-support ids info %d\n", id);
	}

	return ids;
}

int asv_get_table_ver(void)
{
	return asv_tbl.asv_table_ver;
}

int id_get_rev(void)
{
	return id_tbl.main_rev;
}

int id_get_product_line(void)
{
	return id_tbl.product_line;
}

int id_get_asb_ver(void)
{
	return id_tbl.asb_version;
}

int asv_table_init(void)
{
	int i;
	unsigned int *p_table;
	unsigned int *regs;
	unsigned long tmp;

	p_table = (unsigned int *)&asv_tbl;

	for (i = 0; i < ASV_INFO_ADDR_CNT; i++) {
		exynos_smc_readsfr((unsigned long)(ASV_TABLE_BASE + 0x4 * i), &tmp);
		*(p_table + i) = (unsigned int)tmp;
	}

	p_table = (unsigned int *)&id_tbl;

	regs = (unsigned int *)ioremap(ID_TABLE_BASE, ID_INFO_ADDR_CNT * sizeof(int));
	for (i = 0; i < ID_INFO_ADDR_CNT; i++)
		*(p_table + i) = (unsigned int)regs[i];

#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
	pr_info("asv_table_version : %d\n", asv_tbl.asv_table_ver);
	pr_info("  big grp : %d\n", asv_tbl.big_asv_group);
	pr_info("  little grp : %d\n", asv_tbl.little_asv_group);
	pr_info("  g3d grp : %d\n", asv_tbl.g3d_asv_group);
	pr_info("  mif grp : %d\n", asv_tbl.mif_asv_group);
	pr_info("  int grp : %d\n", asv_tbl.int_asv_group);
	pr_info("  fsys grp : %d\n", asv_tbl.fsys_asv_group);
	pr_info("  cp grp : %d\n", asv_tbl.cp_asv_group);
	pr_info("  product_line : %d\n", id_tbl.product_line);
	pr_info("  asb_version : %d\n", id_tbl.asb_version);
#endif

	return asv_tbl.asv_table_ver;
}
#endif
