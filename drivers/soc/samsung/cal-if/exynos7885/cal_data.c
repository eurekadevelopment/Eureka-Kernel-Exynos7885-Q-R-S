#include "../pmucal_common.h"
#include "../pmucal_cpu.h"
#include "../pmucal_local.h"
#include "../pmucal_rae.h"
#include "../pmucal_system.h"

#include "pmucal_cpu_exynos7885.h"
#include "pmucal_local_exynos7885.h"
#include "pmucal_p2vmap_exynos7885.h"
#include "pmucal_system_exynos7885.h"

#include "../pmucal_cp.h"
#include "pmucal_cp_exynos7885.h"

#include "../pmucal_gnss.h"
#include "pmucal_gnss_exynos7885.h"

#include "cmucal-node.c"
#include "cmucal-qch.c"
#include "cmucal-sfr.c"
#include "cmucal-vclk.c"
#include "cmucal-vclklut.c"

#include "clkout_exynos7885.c"
#include "acpm_dvfs_exynos7885.h"
#include "asv_exynos7885.h"

void (*cal_data_init)(void) = NULL;

#if defined(CONFIG_SEC_FACTORY) || defined(CONFIG_SEC_DEBUG)
enum ids_info {
	tg,
	lg,
	bg,
	g3dg,
	mifg,
	bids,
	gids,
};

int asv_ids_information(enum ids_info id) {
	int res;

	switch (id) {
	case tg:
		res = asv_get_table_ver();
		break;
	case lg:
		res = asv_get_grp(dvfs_cpucl1);
		break;
	case bg:
		res = asv_get_grp(dvfs_cpucl0);
		break;
	case g3dg:
		res = asv_get_grp(dvfs_g3d);
		break;
	case mifg:
		res = asv_get_grp(dvfs_mif);
		break;
	case bids:
		res = asv_get_ids_info(dvfs_cpucl0);
		break;
	case gids:
		res = asv_get_ids_info(dvfs_g3d);
		break;
	default:
		res = 0;
		break;
	};

	return res;
}
#endif
