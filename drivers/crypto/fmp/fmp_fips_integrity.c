/*
 * Perform FIPS Integrity test on FMP driver
 *
 * At build time, hmac(sha256) of crypto code, avaiable in different ELF sections
 * of vmlinux file, is generated. vmlinux file is updated with built-time hmac
 * in a read-only data variable, so that it is available at run-time
 *
 * At run time, hmac(sha256) is again calculated using crypto bytes of a running
 * At run time, hmac-fmp(sha256-fmp) is again calculated using crypto bytes of a running
 * kernel.
 * Run time hmac is compared to built time hmac to verify the integrity.
 *
 *
 * Author : Rohit Kothari (r.kothari@samsung.com)
 * Date	  : 11 Feb 2014
 *
 * Copyright (c) 2014 Samsung Electronics
 *
 */

#include <linux/kallsyms.h>
#include <linux/err.h>
#include <linux/scatterlist.h>
#include <linux/smc.h>
#include "fmp_fips_integrity.h"
#include "hmac-sha256.h"
#include "fmp_fips_info.h" /* for FIPS_FMP_FUNC_TEST macro */
#undef FIPS_DEBUG_INTEGRITY

static const unsigned char *integrity_check_key = "The quick brown fox jumps over the lazy dog";

/*
 * This function return kernel offset.
 * If CONFIG_RELOCATABLE is set
 * Then the kernel will be placed into the random offset in memory
 * At the build time, we write self address of the "buildtime_address"
 * to the "buildtime_address" variable
 */
static __u64 get_kernel_offset(void)
{
	static __u64 runtime_address = (__u64) &fmp_buildtime_address;
	__u64 kernel_offset = abs(fmp_buildtime_address - runtime_address);
	return kernel_offset;
}


int do_fmp_integrity_check(void)
{
	int i, rows, err;
	unsigned long start_addr = 0;
	unsigned long end_addr = 0;
	unsigned char runtime_hmac[32];
	struct hmac_sha256_ctx ctx;
	const char *builtime_hmac = 0;
	unsigned int size = 0;
#ifdef FIPS_DEBUG_INTEGRITY
	unsigned int covered = 0;
	unsigned int num_addresses = 0;
#endif

	memset(runtime_hmac, 0x00, 32);

	err = hmac_sha256_init(&ctx, integrity_check_key, strlen(integrity_check_key));
	if (err) {
		pr_err("FIPS(%s): init_hash failed\n", __func__);
		return -1;
	}

	rows = (__u32) ARRAY_SIZE(integrity_fmp_addrs);

	for (i = 0; integrity_fmp_addrs[i].first && i < rows; i++) {

		start_addr = integrity_fmp_addrs[i].first + get_kernel_offset();
		end_addr   = integrity_fmp_addrs[i].last  + get_kernel_offset();

		/* Print addresses for HMAC calculation */
#ifdef FIPS_DEBUG_INTEGRITY
		pr_info("FIPS_DEBUG_INTEGRITY : first last %08llux %08llux\n",
			integrity_fmp_addrs[i].first,
			integrity_fmp_addrs[i].last);
		covered += (end_addr - start_addr);
#endif

		size = end_addr - start_addr;

		err = hmac_sha256_update(&ctx,
			(unsigned char *)start_addr, size);
		if (err) {
			pr_err("FIPS(%s): Error to update hash", __func__);
			return -1;
		}
	}

#if FIPS_FMP_FUNC_TEST == 5
	pr_info("FIPS(%s): Failing Integrity Test\n", __func__);
	err = hmac_sha256_update(&ctx, (unsigned char *)start_addr, 1);
#endif

/* Dump bytes for HMAC */
#ifdef FIPS_DEBUG_INTEGRITY
	num_addresses = i;
	for (i = 0; integrity_fmp_addrs[i].first && i < rows; i++) {
		start_addr = integrity_fmp_addrs[i].first + get_kernel_offset();
		end_addr   = integrity_fmp_addrs[i].last  + get_kernel_offset();
		size = end_addr - start_addr;
		print_hex_dump(KERN_INFO, "FIPS_DEBUG_INTEGRITY : bytes for HMAC = ",
			DUMP_PREFIX_NONE, 16, 1,
			(char *)start_addr, size, false);
	}
#endif
	err = hmac_sha256_final(&ctx, runtime_hmac);
	if (err) {
		pr_err("FIPS(%s): Error in finalize", __func__);
		hmac_sha256_ctx_cleanup(&ctx);
		return -1;
	}

	hmac_sha256_ctx_cleanup(&ctx);
	builtime_hmac = builtime_fmp_hmac;

#ifdef FIPS_DEBUG_INTEGRITY
	pr_info("FIPS_DEBUG_INTEGRITY : %d bytes are covered, Address fragments (%d)", covered, num_addresses);
	print_hex_dump(KERN_INFO, "FIPS FMP RUNTIME : runtime hmac  = ",
		DUMP_PREFIX_NONE, 16, 1, runtime_hmac, sizeof(runtime_hmac), false);
	print_hex_dump(KERN_INFO, "FIPS FMP RUNTIME : builtime_hmac = ",
		DUMP_PREFIX_NONE, 16, 1, builtime_hmac, sizeof(runtime_hmac), false);
#endif


	if (!memcmp(builtime_hmac, runtime_hmac, sizeof(runtime_hmac))) {
		pr_info("FIPS(%s): Integrity Check Passed", __func__);
		return 0;
	}

	pr_err("FIPS(%s): Integrity Check Failed", __func__);
	return -1;
}
EXPORT_SYMBOL_GPL(do_fmp_integrity_check);
