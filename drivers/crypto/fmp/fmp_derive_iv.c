/*
 * Exynos FMP derive iv for eCryptfs
 *
 * Copyright (C) 2014 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <crypto/fmp.h>

#include "sha256.h"

#define FMP_MAX_IV_BYTES	16
#define FMP_MAX_OFFSET_BYTES	16
#define DEFAULT_HASH		"md5"
#define SHA256_HASH		"sha256"

#define SHA256_HASH_SIZE	32
#define MD5_DIGEST_SIZE		16

static DEFINE_SPINLOCK(fmp_tfm_lock);

#ifdef CONFIG_CRYPTO_FIPS
static int calculate_sha256(struct crypto_hash *hash_tfm, char *dst, char *src, int len)
{
	if ((src == NULL) || (dst == NULL))
		return -EINVAL;

	return sha256(src, len, dst);
}
#endif

static int calculate_md5(struct exynos_fmp *fmp,
		struct crypto_hash *hash_tfm, char *dst, char *src, int len)
{
	int ret = -1;
	unsigned long flag;
	struct scatterlist sg;
	struct hash_desc desc = {
		.tfm = hash_tfm,
		.flags = 0
	};

	spin_lock_irqsave(&fmp_tfm_lock, flag);
	sg_init_one(&sg, (u8 *)src, len);

	if (!desc.tfm) {
		desc.tfm = crypto_alloc_hash(DEFAULT_HASH, 0,
					     CRYPTO_ALG_ASYNC);
		if (IS_ERR(desc.tfm)) {
			dev_err(fmp->dev, "%s: Fail to allocate crypto context\n",
					__func__);
			goto out;
		}
		hash_tfm = desc.tfm;
	}

	ret = crypto_hash_init(&desc);
	if (ret) {
		dev_err(fmp->dev, "%s: Error initializing crypto hash(%d)\n",
				__func__, ret);
		goto out_free_hash;
	}
	ret = crypto_hash_update(&desc, &sg, len);
	if (ret) {
		dev_err(fmp->dev, "%s: Error updating crypto hash(%d)\n",
				__func__, ret);
		goto out_free_hash;
	}
	ret = crypto_hash_final(&desc, dst);
	if (ret) {
		dev_err(fmp->dev, "%s: Error finalizing crypto hash(%d)\n",
				__func__, ret);
		goto out_free_hash;
	}

out_free_hash:
	crypto_free_hash(desc.tfm);
out:
	spin_unlock_irqrestore(&fmp_tfm_lock, flag);
	return ret;
}

static int derive_iv(struct exynos_fmp *fmp,
			struct address_space *mapping,
			loff_t offset,
			char *extent_iv)
{
	char src[FMP_MAX_IV_BYTES + FMP_MAX_OFFSET_BYTES];
	int ret;

	if (!fmp || !fmp->dev) {
		pr_err("%s: Invalid fmp device\n", __func__);
		ret = -ENODEV;
		goto out;
	}

	memcpy(src, mapping->iv, FMP_MAX_IV_BYTES);
	memset(src + FMP_MAX_IV_BYTES, 0, FMP_MAX_OFFSET_BYTES);
	snprintf(src + FMP_MAX_IV_BYTES, FMP_MAX_OFFSET_BYTES, "%lld", offset);

#ifdef CONFIG_CRYPTO_FIPS
	if (mapping->cc_enable)
		ret = calculate_sha256(mapping->hash_tfm, extent_iv, src,
				FMP_MAX_IV_BYTES + FMP_MAX_OFFSET_BYTES);
	else
#endif
		ret = calculate_md5(fmp, mapping->hash_tfm, extent_iv, src,
				FMP_MAX_IV_BYTES + FMP_MAX_OFFSET_BYTES);
	if (ret) {
		dev_err(fmp->dev, "%s: Error attempting to compute generating IV(%d)\n",
				__func__, ret);
		goto out;
	}

out:
	return ret;
}

int fmplib_derive_iv(struct exynos_fmp *fmp,
			struct address_space *mapping,
			struct fmp_crypto_setting *crypto,
			enum fmp_crypto_enc_mode enc_mode)
{
	int ret = 0;
#ifdef CONFIG_CRYPTO_FIPS
	char extent_iv[SHA256_HASH_SIZE];
#else
	char extent_iv[MD5_DIGEST_SIZE];
#endif
	uint32_t extent_sector = crypto->sector;

	memset(crypto->iv, 0, FMP_IV_SIZE_16);
	if (crypto->algo_mode == EXYNOS_FMP_ALGO_MODE_AES_XTS) {
		memcpy(crypto->iv, &extent_sector, sizeof(uint32_t));
	} else if (crypto->algo_mode == EXYNOS_FMP_ALGO_MODE_AES_CBC) {
		ret = derive_iv(fmp, mapping, crypto->index, extent_iv);
		if (ret) {
			dev_err(fmp->dev, "%s: Fail to derive IV (%d)\n",
					__func__, ret);
			return ret;
		}
		memcpy(crypto->iv, extent_iv, sizeof(extent_iv));
	} else {
		dev_err(fmp->dev, "%s: Invalid FMP algo mode (%d)\n",
				__func__, crypto->algo_mode);
		ret = -EINVAL;
		goto err;
	}
err:
	return ret;
}
