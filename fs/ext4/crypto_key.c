/*
 * linux/fs/ext4/crypto_key.c
 *
 * Copyright (C) 2015, Google, Inc.
 *
 * This contains encryption key functions for ext4
 *
 * Written by Michael Halcrow, Ildar Muslukhov, and Uday Savagaonkar, 2015.
 */

#include <keys/encrypted-type.h>
#include <keys/user-type.h>
#include <linux/random.h>
#include <linux/scatterlist.h>
#include <uapi/linux/keyctl.h>

#include <crypto/fmp.h>

#include "ext4.h"
#include "xattr.h"

#ifdef CONFIG_EXT4CRYPT_SDP
#include "sdp/fscrypto_sdp_dek_private.h"
#endif

#ifdef CONFIG_SDP_ENHANCED
#ifdef CONFIG_EXT4CRYPT_SDP
static int derive_fek(struct inode *inode,
		const struct ext4_encryption_context *ctx,
		struct ext4_crypt_info *crypt_info,
		u8 *fek, u32 fek_len);
#endif
#endif

static void derive_crypt_complete(struct crypto_async_request *req, int rc)
{
	struct ext4_completion_result *ecr = req->data;

	if (rc == -EINPROGRESS)
		return;

	ecr->res = rc;
	complete(&ecr->completion);
}

/**
 * ext4_derive_key_v1() - Derive a key using AES-128-ECB
 * @deriving_key: Encryption key used for derivation.
 * @source_key:   Source key to which to apply derivation.
 * @derived_key:  Derived key.
 *
 * Return: 0 on success, -errno on failure
 */
static int ext4_derive_key_v1(const char deriving_key[EXT4_AES_128_ECB_KEY_SIZE],
			      const char source_key[EXT4_AES_256_XTS_KEY_SIZE],
			      char derived_key[EXT4_AES_256_XTS_KEY_SIZE])
{
	int res = 0;
	struct ablkcipher_request *req = NULL;
	DECLARE_EXT4_COMPLETION_RESULT(ecr);
	struct scatterlist src_sg, dst_sg;
	struct crypto_ablkcipher *tfm = crypto_alloc_ablkcipher("ecb(aes)", 0,
								0);

	if (IS_ERR(tfm)) {
		res = PTR_ERR(tfm);
		tfm = NULL;
		goto out;
	}
	crypto_ablkcipher_set_flags(tfm, CRYPTO_TFM_REQ_WEAK_KEY);
	req = ablkcipher_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		res = -ENOMEM;
		goto out;
	}
	ablkcipher_request_set_callback(req,
			CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
			derive_crypt_complete, &ecr);
	res = crypto_ablkcipher_setkey(tfm, deriving_key,
				       EXT4_AES_128_ECB_KEY_SIZE);
	if (res < 0)
		goto out;
	sg_init_one(&src_sg, source_key, EXT4_AES_256_XTS_KEY_SIZE);
	sg_init_one(&dst_sg, derived_key, EXT4_AES_256_XTS_KEY_SIZE);
	ablkcipher_request_set_crypt(req, &src_sg, &dst_sg,
				     EXT4_AES_256_XTS_KEY_SIZE, NULL);
	res = crypto_ablkcipher_encrypt(req);
	if (res == -EINPROGRESS || res == -EBUSY) {
		wait_for_completion(&ecr.completion);
		res = ecr.res;
	}

out:
	if (req)
		ablkcipher_request_free(req);
	if (tfm)
		crypto_free_ablkcipher(tfm);
	return res;
}

/**
 * ext4_derive_key_v2() - Derive a key non-reversibly
 * @nonce: the nonce associated with the file
 * @master_key: the master key referenced by the file
 * @derived_key: (output) the resulting derived key
 *
 * This function computes the following:
 *	 derived_key[0:127]   = AES-256-ENCRYPT(master_key[0:255], nonce)
 *	 derived_key[128:255] = AES-256-ENCRYPT(master_key[0:255], nonce ^ 0x01)
 *	 derived_key[256:383] = AES-256-ENCRYPT(master_key[256:511], nonce)
 *	 derived_key[384:511] = AES-256-ENCRYPT(master_key[256:511], nonce ^ 0x01)
 *
 * 'nonce ^ 0x01' denotes flipping the low order bit of the last byte.
 *
 * Unlike the v1 algorithm, the v2 algorithm is "non-reversible", meaning that
 * compromising a derived key does not also compromise the master key.
 *
 * Return: 0 on success, -errno on failure
 */
static int ext4_derive_key_v2(const char nonce[EXT4_KEY_DERIVATION_NONCE_SIZE],
			      const char master_key[EXT4_MAX_KEY_SIZE],
			      char derived_key[EXT4_MAX_KEY_SIZE])
{
	const int noncelen = EXT4_KEY_DERIVATION_NONCE_SIZE;
	struct crypto_cipher *tfm;
	int err;
	int i;

	/*
	 * Since we only use each transform for a small number of encryptions,
	 * requesting just "aes" turns out to be significantly faster than
	 * "ecb(aes)", by about a factor of two.
	 */
	tfm = crypto_alloc_cipher("aes", 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);

	BUILD_BUG_ON(4 * EXT4_KEY_DERIVATION_NONCE_SIZE != EXT4_MAX_KEY_SIZE);
	BUILD_BUG_ON(2 * EXT4_AES_256_ECB_KEY_SIZE != EXT4_MAX_KEY_SIZE);
	for (i = 0; i < 2; i++) {
		memcpy(derived_key, nonce, noncelen);
		memcpy(derived_key + noncelen, nonce, noncelen);
		derived_key[2 * noncelen - 1] ^= 0x01;
		err = crypto_cipher_setkey(tfm, master_key,
					   EXT4_AES_256_ECB_KEY_SIZE);
		if (err)
			break;
		crypto_cipher_encrypt_one(tfm, derived_key, derived_key);
		crypto_cipher_encrypt_one(tfm, derived_key + noncelen,
					  derived_key + noncelen);
		master_key += EXT4_AES_256_ECB_KEY_SIZE;
		derived_key += 2 * noncelen;
	}
	crypto_free_cipher(tfm);
	return err;
}

/**
 * ext4_derive_key() - Derive a per-file key from a nonce and master key
 * @ctx: the encryption context associated with the file
 * @master_key: the master key referenced by the file
 * @derived_key: (output) the resulting derived key
 *
 * Return: 0 on success, -errno on failure
 */
static int ext4_derive_key(const struct ext4_encryption_context *ctx,
			   const char master_key[EXT4_MAX_KEY_SIZE],
			   char derived_key[EXT4_MAX_KEY_SIZE])
{
	BUILD_BUG_ON(EXT4_AES_128_ECB_KEY_SIZE != EXT4_KEY_DERIVATION_NONCE_SIZE);
	BUILD_BUG_ON(EXT4_AES_256_XTS_KEY_SIZE != EXT4_MAX_KEY_SIZE);

	/*
	 * Although the key derivation algorithm is logically independent of the
	 * choice of encryption modes, in this kernel it is bundled with HEH
	 * encryption of filenames, which is another crypto improvement that
	 * requires an on-disk format change and requires userspace to specify
	 * different encryption policies.
	 */
	if (ctx->filenames_encryption_mode == EXT4_ENCRYPTION_MODE_AES_256_HEH)
		return ext4_derive_key_v2(ctx->nonce, master_key, derived_key);
	else
		return ext4_derive_key_v1(ctx->nonce, master_key, derived_key);
}

void ext4_free_crypt_info(struct ext4_crypt_info *ci)
{
	if (!ci)
		return;
#ifdef CONFIG_EXT4CRYPT_SDP
	fscrypt_sdp_put_sdp_info(ci->ci_sdp_info);
#endif

//	if (ci->ci_keyring_key)
//		key_put(ci->ci_keyring_key);
	if (!ci->private_enc_mode)
		crypto_free_ablkcipher(ci->ci_ctfm);
	kmem_cache_free(ext4_crypt_info_cachep, ci);
}

void ext4_free_encryption_info(struct inode *inode,
			       struct ext4_crypt_info *ci)
{
	struct ext4_inode_info *ei = EXT4_I(inode);
	struct ext4_crypt_info *prev;

	if (ci == NULL)
		ci = ACCESS_ONCE(ei->i_crypt_info);
	if (ci == NULL)
		return;
	prev = cmpxchg(&ei->i_crypt_info, ci, NULL);
	if (prev != ci)
		return;

#ifdef CONFIG_EXT4CRYPT_SDP
	fscrypt_sdp_cache_remove_inode_num(inode);
#endif
	ext4_free_crypt_info(ci);
}

int ext4_get_encryption_info(struct inode *inode)
{
	struct ext4_inode_info *ei = EXT4_I(inode);
	struct ext4_crypt_info *crypt_info;
	char full_key_descriptor[EXT4_KEY_DESC_PREFIX_SIZE +
				 (EXT4_KEY_DESCRIPTOR_SIZE * 2) + 1];
	struct key *keyring_key = NULL;
	struct ext4_encryption_key *master_key;
	struct ext4_encryption_context ctx;
	const struct user_key_payload *ukp;
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	struct crypto_ablkcipher *ctfm;
	const char *cipher_str;
	char raw_key[EXT4_MAX_KEY_SIZE];
	char mode;
	int res;

	if (ei->i_crypt_info)
		return 0;

	res = ext4_init_crypto();
	if (res)
		return res;

	res = ext4_xattr_get(inode, EXT4_XATTR_INDEX_ENCRYPTION,
				 EXT4_XATTR_NAME_ENCRYPTION_CONTEXT,
				 &ctx, sizeof(ctx));
	if (res < 0) {
		if (!DUMMY_ENCRYPTION_ENABLED(sbi))
			return res;
		ctx.contents_encryption_mode = EXT4_ENCRYPTION_MODE_AES_256_XTS;
		ctx.filenames_encryption_mode =
			EXT4_ENCRYPTION_MODE_AES_256_CTS;
		ctx.flags = 0;
	} else if (res != sizeof(ctx))
		return -EINVAL;
	res = 0;

	crypt_info = kmem_cache_alloc(ext4_crypt_info_cachep, GFP_NOFS);
	if (!crypt_info)
		return -ENOMEM;

#ifdef CONFIG_EXT4_PRIVATE_ENCRYPTION
	crypt_info->ci_flags = ctx.flags & EXT4_POLICY_FLAGS_PAD_MASK;
#else
	crypt_info->ci_flags = ctx.flags;
#endif
	crypt_info->ci_data_mode = ctx.contents_encryption_mode;
	crypt_info->ci_filename_mode = ctx.filenames_encryption_mode;
#ifdef CONFIG_EXT4_PRIVATE_ENCRYPTION
	if (ctx.filenames_encryption_mode == EXT4_PRIVATE_ENCRYPTION_MODE_AES_256_XTS ||
			ctx.filenames_encryption_mode == EXT4_PRIVATE_ENCRYPTION_MODE_AES_256_CBC ||
			ctx.filenames_encryption_mode == EXT4_ENCRYPTION_MODE_PRIVATE) {
		printk(KERN_WARNING "Private encryption doesn't support filename encryption mode. \
				Forcely, change it to AES_256_CTS mode\n");
		ctx.filenames_encryption_mode = EXT4_ENCRYPTION_MODE_AES_256_CTS;
	}
#endif /* CONFIG_EXT4_PRIVATE_ENCRYPTION */
	crypt_info->ci_ctfm = NULL;
	memcpy(crypt_info->ci_master_key, ctx.master_key_descriptor,
	       sizeof(crypt_info->ci_master_key));
#ifdef CONFIG_EXT4CRYPT_SDP
	crypt_info->ci_sdp_info = NULL;
#endif

	if (S_ISREG(inode->i_mode))
		mode = crypt_info->ci_data_mode;
	else if (S_ISDIR(inode->i_mode) || S_ISLNK(inode->i_mode))
		mode = crypt_info->ci_filename_mode;
	else
		BUG();
	switch (mode) {
	case EXT4_ENCRYPTION_MODE_AES_256_XTS:
		cipher_str = "xts(aes)";
		break;
	case EXT4_ENCRYPTION_MODE_AES_256_CTS:
		cipher_str = "cts(cbc(aes))";
		break;
#ifdef CONFIG_EXT4_PRIVATE_ENCRYPTION
	case EXT4_PRIVATE_ENCRYPTION_MODE_AES_256_XTS:
		cipher_str = "xts(aes)";
		if (ctx.flags & EXT4_POLICY_FLAGS_PRIVATE_ALGO)
			inode->i_mapping->private_algo_mode = EXYNOS_FMP_ALGO_MODE_AES_XTS;
		break;
	case EXT4_PRIVATE_ENCRYPTION_MODE_AES_256_CBC:
		cipher_str = "cbc(aes)";
		if (ctx.flags & EXT4_POLICY_FLAGS_PRIVATE_ALGO)
			inode->i_mapping->private_algo_mode = EXYNOS_FMP_ALGO_MODE_AES_CBC;
		break;
	case EXT4_ENCRYPTION_MODE_PRIVATE:
		cipher_str = "xts(aes)";
		if (ctx.flags & EXT4_POLICY_FLAGS_PRIVATE_ALGO)
			inode->i_mapping->private_algo_mode = EXYNOS_FMP_ALGO_MODE_AES_XTS;
		break;
#endif /* CONFIG_EXT4_PRIVATE_ENCRYPTION */
	case EXT4_ENCRYPTION_MODE_AES_256_HEH:
		cipher_str = "heh(aes)";
		break;
	default:
		printk_once(KERN_WARNING
			    "ext4: unsupported key mode %d (ino %u)\n",
			    mode, (unsigned) inode->i_ino);
		res = -ENOKEY;
		goto out;
	}
	if (DUMMY_ENCRYPTION_ENABLED(sbi)) {
		memset(raw_key, 0x42, EXT4_AES_256_XTS_KEY_SIZE);
		goto got_key;
	}
	memcpy(full_key_descriptor, EXT4_KEY_DESC_PREFIX,
	       EXT4_KEY_DESC_PREFIX_SIZE);
	sprintf(full_key_descriptor + EXT4_KEY_DESC_PREFIX_SIZE,
		"%*phN", EXT4_KEY_DESCRIPTOR_SIZE,
		ctx.master_key_descriptor);
	full_key_descriptor[EXT4_KEY_DESC_PREFIX_SIZE +
			    (2 * EXT4_KEY_DESCRIPTOR_SIZE)] = '\0';
	keyring_key = request_key(&key_type_logon, full_key_descriptor, NULL);
	if (IS_ERR(keyring_key)) {
		res = PTR_ERR(keyring_key);
		keyring_key = NULL;
		goto out;
	}
	if (keyring_key->type != &key_type_logon) {
		printk_once(KERN_WARNING
			    "ext4: key type must be logon\n");
		res = -ENOKEY;
		goto out;
	}
	down_read(&keyring_key->sem);
	ukp = user_key_payload(keyring_key);
	if (!ukp) {
		/* key was revoked before we acquired its semaphore */
		res = -EKEYREVOKED;
		up_read(&keyring_key->sem);
		goto out;
	}
	if (ukp->datalen != sizeof(struct ext4_encryption_key)) {
		res = -EINVAL;
		up_read(&keyring_key->sem);
		goto out;
	}
	master_key = (struct ext4_encryption_key *)ukp->data;
	BUILD_BUG_ON(EXT4_AES_128_ECB_KEY_SIZE !=
		     EXT4_KEY_DERIVATION_NONCE_SIZE);
	if (master_key->size != EXT4_AES_256_XTS_KEY_SIZE) {
		printk_once(KERN_WARNING
			    "ext4: key size incorrect: %d\n",
			    master_key->size);
		res = -ENOKEY;
		up_read(&keyring_key->sem);
		goto out;
	}
#ifdef CONFIG_EXT4CRYPT_SDP
	if ((FSCRYPT_SDP_PARSE_FLAG_SDP_ONLY(ctx.knox_flags) & FSCRYPT_KNOX_FLG_SDP_MASK)) {
		crypt_info->ci_sdp_info = fscrypt_sdp_alloc_sdp_info();
		if (!crypt_info->ci_sdp_info) {
			res = -ENOMEM;
			goto out;
		}
#ifndef CONFIG_SDP_ENHANCED
		crypt_info->ci_sdp_info->sdp_flags = FSCRYPT_SDP_PARSE_FLAG_SDP_ONLY(ctx.knox_flags);

		res = fscrypt_sdp_get_key_if_sensitive(inode, crypt_info, ctx.nonce);
#else
		res = fscrypt_sdp_update_sdp_info(inode, &ctx, crypt_info);
#endif
		if (res)
			goto out;

#ifdef CONFIG_SDP_ENHANCED
		if (fscrypt_sdp_is_classified(crypt_info)) {
			res = derive_fek(inode, &ctx, crypt_info, raw_key, ext4_encryption_key_size(mode));
			if (res)
				goto out;
			fscrypt_sdp_update_conv_status(crypt_info);
			goto sdp_dek;
		}
#endif
	}
#endif
	res = ext4_derive_key(&ctx, master_key->raw, raw_key);
	up_read(&keyring_key->sem);
	if (res)
		goto out;

#ifdef CONFIG_SDP_ENHANCED
#ifdef CONFIG_EXT4CRYPT_SDP
sdp_dek:
#endif
#endif

got_key:
	memset(crypt_info->raw_key, 0, EXT4_MAX_KEY_SIZE);

#ifdef CONFIG_EXT4_PRIVATE_ENCRYPTION
	/* hack to support fbe on gsi */
	if (S_ISREG(inode->i_mode) && (crypt_info->ci_data_mode == EXT4_ENCRYPTION_MODE_AES_256_XTS))
		goto private_crypt;
#endif

	if (mode == EXT4_PRIVATE_ENCRYPTION_MODE_AES_256_XTS ||
			mode == EXT4_PRIVATE_ENCRYPTION_MODE_AES_256_CBC ||
			mode == EXT4_ENCRYPTION_MODE_PRIVATE) {
#ifdef CONFIG_EXT4_PRIVATE_ENCRYPTION
private_crypt:
#endif
		crypt_info->private_enc_mode = EXYNOS_FMP_FILE_ENC;
		memcpy(crypt_info->raw_key, raw_key, ext4_encryption_key_size(mode));
		memcpy(inode->i_mapping->key, crypt_info->raw_key, ext4_encryption_key_size(mode));
		inode->i_mapping->key_length = ext4_encryption_key_size(mode);
	} else {
		crypt_info->private_enc_mode = 0;
		inode->i_mapping->private_algo_mode = EXYNOS_FMP_BYPASS_MODE;
		ctfm = crypto_alloc_ablkcipher(cipher_str, 0, 0);
		if (!ctfm || IS_ERR(ctfm)) {
			res = ctfm ? PTR_ERR(ctfm) : -ENOMEM;
			printk(KERN_DEBUG
			       "%s: error %d (inode %u) allocating crypto tfm\n",
			       __func__, res, (unsigned) inode->i_ino);
			goto out;
		}
		crypt_info->ci_ctfm = ctfm;
		crypto_ablkcipher_clear_flags(ctfm, ~0);
		crypto_tfm_set_flags(crypto_ablkcipher_tfm(ctfm),
				     CRYPTO_TFM_REQ_WEAK_KEY);
		res = crypto_ablkcipher_setkey(ctfm, raw_key,
					       ext4_encryption_key_size(mode));
		if (res)
			goto out;
	}
	inode->i_mapping->private_enc_mode = crypt_info->private_enc_mode;
	if (cmpxchg(&ei->i_crypt_info, NULL, crypt_info) == NULL)
		crypt_info = NULL;
#ifdef CONFIG_EXT4CRYPT_SDP
	if (crypt_info == NULL) { //Call only when i_crypt_info is loaded initially
#ifdef CONFIG_SDP_ENHANCED
		fscrypt_sdp_finalize_tasks(inode, raw_key, (res ? res : ext4_encryption_key_size(mode)));
#else
		fscrypt_sdp_finalize_tasks(inode);
#endif
	}
#endif
out:
	if (res == -ENOKEY)
		res = 0;
	key_put(keyring_key);
	ext4_free_crypt_info(crypt_info);
	memzero_explicit(raw_key, sizeof(raw_key));
	return res;
}

int ext4_has_encryption_key(struct inode *inode)
{
	struct ext4_inode_info *ei = EXT4_I(inode);

	return (ei->i_crypt_info != NULL);
}

#ifdef CONFIG_SDP_ENHANCED
#ifdef CONFIG_EXT4CRYPT_SDP
/* The function is only for regular files */
static int derive_fek(struct inode *inode,
						const struct ext4_encryption_context *ctx,
						struct ext4_crypt_info *crypt_info,
						u8 *fek, u32 fek_len)
{
	int res = 0;
	/*
	 * 1. [ Native / Uninitialized / To_sensitive ]  --> Plain fek
	 * 2. [ Native / Uninitialized / Non_sensitive ] --> Plain fek
	 */
	if (fscrypt_sdp_is_uninitialized(crypt_info))
	{
		res = fscrypt_sdp_derive_uninitialized_dek(crypt_info, fek, fek_len);
	}
	/*
	 * 3. [ Native / Initialized / Sensitive ]     --> { fek }_SDPK
	 * 4. [ Non_native / Initialized / Sensitive ] --> { fek }_SDPK
	 */
	else if (fscrypt_sdp_is_sensitive(crypt_info))
	{
		res = fscrypt_sdp_derive_dek(crypt_info, fek, fek_len);
	}
	/*
	 * 5. [ Native / Initialized / Non_sensitive ] --> { fek }_cekey
	 */
	else if (fscrypt_sdp_is_native(crypt_info))
	{
		res = fscrypt_sdp_derive_fek(inode, crypt_info, fek, fek_len);
	}
	/*
	 * else { N/A }
	 *
	 * Not classified file.
	 * 6. [ Non_native / Initialized / Non_sensitive ]
	 * 7. [ Non_native / Initialized / To_sensitive ]
	 */
	return res;
}

int ext4_get_encryption_key(struct inode *inode, struct ext4_encryption_key *key)
{
	struct ext4_inode_info *ei = EXT4_I(inode);
	struct ext4_crypt_info *crypt_info;
	struct ext4_encryption_context ctx;
	char mode;
	int res;

	if (!ei->i_crypt_info)
		return -EINVAL;
	crypt_info = ei->i_crypt_info;

//	res = ext4_init_crypto();
//	if (res)
//		return res;

	res = ext4_xattr_get(inode, EXT4_XATTR_INDEX_ENCRYPTION,
			EXT4_XATTR_NAME_ENCRYPTION_CONTEXT,
			&ctx, sizeof(ctx));
	if (res < 0) {
		return res;
	} else if (res != sizeof(ctx))
		return -EINVAL;
	res = 0;

	if (S_ISREG(inode->i_mode))
		mode = crypt_info->ci_data_mode;
	else if (S_ISDIR(inode->i_mode) || S_ISLNK(inode->i_mode))
		mode = crypt_info->ci_filename_mode;
	else
		BUG();

	key->mode = mode;
	key->size = ext4_encryption_key_size(mode);
	memcpy(key->raw, crypt_info->raw_key, key->size);
	return res;
}

int fscrypt_get_encryption_kek(struct inode *inode,
							struct ext4_crypt_info *crypt_info,
							struct ext4_encryption_key *kek)
{
	char full_key_descriptor[EXT4_KEY_DESC_PREFIX_SIZE +
				 (EXT4_KEY_DESCRIPTOR_SIZE * 2) + 1];
	struct key *keyring_key = NULL;
	struct ext4_encryption_key *master_key;
	const struct user_key_payload *ukp;
	int res;

	if (!crypt_info)
		return -EINVAL;

	res = 0;
	memcpy(full_key_descriptor, EXT4_KEY_DESC_PREFIX,
		   EXT4_KEY_DESC_PREFIX_SIZE);
	sprintf(full_key_descriptor + EXT4_KEY_DESC_PREFIX_SIZE,
		"%*phN", EXT4_KEY_DESCRIPTOR_SIZE,
		crypt_info->ci_master_key);
	full_key_descriptor[EXT4_KEY_DESC_PREFIX_SIZE +
				(2 * EXT4_KEY_DESCRIPTOR_SIZE)] = '\0';
	keyring_key = request_key(&key_type_logon, full_key_descriptor, NULL);
	if (IS_ERR(keyring_key)) {
		res = PTR_ERR(keyring_key);
		keyring_key = NULL;
		goto out;
	}
	if (keyring_key->type != &key_type_logon) {
		printk_once(KERN_WARNING
				"ext4: key type must be logon\n");
		res = -ENOKEY;
		goto out;
	}
	down_read(&keyring_key->sem);
	ukp = user_key_payload(keyring_key);
	if (!ukp) {
		/* key was revoked before we acquired its semaphore */
		res = -EKEYREVOKED;
		up_read(&keyring_key->sem);
		goto out;
	}
	if (ukp->datalen != sizeof(struct ext4_encryption_key)) {
		res = -EINVAL;
		up_read(&keyring_key->sem);
		goto out;
	}
	master_key = (struct ext4_encryption_key *)ukp->data;
	BUILD_BUG_ON(EXT4_AES_128_ECB_KEY_SIZE !=
			 EXT4_KEY_DERIVATION_NONCE_SIZE);
	if (master_key->size != EXT4_AES_256_XTS_KEY_SIZE) {
		printk_once(KERN_WARNING
				"ext4: key size incorrect: %d\n",
				master_key->size);
		res = -ENOKEY;
		up_read(&keyring_key->sem);
		goto out;
	}
	memcpy(kek, master_key, sizeof(struct ext4_encryption_key));
	up_read(&keyring_key->sem);

out:
	key_put(keyring_key);
	return res;
}
EXPORT_SYMBOL(fscrypt_get_encryption_kek);
#endif
#endif
