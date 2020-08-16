// SPDX-License-Identifier: GPL-2.0
/*
 *  linux/fs/ntfs3/dir.c
 *
 * Copyright (C) 2019-2020 Paragon Software GmbH, All rights reserved.
 *
 *  directory handling functions for ntfs-based filesystems
 *
 */
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/fs.h>
#include <linux/iversion.h>
#include <linux/nls.h>

#include "debug.h"
#include "ntfs.h"
#include "ntfs_fs.h"

/*
 * Convert little endian Unicode 16 to UTF-8.
 */
int uni_to_x8(ntfs_sb_info *sbi, const struct le_str *uni, u8 *buf, int buf_len)
{
	const __le16 *ip;
	u8 *op;
	struct nls_table *nls;
	int uni_len = uni->len;

	static_assert(sizeof(wchar_t) == sizeof(__le16));

	ip = uni->name;
	op = buf;
	nls = sbi->nls;

	while (uni_len--) {
		u16 ec;
		int charlen;

		if (buf_len < NLS_MAX_CHARSET_SIZE) {
			ntfs_warning(
				sbi->sb,
				"filename was truncated while converting.");
			break;
		}

		ec = le16_to_cpu(*ip++);
		charlen = nls->uni2char(ec, op, buf_len);

		if (charlen > 0) {
			op += charlen;
			buf_len -= charlen;
		} else {
			*op++ = ':';
			op = hex_byte_pack(op, ec >> 8);
			op = hex_byte_pack(op, ec);
			buf_len -= 5;
		}
	}

	*op = 0;
	return op - buf;
}

static inline u8 get_digit(u8 d)
{
	u8 x = (d)&0xf;

	return x <= 9 ? ('0' + x) : ('A' + x - 10);
}

/* Convert input string to unicode */
int x8_to_uni(ntfs_sb_info *sbi, const u8 *name, u32 name_len,
	      struct cpu_str *uni, u32 max_ulen, enum utf16_endian endian)
{
	int i, ret;
	int clen;
	const u8 *s, *end;
	u32 tail;
	u16 *uname = uni->name;
	struct nls_table *nls;

	static_assert(sizeof(wchar_t) == sizeof(u16));

	nls = sbi->nls;
	s = name;
	end = name + name_len;

	for (ret = 0; s < end; ret += 1, uname += 1, s += clen) {
		if (ret >= max_ulen)
			return -ENAMETOOLONG;
		tail = end - s;
		if (*s == ':') {
			u8 uc[2];

			if (tail < 5)
				return -EINVAL;

			if (hex2bin(uc, s + 1, 2) < 0)
				return -EINVAL;

			*uname = (uc[0] << 8) | uc[1];
			clen = 5;
			continue;
		}

		clen = nls->char2uni(s, tail, uname);
		if (clen > 0)
			continue;

		ntfs_warning(
			sbi->sb,
			"%s -> unicode failed: '%.*s', pos %d, chars %x %x %x",
			nls->charset, name_len, name, (int)(s - name), s[0],
			tail > 1 ? s[1] : 0, tail > 2 ? s[2] : 0);

		if (tail >= 3) {
			uname[0] = '%';
			uname[1] = get_digit(*s >> 4);
			uname[2] = get_digit(*s >> 0);

			uname += 2;
			ret += 2; // +1 will be added if for ( .... )
			clen = 1;
			continue;
		}

		ntfs_error(
			sbi->sb,
			"input name \"%s\" is too big to fit into 255 unicode symbols",
			name);
		return -ENAMETOOLONG;
	}

#ifdef __BIG_ENDIAN
	if (endian == UTF16_LITTLE_ENDIAN) {
		__le16 *uname = (__le16 *)uni->name;

		for (i = 0; i < ret; i++, uname++)
			*uname = cpu_to_le16(*name);
	}
#else
	if (endian == UTF16_BIG_ENDIAN) {
		__be16 *uname = (__be16 *)uni->name;

		for (i = 0; i < ret; i++, uname++)
			*uname = cpu_to_be16(*name);
	}
#endif

	uni->len = ret;
	return ret;
}

/* helper function */
struct inode *dir_search_u(struct inode *dir, const struct cpu_str *uni,
			   struct ntfs_fnd *fnd)
{
	int err = 0;
	struct super_block *sb = dir->i_sb;
	ntfs_sb_info *sbi = sb->s_fs_info;
	ntfs_inode *ni = ntfs_i(dir);
	NTFS_DE *e;
	int diff;
	struct inode *inode = NULL;
	struct ntfs_fnd *fnd_a = NULL;

	if (!fnd) {
		fnd_a = fnd_get(&ni->dir);
		if (!fnd_a) {
			err = -ENOMEM;
			goto out;
		}
		fnd = fnd_a;
	}

	err = indx_find(&ni->dir, ni, NULL, uni, 0, sbi, &diff, &e, fnd);

	if (err)
		goto out;

	if (diff) {
		err = -ENOENT;
		goto out;
	}

	inode = ntfs_iget5(sb, &e->ref, uni);
	if (!IS_ERR(inode) && is_bad_inode(inode)) {
		iput(inode);
		err = -EINVAL;
	}
out:
	fnd_put(fnd_a);

	return err == -ENOENT ? NULL : err ? ERR_PTR(err) : inode;
}

/* helper function */
struct inode *dir_search(struct inode *dir, const struct qstr *name,
			 struct ntfs_fnd *fnd)
{
	struct super_block *sb = dir->i_sb;
	ntfs_sb_info *sbi = sb->s_fs_info;
	int err;
	struct inode *inode;
	struct cpu_str *uni = __getname();
	const u8 *n = name->name;

	if (!uni)
		return ERR_PTR(-ENOMEM);

	err = x8_to_uni(sbi, n, name->len, uni, NTFS_NAME_LEN,
			UTF16_HOST_ENDIAN);

	inode = err < 0 ? ERR_PTR(err) : dir_search_u(dir, uni, fnd);

	__putname(uni);

	return inode;
}

static inline int ntfs_filldir(ntfs_sb_info *sbi, ntfs_inode *ni,
			       const NTFS_DE *e, u8 *name,
			       struct dir_context *ctx)
{
	const ATTR_FILE_NAME *fname;
	unsigned long ino;
	int name_len;
	u32 dt_type;

	fname = Add2Ptr(e, sizeof(NTFS_DE));

	if (fname->type == FILE_NAME_DOS)
		return 0;

	if (!mi_is_ref(&ni->mi, &fname->home))
		return 0;

	ino = ino_get(&e->ref);

	if (ino == MFT_REC_ROOT)
		return 0;

	/* Skip meta files ( unless option to show metafiles is set ) */
	if (!sbi->options.showmeta && ntfs_is_meta_file(sbi, ino))
		return 0;

	if (sbi->options.nohidden && (fname->dup.fa & FILE_ATTRIBUTE_HIDDEN))
		return 0;

	name_len = uni_to_x8(sbi, (struct le_str *)&fname->name_len, name,
			     PATH_MAX);
	if (name_len <= 0) {
		ntfs_warning(sbi->sb, "failed to convert name for inode %lx.",
			     ino);
		return 0;
	}

	dt_type = (fname->dup.fa & FILE_ATTRIBUTE_DIRECTORY) ? DT_DIR : DT_REG;

	return !dir_emit(ctx, (s8 *)name, name_len, ino, dt_type);
}

/*
 * ntfs_read_hdr
 *
 * helper function 'ntfs_readdir'
 */
static int ntfs_read_hdr(ntfs_sb_info *sbi, ntfs_inode *ni,
			 const INDEX_HDR *hdr, u64 vbo, u64 pos, u8 *name,
			 struct dir_context *ctx)
{
	int err;
	const NTFS_DE *e;
	u32 e_size;
	u32 end = le32_to_cpu(hdr->used);
	u32 off = le32_to_cpu(hdr->de_off);

next:
	if (off + sizeof(NTFS_DE) > end)
		return -1;

	e = Add2Ptr(hdr, off);
	e_size = le16_to_cpu(e->size);
	if (e_size < sizeof(NTFS_DE) || off + e_size > end)
		return -1;

	if (de_is_last(e))
		return 0;

	/* Skip already enumerated*/
	if (vbo + off < pos) {
		off += e_size;
		goto next;
	}

	if (le16_to_cpu(e->key_size) < SIZEOF_ATTRIBUTE_FILENAME)
		return -1;

	ctx->pos = vbo + off;

	/* Submit the name to the filldir callback. */
	err = ntfs_filldir(sbi, ni, e, name, ctx);
	if (err)
		return err;

	off += e_size;
	goto next;
}

/*
 * file_operations::iterate_shared
 *
 * Use non sorted enumeration.
 * We have an example of broken volume where sorted enumeration
 * counts each name twice
 */
static int ntfs_readdir(struct file *file, struct dir_context *ctx)
{
	const INDEX_ROOT *root;
	const INDEX_HDR *hdr;
	u64 vbo;
	size_t bit;
	loff_t eod;
	int err = 0;
	struct inode *dir = file_inode(file);
	ntfs_inode *ni = ntfs_i(dir);
	struct super_block *sb = dir->i_sb;
	ntfs_sb_info *sbi = sb->s_fs_info;
	loff_t i_size = dir->i_size;
	u32 pos = ctx->pos;
	u8 *name = NULL;
	struct indx_node *node = NULL;
	u8 index_bits = ni->dir.index_bits;

	/* name is a buffer of PATH_MAX length */
	static_assert(NTFS_NAME_LEN * 4 < PATH_MAX);

	if (ni->dir.changed) {
		ni->dir.changed = false;
		pos = 0;
	}

	eod = i_size + sbi->record_size;

	if (pos >= eod)
		return 0;

	if (!dir_emit_dots(file, ctx))
		return 0;

	name = __getname();
	if (!name)
		return -ENOMEM;

	ni_lock(ni);

	root = indx_get_root(&ni->dir, ni, NULL, NULL);
	if (!root) {
		err = -EINVAL;
		goto out;
	}

	if (pos >= sbi->record_size) {
		bit = (pos - sbi->record_size) >> index_bits;
		goto index_enum;
	}

	hdr = &root->ihdr;

	err = ntfs_read_hdr(sbi, ni, hdr, 0, pos, name, ctx);
	if (err)
		goto out;

	bit = 0;

index_enum:

	if (!i_size) {
		ctx->pos = eod;
		goto out;
	}

next_vcn:
	vbo = (u64)bit << index_bits;
	if (vbo >= i_size) {
		ctx->pos = eod;
		goto out;
	}

	err = indx_used_bit(&ni->dir, ni, &bit);
	if (err)
		goto out;

	if (bit == MINUS_ONE_T) {
		ctx->pos = eod;
		goto out;
	}

	vbo = (u64)bit << index_bits;
	if (vbo >= i_size)
		goto fs_error;

	err = indx_read(&ni->dir, ni, bit << ni->dir.idx2vbn_bits, &node);
	if (err)
		goto out;

	hdr = &node->index->ihdr;
	err = ntfs_read_hdr(sbi, ni, hdr, vbo + sbi->record_size, pos, name,
			    ctx);
	if (err)
		goto out;

	bit += 1;
	goto next_vcn;

fs_error:
	ntfs_inode_error(dir, "Looks like your dir is corrupt");
	err = -EINVAL;
out:

	__putname(name);
	put_indx_node(node);

	if (err == -ENOENT) {
		err = 0;
		ctx->pos = pos;
	}

	ni_unlock(ni);

	return err;
}

static int ntfs_dir_count(struct inode *dir, bool *is_empty, size_t *dirs,
			  size_t *files)
{
	int err = 0;
	ntfs_inode *ni = ntfs_i(dir);
	NTFS_DE *e = NULL;
	INDEX_ROOT *root;
	INDEX_HDR *hdr;
	const ATTR_FILE_NAME *fname;
	u32 e_size, off, end;
	u64 vbo = 0;
	size_t drs = 0, fles = 0, bit = 0;
	loff_t i_size = ni->vfs_inode.i_size;
	struct indx_node *node = NULL;
	u8 index_bits = ni->dir.index_bits;

	if (is_empty)
		*is_empty = true;

	root = indx_get_root(&ni->dir, ni, NULL, NULL);
	if (!root)
		return -EINVAL;

	hdr = &root->ihdr;

next_vcn:

	end = le32_to_cpu(hdr->used);
	off = le32_to_cpu(hdr->de_off);

next_de:
	if (off + sizeof(NTFS_DE) > end)
		goto next_hdr;

	e = Add2Ptr(hdr, off);
	e_size = le16_to_cpu(e->size);
	if (e_size < sizeof(NTFS_DE) || off + e_size > end)
		goto next_hdr;

	if (de_is_last(e))
		goto next_hdr;

	fname = de_get_fname(e);
	if (!fname)
		goto next_hdr;

	if (fname->type == FILE_NAME_DOS)
		goto next_de;

	if (is_empty) {
		*is_empty = false;
		if (!dirs && !files)
			goto out;
	}

	if (fname->dup.fa & FILE_ATTRIBUTE_DIRECTORY)
		drs += 1;
	else
		fles += 1;

	off += e_size;
	goto next_de;

next_hdr:
	if (vbo >= i_size)
		goto out;

	err = indx_used_bit(&ni->dir, ni, &bit);
	if (err)
		goto out;

	if (bit == MINUS_ONE_T)
		goto out;

	vbo = (u64)bit << index_bits;
	if (vbo >= i_size)
		goto out;

	err = indx_read(&ni->dir, ni, bit << ni->dir.idx2vbn_bits, &node);
	if (err)
		goto out;

	hdr = &node->index->ihdr;
	bit += 1;
	vbo = (u64)bit << ni->dir.idx2vbn_bits;
	goto next_vcn;

out:
	put_indx_node(node);
	if (dirs)
		*dirs = drs;
	if (files)
		*files = fles;

	return err;
}

bool dir_is_empty(struct inode *dir)
{
	bool is_empty = false;

	ntfs_dir_count(dir, &is_empty, NULL, NULL);

	return is_empty;
}

const struct file_operations ntfs_dir_operations = {
	.llseek = generic_file_llseek,
	.read = generic_read_dir,
	.iterate = ntfs_readdir,
	.fsync = ntfs_file_fsync,
	.open = ntfs_file_open,
};
