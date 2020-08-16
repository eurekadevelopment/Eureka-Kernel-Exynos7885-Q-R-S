/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  linux/fs/ntfs3/ntfs_fs.h
 *
 * Copyright (C) 2019-2020 Paragon Software GmbH, All rights reserved.
 *
 */

/* "true" when [s,s+c) intersects with [l,l+w) */
#define IS_IN_RANGE(s, c, l, w)                                                \
	(((c) > 0 && (w) > 0) &&                                               \
	 (((l) <= (s) && (s) < ((l) + (w))) ||                                 \
	  ((s) <= (l) && ((s) + (c)) >= ((l) + (w))) ||                        \
	  ((l) < ((s) + (c)) && ((s) + (c)) < ((l) + (w)))))

/* "true" when [s,se) intersects with [l,le) */
#define IS_IN_RANGE2(s, se, l, le)                                             \
	(((se) > (s) && (le) > (l)) &&                                         \
	 (((l) <= (s) && (s) < (le)) || ((s) <= (l) && (se) >= (le)) ||        \
	  ((l) < (se) && (se) < (le))))

#define MINUS_ONE_T ((size_t)(-1))
/* Biggest MFT / smallest cluster */
#define MAXIMUM_BYTES_PER_MFT 4096 // ??
#define NTFS_BLOCKS_PER_MFT_RECORD (MAXIMUM_BYTES_PER_MFT / 512)

#define MAXIMUM_BYTES_PER_INDEX 4096 // ??
#define NTFS_BLOCKS_PER_INODE (MAXIMUM_BYTES_PER_INDEX / 512)

typedef struct ntfs_inode ntfs_inode;
typedef struct ntfs_sb_info ntfs_sb_info;
struct lznt;

struct mount_options {
	kuid_t fs_uid;
	kgid_t fs_gid;
	u16 fs_fmask;
	u16 fs_dmask;
	unsigned quiet : 1, /* set = fake successful chmods and chowns */
		sys_immutable : 1, /* set = system files are immutable */
		discard : 1, /* Issue discard requests on deletions */
		uid : 1, /* uid was set */
		gid : 1, /* gid was set */
		fmask : 1, /* fmask was set */
		dmask : 1, /*dmask was set*/
		sparse : 1, /*create sparse files*/
		showmeta : 1, /*show meta files*/
		nohidden : 1, /*do not shot hidden files*/
		acl : 1, /*create acl*/
		force : 1, /*rw mount dirty volume*/
		no_acs_rules : 1 /*exclude acs rules*/
		;
};

struct ntfs_run;

/* TODO: use rb tree instead of array */
struct runs_tree {
	struct ntfs_run *runs_;
	size_t count; // Currently used size a ntfs_run storage.
	size_t allocated; // Currently allocated ntfs_run storage size.
};

struct ntfs_buffers {
	/* Biggest MFT / smallest cluster = 4096 / 512 = 8 */
	/* Biggest index / smallest cluster = 4096 / 512 = 8 */
	struct buffer_head *bh[PAGE_SIZE >> SECTOR_SHIFT];
	u32 bytes;
	u32 nbufs;
	u32 off;
};

#define NTFS_FLAGS_NODISCARD 0x00000001
#define NTFS_FLAGS_NEED_REPLAY 0x04000000

enum ALLOCATE_OPT {
	ALLOCATE_DEF = 0, // Allocate all clusters
	ALLOCATE_MFT = 1, // Allocate for MFT
};

enum bitmap_mutex_classes {
	BITMAP_MUTEX_CLUSTERS = 0,
	BITMAP_MUTEX_MFT = 1,
};

typedef struct {
	struct super_block *sb;
	struct rw_semaphore rw_lock;

	struct runs_tree run;
	size_t nbits;

	u16 free_holder[8]; // holder for free_bits

	size_t total_zeroes; // total number of free bits
	u16 *free_bits; // free bits in each window
	size_t nwnd;
	u32 bits_last; // bits in last window

	struct rb_root start_tree; // extents, sorted by 'start'
	struct rb_root count_tree; // extents, sorted by 'count + start'
	size_t count; // extents count
	int uptodated; // -1 Tree is activated but not updated (too many fragments)
		// 0 - Tree is not activated
		// 1 - Tree is activated and updated
	size_t extent_min; // Minimal extent used while building
	size_t extent_max; // Upper estimate of biggest free block

	bool set_tail; // not necessary in driver
	bool inited;

	/* Zone [bit, end) */
	size_t zone_bit;
	size_t zone_end;

} wnd_bitmap;

typedef int (*NTFS_CMP_FUNC)(const void *key1, size_t len1, const void *key2,
			     size_t len2, const void *param);

enum index_mutex_classed {
	INDEX_MUTEX_I30 = 0,
	INDEX_MUTEX_SII = 1,
	INDEX_MUTEX_SDH = 2,
	INDEX_MUTEX_SO = 3,
	INDEX_MUTEX_SQ = 4,
	INDEX_MUTEX_SR = 5,
	INDEX_MUTEX_TOTAL
};

/* This struct works with indexes */
typedef struct {
	struct runs_tree bitmap_run;
	struct runs_tree alloc_run;

	/*TODO: remove 'cmp'*/
	NTFS_CMP_FUNC cmp;

	u8 index_bits; // log2(root->index_block_size)
	u8 idx2vbn_bits; // log2(root->index_block_clst)
	u8 vbn2vbo_bits; // index_block_size < cluster? 9 : cluster_bits
	u8 changed; // set when tree is changed
	u8 type; // index_mutex_classed

} ntfs_index;

/* Set when $LogFile is replaying */
#define NTFS_FLAGS_LOG_REPLAING 0x00000008

/* Set when we changed first MFT's which copy must be updated in $MftMirr */
#define NTFS_FLAGS_MFTMIRR 0x00001000

/* Minimum mft zone */
#define NTFS_MIN_MFT_ZONE 100

struct COMPRESS_CTX {
	u64 chunk_num; // Number of chunk cmpr_buffer/unc_buffer
	u64 first_chunk, last_chunk, total_chunks;
	u64 chunk0_off;
	void *ctx;
	u8 *cmpr_buffer;
	u8 *unc_buffer;
	void *chunk_off_mem;
	size_t chunk_off;
	u32 *chunk_off32; // pointer inside ChunkOffsetsMem
	u64 *chunk_off64; // pointer inside ChunkOffsetsMem
	u32 compress_format;
	u32 offset_bits;
	u32 chunk_bits;
	u32 chunk_size;
};

/* ntfs file system in-core superblock data */
typedef struct ntfs_sb_info {
	struct super_block *sb;

	u32 discard_granularity;
	u64 discard_granularity_mask_inv; // ~(discard_granularity_mask_inv-1)

	u32 cluster_size; // bytes per cluster
	u32 cluster_mask; // == cluster_size - 1
	u64 cluster_mask_inv; // ~(cluster_size - 1)
	u32 block_mask; // sb->s_blocksize - 1
	u32 blocks_per_cluster; // cluster_size / sb->s_blocksize

	u32 record_size;
	u32 sector_size;
	u32 index_size;

	u8 sector_bits;
	u8 cluster_bits;
	u8 record_bits;

	u64 maxbytes; // Maximum size for normal files
	u64 maxbytes_sparse; // Maximum size for sparse file

	u32 flags; // See NTFS_FLAGS_XXX

	CLST bad_clusters; // The count of marked bad clusters

	u16 max_bytes_per_attr; // maximum attribute size in record
	u16 attr_size_tr; // attribute size threshold (320 bytes)

	/* Records in $Extend */
	CLST objid_no;
	CLST quota_no;
	CLST reparse_no;
	CLST usn_jrnl_no;

	ATTR_DEF_ENTRY *def_table; // attribute definition table
	u32 def_entries;

	MFT_REC *new_rec;

	u16 *upcase;

	struct nls_table *nls;

	struct {
		u64 lbo, lbo2;
		ntfs_inode *ni;
		wnd_bitmap bitmap; // $MFT::Bitmap
		ulong reserved_bitmap;
		size_t next_free; // The next record to allocate from
		size_t used;
		u32 recs_mirr; // Number of records MFTMirr
		u8 next_reserved;
		u8 reserved_bitmap_inited;
	} mft;

	struct {
		wnd_bitmap bitmap; // $Bitmap::Data
		CLST next_free_lcn;
	} used;

	struct {
		u64 size; // in bytes
		u64 blocks; // in blocks
		u64 ser_num;
		ntfs_inode *ni;
		__le16 flags; // see VOLUME_FLAG_XXX
		u8 major_ver;
		u8 minor_ver;
		char label[65];
		bool real_dirty; /* real fs state*/
	} volume;

	struct {
		ntfs_index index_sii;
		ntfs_index index_sdh;
		ntfs_inode *ni;
		u32 next_id;
		u64 next_off;

		__le32 def_file_id;
		__le32 def_dir_id;
	} security;

	struct {
		ntfs_index index_r;
		ntfs_inode *ni;
		u64 max_size; // 16K
	} reparse;

	struct {
		ntfs_index index_o;
		ntfs_inode *ni;
	} objid;

	struct {
		/*protect 'frame_unc' and 'ctx'*/
		spinlock_t lock;
		u8 *frame_unc;
		struct lznt *ctx;
	} compress;

	struct mount_options options;
	struct ratelimit_state ratelimit;

} ntfs_sb_info;

typedef struct {
	struct rb_node node;
	ntfs_sb_info *sbi;

	CLST rno;
	MFT_REC *mrec;
	struct ntfs_buffers nb;

	bool dirty;
} mft_inode;

#define NI_FLAG_DIR 0x00000001
#define NI_FLAG_RESIDENT 0x00000002
#define NI_FLAG_UPDATE_PARENT 0x00000004

/* Data attribute is compressed special way */
#define NI_FLAG_COMPRESSED_MASK 0x00000f00 //
/* Data attribute is deduplicated */
#define NI_FLAG_DEDUPLICATED 0x00001000
#define NI_FLAG_EA 0x00002000

/* ntfs file system inode data memory */
typedef struct ntfs_inode {
	mft_inode mi; // base record

	loff_t i_valid; /* valid size */
	struct timespec64 i_crtime;

	struct mutex ni_lock;

	/* file attributes from std */
	FILE_ATTRIBUTE std_fa;
	__le32 std_security_id;

	// subrecords tree
	struct rb_root mi_tree;

	union {
		ntfs_index dir;
		struct {
			struct rw_semaphore run_lock;
			struct runs_tree run;
		} file;
	};

	struct {
		struct runs_tree run;
		void *le; // 1K aligned memory
		size_t size;
		bool dirty;
	} attr_list;

	size_t ni_flags; // NI_FLAG_XXX

	struct inode vfs_inode;
} ntfs_inode;

struct indx_node {
	struct ntfs_buffers nb;
	INDEX_BUFFER *index;
};

struct ntfs_fnd {
	int level;
	struct indx_node *nodes[20];
	NTFS_DE *de[20];
	NTFS_DE *root_de;
};

enum REPARSE_SIGN {
	REPARSE_NONE = 0,
	REPARSE_COMPRESSED = 1,
	REPARSE_DEDUPLICATED = 2,
	REPARSE_LINK = 3
};

/* functions from attrib.c*/
int attr_load_runs(ATTRIB *attr, ntfs_inode *ni, struct runs_tree *run);
int attr_allocate_clusters(ntfs_sb_info *sbi, struct runs_tree *run, CLST vcn,
			   CLST lcn, CLST len, CLST *pre_alloc,
			   enum ALLOCATE_OPT opt, CLST *alen, const size_t fr,
			   CLST *new_lcn);
int attr_set_size(ntfs_inode *ni, ATTR_TYPE type, const __le16 *name,
		  u8 name_len, struct runs_tree *run, u64 new_size,
		  const u64 *new_valid, bool keep_prealloc, ATTRIB **ret);
int attr_data_get_block(ntfs_inode *ni, CLST vcn, CLST *lcn, CLST *len,
			bool *new);
int attr_load_runs_vcn(ntfs_inode *ni, ATTR_TYPE type, const __le16 *name,
		       u8 name_len, struct runs_tree *run, CLST vcn);
int attr_is_frame_compressed(ntfs_inode *ni, ATTRIB *attr, CLST frame,
			     CLST *clst_data, bool *is_compr);
int attr_allocate_frame(ntfs_inode *ni, CLST frame, size_t compr_size,
			u64 new_valid);

/* functions from attrlist.c*/
void al_destroy(ntfs_inode *ni);
bool al_verify(ntfs_inode *ni);
int ntfs_load_attr_list(ntfs_inode *ni, ATTRIB *attr);
ATTR_LIST_ENTRY *al_enumerate(ntfs_inode *ni, ATTR_LIST_ENTRY *le);
ATTR_LIST_ENTRY *al_find_le(ntfs_inode *ni, ATTR_LIST_ENTRY *le,
			    const ATTRIB *attr);
ATTR_LIST_ENTRY *al_find_ex(ntfs_inode *ni, ATTR_LIST_ENTRY *le, ATTR_TYPE type,
			    const __le16 *name, u8 name_len, const CLST *vcn);
int al_add_le(ntfs_inode *ni, ATTR_TYPE type, const __le16 *name, u8 name_len,
	      CLST svcn, __le16 id, const MFT_REF *ref,
	      ATTR_LIST_ENTRY **new_le);
bool al_remove_le(ntfs_inode *ni, ATTR_LIST_ENTRY *le);
bool al_delete_le(ntfs_inode *ni, ATTR_TYPE type, CLST vcn, const __le16 *name,
		  size_t name_len, const MFT_REF *ref);
int al_update(ntfs_inode *ni);
static inline size_t al_aligned(size_t size)
{
	return (size + 1023) & ~(size_t)1023;
}

/* globals from bitfunc.c */
bool are_bits_clear(const ulong *map, size_t bit, size_t nbits);
bool are_bits_set(const ulong *map, size_t bit, size_t nbits);
size_t get_set_bits_ex(const ulong *map, size_t bit, size_t nbits);

/* globals from dir.c */
int uni_to_x8(ntfs_sb_info *sbi, const struct le_str *uni, u8 *buf,
	      int buf_len);
int x8_to_uni(ntfs_sb_info *sbi, const u8 *name, u32 name_len,
	      struct cpu_str *uni, u32 max_ulen, enum utf16_endian endian);
struct inode *dir_search_u(struct inode *dir, const struct cpu_str *uni,
			   struct ntfs_fnd *fnd);
struct inode *dir_search(struct inode *dir, const struct qstr *name,
			 struct ntfs_fnd *fnd);
bool dir_is_empty(struct inode *dir);
extern const struct file_operations ntfs_dir_operations;

/* globals from file.c*/
int ntfs_getattr(const struct path *path, struct kstat *stat, u32 request_mask,
		 u32 flags);
void ntfs_sparse_cluster(struct inode *inode, struct page *page0, loff_t vbo,
			 u32 bytes);
int ntfs_file_fsync(struct file *filp, loff_t start, loff_t end, int datasync);
void ntfs_truncate_blocks(struct inode *inode, loff_t offset);
int ntfs_setattr(struct dentry *dentry, struct iattr *attr);
int ntfs_file_open(struct inode *inode, struct file *file);
extern const struct inode_operations ntfs_special_inode_operations;
extern const struct inode_operations ntfs_file_inode_operations;
extern const struct file_operations ntfs_file_operations;

/* globals from frecord.c */
void ni_remove_mi(ntfs_inode *ni, mft_inode *mi);
ATTR_STD_INFO *ni_std(ntfs_inode *ni);
void ni_clear(ntfs_inode *ni);
int ni_load_mi_ex(ntfs_inode *ni, CLST rno, mft_inode **mi);
int ni_load_mi(ntfs_inode *ni, ATTR_LIST_ENTRY *le, mft_inode **mi);
ATTRIB *ni_find_attr(ntfs_inode *ni, ATTRIB *attr, ATTR_LIST_ENTRY **entry_o,
		     ATTR_TYPE type, const __le16 *name, u8 name_len,
		     const CLST *vcn, mft_inode **mi);
ATTRIB *ni_enum_attr_ex(ntfs_inode *ni, ATTRIB *attr, ATTR_LIST_ENTRY **le);
ATTRIB *ni_load_attr(ntfs_inode *ni, ATTR_TYPE type, const __le16 *name,
		     u8 name_len, CLST vcn, mft_inode **pmi);
int ni_load_all_mi(ntfs_inode *ni);
bool ni_add_subrecord(ntfs_inode *ni, CLST rno, mft_inode **mi);
int ni_remove_attr(ntfs_inode *ni, ATTR_TYPE type, const __le16 *name,
		   size_t name_len, bool base_only, const __le16 *id);
int ni_create_attr_list(ntfs_inode *ni);
int ni_expand_list(ntfs_inode *ni);
int ni_insert_nonresident(ntfs_inode *ni, ATTR_TYPE type, const __le16 *name,
			  u8 name_len, const struct runs_tree *run, CLST svcn,
			  CLST len, __le16 flags, ATTRIB **new_attr,
			  mft_inode **mi);
int ni_insert_resident(ntfs_inode *ni, u32 data_size, ATTR_TYPE type,
		       const __le16 *name, u8 name_len, ATTRIB **new_attr,
		       mft_inode **mi);
int ni_remove_attr_le(ntfs_inode *ni, ATTRIB *attr, ATTR_LIST_ENTRY *le);
int ni_delete_all(ntfs_inode *ni);
ATTR_FILE_NAME *ni_fname_name(ntfs_inode *ni, const struct cpu_str *uni,
			      const MFT_REF *home, ATTR_LIST_ENTRY **entry);
ATTR_FILE_NAME *ni_fname_type(ntfs_inode *ni, u8 name_type,
			      ATTR_LIST_ENTRY **entry);
u16 ni_fnames_count(ntfs_inode *ni);
int ni_init_compress(ntfs_inode *ni, struct COMPRESS_CTX *ctx);
enum REPARSE_SIGN ni_parse_reparse(ntfs_inode *ni, ATTRIB *attr, void *buffer);
int ni_write_inode(struct inode *inode, int sync, const char *hint);
#define _ni_write_inode(i, w) ni_write_inode(i, w, __func__)

/* globals from compress.c */
int ni_readpage_cmpr(ntfs_inode *ni, struct page *page);
int ni_writepage_cmpr(struct page *page, int sync);

/* globals from fslog.c */
int log_replay(ntfs_inode *ni);

/* globals from fsntfs.c */
bool ntfs_fix_pre_write(NTFS_RECORD_HEADER *rhdr, size_t bytes);
int ntfs_fix_post_read(NTFS_RECORD_HEADER *rhdr, size_t bytes, bool simple);
int ntfs_extend_init(ntfs_sb_info *sbi);
int ntfs_loadlog_and_replay(ntfs_inode *ni, ntfs_sb_info *sbi);
const ATTR_DEF_ENTRY *ntfs_query_def(ntfs_sb_info *sbi, ATTR_TYPE Type);
int ntfs_look_for_free_space(ntfs_sb_info *sbi, CLST lcn, CLST len,
			     CLST *new_lcn, CLST *new_len,
			     enum ALLOCATE_OPT opt);
int ntfs_look_free_mft(ntfs_sb_info *sbi, CLST *rno, bool mft, ntfs_inode *ni,
		       mft_inode **mi);
void ntfs_mark_rec_free(ntfs_sb_info *sbi, CLST nRecord);
int ntfs_clear_mft_tail(ntfs_sb_info *sbi, size_t from, size_t to);
int ntfs_refresh_zone(ntfs_sb_info *sbi);
int ntfs_update_mftmirr(ntfs_sb_info *sbi, int wait);
enum NTFS_DIRTY_FLAGS {
	NTFS_DIRTY_DIRTY = 0,
	NTFS_DIRTY_CLEAR = 1,
	NTFS_DIRTY_ERROR = 2,
};
int ntfs_set_state(ntfs_sb_info *sbi, enum NTFS_DIRTY_FLAGS dirty);
int ntfs_sb_read(struct super_block *sb, u64 lbo, size_t bytes, void *buffer);
int ntfs_sb_write(struct super_block *sb, u64 lbo, size_t bytes,
		  const void *buffer, int wait);
int ntfs_sb_write_run(ntfs_sb_info *sbi, struct runs_tree *run, u64 vbo,
		      const void *buf, size_t bytes);
struct buffer_head *ntfs_bread_run(ntfs_sb_info *sbi, struct runs_tree *run,
				   u64 vbo);
int ntfs_read_run_nb(ntfs_sb_info *sbi, struct runs_tree *run, u64 vbo,
		     void *buf, u32 bytes, struct ntfs_buffers *nb);
int ntfs_read_bh_ex(ntfs_sb_info *sbi, struct runs_tree *run, u64 vbo,
		    NTFS_RECORD_HEADER *rhdr, u32 bytes,
		    struct ntfs_buffers *nb);
int ntfs_get_bh(ntfs_sb_info *sbi, struct runs_tree *run, u64 vbo, u32 bytes,
		struct ntfs_buffers *nb);
int ntfs_write_bh_ex(ntfs_sb_info *sbi, NTFS_RECORD_HEADER *rhdr,
		     struct ntfs_buffers *nb, int sync);
int ntfs_vbo_to_pbo(ntfs_sb_info *sbi, struct runs_tree *run, u64 vbo, u64 *pbo,
		    u64 *bytes);
ntfs_inode *ntfs_new_inode(ntfs_sb_info *sbi, CLST nRec, bool dir);
extern const u8 s_dir_security[0x50];
extern const u8 s_file_security[0x58];
int ntfs_security_init(ntfs_sb_info *sbi);
int ntfs_get_security_by_id(ntfs_sb_info *sbi, u32 security_id, void **sd,
			    size_t *size);
int ntfs_insert_security(ntfs_sb_info *sbi, const void *sd, u32 size,
			 __le32 *security_id, bool *inserted);
int ntfs_reparse_init(ntfs_sb_info *sbi);
int ntfs_objid_init(ntfs_sb_info *sbi);
int ntfs_objid_remove(ntfs_sb_info *sbi, GUID *guid);
int ntfs_insert_reparse(ntfs_sb_info *sbi, __le32 rtag, const MFT_REF *ref);
int ntfs_remove_reparse(ntfs_sb_info *sbi, __le32 rtag, const MFT_REF *ref);
void mark_as_free_ex(ntfs_sb_info *sbi, CLST lcn, CLST len, bool trim);
int run_deallocate(ntfs_sb_info *sbi, struct runs_tree *run, bool trim);

/* globals from index.c */
int indx_used_bit(ntfs_index *indx, ntfs_inode *ni, size_t *bit);
void fnd_clear(struct ntfs_fnd *fnd);
struct ntfs_fnd *fnd_get(ntfs_index *indx);
void fnd_put(struct ntfs_fnd *fnd);
void indx_clear(ntfs_index *idx);
int indx_init(ntfs_index *indx, ntfs_sb_info *sbi, const ATTRIB *attr,
	      enum index_mutex_classed type);
INDEX_ROOT *indx_get_root(ntfs_index *indx, ntfs_inode *ni, ATTRIB **attr,
			  mft_inode **mi);
int indx_read(ntfs_index *idx, ntfs_inode *ni, CLST vbn,
	      struct indx_node **node);
int indx_find(ntfs_index *indx, ntfs_inode *dir, const INDEX_ROOT *root,
	      const void *Key, size_t KeyLen, const void *param, int *diff,
	      NTFS_DE **entry, struct ntfs_fnd *fnd);
int indx_find_sort(ntfs_index *indx, ntfs_inode *ni, const INDEX_ROOT *root,
		   NTFS_DE **entry, struct ntfs_fnd *fnd);
int indx_find_raw(ntfs_index *indx, ntfs_inode *ni, const INDEX_ROOT *root,
		  NTFS_DE **entry, size_t *off, struct ntfs_fnd *fnd);
int indx_insert_entry(ntfs_index *indx, ntfs_inode *ni, const NTFS_DE *new_de,
		      const void *param, struct ntfs_fnd *fnd);
int indx_delete_entry(ntfs_index *indx, ntfs_inode *ni, const void *key,
		      u32 key_len, const void *param);
int indx_update_dup(ntfs_inode *ni, ntfs_sb_info *sbi,
		    const ATTR_FILE_NAME *fname, const NTFS_DUP_INFO *dup,
		    int sync);

/* globals from inode.c */
struct inode *ntfs_iget5(struct super_block *sb, const MFT_REF *ref,
			 const struct cpu_str *name);
int ntfs_set_size(struct inode *inode, u64 new_size);
int reset_log_file(struct inode *inode);
int ntfs_get_block(struct inode *inode, sector_t vbn,
		   struct buffer_head *bh_result, int create);
int ntfs_write_inode(struct inode *inode, struct writeback_control *wbc);
int ntfs_sync_inode(struct inode *inode);
int ntfs_flush_inodes(struct super_block *sb, struct inode *i1,
		      struct inode *i2);
int inode_write_data(struct inode *inode, const void *data, size_t bytes);
int ntfs_create_inode(struct inode *dir, struct dentry *dentry,
		      struct file *file, umode_t mode, dev_t dev,
		      const char *symname, unsigned int size, int excl,
		      struct ntfs_fnd *fnd, struct inode **new_inode);
int ntfs_link_inode(struct inode *inode, struct dentry *dentry);
int ntfs_unlink_inode(struct inode *dir, const struct dentry *dentry);
void ntfs_evict_inode(struct inode *inode);
int ntfs_readpage(struct file *file, struct page *page);
extern const struct inode_operations ntfs_link_inode_operations;
extern const struct address_space_operations ntfs_aops;
extern const struct address_space_operations ntfs_aops_cmpr;

/* globals from name_i.c*/
int fill_name_de(ntfs_sb_info *sbi, void *buf, const struct qstr *name);
struct dentry *ntfs_get_parent(struct dentry *child);

extern const struct inode_operations ntfs_dir_inode_operations;

/* globals from record.c */
int mi_get(ntfs_sb_info *sbi, CLST rno, mft_inode **mi);
void mi_put(mft_inode *mi);
int mi_init(mft_inode *mi, ntfs_sb_info *sbi, CLST rno);
int mi_read(mft_inode *mi, bool is_mft);
ATTRIB *mi_enum_attr(mft_inode *mi, ATTRIB *attr);
// TODO: id?
ATTRIB *mi_find_attr(mft_inode *mi, ATTRIB *attr, ATTR_TYPE type,
		     const __le16 *name, size_t name_len, const __le16 *id);
static inline ATTRIB *rec_find_attr_le(mft_inode *rec, ATTR_LIST_ENTRY *le)
{
	return mi_find_attr(rec, NULL, le->type, le_name(le), le->name_len,
			    &le->id);
}
int mi_write(mft_inode *mi, int wait);
int mi_format_new(mft_inode *mi, ntfs_sb_info *sbi, CLST rno, __le16 flags,
		  bool is_mft);
void mi_mark_free(mft_inode *mi);
ATTRIB *mi_insert_attr(mft_inode *mi, ATTR_TYPE type, const __le16 *name,
		       u8 name_len, u32 asize, u16 name_off);

bool mi_remove_attr(mft_inode *mi, ATTRIB *attr);
bool mi_resize_attr(mft_inode *mi, ATTRIB *attr, int bytes);
int mi_pack_runs(mft_inode *mi, ATTRIB *attr, struct runs_tree *run, CLST len);
static inline bool mi_is_ref(const mft_inode *mi, const MFT_REF *ref)
{
	if (le32_to_cpu(ref->low) != mi->rno)
		return false;
	if (ref->seq != mi->mrec->seq)
		return false;

#ifdef NTFS3_64BIT_CLUSTER
	return le16_to_cpu(ref->high) == (mi->rno >> 32);
#else
	return !ref->high;
#endif
}

/* globals from run.c */
bool run_lookup_entry(const struct runs_tree *run, CLST vcn, CLST *lcn,
		      CLST *len, size_t *index);
void run_truncate(struct runs_tree *run, CLST vcn);
void run_truncate_head(struct runs_tree *run, CLST vcn);
bool run_lookup(const struct runs_tree *run, CLST Vcn, size_t *Index);
bool run_add_entry(struct runs_tree *run, CLST vcn, CLST lcn, CLST len);
bool run_get_entry(const struct runs_tree *run, size_t index, CLST *vcn,
		   CLST *lcn, CLST *len);
bool run_is_mapped_full(const struct runs_tree *run, CLST svcn, CLST evcn);

int run_pack(const struct runs_tree *run, CLST svcn, CLST len, u8 *run_buf,
	     u32 run_buf_size, CLST *packed_vcns);
int run_unpack(struct runs_tree *run, ntfs_sb_info *sbi, CLST ino, CLST svcn,
	       CLST evcn, const u8 *run_buf, u32 run_buf_size);

#ifdef NTFS3_CHECK_FREE_CLST
int run_unpack_ex(struct runs_tree *run, ntfs_sb_info *sbi, CLST ino, CLST svcn,
		  CLST evcn, const u8 *run_buf, u32 run_buf_size);
#else
#define run_unpack_ex run_unpack
#endif
int run_get_highest_vcn(CLST vcn, const u8 *run_buf, u64 *highest_vcn);

/* globals from super.c */
void *ntfs_set_shared(void *ptr, u32 bytes);
void *ntfs_put_shared(void *ptr);
void ntfs_unmap_meta(struct super_block *sb, CLST lcn, CLST len);
int ntfs_discard(ntfs_sb_info *sbi, CLST Lcn, CLST Len);

/* globals from ubitmap.c*/
void wnd_close(wnd_bitmap *wnd);
static inline size_t wnd_zeroes(const wnd_bitmap *wnd)
{
	return wnd->total_zeroes;
}
void wnd_trace(wnd_bitmap *wnd);
void wnd_trace_tree(wnd_bitmap *wnd, u32 nExtents, const char *Hint);
int wnd_init(wnd_bitmap *wnd, struct super_block *sb, size_t nBits);
int wnd_set_free(wnd_bitmap *wnd, size_t FirstBit, size_t Bits);
int wnd_set_used(wnd_bitmap *wnd, size_t FirstBit, size_t Bits);
bool wnd_is_free(wnd_bitmap *wnd, size_t FirstBit, size_t Bits);
bool wnd_is_used(wnd_bitmap *wnd, size_t FirstBit, size_t Bits);

/* Possible values for 'flags' 'wnd_find' */
#define BITMAP_FIND_MARK_AS_USED 0x01
#define BITMAP_FIND_FULL 0x02
size_t wnd_find(wnd_bitmap *wnd, size_t to_alloc, size_t hint, size_t flags,
		size_t *allocated);
int wnd_extend(wnd_bitmap *wnd, size_t new_bits);
void wnd_zone_set(wnd_bitmap *wnd, size_t Lcn, size_t Len);
int ntfs_trim_fs(ntfs_sb_info *sbi, struct fstrim_range *range);

/* globals from upcase.c */
int ntfs_cmp_names(const __le16 *s1, size_t l1, const __le16 *s2, size_t l2,
		   const u16 *upcase);
int ntfs_cmp_names_cpu(const struct cpu_str *uni1, const struct le_str *uni2,
		       const u16 *upcase);

/* globals from xattr.c */
struct posix_acl *ntfs_get_acl(struct inode *inode, int type);
int ntfs_set_acl(struct inode *inode, struct posix_acl *acl, int type);
int ntfs_acl_chmod(struct inode *inode);
int ntfs_permission(struct inode *inode, int mask);
ssize_t ntfs_listxattr(struct dentry *dentry, char *buffer, size_t size);
extern const struct xattr_handler *ntfs_xattr_handlers[];

/* globals from lznt.c */
struct lznt *get_compression_ctx(bool std);
size_t compress_lznt(const void *uncompressed, size_t uncompressed_size,
		     void *compressed, size_t compressed_size,
		     struct lznt *ctx);
ssize_t decompress_lznt(const void *compressed, size_t compressed_size,
			void *uncompressed, size_t uncompressed_size);

char *attr_str(const ATTRIB *attr, char *buf, size_t buf_len);

static inline bool is_nt5(ntfs_sb_info *sbi)
{
	return sbi->volume.major_ver >= 3;
}

/*(sb->s_flags & SB_ACTIVE)*/
static inline bool is_mounted(ntfs_sb_info *sbi)
{
	return !!sbi->sb->s_root;
}

static inline bool ntfs_is_meta_file(ntfs_sb_info *sbi, CLST rno)
{
	return rno < MFT_REC_FREE || rno == sbi->objid_no ||
	       rno == sbi->quota_no || rno == sbi->reparse_no ||
	       rno == sbi->usn_jrnl_no;
}

static inline void ntfs_unmap_page(struct page *page)
{
	kunmap(page);
	put_page(page);
}

static inline struct page *ntfs_map_page(struct address_space *mapping,
					 unsigned long index)
{
	struct page *page = read_mapping_page(mapping, index, NULL);

	if (!IS_ERR(page)) {
		kmap(page);
		if (!PageError(page))
			return page;
		ntfs_unmap_page(page);
		return ERR_PTR(-EIO);
	}
	return page;
}

static inline size_t wnd_zone_bit(const wnd_bitmap *wnd)
{
	return wnd->zone_bit;
}

static inline size_t wnd_zone_len(const wnd_bitmap *wnd)
{
	return wnd->zone_end - wnd->zone_bit;
}

static inline void run_init(struct runs_tree *run)
{
	run->runs_ = NULL;
	run->count = 0;
	run->allocated = 0;
}

static inline struct runs_tree *run_alloc(void)
{
	return ntfs_alloc(sizeof(struct runs_tree), 1);
}

static inline void run_close(struct runs_tree *run)
{
	ntfs_free(run->runs_);
	memset(run, 0, sizeof(*run));
}

static inline void run_free(struct runs_tree *run)
{
	if (run) {
		ntfs_free(run->runs_);
		ntfs_free(run);
	}
}

static inline bool run_is_empty(struct runs_tree *run)
{
	return !run->count;
}

/* NTFS uses quad aligned bitmaps */
static inline size_t bitmap_size(size_t bits)
{
	return QuadAlign((bits + 7) >> 3);
}

#define _100ns2seconds 10000000
#define SecondsToStartOf1970 0x00000002B6109100

#define NTFS_TIME_GRAN 100

/*
 * kernel2nt
 *
 * converts in-memory kernel timestamp into nt time
 */
static inline __le64 kernel2nt(const struct timespec64 *ts)
{
	// 10^7 units of 100 nanoseconds one second
	return cpu_to_le64(_100ns2seconds *
				   (ts->tv_sec + SecondsToStartOf1970) +
			   ts->tv_nsec / NTFS_TIME_GRAN);
}

/*
 * nt2kernel
 *
 * converts on-disk nt time into kernel timestamp
 */
static inline void nt2kernel(const __le64 tm, struct timespec64 *ts)
{
	u64 t = le64_to_cpu(tm) - _100ns2seconds * SecondsToStartOf1970;

	// WARNING: do_div changes its first argument(!)
	ts->tv_nsec = do_div(t, _100ns2seconds) * 100;
	ts->tv_sec = t;
}

static inline ntfs_sb_info *ntfs_sb(struct super_block *sb)
{
	return sb->s_fs_info;
}

/* Align up on cluster boundary */
static inline u64 ntfs_up_cluster(const ntfs_sb_info *sbi, u64 size)
{
	return (size + sbi->cluster_mask) & ~((u64)sbi->cluster_mask);
}

/* Align up on cluster boundary */
static inline u64 ntfs_up_block(const struct super_block *sb, u64 size)
{
	return (size + sb->s_blocksize - 1) & ~(u64)(sb->s_blocksize - 1);
}

static inline CLST bytes_to_cluster(const ntfs_sb_info *sbi, u64 size)
{
	return (size + sbi->cluster_mask) >> sbi->cluster_bits;
}

static inline u64 bytes_to_block(const struct super_block *sb, u64 size)
{
	return (size + sb->s_blocksize - 1) >> sb->s_blocksize_bits;
}

/* calculates ((bytes + frame_size - 1)/frame_size)*frame_size; */
static inline u64 ntfs_up_frame(const ntfs_sb_info *sbi, u64 bytes, u8 c_unit)
{
	u32 bytes_per_frame = 1u << (c_unit + sbi->cluster_bits);

	return (bytes + bytes_per_frame - 1) & ~(u64)(bytes_per_frame - 1);
}

static inline struct buffer_head *ntfs_bread(struct super_block *sb,
					     sector_t block)
{
	struct buffer_head *bh;

	bh = sb_bread(sb, block);
	if (bh)
		return bh;

	__ntfs_trace(sb, KERN_ERR, "failed to read volume at offset 0x%llx",
		     (u64)block << sb->s_blocksize_bits);
	return NULL;
}

static inline bool is_power_of2(size_t v)
{
	return v && !(v & (v - 1));
}

static inline ntfs_inode *ntfs_i(struct inode *inode)
{
	return container_of(inode, ntfs_inode, vfs_inode);
}

static inline bool is_compressed(const ntfs_inode *ni)
{
	return (ni->std_fa & FILE_ATTRIBUTE_COMPRESSED) ||
	       (ni->ni_flags & NI_FLAG_COMPRESSED_MASK);
}

static inline bool is_dedup(const ntfs_inode *ni)
{
	return ni->ni_flags & NI_FLAG_DEDUPLICATED;
}

static inline bool is_encrypted(const ntfs_inode *ni)
{
	return ni->std_fa & FILE_ATTRIBUTE_ENCRYPTED;
}

static inline bool is_sparsed(const ntfs_inode *ni)
{
	return ni->std_fa & FILE_ATTRIBUTE_SPARSE_FILE;
}

static inline void le16_sub_cpu(__le16 *var, u16 val)
{
	*var = cpu_to_le16(le16_to_cpu(*var) - val);
}

static inline void le32_sub_cpu(__le32 *var, u32 val)
{
	*var = cpu_to_le32(le32_to_cpu(*var) - val);
}

static inline void nb_put(struct ntfs_buffers *nb)
{
	u32 i, nbufs = nb->nbufs;

	if (!nbufs)
		return;

	for (i = 0; i < nbufs; i++)
		put_bh(nb->bh[i]);
	nb->nbufs = 0;
}

static inline void put_indx_node(struct indx_node *in)
{
	if (!in)
		return;

	ntfs_free(in->index);
	nb_put(&in->nb);
	ntfs_free(in);
}

static inline void mi_clear(mft_inode *mi)
{
	nb_put(&mi->nb);
	ntfs_free(mi->mrec);
	mi->mrec = NULL;
}

static inline void ni_lock(ntfs_inode *ni)
{
	mutex_lock(&ni->ni_lock);
}

static inline void ni_unlock(ntfs_inode *ni)
{
	mutex_unlock(&ni->ni_lock);
}

static inline int ni_trylock(ntfs_inode *ni)
{
	return mutex_trylock(&ni->ni_lock);
}

static inline int ni_has_resident_data(ntfs_inode *ni)
{
	return ni->ni_flags & NI_FLAG_RESIDENT;
}

static inline int attr_load_runs_attr(ntfs_inode *ni, ATTRIB *attr,
				      struct runs_tree *run, CLST vcn)
{
	return attr_load_runs_vcn(ni, attr->type, attr_name(attr),
				  attr->name_len, run, vcn);
}

static inline void le64_sub_cpu(__le64 *var, u64 val)
{
	*var = cpu_to_le64(le64_to_cpu(*var) - val);
}
