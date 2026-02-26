/* SPDX-License-Identifier: GPL-2.0 */





#ifndef __KREE_MEM_H__
#define __KREE_MEM_H__

#if IS_ENABLED(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT)	\
	|| IS_ENABLED(CONFIG_MTK_ENABLE_GENIEZONE)

#include <tz_cross/trustzone.h>
#include <tz_cross/gz_version.h>

#define KREE_SESSION_HANDLE_NULL ((KREE_SESSION_HANDLE)0)
#define KREE_SESSION_HANDLE_FAIL ((KREE_SESSION_HANDLE)-1)


/* / KREE session handle type. */
typedef int32_t KREE_SESSION_HANDLE;

/* Shared memory handle define */
typedef uint32_t KREE_SHAREDMEM_HANDLE;

/* Secure memory handle define */
typedef uint32_t KREE_SECUREMEM_HANDLE;

/* Secure chunk memory handle define */
typedef uint32_t KREE_SECURECM_HANDLE;

/* Release Secure chunk memory handle define */
typedef uint32_t KREE_RELEASECM_HANDLE;

/* ION memory handle define */
typedef uint32_t KREE_ION_HANDLE;
typedef uint32_t *KREE_ION_HANDLE_PTR;

struct kree_shared_mem_param {

	uint32_t size;
	uint32_t region_id;
	void *buffer;
	void *mapAry;
};
#define KREE_SHAREDMEM_PARAM struct kree_shared_mem_param

struct KREE_SHM_RUNLENGTH_ENTRY {
	uint32_t high; /* (uint64_t) start PA address = high | low */
	uint32_t low;
	uint32_t size;
};

struct KREE_SHM_RUNLENGTH_LIST {
	struct KREE_SHM_RUNLENGTH_ENTRY entry;
	struct KREE_SHM_RUNLENGTH_LIST *next;
};



TZ_RESULT KREE_RegisterSharedmem(KREE_SESSION_HANDLE session,
				 KREE_SHAREDMEM_HANDLE *shm_handle,
				 KREE_SHAREDMEM_PARAM *param);

TZ_RESULT KREE_UnregisterSharedmem(KREE_SESSION_HANDLE session,
				   KREE_SHAREDMEM_HANDLE shm_handle);


TZ_RESULT KREE_AllocSecuremem(KREE_SESSION_HANDLE session,
			      KREE_SECUREMEM_HANDLE *mem_handle,
			      uint32_t alignment, uint32_t size);
TZ_RESULT KREE_ZallocSecuremem(KREE_SESSION_HANDLE session,
				     KREE_SECUREMEM_HANDLE *mem_handle,
				     uint32_t alignment, uint32_t size);

/*fix mtee sync*/
TZ_RESULT KREE_AllocSecurememWithTag(KREE_SESSION_HANDLE session,
				     KREE_SECUREMEM_HANDLE *mem_handle,
				     uint32_t alignment, uint32_t size,
				     const char *tag);

TZ_RESULT KREE_ZallocSecurememWithTag(KREE_SESSION_HANDLE session,
				     KREE_SECUREMEM_HANDLE *mem_handle,
				     uint32_t alignment, uint32_t size,
				     const char *tag);
TZ_RESULT KREE_ReferenceSecuremem(KREE_SESSION_HANDLE session,
				  KREE_SECUREMEM_HANDLE mem_handle);

TZ_RESULT KREE_UnreferenceSecuremem(KREE_SESSION_HANDLE session,
				    KREE_SECUREMEM_HANDLE mem_handle);

TZ_RESULT KREE_ReleaseSecureMultichunkmem(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE cm_handle);

TZ_RESULT KREE_ReleaseSecureMultichunkmem_basic(KREE_SESSION_HANDLE session,
					  KREE_SHAREDMEM_HANDLE cm_handle);


TZ_RESULT KREE_AppendSecureMultichunkmem(KREE_SESSION_HANDLE session,
					 KREE_SHAREDMEM_HANDLE *cm_handle,
					 KREE_SHAREDMEM_PARAM *param);

TZ_RESULT KREE_AppendSecureMultichunkmem_basic(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE *cm_handle,
					KREE_SHAREDMEM_PARAM *param);
TZ_RESULT KREE_AllocSecureMultichunkmem(KREE_SESSION_HANDLE session,
				KREE_SHAREDMEM_HANDLE chm_handle,
				KREE_SECUREMEM_HANDLE *mem_handle,
				uint32_t alignment, uint32_t size);

TZ_RESULT KREE_ZallocSecureMultichunkmem(KREE_SESSION_HANDLE session,
				KREE_SHAREDMEM_HANDLE chm_handle,
				KREE_SECUREMEM_HANDLE *mem_handle,
				uint32_t alignment, uint32_t size);

TZ_RESULT KREE_ReferenceSecureMultichunkmem(KREE_SESSION_HANDLE session,
					KREE_SECUREMEM_HANDLE mem_handle);

TZ_RESULT KREE_UnreferenceSecureMultichunkmem(KREE_SESSION_HANDLE session,
	KREE_SECUREMEM_HANDLE mem_handle, uint32_t *count);

TZ_RESULT KREE_ION_AllocChunkmem(KREE_SESSION_HANDLE session,
				 KREE_SHAREDMEM_HANDLE chm_handle,
				 KREE_SECUREMEM_HANDLE *secmHandle,
				 uint32_t alignment,
				 uint32_t size);

TZ_RESULT KREE_ION_ZallocChunkmem(KREE_SESSION_HANDLE session,
				  KREE_SHAREDMEM_HANDLE chm_handle,
				  KREE_SECUREMEM_HANDLE *secmHandle,
				  uint32_t alignment, uint32_t size);

TZ_RESULT KREE_ION_ReferenceChunkmem(KREE_SESSION_HANDLE session,
				     KREE_SECUREMEM_HANDLE secmHandle);

TZ_RESULT KREE_ION_UnreferenceChunkmem(KREE_SESSION_HANDLE session,
				       KREE_SECUREMEM_HANDLE secmHandle);

/*only for test*/
TZ_RESULT KREE_QueryChunkmem_TEST(KREE_SESSION_HANDLE session,
				KREE_SECUREMEM_HANDLE secmHandle, uint32_t cmd);
TZ_RESULT KREE_ION_QueryChunkmem_TEST(KREE_SESSION_HANDLE session,
				      KREE_SECUREMEM_HANDLE secmHandle, uint32_t cmd);

TZ_RESULT KREE_ConfigSecureMultiChunkMemInfo(KREE_SESSION_HANDLE session,
					     uint64_t pa, uint32_t size,
					     uint32_t region_id);

#endif /* CONFIG_MTK_IN_HOUSE_TEE_SUPPORT || CONFIG_MTK_ENABLE_GENIEZONE*/
#endif /* __KREE_MEM_H__ */
