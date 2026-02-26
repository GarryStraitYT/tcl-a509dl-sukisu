#define TCT_DEBUG_EN 1
/*****************************************************************************
* LOG function define here
*****************************************************************************/
#if TCT_DEBUG_EN
#define TCT_DEBUG(fmt, args...) do { \
    pr_debug(KERN_DEBUG "[TCT_BSP_NODE/D]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define TCT_INFO(fmt, args...) do { \
    pr_info(KERN_INFO "[TCT_BSP_NODE/I]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define TCT_ERROR(fmt, args...) do { \
    pr_err(KERN_ERR "[TCT_BSP_NODE/E]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define TCT_FUNC_ENTER() do { \
    pr_info(KERN_NOTICE "[TCT_BSP_NODE]%s: Enter\n", __func__); \
} while (0)

#define TCT_FUNC_EXIT() do { \
    pr_info(KERN_NOTICE "[TCT_BSP_NODE]%s: Exit(%d)\n", __func__, __LINE__); \
} while (0)
#else /* #if TCT_DEBUG_EN*/
#define TCT_DEBUG(fmt, args...)
#define TCT_INFO(fmt, args...)
#define TCT_ERROR(fmt, args...)
#define TCT_FUNC_ENTER()
#define TCT_FUNC_EXIT()
#endif
