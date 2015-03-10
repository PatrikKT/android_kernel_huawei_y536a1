#ifndef __HUAWEI_BOOT_LOG_H__
#define __HUAWEI_BOOT_LOG_H__

#define HUAWEI_BOOT_LOG_ADDR (0x12C80000)

#define MAGIC_NUMBER_SIZE (sizeof(uint32_t))
#define BOOT_LOG_INFORMATION_START_ADDR (HUAWEI_BOOT_LOG_ADDR + MAGIC_NUMBER_SIZE)
#define BOOT_LOG_INFORMATION_LENTH (100)
#define SBL1_LOG_START_ADDR  (BOOT_LOG_INFORMATION_START_ADDR + BOOT_LOG_INFORMATION_LENTH)
#define HUAWEI_BOOT_LOG_SIZE (0x80000)

#define SBL_MASK             (1<<0)   /* bit 0 for sbl log */
#define ABOOT_MASK           (1<<1)   /* bit 1 for aboot log */
#define KERNEL_MASK          (1<<2)   /* bit 2 for kernel log */
#define LOGCAT_MAIN_MASK     (1<<3)   /* bit 3 for android log */
#define LOGCAT_SYSTEM_MASK   (1<<4)   /* bit 4 for android log */

// 32K for aboot log
#define ABOOT_LOG_BUF_SIZE (32*1024)
#define HUAWEI_BOOT_MAGIC_NUMBER   (0x84569875)

struct boot_log_struct {
	uint32_t sbl_addr;
	uint32_t aboot_addr;
	uint32_t kernel_addr;
	uint32_t logcat_main_addr;
	uint32_t logcat_system_addr;
	uint32_t sbl_log_size;
	uint32_t aboot_log_size;
	uint32_t kernel_log_size;
	uint32_t logcat_main_size;
	uint32_t logcat_system_size;
	uint32_t boot_process_mask;
};
#endif

