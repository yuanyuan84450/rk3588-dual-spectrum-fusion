#ifndef __HEIMANN_UAPI_H__
#define __HEIMANN_UAPI_H__

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
typedef uint8_t __u8;
typedef uint32_t __u32;
#endif

#define HEIMANN_EEPROM_SIZE 0x2000u

struct heimann_eeprom_dump {
    __u32 size;
    __u8 data[HEIMANN_EEPROM_SIZE];
};

#define HEIMANN_IOC_MAGIC 'H'
#define HEIMANN_IOC_GET_EEPROM _IOR(HEIMANN_IOC_MAGIC, 0x01, struct heimann_eeprom_dump)

#endif
