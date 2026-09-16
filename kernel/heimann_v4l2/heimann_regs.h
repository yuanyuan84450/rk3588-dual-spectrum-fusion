#ifndef __HEIMANN_REGS_H__
#define __HEIMANN_REGS_H__

/*
 * Heimann sensor I2C address
 */
#define SENSOR_ADDRESS          0x1A
#define EEPROM_ADDRESS          0x50

/*
 * Sensor information
 */
#define NUMBER_OF_PIXEL         1024
#define NUMBER_OF_BLOCKS        4
#define PIXEL_PER_BLOCK         128
#define PIXEL_PER_COLUMN        32
#define PIXEL_PER_ROW           32
#define ROW_PER_BLOCK           4

#define BLOCK_LENGTH            258
#define DATA_POS                2

/*
 * Sensor registers
 */
#define TOP_HALF                0x0A
#define BOTTOM_HALF             0x0B

#define CONFIGURATION_REGISTER  0x01

#define STATUS_REGISTER         0x02

#define TRIM_REGISTER1          0x03
#define TRIM_REGISTER2          0x04
#define TRIM_REGISTER3          0x05
#define TRIM_REGISTER4          0x06
#define TRIM_REGISTER5          0x07
#define TRIM_REGISTER6          0x08
#define TRIM_REGISTER7          0x09

/*
 * EEPROM addresses
 */
#define E_PIXCMIN_1             0x0000
#define E_PIXCMIN_2             0x0001
#define E_PIXCMIN_3             0x0002
#define E_PIXCMIN_4             0x0003

#define E_PIXCMAX_1             0x0004
#define E_PIXCMAX_2             0x0005
#define E_PIXCMAX_3             0x0006
#define E_PIXCMAX_4             0x0007

#define E_GRADSCALE             0x0008
#define E_TABLENUMBER1          0x000B
#define E_TABLENUMBER2          0x000C
#define E_EPSILON               0x000D

#define E_MBIT_CALIB            0x001A
#define E_BIAS_CALIB            0x001B
#define E_CLK_CALIB             0x001C
#define E_BPA_CALIB             0x001D
#define E_PU_CALIB              0x001E

#define E_ARRAYTYPE             0x0022

#define E_VDDTH1_1              0x0026
#define E_VDDTH1_2              0x0027
#define E_VDDTH2_1              0x0028
#define E_VDDTH2_2              0x0029

#define E_PTATGR_1              0x0034
#define E_PTATGR_2              0x0035
#define E_PTATGR_3              0x0036
#define E_PTATGR_4              0x0037

#define E_PTATOFF_1             0x0038
#define E_PTATOFF_2             0x0039
#define E_PTATOFF_3             0x003A
#define E_PTATOFF_4             0x003B

#define E_PTATTH1_1             0x003C
#define E_PTATTH1_2             0x003D
#define E_PTATTH2_1             0x003E
#define E_PTATTH2_2             0x003F

#define E_VDDSCGRAD             0x004E
#define E_VDDSCOFF              0x004F

#define E_GLOBALOFF             0x0054
#define E_GLOBALGAIN_1          0x0055
#define E_GLOBALGAIN_2          0x0056

#define E_MBIT_USER             0x0060
#define E_BIAS_USER             0x0061
#define E_CLK_USER              0x0062
#define E_BPA_USER              0x0063
#define E_PU_USER               0x0064

#define E_ID1                   0x0074
#define E_ID2                   0x0075
#define E_ID3                   0x0076
#define E_ID4                   0x0077

#define E_NROFDEFPIX            0x007F
#define E_DEADPIXADR            0x0080
#define E_DEADPIXMASK           0x00B0

#define E_VDDCOMPGRAD           0x0340
#define E_VDDCOMPOFF            0x0540
#define E_THGRAD                0x0740
#define E_THOFFSET              0x0F40
#define E_PIJ                   0x1740

#define EEPROM_SIZE             0x2000

#endif
