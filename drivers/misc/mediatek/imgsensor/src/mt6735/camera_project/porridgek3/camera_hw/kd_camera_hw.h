#ifndef _KD_CAMERA_HW_H_
#define _KD_CAMERA_HW_H_

#include <linux/types.h>


#if defined CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include "pmic_drv.h"

#ifndef FALSE
  #define FALSE (0)
#endif

#ifndef TRUE
  #define TRUE  (1)
#endif

/*  */
/* Analog */
#define CAMERA_POWER_VCAM_A         PMIC_APP_MAIN_CAMERA_POWER_A
/* Digital */
#define CAMERA_POWER_VCAM_D         PMIC_APP_MAIN_CAMERA_POWER_D
/* AF */
#define CAMERA_POWER_VCAM_AF        PMIC_APP_MAIN_CAMERA_POWER_AF
/* digital io */
#define CAMERA_POWER_VCAM_IO        PMIC_APP_MAIN_CAMERA_POWER_IO
/* Digital for Sub */
#define SUB_CAMERA_POWER_VCAM_D     PMIC_APP_SUB_CAMERA_POWER_D

/* FIXME, should defined in DCT tool */
/* Main sensor */
#define CAMERA_CMRST_PIN            GPIO_CAMERA_CMRST_PIN
#define CAMERA_CMRST_PIN_M_GPIO     GPIO_CAMERA_CMRST_PIN_M_GPIO

#define CAMERA_CMPDN_PIN            GPIO_CAMERA_CMPDN_PIN
#define CAMERA_CMPDN_PIN_M_GPIO     GPIO_CAMERA_CMPDN_PIN_M_GPIO

/* FRONT sensor */
#define CAMERA_CMRST1_PIN           GPIO_CAMERA_CMRST1_PIN
#define CAMERA_CMRST1_PIN_M_GPIO    GPIO_CAMERA_CMRST1_PIN_M_GPIO

#define CAMERA_CMPDN1_PIN           GPIO_CAMERA_CMPDN1_PIN
#define CAMERA_CMPDN1_PIN_M_GPIO    GPIO_CAMERA_CMPDN1_PIN_M_GPIO

/* Define I2C Bus Num */
#define SUPPORT_I2C_BUS_NUM1        0
#define SUPPORT_I2C_BUS_NUM2        2
#else
#define CAMERA_CMRST_PIN            0
#define CAMERA_CMRST_PIN_M_GPIO     0

#define CAMERA_CMPDN_PIN            0
#define CAMERA_CMPDN_PIN_M_GPIO     0

/* FRONT sensor */
#define CAMERA_CMRST1_PIN           0
#define CAMERA_CMRST1_PIN_M_GPIO    0

#define CAMERA_CMPDN1_PIN           0
#define CAMERA_CMPDN1_PIN_M_GPIO    0

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0

#endif /* End of #if defined CONFIG_MTK_LEGACY */


typedef enum KD_REGULATOR_TYPE_TAG {
	VCAMA,
	VCAMD,
	VCAMIO,
	VCAMAF
} KD_REGULATOR_TYPE_T;

typedef enum {
	CAMPDN,
	CAMRST,
	CAM1PDN,
	CAM1RST,
	CAMDLDO,
	CAMALDO
} CAMPowerType;

extern void ISP_MCLK1_EN(bool En);
extern void ISP_MCLK2_EN(bool En);
extern void ISP_MCLK3_EN(bool En);

extern bool _hwPowerDown(KD_REGULATOR_TYPE_T type);
extern bool _hwPowerOn(KD_REGULATOR_TYPE_T type, int powerVolt);

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val);
int mtkcam_gpio_init(struct platform_device *pdev);

#endif

/* sanford.lin add start on 20160122*/
#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000

typedef enum{
    VDD_None = 0,
    SensorId = 0xf1,
    SensorMCLK = 0xf2,
    PDN = 0xf3,
    RST = 0xf4,
    AVDD = 0xf5,
    DVDD = 0xf6,
    DOVDD = 0xf7,
    AFVDD = 0xf8
}PowerType;

typedef enum{
	MAIN_SENSOR = 0,
	SUB_SENSOR = 1,
	MAIN_2_SENSOR = 2,
	Mclk1 = 0xf1,
	Mclk2 = 0xf2,
    Vol_Low = 0xf3,
    Vol_High = 0xf4,
    Vol_1000 = VOL_1000,
    Vol_1200 = VOL_1200,
    Vol_1500 = VOL_1500,
    Vol_1800 = VOL_1800,
    Vol_2800 = VOL_2800
}Voltage;

typedef struct{
    PowerType PowerType;
    Voltage Voltage;
    u32 Delay;
}PowerInformation;

typedef struct{
    char* SensorName;
    PowerInformation PowerInfo[15];
}PowerSequence;

typedef struct{
    PowerSequence PowerSeq[16];
}PowerUp;

/* sanford.lin add end on 20160122*/
