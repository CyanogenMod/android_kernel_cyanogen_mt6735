#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_camera_hw.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(PFX fmt, ##arg)//pr_debug(PFX fmt, ##arg)

//#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)   printk(fmt, ##arg)//pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
		do {    \
		   pr_debug(PFX fmt, ##arg); \
		} while (0)
#else
#define PK_DBG PK_DBG_NONE
#define PK_ERR(fmt, arg...)    printk(fmt, ##arg)//pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...)
#endif

#define AEON_CAM_SUPPORT
//#define GPIO_VCAMA_LDO
//#define GPIO_VCAMD_LDO

PowerUp PowerOnList={
    {
	#if defined(S5K4H5YC_MIPI_RAW)
	    {SENSOR_DRVNAME_S5K4H5YC_MIPI_RAW,
		   {
				{SensorId, MAIN_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST, Vol_Low, 0},
				{PDN, Vol_Low, 0},
				{DVDD, Vol_1200, 10},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{AFVDD, Vol_2800, 10},
				{RST, Vol_High, 10},
				{PDN, Vol_High, 10},
		   },
	    },
	#endif
    #if defined(OV5648_MIPI_RAW)
        {SENSOR_DRVNAME_OV5648_MIPI_RAW,
            {
                {SensorId, MAIN_SENSOR, 0},
                {SensorMCLK, Mclk1, 0},
                {RST,    Vol_Low,  0},
                {PDN,    Vol_Low,  0},
                {AVDD,    Vol_2800, 10},
                {DOVDD, Vol_1800, 10},
                {DVDD,    Vol_1500, 10},
                {AFVDD, Vol_2800, 10},
                {RST,    Vol_High, 10},
                {PDN,    Vol_High, 10},
            },
        },
    #endif

    #if defined(OV8865_MIPI_RAW)
	    {SENSOR_DRVNAME_OV8865_MIPI_RAW,
		   {
				{SensorId, MAIN_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST, Vol_Low, 0},
				{PDN, Vol_Low, 0},
				{DVDD, Vol_1200, 10},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{AFVDD, Vol_2800, 10},
				{RST, Vol_High, 10},
				{PDN, Vol_High, 10},
		   },
	    },
    #endif

    #if defined(S5K3M2_MIPI_RAW)
	    {SENSOR_DRVNAME_S5K3M2_MIPI_RAW,
		   {
				{SensorId, MAIN_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST, Vol_Low, 0},
				{PDN, Vol_Low, 0},
				{DVDD, Vol_1000, 10},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{AFVDD, Vol_2800, 10},
				{RST, Vol_High, 10},
				{PDN, Vol_High, 10},
		   },
	    },
    #endif

    #if defined(S5K3M2_MIPI_RAW_YD)
	    {SENSOR_DRVNAME_S5K3M2_MIPI_RAW_YD,
		   {
				{SensorId, MAIN_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST, Vol_Low, 0},
				{PDN, Vol_Low, 0},
				{DVDD, Vol_1000, 10},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{AFVDD, Vol_2800, 10},
				{RST, Vol_High, 10},
				{PDN, Vol_High, 10},
		   },
	    },
    #endif

    #if defined(S5K3M2_MIPI_RAW_SEASONS)
	    {SENSOR_DRVNAME_S5K3M2_MIPI_RAW_SEASONS,
		   {
				{SensorId, MAIN_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST, Vol_Low, 0},
				{PDN, Vol_Low, 0},
				{DVDD, Vol_1000, 10},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{AFVDD, Vol_2800, 10},
				{RST, Vol_High, 10},
				{PDN, Vol_High, 10},
		   },
	    },
    #endif

    #if defined(S5K3M2_MIPI_RAW_QT)
	    {SENSOR_DRVNAME_S5K3M2_MIPI_RAW_QT,
		   {
				{SensorId, MAIN_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST, Vol_Low, 0},
				{PDN, Vol_Low, 0},
				{DVDD, Vol_1000, 10},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{AFVDD, Vol_2800, 10},
				{RST, Vol_High, 10},
				{PDN, Vol_High, 10},
		   },
	    },
    #endif

    #if defined(S5K4H8_MIPI_RAW)
	    {SENSOR_DRVNAME_S5K4H8_MIPI_RAW,
		   {
				{SensorId, SUB_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST, Vol_Low, 0},
				{PDN, Vol_Low, 0},
				{DVDD, Vol_1200, 10},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{AFVDD, Vol_2800, 10},
				{RST, Vol_High, 10},
				{PDN, Vol_High, 10},
		   },
	    },
    #endif
    #if defined(OV5670_MIPI_RAW)
        {SENSOR_DRVNAME_OV5670_MIPI_RAW,
            {
                {SensorId, SUB_SENSOR, 0},
                {SensorMCLK, Mclk1, 0},
                {RST,    Vol_Low,  0},
                {PDN,    Vol_Low,  0},
                {AVDD,    Vol_2800, 10},
                {DOVDD, Vol_1800, 10},
                {DVDD,    Vol_1200, 10},
                //{AFVDD, Vol_2800, 10},
                {RST,    Vol_High, 10},
                {PDN,    Vol_High, 10},
            },
        },
    #endif
	#if defined(SP2509_MIPI_RAW)
		{SENSOR_DRVNAME_SP2509_MIPI_RAW,
            {
				{SensorId, SUB_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST,	Vol_Low,  0},
				{PDN,	Vol_High,  0},
				{AVDD,	Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{DVDD,	Vol_1500, 10},
				//{AFVDD, Vol_2800, 10},
				{RST,	Vol_High, 10},
				{PDN,	Vol_Low, 10},
				{PDN,	Vol_High, 20},
				{PDN,	Vol_Low, 10},
            },
        },
	#endif

    #if defined(OV8858_MIPI_RAW)
		{SENSOR_DRVNAME_OV8858_MIPI_RAW,
            {
				{SensorId, SUB_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST,	Vol_Low,  0},
				{PDN,	Vol_Low,  0},
				{AVDD,	Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{DVDD,	Vol_1200, 10},
				//{AFVDD, Vol_2800, 10},
				{RST,	Vol_High, 10},
				{PDN,	Vol_High, 10},
            },
        },
	#endif
    #if defined(OV8856_MIPI_RAW)
		{SENSOR_DRVNAME_OV8856_MIPI_RAW,
            {
				{SensorId, SUB_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST,	Vol_Low,  0},
				{PDN,	Vol_Low,  0},
				{AVDD,	Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{DVDD,	Vol_1200, 10},
				//{AFVDD, Vol_2800, 10},
				{RST,	Vol_High, 10},
				{PDN,	Vol_High, 10},
            },
        },
	#endif
    #if defined(GC2755MIPI_RAW)
		{SENSOR_DRVNAME_GC2755_MIPI_RAW,
            {
				{SensorId, SUB_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST,	Vol_Low,  0},
				{PDN,	Vol_High,  0},
				{AVDD,	Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{DVDD,	Vol_1500, 10},
				//{AFVDD, Vol_2800, 10},
				{RST,	Vol_High, 10},
				{PDN,	Vol_Low, 10},
				{PDN,	Vol_High, 20},
				{PDN,	Vol_Low, 10},
            },
        },
	#endif
    #if defined(GC2355_MIPI_RAW)
		{SENSOR_DRVNAME_GC2355_MIPI_RAW,
            {
				{SensorId, SUB_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST,	Vol_Low,  0},
				{PDN,	Vol_High,  0},
				{AVDD,	Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{DVDD,	Vol_1500, 10},
				//{AFVDD, Vol_2800, 10},
				{RST,	Vol_High, 10},
				{PDN,	Vol_Low, 10},
				{PDN,	Vol_High, 20},
				{PDN,	Vol_Low, 10},
            },
        },
	#endif
    #if defined(S5K5E2YA_MIPI_RAW)
		{SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW,
            {
				{SensorId, SUB_SENSOR, 0},
				{SensorMCLK, Mclk1, 0},
				{RST,	Vol_Low,  0},
				{PDN,	Vol_Low,  0},
				{AVDD,	Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{DVDD,	Vol_1500, 10},
				//{AFVDD, Vol_2800, 10},
				{RST,	Vol_High, 10},
				{PDN,	Vol_High, 10},
            },
        },
	#endif

        {NULL,},
    }
};

static u32 pinSetIdx = 0;//default main sensor
/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam_ldo0_h = NULL; //CAMA
struct pinctrl_state *cam_ldo0_l = NULL; //CAMA
struct pinctrl_state *cam_ldo1_h = NULL; //CAMD
struct pinctrl_state *cam_ldo1_l = NULL; //CAMD

int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}

	/*Cam0 Power/Rst Ping initialization */
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		PK_ERR("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}
	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		PK_ERR("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}

	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		PK_ERR("%s : pinctrl err, cam0_rst_h\n", __func__);
	}
	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		PK_ERR("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/*Cam1 Power/Rst Ping initialization */
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		PK_ERR("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}
	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l )) {
		ret = PTR_ERR(cam1_pnd_l );
		PK_ERR("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}

	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		PK_ERR("%s : pinctrl err, cam1_rst_h\n", __func__);
	}
	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		PK_ERR("%s : pinctrl err, cam1_rst_l\n", __func__);
	}

	/*externel LDO enable */
#ifdef GPIO_VCAMA_LDO
	cam_ldo0_h = pinctrl_lookup_state(camctrl, "cam_ldo0_1");
	if (IS_ERR(cam_ldo0_h)) {
		ret = PTR_ERR(cam_ldo0_h);
		PK_ERR("%s : pinctrl err, cam_ldo0_h\n", __func__);
	}
	cam_ldo0_l = pinctrl_lookup_state(camctrl, "cam_ldo0_0");
	if (IS_ERR(cam_ldo0_l)) {
		ret = PTR_ERR(cam_ldo0_l);
		PK_ERR("%s : pinctrl err, cam_ldo0_l\n", __func__);
	}
#endif
#ifdef GPIO_VCAMD_LDO
	cam_ldo1_h = pinctrl_lookup_state(camctrl, "cam_ldo1_1");
	if (IS_ERR(cam_ldo1_h)) {
		ret = PTR_ERR(cam_ldo1_h);
		PK_ERR("%s : pinctrl err, cam_ldo1_h\n", __func__);
	}
	cam_ldo1_l = pinctrl_lookup_state(camctrl, "cam_ldo1_0");
	if (IS_ERR(cam_ldo1_l)) {
		ret = PTR_ERR(cam_ldo1_l);
		PK_ERR("%s : pinctrl err, cam_ldo1_l\n", __func__);
	}
#endif
	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;

	switch (PwrType) {
	case CAMRST:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		}
		break;

	case CAMPDN:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else
				pinctrl_select_state(camctrl, cam0_pnd_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else
				pinctrl_select_state(camctrl, cam1_pnd_h);
		}
		break;
#ifdef GPIO_VCAMA_LDO
	case CAMALDO:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo0_l);
		else
			pinctrl_select_state(camctrl, cam_ldo0_h);
		break;
#endif
#ifdef GPIO_VCAMD_LDO
	case CAMDLDO:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo1_l);
		else
			pinctrl_select_state(camctrl, cam_ldo1_h);
		break;
#endif
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);
	return ret;
}

BOOL hwpoweron(PowerInformation pwInfo, char* mode_name)
{
    if(pwInfo.PowerType == AVDD)
    {
    #ifdef GPIO_VCAMA_LDO
		mtkcam_gpio_set(pinSetIdx, CAMALDO, 1);
	#endif
		if(TRUE != _hwPowerOn(VCAMA,pwInfo.Voltage))
		{
			PK_ERR("[CAMERA AVDD] Fail to enable power\n");
			return FALSE;
		}
    }
    else if(pwInfo.PowerType == DVDD)
    {
    #ifdef GPIO_VCAMD_LDO
		mtkcam_gpio_set(pinSetIdx, CAMDLDO, 1);
	#endif
		if(TRUE != _hwPowerOn(VCAMD,pwInfo.Voltage))
		{
			PK_ERR("[CAMERA DVDD] Fail to enable power\n");
			return FALSE;
		}
    }
    else if(pwInfo.PowerType == DOVDD)
    {
		if(TRUE != _hwPowerOn(VCAMIO,pwInfo.Voltage))
		{
			PK_ERR("[CAMERA DOVDD] Fail to enable power\n");
			return FALSE;
		}
    }
    else if(pwInfo.PowerType == AFVDD)
    {
		if(TRUE != _hwPowerOn(VCAMAF,pwInfo.Voltage))
		{
			PK_ERR("[CAMERA AFVDD] Fail to enable power\n");
			return FALSE;
		}
    }
    else if(pwInfo.PowerType==PDN)
    {
        if(pwInfo.Voltage == Vol_High)
        {
			mtkcam_gpio_set(pinSetIdx, CAMPDN, 1);
        }
        else
        {
			mtkcam_gpio_set(pinSetIdx, CAMPDN, 0);
        }
    }
    else if(pwInfo.PowerType==RST)
    {
        if(pwInfo.Voltage == Vol_High)
        {
			mtkcam_gpio_set(pinSetIdx, CAMRST, 1);
        }
        else
        {
			mtkcam_gpio_set(pinSetIdx, CAMRST, 0);
        }
    }
    else if(pwInfo.PowerType==SensorMCLK)
    {
        if(pwInfo.Voltage == Mclk1)
        {
            PK_DBG("Sensor MCLK1 On");
            ISP_MCLK1_EN(TRUE);
        }
        else if(pwInfo.Voltage == Mclk2)
        {
            PK_DBG("Sensor MCLK2 On");
            ISP_MCLK2_EN(TRUE);
        }
    }
    else
	{
	}

    if(pwInfo.Delay>0)
        mdelay(pwInfo.Delay);

    return TRUE;
}

BOOL hwpowerdown(PowerInformation pwInfo, char* mode_name)
{
    if(pwInfo.PowerType == AVDD)
    {
    #ifdef GPIO_VCAMA_LDO
		mtkcam_gpio_set(pinSetIdx, CAMALDO, 0);
	#endif
		if(TRUE != _hwPowerDown(VCAMA))
		{
			PK_ERR("[CAMERA AVDD] Fail to disable power\n");
			return FALSE;
		}
    }
    else if(pwInfo.PowerType == DVDD)
    {
    #ifdef GPIO_VCAMD_LDO
		mtkcam_gpio_set(pinSetIdx, CAMDLDO, 0);
	#endif
		if(TRUE != _hwPowerDown(VCAMD))
		{
			PK_ERR("[CAMERA DVDD] Fail to disable power\n");
			return FALSE;
		}
    }
    else if(pwInfo.PowerType == DOVDD)
    {
		if(TRUE != _hwPowerDown(VCAMIO))
		{
			PK_ERR("[CAMERA DOVDD] Fail to disable power\n");
			return FALSE;
		}
    }
    else if(pwInfo.PowerType == AFVDD)
    {
		if(TRUE != _hwPowerDown(VCAMAF))
		{
			PK_ERR("[CAMERA AFVDD] Fail to disable power\n");
			return FALSE;
		}
    }
    else if(pwInfo.PowerType==PDN)
    {
        if(pwInfo.Voltage == Vol_High)
        {
			mtkcam_gpio_set(pinSetIdx, CAMPDN, 1);
        }
        else
        {
			mtkcam_gpio_set(pinSetIdx, CAMPDN, 0);
        }
    }
    else if(pwInfo.PowerType==RST)
    {
        if(pwInfo.Voltage == Vol_High)
        {
			mtkcam_gpio_set(pinSetIdx, CAMRST, 1);
        }
        else
        {
			mtkcam_gpio_set(pinSetIdx, CAMRST, 0);
        }
    }
    else if(pwInfo.PowerType==SensorMCLK)
    {
        if(pwInfo.Voltage == Mclk1)
        {
            PK_DBG("Sensor MCLK1 Off");
			ISP_MCLK1_EN(FALSE);
        }
        else if(pwInfo.Voltage == Mclk2)
        {
			PK_DBG("Sensor MCLK2 Off");
            ISP_MCLK2_EN(FALSE);
        }
    }
    else
	{
	}

    return TRUE;
}

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
    int pwListIdx,pwIdx;
    BOOL sensorInPowerList = KAL_FALSE;

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }

    //power ON
    if (On) {
        PK_ERR("kdCISModulePowerOn :currSensorName=%s,pinSetIdx=%d\n",currSensorName,pinSetIdx);

		for(pwListIdx=0; pwListIdx<16; pwListIdx++)
        {
            if(currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
            {
                PK_DBG("kdCISModulePowerOn get in---sensorIdx:%d\n",SensorIdx);

                sensorInPowerList = KAL_TRUE;

                for(pwIdx=1; pwIdx<15; pwIdx++)
                {
                    if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType != VDD_None)
                    {
					#ifdef AEON_CAM_SUPPORT //sanford.lin
                        if((PowerOnList.PowerSeq[pwListIdx].PowerInfo[0].PowerType == SensorId) && (PowerOnList.PowerSeq[pwListIdx].PowerInfo[0].Voltage != pinSetIdx))
						{
							PK_ERR("kdCISModulePowerOn %s is not %s\n",PowerOnList.PowerSeq[pwListIdx].SensorName, pinSetIdx ? "SUB_SENSOR":"MAIN_SENSOR");
							return ((int)SensorIdx << 1); //goto _kdCISModulePowerOn_exit_;
						}
					#endif
						if(hwpoweron(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx],mode_name)==FALSE)
                            goto _kdCISModulePowerOn_exit_;
                    }
                    else
                    {
                        PK_DBG("kdCISModulePowerOn pwIdx=%d\n",pwIdx);
                        break;
                    }
                }
                break;
            }
            else if(PowerOnList.PowerSeq[pwListIdx].SensorName == NULL)
            {
                break;
            }
            else
			{
			}
        }

		// Temp solution: default power on/off sequence
        if(KAL_FALSE == sensorInPowerList)
		{
			//PK_DBG("Default power on sequence");
			// add ..
			PK_ERR("Pls. add power on sequence !!!");
			return ((int)SensorIdx << 1);
		}

	}
    else {//power OFF
        PK_ERR("kdCISModulePowerOff :currSensorName=%s,pinSetIdx=%d\n",currSensorName,pinSetIdx);

        for(pwListIdx=0; pwListIdx<16; pwListIdx++)
        {
            if(currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
            {
                PK_DBG("kdCISModulePowerOff get in---sensorIdx:%d\n",SensorIdx);

                sensorInPowerList = KAL_TRUE;

                for(pwIdx=14; pwIdx>0; pwIdx--)
                {
                    if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType != VDD_None)
                    {
					#ifdef AEON_CAM_SUPPORT //sanford.lin
					    if((PowerOnList.PowerSeq[pwListIdx].PowerInfo[0].PowerType == SensorId) && (PowerOnList.PowerSeq[pwListIdx].PowerInfo[0].Voltage != pinSetIdx))
						{
							PK_ERR("kdCISModulePowerOff %s is not %s\n",PowerOnList.PowerSeq[pwListIdx].SensorName,pinSetIdx ? "SUB_SENSOR":"MAIN_SENSOR");
							return ((int)SensorIdx << 1); //goto _kdCISModulePowerOn_exit_;
						}
					#endif
                        if(hwpowerdown(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx],mode_name)==FALSE)
                            goto _kdCISModulePowerOn_exit_;
                        if(pwIdx>0)
                        {
                            if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx-1].Delay > 0)
                                mdelay(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx-1].Delay);
                        }
                    }
                    else
                    {
                        PK_DBG("kdCISModulePowerOff pwIdx=%d \n",pwIdx);
                    }
                }
            }
            else if(PowerOnList.PowerSeq[pwListIdx].SensorName == NULL)
            {
                break;
            }
            else
			{
			}
        }

		// Temp solution: default power on/off sequence
        if(KAL_FALSE == sensorInPowerList)
		{
			//PK_DBG("Default power on sequence");
			// add ..
			PK_ERR("Pls. add power off sequence !!!");
			return ((int)SensorIdx << 1);
		}
	}
    return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);
