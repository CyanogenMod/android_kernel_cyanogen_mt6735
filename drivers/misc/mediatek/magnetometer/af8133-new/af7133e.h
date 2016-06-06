/* include/linux/af7133E.h - AF7133 compass driver
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Definitions for af7133 compass chip.
 */
#ifndef AF7133E_AF8133I_H
#define AF7133E_AF8133I_H

#include <linux/ioctl.h>

//#define AF7133E_AF8133I_I2C_ADDRESS 	0x1E  //7-bit address 
#define AF7133E_AF8133I_I2C_ADDRESS 	0x1C  //7-bit address 
#define AF7133E_AF8133I_BUFSIZE      6


/*AF7133 register address*/
#define AF7133E_AF8133I_REG_PCODE  	0x00
#define AF7133E_AF8133I_REG_STATUS	0x02
#define AF7133E_AF8133I_REG_DATA  	0x03
#define AF7133E_AF8133I_REG_STATE	0x0A
#define AF7133E_AF8133I_REG_RANGE	0x0B
#define AF7133E_AF8133I_REG_SWR		0x11


/*AF7133 parameters*/
#define AF7133E_AF8133I_MODE_IDLE  	0x00
#define AF7133E_AF8133I_MODE_SINGLE   	0x01
#define AF7133E_AF8133I_MODE_WAKE  0x02

#define AF7133E_AF8133I_PCODE_VALUE  0x50

// conversion of magnetic data (for AF7133) to uT units
#define CONVERT_M			      36
#define CONVERT_M_DIV		    1000		

// conversion of orientation data to degree units
#define CONVERT_O			      1
#define CONVERT_O_DIV		    200



#endif

