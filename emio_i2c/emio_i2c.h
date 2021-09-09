// --------------------------------------------------------------------
// Copyright (c) 2019 by MicroPhase Technologies Inc.
// --------------------------------------------------------------------
//
// Permission:
//
//   MicroPhase grants permission to use and modify this code for use
//   in synthesis for all MicroPhase Development Boards.
//   Other use of this code, including the selling
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  MicroPhase provides no warranty regarding the use
//   or functionality of this code.
//
// --------------------------------------------------------------------
//
//                     MicroPhase Technologies Inc
//                     Shanghai, China
//
//                     web: http://www.microphase.cn/
//                     email: support@microphase.cn
//
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//
// Major Functions:
//
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//
//  Revision History:
//  Date          By            Revision    Change	 Description
//---------------------------------------------------------------------
//2020-03-17      Wang 			1.0          		Original
//2020-                         1.1
// --------------------------------------------------------------------
// --------------------------------------------------------------------

#include "xgpiops.h"
#include "sleep.h"

#ifndef _EMIO_I2C_
#define _EMIO_I2C_

#define EMIO_SCL1_NUM 82
#define EMIO_SDA1_NUM 81

#define EMIO_SCL0_NUM 79
#define EMIO_SDA0_NUM 78

void emio_init(void);
void i2c0_start(void);
void i2c0_stop(void);
void i2c0_ack(void);
void i2c0_nack(void);
void i2c0_send_byte(u8 txd);
u8  i2c0_recv_byte(void);
u8  i2c0_recv_ack(void);
void i2c1_start(void);
void i2c1_stop(void);
void i2c1_ack(void);
void i2c1_nack(void);
void i2c1_send_byte(u8 txd);
u8  i2c1_recv_byte(void);
u8  i2c1_recv_ack(void);

int emio_i2c0_reg8_write(char IIC_ADDR, char Addr, char Data);
int emio_i2c0_reg8_read(char IIC_ADDR, char Addr, u8 * ret);
int emio_i2c0_reg16_write(char IIC_ADDR, unsigned short Addr, char Data);
int emio_i2c0_reg16_read(char IIC_ADDR, unsigned short Addr, u8 * ret);
int emio_i2c1_reg8_write(char IIC_ADDR, char Addr, char Data);
int emio_i2c1_reg8_read(char IIC_ADDR, char Addr, u8 * ret);
int emio_i2c1_reg16_write(char IIC_ADDR, unsigned short Addr, char Data);
int emio_i2c1_reg16_read(char IIC_ADDR, unsigned short Addr, u8 * ret);


#endif /* _EMIO_I2C_ */
