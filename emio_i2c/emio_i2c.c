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

#include "emio_i2c.h"

#define  GPIOPS_ID  XPAR_XGPIOPS_0_DEVICE_ID  //PS 端 GPIO 器件 ID

static  XGpioPs  gpiops_inst; //PS 端 GPIO 驱动实例

//EMIO初始化
void emio_init(void)
{

	XGpioPs_Config *gpiops_cfg_ptr; //PS 端 GPIO 配置信息

	//根据器件 ID 查找配置信息
	gpiops_cfg_ptr = XGpioPs_LookupConfig(GPIOPS_ID);
	//初始化器件驱动
	XGpioPs_CfgInitialize(&gpiops_inst, gpiops_cfg_ptr, gpiops_cfg_ptr->BaseAddr);

	//设置 gpio端口 为输出
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL0_NUM, 1);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA0_NUM, 1);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL1_NUM, 1);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA1_NUM, 1);

	//使能 gpio端口 输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL0_NUM, 1);
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA0_NUM, 1);
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL1_NUM, 1);
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA1_NUM, 1);

	//将SCLK和SDA都拉高
	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 1);
	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA0_NUM, 1);
	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 1);
	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA1_NUM, 1);
}

//产生起始信号
void i2c0_start(void)
{
	//设置 gpio端口 为输出
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL0_NUM, 1);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA0_NUM, 1);
	//使能 gpio端口 输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL0_NUM, 1);
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA0_NUM, 1);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 1);
	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA0_NUM, 1);

	usleep(4);

 	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA0_NUM, 0);  //START:when CLK is high,DATA change form high to low

 	usleep(4);

 	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);  //钳住I2C总线，准备发送或接收数据
}
void i2c1_start(void)
{
	//设置 gpio端口 为输出
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL1_NUM, 1);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA1_NUM, 1);
	//使能 gpio端口 输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL1_NUM, 1);
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA1_NUM, 1);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 1);
	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA1_NUM, 1);

	usleep(4);

 	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA1_NUM, 0);  //START:when CLK is high,DATA change form high to low

 	usleep(4);

 	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);  //钳住I2C总线，准备发送或接收数据
}
//产生停止信号
void i2c0_stop(void)
{

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);

	XGpioPs_WritePin(&gpiops_inst,EMIO_SDA0_NUM, 0);  //STOP:when CLK is high DATA change form low to high

 	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 1);

	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA0_NUM, 1);  //发送I2C总线结束信号

}
void i2c1_stop(void)
{

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);

	XGpioPs_WritePin(&gpiops_inst,EMIO_SDA1_NUM, 0);  //STOP:when CLK is high DATA change form low to high

 	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 1);

	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA1_NUM, 1);  //发送I2C总线结束信号

}

//发送一个字节
void i2c0_send_byte(u8 txd)
{
    u8 t;
    XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA0_NUM, 1);//SDA设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA0_NUM, 1);//使能SDA输出
    XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL0_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL0_NUM, 1);//使能SCL输出

    XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);  //拉低时钟开始数据传输

    for(t=0; t<8; t++)
    {
        XGpioPs_WritePin(&gpiops_inst, EMIO_SDA0_NUM, (txd&0x80)>>7);

        txd<<=1;

        usleep(2);

        XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 1);

        usleep(4);

        XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);

        usleep(2);
    }
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA0_NUM, 0);
    XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA0_NUM, 0);  //SDA设置为输入
}
void i2c1_send_byte(u8 txd)
{
    u8 t;
    XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA1_NUM, 1);//SDA设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA1_NUM, 1);//使能SDA输出
    XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL1_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL1_NUM, 1);//使能SCL输出

    XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);  //拉低时钟开始数据传输

    for(t=0; t<8; t++)
    {
        XGpioPs_WritePin(&gpiops_inst, EMIO_SDA1_NUM, (txd&0x80)>>7);

        txd<<=1;

        usleep(2);

        XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 1);

        usleep(4);

        XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);

        usleep(2);
    }
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA1_NUM, 0);
    XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA1_NUM, 0);  //SDA设置为输入
}

//接收一个字节
u8  i2c0_recv_byte(void)
{
	unsigned char i=0 , rxd=0;

	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA0_NUM, 0);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA0_NUM, 0);//SDA设置为输入
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL0_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL0_NUM, 1);//使能SCL输出
	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);
	usleep(4);

	for(i=0;i<8;i++ )
	{
        XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 1);
        usleep(2);

        rxd <<= 1;
        if( XGpioPs_ReadPin(&gpiops_inst, EMIO_SDA0_NUM) ) {
        	rxd = rxd | 0x01;
        }
		usleep(2);

		XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);
		usleep(4);
    }

	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA0_NUM, 1);  //SDA设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA0_NUM, 1);//使能SDA输出

    return rxd;

}
u8  i2c1_recv_byte(void)
{
	unsigned char i=0 , rxd=0;

	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA1_NUM, 0);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA1_NUM, 0);//SDA设置为输入
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL1_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL1_NUM, 1);//使能SCL输出
	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);
	usleep(4);

	for(i=0;i<8;i++ )
	{
        XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 1);
        usleep(2);

        rxd <<= 1;
        if( XGpioPs_ReadPin(&gpiops_inst, EMIO_SDA1_NUM) ) {
        	rxd = rxd | 0x01;
        }
		usleep(2);

		XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);
		usleep(4);
    }

	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA1_NUM, 1);  //SDA设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA1_NUM, 1);//使能SDA输出

    return rxd;

}

//产生ACK应答
void i2c0_ack(void)
{
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA0_NUM, 1);  //SDA设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA0_NUM, 1);//使能SDA输出
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL0_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL0_NUM, 1);//使能SCL输出

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);
	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA0_NUM, 0);

	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 1);

	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);

	usleep(4);
}
void i2c1_ack(void)
{
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA1_NUM, 1);  //SDA设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA1_NUM, 1);//使能SDA输出
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL1_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL1_NUM, 1);//使能SCL输出

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);
	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA1_NUM, 0);

	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 1);

	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);

	usleep(4);
}

//产生NACK应答
void i2c0_nack(void)
{
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA0_NUM, 1);  //SDA设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA0_NUM, 1);//使能SDA输出
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL0_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL0_NUM, 1);//使能SCL输出

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);
	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA0_NUM, 1);

	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 1);

	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);

	usleep(4);
}
void i2c1_nack(void)
{
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA1_NUM, 1);  //SDA设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA1_NUM, 1);//使能SDA输出
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL1_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL1_NUM, 1);//使能SCL输出

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);
	XGpioPs_WritePin(&gpiops_inst, EMIO_SDA1_NUM, 1);

	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 1);

	usleep(4);

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);

	usleep(4);
}


u8  i2c0_recv_ack(void)
{
	u8 check;
	u16 ucErrTime=0;

	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA0_NUM, 0);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA0_NUM, 0);//SDA设置为输入
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL0_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL0_NUM, 1);//使能SCL输出
	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);

	usleep(10);

	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL0_NUM, 0);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL0_NUM, 0);//SCL设置为输入

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);
	while(XGpioPs_ReadPin(&gpiops_inst, EMIO_SCL0_NUM) == 0)
	{
      ucErrTime++;
      usleep(1);
      if(ucErrTime>1000000)
      {
        ucErrTime = 0;
        break;
      }
    }
    usleep(10);
	
	check = 0;
	if(XGpioPs_ReadPin(&gpiops_inst, EMIO_SDA0_NUM) == 1)
	{
		check = 1;
	}

	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL0_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL0_NUM, 1);//使能SCL输出
	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL0_NUM, 0);//拉低SCL
	usleep(2);

	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA0_NUM, 1);//SDA设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA0_NUM, 1);//使能SDA输出
	
	return check; 
}

u8  i2c1_recv_ack(void)
{
	u8 check;
	u16 ucErrTime=0;

	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA1_NUM, 0);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA1_NUM, 0);//SDA设置为输入
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL1_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL1_NUM, 1);//使能SCL输出
	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);

	usleep(10);

	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL1_NUM, 0);
	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL1_NUM, 0);//SCL设置为输入

	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);
	while(XGpioPs_ReadPin(&gpiops_inst, EMIO_SCL1_NUM) == 0)
	{
      ucErrTime++;
      usleep(1);
      if(ucErrTime>1000000)
      {
        ucErrTime = 0;
        break;
      }
    }
    usleep(10);

	check = 0;
	if(XGpioPs_ReadPin(&gpiops_inst, EMIO_SDA1_NUM) == 1)
	{
		check = 1;
	}

	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SCL1_NUM, 1);//SCL设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SCL1_NUM, 1);//使能SCL输出
	XGpioPs_WritePin(&gpiops_inst, EMIO_SCL1_NUM, 0);//拉低SCL
	usleep(2);

	XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_SDA1_NUM, 1);//SDA设置为输出
	XGpioPs_SetOutputEnablePin(&gpiops_inst, EMIO_SDA1_NUM, 1);//使能SDA输出

	return check;
}

int emio_i2c0_reg8_write(char IIC_ADDR, char Addr, char Data)
{
	u8 ack=0;

	i2c0_start();

	i2c0_send_byte(IIC_ADDR<<1);
	ack=i2c0_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	i2c0_send_byte(Addr);
	ack=i2c0_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	i2c0_send_byte(Data);
	ack=i2c0_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	i2c0_stop();

	return XST_SUCCESS;
}

int emio_i2c1_reg8_write(char IIC_ADDR, char Addr, char Data)
{
	u8 ack=0;

	i2c1_start();

	i2c1_send_byte(IIC_ADDR<<1);
	ack=i2c1_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	i2c1_send_byte(Addr);
	ack=i2c1_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	i2c1_send_byte(Data);
	ack=i2c1_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	i2c1_stop();

	return XST_SUCCESS;
}

int emio_i2c0_reg8_read(char IIC_ADDR, char Addr, u8 * ret)
{
	u8 rxd;
	u8 ack=0;

	i2c0_start();

	i2c0_send_byte(IIC_ADDR<<1);
	ack=i2c0_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	i2c0_send_byte(Addr);
	ack=i2c0_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	//i2c0_stop();

  	i2c0_start();

  	i2c0_send_byte(IIC_ADDR<<1 | 0x01);
  	ack=i2c0_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	rxd = i2c0_recv_byte();
	i2c0_nack();

  	i2c0_stop();

  	*ret = rxd;

  	return XST_SUCCESS;
}

int emio_i2c1_reg8_read(char IIC_ADDR, char Addr, u8 * ret)
{
	u8 rxd;
	u8 ack=0;

	i2c1_start();

	i2c1_send_byte(IIC_ADDR<<1);
	ack=i2c1_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	i2c1_send_byte(Addr);
	ack=i2c1_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	//i2c1_stop();

  	i2c1_start();

  	i2c1_send_byte(IIC_ADDR<<1 | 0x01);
  	ack=i2c1_recv_ack();
	if(ack)
	{
		return XST_FAILURE;
	}

	rxd = i2c1_recv_byte();
	i2c1_nack();

  	i2c1_stop();

  	*ret = rxd;

  	return XST_SUCCESS;
}

int emio_i2c0_reg16_write(char IIC_ADDR, unsigned short Addr, char Data)
{
	u8 ack=0;

	i2c0_start();

	i2c0_send_byte(IIC_ADDR<<1);
	ack=i2c0_recv_ack();
	if(ack)
	{
		i2c0_stop();
		return XST_FAILURE;
	}

	i2c0_send_byte(Addr >> 8);
	ack=i2c0_recv_ack();
	if(ack)
	{
		i2c0_stop();
		return XST_FAILURE;
	}

	i2c0_send_byte(Addr & 0x00FF);
	ack=i2c0_recv_ack();
	if(ack)
	{
		i2c0_stop();
		return XST_FAILURE;
	}

	i2c0_send_byte(Data);
	ack=i2c0_recv_ack();
	if(ack)
	{
		i2c0_stop();
		return XST_FAILURE;
	}

	i2c0_stop();

	return XST_SUCCESS;
}

int emio_i2c1_reg16_write(char IIC_ADDR, unsigned short Addr, char Data)
{
	u8 ack=0;

	i2c1_start();

	i2c1_send_byte(IIC_ADDR<<1);
	ack=i2c1_recv_ack();
	if(ack)
	{
		i2c1_stop();
		return XST_FAILURE;
	}

	i2c1_send_byte(Addr >> 8);
	ack=i2c1_recv_ack();
	if(ack)
	{
		i2c1_stop();
		return XST_FAILURE;
	}

	i2c1_send_byte(Addr & 0x00FF);
	ack=i2c1_recv_ack();
	if(ack)
	{
		i2c1_stop();
		return XST_FAILURE;
	}

	i2c1_send_byte(Data);
	ack=i2c1_recv_ack();
	if(ack)
	{
		i2c1_stop();
		return XST_FAILURE;
	}

	i2c1_stop();

	return XST_SUCCESS;
}

int emio_i2c0_reg16_read(char IIC_ADDR, unsigned short Addr, u8 * ret)
{
	u8 rxd;
	u8 ack=0;

	i2c0_start();

	i2c0_send_byte(IIC_ADDR<<1);
//	i2c0_ack();
	ack=i2c0_recv_ack();
	if(ack)
	{
		i2c0_stop();
		return XST_FAILURE;
	}

	i2c0_send_byte(Addr >> 8);
//	i2c0_ack();
	ack=i2c0_recv_ack();
	if(ack)
	{
		i2c0_stop();
		return XST_FAILURE;
	}

	i2c0_send_byte(Addr & 0x00FF);
//	i2c0_ack();
	ack=i2c0_recv_ack();
	if(ack)
	{
		i2c0_stop();
		return XST_FAILURE;
	}

	//i2c0_stop();

  	i2c0_start();

  	i2c0_send_byte(IIC_ADDR<<1 | 0x01);
//  	i2c0_ack();
  	ack=i2c0_recv_ack();
	if(ack)
	{
		i2c0_stop();
		return XST_FAILURE;
	}

	rxd = i2c0_recv_byte();
	i2c0_nack();
//	i2c0_ack();

  	i2c0_stop();

  	*ret = rxd;

  	return  XST_SUCCESS ;
}

int emio_i2c1_reg16_read(char IIC_ADDR, unsigned short Addr, u8 * ret)
{
	u8 rxd;
	u8 ack=0;

	i2c1_start();

	i2c1_send_byte(IIC_ADDR<<1);
//	i2c1_ack();
	ack=i2c1_recv_ack();
	if(ack)
	{
		i2c1_stop();
		return XST_FAILURE;
	}

	i2c1_send_byte(Addr >> 8);
//	i2c1_ack();
	ack=i2c1_recv_ack();
	if(ack)
	{
		i2c1_stop();
		return XST_FAILURE;
	}

	i2c1_send_byte(Addr & 0x00FF);
//	i2c1_ack();
	ack=i2c1_recv_ack();
	if(ack)
	{
		i2c1_stop();
		return XST_FAILURE;
	}

	//i2c1_stop();

  	i2c1_start();

  	i2c1_send_byte(IIC_ADDR<<1 | 0x01);
//  	i2c1_ack();
  	ack=i2c1_recv_ack();
	if(ack)
	{
		i2c1_stop();
		return XST_FAILURE;
	}

	rxd = i2c1_recv_byte();
	i2c1_nack();
//	i2c1_ack();

  	i2c1_stop();

  	*ret = rxd;

  	return  XST_SUCCESS ;
}
