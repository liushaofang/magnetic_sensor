/*
 * MCU Voltage Detection driver
 *
 * 支持两种参考电压的切换，以支持更大的测量范围
 * 自切换只做一次，我们假设在一次上电后的测量过程永远是从高到低
 *
 * @author CaoHua
 *
 */

#include "Voltage.h"

/**
 * 电量使用1个字节表示,最高位为参考电压,低7位为真正的数值.
 */
#define PWR_REF_2_5V  0x80        // 参考电压2.5v
#define PWR_REF_1_5V  0x00        // 参考电压1.5v
#define PWR_VAL_MASK  0x7F

/* 电量高低阈值 */
#define POWER_LOW 2.4f
#define POWER_HIGH 3.8f

/**
 * 我们设定的切换参考电压的域值为 2.8v
 * 当参考电压设为2.5v，测量目标为2.8v时，对应原始数据是0x023C
 *
 * 在初始化时，参考电压永远设为2.5v
 * 如果读回的数据 <= 0x023C，我们就将参考电压切换到1.5v重新读数
 */
 //TODO caohua 这个值精度不够，需要重新算，我算的结果是0x0263
#define POWER_2_5V_THRESHOLD 0x023C

/**
 * 当前是否使用了默认的参考电压
 * 1：当前使用默认参考电压 (2.5v)
 * 2: 当前使用的是另一参考电压 (1.5v)
 */
static char defaultRef = 1;

//TODO 是否必要
__monitor static uint16_t detectBatteryMeter_BSP();

/**
 * 读出VCC电压
 *
 * @return：电压值
 * power最高位标志参考电压:1:2.5v 0:1.5v
 */
uint16_t getVoltage_BSP(void) {
	uint16_t power = detectBatteryMeter_BSP();

	if(defaultRef) {
		if(power <= POWER_2_5V_THRESHOLD) {
                        //2014.01.19: liusf modified for debug
			/* 更改参考电压为1.5v */
			//ADC10CTL0 &= ~REF2_5V;
                        /* 更改参考电压为2.5v */
                        ADC10CTL0 |= REF2_5V;
                        //2014.01.19:liusf modified end

			/* 切换参考电压标志 */
			defaultRef = 0;

			/* 再次读数，并打上1.5标记*/
			power = detectBatteryMeter_BSP();

			//TODO YIMU 0x7FFF啥意思？
			//TODO Caohua 是给PC端做标记用的
			power &= 0x7FFF;
		} else {

			//TODO YIMU 0x8000啥意思？
			//TODO Caohua 是给PC端做标记用的
//			power |= 0x8000;
		}
	} else {
		power &= 0x7FFF;
	}

	return power;
}

/**
 * 初始化函数，设置AD转换通道，11通道测量VCC电压
 */
void initPower_BSP(void) {
	/* ADC设置:16 x ADC10CLKs, V(+)=V(Ref),REF on, V(Ref)=2.5V */
	ADC10CTL0 = SREF_1 | ADC10SHT_2 | REFON | REF2_5V;

	/* 通道选择A11,使用MSP430内部自带的电路测电量 */
	ADC10CTL1 = INCH_11;
}

/**
 * 基本检测过程，使用当前的设置，读出原始数据
 * @return：电压值
 * power最高位标志参考电压:1-2.5v 0-1.5v
 */
__monitor static uint16_t detectBatteryMeter_BSP() {



	uint16_t power = 0;

	/* 打开AD,开始测量 */
	ADC10CTL0 |= REFON | ADC10ON | ENC | ADC10SC;

	/* 等待转换结束 */
	while (ADC10CTL1 & ADC10BUSY)
		;

	power = ADC10MEM;

	/* 关闭参考电源、屏蔽模块功能以便省电
	 * turn off A/D to save power */

	ADC10CTL0 &= ~(ENC | REFON | ADC10ON);

	return power;
}
