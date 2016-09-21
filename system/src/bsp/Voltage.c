/*
 * MCU Voltage Detection driver
 *
 * ֧�����ֲο���ѹ���л�����֧�ָ���Ĳ�����Χ
 * ���л�ֻ��һ�Σ����Ǽ�����һ���ϵ��Ĳ���������Զ�ǴӸߵ���
 *
 * @author CaoHua
 *
 */

#include "Voltage.h"

/**
 * ����ʹ��1���ֽڱ�ʾ,���λΪ�ο���ѹ,��7λΪ��������ֵ.
 */
#define PWR_REF_2_5V  0x80        // �ο���ѹ2.5v
#define PWR_REF_1_5V  0x00        // �ο���ѹ1.5v
#define PWR_VAL_MASK  0x7F

/* �����ߵ���ֵ */
#define POWER_LOW 2.4f
#define POWER_HIGH 3.8f

/**
 * �����趨���л��ο���ѹ����ֵΪ 2.8v
 * ���ο���ѹ��Ϊ2.5v������Ŀ��Ϊ2.8vʱ����Ӧԭʼ������0x023C
 *
 * �ڳ�ʼ��ʱ���ο���ѹ��Զ��Ϊ2.5v
 * ������ص����� <= 0x023C�����Ǿͽ��ο���ѹ�л���1.5v���¶���
 */
 //TODO caohua ���ֵ���Ȳ�������Ҫ�����㣬����Ľ����0x0263
#define POWER_2_5V_THRESHOLD 0x023C

/**
 * ��ǰ�Ƿ�ʹ����Ĭ�ϵĲο���ѹ
 * 1����ǰʹ��Ĭ�ϲο���ѹ (2.5v)
 * 2: ��ǰʹ�õ�����һ�ο���ѹ (1.5v)
 */
static char defaultRef = 1;

//TODO �Ƿ��Ҫ
__monitor static uint16_t detectBatteryMeter_BSP();

/**
 * ����VCC��ѹ
 *
 * @return����ѹֵ
 * power���λ��־�ο���ѹ:1:2.5v 0:1.5v
 */
uint16_t getVoltage_BSP(void) {
	uint16_t power = detectBatteryMeter_BSP();

	if(defaultRef) {
		if(power <= POWER_2_5V_THRESHOLD) {
                        //2014.01.19: liusf modified for debug
			/* ���Ĳο���ѹΪ1.5v */
			//ADC10CTL0 &= ~REF2_5V;
                        /* ���Ĳο���ѹΪ2.5v */
                        ADC10CTL0 |= REF2_5V;
                        //2014.01.19:liusf modified end

			/* �л��ο���ѹ��־ */
			defaultRef = 0;

			/* �ٴζ�����������1.5���*/
			power = detectBatteryMeter_BSP();

			//TODO YIMU 0x7FFFɶ��˼��
			//TODO Caohua �Ǹ�PC��������õ�
			power &= 0x7FFF;
		} else {

			//TODO YIMU 0x8000ɶ��˼��
			//TODO Caohua �Ǹ�PC��������õ�
//			power |= 0x8000;
		}
	} else {
		power &= 0x7FFF;
	}

	return power;
}

/**
 * ��ʼ������������ADת��ͨ����11ͨ������VCC��ѹ
 */
void initPower_BSP(void) {
	/* ADC����:16 x ADC10CLKs, V(+)=V(Ref),REF on, V(Ref)=2.5V */
	ADC10CTL0 = SREF_1 | ADC10SHT_2 | REFON | REF2_5V;

	/* ͨ��ѡ��A11,ʹ��MSP430�ڲ��Դ��ĵ�·����� */
	ADC10CTL1 = INCH_11;
}

/**
 * ���������̣�ʹ�õ�ǰ�����ã�����ԭʼ����
 * @return����ѹֵ
 * power���λ��־�ο���ѹ:1-2.5v 0-1.5v
 */
__monitor static uint16_t detectBatteryMeter_BSP() {



	uint16_t power = 0;

	/* ��AD,��ʼ���� */
	ADC10CTL0 |= REFON | ADC10ON | ENC | ADC10SC;

	/* �ȴ�ת������ */
	while (ADC10CTL1 & ADC10BUSY)
		;

	power = ADC10MEM;

	/* �رղο���Դ������ģ�鹦���Ա�ʡ��
	 * turn off A/D to save power */

	ADC10CTL0 &= ~(ENC | REFON | ADC10ON);

	return power;
}
