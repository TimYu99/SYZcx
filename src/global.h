#ifndef GLOBAL_H
#define GLOBAL_H

#include <cstdint> // ���� uint16_t �Ķ���

// ����ȫ�ֱ���
extern uint16_t globalPn;
extern uint16_t globalSn;
extern uint8_t globalstatus;
extern uint16_t globalangle;
extern uint8_t globalspeed;
extern uint8_t globalminrange;
extern uint8_t globalmaxrange;
extern int globalx;
extern int globalcount;
extern int count_0_9999;
extern int count_10000_19999;
extern int count_20000_29999;
extern int count_30000_39999;
extern int count_40000_49999;
extern int count_50000_59999;
extern int count_60000_65535;
extern int Accumulate;
extern int thereold;
extern char sendBuffer[256];
// ��־λ
extern int flag_shanxing;   // ��־λ 1: �������δ���, 0: ����
extern int flag_target;     // ��־λ 1: ��⵽Ŀ��, 0: δ��⵽
extern int flag_zhiling;    // ��־λ 1: ִ��ĳָ��, 0: δִ��ָ��
//������
extern int jishu_shanxing;
#endif // GLOBAL_H