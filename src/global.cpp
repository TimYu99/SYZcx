// global.cpp

#include "global.h"

// ȫ�ֱ�������
uint16_t globalPn = 0;
uint16_t globalSn = 0;
uint8_t globalstatus = 0x0F;
uint16_t globalangle = 0;
uint8_t globalspeed = 0;
uint8_t globalminrange = 0;
uint8_t globalmaxrange = 0;
int globalcount = 0;
int globalx = 0;
int count_0_9999;
int count_10000_19999;
int count_20000_29999;
int count_30000_39999;
int count_40000_49999;
int count_50000_59999;
int count_60000_65535;

int Accumulate;
int thereold;
char sendBuffer[256]="";
// ȫ�ֱ�������Ŀ���б�ʱ�ı�־�ͳ�ʼ��
int flag_shanxing = 0;   // Ĭ�Ϲر�
int flag_target = 0;     // Ĭ��δ��⵽Ŀ��
int flag_zhiling = 1;    // Ĭ��δִ��ָ��
//������
int jishu_shanxing =0;