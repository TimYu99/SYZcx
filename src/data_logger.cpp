#ifndef _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_DEPRECATE // ����unsafe
#endif // !_CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include <time.h>
#include <string.h>

void saveData(const char* dataFilename, const char* data, size_t dataSize, const char* direction, int isHex) {
    // �������ļ�����׷��д��
    FILE* file = fopen(dataFilename, "a");
    if (file == NULL) {
        printf("�޷����ļ� %s ����д��\n", dataFilename);
        return;
    }

    // ��ȡ��ǰʱ��
    time_t now;
    time(&now);
    struct tm local;
    localtime_s(&local, &now);
    // д��ʱ����ͷ���
    fprintf(file, "[%04d-%02d-%02d %02d:%02d:%02d] %s\n",
        local.tm_year + 1900, local.tm_mon + 1, local.tm_mday,
        local.tm_hour, local.tm_min, local.tm_sec, direction);
    // д������
    if (isHex) {
        for (size_t i = 0; i < dataSize; ++i) {
            fprintf(file, "%02X ", (unsigned char)data[i]);
        }
    }
    else {
        for (size_t i = 0; i < dataSize; ++i) {
            fprintf(file, "%c", data[i]);
        }
    }




    // �����Էָ�ÿ��׷�ӵ�����
    fprintf(file, "\n");
    fclose(file);
    //printf("���ݳɹ����浽 %s\n", dataFilename);
}
