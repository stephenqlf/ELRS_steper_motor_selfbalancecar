#ifndef CRSF_PARSE_H
#define CRSF_PARSE_H
#include "main.h"  // 根据实际芯片修改

/*
bilibili 小努班 UID:437280309
@time时间: 2025.8.25
@version版本:V1_0
@Encoding :UTF-8
@attention:
具体详细操作，看视频或文档,注意波特率。
*/

// CRSF协议常量
#define CRSF_SYNC_BYTE 0xC8     //同步字节(穿越机CRSF常用)
#define CRSF_LENGTH_BYTE 0x18     //长度字节
#define CRSF_Type_BYTE 0x16     //类型字节
#define CRSF_CHANNELS_COUNT 16	//通道数量
#define CRSF_CRC_POLY 0xD5		//CRC校验多项式
#define CRSF_DMA_BUF_SIZE 128   //DMA接收缓冲区

// 数据结构
typedef struct {
    uint8_t sync;		//实际的同步字节
    uint8_t length;  // 实际的类型+负载的总长度
    uint8_t crc;	//CRC值
    uint8_t connected;//连接状态
    uint16_t CH[CRSF_CHANNELS_COUNT];  // 通道值范围172-1811
} CRSF_Packet;

extern CRSF_Packet CRSF_RX_packet;//数据解析后你想查看可以通过它

//////////////////函数声明/////////////////
void CRSF_Init(UART_HandleTypeDef *huart);//初始化串口RXDMA接收中断
uint8_t CRSF_ProcessIdleIRQ(void);  //(在你的空闲中断里调用它)空闲中断处理

void CRSF_Debug(void);//检查到底怎么个事

#endif
