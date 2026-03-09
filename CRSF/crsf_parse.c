#include "crsf_parse.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"

CRSF_Packet CRSF_RX_packet;//解析器结构体
static UART_HandleTypeDef *crsf_huart;
static uint8_t dma_rx_buf[CRSF_DMA_BUF_SIZE];//DMA接收缓冲区

//首先你先在CRSF_Init里面，初始化你单片机的串口接收中断
//然后修改这个代码就可以移植，你只要提供一个，串口DMA接收空闲中断就好
#define CRSF_USART_IDLE_RX_DMA(rx_buffer,bufer_len) HAL_UARTEx_ReceiveToIdle_DMA(crsf_huart, rx_buffer, bufer_len)


//CRC-8 DVB-S2计算（逐位处理）
static uint8_t calcCRC(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        // 处理每个字节的8位
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {  // 最高位为1时移位并异或多项式
                crc = (crc << 1) ^ 0xD5;
            } else {  // 最高位为0时仅移位
                crc <<= 1;
            }
        }
    }
    return crc;
}
//解析通道数据
static void CRSF_ParseChannel() {
	// 按位提取 16 个通道的 11 位数据（与之前逻辑一致，仅数据源改为 dma_rx_buf）
	CRSF_RX_packet.CH[0]  = ((int16_t)dma_rx_buf[ 3] >> 0 | ((int16_t)dma_rx_buf[ 4] << 8 )) & 0x07FF;
	CRSF_RX_packet.CH[1]  = ((int16_t)dma_rx_buf[ 4] >> 3 | ((int16_t)dma_rx_buf[ 5] << 5 )) & 0x07FF;
	CRSF_RX_packet.CH[2]  = ((int16_t)dma_rx_buf[ 5] >> 6 | ((int16_t)dma_rx_buf[ 6] << 2 ) | (int16_t)dma_rx_buf[ 7] << 10 ) & 0x07FF;
	CRSF_RX_packet.CH[3]  = ((int16_t)dma_rx_buf[ 7] >> 1 | ((int16_t)dma_rx_buf[ 8] << 7 )) & 0x07FF;
	CRSF_RX_packet.CH[4]  = ((int16_t)dma_rx_buf[ 8] >> 4 | ((int16_t)dma_rx_buf[ 9] << 4 )) & 0x07FF;
	CRSF_RX_packet.CH[5] = ((int16_t)dma_rx_buf[ 9] >> 7 | ((int16_t)dma_rx_buf[10] << 1 ) | (int16_t)dma_rx_buf[11] << 9 ) & 0x07FF;
	CRSF_RX_packet.CH[6] = ((int16_t)dma_rx_buf[11] >> 2 | ((int16_t)dma_rx_buf[12] << 6 )) & 0x07FF;
	CRSF_RX_packet.CH[7] = ((int16_t)dma_rx_buf[12] >> 5 | ((int16_t)dma_rx_buf[13] << 3 )) & 0x07FF;
	CRSF_RX_packet.CH[8] = ((int16_t)dma_rx_buf[14] << 0 | ((int16_t)dma_rx_buf[15] << 8 )) & 0x07FF;
	CRSF_RX_packet.CH[9] = ((int16_t)dma_rx_buf[15] >> 3 | ((int16_t)dma_rx_buf[16] << 5 )) & 0x07FF;
	CRSF_RX_packet.CH[10] = ((int16_t)dma_rx_buf[16] >> 6 | ((int16_t)dma_rx_buf[17] << 2 ) | (int16_t)dma_rx_buf[18] << 10 ) & 0x07FF;
	CRSF_RX_packet.CH[11] = ((int16_t)dma_rx_buf[18] >> 1 | ((int16_t)dma_rx_buf[19] << 7 )) & 0x07FF;
	CRSF_RX_packet.CH[12] = ((int16_t)dma_rx_buf[19] >> 4 | ((int16_t)dma_rx_buf[20] << 4 )) & 0x07FF;
	CRSF_RX_packet.CH[13] = ((int16_t)dma_rx_buf[20] >> 7 | ((int16_t)dma_rx_buf[21] << 1 ) | (int16_t)dma_rx_buf[22] << 9 ) & 0x07FF;
	CRSF_RX_packet.CH[14] = ((int16_t)dma_rx_buf[22] >> 2 | ((int16_t)dma_rx_buf[23] << 6 )) & 0x07FF;
	CRSF_RX_packet.CH[15] = ((int16_t)dma_rx_buf[23] >> 5 | ((int16_t)dma_rx_buf[24] << 3 )) & 0x07FF;
}

// 初始化CRSF（关联UART句柄）
void CRSF_Init(UART_HandleTypeDef *huart) {
    crsf_huart = huart;
    CRSF_RX_packet.connected = 0;
    for (uint8_t i = 0; i < CRSF_CHANNELS_COUNT; i++) {    // 初始化通道为0
    	CRSF_RX_packet.CH[i] = 0;
    }
    CRSF_USART_IDLE_RX_DMA(dma_rx_buf, CRSF_DMA_BUF_SIZE);//开启DMA转换
    __HAL_UART_ENABLE_IT(crsf_huart, UART_IT_IDLE);  // 使能空闲中断
}


void CRSF_ReStart_DMA() {
    CRSF_USART_IDLE_RX_DMA(dma_rx_buf, CRSF_DMA_BUF_SIZE);//开启DMA转换
}
/* return 0 同步字节错误
 * return 1 CRSF长度错误
 * return 2 类型错误
 * return 3 CRC校验错误
 * return 4 解析成功
 * */
uint8_t CRSF_ProcessIdleIRQ(void) {
  //计算数据包长度（针对16通道数据）
	CRSF_RX_packet.sync = dma_rx_buf[0];		//实际的同步字节
	if(CRSF_RX_packet.sync!=CRSF_SYNC_BYTE){//同步字节错误,直接返回
		CRSF_RX_packet.connected = 0;//连接失败
		CRSF_USART_IDLE_RX_DMA(dma_rx_buf, CRSF_DMA_BUF_SIZE);//开启DMA转换
		return 0;
	}
	if(dma_rx_buf[1] != CRSF_LENGTH_BYTE){
		CRSF_RX_packet.connected = 0;//连接失败
		CRSF_USART_IDLE_RX_DMA(dma_rx_buf, CRSF_DMA_BUF_SIZE);//开启DMA转换
		return 1;
	}
	if(dma_rx_buf[2] != CRSF_Type_BYTE){
		CRSF_RX_packet.connected = 0;//连接失败
		CRSF_USART_IDLE_RX_DMA(dma_rx_buf, CRSF_DMA_BUF_SIZE);//开启DMA转换
		return 2;
	}
	  CRSF_RX_packet.length = dma_rx_buf[1];  // 应为0x18（十进制24）
	  uint16_t expectedLength = CRSF_RX_packet.length + 2;  // 24 + 2 = 26字节
	  //计算CRC范围为类型字段到有效载荷末尾（共len_field-1字节）
	  uint8_t crc_data_len = CRSF_RX_packet.length - 1;  // 24 - 1 = 23字节（类型+22字节通道数据）
	  CRSF_RX_packet.crc = calcCRC(&dma_rx_buf[2], crc_data_len);  // 从类型字段（0x16）开始

	  //校验CRC（最后1字节为接收的CRC）
	  if (CRSF_RX_packet.crc == dma_rx_buf[expectedLength - 1]) {
		  CRSF_ParseChannel();//通道数据解析
		  CRSF_RX_packet.connected = 1;//连接成功
		  CRSF_USART_IDLE_RX_DMA(dma_rx_buf, CRSF_DMA_BUF_SIZE);//开启DMA转换
		  return 4;
	  }else{
		  CRSF_RX_packet.connected = 0;//连接失败
		  CRSF_USART_IDLE_RX_DMA(dma_rx_buf, CRSF_DMA_BUF_SIZE);//开启DMA转换
		  return 3;
	  }
}

void CRSF_Debug(void){
	//[起始字节] [长度] [类型] [负载数据] [CRC校验]
	//打印出来原始数据0xC8	0x17	0x16	[通道 1 数据]...[通道 16 数据]	CRC
	//////////输出原始数据///////////
	printf("%x,%x,%x,%x,%x,%x,%x,%x\n",dma_rx_buf[0],dma_rx_buf[1],dma_rx_buf[2],dma_rx_buf[3]
			,dma_rx_buf[4],dma_rx_buf[5],dma_rx_buf[6],dma_rx_buf[7]);
	//////////输出解析数据///////////
//	printf("%d,%d,%d,%d,%d,%d,%d,%d \r\n",CRSF_RX_packet.CH[0],CRSF_RX_packet.CH[1]
//					,CRSF_RX_packet.CH[2],CRSF_RX_packet.CH[3]
//					,CRSF_RX_packet.CH[4],CRSF_RX_packet.CH[5]
//					,CRSF_RX_packet.CH[6],CRSF_RX_packet.CH[7]);
//	printf("%ld\n",hdma_usart1_rx.Instance->CNDTR);//DMA数据搬运数据
}
