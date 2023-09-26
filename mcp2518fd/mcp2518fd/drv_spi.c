#include "drv_spi.h"

#define BIT(_x_) ((uint32_t)(1U) << (_x_))

#define MTT_MAX_DATA_LEN 64
#define CAN_FMT 0x80000000U	/*1= EXT/ 0 = STD */
#define CAN_RTR 0x40000000U	/* RTR */
#define CAN_ERR 0x20000000U	/* ERR/Data  message frame */

struct canfd_frame {
	uint32_t can_id;	/* FMT/RTR/ERR/ID */
	uint8_t d_len;		/* data length */
	uint8_t flags;		/* FD flags */
	uint8_t resv0;
	uint8_t resv1;
	uint8_t data[MTT_MAX_DATA_LEN];
	/* Any new structure entries should be placed below the comment */
	uint32_t tstamp;
};

void DRV_SPI_Initialize(void)
{
    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);

    CAN_OSC_CTRL oscCtrl;
    DRV_CANFDSPI_OscillatorControlObjectReset(&oscCtrl);
    oscCtrl.ClkOutDivide = OSC_CLKO_DIV10;
    DRV_CANFDSPI_OscillatorControlSet(DRV_CANFDSPI_INDEX_0, oscCtrl);

    DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
    CAN_CONFIG config;
    DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 1;
    config.TXQEnable = 1;
    DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);
    DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_2M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
    CAN_TEF_CONFIG tefConfig;
    tefConfig.FifoSize = 11;
    tefConfig.TimeStampEnable = 1;
    DRV_CANFDSPI_TefConfigure(DRV_CANFDSPI_INDEX_0, &tefConfig);
    CAN_TX_QUEUE_CONFIG txqConfig;
    DRV_CANFDSPI_TransmitQueueConfigureObjectReset(&txqConfig);
    txqConfig.TxPriority = 1;
    txqConfig.FifoSize = 7;
    txqConfig.PayLoadSize = CAN_PLSIZE_32;
    DRV_CANFDSPI_TransmitQueueConfigure(DRV_CANFDSPI_INDEX_0, &txqConfig);

    CAN_TX_FIFO_CONFIG txfConfig;
    DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txfConfig);
    txfConfig.FifoSize = 4;
    txfConfig.PayLoadSize = CAN_PLSIZE_64;
    txfConfig.TxPriority = 0;
    DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, &txfConfig);

    CAN_RX_FIFO_CONFIG rxfConfig;
    rxfConfig.FifoSize = 15;
    rxfConfig.PayLoadSize = CAN_PLSIZE_64;
    rxfConfig.RxTimeStampEnable = 1;
    DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, &rxfConfig);

    DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);
    DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xff);
    DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);

}

/**
  * @brief  SPI片选信号控制
  * @param  spiSlaveDeviceIndex: 多路SPI选择，要根据芯片型号来判断可出几路
  * @param  assert: 是否选择模块。true——拉低；false——拉高
  * @retval -1——片选失败；0——片选完成
  */
int8_t DRV_SPI_ChipSelectAssert(uint8_t spiSlaveDeviceIndex, bool assert)
{
    int8_t error = 0;

    // Select Chip Select
    switch (spiSlaveDeviceIndex) {
        case DRV_CANFDSPI_INDEX_0:
            if (assert)
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            else
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
            break;
        default:
            error = -1;
            break;
    }
    return error;
}


/**
  * @brief  MCP2517FD模块数据发送接收函数
  * @param  spiSlaveDeviceIndex: 多路SPI选择
  * @param  SpiTxData: 发送的数据
  * @param  SpiRxData: 接收的数据
  * @param  spiTransferSize: 发送数据的长度
  * @retval 片选信号状态  -1——片选失败；0——片选成功
  */
int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
    int8_t error = 0;
    // Assert CS
    error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, true);
    if (error != 0)
        return error;

    switch (spiSlaveDeviceIndex){
        case DRV_CANFDSPI_INDEX_0:
            HAL_SPI_TransmitReceive(&hspi1,SpiTxData,SpiRxData,spiTransferSize,1000);
            break;
        default:
            break;
    }
    // De‐assert CS
    error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, false);

    return error;
} 


void mcp2518fd_transpond(void)
{
		CAN_RX_FIFO_EVENT rxFlags;
		CAN_RX_MSGOBJ rxObj;
		struct canfd_frame frame;
		uint32_t rxif;
		CAN_MODULE_EVENT eventflags;
		CAN_ECC_EVENT eccflags;
		CAN_RXCODE rxCode;

		memset(&frame, 0, sizeof(frame));

		DRV_CANFDSPI_ModuleEventGet(DRV_CANFDSPI_INDEX_0, &eventflags);
		DRV_CANFDSPI_ReceiveEventGet(DRV_CANFDSPI_INDEX_0, &rxif);
		DRV_CANFDSPI_ModuleEventRxCodeGet(DRV_CANFDSPI_INDEX_0, &rxCode);
		// printf("rxif:%d, rxCode:%d\r\n",(int)rxif, (int)rxCode);
		if ((!rxif) && (!rxCode)) {
			printf("111eventflags:%d\r\n", eventflags);
			printf("rxif:%d\r\n",(int)rxif);
			DRV_CANFDSPI_ModuleEventRxCodeGet(DRV_CANFDSPI_INDEX_0, &rxCode);
			printf("0000rxCode:%d\r\n", rxCode);
			CAN_TXCODE txCode;
			DRV_CANFDSPI_ModuleEventTxCodeGet(DRV_CANFDSPI_INDEX_0, &txCode);
			printf("0000txCode:%d\r\n", txCode);
			CAN_FILTER filterHit;
			DRV_CANFDSPI_ModuleEventFilterHitGet(DRV_CANFDSPI_INDEX_0, &filterHit);
			printf("0000filterHits:%d\r\n", filterHit);
			CAN_ICODE icode;
			DRV_CANFDSPI_ModuleEventIcodeGet(DRV_CANFDSPI_INDEX_0, &icode);
			printf("0000icode:%d\r\n", icode);
			CAN_CRC_EVENT crcflags;
			DRV_CANFDSPI_CrcEventGet(DRV_CANFDSPI_INDEX_0, &crcflags);
			printf("0000crcflags:%d\r\n", crcflags);
			CAN_TEF_FIFO_EVENT tefflags;
			DRV_CANFDSPI_TefEventGet(DRV_CANFDSPI_INDEX_0, &tefflags);
			printf("0000tefflags:%d\r\n", tefflags);
			DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0,CAN_FIFO_CH2, &rxFlags);
			printf("0000rxFlags:%d\r\n", rxFlags);
			DRV_CANFDSPI_ModuleEventGet(DRV_CANFDSPI_INDEX_0, &eventflags);
			printf("2222eventflags:%d\r\n", eventflags);
			uint32_t rxovif;
			DRV_CANFDSPI_ReceiveEventOverflowGet(DRV_CANFDSPI_INDEX_0, &rxovif);
			printf("0000rxovif:%d\r\n", rxovif);
			uint8_t tec, rec;
			CAN_ERROR_STATE flags;
			DRV_CANFDSPI_ErrorCountStateGet(DRV_CANFDSPI_INDEX_0, &tec, &rec, &flags);
			printf("tec:%d, rec:%d, flags:%d\r\n", tec, rec, flags);
			CAN_BUS_DIAGNOSTIC bd;
			DRV_CANFDSPI_BusDiagnosticsGet(DRV_CANFDSPI_INDEX_0, &bd);
			printf("bd:%d\r\n", bd);
			DRV_CANFDSPI_ReceiveChannelEventOverflowClear(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2);
			return;
		}

		int count, start;
		for (count = 0, start = CAN_FIFO_CH2;
			count < 1; count++, start++) {
			if (rxif & BIT(start) || (rxCode == start)) {
			// 	printf("BIT(start):%d\r\n",(int)BIT(start));
				// DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, start, &rxFlags);
				// printf("111rxFlags:%d\r\n", rxFlags);
				// if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
					DRV_CANFDSPI_ReceiveMessageGetBulk(DRV_CANFDSPI_INDEX_0, start, &rxObj, &frame.data[0], MAX_DATA_BYTES);
					if (rxObj.bF.ctrl.IDE) {
						frame.can_id = (rxObj.bF.id.SID << 18) + rxObj.bF.id.EID;
						frame.can_id |= CAN_FMT;
					} else {
						frame.can_id = rxObj.bF.id.SID;
					}

					// if (rxObj.bF.ctrl.ESI) {
					// 	frame.flags |= CAN_ESI_FLAG;
					// 	frame.can_id |= CAN_ERR;
					// } else {
					// 	frame.flags &= ~CAN_ESI_FLAG;
					// 	frame.can_id &= ~CAN_ERR;
					// }

					// if (rxObj.bF.ctrl.RTR) {
					// 	frame.can_id |= CAN_RTR;
					// } else {
					// 	frame.can_id &= ~CAN_RTR;
					// }
					// printf("\r\n Receive message's IDs = %x, DLC = %d, FLAG = %x\r\n", (unsigned int)frame.can_id, rxObj.bF.ctrl.DLC, frame.flags);
					frame.d_len = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)rxObj.bF.ctrl.DLC);
				// }
				// DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, start, &rxFlags);
				// printf("222rxFlags:%d\r\n", rxFlags);
			}
		}
		// DRV_CANFDSPI_ModuleEventGet(DRV_CANFDSPI_INDEX_0, &eventflags);
		// printf("222eventflags:%d\r\n", eventflags);
		// DRV_CANFDSPI_EccEventGet(DRV_CANFDSPI_INDEX_0, &eccflags);
		// printf("eccflags:%d\r\n", eccflags);
		DRV_CANFDSPI_EccEventClear(DRV_CANFDSPI_INDEX_0, CAN_ECC_ALL_EVENTS);
		DRV_CANFDSPI_ModuleEventClear(DRV_CANFDSPI_INDEX_0, CAN_ALL_EVENTS);
}
