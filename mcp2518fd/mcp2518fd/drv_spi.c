#include "drv_spi.h"

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
  * @brief  SPIƬѡ�źſ���
  * @param  spiSlaveDeviceIndex: ��·SPIѡ��Ҫ����оƬ�ͺ����жϿɳ���·
  * @param  assert: �Ƿ�ѡ��ģ�顣true�������ͣ�false��������
  * @retval -1����Ƭѡʧ�ܣ�0����Ƭѡ���
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
  * @brief  MCP2517FDģ�����ݷ��ͽ��պ���
  * @param  spiSlaveDeviceIndex: ��·SPIѡ��
  * @param  SpiTxData: ���͵�����
  * @param  SpiRxData: ���յ�����
  * @param  spiTransferSize: �������ݵĳ���
  * @retval Ƭѡ�ź�״̬  -1����Ƭѡʧ�ܣ�0����Ƭѡ�ɹ�
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
    // De�\assert CS
    error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, false);

    return error;
} 


