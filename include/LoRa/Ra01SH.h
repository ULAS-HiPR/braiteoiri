#pragma once

#include <SPI/SPI_Handler.h>
#include <cstdio>
#include <cstdint>
#include <cmath>

#define ERR_NONE                        0
#define ERR_PACKET_TOO_LONG             1
#define ERR_UNKNOWN                     2
#define ERR_TX_TIMEOUT                  3
#define ERR_RX_TIMEOUT                  4
#define ERR_CRC_MISMATCH                5
#define ERR_WRONG_MODEM                 6
#define ERR_INVALID_BANDWIDTH           7
#define ERR_INVALID_SPREADING_FACTOR    8
#define ERR_INVALID_CODING_RATE         9
#define ERR_INVALID_FREQUENCY_DEVIATION 10
#define ERR_INVALID_BIT_RATE            11
#define ERR_INVALID_RX_BANDWIDTH        12
#define ERR_INVALID_DATA_SHAPING        13
#define ERR_INVALID_SYNC_WORD           14
#define ERR_INVALID_OUTPUT_POWER        15
#define ERR_INVALID_MODE                16
#define ERR_INVALID_TRANCEIVER          17

#define XTAL_FREQ                       ( double )32000000
#define FREQ_DIV                        ( double )pow( 2.0, 25.0 )
#define FREQ_STEP                       ( double )( XTAL_FREQ / FREQ_DIV )
#define BUSY_WAIT                       5000

#define SX126X_CMD_NOP                                0x00
#define SX126X_CMD_SET_SLEEP                          0x84
#define SX126X_CMD_SET_STANDBY                        0x80
#define SX126X_CMD_SET_FS                             0xC1
#define SX126X_CMD_SET_TX                             0x83
#define SX126X_CMD_SET_RX                             0x82
#define SX126X_CMD_STOP_TIMER_ON_PREAMBLE             0x9F
#define SX126X_CMD_SET_RX_DUTY_CYCLE                  0x94
#define SX126X_CMD_SET_CAD                            0xC5
#define SX126X_CMD_SET_TX_CONTINUOUS_WAVE             0xD1
#define SX126X_CMD_SET_TX_INFINITE_PREAMBLE           0xD2
#define SX126X_CMD_SET_REGULATOR_MODE                 0x96
#define SX126X_CMD_CALIBRATE                          0x89
#define SX126X_CMD_CALIBRATE_IMAGE                    0x98
#define SX126X_CMD_SET_PA_CONFIG                      0x95
#define SX126X_CMD_SET_RX_TX_FALLBACK_MODE            0x93
#define SX126X_CMD_WRITE_REGISTER                     0x0D
#define SX126X_CMD_READ_REGISTER                      0x1D
#define SX126X_CMD_WRITE_BUFFER                       0x0E
#define SX126X_CMD_READ_BUFFER                        0x1E
#define SX126X_CMD_SET_DIO_IRQ_PARAMS                 0x08
#define SX126X_CMD_GET_IRQ_STATUS                     0x12
#define SX126X_CMD_CLEAR_IRQ_STATUS                   0x02
#define SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL         0x9D
#define SX126X_CMD_SET_DIO3_AS_TCXO_CTRL              0x97
#define SX126X_CMD_SET_RF_FREQUENCY                   0x86
#define SX126X_CMD_SET_PACKET_TYPE                    0x8A
#define SX126X_CMD_GET_PACKET_TYPE                    0x11
#define SX126X_CMD_SET_TX_PARAMS                      0x8E
#define SX126X_CMD_SET_MODULATION_PARAMS              0x8B
#define SX126X_CMD_SET_PACKET_PARAMS                  0x8C
#define SX126X_CMD_SET_CAD_PARAMS                     0x88
#define SX126X_CMD_SET_BUFFER_BASE_ADDRESS            0x8F
#define SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT          0xA0
#define SX126X_CMD_GET_STATUS                         0xC0
#define SX126X_CMD_GET_RSSI_INST                      0x15
#define SX126X_CMD_GET_RX_BUFFER_STATUS               0x13
#define SX126X_CMD_GET_PACKET_STATUS                  0x14
#define SX126X_CMD_GET_DEVICE_ERRORS                  0x17
#define SX126X_CMD_CLEAR_DEVICE_ERRORS                0x07

#define SX126X_REG_IQ_POLARITY_SETUP                  0x0736
#define SX126X_REG_LORA_SYNC_WORD_MSB                 0x0740
#define SX126X_REG_LORA_SYNC_WORD_LSB                 0x0741
#define SX126X_REG_RANDOM_NUMBER_0                    0x0819
#define SX126X_REG_TX_MODULETION                      0x0889
#define SX126X_REG_RX_GAIN                            0x08AC
#define SX126X_REG_TX_CLAMP_CONFIG                    0x08D8
#define SX126X_REG_OCP_CONFIGURATION                  0x08E7
#define SX126X_REG_RTC_CONTROL                        0x0902
#define SX126X_REG_XTA_TRIM                           0x0911
#define SX126X_REG_XTB_TRIM                           0x0912
#define SX126X_REG_DIO3_OUTPUT_VOLTAGE_CONTROL        0x0920
#define SX126X_REG_EVENT_MASK                         0x0944

#define SX126X_STANDBY_RC                             0x00
#define SX126X_STANDBY_XOSC                           0x01

#define SX126X_PACKET_TYPE_GFSK                       0x00
#define SX126X_PACKET_TYPE_LORA                       0x01

#define SX126X_PA_RAMP_10U                            0x00
#define SX126X_PA_RAMP_20U                            0x01
#define SX126X_PA_RAMP_40U                            0x02
#define SX126X_PA_RAMP_80U                            0x03
#define SX126X_PA_RAMP_200U                           0x04
#define SX126X_PA_RAMP_800U                           0x05
#define SX126X_PA_RAMP_1700U                          0x06
#define SX126X_PA_RAMP_3400U                          0x07

#define SX126X_CALIBRATE_IMAGE_ON                     0x40
#define SX126X_CALIBRATE_ADC_BULK_P_ON                0x20
#define SX126X_CALIBRATE_ADC_BULK_N_ON                0x10
#define SX126X_CALIBRATE_ADC_PULSE_ON                 0x08
#define SX126X_CALIBRATE_PLL_ON                       0x04
#define SX126X_CALIBRATE_RC13M_ON                     0x02
#define SX126X_CALIBRATE_RC64K_ON                     0x01

#define SX126X_REGULATOR_LDO                          0x00
#define SX126X_REGULATOR_DC_DC                        0x01

#define SX126X_DIO3_OUTPUT_1_6                        0x00
#define SX126X_DIO3_OUTPUT_1_7                        0x01
#define SX126X_DIO3_OUTPUT_1_8                        0x02
#define SX126X_DIO3_OUTPUT_2_2                        0x03
#define SX126X_DIO3_OUTPUT_2_4                        0x04
#define SX126X_DIO3_OUTPUT_2_7                        0x05
#define SX126X_DIO3_OUTPUT_3_0                        0x06
#define SX126X_DIO3_OUTPUT_3_3                        0x07
#define RADIO_TCXO_SETUP_TIME                         5000

#define SX126X_IRQ_TIMEOUT                            0b1000000000
#define SX126X_IRQ_CAD_DETECTED                       0b0100000000
#define SX126X_IRQ_CAD_DONE                           0b0010000000
#define SX126X_IRQ_CRC_ERR                            0b0001000000
#define SX126X_IRQ_HEADER_ERR                         0b0000100000
#define SX126X_IRQ_HEADER_VALID                       0b0000010000
#define SX126X_IRQ_SYNC_WORD_VALID                    0b0000001000
#define SX126X_IRQ_PREAMBLE_DETECTED                  0b0000000100
#define SX126X_IRQ_RX_DONE                            0b0000000010
#define SX126X_IRQ_TX_DONE                            0b0000000001
#define SX126X_IRQ_ALL                                0b1111111111
#define SX126X_IRQ_NONE                               0b0000000000

#define SX126X_LORA_BW_7_8                            0x00
#define SX126X_LORA_BW_10_4                           0x08
#define SX126X_LORA_BW_15_6                           0x01
#define SX126X_LORA_BW_20_8                           0x09
#define SX126X_LORA_BW_31_25                          0x02
#define SX126X_LORA_BW_41_7                           0x0A
#define SX126X_LORA_BW_62_5                           0x03
#define SX126X_LORA_BW_125_0                          0x04
#define SX126X_LORA_BW_250_0                          0x05
#define SX126X_LORA_BW_500_0                          0x06

#define SX126X_LORA_CR_4_5                            0x01
#define SX126X_LORA_CR_4_6                            0x02
#define SX126X_LORA_CR_4_7                            0x03
#define SX126X_LORA_CR_4_8                            0x04

#define SX126X_LORA_IQ_STANDARD                       0x00
#define SX126X_LORA_IQ_INVERTED                       0x01

#define SX126X_STATUS_CMD_TIMEOUT                     0b00000110
#define SX126X_STATUS_CMD_INVALID                     0b00001000
#define SX126X_STATUS_CMD_FAILED                      0b00001010
#define SX126X_STATUS_SPI_FAILED                      0xFF

#define SX126x_TXMODE_ASYNC                           0x01
#define SX126x_TXMODE_SYNC                            0x02

#define SX126X_SYNC_WORD_PUBLIC                       0x3444
#define SX126X_SYNC_WORD_PRIVATE                      0x1424

struct Ra01SH_Pins {
    GPIO_TypeDef* reset_port;
    uint16_t      reset_pin;
    GPIO_TypeDef* busy_port;
    uint16_t      busy_pin;
};

class Ra01SH {
public:
    Ra01SH(SPI_Handler& spi_handler, int cs_id, Ra01SH_Pins pins);

    int16_t  begin(uint32_t frequencyInHz, int8_t txPowerInDbm, float tcxoVoltage = 0.0, bool useRegulatorLDO = false);
    void     LoRaConfig(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq);
    uint8_t  Receive(uint8_t *pData, uint16_t len);
    bool     Send(uint8_t *pData, uint8_t len, uint8_t mode);
    bool     ReceiveMode(void);
    void     GetPacketStatus(int8_t *rssiPacket, int8_t *snrPacket);
    void     SetTxPower(int8_t txPowerInDbm);

private:
    SPI_Handler* spi;
    int          cs;
    Ra01SH_Pins  pins;
    uint8_t      PacketParams[6] = {0};
    bool         txActive;

    void     SpiWrite(const uint8_t* data, size_t len);
    void     SpiRead(uint8_t* data, size_t len);
    void     SpiTransfer(const uint8_t* tx, uint8_t* rx, size_t len);

    void     ResetPinHigh(void);
    void     ResetPinLow(void);
    bool     BusyPinRead(void);

    void     WriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
    uint8_t  WriteCommand2(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
    void     ReadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
    void     WriteRegister(uint16_t reg, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
    void     ReadRegister(uint16_t reg, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
    void     WriteBuffer(uint8_t *txData, uint8_t txDataLen);
    uint8_t  ReadBuffer(uint8_t *rxData, uint8_t maxLen);
    void     WaitForIdle(unsigned long timeout, const char *text, bool stop);

    void     Reset(void);
    void     Wakeup(void);
    void     SetSleep(uint8_t mode);
    void     SetStandby(uint8_t mode);
    void     SetRfFrequency(uint32_t frequency);
    void     CalibrateImage(uint32_t frequency);
    void     Calibrate(uint8_t calibParam);
    void     SetRegulatorMode(uint8_t mode);
    void     SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    void     SetPowerConfig(int8_t power, uint8_t rampTime);
    void     SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);
    void     SetOvercurrentProtection(float currentLimit);
    void     SetDio2AsRfSwitchCtrl(uint8_t enable);
    void     SetDio3AsTcxoCtrl(float voltage, uint32_t delay);
    void     SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    void     SetStopRxTimerOnPreambleDetect(bool enable);
    void     SetLoRaSymbNumTimeout(uint8_t SymbNum);
    void     SetPacketType(uint8_t packetType);
    void     SetModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t lowDataRateOptimize);
    void     FixInvertedIQ(uint8_t iqConfig);
    uint8_t  GetStatus(void);
    uint16_t GetIrqStatus(void);
    void     ClearIrqStatus(uint16_t irq);
    void     SetRx(uint32_t timeout);
    void     SetTx(uint32_t timeoutInMs);
    void     SetRxEnable(void);
    void     SetTxEnable(void);
    uint8_t  GetRssiInst(void);
    void     GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer);
};
