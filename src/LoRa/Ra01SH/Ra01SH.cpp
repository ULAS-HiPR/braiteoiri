
#include <LoRa/Ra01SH.h>
#include <cstring>


Ra01SH::Ra01SH(SPI_Handler& spi_handler, int cs_id, Ra01SH_Pins gpio_pins)
    : spi(&spi_handler), cs(cs_id), pins(gpio_pins), txActive(false)
{

    printf("Ra01SH created\n");
}


void Ra01SH::ResetPinHigh(void) {
    HAL_GPIO_WritePin(pins.reset_port, pins.reset_pin, GPIO_PIN_SET);
}

void Ra01SH::ResetPinLow(void) {
    HAL_GPIO_WritePin(pins.reset_port, pins.reset_pin, GPIO_PIN_RESET);
}

bool Ra01SH::BusyPinRead(void) {
    return HAL_GPIO_ReadPin(pins.busy_port, pins.busy_pin) == GPIO_PIN_SET;
}


void Ra01SH::SpiWrite(const uint8_t* data, size_t len) {
    uint8_t rx_dummy[len];
    spi->transfer_no_cs(data, rx_dummy, len);
}

void Ra01SH::SpiRead(uint8_t* data, size_t len) {
    uint8_t tx_nop[len];
    memset(tx_nop, SX126X_CMD_NOP, len);
    spi->transfer_no_cs(tx_nop, data, len);
}

void Ra01SH::SpiTransfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    spi->transfer_no_cs(tx, rx, len);
}

void Ra01SH::WaitForIdle(unsigned long timeout, const char *text, bool stop) {
    uint32_t start = HAL_GetTick();
    while (BusyPinRead()) {
        if ((HAL_GetTick() - start) >= timeout) {
            printf("WaitForIdle [%s] Timeout %lu\n", text, timeout);
            if (stop) {
                while (1) { HAL_Delay(1); }
            }
            return;
        }
    }
}

void Ra01SH::Reset(void) {
    HAL_Delay(10);
    ResetPinLow();
    HAL_Delay(20);
    ResetPinHigh();
    HAL_Delay(10);
    WaitForIdle(BUSY_WAIT, "Reset", true);
}

void Ra01SH::Wakeup(void) {
    spi->cs_select(cs);
    HAL_Delay(1);
    spi->cs_deselect(cs);
}

void Ra01SH::WriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
    uint8_t status;
    for (int retry = 1; retry < 10; retry++) {
        status = WriteCommand2(cmd, data, numBytes, waitForBusy);
        if (status == 0) break;
    }
    if (status != 0) {
        printf("SPI Transaction error: %d\n", status);
        while (1) { HAL_Delay(1); }
    }
}

uint8_t Ra01SH::WriteCommand2(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
    WaitForIdle(BUSY_WAIT, "start WriteCommand2", true);

    spi->cs_select(cs);

    uint8_t rx_byte;
    spi->transfer_no_cs(&cmd, &rx_byte, 1);

    uint8_t status = 0;
    for (uint8_t n = 0; n < numBytes; n++) {
        uint8_t in;
        spi->transfer_no_cs(&data[n], &in, 1);

        if (((in & 0x0E) == SX126X_STATUS_CMD_TIMEOUT) ||
            ((in & 0x0E) == SX126X_STATUS_CMD_INVALID) ||
            ((in & 0x0E) == SX126X_STATUS_CMD_FAILED)) {
            status = in & 0x0E;
            break;
        } else if (in == 0x00 || in == 0xFF) {
            status = SX126X_STATUS_SPI_FAILED;
            break;
        }
    }

    spi->cs_deselect(cs);

    if (waitForBusy) {
        WaitForIdle(BUSY_WAIT, "end WriteCommand2", false);
    }

    if (status != 0) {
        printf("SPI Transaction error: %d\n", status);
    }
    return status;
}


void Ra01SH::ReadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
    WaitForIdle(BUSY_WAIT, "start ReadCommand", true);

    spi->cs_select(cs);

    uint8_t rx_dummy;
    spi->transfer_no_cs(&cmd, &rx_dummy, 1);

    SpiRead(data, numBytes);

    spi->cs_deselect(cs);

    if (waitForBusy) {
        WaitForIdle(BUSY_WAIT, "end ReadCommand", false);
    }
}

void Ra01SH::WriteRegister(uint16_t reg, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
    WaitForIdle(BUSY_WAIT, "start WriteRegister", true);

    spi->cs_select(cs);

    uint8_t header[3] = {
        SX126X_CMD_WRITE_REGISTER,
        (uint8_t)((reg >> 8) & 0xFF),
        (uint8_t)(reg & 0xFF)
    };
    uint8_t rx_dummy[3];
    spi->transfer_no_cs(header, rx_dummy, 3);

    for (uint8_t n = 0; n < numBytes; n++) {
        uint8_t rx;
        spi->transfer_no_cs(&data[n], &rx, 1);
    }

    spi->cs_deselect(cs);

    if (waitForBusy) {
        WaitForIdle(BUSY_WAIT, "end WriteRegister", false);
    }
}

void Ra01SH::ReadRegister(uint16_t reg, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
    WaitForIdle(BUSY_WAIT, "start ReadRegister", true);

    spi->cs_select(cs);

    uint8_t header[4] = {
        SX126X_CMD_READ_REGISTER,
        (uint8_t)((reg >> 8) & 0xFF),
        (uint8_t)(reg & 0xFF),
        SX126X_CMD_NOP
    };
    uint8_t rx_dummy[4];
    spi->transfer_no_cs(header, rx_dummy, 4);

    SpiRead(data, numBytes);

    spi->cs_deselect(cs);

    if (waitForBusy) {
        WaitForIdle(BUSY_WAIT, "end ReadRegister", false);
    }
}

void Ra01SH::WriteBuffer(uint8_t *txData, uint8_t txDataLen) {
    WaitForIdle(BUSY_WAIT, "start WriteBuffer", true);

    spi->cs_select(cs);

    uint8_t header[2] = { SX126X_CMD_WRITE_BUFFER, 0x00 };
    uint8_t rx_dummy[2];
    spi->transfer_no_cs(header, rx_dummy, 2);

    for (uint16_t i = 0; i < txDataLen; i++) {
        uint8_t rx;
        spi->transfer_no_cs(&txData[i], &rx, 1);
    }

    spi->cs_deselect(cs);
    WaitForIdle(BUSY_WAIT, "end WriteBuffer", false);
}

uint8_t Ra01SH::ReadBuffer(uint8_t *rxData, uint8_t maxLen) {
    uint8_t offset = 0;
    uint8_t payloadLength = 0;
    GetRxBufferStatus(&payloadLength, &offset);
    if (payloadLength > maxLen) {
        printf("ReadBuffer maxLen too small\n");
        return 0;
    }

    WaitForIdle(BUSY_WAIT, "start ReadBuffer", true);

    spi->cs_select(cs);

    uint8_t header[3] = { SX126X_CMD_READ_BUFFER, offset, SX126X_CMD_NOP };
    uint8_t rx_dummy[3];
    spi->transfer_no_cs(header, rx_dummy, 3);

    SpiRead(rxData, payloadLength);

    spi->cs_deselect(cs);
    WaitForIdle(BUSY_WAIT, "end ReadBuffer", false);

    return payloadLength;
}

int16_t Ra01SH::begin(uint32_t frequencyInHz, int8_t txPowerInDbm, float tcxoVoltage, bool useRegulatorLDO) {
    if (txPowerInDbm > 22)  txPowerInDbm = 22;
    if (txPowerInDbm < -3)  txPowerInDbm = -3;

    Reset();

    uint8_t wk[2];
    ReadRegister(SX126X_REG_LORA_SYNC_WORD_MSB, wk, 2);
    uint16_t syncWord = (wk[0] << 8) + wk[1];
    printf("syncWord=0x%04X\n", syncWord);
    if (syncWord != SX126X_SYNC_WORD_PUBLIC && syncWord != SX126X_SYNC_WORD_PRIVATE) {
        printf("SX126x error, maybe no SPI connection\n");
        return ERR_INVALID_MODE;
    }

    printf("SX126x installed\n");
    SetStandby(SX126X_STANDBY_RC);
    SetDio2AsRfSwitchCtrl(true);

    if (tcxoVoltage > 0.0) {
        SetDio3AsTcxoCtrl(tcxoVoltage, RADIO_TCXO_SETUP_TIME);
    }

    Calibrate(SX126X_CALIBRATE_IMAGE_ON
              | SX126X_CALIBRATE_ADC_BULK_P_ON
              | SX126X_CALIBRATE_ADC_BULK_N_ON
              | SX126X_CALIBRATE_ADC_PULSE_ON
              | SX126X_CALIBRATE_PLL_ON
              | SX126X_CALIBRATE_RC13M_ON
              | SX126X_CALIBRATE_RC64K_ON);

    if (useRegulatorLDO) {
        SetRegulatorMode(SX126X_REGULATOR_LDO);
    } else {
        SetRegulatorMode(SX126X_REGULATOR_DC_DC);
    }

    SetBufferBaseAddress(0, 0);
    SetPaConfig(0x04, 0x07, 0x00, 0x01);
    SetOvercurrentProtection(60.0);
    SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U);
    SetRfFrequency(frequencyInHz);
    return ERR_NONE;
}

void Ra01SH::LoRaConfig(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate,
                        uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq) {
    SetStopRxTimerOnPreambleDetect(false);
    SetLoRaSymbNumTimeout(0);
    SetPacketType(SX126X_PACKET_TYPE_LORA);
    SetModulationParams(spreadingFactor, bandwidth, codingRate, 0);

    PacketParams[0] = (preambleLength >> 8) & 0xFF;
    PacketParams[1] = preambleLength;
    if (payloadLen) {
        PacketParams[2] = 0x01;
        PacketParams[3] = payloadLen;
    } else {
        PacketParams[2] = 0x00;
        PacketParams[3] = 0xFF;
    }
    PacketParams[4] = crcOn ? SX126X_LORA_IQ_INVERTED : SX126X_LORA_IQ_STANDARD;
    PacketParams[5] = invertIrq ? 0x01 : 0x00;

    FixInvertedIQ(PacketParams[5]);
    WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6);

    SetDioIrqParams(SX126X_IRQ_ALL, SX126X_IRQ_NONE, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    SetRx(0xFFFFFF);
}

uint8_t Ra01SH::Receive(uint8_t *pData, uint16_t len) {
    uint8_t rxLen = 0;
    uint16_t irqRegs = GetIrqStatus();

    if (irqRegs & SX126X_IRQ_RX_DONE) {
        ClearIrqStatus(SX126X_IRQ_ALL);
        rxLen = ReadBuffer(pData, len);
    }
    return rxLen;
}

bool Ra01SH::Send(uint8_t *pData, uint8_t len, uint8_t mode) {
    uint16_t irqStatus;
    bool rv = false;

    if (!txActive) {
        txActive = true;
        PacketParams[2] = 0x00;
        PacketParams[3] = len;
        WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6);

        ClearIrqStatus(SX126X_IRQ_ALL);
        WriteBuffer(pData, len);
        SetTx(500);

        if (mode & SX126x_TXMODE_SYNC) {
            irqStatus = GetIrqStatus();
            while (!(irqStatus & SX126X_IRQ_TX_DONE) && !(irqStatus & SX126X_IRQ_TIMEOUT)) {
                HAL_Delay(1);
                irqStatus = GetIrqStatus();
            }
            txActive = false;
            SetRx(0xFFFFFF);
            if (irqStatus & SX126X_IRQ_TX_DONE) {
                rv = true;
            }
        } else {
            rv = true;
        }
    }
    return rv;
}

bool Ra01SH::ReceiveMode(void) {
    if (!txActive) return true;

    uint16_t irq = GetIrqStatus();
    if (irq & (SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT)) {
        SetRx(0xFFFFFF);
        txActive = false;
        return true;
    }
    return false;
}

void Ra01SH::GetPacketStatus(int8_t *rssiPacket, int8_t *snrPacket) {
    uint8_t buf[4];
    ReadCommand(SX126X_CMD_GET_PACKET_STATUS, buf, 4);
    *rssiPacket = (buf[3] >> 1) * -1;
    *snrPacket = (buf[2] < 128) ? (buf[2] >> 2) : ((buf[2] - 256) >> 2);
}

void Ra01SH::SetTxPower(int8_t txPowerInDbm) {
    SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U);
}

void Ra01SH::FixInvertedIQ(uint8_t iqConfig) {
    uint8_t iqConfigCurrent = 0;
    ReadRegister(SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1);
    if (iqConfig == SX126X_LORA_IQ_STANDARD) {
        iqConfigCurrent &= 0xFB;
    } else {
        iqConfigCurrent |= 0x04;
    }
    WriteRegister(SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1);
}

void Ra01SH::SetSleep(uint8_t mode) {
    uint8_t data = mode;
    WriteCommand(SX126X_CMD_SET_SLEEP, &data, 1);
}

void Ra01SH::SetStandby(uint8_t mode) {
    uint8_t data = mode;
    WriteCommand(SX126X_CMD_SET_STANDBY, &data, 1);
}

uint8_t Ra01SH::GetStatus(void) {
    uint8_t rv;
    ReadCommand(SX126X_CMD_GET_STATUS, &rv, 1);
    return rv;
}

void Ra01SH::SetDio3AsTcxoCtrl(float voltage, uint32_t delay_val) {
    uint8_t buf[4];
    if      (fabs(voltage - 1.6) <= 0.001) buf[0] = SX126X_DIO3_OUTPUT_1_6;
    else if (fabs(voltage - 1.7) <= 0.001) buf[0] = SX126X_DIO3_OUTPUT_1_7;
    else if (fabs(voltage - 1.8) <= 0.001) buf[0] = SX126X_DIO3_OUTPUT_1_8;
    else if (fabs(voltage - 2.2) <= 0.001) buf[0] = SX126X_DIO3_OUTPUT_2_2;
    else if (fabs(voltage - 2.4) <= 0.001) buf[0] = SX126X_DIO3_OUTPUT_2_4;
    else if (fabs(voltage - 2.7) <= 0.001) buf[0] = SX126X_DIO3_OUTPUT_2_7;
    else if (fabs(voltage - 3.0) <= 0.001) buf[0] = SX126X_DIO3_OUTPUT_3_0;
    else                                    buf[0] = SX126X_DIO3_OUTPUT_3_3;

    uint32_t delayValue = (uint32_t)((float)delay_val / 15.625);
    buf[1] = (uint8_t)((delayValue >> 16) & 0xFF);
    buf[2] = (uint8_t)((delayValue >> 8) & 0xFF);
    buf[3] = (uint8_t)(delayValue & 0xFF);
    WriteCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4);
}

void Ra01SH::Calibrate(uint8_t calibParam) {
    uint8_t data = calibParam;
    WriteCommand(SX126X_CMD_CALIBRATE, &data, 1);
}

void Ra01SH::SetDio2AsRfSwitchCtrl(uint8_t enable) {
    uint8_t data = enable;
    WriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1);
}

void Ra01SH::SetRfFrequency(uint32_t frequency) {
    CalibrateImage(frequency);
    uint32_t freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
    uint8_t buf[4];
    buf[0] = (uint8_t)((freq >> 24) & 0xFF);
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);
    WriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);
}

void Ra01SH::CalibrateImage(uint32_t frequency) {
    uint8_t calFreq[2];
    if      (frequency > 900000000) { calFreq[0] = 0xE1; calFreq[1] = 0xE9; }
    else if (frequency > 850000000) { calFreq[0] = 0xD7; calFreq[1] = 0xDB; }
    else if (frequency > 770000000) { calFreq[0] = 0xC1; calFreq[1] = 0xC5; }
    else if (frequency > 460000000) { calFreq[0] = 0x75; calFreq[1] = 0x81; }
    else if (frequency > 425000000) { calFreq[0] = 0x6B; calFreq[1] = 0x6F; }
    WriteCommand(SX126X_CMD_CALIBRATE_IMAGE, calFreq, 2);
}

void Ra01SH::SetRegulatorMode(uint8_t mode) {
    uint8_t data = mode;
    WriteCommand(SX126X_CMD_SET_REGULATOR_MODE, &data, 1);
}

void Ra01SH::SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
    uint8_t buf[2] = { txBaseAddress, rxBaseAddress };
    WriteCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2);
}

void Ra01SH::SetPowerConfig(int8_t power, uint8_t rampTime) {
    if (power > 22)  power = 22;
    if (power < -3)  power = -3;
    uint8_t buf[2] = { (uint8_t)power, rampTime };
    WriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2);
}

void Ra01SH::SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut) {
    uint8_t buf[4] = { paDutyCycle, hpMax, deviceSel, paLut };
    WriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);
}

void Ra01SH::SetOvercurrentProtection(float currentLimit) {
    if (currentLimit >= 0.0 && currentLimit <= 140.0) {
        uint8_t buf[1] = { (uint8_t)(currentLimit / 2.5) };
        WriteRegister(SX126X_REG_OCP_CONFIGURATION, buf, 1);
    }
}

void Ra01SH::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
    uint8_t buf[8];
    buf[0] = (uint8_t)((irqMask >> 8) & 0xFF);
    buf[1] = (uint8_t)(irqMask & 0xFF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0xFF);
    buf[3] = (uint8_t)(dio1Mask & 0xFF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0xFF);
    buf[5] = (uint8_t)(dio2Mask & 0xFF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0xFF);
    buf[7] = (uint8_t)(dio3Mask & 0xFF);
    WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}

void Ra01SH::SetStopRxTimerOnPreambleDetect(bool enable) {
    uint8_t data = (uint8_t)enable;
    WriteCommand(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1);
}

void Ra01SH::SetLoRaSymbNumTimeout(uint8_t SymbNum) {
    uint8_t data = SymbNum;
    WriteCommand(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1);
}

void Ra01SH::SetPacketType(uint8_t packetType) {
    uint8_t data = packetType;
    WriteCommand(SX126X_CMD_SET_PACKET_TYPE, &data, 1);
}

void Ra01SH::SetModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t lowDataRateOptimize) {
    uint8_t data[4] = { spreadingFactor, bandwidth, codingRate, lowDataRateOptimize };
    WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, 4);
}

uint16_t Ra01SH::GetIrqStatus(void) {
    uint8_t data[3];
    ReadCommand(SX126X_CMD_GET_IRQ_STATUS, data, 3);
    return (data[1] << 8) | data[2];
}

void Ra01SH::ClearIrqStatus(uint16_t irq) {
    uint8_t buf[2] = { (uint8_t)((irq >> 8) & 0xFF), (uint8_t)(irq & 0xFF) };
    WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);
}

void Ra01SH::SetRx(uint32_t timeout) {
    SetStandby(SX126X_STANDBY_RC);
    SetRxEnable();
    uint8_t buf[3];
    buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[2] = (uint8_t)(timeout & 0xFF);
    WriteCommand(SX126X_CMD_SET_RX, buf, 3);

    for (int retry = 0; retry < 10; retry++) {
        if ((GetStatus() & 0x70) == 0x50) break;
        HAL_Delay(1);
    }
    if ((GetStatus() & 0x70) != 0x50) {
        printf("SetRx Illegal Status\n");
        while (1) { HAL_Delay(1); }
    }
}

void Ra01SH::SetTx(uint32_t timeoutInMs) {
    SetStandby(SX126X_STANDBY_RC);
    SetTxEnable();
    uint32_t tout = timeoutInMs;
    if (timeoutInMs != 0) {
        uint32_t timeoutInUs = timeoutInMs * 1000;
        tout = (uint32_t)(timeoutInUs / 15.625);
    }
    uint8_t buf[3];
    buf[0] = (uint8_t)((tout >> 16) & 0xFF);
    buf[1] = (uint8_t)((tout >> 8) & 0xFF);
    buf[2] = (uint8_t)(tout & 0xFF);
    WriteCommand(SX126X_CMD_SET_TX, buf, 3);

    for (int retry = 0; retry < 10; retry++) {
        if ((GetStatus() & 0x70) == 0x60) break;
        HAL_Delay(1);
    }
    if ((GetStatus() & 0x70) != 0x60) {
        printf("SetTx Illegal Status\n");
        while (1) { HAL_Delay(1); }
    }
}

void Ra01SH::SetRxEnable(void) {
}

void Ra01SH::SetTxEnable(void) {
}

uint8_t Ra01SH::GetRssiInst() {
    uint8_t buf[2];
    ReadCommand(SX126X_CMD_GET_RSSI_INST, buf, 2);
    return buf[1];
}

void Ra01SH::GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer) {
    uint8_t buf[3];
    ReadCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 3);
    *payloadLength = buf[1];
    *rxStartBufferPointer = buf[2];
}
