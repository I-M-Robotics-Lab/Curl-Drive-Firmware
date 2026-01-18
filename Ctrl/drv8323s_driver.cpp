#include "drv8323s_driver.hpp"
#include <usbs/usb.hpp>

namespace {
inline void wait_us(uint32_t us) { while (us--) delay_us_1(); }
}

bool Driver8323s::readReg(uint8_t addr, uint16_t& value11) {
    const uint16_t cmd = packReadCmd_(addr);
    uint16_t rx = 0;

    delay_us_1();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    delay_us_1();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&cmd, (uint8_t*)&rx, 1, 10);
    delay_us_1();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    delay_us_1();

    if (st != HAL_OK) return false;
    value11 = static_cast<uint16_t>(rx & 0x07FFu);
    return true;
}

bool Driver8323s::writeReg(uint8_t addr, uint16_t value11) {
    uint16_t rx = 0;
    const uint16_t tx = packWrite_(addr, value11);

    delay_us_1();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    delay_us_1();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&tx, (uint8_t*)&rx, 1, 10);
    delay_us_1();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    delay_us_1();

    if (st != HAL_OK) return false;
    for (int attempt = 0; attempt < 2; ++attempt) {
        uint16_t rb = 0;
        delay_us_1();
        if (!readReg(addr, rb)) return false;
        if ((rb & 0x07FFu) == (value11 & 0x07FFu)) return true;
        delay_us_1();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        delay_us_1();
        st = HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&tx, (uint8_t*)&rx, 1, 10);
        delay_us_1();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        delay_us_1();
        if (st != HAL_OK) return false;
    }
    return false;
}

bool Driver8323s::readStatus(Status& out) {
    uint16_t s1=0, s2=0, c2=0, c3=0, c4=0, c5=0, c6=0;
    if (!readReg(0x00, s1)) return false;
    if (!readReg(0x01, s2)) return false;
    if (!readReg(0x02, c2)) return false;
    if (!readReg(0x03, c3)) return false;
    if (!readReg(0x04, c4)) return false;
    if (!readReg(0x05, c5)) return false;
    if (!readReg(0x06, c6)) return false;

    out.raw_status1 = s1; out.raw_status2 = s2;
    out.raw_ctrl2 = c2;   out.raw_ctrl3 = c3; out.raw_ctrl4 = c4;
    out.raw_ctrl5 = c5;   out.raw_ctrl6 = c6;

    out.fault   = (s1 >> 10) & 1;
    out.vds_ocp = (s1 >> 9)  & 1;
    out.gdf     = (s1 >> 8)  & 1;
    out.uvlo    = (s1 >> 7)  & 1;
    out.oTsd    = (s1 >> 6)  & 1;

    out.sa_oc   = (s2 >> 10) & 1;
    out.sb_oc   = (s2 >> 9)  & 1;
    out.sc_oc   = (s2 >> 8)  & 1;
    out.otw     = (s2 >> 7)  & 1;
    out.cp_uv   = (s2 >> 6)  & 1;

    return true;
}


void Driver8323s::clearFaults() {
    uint16_t r2 = 0;
    if (readReg(0x02, r2)) {
        (void)writeReg(0x02, r2 | 0x0001u);
    }
}

bool Driver8323s::calVerifyCleared() {
    uint16_t r6 = 0; if (!readReg(0x06, r6)) return false;
    return ((r6 & (0b111u << 2)) == 0);
}

bool Driver8323s::setCsaGain(uint16_t gain_idx) {
    uint16_t r6 = 0; if (!readReg(0x06, r6)) return false;
    r6 = (r6 & ~(0b11u << 6)) | (static_cast<uint16_t>(gain_idx) << 6);
    return writeReg(0x06, r6);
}

bool Driver8323s::setVdsLevel(uint16_t idx) {
    uint16_t r5 = 0; if (!readReg(0x05, r5)) return false;
    r5 = (r5 & ~0x000Fu) | static_cast<uint16_t>(idx);
    return writeReg(0x05, r5);
}

bool Driver8323s::setOcpMode(OcpMode m) {
    uint16_t r5 = 0; if (!readReg(0x05, r5)) return false;
    r5 = (r5 & ~(0b11u << 6)) | (static_cast<uint16_t>(m) << 6);
    return writeReg(0x05, r5);
}

bool Driver8323s::setGateDriveHS(uint16_t idrivep_idx, uint16_t idriven_idx) {
    uint16_t r3 = static_cast<uint16_t>(0b011u << 8);
    r3 |= static_cast<uint16_t>(idrivep_idx) << 4;
    r3 |= static_cast<uint16_t>(idriven_idx);
    return writeReg(0x03, r3);
}

bool Driver8323s::setGateDriveLS(uint16_t idrivep_idx, uint16_t idriven_idx, uint16_t tdrive_idx) {
    uint16_t r4 = 0;
    r4 |= static_cast<uint16_t>(tdrive_idx)  << 8;
    r4 |= static_cast<uint16_t>(idrivep_idx) << 4;
    r4 |= static_cast<uint16_t>(idriven_idx);
    return writeReg(0x04, r4);
}

bool Driver8323s::setPwmMode3x() {
    uint16_t r2 = 0; if (!readReg(0x02, r2)) return false;
    r2 = (r2 & ~(0b11u << 5)) | (0b01u << 5);
    return writeReg(0x02, r2);
}

bool Driver8323s::setDeadtime(uint16_t idx) {
    uint16_t r5 = 0; if (!readReg(0x05, r5)) return false;
    r5 = (r5 & ~(0b11u << 8)) | (static_cast<uint16_t>(idx) << 8);
    return writeReg(0x05, r5);
}

bool Driver8323s::setCoast() {
    uint16_t r2 = 0; if (!readReg(0x02, r2)) return false;
    r2 = (uint16_t)((r2 | (1u << 2)) & ~(1u << 1));
    if (!writeReg(0x02, r2)) return false;
    mode_ = 0;
    return true;
}

bool Driver8323s::setBrake() {
    uint16_t r2 = 0; if (!readReg(0x02, r2)) return false;
    r2 = (uint16_t)((r2 | (1u << 1)) & ~(1u << 2));
    if (!writeReg(0x02, r2)) return false;
    mode_ = 1;
    return true;
}

bool Driver8323s::setRun() {
    uint16_t r2 = 0; if (!readReg(0x02, r2)) return false;
    r2 &= ~((1u << 2) | (1u << 1));
    if (!writeReg(0x02, r2)) return false;
    mode_ = 2;
    return true;
}

bool Driver8323s::enable() {
	if (!init()) return false;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	return true;
}

bool Driver8323s::disable() {
	if (!setCoast()) return false;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	return true;
}

bool Driver8323s::offhighz() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	return true;
}

bool Driver8323s::onhighz() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	return true;
}

int Driver8323s::init() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(5);

    // Default no High-Z
    offhighz();

    if (!setPwmMode3x()) return 1;
    if (!setCoast()) return 2;
    clearFaults();

    HAL_Delay(1);
    if (!setDeadtime(cfg_.dead_time_idx)) return 3;
    if (!setGateDriveHS(cfg_.idrivep_hs_idx, cfg_.idriven_hs_idx)) return 4;
    if (!setGateDriveLS(cfg_.idrivep_ls_idx, cfg_.idriven_ls_idx, cfg_.tdrive_idx)) return 5;
    if (!setCsaGain(cfg_.csa_gain_idx)) return 6;

    {
        uint16_t r6 = 0;
        if (!readReg(0x06, r6)) return 7;
        r6 = static_cast<uint16_t>((r6 & ~((1u << 10) | (1u << 9))) | (0u << 10) | (1u << 9));
        if (!writeReg(0x06, r6)) return 8;
    }
    HAL_Delay(1);

    clearFaults();
    return 0;
}

