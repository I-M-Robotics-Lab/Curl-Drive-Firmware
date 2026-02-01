#include "as5047_encoder.hpp"

void delay_us_1() {
    const uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) < 8u) { }
}

uint8_t calcEvenParity(uint16_t v) noexcept {
    v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
    return static_cast<uint8_t>(v & 1u);
}

uint16_t buildCommandAS5047() noexcept {
    uint16_t cmd = 0x3FFFu | (1u << 14);
    cmd |= static_cast<uint16_t>(calcEvenParity(cmd) << 15);
    return cmd;
}

uint16_t align14(uint16_t raw) noexcept {

    const int shift = 1 + 13 - 14; // 1 + data_start_bit - bit_resolution
    const uint16_t mask = static_cast<uint16_t>(0xFFFFu >> (16 - 14)); // 16 - bit_resolution
    return (shift >= 0) ? ((raw >> shift) & mask) : ((raw << (-shift)) & mask);
}

static inline float counts_to_rad(uint16_t c) {
    const float k = 6.283185307179586f / 16384.0f;
    return k * static_cast<float>(c & 0x3FFFu);
}

static inline float counts_to_deg(uint16_t c) {
    const float k = 360.0f / 16384.0f;
    return k * static_cast<float>(c & 0x3FFFu);
}

bool EncoderAS5047P::init() {

    updateAngle();
    delay_us_1();
    updateAngle();
    delay_us_1();
    updateAngle();
    delay_us_1();
    tick_ = reinterpret_cast<const volatile uint32_t*>(&TIM2->CNT);
    return true;
}
uint16_t EncoderAS5047P::updateAngle() {
    uint16_t rx = 0;
    uint16_t tx = 0;

    SPI2->CR1 |= SPI_CR1_SPE;

    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {

        tx = buildCommandAS5047();
        GPIOB->BSRR = (uint32_t)GPIO_PIN_12 << 16U;

        while (!(SPI2->SR & SPI_SR_TXE));
        SPI2->DR = tx;
        while (!(SPI2->SR & SPI_SR_RXNE));
        rx = SPI2->DR;
        while (SPI2->SR & SPI_SR_BSY);

        GPIOB->BSRR = GPIO_PIN_12;

        delay_us_1();

        tx = 0x000u;
        GPIOB->BSRR = (uint32_t)GPIO_PIN_12 << 16U;

        while (!(SPI2->SR & SPI_SR_TXE));
        SPI2->DR = tx;
        while (!(SPI2->SR & SPI_SR_RXNE));
        rx = SPI2->DR;
        while (SPI2->SR & SPI_SR_BSY);

        GPIOB->BSRR = GPIO_PIN_12;

        ok = (calcEvenParity(rx) == 0);
    }

    if (!ok) {
        return data_.rawAngle;
    }

    const uint16_t raw14 = align14(rx);
    data_.rawAngle = raw14;
    data_.degrees = counts_to_deg(data_.rawAngle);
    data_.radians = counts_to_rad(data_.rawAngle);
    if (tick_) {
        data_.tstamp = *tick_;
    }

    return data_.rawAngle;
}

uint16_t EncoderAS5047P::readDiaagc() {
    constexpr uint16_t DIAAGC_ADDR = 0x3FFCu;
    uint16_t rx = 0, tx = 0;
    bool ok = false;

    SPI2->CR1 |= SPI_CR1_SPE;

    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
        uint16_t cmd = static_cast<uint16_t>((1u << 14) | DIAAGC_ADDR);
        cmd |= static_cast<uint16_t>(calcEvenParity(cmd) << 15);

        GPIOB->BSRR = (uint32_t)GPIO_PIN_12 << 16U;
        while (!(SPI2->SR & SPI_SR_TXE));
        SPI2->DR = cmd;
        while (!(SPI2->SR & SPI_SR_RXNE));
        rx = SPI2->DR;
        while (SPI2->SR & SPI_SR_BSY);
        GPIOB->BSRR = GPIO_PIN_12;

        delay_us_1();

        tx = 0x0000u;
        GPIOB->BSRR = (uint32_t)GPIO_PIN_12 << 16U;
        while (!(SPI2->SR & SPI_SR_TXE));
        SPI2->DR = tx;
        while (!(SPI2->SR & SPI_SR_RXNE));
        rx = SPI2->DR;
        while (SPI2->SR & SPI_SR_BSY);
        GPIOB->BSRR = GPIO_PIN_12;

        ok = (calcEvenParity(rx) == 0);
    }

    if (!ok) return 0xFFFFu;
    return align14(rx);
}

/*
uint16_t EncoderAS5047P::updateAngle() {
    uint16_t rx = 0;
    uint16_t tx = 0;

    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {

        tx = buildCommandAS5047();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        (void)HAL_SPI_TransmitReceive(&hspi2, reinterpret_cast<uint8_t*>(&tx), reinterpret_cast<uint8_t*>(&rx), 1, 10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

        delay_us_1();

        tx = 0x000u;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        (void)HAL_SPI_TransmitReceive(&hspi2, reinterpret_cast<uint8_t*>(&tx), reinterpret_cast<uint8_t*>(&rx), 1, 10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

        ok = (calcEvenParity(rx) == 0);
    }

    if (!ok) {
        return data_.rawAngle;
    }

    const uint16_t raw14 = align14(rx);
    data_.rawAngle = raw14;
    data_.degrees = counts_to_deg(data_.rawAngle);
	data_.radians = counts_to_rad(data_.rawAngle);
    if (tick_) {
        data_.tstamp = *tick_;
    }

    return data_.rawAngle;
}

uint16_t EncoderAS5047P::readDiaagc() {
    constexpr uint16_t DIAAGC_ADDR = 0x3FFCu;
    uint16_t rx = 0, tx = 0;
    bool ok = false;

    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
        uint16_t cmd = static_cast<uint16_t>((1u << 14) | DIAAGC_ADDR);
        cmd |= static_cast<uint16_t>(calcEvenParity(cmd) << 15);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        (void)HAL_SPI_TransmitReceive(&hspi2, reinterpret_cast<uint8_t*>(&cmd), reinterpret_cast<uint8_t*>(&rx), 1, 10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        delay_us_1();

        tx = 0x0000u;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        (void)HAL_SPI_TransmitReceive(&hspi2, reinterpret_cast<uint8_t*>(&tx), reinterpret_cast<uint8_t*>(&rx), 1, 10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

        ok = (calcEvenParity(rx) == 0);
    }

    if (!ok) return 0xFFFFu;
    return align14(rx);
}
*/


