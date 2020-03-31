/*
 * Copyright (c) 2020, Nuvoton Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "qspi_api.h"

#if DEVICE_QSPI

#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "gpio_api.h"
#include "mbed_toolchain.h"
#include "nu_modutil.h"
#include "nu_miscutil.h"
#include "nu_bitutil.h"

#define NU_QSPI_FRAME_MIN   8
#define NU_QSPI_FRAME_MAX   32

/* Synchronous version of QSPI_ENABLE()/QSPI_DISABLE() macros
 *
 * The QSPI peripheral clock is asynchronous with the system clock. In order to make sure the SPI
 * control logic is enabled/disabled, this bit indicates the real status of QSPI controller.
 *
 * NOTE: All configurations shall be ready before calling QSPI_ENABLE_SYNC().
 * NOTE: Before changing the configurations of QSPIx_CTL, QSPIx_CLKDIV, QSPIx_SSCTL and QSPIx_FIFOCTL registers,
 *       user shall clear the SPIEN (QSPIx_CTL[0]) and confirm the SPIENSTS (QSPIx_STATUS[15]) is 0
 *       (by QSPI_DISABLE_SYNC here).
 */
__STATIC_INLINE void QSPI_ENABLE_SYNC(QSPI_T *qspi_base)
{
    if (! (qspi_base->CTL & QSPI_CTL_QSPIEN_Msk)) {
        QSPI_ENABLE(qspi_base);
    }
    while (! (qspi_base->STATUS & QSPI_STATUS_QSPIENSTS_Msk));
}
__STATIC_INLINE void QSPI_DISABLE_SYNC(QSPI_T *qspi_base)
{
    if (qspi_base->CTL & QSPI_CTL_QSPIEN_Msk) {
        // NOTE: QSPI H/W may get out of state without the busy check.
        while (QSPI_IS_BUSY(qspi_base));
    
        QSPI_DISABLE(qspi_base);
    }
    while (qspi_base->STATUS & QSPI_STATUS_QSPIENSTS_Msk);
}

__STATIC_INLINE void QSPI_GUARD_WRITE_TX(QSPI_T *qspi_base, uint32_t value)
{
    while (QSPI_GET_TX_FIFO_FULL_FLAG(qspi_base));
    QSPI_WRITE_TX(qspi_base, value);
}

__STATIC_INLINE uint32_t QSPI_GUARD_READ_RX(QSPI_T *qspi_base)
{
    while (QSPI_GET_RX_FIFO_EMPTY_FLAG(qspi_base));
    return QSPI_READ_RX(qspi_base);
}

/* Assert SS */
__STATIC_INLINE void QSPI_ASSERT_SS(gpio_t *ssel)
{
    gpio_write(ssel, 0);
}

/* De-assert SS */
__STATIC_INLINE void QSPI_DEASSERT_SS(gpio_t *ssel)
{
    gpio_write(ssel, 1);
}

static uint32_t qspi_fifo_depth(QSPI_T *qspi_base);
static qspi_status_t qspi_write_command(QSPI_T *qspi_base, const qspi_command_t *command);
static qspi_status_t qspi_write_instruction(QSPI_T *qspi_base, qspi_bus_width_t bus_width, uint32_t value);
static qspi_status_t qspi_write_address(QSPI_T *qspi_base, qspi_bus_width_t bus_width, qspi_address_size_t size, uint32_t value);
static qspi_status_t qspi_write_alt(QSPI_T *qspi_base, qspi_bus_width_t bus_width, qspi_alt_size_t size_bits, uint32_t value);
static qspi_status_t qspi_write_dummy_cycles(QSPI_T *qspi_base, uint32_t dummy_count);
static qspi_status_t qspi_switch_bus_width_n_data_direction(QSPI_T *qspi_base, qspi_bus_width_t bus_width, bool dir_in);

/**
 * @brief   Transmit QSPI data  until transmit limit is reached or TX FIFO is full
 *
 * @param   qspi_base   Base address of QSPI IP
 * @param   tx_data     QSPI data to transmit. Can be NULL for dummy transmit
 * @param   tx_size     Size of QSPI data to transmit. Can be smaller than transmit limit for dummy transmit
 * @param   tx_limit    Limit of QSPI data to transmit
 *
 * @return              Number of QSPI data that have been transmitted
 */
static uint32_t qspi_transmit(QSPI_T *qspi_base, const void *tx_data, uint32_t tx_size, uint32_t tx_limit);

/**
 * @brief   Receive QSPI data until receive limit is reached or RX FIFO is empty
 *
 * @param   qspi_base   Base address of QSPI IP
 * @param   rx_data     QSPI data receive buffer
 * @param   tx_size     Size of QSPI data receive buffer. Can be smaller than receive limit for dummy receive
 * @param   rx_limit    Limit of QSPI data to receive
 *
 * @return              Number of QSPI data that have been received
 */
static uint32_t qspi_receive(QSPI_T *qspi_base, void *rx_data, uint32_t rx_size, uint32_t rx_limit);

static uint32_t qspi_modinit_mask = 0;

static const struct nu_modinit_s qspi_modinit_tab[] = {
    {QSPI_0, QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk, QSPI0_RST, QSPI0_IRQn, NULL},
    {QSPI_1, QSPI1_MODULE, CLK_CLKSEL3_QSPI1SEL_PCLK1, MODULE_NoMsk, QSPI1_RST, QSPI1_IRQn, NULL},

    {NC, 0, 0, 0, 0, (IRQn_Type) 0, NULL}
};

qspi_status_t qspi_init(qspi_t *obj, PinName io0, PinName io1, PinName io2, PinName io3, PinName sclk, PinName ssel, uint32_t hz, uint8_t mode)
{
    // Determine which QSPI_x the pins are used for
    uint32_t qspi_data0 = pinmap_peripheral(io0, PinMap_QSPI_DATA0);
    uint32_t qspi_data1 = pinmap_peripheral(io1, PinMap_QSPI_DATA1);
    uint32_t qspi_data2 = pinmap_peripheral(io2, PinMap_QSPI_DATA2);
    uint32_t qspi_data3 = pinmap_peripheral(io3, PinMap_QSPI_DATA3);
    uint32_t qspi_sclk = pinmap_peripheral(sclk, PinMap_QSPI_SCLK);
    uint32_t qspi_ssel = pinmap_peripheral(ssel, PinMap_QSPI_SSEL);
    uint32_t qspi_data01 = pinmap_merge(qspi_data0, qspi_data1);
    uint32_t qspi_data23 = pinmap_merge(qspi_data2, qspi_data3);
    uint32_t qspi_data = pinmap_merge(qspi_data01, qspi_data23);
    uint32_t qspi_cntl = pinmap_merge(qspi_sclk, qspi_ssel);
    obj->qspi = (QSPIName) pinmap_merge(qspi_data, qspi_cntl);
    MBED_ASSERT((int)obj->qspi != NC);

    const struct nu_modinit_s *modinit = get_modinit(obj->qspi, qspi_modinit_tab);
    MBED_ASSERT(modinit != NULL);
    MBED_ASSERT(modinit->modname == (int) obj->qspi);

    obj->pin_data0 = io0;
    obj->pin_data1 = io1;
    obj->pin_data2 = io2;
    obj->pin_data3 = io3;
    obj->pin_sclk = sclk;
    obj->pin_ssel = ssel;

    pinmap_pinout(io0, PinMap_QSPI_DATA0);
    pinmap_pinout(io1, PinMap_QSPI_DATA1);
    pinmap_pinout(io2, PinMap_QSPI_DATA2);
    pinmap_pinout(io3, PinMap_QSPI_DATA3);
    pinmap_pinout(sclk, PinMap_QSPI_SCLK);
    pinmap_pinout(ssel, PinMap_QSPI_SSEL);

    // Select IP clock source
    CLK_SetModuleClock(modinit->clkidx, modinit->clksrc, modinit->clkdiv);

    // Enable IP clock
    CLK_EnableModuleClock(modinit->clkidx);

    // Reset this module
    SYS_ResetModule(modinit->rsetidx);

    // Mark this module to be inited.
    int i = modinit - qspi_modinit_tab;
    qspi_modinit_mask |= 1 << i;

    // IP base address
    QSPI_T *qspi_base = (QSPI_T *) NU_MODBASE(obj->qspi);

    // Configure bus clock
    QSPI_SetBusClock(qspi_base, hz);

    // master mode, mode 0, 8-bit data, MSB first
    QSPI_Open(qspi_base,
              QSPI_MASTER,                      // Only support master
              (mode == 0) ? QSPI_MODE_0 : 
              (mode == 1) ? QSPI_MODE_1 : 
              (mode == 2) ? QSPI_MODE_2 : 
              QSPI_MODE_3,                      // Polarity and phase
              8,                                // 8-bit data word
              QSPI_GetBusClock(qspi_base));     // QSPI bus clock as above
    // NOTE: Assume MSB first.
    QSPI_SET_MSB_FIRST(qspi_base);

    // Don't go auto SS because its behavior doesn't meet QSPI.
    QSPI_DisableAutoSS(qspi_base);

    // Assert/de-assert SS manually for explicit behavior. This will change pin function back to GPIO.
    gpio_init_out_ex(&obj->ssel, obj->pin_ssel, 1);

    return QSPI_STATUS_OK;
}

qspi_status_t qspi_free(qspi_t *obj)
{
    // Free SS GPIO object
    gpio_free(&obj->ssel);

    // IP base address
    QSPI_T *qspi_base = (QSPI_T *) NU_MODBASE(obj->qspi);

    QSPI_Close(qspi_base);

    const struct nu_modinit_s *modinit = get_modinit(obj->qspi, qspi_modinit_tab);
    MBED_ASSERT(modinit != NULL);
    MBED_ASSERT(modinit->modname == (int) obj->qspi);

    QSPI_DisableInt(((QSPI_T *) NU_MODBASE(obj->qspi)), (QSPI_FIFO_RXOV_INT_MASK | QSPI_FIFO_RXTH_INT_MASK | QSPI_FIFO_TXTH_INT_MASK));
    NVIC_DisableIRQ(modinit->irq_n);

    // Disable IP clock
    CLK_DisableModuleClock(modinit->clkidx);

    // Mark this module to be deinited.
    int i = modinit - qspi_modinit_tab;
    qspi_modinit_mask &= ~(1 << i);

    // Free up pins
    gpio_set(obj->pin_data0);
    gpio_set(obj->pin_data1);
    gpio_set(obj->pin_data2);
    gpio_set(obj->pin_data3);
    gpio_set(obj->pin_sclk);
    gpio_set(obj->pin_ssel);
    obj->pin_data0 = NC;
    obj->pin_data1 = NC;
    obj->pin_data2 = NC;
    obj->pin_data3 = NC;
    obj->pin_sclk = NC;
    obj->pin_ssel = NC;

    return QSPI_STATUS_OK;
}

qspi_status_t qspi_frequency(qspi_t *obj, int hz)
{
    QSPI_T *qspi_base = (QSPI_T *) NU_MODBASE(obj->qspi);

    QSPI_DISABLE_SYNC(qspi_base);

    QSPI_SetBusClock(qspi_base, hz);

    return QSPI_STATUS_OK;
}

/** Send a command and block of data
 *
 * @param obj QSPI object
 * @param command QSPI command
 * @param data TX buffer
 * @param[in,out] length in - TX buffer length in bytes, out - number of bytes written
 * @return QSPI_STATUS_OK if the data has been succesfully sent
           QSPI_STATUS_INVALID_PARAMETER if invalid parameter found
           QSPI_STATUS_ERROR otherwise
 */
qspi_status_t qspi_write(qspi_t *obj, const qspi_command_t *command, const void *data, size_t *length)
{
    MBED_ASSERT(obj && command && data && length);

    /* Assert SS */
    QSPI_ASSERT_SS(&obj->ssel);

    QSPI_T *qspi_base = (QSPI_T *) NU_MODBASE(obj->qspi);

    /* Enable QSPI */
    QSPI_ENABLE_SYNC(qspi_base);

    /* Write command */
    qspi_status_t status = qspi_write_command(qspi_base, command);
    if (status != QSPI_STATUS_OK) {
        goto clean_up;
    }

    /* Write data */
    /* Switch bus width for following data out */
    status = qspi_switch_bus_width_n_data_direction(qspi_base, command->data.bus_width, false);
    if (status != QSPI_STATUS_OK) {
        goto clean_up;
    }

    /* NOTE: Assume 8-bit data word */
    const uint8_t *data_beg = (const uint8_t *) data;
    const uint8_t *data_ind = data_beg;
    const uint8_t *data_end = data_beg + *length;

    /* Need only tx rmn */
    uint32_t tx_rmn = data_end - data_ind;

    /* Write Tx FIFO to the full */
    uint32_t n;
    while (tx_rmn) {
        n = qspi_transmit(qspi_base, data_ind, tx_rmn, tx_rmn);
        data_ind += n;
        tx_rmn -= n;
    }
    MBED_ASSERT(!tx_rmn);

    /* How many transferred */
    *length = data_ind - data_beg;

clean_up:
    /* Safe return with bus idle and SS de-asserted */

    /* No QSPI bus activity. Necessary to guarantee this QSPI transfer is really completed and the next one is clean. */
    while(QSPI_IS_BUSY(qspi_base));

    /* De-assert SS */
    QSPI_DEASSERT_SS(&obj->ssel);

    return status;
}

qspi_status_t qspi_command_transfer(qspi_t *obj, const qspi_command_t *command, const void *tx_data, size_t tx_size, void *rx_data, size_t rx_size)
{
    qspi_status_t status = QSPI_STATUS_OK;

    /* Assert SS */
    QSPI_ASSERT_SS(&obj->ssel);

    QSPI_T *qspi_base = (QSPI_T *) NU_MODBASE(obj->qspi);

    /* Enable QSPI */
    QSPI_ENABLE_SYNC(qspi_base);

    if ((tx_data == NULL || tx_size == 0) && (rx_data == NULL || rx_size == 0)) {
        /* Write command only */
        status = qspi_write_command(qspi_base, command);
        if (status != QSPI_STATUS_OK) {
            goto clean_up;
        }
    } else {
        if (tx_data != NULL && tx_size) {
            size_t tx_length = tx_size;
            status = qspi_write(obj, command, tx_data, &tx_length);
            if (status != QSPI_STATUS_OK) {
                goto clean_up;
            }
        }

        if (rx_data != NULL && rx_size) {
            size_t rx_length = rx_size;
            status = qspi_read(obj, command, rx_data, &rx_length);
            if (status != QSPI_STATUS_OK) {
                goto clean_up;
            }
        }
    }

clean_up:
    /* Safe return with bus idle and SS de-asserted */

    /* No QSPI bus activity. Necessary to guarantee this QSPI transfer is really completed and the next one is clean. */
    while(QSPI_IS_BUSY(qspi_base));

    /* De-assert SS */
    QSPI_DEASSERT_SS(&obj->ssel);
    
    return status;
}

qspi_status_t qspi_read(qspi_t *obj, const qspi_command_t *command, void *data, size_t *length)
{
    MBED_ASSERT(obj && command && data && length);

    /* Assert SS */
    QSPI_ASSERT_SS(&obj->ssel);

    QSPI_T *qspi_base = (QSPI_T *) NU_MODBASE(obj->qspi);

    /* Enable QSPI */
    QSPI_ENABLE_SYNC(qspi_base);

    /* Write command */
    qspi_status_t status = qspi_write_command(qspi_base, command);
    if (status != QSPI_STATUS_OK) {
        goto clean_up;
    }

    /* Read data */
    /* Switch bus width for following data in */
    status = qspi_switch_bus_width_n_data_direction(qspi_base, command->data.bus_width, true);
    if (status != QSPI_STATUS_OK) {
        goto clean_up;
    }

    /* No QSPI bus activity */
    while(QSPI_IS_BUSY(qspi_base));

    /* Clear RX FIFO */
    QSPI_ClearRxFIFO(qspi_base);

    /* NOTE: Assume 8-bit data word */
    uint8_t *data_beg = (uint8_t *) data;
    uint8_t *data_ind = data_beg;
    uint8_t *data_end = data_beg + *length;

    /* Need separate tx/rx rmn */
    uint32_t tx_rmn = data_end - data_ind;
    uint32_t rx_rmn = tx_rmn;

    /* To minimize tx/rx FIFO latency, try with just half the FIFO size first and continue with remaining. */
    uint32_t n = qspi_fifo_depth(qspi_base);
    while (rx_rmn) {
        n = NU_MIN(tx_rmn, n);
        n = qspi_transmit(qspi_base, NULL, 0, n);
        tx_rmn -= n;
        n = qspi_receive(qspi_base, data_ind, rx_rmn, rx_rmn);
        data_ind += n;
        rx_rmn -= n;
    }
    MBED_ASSERT(!tx_rmn && !rx_rmn);

    /* How many transferred */
    *length = data_ind - data_beg;

clean_up:
    /* Safe return with bus idle and SS de-asserted */

    /* No QSPI bus activity. Necessary to guarantee this QSPI transfer is really completed and the next one is clean. */
    while(QSPI_IS_BUSY(qspi_base));

    /* De-assert SS */
    QSPI_DEASSERT_SS(&obj->ssel);

    return status;
}

const PinMap *qspi_master_sclk_pinmap()
{
    return PinMap_QSPI_SCLK;
}

const PinMap *qspi_master_ssel_pinmap()
{
    return PinMap_QSPI_SSEL;
}

const PinMap *qspi_master_data0_pinmap()
{
    return PinMap_QSPI_DATA0;
}

const PinMap *qspi_master_data1_pinmap()
{
    return PinMap_QSPI_DATA1;
}

const PinMap *qspi_master_data2_pinmap()
{
    return PinMap_QSPI_DATA2;
}

const PinMap *qspi_master_data3_pinmap()
{
    return PinMap_QSPI_DATA3;
}

/* Return FIFO depth of the SPI peripheral */
static uint32_t qspi_fifo_depth(QSPI_T *qspi_base)
{
    MBED_ASSERT(qspi_base);

    return 8;
}

static qspi_status_t qspi_write_command(QSPI_T *qspi_base, const qspi_command_t *command)
{
    MBED_ASSERT(qspi_base && command);

    qspi_status_t status = QSPI_STATUS_OK;

    /* Write instruction */
    if (!command->instruction.disabled) {
        status = qspi_write_instruction(qspi_base, command->instruction.bus_width, command->instruction.value);
        if (status != QSPI_STATUS_OK) {
            return status;
        }
    }

    /* Write address */
    if (!command->address.disabled) {
        status = qspi_write_address(qspi_base, command->address.bus_width, command->address.size, command->address.value);
        if (status != QSPI_STATUS_OK) {
            return status;
        }
    }

    /* Write alt */
    if (!command->alt.disabled) {
        status = qspi_write_alt(qspi_base, command->alt.bus_width, command->alt.size, command->alt.value);
        if (status != QSPI_STATUS_OK) {
            return status;
        }
    }

    /* Write dummy cycles */
    status = qspi_write_dummy_cycles(qspi_base, command->dummy_count);
    if (status != QSPI_STATUS_OK) {
        return status;
    }

    return status;
}

static qspi_status_t qspi_write_instruction(QSPI_T *qspi_base, qspi_bus_width_t bus_width, uint32_t value)
{
    /* Switch bus width & direction */
    qspi_status_t status = qspi_switch_bus_width_n_data_direction(qspi_base, bus_width, false);
    if (status != QSPI_STATUS_OK) {
        return status;
    }

    /* Write instruction */
    QSPI_GUARD_WRITE_TX(qspi_base, value);

    return status;
}

static qspi_status_t qspi_write_address(QSPI_T *qspi_base, qspi_bus_width_t bus_width, qspi_address_size_t size, uint32_t value)
{
    MBED_ASSERT(qspi_base);

    /* Switch bus width & direction */
    qspi_status_t status = qspi_switch_bus_width_n_data_direction(qspi_base, bus_width, false);
    if (status != QSPI_STATUS_OK) {
        return status;
    }

    /* NOTE: Assume MSB first and 8-bit data word */
    switch (size) {
        case QSPI_CFG_ADDR_SIZE_32:
            QSPI_GUARD_WRITE_TX(qspi_base, (value >> 24) & 0xFF);
            MBED_FALLTHROUGH;
        case QSPI_CFG_ADDR_SIZE_24:
            QSPI_GUARD_WRITE_TX(qspi_base, (value >> 16) & 0xFF);
            MBED_FALLTHROUGH;
        case QSPI_CFG_ADDR_SIZE_16:
            QSPI_GUARD_WRITE_TX(qspi_base, (value >> 8) & 0xFF);
            MBED_FALLTHROUGH;
        case QSPI_CFG_ADDR_SIZE_8:
            QSPI_GUARD_WRITE_TX(qspi_base, value & 0xFF);
            break;

        default:
            return QSPI_STATUS_INVALID_PARAMETER;
    }

    return status;
}

static qspi_status_t qspi_write_alt(QSPI_T *qspi_base, qspi_bus_width_t bus_width, qspi_alt_size_t size_bits, uint32_t value)
{
    MBED_ASSERT(qspi_base);

    /* Switch bus width & direction */
    qspi_status_t status = qspi_switch_bus_width_n_data_direction(qspi_base, bus_width, false);
    if (status != QSPI_STATUS_OK) {
        return status;
    }

    /* NOTE: Assume MSB first and 8-bit data word */
    /* NOTE: Just support 8-bit aligned */
    switch (size_bits) {
        case QSPI_CFG_ALT_SIZE_32:
            QSPI_GUARD_WRITE_TX(qspi_base, (value >> 24) & 0xFF);
            MBED_FALLTHROUGH;
        case QSPI_CFG_ALT_SIZE_24:
            QSPI_GUARD_WRITE_TX(qspi_base, (value >> 16) & 0xFF);
            MBED_FALLTHROUGH;
        case QSPI_CFG_ALT_SIZE_16:
            QSPI_GUARD_WRITE_TX(qspi_base, (value >> 8) & 0xFF);
            MBED_FALLTHROUGH;
        case QSPI_CFG_ALT_SIZE_8:
            QSPI_GUARD_WRITE_TX(qspi_base, value & 0xFF);
            break;

        default:
            return QSPI_STATUS_INVALID_PARAMETER;
    }

    return status;
}

static qspi_status_t qspi_write_dummy_cycles(QSPI_T *qspi_base, uint32_t dummy_count)
{
    MBED_ASSERT(qspi_base);

    /* Support only even number of dummy cycles */
    if (dummy_count & 1) {
        return QSPI_STATUS_INVALID_PARAMETER;
    }

    /* Assume data word size hard-coded to 8-bit, plus bus width switched to Quad mode,
     * dummy writes = dummy cycles * 4 / 8 = dummy cycles / 2 */

    /* Switch to Quad mode */
    qspi_status_t status = qspi_switch_bus_width_n_data_direction(qspi_base, QSPI_CFG_BUS_QUAD, false);
    if (status != QSPI_STATUS_OK) {
        return status;
    }

    /* Write dummy */
    uint32_t dummy_write_rmn = dummy_count / 2;
    while (dummy_write_rmn) {
        dummy_write_rmn = dummy_write_rmn - qspi_transmit(qspi_base, NULL, 0, dummy_write_rmn);
    }

    return status;
}

static qspi_status_t qspi_switch_bus_width_n_data_direction(QSPI_T *qspi_base, qspi_bus_width_t bus_width, bool dir_in)
{
    /* Hazards of switching bus width and data direction
     *
     * 1. Don't switch bus width and data direction when QSPI bus is still busy.
     * 2. Don't have Dual/Quad modes enabled simultaneously.
     */
    while(QSPI_IS_BUSY(qspi_base));
    switch (bus_width) {
        /* Single mode */
        case QSPI_CFG_BUS_SINGLE:
            QSPI_DISABLE_DUAL_MODE(qspi_base);
            QSPI_DISABLE_QUAD_MODE(qspi_base);
            break;

        /* Dual mode */
        case QSPI_CFG_BUS_DUAL:
            QSPI_DISABLE_QUAD_MODE(qspi_base);
            if (dir_in) {
                QSPI_ENABLE_DUAL_INPUT_MODE(qspi_base);
            } else {
                QSPI_ENABLE_DUAL_OUTPUT_MODE(qspi_base);
            }
            break;

        /* Quad mode */
        case QSPI_CFG_BUS_QUAD:
            QSPI_DISABLE_DUAL_MODE(qspi_base);
            if (dir_in) {
                QSPI_ENABLE_QUAD_INPUT_MODE(qspi_base);
            } else {
                QSPI_ENABLE_QUAD_OUTPUT_MODE(qspi_base);
            }
            break;

        default:
            return QSPI_STATUS_INVALID_PARAMETER;
    }

    return QSPI_STATUS_OK;
}

static uint32_t qspi_transmit(QSPI_T *qspi_base, const void *tx_data, uint32_t tx_size, uint32_t tx_limit)
{
    MBED_ASSERT(qspi_base);

    uint32_t xfered = 0;
    uint32_t rmn = tx_limit;

     /* NOTE: Assume 8-bit data word */
    uint8_t *tx_data_ind = tx_data ? (uint8_t *) tx_data : NULL;
    uint8_t *tx_data_end = tx_data_ind ? (tx_data_ind + tx_size) : NULL;

    while (rmn && !QSPI_GET_TX_FIFO_FULL_FLAG(qspi_base)) {
        if (tx_data_ind < tx_data_end) {
            QSPI_WRITE_TX(qspi_base, *tx_data_ind ++);
        } else {
            /* Transmit dummy when transmit buffer gets empty */
            QSPI_WRITE_TX(qspi_base, 0);
        }

        xfered ++;
        rmn --;
    }

    /* Number of QSPI data that have been transmitted */
    return xfered;
}

static uint32_t qspi_receive(QSPI_T *qspi_base, void *rx_data, uint32_t rx_size, uint32_t rx_limit)
{
    MBED_ASSERT(qspi_base);

    uint32_t xfered = 0;
    uint32_t rmn = rx_limit;

     /* NOTE: Assume 8-bit data word */
    uint8_t *rx_data_ind = rx_data ? (uint8_t *) rx_data : NULL;
    uint8_t *rx_data_end = rx_data_ind ? (rx_data_ind + rx_size) : NULL;

    while (rmn && !QSPI_GET_RX_FIFO_EMPTY_FLAG(qspi_base)) {
        if (rx_data_ind < rx_data_end) {
            *rx_data_ind ++ = QSPI_READ_RX(qspi_base);
        } else {
            /* Disregard when receive buffer gets full */
            QSPI_READ_RX(qspi_base);
        }

        xfered ++;
        rmn --;
    }

    /* Number of QSPI data that have been received */
    return xfered;
}

#endif
