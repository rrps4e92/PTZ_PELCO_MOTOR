#include "tmc2160.h"

/* SPI handle comes from main.c */
extern SPI_HandleTypeDef hspi1;

/* ===========================================================
   PIN DEFINES
   =========================================================== */

#define PAN_CS_PORT      GPIOA
#define PAN_CS_PIN       GPIO_PIN_4

#define TILT_CS_PORT     GPIOB
#define TILT_CS_PIN      GPIO_PIN_14

/* ===========================================================
   NORMAL RUN SETTINGS
   =========================================================== */

#define PAN_MICROSTEP_RUN       64
#define PAN_IRUN_RUN            8
#define PAN_IHOLD_RUN           5
#define PAN_IHOLDDELAY_RUN      6

#define TILT_MICROSTEP_RUN      64
#define TILT_IRUN_RUN           8
#define TILT_IHOLD_RUN          5
#define TILT_IHOLDDELAY_RUN     6

/* ===========================================================
   HOMING SETTINGS
   =========================================================== */

#define PAN_MICROSTEP_HOME      128

#define TILT_MICROSTEP_HOME     128

/* ===========================================================
   PRIVATE FUNCTIONS
   =========================================================== */

static void TMC_Write_PAN(uint8_t addr, uint32_t data);
static void TMC_Write_TILT(uint8_t addr, uint32_t data);

static void TMC2160_Configure_PAN(
        uint16_t microstep,
        uint8_t irun_val,
        uint8_t ihold_val,
        uint8_t iholddelay_val);

static void TMC2160_Configure_TILT(
        uint16_t microstep,
        uint8_t irun_val,
        uint8_t ihold_val,
        uint8_t iholddelay_val);

/* ===========================================================
   SPI WRITE PAN
   =========================================================== */

static void TMC_Write_PAN(uint8_t addr, uint32_t data)
{
    uint8_t tx[5];

    tx[0] = addr | 0x80;
    tx[1] = (data >> 24) & 0xFF;
    tx[2] = (data >> 16) & 0xFF;
    tx[3] = (data >> 8) & 0xFF;
    tx[4] = data & 0xFF;

    HAL_GPIO_WritePin(
            PAN_CS_PORT,
            PAN_CS_PIN,
            GPIO_PIN_RESET);

    __NOP();
    __NOP();

    HAL_SPI_Transmit(
            &hspi1,
            tx,
            5,
            HAL_MAX_DELAY);

    HAL_GPIO_WritePin(
            PAN_CS_PORT,
            PAN_CS_PIN,
            GPIO_PIN_SET);

    __NOP();
    __NOP();
}

/* ===========================================================
   SPI WRITE TILT
   =========================================================== */

static void TMC_Write_TILT(uint8_t addr, uint32_t data)
{
    uint8_t tx[5];

    tx[0] = addr | 0x80;
    tx[1] = (data >> 24) & 0xFF;
    tx[2] = (data >> 16) & 0xFF;
    tx[3] = (data >> 8) & 0xFF;
    tx[4] = data & 0xFF;

    HAL_GPIO_WritePin(
            TILT_CS_PORT,
            TILT_CS_PIN,
            GPIO_PIN_RESET);

    __NOP();
    __NOP();

    HAL_SPI_Transmit(
            &hspi1,
            tx,
            5,
            HAL_MAX_DELAY);

    HAL_GPIO_WritePin(
            TILT_CS_PORT,
            TILT_CS_PIN,
            GPIO_PIN_SET);

    __NOP();
    __NOP();
}

/* ===========================================================
   CONFIGURE PAN
   =========================================================== */

static void TMC2160_Configure_PAN(
        uint16_t microstep,
        uint8_t irun_val,
        uint8_t ihold_val,
        uint8_t iholddelay_val)
{
    if(irun_val > 31)
        irun_val = 31;

    if(ihold_val > 31)
        ihold_val = 31;

    if(iholddelay_val > 15)
        iholddelay_val = 15;

    if(ihold_val > irun_val)
        ihold_val = irun_val;

    uint32_t mres;

    switch(microstep)
    {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        default:  mres = 4; break;
    }

    TMC_Write_PAN(0x00, 0x00000004);

    uint32_t chopconf =
            (mres << 24)
            | 0x000100C3;

    TMC_Write_PAN(0x6C, chopconf);

    uint32_t ihold_irun =
            ((uint32_t)iholddelay_val << 16)
            | ((uint32_t)irun_val << 8)
            | ihold_val;

    TMC_Write_PAN(0x10, ihold_irun);
}

/* ===========================================================
   CONFIGURE TILT
   =========================================================== */

static void TMC2160_Configure_TILT(
        uint16_t microstep,
        uint8_t irun_val,
        uint8_t ihold_val,
        uint8_t iholddelay_val)
{
    if(irun_val > 31)
        irun_val = 31;

    if(ihold_val > 31)
        ihold_val = 31;

    if(iholddelay_val > 15)
        iholddelay_val = 15;

    if(ihold_val > irun_val)
        ihold_val = irun_val;

    uint32_t mres;

    switch(microstep)
    {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        default:  mres = 4; break;
    }

    TMC_Write_TILT(0x00, 0x00000004);

    uint32_t chopconf =
            (mres << 24)
            | 0x000100C3;

    TMC_Write_TILT(0x6C, chopconf);

    uint32_t ihold_irun =
            ((uint32_t)iholddelay_val << 16)
            | ((uint32_t)irun_val << 8)
            | ihold_val;

    TMC_Write_TILT(0x10, ihold_irun);
}

/* ===========================================================
   PUBLIC FUNCTIONS
   =========================================================== */

void TMC2160_Init_PAN(void)
{
    TMC2160_Configure_PAN(
            PAN_MICROSTEP_RUN,
            PAN_IRUN_RUN,
            PAN_IHOLD_RUN,
            PAN_IHOLDDELAY_RUN);
}

void TMC2160_Init_TILT(void)
{
    TMC2160_Configure_TILT(
            TILT_MICROSTEP_RUN,
            TILT_IRUN_RUN,
            TILT_IHOLD_RUN,
            TILT_IHOLDDELAY_RUN);
}

void TMC2160_PAN_HomingMode(void)
{
    TMC2160_Configure_PAN(
            PAN_MICROSTEP_HOME,
            PAN_IRUN_RUN,
            PAN_IHOLD_RUN,
            PAN_IHOLDDELAY_RUN);
}

void TMC2160_PAN_RunMode(void)
{
    TMC2160_Configure_PAN(
            PAN_MICROSTEP_RUN,
            PAN_IRUN_RUN,
            PAN_IHOLD_RUN,
            PAN_IHOLDDELAY_RUN);
}

void TMC2160_TILT_HomingMode(void)
{
    TMC2160_Configure_TILT(
            TILT_MICROSTEP_HOME,
            TILT_IRUN_RUN,
            TILT_IHOLD_RUN,
            TILT_IHOLDDELAY_RUN);
}

void TMC2160_TILT_RunMode(void)
{
    TMC2160_Configure_TILT(
            TILT_MICROSTEP_RUN,
            TILT_IRUN_RUN,
            TILT_IHOLD_RUN,
            TILT_IHOLDDELAY_RUN);
}
