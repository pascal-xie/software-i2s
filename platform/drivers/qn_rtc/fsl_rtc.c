/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_rtc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SECONDS_IN_A_DAY (86400U)
#define SECONDS_IN_A_HOUR (3600U)
#define SECONDS_IN_A_MINUTE (60U)
#define DAYS_IN_A_YEAR (365U)
#define YEAR_RANGE_START (1970U)
#define YEAR_RANGE_END (2099U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Checks whether the date and time passed in is valid
 *
 * @param datetime Pointer to structure where the date and time details are stored
 *
 * @return Returns false if the date & time details are out of range; true if in range
 */
static bool RTC_CheckDatetimeFormat(const rtc_datetime_t *datetime);

/*!
 * @brief Converts time data from datetime to seconds
 *
 * @param datetime Pointer to datetime structure where the date and time details are stored
 *
 * @return The result of the conversion in seconds
 */
static uint32_t RTC_ConvertDatetimeToSeconds(const rtc_datetime_t *datetime);

/*!
 * @brief Converts time data from seconds to a datetime structure
 *
 * @param seconds  Seconds value that needs to be converted to datetime format
 * @param datetime Pointer to the datetime structure where the result of the conversion is stored
 */
static void RTC_ConvertSecondsToDatetime(uint32_t seconds, rtc_datetime_t *datetime);

/*******************************************************************************
 * Code
 ******************************************************************************/
static bool RTC_CheckDatetimeFormat(const rtc_datetime_t *datetime)
{
    assert(datetime != NULL);

    /* Table of days in a month for a non leap year. First entry in the table is not used,
     * valid months start from 1
     */
    uint8_t daysPerMonth[] = {0U, 31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

    /* Check year, month, hour, minute, seconds */
    if ((datetime->year < YEAR_RANGE_START) || (datetime->year > YEAR_RANGE_END) || (datetime->month > 12U) ||
        (datetime->month < 1U) || (datetime->hour >= 24U) || (datetime->minute >= 60U) || (datetime->second >= 60U))
    {
        /* If not correct then error*/
        return false;
    }

    /* Adjust the days in February for a leap year */
    if ((((datetime->year & 3U) == 0) && (datetime->year % 100 != 0)) || (datetime->year % 400 == 0))
    {
        daysPerMonth[2] = 29U;
    }

    /* Check the validity of the day */
    if ((datetime->day > daysPerMonth[datetime->month]) || (datetime->day < 1U))
    {
        return false;
    }

    return true;
}

static uint32_t RTC_ConvertDatetimeToSeconds(const rtc_datetime_t *datetime)
{
    assert(datetime != NULL);

    /* Number of days from begin of the non Leap-year*/
    /* Number of days from begin of the non Leap-year*/
    uint16_t monthDays[] = {0U, 0U, 31U, 59U, 90U, 120U, 151U, 181U, 212U, 243U, 273U, 304U, 334U};
    uint32_t seconds;

    /* Compute number of days from 1970 till given year*/
    seconds = (datetime->year - 1970U) * DAYS_IN_A_YEAR;
    /* Add leap year days */
    seconds += ((datetime->year / 4) - (1970U / 4));
    /* Add number of days till given month*/
    seconds += monthDays[datetime->month];
    /* Add days in given month. We subtract the current day as it is
     * represented in the hours, minutes and seconds field*/
    seconds += (datetime->day - 1);
    /* For leap year if month less than or equal to Febraury, decrement day counter*/
    if ((!(datetime->year & 3U)) && (datetime->month <= 2U))
    {
        seconds--;
    }

    seconds = (seconds * SECONDS_IN_A_DAY) + (datetime->hour * SECONDS_IN_A_HOUR) +
              (datetime->minute * SECONDS_IN_A_MINUTE) + datetime->second;

    return seconds;
}

static void RTC_ConvertSecondsToDatetime(uint32_t seconds, rtc_datetime_t *datetime)
{
    assert(datetime != NULL);

    uint32_t x;
    uint32_t secondsRemaining, days;
    uint16_t daysInYear;
    /* Table of days in a month for a non leap year. First entry in the table is not used,
     * valid months start from 1
     */
    uint8_t daysPerMonth[] = {0U, 31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

    /* Start with the seconds value that is passed in to be converted to date time format */
    secondsRemaining = seconds;

    /* Calcuate the number of days, we add 1 for the current day which is represented in the
     * hours and seconds field
     */
    days = secondsRemaining / SECONDS_IN_A_DAY + 1;

    /* Update seconds left*/
    secondsRemaining = secondsRemaining % SECONDS_IN_A_DAY;

    /* Calculate the datetime hour, minute and second fields */
    datetime->hour = secondsRemaining / SECONDS_IN_A_HOUR;
    secondsRemaining = secondsRemaining % SECONDS_IN_A_HOUR;
    datetime->minute = secondsRemaining / 60U;
    datetime->second = secondsRemaining % SECONDS_IN_A_MINUTE;

    /* Calculate year */
    daysInYear = DAYS_IN_A_YEAR;
    datetime->year = YEAR_RANGE_START;
    while (days > daysInYear)
    {
        /* Decrease day count by a year and increment year by 1 */
        days -= daysInYear;
        datetime->year++;

        /* Adjust the number of days for a leap year */
        if (datetime->year & 3U)
        {
            daysInYear = DAYS_IN_A_YEAR;
        }
        else
        {
            daysInYear = DAYS_IN_A_YEAR + 1;
        }
    }

    /* Adjust the days in February for a leap year */
    if (!(datetime->year & 3U))
    {
        daysPerMonth[2] = 29U;
    }

    for (x = 1U; x <= 12U; x++)
    {
        if (days <= daysPerMonth[x])
        {
            datetime->month = x;
            break;
        }
        else
        {
            days -= daysPerMonth[x];
        }
    }

    datetime->day = days;
}

status_t RTC_SetDatetime(RTC_Type *base, const rtc_datetime_t *datetime)
{
    assert(datetime != NULL);

    /* Return error if the time provided is not valid */
    if (!(RTC_CheckDatetimeFormat(datetime)))
    {
        return kStatus_InvalidArgument;
    }

    /* Set time in seconds */
    base->SEC = RTC_ConvertDatetimeToSeconds(datetime);
    /* Wait for synchronize */
    while (base->STATUS & RTC_STATUS_SEC_SYNC_MASK)
        ;

    base->CTRL |= RTC_CTRL_CFG_MASK;
    /* Wait for synchronize */
    while (base->STATUS & RTC_STATUS_CTRL_SYNC_MASK)
        ;

    return kStatus_Success;
}

void RTC_GetDatetime(RTC_Type *base, rtc_datetime_t *datetime)
{
    assert(datetime != NULL);

    uint32_t seconds = base->SEC;
    RTC_ConvertSecondsToDatetime(seconds, datetime);
}

void RTC_Calibration(RTC_Type *base, rtc_calibration_direction_t dir, uint16_t value)
{
    base->CAL = (base->CAL & ~(RTC_CAL_PPM_MASK | RTC_CAL_DIR_MASK)) | (RTC_CAL_DIR(dir) | RTC_CAL_PPM(value));
    while (base->STATUS & RTC_STATUS_CALIB_SYNC_MASK)
        ;

    base->CTRL |= RTC_CTRL_CAL_EN_MASK;
    while (base->STATUS & RTC_STATUS_CTRL_SYNC_MASK)
        ;
}

void RTC_EnableFreeRunningReset(RTC_Type *base, bool enable)
{
    uint32_t tmp32;

    tmp32 = base->CNT2_CTRL;
    if (enable)
    {
        tmp32 |= RTC_CNT2_CTRL_CNT2_RST_MASK;
    }
    else
    {
        tmp32 &= ~RTC_CNT2_CTRL_CNT2_RST_MASK;
    }

    base->CNT2_CTRL = (tmp32 | RTC_FREERUNNING_MAGIC_NUM);
    while (base->STATUS & RTC_STATUS_FREE_SYNC_MASK)
        ;
}

void RTC_SetFreeRunningInterruptThreshold(RTC_Type *base, uint32_t value)
{
    base->THR_INT = value;
    while (base->STATUS & RTC_STATUS_THR_INT_SYNC_MASK)
        ;
}

void RTC_SetFreeRunningResetThreshold(RTC_Type *base, uint32_t value)
{
    base->THR_RST = value;
    while (base->STATUS & RTC_STATUS_THR_RST_SYNC_MASK)
        ;
}

void RTC_FreeRunningEnable(RTC_Type *base, bool enable)
{
    uint32_t tmp32;

    tmp32 = base->CNT2_CTRL;
    if (enable)
    {
        tmp32 |= RTC_CNT2_CTRL_CNT2_EN_MASK;
    }
    else
    {
        tmp32 &= ~RTC_CNT2_CTRL_CNT2_EN_MASK;
    }

    base->CNT2_CTRL = (tmp32 | RTC_FREERUNNING_MAGIC_NUM);
    while (base->STATUS & RTC_STATUS_FREE_SYNC_MASK)
        ;
}

void RTC_EnableInterrupts(RTC_Type *base, uint32_t mask)
{
    uint32_t tmp32;

    if (mask & kRTC_SecondInterruptEnable)
    {
        base->CTRL |= RTC_CTRL_SEC_INT_EN_MASK;
        while (base->STATUS & RTC_STATUS_CTRL_SYNC_MASK)
            ;
    }
    if (mask & kRTC_FreeRunningInterruptEnable)
    {
        tmp32 = base->CNT2_CTRL;
        tmp32 |= (RTC_CNT2_CTRL_CNT2_INT_EN_MASK | RTC_CNT2_CTRL_CNT2_WAKEUP_MASK);
        base->CNT2_CTRL = (tmp32 | RTC_FREERUNNING_MAGIC_NUM);
        while (base->STATUS & RTC_STATUS_FREE_SYNC_MASK)
            ;
    }
}

void RTC_DisableInterrupts(RTC_Type *base, uint32_t mask)
{
    uint32_t tmp32;

    if (mask & kRTC_SecondInterruptEnable)
    {
        base->CTRL &= ~RTC_CTRL_SEC_INT_EN_MASK;
        while (base->STATUS & RTC_STATUS_CTRL_SYNC_MASK)
            ;
    }
    if (mask & kRTC_FreeRunningInterruptEnable)
    {
        tmp32 = base->CNT2_CTRL;
        tmp32 &= ~(RTC_CNT2_CTRL_CNT2_INT_EN_MASK | RTC_CNT2_CTRL_CNT2_WAKEUP_MASK);
        base->CNT2_CTRL = (tmp32 | RTC_FREERUNNING_MAGIC_NUM);
        while (base->STATUS & RTC_STATUS_FREE_SYNC_MASK)
            ;
    }
}

void RTC_ClearStatusFlags(RTC_Type *base, uint32_t mask)
{
    base->STATUS = mask;
    while (base->STATUS & RTC_STATUS_STATUS_SYNC_MASK)
        ;
}
