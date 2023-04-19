/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
 * Author: Luca Erbetta
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * This is a quick and dirty test of the RTC time keeping functionality with
 * the 32.768KHz crystal on board the compute unit. A proper driver will need to
 * be developed!
 */

// PA14 -> OSC32_IN
// PA15 -> OSC32_OUT

#include <miosix.h>

using namespace miosix;

typedef struct RTC_Date
{
    int day;
    int month;
    int year;
} RTC_Date;
typedef struct RTC_Time
{
    int hour;
    int minutes;
    int seconds;
} RTC_Time;

int main()
{
    for (int i = 5; i > 0; i--)
    {
        printf("Waiting... %d seconds left\n", i);
        Thread::sleep(1000);
    }

    // Enable clock to RTC and PWR peripherals
    RCC->APB1ENR |= RCC_APB1ENR_RTCEN;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC_SYNC();

    // Disable backup domain write protection
    PWR->CR1 |= PWR_CR1_DBP;

    // Enable the LSE clock
    RCC->BDCR |= RCC_BDCR_LSEDRV;    // High drive
    RCC->BDCR |= RCC_BDCR_LSEON;     // Enable LSE clock
    RCC->BDCR |= RCC_BDCR_RTCSEL_0;  // LSE as RTC clock
    RCC->BDCR |= RCC_BDCR_RTCEN;     // Enable RTC clock
    RCC_SYNC();

    while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0)
    {
        printf("LSE clock status %d\n", (RCC->BDCR & RCC_BDCR_LSERDY) != 0);
        Thread::sleep(250);
    }
    printf("LSE clock now stable\n");

    // Enable access to RTC registers
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    // Enter RTC initialization mode
    RTC->ISR |= RTC_ISR_INIT;
    while (!(RTC->ISR & RTC_ISR_INITF))
    {
        printf("Waiting for RTC to enter initialization mode 0x%lx\n",
               RTC->ISR);
        Thread::sleep(250);
    }

    // The RTC prescalers are configured by default for a 32.768KHz clock
    // f_CK_APRE = f_RTCCLK  / (PREDIV_A + 1) = f_RTCCLK / (127 + 1)
    // f_CK_SPRE = f_CK_APRE / (PREDIV_B + 1) = f_RTCCLK / (255 + 1)

    uint32_t ht, hu, mt, mu, st, su, yt, yu, wd, dt, du;

    // Configure the time
    ht      = 17 / 10;
    hu      = 17 % 10;
    mt      = 15 / 10;
    mu      = 15 % 10;
    st      = 30 / 10;
    su      = 30 % 10;
    RTC->TR = ht << RTC_TR_HT_Pos | hu << RTC_TR_HU_Pos | mt << RTC_TR_MNT_Pos |
              mu << RTC_TR_MNU_Pos | st << RTC_TR_ST_Pos | su << RTC_TR_SU_Pos;

    // Configure the date
    yt      = 23 / 10;
    yu      = 23 % 10;
    wd      = 1;
    mt      = 3 / 10;
    mu      = 3 % 10;
    dt      = 27 / 10;
    du      = 27 % 10;
    RTC->DR = yt << RTC_DR_YT_Pos | yu << RTC_DR_YU_Pos | wd << RTC_DR_WDU_Pos |
              mt << RTC_DR_MT_Pos | mu << RTC_DR_MU_Pos | dt << RTC_DR_DT_Pos |
              du << RTC_DR_DU_Pos;

    // Exit RTC initialization mode
    RTC->ISR &= ~RTC_ISR_INIT;

    // Reactivate the write protection
    RTC->WPR = 0xFF;
    PWR->CR1 &= ~PWR_CR1_DBP;

    Thread::sleep(1000);

    RTC_Time time;
    RTC_Date date;

    while (true)
    {
        uint32_t rtc_tr = RTC->TR;
        uint32_t rtc_dr = RTC->DR;

        int hour_tens    = (rtc_tr & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos;
        int hour_units   = (rtc_tr & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos;
        time.hour        = (hour_tens * 10) + hour_units;
        int mins_tens    = (rtc_tr & RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos;
        int mins_units   = (rtc_tr & RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos;
        time.minutes     = (mins_tens * 10) + mins_units;
        int second_tens  = (rtc_tr & RTC_TR_ST_Msk) >> RTC_TR_ST_Pos;
        int second_units = (rtc_tr & RTC_TR_SU_Msk) >> RTC_TR_SU_Pos;
        time.seconds     = (second_tens * 10) + second_units;

        int year_tens  = (rtc_dr & RTC_DR_YT_Msk) >> RTC_DR_YT_Pos;
        int year_unit  = (rtc_dr & RTC_DR_YU_Msk) >> RTC_DR_YU_Pos;
        date.year      = (year_tens * 10) + year_unit + 2000;
        int month_tens = (rtc_dr & RTC_DR_MT_Msk) >> RTC_DR_MT_Pos;
        int month_unit = (rtc_dr & RTC_DR_MU_Msk) >> RTC_DR_MU_Pos;
        date.month     = (month_tens * 10) + month_unit;
        int day_tens   = (rtc_dr & RTC_DR_DT_Msk) >> RTC_DR_DT_Pos;
        int day_units  = (rtc_dr & RTC_DR_DU_Msk) >> RTC_DR_DU_Pos;
        date.day       = (day_tens * 10) + day_units;

        printf("%04d/%02d/%02d %02d:%02d:%02d\n", date.year, date.month,
               date.day, time.hour, time.minutes, time.seconds);
        Thread::sleep(1000);
    }
}
