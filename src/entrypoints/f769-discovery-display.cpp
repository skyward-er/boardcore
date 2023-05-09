#include <miosix.h>

/**
 * The 4-inch 800x480 LCD-TFT color display with capacitive touch panel is
 * connected to the MIPI DSI interface of the STM32F769NIH6.
 *
 * RGB vertical stripe of pixel arrangement.
 *
 * Up to two lanes of MIPI/DSI data.
 *
 * The self-capacitive touch panel on LCD has an I2C interface whose address is
 * 0x1010100. It also provides an interrupt signal.
 *
 * Acronyms:
 * - DSI: Display Serial Interface
 * - MIPI: Mobile Industry Processor Interface
 *
 * MIPI had defined 3 sets of standards:
 * - DBI: Display Bus Interface
 *   For displays with integrated display controller and frame buffer
 * - DCS: Display Command Set
 *   Set of commands to be used with display supporting the MIPI-DBI interface.
 * - DPI: Display Pixel Interface
 *   Standards for displays without controller nor frame buffer.
 *
 * To decrease the number of wires between MCU and displays, the MIPI display
 * working group has defined the DSI (Display Serial Interface).
 * The DSI encapsulates either DBI commands (called command mode) or DPI signals
 * (called video mode) and transmits them to the display in a serial manner.
 * This enables getting interfaced with a standard display using only four or
 * six pins, and achieving the same performance than a DPI.
 *
 * The DSI standard defines two operating modes for the DSI host and the DSI
 * display:
 * - The command mode: Transactions sends commands and data to the display. The
 * display needs to incorporate a controller and a frame buffer.
 * - The video mode: A real time pixel stream. The display module relies on the
 * host processor to provide the image data at a sufficient bandwidth to avoid
 * flicker or other visible artifacts in the displayed image.
 *
 * The DSI is a packet based protocol. The parallel data and the commands are
 * encapsulated into packets and upended with packet-protocol information and
 * headers.
 *
 * There are two types of packets: short packets and long packets.
 *
 * The LCD on the Discovery 769 uses a DSI interface in video mode (the display
 * does not have an integrated frame buffer).
 *
 * The LCD controller is one of:
 * - ST7701S
 * - ILI9806E
 * - OTM8009A <---
 */

#if HSE_VALUE != 25000000
#error "This driver only supports HSE at 25MHz"
#endif

using namespace miosix;

GpioPin dsiTe(GPIOJ_BASE, 2);
GpioPin dsiReset(GPIOJ_BASE, 15);

uint32_t HSYNC = 1;    // Horizontal start active time
uint32_t HFP   = 16;   // Horizontal Front Porch time
uint32_t HBP   = 15;   // Horizontal Back Porch time
uint32_t HACT  = 800;  // Horizontal Active time = X size
uint32_t VSYNC = 2;    // Vertical start active time
uint32_t VFP   = 34;   // Vertical Front Porch time
uint32_t VBP   = 34;   // Vertical Back Porch time
uint32_t VACT  = 480;  // Vertical Active time = Y size

/**
 * @brief  DCS or Generic short write command
 * @param  ChannelID: Virtual channel ID.
 * @param  Mode: DSI short packet data type.
 *               This parameter can be any value of @ref
 * DSI_SHORT_WRITE_PKT_Data_Type.
 * @param  Param1: DSC command or first generic parameter.
 *                 This parameter can be any value of @ref DSI_DCS_Command or a
 *                 generic command code.
 * @param  Param2: DSC parameter or second generic parameter.
 * @retval HAL status
 */
void HAL_DSI_ShortWrite(uint32_t ChannelID, uint32_t Mode, uint32_t Param1,
                        uint32_t Param2)
{
    auto start = getTick();

    // Wait for Command FIFO Empty
    while (!(DSI->GPSR & DSI_GPSR_CMDFE))
        if (getTick() - start > 1000)
        {
            printf("Timeout 1\n");
        };

    // Configure the packet to send a short DCS command with 0 or 1 parameter
    DSI->GHCR = (Mode | (ChannelID << 6) | (Param1 << 8) | (Param2 << 16));
}

/**
 * @brief  DCS or Generic long write command
 * @param  ChannelID: Virtual channel ID.
 * @param  Mode: DSI long packet data type.
 *               This parameter can be any value of @ref
 * DSI_LONG_WRITE_PKT_Data_Type.
 * @param  NbParams: Number of parameters.
 * @param  Param1: DSC command or first generic parameter.
 *                 This parameter can be any value of @ref DSI_DCS_Command or a
 *                 generic command code
 * @param  ParametersTable: Pointer to parameter values table.
 * @retval HAL status
 */
void HAL_DSI_LongWrite(uint32_t ChannelID, uint32_t Mode, uint32_t NbParams,
                       uint32_t Param1, uint8_t *ParametersTable)
{
    auto start = getTick();

    // Wait for Command FIFO Empty
    while (!(DSI->GPSR & DSI_GPSR_CMDFE))
        if (getTick() - start > 1000)
        {
            printf("Timeout 2\n");
        };

    // Set the DCS code hexadecimal on payload byte 1, and the other parameters
    // on the write FIFO command
    for (uint32_t uicounter = 0; uicounter < NbParams;)
    {
        if (uicounter == 0x00)
        {
            DSI->GPDR =
                (Param1 | ((uint32_t)(*(ParametersTable + uicounter)) << 8) |
                 ((uint32_t)(*(ParametersTable + uicounter + 1)) << 16) |
                 ((uint32_t)(*(ParametersTable + uicounter + 2)) << 24));
            uicounter += 3;
        }
        else
        {
            DSI->GPDR =
                ((uint32_t)(*(ParametersTable + uicounter)) |
                 ((uint32_t)(*(ParametersTable + uicounter + 1)) << 8) |
                 ((uint32_t)(*(ParametersTable + uicounter + 2)) << 16) |
                 ((uint32_t)(*(ParametersTable + uicounter + 3)) << 24));
            uicounter += 4;
        }
    }

    // Configure the packet to send a long DCS command
    DSI->GHCR = (Mode | (ChannelID << 6) | (((NbParams + 1) & 0x00FF) << 8) |
                 (((NbParams + 1) & 0xFF00) << 16));
}

/**
 * @brief  DCS or Generic short/long write command
 * @param  NbParams: Number of parameters. It indicates the write command mode:
 *                 If inferior to 2, a long write command is performed else
 * short.
 * @param  pParams: Pointer to parameter values table.
 * @retval HAL status
 */
void DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t *pParams)
{
#define LCD_OTM8009A_ID ((uint32_t)0)

    if (NbrParams <= 1)
    {
#define DSI_DCS_SHORT_PKT_WRITE_P1 \
    ((uint32_t)0x00000015U)  // DCS short write, one parameter
        HAL_DSI_ShortWrite(LCD_OTM8009A_ID, DSI_DCS_SHORT_PKT_WRITE_P1,
                           pParams[0], pParams[1]);
    }
    else
    {
#define DSI_DCS_LONG_PKT_WRITE ((uint32_t)0x00000039U)  // DCS long write
        HAL_DSI_LongWrite(LCD_OTM8009A_ID, DSI_DCS_LONG_PKT_WRITE, NbrParams,
                          pParams[NbrParams], pParams);
    }
}

// Step 1
void bspSetup()
{
    // All GPIOs ports already enabled by the bsp

    // LCD GPIOs configuration
    dsiTe.mode(Mode::ALTERNATE);  // TODO: Is TE necessary?
    dsiTe.alternateFunction(13);
    dsiReset.mode(Mode::OUTPUT_PULL_UP);
    dsiReset.speed(Speed::_100MHz);

    // Toggle Hardware Reset of the DSI LCD using its XRES signal
    dsiReset.low();  // Active low
    delayUs(20);     // 10us minimum
    dsiReset.high();
    delayMs(10);  // 5ms minumum

    // Enable the LTDC clock
    RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;
    RCC_SYNC();

    // Toggle soft reset of LTDC IP
    RCC->APB2RSTR |= RCC_APB2RSTR_LTDCRST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_LTDCRST;

    // // Enable the DMA2D clock
    // RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;
    //     RCC_SYNC();

    // // Toggle soft reset of DMA2D IP
    // RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA2DRST;
    // RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA2DRST;

    // Enable DSI Host and wrapper clocks
    RCC->APB2ENR |= RCC_APB2ENR_DSIEN;
    RCC_SYNC();

    // Toggle soft reset the DSI Host and wrapper
    RCC->APB2RSTR |= RCC_APB2RSTR_DSIRST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_DSIRST;

    // // NVIC configuration for LTDC interrupt that is now enabled
    // NVIC_SetPriority(LTDC_IRQn, 3);
    // NVIC_EnableIRQ(LTDC_IRQn);

    // // NVIC configuration for DMA2D interrupt that is now enabled
    // NVIC_SetPriority(DMA2D_IRQn, 3);
    // NVIC_EnableIRQ(DMA2D_IRQn);

    // // NVIC configuration for DSI interrupt that is now enabled
    // NVIC_SetPriority(DSI_IRQn, 3);
    // NVIC_EnableIRQ(DSI_IRQn);
}

// Step 2
void dsiSetup()
{
    // Turn on the regulator
    {
        // Enable the regulator
        DSI->WRPCR |= DSI_WRPCR_REGEN;

        // Wait until the regulator is ready
        while (!(DSI->WISR & DSI_WISR_RRS))
            ;
    }

    // Set the DSI regulator and PLL parameters
    {
        /**
         * DSI input clock is HSE at 25MHz
         *
         * F_VCO = CLK_IN / IDF * 2 * NDIV
         *       = 25MHz / 5 * 2 * 100
         *       = 1GHz (between 500MHz and 1GHz)
         *
         * HS_CLK = F_vco / 2 / ODF
         *        = 1GHz / 2 / 1
         *        = 500MHz
         *
         * HS_CLK is the lane frequency in high speed mode
         *
         * LANE_BYTE_CLK = PHI / 8
         *               = 62.5MHz
         *
         * LANE_BYTE_CLK must be between 31.25 MHz and 82.5 MHz
         *
         * LANE_BYTE_CLK can also use PLLR as clock source. DSI_CFGR2_DSISEL is
         * used to select the input clock.
         *
         * Then we have:
         * TX_ESC_CLK = LANE_BYTE_CLK / TXECKDIV
         * TO_CLK = LANE_BYTE_CLK / TOCKDIV
         * DDR_CLK = HS_CLK / 2
         *
         * The TX escape clock is used in Low Power mode. It takes two cycles of
         * TX_ESC_CLK to transmit one bit in LP mode.
         *
         * TX_ESC_CLK must be less that 20MHz and TXECKDIV must be >= 2. A value
         * of 0 or 1 disables TX_ESC_CLK.
         */

        // Set the PLL division factors
        DSI->WRPCR |= 100 << DSI_WRPCR_PLL_NDIV_Pos;  // NDIV = 100
        DSI->WRPCR |= 5 << DSI_WRPCR_PLL_IDF_Pos;     // IDF = 5
        DSI->WRPCR |= 0 << DSI_WRPCR_PLL_ODF_Pos;     // ODF = 1

        // Enable the DSI PLL
        DSI->WRPCR |= DSI_WRPCR_PLLEN;

        // Wait for the lock of the PLL
        while (!(DSI->WISR & DSI_WISR_PLLLS))
            ;
    }

    // Set the PHY parameters
    {
        // Enable D-PHY clock and data lane
        DSI->PCTLR |= DSI_PCTLR_CKE;
        DSI->PCTLR |= DSI_PCTLR_DEN;

        // Set clock lane in high speed mode
        DSI->CLCR |= DSI_CLCR_DPCC;

        // Configure the number of active data lanes
        DSI->PCONFR |= DSI_PCONFR_NL0;  // Two lanes
    }

    // Set the DSI clock parameters
    {
        // Set the TX escape clock division factor
        // TX_ESC_CLK = LANE_BYTE_CLK / 4 = 15.625MHz
        // 4 is the highest possible division factor such that TX_ESC_CLK is
        // less than 20MHz
        DSI->CCR = 4;

        /**
         * Calculate the bit period in high-speed mode in unit of 0.25ns
         *
         * 500MHz -> 2ns bit period -> 2ns = 8 * 0.25ns
         *
         * UIX4 = IntegerPart((1000 / F_PHY_Mhz) * 4)
         *      = IntegerPart((1000 / 500) * 4)
         *      = IntegerPart(2 * 4)
         *      = 8
         */
        DSI->WPCR[0] = 8;
    }

    // Configure DSI Video mode timings
    {
        // Select video mode by resetting CMDM and DSIM bits
        // Default values

        // Configure burst mode
        DSI->VMCR |= DSI_VMCR_VMT1;

        // Configure the video packet size
        DSI->VPCR |= HACT;  // Packet size is the horizontal size

        // The number of chunks is 0

        // Set the size of the null packet
        DSI->VNPCR |= 0xfff;

        // The virtual channel id for the LTDC interface is 0

        // The polarity of control signals is active high (default)

        // Select 24bit color coding for the host
        DSI->LCOLCR |= 0b0101 << DSI_LCOLCR_COLC_Pos;

        // Select the color coding for the wrapper
        DSI->WCFGR |= 0b101 << DSI_WCFGR_COLMUX_Pos;

        // Set the Horizontal Synchronization time in lane byte clock cycles
        DSI->VHSACR |= HSYNC * 62500 / 27429;

        // Set the Horizontal Back Porch in lane byte clock cycles
        DSI->VHBPCR |= HBP * 62500 / 27429;

        // Set the total line time in lane byte clock cycles
        DSI->VLCR |= (HSYNC + HBP + HACT + HFP) * 62500 / 27429;

        // Set the Vertical Synchronization time
        DSI->VVSACR |= VSYNC;

        // Set the Vertical Back Porch
        DSI->VVBPCR |= VBP;

        // Set the Vertical Active period
        DSI->VVACR |= VACT;

        // Set the Vertical Front Porch
        DSI->VVFPCR |= VFP;

        // Enable sending commands in Low Power mode
        DSI->VMCR |= DSI_VMCR_LPCE;

        // Low power largest packet size
        // Largest packet size possible to transmit in LP mode in VSA, VBP,
        // VFP regions. Only useful when sending LP packets is allowed while
        // streaming is active in video mode
        DSI->LPMCR |= 64 << DSI_LPMCR_LPSIZE_Pos;

        // Low power VACT largest packet size
        // Largest packet size possible to transmit in LP mode in HFP region
        // during VACT period. Only useful when sending LP packets is
        // allowed while streaming is active in video mode
        DSI->LPMCR |= 64;

        // Enable LP transition in HFP period
        DSI->VMCR |= DSI_VMCR_LPHFPE;

        // Enable LP transition in HBP period
        DSI->VMCR |= DSI_VMCR_LPHBPE;

        // Enable LP transition in VACT period
        DSI->VMCR |= DSI_VMCR_LPVAE;

        // Enable LP transition in VFP period
        DSI->VMCR |= DSI_VMCR_LPVFPE;

        // Enable LP transition in VBP period
        DSI->VMCR |= DSI_VMCR_LPVFPE;

        // Enable LP transition in vertical sync period
        DSI->VMCR |= DSI_VMCR_LPVSAE;

        // Disable the request for an acknowledge response at the end of a
        // frame (default)
    }

    // Enable the DSI host and wrapper: but LTDC is not started yet
    {
        // Enable the DSI host
        DSI->CR |= DSI_CR_EN;

        // Enable the DSI wrapper
        DSI->WCR |= DSI_WCR_DSIEN;
    }
}

// Step 3
void pllsaiSetup()
{
    /**
     * The LCD-TFT controller peripheral uses 3 clock domains:
     * - AHB clock domain (HCLK): For data transfer from the memories to the
     * Layer FIFO and the frame buffer
     * - APB2 clock domain (PCLK2): Configuration registers
     * - Pixel clock domain (LCD_CLK): This domain contains the pixel data
     * generation, the layer configuration register as well as the LCD-TFT
     * interface signal generator.
     *
     * Theoretically:
     * LCD_CLK = width x height x refresh rate
     *         = 800 x 480 x 60
     *         = 23.04MHz
     *
     * The LTDC clock is connected to the PLLLSAIR clock
     *
     * LCD_CLK = 1MHz * N / R / DIV
     *
     * Where:
     *   N = 384
     *   R = 6
     *   DIV = 2
     *
     * LCD_CLK = 1MHz * 384 / 7 / 2 = 31.25
     */

    // Disable PLLSAI Clock
    RCC->CR &= ~RCC_CR_PLLSAION;

    // Wait till PLLSAI is disabled
    while (RCC->CR & RCC_CR_PLLSAIRDY)
        ;

    // Note that this registers are set at 0 after reset
    RCC->PLLSAICFGR |= 384 << RCC_PLLSAICFGR_PLLSAIN_Pos;
    RCC->PLLSAICFGR |= 7 << RCC_PLLSAICFGR_PLLSAIR_Pos;
    RCC->DCKCFGR1 |= 0 << RCC_DCKCFGR1_PLLSAIDIVR_Pos;

    // Enable PLLSAI Clock
    RCC->CR |= RCC_CR_PLLSAION;

    // Wait till PLLSAI is ready
    while (!(RCC->CR & RCC_CR_PLLSAIRDY))
        ;
}

// Step 4
void ltdcSetup()
{
    // Initialize the LTDC
    {
        // The polarity for all control signals is active low (default)

        // Set Synchronization size
        LTDC->SSCR = (HSYNC - 1) << LTDC_SSCR_HSW_Pos;
        LTDC->SSCR |= (VSYNC - 1) << LTDC_SSCR_VSH_Pos;

        // Set Accumulated Back porch
        LTDC->BPCR = (HSYNC + HBP - 1) << LTDC_BPCR_AHBP_Pos;
        LTDC->BPCR |= (VSYNC + VBP - 1) << LTDC_BPCR_AVBP_Pos;

        // Set Accumulated Active Width
        LTDC->AWCR = (HSYNC + HBP + HACT - 1) << LTDC_AWCR_AAW_Pos;
        LTDC->AWCR |= (VSYNC + VBP + VACT - 1) << LTDC_AWCR_AAH_Pos;

        // Set Total Width
        LTDC->TWCR = (HSYNC + HBP + HACT + HFP - 1) << LTDC_TWCR_TOTALW_Pos;
        LTDC->TWCR |= (VSYNC + VBP + VACT + VFP - 1) << LTDC_TWCR_TOTALH_Pos;

        // Set the background color value
        LTDC->BCCR = 0 << LTDC_BCCR_BCRED_Pos;
        LTDC->BCCR |= 255 << LTDC_BCCR_BCGREEN_Pos;
        LTDC->BCCR |= 0 << LTDC_BCCR_BCBLUE_Pos;
    }

    // Layer setup
    {
        // Configure the horizontal start and stop position
        LTDC_Layer1->WHPCR = (HSYNC + HBP + HACT - 1)
                             << LTDC_LxWHPCR_WHSPPOS_Pos;
        LTDC_Layer1->WHPCR |= (HSYNC + HBP) << LTDC_LxWHPCR_WHSTPOS_Pos;

        // Configure the vertical start and stop position
        LTDC_Layer1->WVPCR = (VSYNC + VBP + VACT - 1)
                             << LTDC_LxWVPCR_WVSPPOS_Pos;
        LTDC_Layer1->WVPCR |= (VSYNC + VBP) << LTDC_LxWVPCR_WVSTPOS_Pos;

        // Specify the pixel format
        LTDC_Layer1->PFCR = 0x1;  // 1 for RGB888 (24bit)

        // Configures the color frame buffer start address
        // The frame buffer will be in the SDRAM and is 1152000B = 1.1MB
        LTDC_Layer1->CFBAR = 0xC0000000;

        // Configures the color frame buffer pitch
        // The pitch is the increment in bytes to jump to the next line
        LTDC_Layer1->CFBLR |= (HACT * 3) << LTDC_LxCFBLR_CFBP_Pos;
        LTDC_Layer1->CFBLR |= (HACT * 3 + 3) << LTDC_LxCFBLR_CFBLL_Pos;

        // Configures the frame buffer line number
        LTDC_Layer1->CFBLNR = VACT;

        // Configure the background color
        LTDC_Layer1->DCCR = 255 << LTDC_LxDCCR_DCBLUE_Pos;  // Black

        // Specifies the constant alpha value
        LTDC_Layer1->CACR = 255;

        // Specifies the blending factors
        // Blanded color = const alpha x current color + cont alpha x back color
        LTDC_Layer1->BFCR |= 4 << LTDC_LxBFCR_BF1_Pos;
        LTDC_Layer1->BFCR |= 5 << LTDC_LxBFCR_BF1_Pos;

        // Enable the layer
        LTDC_Layer1->CR |= LTDC_LxCR_LEN;

        // Reload shadow registers
        LTDC->SRCR = LTDC_SRCR_IMR;
    }

    // Finally enable the display
    LTDC->GCR |= LTDC_GCR_LTDCEN;
}

void otm8009aSetup()
{
/* List of OTM8009A used commands                                  */
/* Detailed in OTM8009A Data Sheet 'DATA_SHEET_OTM8009A_V0 92.pdf' */
/* Version of 14 June 2012                                         */
#define OTM8009A_CMD_NOP 0x00     /* NOP command      */
#define OTM8009A_CMD_SWRESET 0x01 /* Sw reset command */
#define OTM8009A_CMD_RDDMADCTL \
    0x0B /* Read Display MADCTR command : read memory display access ctrl */
#define OTM8009A_CMD_RDDCOLMOD 0x0C /* Read Display pixel format */
#define OTM8009A_CMD_SLPIN 0x10     /* Sleep In command */
#define OTM8009A_CMD_SLPOUT 0x11    /* Sleep Out command */
#define OTM8009A_CMD_PTLON 0x12     /* Partial mode On command */

#define OTM8009A_CMD_DISPOFF 0x28 /* Display Off command */
#define OTM8009A_CMD_DISPON 0x29  /* Display On command */

#define OTM8009A_CMD_CASET 0x2A /* Column address set command */
#define OTM8009A_CMD_PASET 0x2B /* Page address set command */

#define OTM8009A_CMD_RAMWR 0x2C /* Memory (GRAM) write command */
#define OTM8009A_CMD_RAMRD 0x2E /* Memory (GRAM) read command  */

#define OTM8009A_CMD_PLTAR 0x30 /* Partial area command (4 parameters) */

#define OTM8009A_CMD_TEOFF \
    0x34 /* Tearing Effect Line Off command : command with no parameter */

#define OTM8009A_CMD_TEEON                                                    \
    0x35 /* Tearing Effect Line On command : command with 1 parameter 'TELOM' \
          */

/* Parameter TELOM : Tearing Effect Line Output Mode : possible values */
#define OTM8009A_TEEON_TELOM_VBLANKING_INFO_ONLY 0x00
#define OTM8009A_TEEON_TELOM_VBLANKING_AND_HBLANKING_INFO 0x01

#define OTM8009A_CMD_MADCTR 0x36 /* Memory Access write control command  */

/* Possible used values of MADCTR */
#define OTM8009A_MADCTR_MODE_PORTRAIT 0x00
#define OTM8009A_MADCTR_MODE_LANDSCAPE \
    0x60 /* MY = 0, MX = 1, MV = 1, ML = 0, RGB = 0 */

#define OTM8009A_CMD_IDMOFF 0x38 /* Idle mode Off command */
#define OTM8009A_CMD_IDMON 0x39  /* Idle mode On command  */

#define OTM8009A_CMD_COLMOD 0x3A /* Interface Pixel format command */

/* Possible values of COLMOD parameter corresponding to used pixel formats */
#define OTM8009A_COLMOD_RGB565 0x55
#define OTM8009A_COLMOD_RGB888 0x77

#define OTM8009A_CMD_RAMWRC 0x3C /* Memory write continue command */
#define OTM8009A_CMD_RAMRDC 0x3E /* Memory read continue command  */

#define OTM8009A_CMD_WRTESCN 0x44 /* Write Tearing Effect Scan line command */
#define OTM8009A_CMD_RDSCNL 0x45  /* Read  Tearing Effect Scan line command */

/* CABC Management : ie : Content Adaptive Back light Control in IC OTM8009a */
#define OTM8009A_CMD_WRDISBV \
    0x51 /* Write Display Brightness command          */
#define OTM8009A_CMD_WRCTRLD \
    0x53 /* Write CTRL Display command                */
#define OTM8009A_CMD_WRCABC                           \
    0x55 /* Write Content Adaptive Brightness command \
          */
#define OTM8009A_CMD_WRCABCMB \
    0x5E /* Write CABC Minimum Brightness command     */

    const uint8_t ShortRegData1[]  = {OTM8009A_CMD_NOP, 0x00};
    const uint8_t ShortRegData2[]  = {OTM8009A_CMD_NOP, 0x80};
    const uint8_t ShortRegData3[]  = {0xC4, 0x30};
    const uint8_t ShortRegData4[]  = {OTM8009A_CMD_NOP, 0x8A};
    const uint8_t ShortRegData5[]  = {0xC4, 0x40};
    const uint8_t ShortRegData6[]  = {OTM8009A_CMD_NOP, 0xB1};
    const uint8_t ShortRegData7[]  = {0xC5, 0xA9};
    const uint8_t ShortRegData8[]  = {OTM8009A_CMD_NOP, 0x91};
    const uint8_t ShortRegData9[]  = {0xC5, 0x34};
    const uint8_t ShortRegData10[] = {OTM8009A_CMD_NOP, 0xB4};
    const uint8_t ShortRegData11[] = {0xC0, 0x50};
    const uint8_t ShortRegData12[] = {0xD9, 0x4E};
    const uint8_t ShortRegData13[] = {OTM8009A_CMD_NOP, 0x81};
    const uint8_t ShortRegData14[] = {0xC1, 0x66};
    const uint8_t ShortRegData15[] = {OTM8009A_CMD_NOP, 0xA1};
    const uint8_t ShortRegData16[] = {0xC1, 0x08};
    const uint8_t ShortRegData17[] = {OTM8009A_CMD_NOP, 0x92};
    const uint8_t ShortRegData18[] = {0xC5, 0x01};
    const uint8_t ShortRegData19[] = {OTM8009A_CMD_NOP, 0x95};
    const uint8_t ShortRegData20[] = {OTM8009A_CMD_NOP, 0x94};
    const uint8_t ShortRegData21[] = {0xC5, 0x33};
    const uint8_t ShortRegData22[] = {OTM8009A_CMD_NOP, 0xA3};
    const uint8_t ShortRegData23[] = {0xC0, 0x1B};
    const uint8_t ShortRegData24[] = {OTM8009A_CMD_NOP, 0x82};
    const uint8_t ShortRegData25[] = {0xC5, 0x83};
    const uint8_t ShortRegData26[] = {0xC4, 0x83};
    const uint8_t ShortRegData27[] = {0xC1, 0x0E};
    const uint8_t ShortRegData28[] = {OTM8009A_CMD_NOP, 0xA6};
    const uint8_t ShortRegData29[] = {OTM8009A_CMD_NOP, 0xA0};
    const uint8_t ShortRegData30[] = {OTM8009A_CMD_NOP, 0xB0};
    const uint8_t ShortRegData31[] = {OTM8009A_CMD_NOP, 0xC0};
    const uint8_t ShortRegData32[] = {OTM8009A_CMD_NOP, 0xD0};
    const uint8_t ShortRegData33[] = {OTM8009A_CMD_NOP, 0x90};
    const uint8_t ShortRegData34[] = {OTM8009A_CMD_NOP, 0xE0};
    const uint8_t ShortRegData35[] = {OTM8009A_CMD_NOP, 0xF0};
    const uint8_t ShortRegData36[] = {OTM8009A_CMD_SLPOUT, 0x00};
    const uint8_t ShortRegData37[] = {OTM8009A_CMD_COLMOD,
                                      OTM8009A_COLMOD_RGB565};
    const uint8_t ShortRegData38[] = {OTM8009A_CMD_COLMOD,
                                      OTM8009A_COLMOD_RGB888};
    const uint8_t ShortRegData39[] = {OTM8009A_CMD_MADCTR,
                                      OTM8009A_MADCTR_MODE_LANDSCAPE};
    const uint8_t ShortRegData40[] = {OTM8009A_CMD_WRDISBV, 0x7F};
    const uint8_t ShortRegData41[] = {OTM8009A_CMD_WRCTRLD, 0x2C};
    const uint8_t ShortRegData42[] = {OTM8009A_CMD_WRCABC, 0x02};
    const uint8_t ShortRegData43[] = {OTM8009A_CMD_WRCABCMB, 0xFF};
    const uint8_t ShortRegData44[] = {OTM8009A_CMD_DISPON, 0x00};
    const uint8_t ShortRegData45[] = {OTM8009A_CMD_RAMWR, 0x00};
    const uint8_t ShortRegData46[] = {0xCF, 0x00};
    const uint8_t ShortRegData47[] = {0xC5, 0x66};
    const uint8_t ShortRegData48[] = {OTM8009A_CMD_NOP, 0xB6};
    const uint8_t ShortRegData49[] = {0xF5, 0x06};
    const uint8_t ShortRegData50[] = {OTM8009A_CMD_NOP, 0xB1};
    const uint8_t ShortRegData51[] = {0xC6, 0x06};

    /*
     * @brief Constant tables of register settings used to transmit DSI
     * command packets as power up initialization sequence of the KoD LCD
     * (OTM8009A LCD Driver)
     */
    const uint8_t lcdRegData1[]  = {0x80, 0x09, 0x01, 0xFF};
    const uint8_t lcdRegData2[]  = {0x80, 0x09, 0xFF};
    const uint8_t lcdRegData3[]  = {0x00, 0x09, 0x0F, 0x0E, 0x07, 0x10,
                                    0x0B, 0x0A, 0x04, 0x07, 0x0B, 0x08,
                                    0x0F, 0x10, 0x0A, 0x01, 0xE1};
    const uint8_t lcdRegData4[]  = {0x00, 0x09, 0x0F, 0x0E, 0x07, 0x10,
                                    0x0B, 0x0A, 0x04, 0x07, 0x0B, 0x08,
                                    0x0F, 0x10, 0x0A, 0x01, 0xE2};
    const uint8_t lcdRegData5[]  = {0x79, 0x79, 0xD8};
    const uint8_t lcdRegData6[]  = {0x00, 0x01, 0xB3};
    const uint8_t lcdRegData7[]  = {0x85, 0x01, 0x00, 0x84, 0x01, 0x00, 0xCE};
    const uint8_t lcdRegData8[]  = {0x18, 0x04, 0x03, 0x39, 0x00,
                                    0x00, 0x00, 0x18, 0x03, 0x03,
                                    0x3A, 0x00, 0x00, 0x00, 0xCE};
    const uint8_t lcdRegData9[]  = {0x18, 0x02, 0x03, 0x3B, 0x00,
                                    0x00, 0x00, 0x18, 0x01, 0x03,
                                    0x3C, 0x00, 0x00, 0x00, 0xCE};
    const uint8_t lcdRegData10[] = {0x01, 0x01, 0x20, 0x20, 0x00, 0x00,
                                    0x01, 0x02, 0x00, 0x00, 0xCF};
    const uint8_t lcdRegData11[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0xCB};
    const uint8_t lcdRegData12[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0xCB};
    const uint8_t lcdRegData13[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0xCB};
    const uint8_t lcdRegData14[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0xCB};
    const uint8_t lcdRegData15[] = {0x00, 0x04, 0x04, 0x04, 0x04, 0x04,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0xCB};
    const uint8_t lcdRegData16[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x04, 0x04, 0x04, 0x04, 0x04, 0x00,
                                    0x00, 0x00, 0x00, 0xCB};
    const uint8_t lcdRegData17[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0xCB};
    const uint8_t lcdRegData18[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xCB};
    const uint8_t lcdRegData19[] = {0x00, 0x26, 0x09, 0x0B, 0x01, 0x25,
                                    0x00, 0x00, 0x00, 0x00, 0xCC};
    const uint8_t lcdRegData20[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x26,
                                    0x0A, 0x0C, 0x02, 0xCC};
    const uint8_t lcdRegData21[] = {0x25, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0xCC};
    const uint8_t lcdRegData22[] = {0x00, 0x25, 0x0C, 0x0A, 0x02, 0x26,
                                    0x00, 0x00, 0x00, 0x00, 0xCC};
    const uint8_t lcdRegData23[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x25,
                                    0x0B, 0x09, 0x01, 0xCC};
    const uint8_t lcdRegData24[] = {0x26, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0xCC};
    const uint8_t lcdRegData25[] = {0xFF, 0xFF, 0xFF, 0xFF};
    /*
     * CASET value (Column Address Set) : X direction LCD GRAM boundaries
     * depending on LCD orientation mode and PASET value (Page Address Set) : Y
     * direction LCD GRAM boundaries depending on LCD orientation mode XS[15:0]
     * = 0x000 = 0, XE[15:0] = 0x31F = 799 for landscape mode : apply to CASET
     * YS[15:0] = 0x000 = 0, YE[15:0] = 0x31F = 799 for portrait mode : : apply
     * to PASET
     */
    const uint8_t lcdRegData27[] = {0x00, 0x00, 0x03, 0x1F, OTM8009A_CMD_CASET};
    /*
     * XS[15:0] = 0x000 = 0, XE[15:0] = 0x1DF = 479 for portrait mode : apply to
     * CASET YS[15:0] = 0x000 = 0, YE[15:0] = 0x1DF = 479 for landscape mode :
     * apply to PASET
     */
    const uint8_t lcdRegData28[] = {0x00, 0x00, 0x01, 0xDF, OTM8009A_CMD_PASET};

    // LCD Initialization
    int i = 1;
    {
        /* Enable CMD2 to access vendor specific commands */
        /* Enter in command 2 mode and set EXTC to enable address shift function
         * (0x00) */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData1);
        DSI_IO_WriteCmd(3, (uint8_t *)lcdRegData1);
        printf("%d\n", i++);

        /* Enter ORISE Command 2 */
        DSI_IO_WriteCmd(0,
                        (uint8_t *)ShortRegData2); /* Shift address to 0x80 */
        DSI_IO_WriteCmd(2, (uint8_t *)lcdRegData2);
        printf("%d\n", i++);

        /////////////////////////////////////////////////////////////////////
        /* SD_PCH_CTRL - 0xC480h - 129th parameter - Default 0x00          */
        /* Set SD_PT                                                       */
        /* -> Source output level during porch and non-display area to GND */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData2);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData3);
        printf("%d\n", i++);
        delayMs(10);
        /* Not documented */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData4);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData5);
        printf("%d\n", i++);
        delayMs(10);
        /////////////////////////////////////////////////////////////////////

        /* PWR_CTRL4 - 0xC4B0h - 178th parameter - Default 0xA8 */
        /* Set gvdd_en_test                                     */
        /* -> enable GVDD test mode !!!                         */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData6);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData7);
        printf("%d\n", i++);

        /* PWR_CTRL2 - 0xC590h - 146th parameter - Default 0x79      */
        /* Set pump 4 vgh voltage                                    */
        /* -> from 15.0v down to 13.0v                               */
        /* Set pump 5 vgh voltage                                    */
        /* -> from -12.0v downto -9.0v                               */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData8);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData9);
        printf("%d\n", i++);

        /* P_DRV_M - 0xC0B4h - 181th parameter - Default 0x00 */
        /* -> Column inversion                                */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData10);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData11);
        printf("%d\n", i++);

        /* VCOMDC - 0xD900h - 1st parameter - Default 0x39h */
        /* VCOM Voltage settings                            */
        /* -> from -1.0000v downto -1.2625v                 */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData1);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData12);
        printf("%d\n", i++);

        /* Oscillator adjustment for Idle/Normal mode (LPDT only) set to 65Hz
         * (default is 60Hz) */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData13);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData14);
        printf("%d\n", i++);

        /* Video mode internal */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData15);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData16);
        printf("%d\n", i++);

        /* PWR_CTRL2 - 0xC590h - 147h parameter - Default 0x00 */
        /* Set pump 4&5 x6                                     */
        /* -> ONLY VALID when PUMP4_EN_ASDM_HV = "0"           */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData17);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData18);
        printf("%d\n", i++);

        /* PWR_CTRL2 - 0xC590h - 150th parameter - Default 0x33h */
        /* Change pump4 clock ratio                              */
        /* -> from 1 line to 1/2 line                            */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData19);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData9);
        printf("%d\n", i++);

        /* GVDD/NGVDD settings */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData1);
        DSI_IO_WriteCmd(2, (uint8_t *)lcdRegData5);
        printf("%d\n", i++);

        /* PWR_CTRL2 - 0xC590h - 149th parameter - Default 0x33h */
        /* Rewrite the default value !                           */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData20);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData21);
        printf("%d\n", i++);

        /* Panel display timing Setting 3 */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData22);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData23);
        printf("%d\n", i++);

        /* Power control 1 */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData24);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData25);
        printf("%d\n", i++);

        /* Source driver precharge */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData13);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData26);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData15);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData27);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData28);
        DSI_IO_WriteCmd(2, (uint8_t *)lcdRegData6);
        printf("%d\n", i++);

        /* GOAVST */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData2);
        DSI_IO_WriteCmd(6, (uint8_t *)lcdRegData7);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData29);
        DSI_IO_WriteCmd(14, (uint8_t *)lcdRegData8);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData30);
        DSI_IO_WriteCmd(14, (uint8_t *)lcdRegData9);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData31);
        DSI_IO_WriteCmd(10, (uint8_t *)lcdRegData10);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData32);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData46);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData2);
        DSI_IO_WriteCmd(10, (uint8_t *)lcdRegData11);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData33);
        DSI_IO_WriteCmd(15, (uint8_t *)lcdRegData12);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData29);
        DSI_IO_WriteCmd(15, (uint8_t *)lcdRegData13);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData30);
        DSI_IO_WriteCmd(10, (uint8_t *)lcdRegData14);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData31);
        DSI_IO_WriteCmd(15, (uint8_t *)lcdRegData15);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData32);
        DSI_IO_WriteCmd(15, (uint8_t *)lcdRegData16);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData34);
        DSI_IO_WriteCmd(10, (uint8_t *)lcdRegData17);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData35);
        DSI_IO_WriteCmd(10, (uint8_t *)lcdRegData18);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData2);
        DSI_IO_WriteCmd(10, (uint8_t *)lcdRegData19);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData33);
        DSI_IO_WriteCmd(15, (uint8_t *)lcdRegData20);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData29);
        DSI_IO_WriteCmd(15, (uint8_t *)lcdRegData21);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData30);
        DSI_IO_WriteCmd(10, (uint8_t *)lcdRegData22);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData31);
        DSI_IO_WriteCmd(15, (uint8_t *)lcdRegData23);
        printf("%d\n", i++);

        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData32);
        DSI_IO_WriteCmd(15, (uint8_t *)lcdRegData24);
        printf("%d\n", i++);

        /////////////////////////////////////////////////////////////////////////////
        /* PWR_CTRL1 - 0xc580h - 130th parameter - default 0x00 */
        /* Pump 1 min and max DM                                */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData13);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData47);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData48);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData49);
        printf("%d\n", i++);
        /////////////////////////////////////////////////////////////////////////////

        /* CABC LEDPWM frequency adjusted to 19,5kHz */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData50);
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData51);
        printf("%d\n", i++);

        /* Exit CMD2 mode */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData1);
        DSI_IO_WriteCmd(3, (uint8_t *)lcdRegData25);
        printf("%d\n", i++);

        /***************************************************************************
         */
        /* Standard DCS Initialization TO KEEP CAN BE DONE IN HSDT */
        /***************************************************************************
         */

        /* NOP - goes back to DCS std command ? */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData1);
        printf("%d\n", i++);

        /* Gamma correction 2.2+ table (HSDT possible) */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData1);
        DSI_IO_WriteCmd(16, (uint8_t *)lcdRegData3);
        printf("%d\n", i++);

        /* Gamma correction 2.2- table (HSDT possible) */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData1);
        DSI_IO_WriteCmd(16, (uint8_t *)lcdRegData4);
        printf("%d\n", i++);

        /* Send Sleep Out command to display : no parameter */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData36);
        printf("%d\n", i++);

        /* Wait for sleep out exit */
        delayMs(120);

        /* Set Pixel color format to RGB888 */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData38);
        printf("%d\n", i++);

        /* Send command to configure display in landscape orientation mode. By
           default the orientation mode is portrait  */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData39);
        DSI_IO_WriteCmd(4, (uint8_t *)lcdRegData27);
        DSI_IO_WriteCmd(4, (uint8_t *)lcdRegData28);
        printf("%d\n", i++);

        /** CABC : Content Adaptive Backlight Control section start >> */
        /* Note : defaut is 0 (lowest Brightness), 0xFF is highest Brightness,
         * try 0x7F : intermediate value */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData40);
        printf("%d\n", i++);

        /* defaut is 0, try 0x2C - Brightness Control Block, Display Dimming &
         * BackLight on */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData41);
        printf("%d\n", i++);

        /* defaut is 0, try 0x02 - image Content based Adaptive Brightness
         * [Still Picture] */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData42);
        printf("%d\n", i++);

        /* defaut is 0 (lowest Brightness), 0xFF is highest Brightness */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData43);
        printf("%d\n", i++);

        /** CABC : Content Adaptive Backlight Control section end << */

        /* Send Command Display On */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData44);
        printf("%d\n", i++);

        /* NOP command */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData1);
        printf("%d\n", i++);

        /* Send Command GRAM memory write (no parameters) : this initiates frame
         * write via other DSI commands sent by */
        /* DSI host from LTDC incoming pixels in video mode */
        DSI_IO_WriteCmd(0, (uint8_t *)ShortRegData45);
        printf("%d\n", i++);
    }
}

int main()
{
    bspSetup();
    printf("1. BSP setup done\n");

    dsiSetup();
    printf("2. DSI setup done\n");

    pllsaiSetup();
    printf("3. PLLSAI setup done\n");

    ltdcSetup();
    printf("4. LTDC setup done\n");

    // otm8009aSetup();
    // printf("4. OTM8009A setup done\n");

    // // Disable command trasmission only in low power mode
    // DSI->CMCR &= ~(DSI_CMCR_GSW0TX | DSI_CMCR_GSW1TX | DSI_CMCR_GSW2TX |
    //                DSI_CMCR_GSR0TX | DSI_CMCR_GSR1TX | DSI_CMCR_GSR2TX |
    //                DSI_CMCR_GLWTX | DSI_CMCR_DSW0TX | DSI_CMCR_DSW1TX |
    //                DSI_CMCR_DSR0TX | DSI_CMCR_DLWTX | DSI_CMCR_MRDPS);

    // DSI->PCR &= ~(DSI_PCR_CRCRXE | DSI_PCR_ECCRXE | DSI_PCR_BTAE |
    //               DSI_PCR_ETRXE | DSI_PCR_ETTXE);
    // DSI->PCR |= DSI_PCR_BTAE;

    // // Enable the LTDC
    // LTDC->GCR |= LTDC_GCR_LTDCEN;

    // // Start the LTDC flow through the DSI wrapper (CR.LTDCEN = 1).
    // // In video mode, the data streaming starts as soon as the LTDC is
    // enabled.
    // // In adapted command mode, the frame buffer update is launched as soon
    // as
    // // the CR.LTDCEN bit is set.
    // DSI->CR |= DSI_CR_EN;

    // // Update the display
    // DSI->WCR |= DSI_WCR_LTDCEN;

    printf("LCD initialization completed!\n");

#define FRAME_BUFFER_START ((uint8_t volatile *)((uint32_t)0xC0000000))

    // int i = 0;
    // while (true)
    // {
    //     FRAME_BUFFER_START[i] = 255;
    //     printf("%d: %d\n", i, FRAME_BUFFER_START[i]);
    //     i++;
    // }

    while (true)
    {
        printf("Hi mom!\n");
        Thread::sleep(1000);
    }
}
