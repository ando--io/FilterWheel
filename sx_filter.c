/*
 Based on LUFA GenericHID sample
 */

#include "sx_filter.h"
#include "stepper.h"
#include "LEDS.h"

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevHIDReportBuffer[GENERIC_REPORT_SIZE];
static uint8_t mPendingResponseIndex;
static uint8_t mPendingResponse[GENERIC_REPORT_SIZE];
static bool mPendingResponseReady;
static uint8_t gReportID;

uint8_t gCommandBuffer[2];
uint8_t gCommandBufferReady;
uint8_t gCurrentFilter;
uint8_t gTargetFilter;
uint8_t gTransiantFilter;
uint8_t gMaxFilter = 5;
uint8_t gPreviousState;
unsigned gPreviousPos;
uint8_t gShouldBlink;

int magnetPin = 9;
unsigned gPos = 0;

void filter_task(void) ;
void gotoFilter( int filter );

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Generic_HID_Interface =
{
   .Config =
			{
            .InterfaceNumber              = INTERFACE_ID_GenericHID,
            .ReportINEndpoint             =
            {
               .Address              = GENERIC_IN_EPADDR,
               .Size                 = GENERIC_EPSIZE,
               .Banks                = 1,
            },
            .PrevReportINBuffer           = PrevHIDReportBuffer,
            .PrevReportINBufferSize       = sizeof(PrevHIDReportBuffer),
         },
};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
   SetupHardware();
   
   LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
   GlobalInterruptEnable();
   
   for (;;)
   {
      HID_Device_USBTask(&Generic_HID_Interface);
      USB_USBTask();
      stepper_task();
      filter_task();
      led_task();
   }
}

/** Configures the board hardware. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
   /* Disable watchdog if enabled by bootloader/fuses */
   MCUSR &= ~(1 << WDRF);
   wdt_disable();
   
   /* Disable clock division */
   clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
   /* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
   XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
   XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);
   
   /* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
   XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
   XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);
   
   PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif
   
   /* Hardware Initialization */
   LEDs_Init();
   USB_Init();
   stepper_init();
   led_init();
   
   gCurrentFilter = 0;
   gPreviousState = 0;
   gTransiantFilter = 0;
   gPreviousPos = 0;
   gPos = 0;
   gCommandBufferReady = false;
   gShouldBlink = 0;
   
   DDRC &= ~_BV(PC6);		// port C, pin 6 input
   DDRB &= ~_BV(PB4);		// port C, pin 6 input
   
   mPendingResponse[0] = 0;
   mPendingResponse[1] = 0;
   mPendingResponseIndex = 0;
   mPendingResponseReady = false;
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
   LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
   LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
   bool ConfigSuccess = true;
   
   ConfigSuccess &= HID_Device_ConfigureEndpoints(&Generic_HID_Interface);
   
   USB_Device_EnableSOFEvents();
   
   LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
   
   if (ConfigSuccess) {
      gotoFilter(1);
   }
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
   HID_Device_ProcessControlRequest(&Generic_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
   HID_Device_MillisecondElapsed(&Generic_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */

bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
   uint8_t* Data        = (uint8_t*)ReportData;
   
   if (mPendingResponseReady == true) {
      mPendingResponseReady = false;
      
      *ReportID = 0x00;
      
      Data[0] = mPendingResponse[0];
      Data[1] = mPendingResponse[1];
      
      memset(mPendingResponse, 0, sizeof(mPendingResponse));
      *ReportSize = 2;
   } else {
      *ReportSize = 0;
   }
   return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
   uint8_t* Data       = (uint8_t*)ReportData;
   gReportID = ReportID;
   // Reverse engineering may lead to strange situation.
   // ==> need to compose with software bug.
   //  AstroImager is sending a report without report ID. which them create a ReportSize = 1 and first byte is actually the reportId
   //  Nebulosity is sending a reportID so the ReportSize = 2 and the data are in the Date array....
   
   if (ReportSize == 1) {
      gCommandBuffer[0] = ReportID;
      gCommandBuffer[1] = *Data;
      gCommandBufferReady = true;
   } else
      if (ReportSize == 2) {
         gCommandBuffer[0] = *Data;
         gCommandBuffer[1] = *(++Data);
         gCommandBufferReady = true;
      }
}

void gotoFilter( int filter ) {
   if (filter > gMaxFilter) {
      filter = gMaxFilter;
   }
   
   if (gTransiantFilter == filter) {
      return;
   }
   
   gTargetFilter = filter;
   // during rotation, transiant filter is 0. it will be used to report current position to host
   gTransiantFilter = 0;
   stepper_enable_output();
   // move Counter Clockwise
   stepper_move(CCW);
}

void filter_task(void) {
   
   uint8_t val;
   
   // process the 3 starlight xpress documented commands
   if (gCommandBufferReady) {
      
      if (gCommandBuffer[0]==0 && gCommandBuffer[1]==0) {
         // Request current filter number
         mPendingResponse[0] = gTransiantFilter;
         mPendingResponse[1] = gMaxFilter;
         mPendingResponseReady = true;
         if (gCurrentFilter == 0) {
            gotoFilter(1);
         }
      } else if (gCommandBuffer[0]==0 && gCommandBuffer[1]==1) {
         // Get Filter Total
         mPendingResponse[0] = gMaxFilter;
         mPendingResponse[1] = 0;
         mPendingResponseReady = true;
      } else  if (gCommandBuffer[0]>0 && gCommandBuffer[1]==0) {
         // Select New filter
         gotoFilter(gCommandBuffer[0]);
         mPendingResponse[0] = gTransiantFilter;
         mPendingResponse[1] = gMaxFilter;
         mPendingResponseReady = true;
      }
      gCommandBufferReady = false;
   }
   
   val = PINC & _BV(PC6);
   
   if (val != gPreviousState) {
      
      gPreviousState = val;
      
      unsigned current = stepper_current_position();
      unsigned delta =  gPreviousPos - current;
      gPreviousPos = current;
      gPos <<= 1;
      if (delta < 30) {     // the hardware reports around 20 steps for the distance of the magnetic index.
         gPos |= 0x1;        // magnetic index are represented by 1, no index is a 0.
      }
      
      // then identifying the current position or the initial position is just a matter a comparing a bit scheme.
      if ((gPos & 0b111111) == 0b101011) {
         gCurrentFilter = 1;
      } else  if (((gPos & 0b11) == 2) && (gCurrentFilter > 0) && (gCurrentFilter < 5)) {
         gCurrentFilter  ++;
      }
      
      // if target pos is reached, stop motor a disable output to prevend current consumption or heat.
      if (gCurrentFilter == gTargetFilter) {
         gTransiantFilter = gCurrentFilter;
         stepper_disable_output();
         led_blink(gTransiantFilter);
      }
   }
}
