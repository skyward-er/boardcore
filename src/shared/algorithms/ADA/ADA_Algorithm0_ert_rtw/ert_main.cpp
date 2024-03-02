//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ert_main.cpp
//
// Code generated for Simulink model 'ADA_Algorithm0'.
//
// Model version                  : 1.11
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Wed Nov  1 00:43:29 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Safety precaution
// Validation result: Passed (20), Warning (1), Error (0)
//
#include <stdio.h>  // This example main program uses printf/fflush

#include "ADA_Algorithm0.h"  // Model header file

static ADA_Algorithm0 ADA_Algorithm0_Obj;  // Instance of model class

//
// Associating rt_OneStep with a real-time clock or interrupt service routine
// is what makes the generated code "real-time".  The function rt_OneStep is
// always associated with the base rate of the model.  Subrates are managed
// by the base rate from inside the generated code.  Enabling/disabling
// interrupts and floating point context switches are target specific.  This
// example code indicates where these should take place relative to executing
// the generated code step function.  Overrun behavior should be tailored to
// your application needs.  This example simply sets an error status in the
// real-time model and returns from rt_OneStep.
//
void rt_OneStep(void);
void rt_OneStep(void)
{
    static bool OverrunFlag{false};

    // Disable interrupts here

    // Check for overrun
    if (OverrunFlag)
    {
        return;
    }

    OverrunFlag = true;

    // Save FPU context here (if necessary)
    // Re-enable timer or interrupt here
    // Set model inputs here

    // Step the model
    ADA_Algorithm0_Obj.step();

    // Get model outputs here

    // Indicate task complete
    OverrunFlag = false;

    // Disable interrupts here
    // Restore FPU context here (if necessary)
    // Enable interrupts here
}

//
// The example main function illustrates what is required by your
// application code to initialize, execute, and terminate the generated code.
// Attaching rt_OneStep to a real-time clock is target specific. This example
// illustrates how you do this relative to initializing the model.
//
int main(int argc, const char *argv[])
{
    // Unused arguments
    (void)(argc);
    (void)(argv);

    // Initialize model
    ADA_Algorithm0_Obj.initialize();

    // Attach rt_OneStep to a timer or interrupt service routine with
    //  period 0.02 seconds (base rate of the model) here.
    //  The call syntax for rt_OneStep is
    //
    //   rt_OneStep();

    printf(
        "Warning: The simulation will run forever. "
        "Generated ERT main won't simulate model step behavior. "
        "To change this behavior select the 'MAT-file logging' option.\n");
    fflush((nullptr));
    while (1)
    {
        //  Perform application tasks here
    }

    // The option 'Remove error status field in real-time model data structure'
    //  is selected, therefore the following code does not need to execute.

    // Terminate model
    ADA_Algorithm0_Obj.terminate();
    return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
