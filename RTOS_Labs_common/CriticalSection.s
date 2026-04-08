     .text
     .thumb
     .align 2
     .global  StartCritical
     .global  EndCritical 

// *********** StartCritical ************************
//make a copy of previous I bit, disable interrupts
// inputs :  none
// outputs : R0=previous I bit
StartCritical:
        MRS    R0, PRIMASK  // save old status
        CPSID  I            // mask all (except faults)
        BX     LR

// *********** EndCritical ************************
// using the copy of previous I bit, restore I bit to previous value
// inputs :  R0=previous I bit
// outputs : none
EndCritical:
        MSR    PRIMASK, R0
        BX     LR
