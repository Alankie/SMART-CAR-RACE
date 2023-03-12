; generated by Component: ARM Compiler 5.06 update 7 (build 960) Tool: ArmCC [4d365d]
; commandline ArmCC [--debug -c -S -o..\obj\core_cm3.s --depend=..\obj\core_cm3.d --cpu=Cortex-M3 --apcs=interwork -O0 --diag_suppress=9931 -I..\User -I..\MiniBalance_COER -I..\STM32F10x_FWLib\inc -I..\MiniBalance_HARDWARE -I..\SYSTEM -I..\SYSTEM\delay -I..\SYSTEM\sys -I..\SYSTEM\usart -I..\MiniBalance_HARDWARE\DMP -I..\MiniBalance_HARDWARE\IIC -I..\MiniBalance_HARDWARE\ENCODER -I..\MiniBalance_HARDWARE\OLED -I..\MiniBalance\show -I..\MiniBalance_HARDWARE\LED -I..\MiniBalance_HARDWARE\KEY -I..\MiniBalance_HARDWARE\ADC -I..\MiniBalance_HARDWARE\MOTOR -I..\MiniBalance_HARDWARE\TIMER -I..\MiniBalance_HARDWARE\EXTI -I..\MiniBalance\DataScope_DP -I..\MiniBalance\filter -I..\MiniBalance_HARDWARE\STMFLASH -I..\MiniBalance_HARDWARE\USARTX -I..\MiniBalance_HARDWARE\NRF24L01 -I..\MiniBalance_HARDWARE\SPI -I..\MiniBalance_HARDWARE\CAN -I..\MiniBalance_HARDWARE\PS2 -I..\MiniBalance_HARDWARE\MPU9250 -ID:\Arm\Packs\Keil\STM32F1xx_DFP\2.3.0\Device\Include -D__UVISION_VERSION=535 -DSTM32F10X_HD -DSTM32F10X_ -DHD -DUSE_STDPERIPH_DRIVER --omf_browse=..\obj\core_cm3.crf ..\MiniBalance_COER\core_cm3.c]
        THUMB
        PRESERVE8

        AREA ||.arm_vfe_header||, DATA, READONLY, NOALLOC, ALIGN=2

        DCD      0x00000000

;*** Start embedded assembler ***

#line 1 "..\\MiniBalance_COER\\core_cm3.c"
	AREA ||.emb_text||, CODE
	THUMB
	EXPORT |__get_PSP|
#line 58
|__get_PSP| PROC
#line 59

 mrs r0, psp
 bx lr
	ENDP
	AREA ||.emb_text||, CODE
	THUMB
	EXPORT |__set_PSP|
#line 72
|__set_PSP| PROC
#line 73

 msr psp, r0
 bx lr
	ENDP
	AREA ||.emb_text||, CODE
	THUMB
	EXPORT |__get_MSP|
#line 86
|__get_MSP| PROC
#line 87

 mrs r0, msp
 bx lr
	ENDP
	AREA ||.emb_text||, CODE
	THUMB
	EXPORT |__set_MSP|
#line 100
|__set_MSP| PROC
#line 101

 msr msp, r0
 bx lr
	ENDP
	AREA ||.emb_text||, CODE
	THUMB
	EXPORT |__REV16|
#line 114
|__REV16| PROC
#line 115

 rev16 r0, r0
 bx lr
	ENDP
	AREA ||.emb_text||, CODE
	THUMB
	EXPORT |__REVSH|
#line 128
|__REVSH| PROC
#line 129

 revsh r0, r0
 bx lr
	ENDP

;*** End   embedded assembler ***

        IMPORT ||Lib$$Request$$armlib|| [CODE,WEAK]

        ATTR FILESCOPE
        ATTR SETVALUE Tag_ABI_PCS_wchar_t,2
        ATTR SETVALUE Tag_ABI_enum_size,1
        ATTR SETVALUE Tag_ABI_optimization_goals,6
        ATTR SETSTRING Tag_conformance,"2.09"
        ATTR SETVALUE AV,18,1

        ASSERT {ENDIAN} = "little"
        ASSERT {INTER} = {TRUE}
        ASSERT {ROPI} = {FALSE}
        ASSERT {RWPI} = {FALSE}
        ASSERT {IEEE_FULL} = {FALSE}
        ASSERT {IEEE_PART} = {FALSE}
        ASSERT {IEEE_JAVA} = {FALSE}
        END
