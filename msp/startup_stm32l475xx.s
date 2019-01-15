;********************** COPYRIGHT(c) 2017  STMicroelectronics ******************
;* File Name          : startup_stm32l475xx.s
;* Author             : MCD Application Team
;* Description        : STM32L475xx Ultra Low Power devices vector table for MDK-ARM toolchain.
;*                      ！STM32L475XX低功耗设备的MDK-ARM工具链向量表
;*                      This module performs:
;*                      - Set the initial SP ！设置初始化指针
;*                      - Set the initial PC == Reset_Handler ！设置初始化
;*                      - Set the vector table entries with the exceptions ISR address;!设置中断异常向量表的入口
;*                      - Branches to __main in the C library (which eventually !分别调用c库中的_main 最终指向用户定义的mian函数
;*                        calls main()).
;*                      After Reset the Cortex-M4 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;*                      M4内核在重启后会在线程模式下，优先级很高，会将栈区指向mian。
;*!当CPU复位时，处理器工作在线程模式（线程模式和handler模式相对），权限为特权级，栈指针设置为MSP（和PSP相对，
;*!注意：进入异常处理时，不管之前使用的是PSP还是MSP都是用MSP。MSP和PSP的选择使用主要在操作系统下有必要区分，在裸机情况下直接使用MSP即可）。
;*!当发生异常后，会根据向量表偏移寄存器找到向量表所在的位置。向量表偏移寄存器记录了两个信息：
;*!1.向量表是在code区还是在SRAM区，默认为code区。

;*!2.相对于code区或者SRAM区的地址是多少。复位后，CPU会根据启动模式的不同到不同的地方找向量表的入口
;*!（可能是从flash启动，也可能是从SRAM中启动，对于CM4来说FLASH地址和SRAM地址都被映射到0地址）。
;*!找到向量表的入口后，接下来CPU会做两件事：

;*!将偏移地址为0的地址的值读取出来赋值给MSP。
;*!将偏移地址为4的地址的值读取出来赋值给PC（这个值也就是Reset_Handler函数的地址，即复位后执行的第一个函数）。
;*!注意： CM4这里的设计和大多数单片机不同，大多数单片机在向量表的每一个表项中放置一条跳转指令。所以对于CM4内核的CPU来说，
;*!当发生异常时，CPU需要从对应的向量表表项中取出表项值赋值给PC寄存器。

;* <<< Use Configuration Wizard in Context Menu >>>
;*******************************************************************************
;*
;* Redistribution and use in source and binary forms, with or without modification,
;* 
;* are permitted provided that the following conditions are met:
;*   1. Redistributions of source code must retain the above copyright notice,
;*      this list of conditions and the following disclaimer.
;*   2. Redistributions in binary form must reproduce the above copyright notice,
;*      this list of conditions and the following disclaimer in the documentation
;*      and/or other materials provided with the distribution.
;*   3. Neither the name of STMicroelectronics nor the names of its contributors
;*      may be used to endorse or promote products derived from this software
;*      without specific prior written permission.
;*
;* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
;* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
;* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;*
;*******************************************************************************
;
; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
//!目录：STM32Cube_FW_L4_V1.13.0\STM32Cube_FW_L4_V1.13.0\Drivers\CMSIS\Device\ST
//!     \STM32L4xx\Source\Templates\arm
//!栈的大小400
Stack_Size      EQU     0x400;
//! 伪指令 AREA，表示伪指令是不可执行代码，一般起辅助作用，是为编译系统服务的，
//!编译时不会分配存储单元，只是按伪指令的功能定位程序数据或指令位置。
                AREA    STACK, NOINIT, READWRITE, ALIGN=3
//!分配内存
Stack_Mem       SPACE   Stack_Size
//!初始化指针
__initial_sp

; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
//!堆大小0x200
Heap_Size       EQU     0x200;
//! 伪指令 AREA，表示伪指令是不可执行代码，一般起辅助作用，是为编译系统服务的，
//!编译时不会分配存储单元，只是按伪指令的功能定位程序数据或指令位置。
                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
; //!堆的起始地址
__heap_base
//! 分配堆的内存空间
Heap_Mem        SPACE   Heap_Size
//! 堆的终止地址
__heap_limit
//! 当前文件堆栈需要按照8字节对齐
                PRESERVE8
//!表示后面指令兼容 THUMB 指令。THUBM 是ARM 以前的指令集，16bit，
//!现在 Cortex-M 系列的都使用 THUMB-2 指令集，THUMB-2 是32 位的，
//!兼容 16 位和 32 位的指令，是 THUMB 的超级。 
                THUMB
; Vector Table Mapped to Address 0 at Reset
; 重启后向量表的起始地址
//! 汇编新的只读数据段，
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors                  ;!代码段开始的声明，全局性
                EXPORT  __Vectors_End              ;!代码段的结束
                EXPORT  __Vectors_Size             ;!代码段的大小
;!向量表从 FLASH 的 0 地址开始放置，以 4 个字节为一个单位，地址 0 存放的是栈顶地址，
;!0X04 存放的是复位程序的地址，以此类推。从代码上看，向量表中存放的都是中断服务函数的函数名，
;!可我们知道 C 语言中的函数名就是一个地址。
__Vectors       DCD     __initial_sp               ; Top of Stack
                DCD     Reset_Handler              ; Reset Handler
                DCD     NMI_Handler                ; NMI Handler
                DCD     HardFault_Handler          ; Hard Fault Handler
                DCD     MemManage_Handler          ; MPU Fault Handler
                DCD     BusFault_Handler           ; Bus Fault Handler
                DCD     UsageFault_Handler         ; Usage Fault Handler
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     SVC_Handler                ; SVCall Handler
                DCD     DebugMon_Handler           ; Debug Monitor Handler
                DCD     0                          ; Reserved
                DCD     PendSV_Handler             ; PendSV Handler
                DCD     SysTick_Handler            ; SysTick Handler

                ; External Interrupts
                DCD     WWDG_IRQHandler                   ; Window WatchDog
                DCD     PVD_PVM_IRQHandler                ; PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection
                DCD     TAMP_STAMP_IRQHandler             ; Tamper and TimeStamps through the EXTI line
                DCD     RTC_WKUP_IRQHandler               ; RTC Wakeup through the EXTI line
                DCD     FLASH_IRQHandler                  ; FLASH
                DCD     RCC_IRQHandler                    ; RCC
                DCD     EXTI0_IRQHandler                  ; EXTI Line0
                DCD     EXTI1_IRQHandler                  ; EXTI Line1
                DCD     EXTI2_IRQHandler                  ; EXTI Line2
                DCD     EXTI3_IRQHandler                  ; EXTI Line3
                DCD     EXTI4_IRQHandler                  ; EXTI Line4
                DCD     DMA1_Channel1_IRQHandler          ; DMA1 Channel 1
                DCD     DMA1_Channel2_IRQHandler          ; DMA1 Channel 2
                DCD     DMA1_Channel3_IRQHandler          ; DMA1 Channel 3
                DCD     DMA1_Channel4_IRQHandler          ; DMA1 Channel 4
                DCD     DMA1_Channel5_IRQHandler          ; DMA1 Channel 5
                DCD     DMA1_Channel6_IRQHandler          ; DMA1 Channel 6
                DCD     DMA1_Channel7_IRQHandler          ; DMA1 Channel 7
                DCD     ADC1_2_IRQHandler                 ; ADC1, ADC2
                DCD     CAN1_TX_IRQHandler                ; CAN1 TX
                DCD     CAN1_RX0_IRQHandler               ; CAN1 RX0
                DCD     CAN1_RX1_IRQHandler               ; CAN1 RX1
                DCD     CAN1_SCE_IRQHandler               ; CAN1 SCE
                DCD     EXTI9_5_IRQHandler                ; External Line[9:5]s
                DCD     TIM1_BRK_TIM15_IRQHandler         ; TIM1 Break and TIM15
                DCD     TIM1_UP_TIM16_IRQHandler          ; TIM1 Update and TIM16
                DCD     TIM1_TRG_COM_TIM17_IRQHandler     ; TIM1 Trigger and Commutation and TIM17
                DCD     TIM1_CC_IRQHandler                ; TIM1 Capture Compare
                DCD     TIM2_IRQHandler                   ; TIM2
                DCD     TIM3_IRQHandler                   ; TIM3
                DCD     TIM4_IRQHandler                   ; TIM4
                DCD     I2C1_EV_IRQHandler                ; I2C1 Event
                DCD     I2C1_ER_IRQHandler                ; I2C1 Error
                DCD     I2C2_EV_IRQHandler                ; I2C2 Event
                DCD     I2C2_ER_IRQHandler                ; I2C2 Error
                DCD     SPI1_IRQHandler                   ; SPI1
                DCD     SPI2_IRQHandler                   ; SPI2
                DCD     USART1_IRQHandler                 ; USART1
                DCD     USART2_IRQHandler                 ; USART2
                DCD     USART3_IRQHandler                 ; USART3
                DCD     EXTI15_10_IRQHandler              ; External Line[15:10]
                DCD     RTC_Alarm_IRQHandler              ; RTC Alarm (A and B) through EXTI Line
                DCD     DFSDM1_FLT3_IRQHandler            ; DFSDM1 Filter 3 global Interrupt
                DCD     TIM8_BRK_IRQHandler               ; TIM8 Break Interrupt
                DCD     TIM8_UP_IRQHandler                ; TIM8 Update Interrupt
                DCD     TIM8_TRG_COM_IRQHandler           ; TIM8 Trigger and Commutation Interrupt
                DCD     TIM8_CC_IRQHandler                ; TIM8 Capture Compare Interrupt
                DCD     ADC3_IRQHandler                   ; ADC3 global  Interrupt
                DCD     FMC_IRQHandler                    ; FMC
                DCD     SDMMC1_IRQHandler                 ; SDMMC1
                DCD     TIM5_IRQHandler                   ; TIM5
                DCD     SPI3_IRQHandler                   ; SPI3
                DCD     UART4_IRQHandler                  ; UART4
                DCD     UART5_IRQHandler                  ; UART5
                DCD     TIM6_DAC_IRQHandler               ; TIM6 and DAC1&2 underrun errors
                DCD     TIM7_IRQHandler                   ; TIM7
                DCD     DMA2_Channel1_IRQHandler          ; DMA2 Channel 1
                DCD     DMA2_Channel2_IRQHandler          ; DMA2 Channel 2
                DCD     DMA2_Channel3_IRQHandler          ; DMA2 Channel 3
                DCD     DMA2_Channel4_IRQHandler          ; DMA2 Channel 4
                DCD     DMA2_Channel5_IRQHandler          ; DMA2 Channel 5
                DCD     DFSDM1_FLT0_IRQHandler            ; DFSDM1 Filter 0 global Interrupt
                DCD     DFSDM1_FLT1_IRQHandler            ; DFSDM1 Filter 1 global Interrupt
                DCD     DFSDM1_FLT2_IRQHandler            ; DFSDM1 Filter 2 global Interrupt
                DCD     COMP_IRQHandler                   ; COMP Interrupt
                DCD     LPTIM1_IRQHandler                 ; LP TIM1 interrupt
                DCD     LPTIM2_IRQHandler                 ; LP TIM2 interrupt
                DCD     OTG_FS_IRQHandler                 ; USB OTG FS
                DCD     DMA2_Channel6_IRQHandler          ; DMA2 Channel 6
                DCD     DMA2_Channel7_IRQHandler          ; DMA2 Channel 7
                DCD     LPUART1_IRQHandler                ; LP UART1 interrupt
                DCD     QUADSPI_IRQHandler                ; Quad SPI global interrupt
                DCD     I2C3_EV_IRQHandler                ; I2C3 event
                DCD     I2C3_ER_IRQHandler                ; I2C3 error
                DCD     SAI1_IRQHandler                   ; Serial Audio Interface 1 global interrupt
                DCD     SAI2_IRQHandler                   ; Serial Audio Interface 2 global interrupt
                DCD     SWPMI1_IRQHandler                 ; Serial Wire Interface 1 global interrupt
                DCD     TSC_IRQHandler                    ; Touch Sense Controller global interrupt
                DCD     0                                 ; Reserved                
                DCD     0                                 ; Reserved                
                DCD     RNG_IRQHandler                    ; RNG global interrupt
                DCD     FPU_IRQHandler                    ; FPU

__Vectors_End
;!相减为数据段的大小
__Vectors_Size  EQU  __Vectors_End - __Vectors
;! 定义一个名为.text,可读的代码段
                AREA    |.text|, CODE, READONLY

; Reset handler
;! PROC定义子程序
Reset_Handler    PROC
;!全局声明，可被外部使用
;!WEAK:弱定义，如果外部文件声明了一个标号，
;!则优先使用外部文件定义的标号，如果外部文件没有定义也不出错。
                 EXPORT  Reset_Handler             [WEAK]
;!IMPORT声明标号来自外部文件
        IMPORT  SystemInit
        IMPORT  __main
;!LDR 从存储器中加载到寄存器中
                 LDR     R0, =SystemInit
;!BLX跳转到由寄存器给出的地址，并根据寄存器的 LSE 确定处理器的状态，
;!还要把跳转前的下条指令地址保存到 LR
                 BLX     R0
                 LDR     R0, =__main
;!跳转到由寄存器/标号给出的地址，不用返回
                 BX      R0
;!与PROC形成一对，表示子程序的结束
                 ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
;! B：跳到一个“.”，表示无限循环。
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler          [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler          [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler           [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler           [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler             [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .
                ENDP

Default_Handler PROC

        EXPORT     WWDG_IRQHandler                   [WEAK]
        EXPORT     PVD_PVM_IRQHandler                [WEAK]
        EXPORT     TAMP_STAMP_IRQHandler             [WEAK]
        EXPORT     RTC_WKUP_IRQHandler               [WEAK]
        EXPORT     FLASH_IRQHandler                  [WEAK]
        EXPORT     RCC_IRQHandler                    [WEAK]
        EXPORT     EXTI0_IRQHandler                  [WEAK]
        EXPORT     EXTI1_IRQHandler                  [WEAK]
        EXPORT     EXTI2_IRQHandler                  [WEAK]
        EXPORT     EXTI3_IRQHandler                  [WEAK]
        EXPORT     EXTI4_IRQHandler                  [WEAK]
        EXPORT     DMA1_Channel1_IRQHandler          [WEAK]
        EXPORT     DMA1_Channel2_IRQHandler          [WEAK]
        EXPORT     DMA1_Channel3_IRQHandler          [WEAK]
        EXPORT     DMA1_Channel4_IRQHandler          [WEAK]
        EXPORT     DMA1_Channel5_IRQHandler          [WEAK]
        EXPORT     DMA1_Channel6_IRQHandler          [WEAK]
        EXPORT     DMA1_Channel7_IRQHandler          [WEAK]
        EXPORT     ADC1_2_IRQHandler                 [WEAK]
        EXPORT     CAN1_TX_IRQHandler                [WEAK]
        EXPORT     CAN1_RX0_IRQHandler               [WEAK]
        EXPORT     CAN1_RX1_IRQHandler               [WEAK]
        EXPORT     CAN1_SCE_IRQHandler               [WEAK]
        EXPORT     EXTI9_5_IRQHandler                [WEAK]
        EXPORT     TIM1_BRK_TIM15_IRQHandler         [WEAK]
        EXPORT     TIM1_UP_TIM16_IRQHandler          [WEAK]
        EXPORT     TIM1_TRG_COM_TIM17_IRQHandler     [WEAK]
        EXPORT     TIM1_CC_IRQHandler                [WEAK]
        EXPORT     TIM2_IRQHandler                   [WEAK]
        EXPORT     TIM3_IRQHandler                   [WEAK]
        EXPORT     TIM4_IRQHandler                   [WEAK]
        EXPORT     I2C1_EV_IRQHandler                [WEAK]
        EXPORT     I2C1_ER_IRQHandler                [WEAK]
        EXPORT     I2C2_EV_IRQHandler                [WEAK]
        EXPORT     I2C2_ER_IRQHandler                [WEAK]
        EXPORT     SPI1_IRQHandler                   [WEAK]
        EXPORT     SPI2_IRQHandler                   [WEAK]
        EXPORT     USART1_IRQHandler                 [WEAK]
        EXPORT     USART2_IRQHandler                 [WEAK]
        EXPORT     USART3_IRQHandler                 [WEAK]
        EXPORT     EXTI15_10_IRQHandler              [WEAK]
        EXPORT     RTC_Alarm_IRQHandler              [WEAK]
        EXPORT     DFSDM1_FLT3_IRQHandler            [WEAK]
        EXPORT     TIM8_BRK_IRQHandler               [WEAK]
        EXPORT     TIM8_UP_IRQHandler                [WEAK]
        EXPORT     TIM8_TRG_COM_IRQHandler           [WEAK]
        EXPORT     TIM8_CC_IRQHandler                [WEAK]
        EXPORT     ADC3_IRQHandler                   [WEAK]
        EXPORT     FMC_IRQHandler                    [WEAK]
        EXPORT     SDMMC1_IRQHandler                 [WEAK]
        EXPORT     TIM5_IRQHandler                   [WEAK]
        EXPORT     SPI3_IRQHandler                   [WEAK]
        EXPORT     UART4_IRQHandler                  [WEAK]
        EXPORT     UART5_IRQHandler                  [WEAK]
        EXPORT     TIM6_DAC_IRQHandler               [WEAK]
        EXPORT     TIM7_IRQHandler                   [WEAK]
        EXPORT     DMA2_Channel1_IRQHandler          [WEAK]
        EXPORT     DMA2_Channel2_IRQHandler          [WEAK]
        EXPORT     DMA2_Channel3_IRQHandler          [WEAK]
        EXPORT     DMA2_Channel4_IRQHandler          [WEAK]
        EXPORT     DMA2_Channel5_IRQHandler          [WEAK]
        EXPORT     DFSDM1_FLT0_IRQHandler            [WEAK]
        EXPORT     DFSDM1_FLT1_IRQHandler            [WEAK]
        EXPORT     DFSDM1_FLT2_IRQHandler            [WEAK]
        EXPORT     COMP_IRQHandler                   [WEAK]
        EXPORT     LPTIM1_IRQHandler                 [WEAK]
        EXPORT     LPTIM2_IRQHandler                 [WEAK]
        EXPORT     OTG_FS_IRQHandler                 [WEAK]
        EXPORT     DMA2_Channel6_IRQHandler          [WEAK]
        EXPORT     DMA2_Channel7_IRQHandler          [WEAK]
        EXPORT     LPUART1_IRQHandler                [WEAK]
        EXPORT     QUADSPI_IRQHandler                [WEAK]
        EXPORT     I2C3_EV_IRQHandler                [WEAK]
        EXPORT     I2C3_ER_IRQHandler                [WEAK]
        EXPORT     SAI1_IRQHandler                   [WEAK]
        EXPORT     SAI2_IRQHandler                   [WEAK]
        EXPORT     SWPMI1_IRQHandler                 [WEAK]
        EXPORT     TSC_IRQHandler                    [WEAK]
        EXPORT     RNG_IRQHandler                    [WEAK]
        EXPORT     FPU_IRQHandler                    [WEAK]

WWDG_IRQHandler
PVD_PVM_IRQHandler
TAMP_STAMP_IRQHandler
RTC_WKUP_IRQHandler
FLASH_IRQHandler
RCC_IRQHandler
EXTI0_IRQHandler
EXTI1_IRQHandler
EXTI2_IRQHandler
EXTI3_IRQHandler
EXTI4_IRQHandler
DMA1_Channel1_IRQHandler
DMA1_Channel2_IRQHandler
DMA1_Channel3_IRQHandler
DMA1_Channel4_IRQHandler
DMA1_Channel5_IRQHandler
DMA1_Channel6_IRQHandler
DMA1_Channel7_IRQHandler
ADC1_2_IRQHandler
CAN1_TX_IRQHandler
CAN1_RX0_IRQHandler
CAN1_RX1_IRQHandler
CAN1_SCE_IRQHandler
EXTI9_5_IRQHandler
TIM1_BRK_TIM15_IRQHandler
TIM1_UP_TIM16_IRQHandler
TIM1_TRG_COM_TIM17_IRQHandler
TIM1_CC_IRQHandler
TIM2_IRQHandler
TIM3_IRQHandler
TIM4_IRQHandler
I2C1_EV_IRQHandler
I2C1_ER_IRQHandler
I2C2_EV_IRQHandler
I2C2_ER_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
USART1_IRQHandler
USART2_IRQHandler
USART3_IRQHandler
EXTI15_10_IRQHandler
RTC_Alarm_IRQHandler
DFSDM1_FLT3_IRQHandler
TIM8_BRK_IRQHandler
TIM8_UP_IRQHandler
TIM8_TRG_COM_IRQHandler
TIM8_CC_IRQHandler
ADC3_IRQHandler
FMC_IRQHandler
SDMMC1_IRQHandler
TIM5_IRQHandler
SPI3_IRQHandler
UART4_IRQHandler
UART5_IRQHandler
TIM6_DAC_IRQHandler
TIM7_IRQHandler
DMA2_Channel1_IRQHandler
DMA2_Channel2_IRQHandler
DMA2_Channel3_IRQHandler
DMA2_Channel4_IRQHandler
DMA2_Channel5_IRQHandler
DFSDM1_FLT0_IRQHandler
DFSDM1_FLT1_IRQHandler
DFSDM1_FLT2_IRQHandler
COMP_IRQHandler
LPTIM1_IRQHandler
LPTIM2_IRQHandler
OTG_FS_IRQHandler
DMA2_Channel6_IRQHandler
DMA2_Channel7_IRQHandler
LPUART1_IRQHandler
QUADSPI_IRQHandler
I2C3_EV_IRQHandler
I2C3_ER_IRQHandler
SAI1_IRQHandler
SAI2_IRQHandler
SWPMI1_IRQHandler
TSC_IRQHandler
RNG_IRQHandler
FPU_IRQHandler

                B       .

                ENDP
;! ALIGN：对指令或者数据存放的地址进行对齐，后面会跟一个立即数。缺省表示 4 字节对齐。
                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB//!使用微库，即keil中那个micro lib

                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit

                 ELSE;!不使用微库

                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap

__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                 ALIGN

                 ENDIF

                 END

;************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE*****
