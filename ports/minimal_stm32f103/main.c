#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "py/nlr.h"
#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "lib/utils/pyexec.h"

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
} periph_rcc_t;

typedef struct {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} periph_gpio_t;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
} periph_uart_t;

typedef struct {
    volatile uint32_t ACR;
    volatile uint32_t KEYR ;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
} periph_flash_t;

#define USART1 ((periph_uart_t*) 0x40013800)///Phenix modify 0x40011000
#define GPIOA  ((periph_gpio_t*) 0x40010800)///Phenix modify 0x40020000
#define GPIOB  ((periph_gpio_t*) 0x40010C00)///Phenix modify 0x40020400
#define RCC    ((periph_rcc_t*)  0x40021000)///Phenix modify 0x40023800
#define FLASH  ((periph_flash_t*)0x40022000)

// simple GPIO interface
#define GPIO_MODE_IN (0)
#define GPIO_MODE_OUT (1)
#define GPIO_MODE_ALT (2)
#define GPIO_PULL_NONE (0)
#define GPIO_PULL_UP (0)
#define GPIO_PULL_DOWN (1)
void gpio_init(periph_gpio_t *gpio, int pin, int mode, int pull, int alt) {
	if(pin <8)
	{
		gpio->CRL = (gpio->CRL & ~(3 << (4 * pin))) | (mode << (4 * pin));
		//if(mode == GPIO_MODE_OUT)
		{
			gpio->CRL = (gpio->CRL & ~(0xc << (4 * pin))) | (pull << (4 * pin));
		}
	}
	else
	{
		gpio->CRH = (gpio->CRH & ~(3 << (4 * (pin-8)))) | (mode << (4 * (pin-8)));
		//if(mode == GPIO_MODE_OUT)
		{
			gpio->CRH = (gpio->CRH & ~(0xc << (4 * (pin-8)))) | (pull << (4 * (pin-8)));
		}
	}
}
#define gpio_get(gpio, pin) ((gpio->IDR >> (pin)) & 1)
#define gpio_set(gpio, pin, value) do { gpio->ODR = (gpio->ODR & ~(1 << (pin))) | (value << pin); } while (0)
#define gpio_low(gpio, pin) do { gpio->BRR |= (1 << (pin)); } while (0)
#define gpio_high(gpio, pin) do { gpio->BSRR |= (1 << (pin)); } while (0)


#if MICROPY_ENABLE_COMPILER
void do_str(const char *src, mp_parse_input_kind_t input_kind) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_lexer_t *lex = mp_lexer_new_from_str_len(MP_QSTR__lt_stdin_gt_, src, strlen(src), 0);
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, input_kind);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, MP_EMIT_OPT_NONE, true);
        mp_call_function_0(module_fun);
        nlr_pop();
    } else {
        // uncaught exception
        mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
    }
}
#endif

static char *stack_top;
static char heap[2048];

int main(int argc, char **argv) {
    int stack_dummy;
    stack_top = (char*)&stack_dummy;
    #if MICROPY_ENABLE_GC
    gc_init(heap, heap + sizeof(heap));
    #endif

    mp_init();

    #if MICROPY_ENABLE_COMPILER
    #if MICROPY_REPL_EVENT_DRIVEN
    
    pyexec_event_repl_init();

    for (;;) {
        int c = mp_hal_stdin_rx_chr();
        if (pyexec_event_repl_process_char(c)) {
            break;
        }
    }
    #else
    pyexec_friendly_repl();
    #endif
    //do_str("print('hello world!', list(x+1 for x in range(10)), end='eol\\n')", MP_PARSE_SINGLE_INPUT);
    //do_str("for i in range(10):\r\n  print(i)", MP_PARSE_FILE_INPUT);
    #else
    pyexec_frozen_module("frozentest.py");
    #endif

	
    mp_deinit();
    return 0; 
}

void gc_collect(void) {
    // WARNING: This gc_collect implementation doesn't try to get root
    // pointers from CPU registers, and thus may function incorrectly.
    void *dummy;
    gc_collect_start();
    gc_collect_root(&dummy, ((mp_uint_t)stack_top - (mp_uint_t)&dummy) / sizeof(mp_uint_t));
    gc_collect_end();
    gc_dump_info();
}

mp_lexer_t *mp_lexer_new_from_file(const char *filename) {
    mp_raise_OSError(MP_ENOENT);
}

mp_import_stat_t mp_import_stat(const char *path) {
    return MP_IMPORT_STAT_NO_EXIST;
}

mp_obj_t mp_builtin_open(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_open_obj, 1, mp_builtin_open);

void nlr_jump_fail(void *val) {
    while (1);
}

void NORETURN __fatal_error(const char *msg) {
    while (1);
}

#ifndef NDEBUG
void MP_WEAK __assert_func(const char *file, int line, const char *func, const char *expr) {
    printf("Assertion '%s' failed, at file %s:%d\n", expr, file, line);
    __fatal_error("Assertion failed");
}
#endif

#if MICROPY_MIN_USE_CORTEX_CPU

// this is a minimal IRQ and reset framework for any Cortex-M CPU

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;

void Reset_Handler(void) __attribute__((naked));
void Reset_Handler(void) {
    // set stack pointer
    __asm volatile ("ldr sp, =_estack");
    // copy .data section from flash to RAM
    for (uint32_t *src = &_sidata, *dest = &_sdata; dest < &_edata;) {
        *dest++ = *src++;
    }
    // zero out .bss section
    for (uint32_t *dest = &_sbss; dest < &_ebss;) {
        *dest++ = 0;
    }
    // jump to board initialisation
    void _start(void);
    _start();
}

void Default_Handler_NMI(void) {
    for (;;) {
	//printf("calling Default_Handler\n");
    }
}
void Default_Handler_HardFault(void) {
    for (;;) {
	//printf("calling Default_Handler\n");
    }
}

void Default_Handler_MemManage(void) {
    for (;;) {
	//printf("calling Default_Handler\n");
    }
}

void Default_Handler_BusFault(void) {
    for (;;) {
	//printf("calling Default_Handler\n");
    }
}

void Default_Handler_UsageFault(void) {
    for (;;) {
	//printf("calling Default_Handler\n");
    }
}

void Default_Handler(void) {
    for (;;) {
	//printf("calling Default_Handler\n");
    }
}


const uint32_t isr_vector[] __attribute__((section(".isr_vector"))) = {
    (uint32_t)&_estack,
    (uint32_t)&Reset_Handler,
    (uint32_t)&Default_Handler_NMI, // NMI_Handler
    (uint32_t)&Default_Handler_HardFault, // HardFault_Handler
    (uint32_t)&Default_Handler_MemManage, // MemManage_Handler
    (uint32_t)&Default_Handler_BusFault, // BusFault_Handler
    (uint32_t)&Default_Handler_UsageFault, // UsageFault_Handler
    0,
    0,
    0,
    0,
    (uint32_t)&Default_Handler, // SVC_Handler
    (uint32_t)&Default_Handler, // DebugMon_Handler
    0,
    (uint32_t)&Default_Handler, // PendSV_Handler
    (uint32_t)&Default_Handler, // SysTick_Handler
};

void _start(void) {
    // when we get here: stack is initialised, bss is clear, data is copied

    // SCB->CCR: enable 8-byte stack alignment for IRQ handlers, in accord with EABI
    //*((volatile uint32_t*)0xe000ed14) |= 1 << 9;

    // initialise the cpu and peripherals
    #if MICROPY_MIN_USE_STM32_MCU
    void stm32_init(void);
    stm32_init();
    #endif

    // now that we have a basic system up and running we can call main
    main(0, NULL);

    // we must not return
    for (;;) {
    }
}

#endif

#if MICROPY_MIN_USE_STM32_MCU
void MYRCC_DeInit(void)
{ 
  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;
  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  RCC->CFGR &= (uint32_t)0xF8FF0000;
  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;
  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;
  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;
  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;
  
} 
void Stm32_Clock_Init(uint32_t PLL)
{
	unsigned char temp=0;    
    
	MYRCC_DeInit();
	
	RCC->CR|=0x00010000;  //设置外部高速晶振(HSE)  HSE晶振打开(ON)
	while(!(RCC->CR>>17));

	RCC->CFGR=0X00000400; //AHB时钟 = 系统时钟  APB2时钟 = HCLK   APB1时钟 = HCLK/2
	
	FLASH->ACR|=0x32; //设置FLASH存储器延时时钟周期数为 2延时周期,选择FLASH预取指缓存的模,预取指缓存使能

	//设置PLL时钟源及倍频系数  
	PLL-=2;
	RCC->CFGR|=PLL<<18;  //12M*6 = 72M
	RCC->CFGR|=1<<16;    //HSE时钟作为PLL输入时钟  = 12M

    RCC->CR|=0x01000000; ////使能PLL
	
	while(!(RCC->CR>>25));//检查指定的RCC标志位(PLL准备好标志)设置与否
	
	RCC->CFGR|=0x00000002;//设置系统时钟(SYSCLK)
	while(temp!=0x02)    //检查是否PLL作为系统时钟
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}    
}   

void stm32_init(void) {
	
	#if 0
    // basic MCU config
    RCC->CR |= (uint32_t)0x00000001; // set HSION
    RCC->CFGR = 0x00000000; // reset all
    RCC->CR &= (uint32_t)0xfef6ffff; // reset HSEON, CSSON, PLLON
    //RCC->PLLCFGR = 0x24003010; // reset PLLCFGR
    RCC->CR &= (uint32_t)0xfffbffff; // reset HSEBYP
    RCC->CIR = 0x00000000; // disable IRQs

    // leave the clock as-is (internal 16MHz)
	#else
	Stm32_Clock_Init(6);
	#endif

    // enable GPIO clocks
    RCC->APB2ENR |= 0x0000000C; // GPIOAEN, GPIOBEN

    // turn on an LED! (on pyboard it's the red one)
    gpio_init(GPIOB, 6, GPIO_MODE_OUT, GPIO_PULL_NONE, 0);
    gpio_low(GPIOB, 6);
    // enable UART1 at 9600 baud (TX=B6, RX=B7)
    //gpio_init(GPIOA, 9, GPIO_MODE_OUT, 2, 7);  //TX
    //gpio_init(GPIOA, 10, GPIO_MODE_IN, 2, 7); //RX
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置 PA9 PA10
    GPIOA->CRH|=0X000008B0;//IO状态设置
	
    RCC->APB2ENR |= (0x1<<14); // USART1EN
	
	RCC->APB2RSTR |= 1<<14;
	RCC->APB2RSTR &= ~(1<<14);
	
    USART1->BRR = (468 << 4) | 12; // 16MHz/(16*104.1875) = 9598 baud
    USART1->CR1 = 0x0000200c; // USART enable, tx enable, rx enable
}

#endif
