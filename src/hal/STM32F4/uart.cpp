#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "common/assert.h"

#include "hal/STM32F4/uart.hpp"
#include "hal/STM32F4/rcc.hpp"
#include "hal/STM32F4/dma.hpp"
#include "hal/STM32F4/gpio.hpp"

#include "hal/STM32F4/CMSIS/device-support/include/stm32f4xx.h"
#include "hal/STM32F4/CMSIS/core-support/core_cm4.h"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/semphr.h"

using namespace hal;

#define IRQ_PRIORITY 6
#define MAX_BRR_VAL 0xFFFF

static USART_TypeDef *const uart_list[UART_END] =
{
	USART1,
	USART2,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	USART3,
#else
	NULL,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	UART4,
	UART5,
#else
	NULL,
	NULL,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	USART6
#else
	NULL
#endif
};

static IRQn_Type const irq_list[UART_END] =
{
	USART1_IRQn,
	USART2_IRQn,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	USART3_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	UART4_IRQn,
	UART5_IRQn,
#else
	static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	USART6_IRQn
#else
	static_cast<IRQn_Type>(0)
#endif
};

static uint32_t const rcc_list[UART_END] =
{
	RCC_APB2ENR_USART1EN,
	RCC_APB1ENR_USART2EN,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1ENR_USART3EN,
#else
	0,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB1ENR_UART4EN,
	RCC_APB1ENR_UART5EN,
#else
	0,
	0,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB2ENR_USART6EN
#else
	0
#endif
};

static uint32_t const reset_list[UART_END] =
{
	RCC_APB2RSTR_USART1RST,
	RCC_APB1RSTR_USART2RST,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1RSTR_USART3RST,
#else
	0,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB1RSTR_UART4RST,
	RCC_APB1RSTR_UART5RST,
#else
	0,
	0,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB2RSTR_USART6RST
#else
	0
#endif
};

static volatile uint32_t *rcc_addr_list[UART_END] =
{
	&RCC->APB2ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB2ENR
};

static volatile uint32_t *reset_addr_list[UART_END] =
{
	&RCC->APB2RSTR,
	&RCC->APB1RSTR,
	&RCC->APB1RSTR,
	&RCC->APB1RSTR,
	&RCC->APB1RSTR,
	&RCC->APB2RSTR
};

static rcc_src_t const rcc_src_list[UART_END] =
{
	RCC_SRC_APB2,
	RCC_SRC_APB1,
	RCC_SRC_APB1,
	RCC_SRC_APB1,
	RCC_SRC_APB1,
	RCC_SRC_APB2
};

static uint8_t const gpio_af_list[UART_END] =
{
	0x07,
	0x07,
	0x07,
	0x08,
	0x08,
	0x08
};

static GPIO_TypeDef *const gpio_list[PORT_QTY] =
{
	GPIOA, GPIOB, GPIOC,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	GPIOD, GPIOE,
#else
	NULL, NULL,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	GPIOF, GPIOG,
#else
	NULL, NULL,
#endif
	GPIOH,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || \
	defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	GPIOI,
#else
	NULL,
#endif
#if defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	GPIOJ, GPIOK
#else
	NULL, NULL
#endif
};

static uart *obj_list[UART_END];

static void gpio_af_init(uart_t uart, gpio &gpio);

#if configUSE_TRACE_FACILITY
static traceHandle isr_dma_tx, isr_dma_rx, isr_uart;
#endif

uart::uart(uart_t uart, uint32_t baud, uart_stopbit_t stopbit,
	uart_parity_t parity, dma &dma_tx, dma &dma_rx, gpio &gpio_tx,
	gpio &gpio_rx):
	_uart(uart),
	_baud(baud),
	_stopbit(stopbit),
	_parity(parity),
	tx_dma(dma_tx),
	tx_gpio(gpio_tx),
	tx_api_lock(NULL),
	tx_irq_lock(NULL),
	tx_irq_res(UART_ERR_NONE),
	rx_dma(dma_rx),
	rx_gpio(gpio_rx),
	rx_cnt(NULL),
	rx_api_lock(NULL),
	rx_irq_lock(NULL),
	rx_irq_res(UART_ERR_NONE)
{
	ASSERT(_uart < UART_END && uart_list[_uart]);
	ASSERT(_baud > 0);
	ASSERT(tx_dma.dir() == dma::dir_t::DIR_MEM_TO_PERIPH);
	ASSERT(tx_dma.inc_size() == dma::inc_size_t::INC_SIZE_8);
	ASSERT(rx_dma.dir() == dma::dir_t::DIR_PERIPH_TO_MEM);
	ASSERT(rx_dma.inc_size() == dma::inc_size_t::INC_SIZE_8);
	ASSERT(tx_gpio.mode() == gpio::MODE_AF);
	ASSERT(rx_gpio.mode() == gpio::MODE_AF);
	
	tx_api_lock = xSemaphoreCreateMutex();
	ASSERT(tx_api_lock);
	tx_irq_lock = xSemaphoreCreateBinary();
	ASSERT(tx_irq_lock);
	rx_api_lock = xSemaphoreCreateMutex();
	ASSERT(rx_api_lock);
	rx_irq_lock = xSemaphoreCreateBinary();
	ASSERT(rx_irq_lock);
	
#if configUSE_TRACE_FACILITY
	vTraceSetMutexName((void *)tx_api_lock, "uart_tx_api_lock");
	vTraceSetSemaphoreName((void *)tx_irq_lock, "uart_tx_irq_lock");
	vTraceSetMutexName((void *)rx_api_lock, "uart_rx_api_lock");
	vTraceSetSemaphoreName((void *)rx_irq_lock, "uart_rx_irq_lock");
	isr_dma_tx = xTraceSetISRProperties("ISR_dma_uart_tx", 1);
	isr_dma_rx = xTraceSetISRProperties("ISR_dma_uart_rx", 1);
	isr_uart = xTraceSetISRProperties("ISR_uart", 1);
#endif
	
	obj_list[_uart] = this;
	
	*rcc_addr_list[_uart] |= rcc_list[_uart];
	*reset_addr_list[_uart] |= reset_list[_uart];
	*reset_addr_list[_uart] &= ~reset_list[_uart];
	
	gpio_af_init(_uart, tx_gpio);
	gpio_af_init(_uart, rx_gpio);
	
	USART_TypeDef *uart_base = uart_list[_uart];
	
	switch(_stopbit)
	{
		case UART_STOPBIT_0_5: uart_base->CR2 |= USART_CR2_STOP_0; break;
		case UART_STOPBIT_1: uart_base->CR2 &= ~USART_CR2_STOP; break;
		case UART_STOPBIT_1_5: uart_base->CR2 |= USART_CR2_STOP; break;
		case UART_STOPBIT_2: uart_base->CR2 |= USART_CR2_STOP_1; break;
	}
	
	switch(_parity)
	{
		case UART_PARITY_NONE:
			uart_base->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
			break;
		case UART_PARITY_EVEN:
			uart_base->CR1 |= USART_CR1_PCE;
			uart_base->CR1 &= ~USART_CR1_PS;
			break;
		case UART_PARITY_ODD:
			uart_base->CR1 |= USART_CR1_PCE | USART_CR1_PS;
			break;
	}
	
	/* Calculate UART prescaller */
	uint32_t div = rcc_get_freq(rcc_src_list[_uart]) / _baud;
	/* Baud rate is too low or too high */
	ASSERT(div > 0 && div <= MAX_BRR_VAL);
	uart_base->BRR = (uint16_t)div;
	
	tx_dma.dst((uint8_t *)&uart_base->DR);
	rx_dma.src((uint8_t *)&uart_base->DR);
	
	uart_base->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_IDLEIE |
		USART_CR1_PEIE;
	uart_base->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_EIE |
		USART_CR3_ONEBIT;
	
	NVIC_ClearPendingIRQ(irq_list[_uart]);
	NVIC_SetPriority(irq_list[_uart], IRQ_PRIORITY);
	NVIC_EnableIRQ(irq_list[_uart]);
}

uart::~uart()
{
	*reset_addr_list[_uart] |= reset_list[_uart];
	*reset_addr_list[_uart] &= ~reset_list[_uart];
	*rcc_addr_list[_uart] &= ~rcc_list[_uart];
}

void uart::baud(uint32_t baud)
{
	ASSERT(baud > 0);
	
	xSemaphoreTake(tx_api_lock, portMAX_DELAY);
	xSemaphoreTake(rx_api_lock, portMAX_DELAY);
	
	_baud = baud;
	USART_TypeDef *uart = uart_list[_uart];
	uart->CR1 &= ~USART_CR1_UE;
	uint32_t div = rcc_get_freq(rcc_src_list[_uart]) / _baud;
	/* Baud rate is too low or too high */
	ASSERT(div > 0 && div <= MAX_BRR_VAL);
	
	uart->BRR = (uint16_t)div;
	uart->CR1 |= USART_CR1_UE;
	
	xSemaphoreGive(rx_api_lock);
	xSemaphoreGive(tx_api_lock);
}

int8_t uart::tx(const uint8_t *buff, uint16_t size)
{
	ASSERT(buff);
	ASSERT(size > 0);
	
	xSemaphoreTake(tx_api_lock, portMAX_DELAY);
	
	tx_dma.src((uint8_t*)buff);
	tx_dma.size(size);
	tx_dma.start_once(on_dma_tx, this);
	
	/* Task will be blocked during uart tx operation */
	/* irq_lock will be given later from irq handler */
	xSemaphoreTake(tx_irq_lock, portMAX_DELAY);
	
	xSemaphoreGive(tx_api_lock);
	
	return tx_irq_res;
}

int8_t uart::rx(uint8_t *buff, uint16_t *size, uint32_t timeout)
{
	ASSERT(buff);
	ASSERT(size);
	ASSERT(*size > 0);
	
	xSemaphoreTake(rx_api_lock, portMAX_DELAY);
	
	rx_dma.dst(buff);
	rx_dma.size(*size);
	*size = 0;
	rx_cnt = size;
	
	USART_TypeDef *uart = uart_list[_uart];
	uart->CR1 |= USART_CR1_RE;
	rx_dma.start_once(on_dma_rx, this);
	
	/* Task will be blocked during uart rx operation */
	/* irq_lock will be given later from irq handler */
	if(xSemaphoreTake(rx_irq_lock, timeout) != pdTRUE)
	{
		vPortEnterCritical();
		/* Prevent common (non-DMA) UART IRQ */
		uart->CR1 &= ~USART_CR1_RE;
		uint32_t sr = uart->SR;
		uint32_t dr = uart->DR;
		NVIC_ClearPendingIRQ(irq_list[_uart]);
		/* Prevent DMA IRQ */
		rx_dma.stop();
		rx_irq_res = UART_ERR_RX_TIMEOUT;
		vPortExitCritical();
	}
	xSemaphoreGive(rx_api_lock);
	
	return rx_irq_res;
}

int8_t uart::exch(uint8_t *tx_buff, uint16_t tx_size, uint8_t *rx_buff,
	uint16_t *rx_size, uint32_t timeout)
{
	ASSERT(tx_buff);
	ASSERT(rx_buff);
	ASSERT(tx_size > 0);
	ASSERT(rx_size);
	ASSERT(*rx_size > 0);
	
	xSemaphoreTake(tx_api_lock, portMAX_DELAY);
	xSemaphoreTake(rx_api_lock, portMAX_DELAY);
	
	/* Prepare tx */
	tx_dma.src((uint8_t *)tx_buff);
	tx_dma.size(tx_size);
	
	/* Prepare rx */
	rx_dma.dst(rx_buff);
	rx_dma.size(*rx_size);
	*rx_size = 0;
	rx_cnt = rx_size;
	
	/* Start rx */
	USART_TypeDef *uart = uart_list[_uart];
	uart->CR1 |= USART_CR1_RE;
	rx_dma.start_once(on_dma_rx, this);
	/* Start tx */
	tx_dma.start_once(on_dma_tx, this);
	
	/* Task will be blocked during uart tx operation */
	/* irq_lock will be given later from irq handler */
	xSemaphoreTake(tx_irq_lock, portMAX_DELAY);
	
	/* Task will be blocked during uart rx operation */
	/* irq_lock will be given later from irq handler */
	if(xSemaphoreTake(rx_irq_lock, timeout) != pdTRUE)
	{
		vPortEnterCritical();
		/* Prevent common (non-DMA) UART IRQ */
		uart->CR1 &= ~USART_CR1_RE;
		uint32_t sr = uart->SR;
		uint32_t dr = uart->DR;
		NVIC_ClearPendingIRQ(irq_list[_uart]);
		/* Prevent DMA IRQ */
		rx_dma.stop();
		rx_irq_res = UART_ERR_RX_TIMEOUT;
		vPortExitCritical();
	}
	
	xSemaphoreGive(rx_api_lock);
	xSemaphoreGive(tx_api_lock);
	
	if(tx_irq_res != UART_ERR_NONE)
		return tx_irq_res;
	
	return rx_irq_res;
}

static void gpio_af_init(uart_t uart, gpio &gpio)
{
	GPIO_TypeDef *gpio_reg = gpio_list[gpio.port()];
	
	uint8_t pin = gpio.pin();
	/* Push-pull type */
	gpio_reg->OTYPER &= ~(GPIO_OTYPER_OT0 << pin);
	
	/* Configure alternate function */
	if(pin < 8)
	{
		gpio_reg->AFR[0] &= ~(0x0F << (pin * 4));
		gpio_reg->AFR[0] |= gpio_af_list[uart] << (pin * 4);
	}
	else
	{
		gpio_reg->AFR[1] &= ~(0x0F << ((pin - 8) * 4));
		gpio_reg->AFR[1] |= gpio_af_list[uart] << ((pin - 8) * 4);
	}
}

void uart::on_dma_tx(dma *dma, dma::event_t event, void *ctx)
{
	if(event == dma::event_t::EVENT_HALF)
		return;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_dma_tx);
#endif
	uart *obj = static_cast<uart *>(ctx);
	
	if(event == dma::event_t::EVENT_CMPLT)
		obj->tx_irq_res = UART_ERR_NONE;
	else if(event == dma::event_t::EVENT_ERROR)
		obj->tx_irq_res = UART_ERR_TX_FAIL;
	
	BaseType_t hi_task_woken = 0;
	xSemaphoreGiveFromISR(obj->tx_irq_lock, &hi_task_woken);
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
	portYIELD_FROM_ISR(hi_task_woken);
}

void uart::on_dma_rx(dma *dma, dma::event_t event, void *ctx)
{
	if(event == dma::event_t::EVENT_HALF)
		return;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_dma_rx);
#endif
	uart *obj = static_cast<uart *>(ctx);
	USART_TypeDef *uart = uart_list[obj->_uart];
	
	/* Prevent common (non-DMA) UART IRQ */
	uart->CR1 &= ~USART_CR1_RE;
	uint32_t sr = uart->SR;
	uint32_t dr = uart->DR;
	NVIC_ClearPendingIRQ(irq_list[obj->_uart]);
	
	if(event == dma::event_t::EVENT_CMPLT)
		obj->rx_irq_res = UART_ERR_NONE;
	else if(event == dma::event_t::EVENT_ERROR)
		obj->rx_irq_res = UART_ERR_RX_FAIL;
	/* Rx buffer has partly filled (package has received) or Rx buffer has
	totally filled */
	if(obj->rx_cnt)
		*obj->rx_cnt = obj->rx_dma.transfered();
	
	BaseType_t hi_task_woken = 0;
	xSemaphoreGiveFromISR(obj->rx_irq_lock, &hi_task_woken);
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
	portYIELD_FROM_ISR(hi_task_woken);
}

extern "C" void uart_irq_hndlr(hal::uart *obj)
{
	USART_TypeDef *uart = uart_list[obj->_uart];
	uint32_t sr = uart->SR;
	uint32_t dr = uart->DR;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_uart);
#endif
	if((uart->CR1 & USART_CR1_IDLEIE) && (sr & USART_SR_IDLE))
	{
		/* IDLE event has happened (package has received) */
		obj->rx_irq_res = UART_ERR_NONE;
	}
	else if((uart->CR3 & USART_CR3_EIE) && (sr & (USART_SR_PE | USART_SR_FE |
		USART_SR_NE | USART_SR_ORE)))
	{
		/* Error event has happened */
		obj->rx_irq_res = UART_ERR_RX_FAIL;
	}
	else
	{
#if configUSE_TRACE_FACILITY
		vTraceStoreISREnd(0);
#endif
		return;
	}
	
	/* Prevent DMA IRQ */
	obj->rx_dma.stop();
	
	uart->CR1 &= ~USART_CR1_RE;
	if(obj->rx_cnt)
		*obj->rx_cnt = obj->rx_dma.transfered();
	BaseType_t hi_task_woken = 0;
	xSemaphoreGiveFromISR(obj->rx_irq_lock, &hi_task_woken);
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
	portYIELD_FROM_ISR(hi_task_woken);
}

extern "C" void USART1_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[UART_1]);
}

extern "C" void USART2_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[UART_2]);
}

#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
extern "C" void USART3_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[UART_3]);
}
#endif

#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
extern "C" void UART4_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[UART_4]);
}

extern "C" void UART5_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[UART_5]);
}
#endif

#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
extern "C" void USART6_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[UART_6]);
}
#endif
