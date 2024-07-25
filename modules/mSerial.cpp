#include "mSerial.h"
#include "source/globals.h"

uint32_t mSerial::numberOfInstances = 0;

extern "C" {
void UART0_RX_TX_IRQHandler(void) {
	uint8_t data;

	/* If new data arrived. */
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART0)) {
		data = UART_ReadByte(UART0);

		/* If ring buffer is not full, add data to ring buffer. */
		if (((rxIndex + 1) % rx_buffer_size) != txIndex) {
			rx_buffer[rxIndex] = data;
			rxIndex++;
			rxIndex %= rx_buffer_size;
		}
	}
	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
		 exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
	__DSB();
#endif
}
}

mSerial::mSerial() {
	numberOfInstances++;
	if (numberOfInstances > 1) {
		while (true);
	}
}

uint32_t mSerial::setup() {

	uint8_t txbuff_startup[] = "\n\nRESTART\n\n";

	uart_config_t config;
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableTx = true;
	config.enableRx = true;
	UART_Init(UART0, &config, CLOCK_GetFreq(UART0_CLK_SRC));

	UART_WriteBlocking(UART0, txbuff_startup, sizeof(txbuff_startup) - 1);

	// Enable RX interrupt
	UART_EnableInterrupts(UART0, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
	EnableIRQ(UART0_RX_TX_IRQn);

	return 0;
}

uint32_t mSerial::writeInit() {
	//uint8_t txexample[] = "This is a welcome message which should give you some help. Ask VPY to implement it\n";
	//UART_WriteBlocking(UART0, txexample, sizeof(txexample) - 1);

	return 0;
}

//----------------------------------------------------------
// UINT8_T
//----------------------------------------------------------
uint32_t mSerial::print(uint8_t data) {
	char buffer [5];
	int cx;
	cx = snprintf (buffer, 5, "%d", data);
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}

uint32_t mSerial::println(uint8_t data) {
	char buffer [6];
	int cx;
	cx = snprintf (buffer, 5, "%d", data);
	buffer[cx++]='\n';
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}


//----------------------------------------------------------
// INT32
//----------------------------------------------------------
uint32_t mSerial::print(int32_t data) {
	char buffer [13];
	int cx;
	cx = snprintf (buffer, 13, "%ld", data);
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}

uint32_t mSerial::println(int32_t data) {
	char buffer [14];
	int cx;
	cx = snprintf (buffer, 13, "%ld", data);
	buffer[cx++]='\n';
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}


//----------------------------------------------------------
// UINT32
//----------------------------------------------------------
uint32_t mSerial::print(uint32_t data) {
	char buffer[12];
	int cx;
	cx = snprintf (buffer, 12, "%lu", data);
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}

uint32_t mSerial::println(uint32_t data) {
	char buffer[13];
	int cx;
	cx = snprintf (buffer, 12, "%lu", data);
	buffer[cx++]='\n';
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}

uint32_t mSerial::print_time(uint32_t data) {
	char buffer[10];
	int cx;
	cx = snprintf (buffer, 10, "%lu", data);
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}

uint32_t mSerial::print_current(uint32_t data) {
	char buffer[8];
	int cx;
	cx = snprintf (buffer, 10, "%0lu", data);
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}

//----------------------------------------------------------
// DOUBLE
//----------------------------------------------------------
uint32_t mSerial::print(double data) {
	char buffer [12];
	int cx;
	cx = snprintf (buffer, 12, "%6f", data);
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}

uint32_t mSerial::println(double data) {
	char buffer [13];
	int cx;
	cx = snprintf (buffer, 12, "%6f", data);
	buffer[cx++]='\n';
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}

uint32_t mSerial::print_hv_vm(double data) {
	char buffer [8];
	int cx;
	cx = snprintf (buffer, 12, "%04.3f", data);
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}

uint32_t mSerial::print_lv_vm(double data) {
	char buffer[6];
	int cx;
	cx = snprintf (buffer, 12, "%02.3f", data);
	UART_WriteBlocking(UART0, (const uint8_t *)buffer, cx);
	return 0;
}

//----------------------------------------------------------
// CHAR
//----------------------------------------------------------
uint32_t mSerial::print(char data){
	while (!(kUART_TransmissionCompleteFlag & UART_GetStatusFlags(UART0)));
	UART_WriteByte(UART0, data);
	return 0;
}

uint32_t mSerial::println(char data){
	while (!(kUART_TransmissionCompleteFlag & UART_GetStatusFlags(UART0)));
	UART_WriteByte(UART0, data);
	while (!(kUART_TransmissionCompleteFlag & UART_GetStatusFlags(UART0)));
	UART_WriteByte(UART0, '\n');
	return 0;
}

//----------------------------------------------------------
// STRING (null terminated)
//----------------------------------------------------------
uint8_t mSerial::print(const char *aDataPtr) {
	// Send char until NULL char
	while(*aDataPtr!=0) {
		// Wait end of transmit
		while (!(kUART_TransmissionCompleteFlag & UART_GetStatusFlags(UART0)));

		// Write data
		UART_WriteByte(UART0, *aDataPtr);

		// Next char
		aDataPtr++;
	}
	return 0;
}

uint8_t mSerial::println(const char *aDataPtr) {
	// Send char until NULL char
	while(*aDataPtr!=0) {
		// Wait end of transmit
		while (!(kUART_TransmissionCompleteFlag & UART_GetStatusFlags(UART0)));

		// Write data
		UART_WriteByte(UART0, *aDataPtr);

		// Next char
		aDataPtr++;
	}
	while (!(kUART_TransmissionCompleteFlag & UART_GetStatusFlags(UART0)));
	UART_WriteByte(UART0, '\n');
	return 0;
}


//-----------------------------------------------------------------------------
// String send (fixed size)
// The transmission is size defined
// *aDataPtr	: string address
// aSize		: length of the frame (bytes)
//-----------------------------------------------------------------------------
uint8_t mSerial::print(const char *aDataPtr, uint16_t aSize) {
	// Send char until NULL char
	uint16_t i;
	for(i=0; i<aSize; i++) {
		// Wait end of transmit
		while (!(kUART_TransmissionCompleteFlag & UART_GetStatusFlags(UART0)));

		// Write data
		UART_WriteByte(UART0, *aDataPtr);

		// Next char
		aDataPtr++;
	}
	return 0;
}

uint32_t mSerial::getInputBufferCount(){
	return ((rx_buffer_size + rxIndex - txIndex) % rx_buffer_size);
}

uint8_t mSerial::readByte(){
	uint8_t retVal = 0;

	if (rxIndex != txIndex) {
		retVal = rx_buffer[txIndex];
		//			UART_WriteByte(UART0, rx_buffer[txIndex]);
		txIndex++;
		txIndex %= rx_buffer_size;
		//			while (!(kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(UART0)));
	}
	return retVal;
}


//void mSerial::processRxData() {
//	// Send data only when UART TX register is empty and ring buffer has data to send out.
//	while ((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(UART0)) && (rxIndex != txIndex)) {
//		UART_WriteByte(UART0, rx_buffer[txIndex]);
//		txIndex++;
//		txIndex %= rx_buffer_size;
//	}
//}
//
//void mSerial::processRxDataBlocking() {
//	// Send data only when UART TX register is empty and ring buffer has data to send out.
//	while (rxIndex != txIndex) {
//		UART_WriteByte(UART0, rx_buffer[txIndex]);
//		txIndex++;
//		txIndex %= rx_buffer_size;
//		while (!(kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(UART0)));
//	}
//}
