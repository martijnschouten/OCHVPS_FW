#include <stdint.h>

#ifndef MSERIAL_H_
#define MSERIAL_H_

static const uint32_t rx_buffer_size __attribute__((used)) = 32;
static uint8_t rx_buffer[rx_buffer_size] __attribute__((used));
static volatile uint16_t txIndex; // Index of the data to send out.
static volatile uint16_t rxIndex; // Index of the memory to save new arrived data.

class mSerial{
	public:
		mSerial();
		uint32_t setup();
		uint32_t writeInit();

		//uint32
		uint32_t print(uint8_t data);
		uint32_t println(uint8_t data);

		//int32_t
		uint32_t print(int32_t data);
		uint32_t println(int32_t data);

		//uint32
		uint32_t print(uint32_t data);
		uint32_t println(uint32_t data);
		uint32_t print_time(uint32_t data);
		uint32_t print_current(uint32_t data);

		//double
		uint32_t print(double data);
		uint32_t println(double data);
		uint32_t print_hv_vm(double data);
		uint32_t print_lv_vm(double data);


		//char
		uint32_t print(char data);
		uint32_t println(char data);


		//string
		uint8_t print(const char *aDataPtr);
		uint8_t println(const char *aDataPtr);

		uint8_t print(const char *aDataPtr, uint16_t aSize);

		uint32_t getInputBufferCount();
		uint8_t readByte();

	private:
		static uint32_t numberOfInstances;

};

#endif /* MSERIAL_H_ */
