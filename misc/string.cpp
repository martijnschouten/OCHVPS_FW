#include "misc/string.h"

//Convert a uint8_t to array of char (hexadecimal)
//output must be a pointer to an array of char with at least 2 free elements after the pointer
void string_uint8toCharArrayHex(uint8_t input, uint8_t *output) {
	uint8_t nibble = 0;

	//Left nibble
	nibble = input / 16;

	if (nibble <= 9) {
		*output = '0' + nibble;
	} else {
		*output = 'A' + (nibble - 10);
	}

	//Right nibble
	nibble = input % 16;

	if (nibble <= 9) {
		*(output + 1) = '0' + nibble;
	} else {
		*(output + 1) = 'A' + (nibble - 10);
	}

	return;
}

//Convert a uint8_t to array of 3 char (decimal)
//output must be a pointer to an array of char with at least 3 free elements after the pointer
void string_uint8toCharArrayDec(uint8_t input, uint8_t *output) {
	//hundreds
	*(output + 0) = input / 100;

	//tens
	input -= (*(output + 0) * 100);
	*(output + 1) = input / 10;

	//units
	input -= (*(output + 1) * 10);
	*(output + 2) = input;

	//Convert to ASCII
	*(output + 0) += '0';
	*(output + 1) += '0';
	*(output + 2) += '0';

	return;
}

//Convert a uint16_t to array of 5 char (decimal)
//output must be a pointer to an array of char with at least 5 free elements after the pointer
void string_uint16toCharArrayDec(uint16_t input, uint8_t *output) {
	//10*thousands
	*(output + 0) = input / 10000;

	//thousands
	input -= (*(output + 0) * 10000);
	*(output + 1) = input / 1000;

	//hundreds
	input -= (*(output + 1) * 1000);
	*(output + 2) = input / 100;

	//tens
	input -= (*(output + 2) * 100);
	*(output + 3) = input / 10;

	//units
	input -= (*(output + 3) * 10);
	*(output + 4) = input;

	//Convert to ASCII
	*(output + 0) += '0';
	*(output + 1) += '0';
	*(output + 2) += '0';
	*(output + 3) += '0';
	*(output + 4) += '0';

	return;
}

//Convert a uint32_t to array of 10 char (decimal)
//output must be a pointer to an array of char with at least 10 free elements after the pointer
void string_uint32toCharArrayDec(uint32_t input, uint8_t *output) {
	//x*1000'000'000
	*(output + 0) = input / (1000000000);

	//x*100'000'000
	input -= (*(output + 0) * (1000000000));
	*(output + 1) = input / (100000000);

	//x*10'000'000
	input -= (*(output + 1) * (100000000));
	*(output + 2) = input / (10000000);

	//x*1000'000
	input -= (*(output + 2) * (10000000));
	*(output + 3) = input / (1000000);

	//x*100'000
	input -= (*(output + 3) * (1000000));
	*(output + 4) = input / (100000);

	//x*10'000
	input -= (*(output + 4) * (100000));
	*(output + 5) = input / (10000);

	//thousands
	input -= (*(output + 5) * (10000));
	*(output + 6) = input / (1000);

	//hundreds
	input -= (*(output + 6) * (1000));
	*(output + 7) = input / (100);

	//tens
	input -= (*(output + 7) * (100));
	*(output + 8) = input / (10);

	//units
	input -= (*(output + 8) * 10);
	*(output + 9) = input;

	//Convert to ASCII
	uint8_t i;
	for (i = 0; i < 10; i++) {
		*(output + i) += '0';
	}

	return;
}

//Convert a int32_t to array of 10 char (decimal), preceded with the sign
//output must be a pointer to an array of char with at least 11 free elements after the pointer
void string_int32toCharArrayDec(int32_t input, uint8_t *output) {
	if (input > 0) {
		*(output + 0) = '+';
	} else {
		*(output + 0) = '-';
		input *= -1;
	}

	//x*1000'000'000
	*(output + 1) = input / (1000000000);

	//x*100'000'000
	input -= (*(output + 1) * (1000000000));
	*(output + 2) = input / (100000000);

	//x*10'000'000
	input -= (*(output + 2) * (100000000));
	*(output + 3) = input / (10000000);

	//x*1000'000
	input -= (*(output + 3) * (10000000));
	*(output + 4) = input / (1000000);

	//x*100'000
	input -= (*(output + 4) * (1000000));
	*(output + 5) = input / (100000);

	//x*10'000
	input -= (*(output + 5) * (100000));
	*(output + 6) = input / (10000);

	//thousands
	input -= (*(output + 6) * (10000));
	*(output + 7) = input / (1000);

	//hundreds
	input -= (*(output + 7) * (1000));
	*(output + 8) = input / (100);

	//tens
	input -= (*(output + 8) * (100));
	*(output + 9) = input / (10);

	//units
	input -= (*(output + 9) * 10);
	*(output + 10) = input;

	//Convert to ASCII
	uint8_t i;
	for (i = 1; i < 11; i++) {
		*(output + i) += '0';
	}

	return;
}
