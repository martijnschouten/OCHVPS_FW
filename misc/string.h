#ifndef STRING_H_
#define STRING_H_

#include <stdint.h>

//Convert a uint8_t to array of char (hexadecimal)
//output must be a pointer to an array of char with at least 2 free elements after the pointer
void string_uint8toCharArrayHex(uint8_t input, uint8_t* output);

//Convert a uint8_t to array of 3 char (decimal)
//output must be a pointer to an array of char with at least 3 free elements after the pointer
void string_uint8toCharArrayDec(uint8_t input, uint8_t* output);

//Convert a uint16_t to array of 5 char (decimal)
//output must be a pointer to an array of char with at least 5 free elements after the pointer
void string_uint16toCharArrayDec(uint16_t input, uint8_t* output);

//Convert a uint32_t to array of 10 char (decimal)
//output must be a pointer to an array of char with at least 10 free elements after the pointer
void string_uint32toCharArrayDec(uint32_t input, uint8_t* output);

//Convert a int32_t to array of 10 char (decimal), preceded with the sign
//output must be a pointer to an array of char with at least 11 free elements after the pointer
void string_int32toCharArrayDec(int32_t input, uint8_t* output);

#endif /* STRING_H_ */
