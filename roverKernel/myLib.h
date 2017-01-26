/*
 * myLib.h
 *
 *  Created on: 20. 4. 2015.
 *      Author: Vedran
 */

#ifndef MYLIB_H_
#define MYLIB_H_

#include <stdint.h>

#define STATUS_OK 					 0
#define STATUS_ARG_ERR				-1

#define PI_CONST 	3.14159265f


/*		Math-related function		*/
extern int32_t interpolate(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t _x);
extern float finterpolatef(float x1, float y1, float x2, float y2, float _x);
extern int32_t min(int32_t arg1, int32_t arg2);

/*		Functions for converting string to number		*/
extern float stof (uint8_t *nums, uint8_t strLen);
extern int32_t stoi (uint8_t *nums, uint8_t strLen);

#endif /* MYLIB_H_ */
