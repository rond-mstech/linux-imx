/*
 * This header provides constants for most input bindings.
 *
 * Most input bindings include key code, matrix key code format.
 * In most cases, key code and matrix key code format uses
 * the standard values/macro defined in this header.
 */

#ifndef _DT_BINDINGS_INPUT_INPUT_H
#define _DT_BINDINGS_INPUT_INPUT_H

#include "linux-event-codes.h"

#define MATRIX_KEY(row, col, code)	\
	((((row) & 0xFF) << 24) | (((col) & 0xFF) << 16) | ((code) & 0xFFFF))

#define FT5416     0x54160002
#define FT5426     0x54260002
#define FT6236U    0x6236D003
#define FT6336G    0x6336A003
#define FT6336U    0x6336D003
#define FT6436U    0x6436D003


#endif /* _DT_BINDINGS_INPUT_INPUT_H */
