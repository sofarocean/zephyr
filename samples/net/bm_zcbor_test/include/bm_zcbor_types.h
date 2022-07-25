/*
 * Generated using zcbor version 0.5.99
 * https://github.com/NordicSemiconductor/zcbor
 * Generated with a --default-max-qty of 3
 */

#ifndef BM_ZCBOR_TYPES_H__
#define BM_ZCBOR_TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "zcbor_encode.h"

/** Which value for --default-max-qty this file was created with.
 *
 *  The define is used in the other generated file to do a build-time
 *  compatibility check.
 *
 *  See `zcbor --help` for more information about --default-max-qty
 */
#define DEFAULT_MAX_QTY 3

struct Pet {
	struct zcbor_string _Pet_name_names[3];
	uint_fast32_t _Pet_name_names_count;
	struct zcbor_string _Pet_birthday;
	enum {
		_Pet_species_cat = 1,
		_Pet_species_dog = 2,
		_Pet_species_other = 3,
	} _Pet_species_choice;
};


#endif /* BM_ZCBOR_TYPES_H__ */
