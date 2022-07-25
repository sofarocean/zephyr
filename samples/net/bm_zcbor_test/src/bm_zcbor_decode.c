/*
 * Generated using zcbor version 0.5.99
 * https://github.com/NordicSemiconductor/zcbor
 * Generated with a --default-max-qty of 3
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "zcbor_decode.h"
#include "bm_zcbor_decode.h"

#if DEFAULT_MAX_QTY != 3
#error "The type file was generated with a different default_max_qty than this file"
#endif

static bool decode_Pet(zcbor_state_t *state, struct Pet *result);


static bool decode_Pet(
		zcbor_state_t *state, struct Pet *result)
{
	zcbor_print("%s\r\n", __func__);

	bool tmp_result = (((zcbor_list_start_decode(state) && ((((zcbor_list_start_decode(state) && ((zcbor_multi_decode(1, 3, &(*result)._Pet_name_names_count, (zcbor_decoder_t *)zcbor_tstr_decode, state, (&(*result)._Pet_name_names), sizeof(struct zcbor_string))) || (zcbor_list_map_end_force_decode(state), false)) && zcbor_list_end_decode(state)))
	&& ((zcbor_bstr_decode(state, (&(*result)._Pet_birthday)))
	&& ((((((*result)._Pet_birthday.len >= 8)
	&& ((*result)._Pet_birthday.len <= 8)) || (zcbor_error(state, ZCBOR_ERR_WRONG_RANGE), false))) || (zcbor_error(state, ZCBOR_ERR_WRONG_RANGE), false)))
	&& ((((zcbor_int_decode(state, &(*result)._Pet_species_choice, sizeof((*result)._Pet_species_choice)))) && ((((((*result)._Pet_species_choice == _Pet_species_cat) && ((1)))
	|| (((*result)._Pet_species_choice == _Pet_species_dog) && ((1)))
	|| (((*result)._Pet_species_choice == _Pet_species_other) && ((1)))) || (zcbor_error(state, ZCBOR_ERR_WRONG_VALUE), false)))))) || (zcbor_list_map_end_force_decode(state), false)) && zcbor_list_end_decode(state))));

	if (!tmp_result)
		zcbor_trace();

	return tmp_result;
}



int cbor_decode_Pet(
		const uint8_t *payload, size_t payload_len,
		struct Pet *result,
		size_t *payload_len_out)
{
	zcbor_state_t states[4];

	zcbor_new_state(states, sizeof(states) / sizeof(zcbor_state_t), payload, payload_len, 1);

	bool ret = decode_Pet(states, result);

	if (ret && (payload_len_out != NULL)) {
		*payload_len_out = MIN(payload_len,
				(size_t)states[0].payload - (size_t)payload);
	}

	if (!ret) {
		int err = zcbor_pop_error(states);

		zcbor_print("Return error: %d\r\n", err);
		return (err == ZCBOR_SUCCESS) ? ZCBOR_ERR_UNKNOWN : err;
	}
	return ZCBOR_SUCCESS;
}
