/*
 * (c) 2018 - idlab - UGent - imec
 *
 * Bart Moons
 *
 * This file is part of the SCHC stack implementation
 *
 */

#include <string.h>
#include <stdio.h>

#include "compressor.h"
#include "schc_config.h"

#include "rules/rule_config.h"

#if CLICK
#include <click/config.h>
#endif

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "SCHC-COMPRESS"
#define LOG_LEVEL LOG_LEVEL_6LOWPAN

static schc_ipaddr_t node_ip_6;


////////////////////////////////////////////////////////////////////////////////////
//                                LOCAL FUNCIONS                                  //
////////////////////////////////////////////////////////////////////////////////////

/**
 * Get the node its IP address as set during initialization
 *
 * @return node_ip_6 the node its IP address
 *
 */
static void get_node_ip(schc_ipaddr_t *node_ip) {
	memcpy(node_ip, node_ip_6, sizeof(schc_ipaddr_t));
}

/**
 * Get a device by it's id
 *
 * @param device_id 	the id of the device
 *
 * @return schc_device 	the device which is found
 *         NULL			if no device was found
 *
 */
static struct schc_device* get_device_by_id(uint32_t device_id) {
	int i = 0;

	for (i = 0; i < DEVICE_COUNT; i++) {
		if (devices[i]->device_id == device_id) {
			return (struct schc_device*) devices[i];
		}
	}

	return NULL;
}

/*
 * Set the rule id of the compressed packet
 *
 * @param 	schc_rule 		the schc rule to use
 * @param 	data			the compressed packet buffer
 *
 * @return 	err				error codes
 * 			1				SUCCESS
 *
 */
int8_t set_rule_id(struct schc_rule_t* schc_rule, uint8_t* data) {
	// copy rule id in front of the buffer
	uint8_t pos = get_position_in_first_byte(RULE_SIZE_BITS);
	clear_bits(data, 0, RULE_SIZE_BITS); // clear bits before setting

	if(schc_rule != NULL) {
		copy_bits(data, 0, schc_rule->id, pos, RULE_SIZE_BITS);
	} else {
		copy_bits(data, 0, UNCOMPRESSED_ID, pos, RULE_SIZE_BITS);
	}

	return 1;
}

/*
 * Find a replacement rule with the correct reliability mode
 *
 * @param 	schc_rule 		the schc rule to find a replacement rule for
 * @param 	mode			the mode for which a rule should be found
 * @param 	device_id		the device to find a rule for
 *
 * @return 	schc_rule		the rule that was found
 * 			NULL			if no rule was found
 *
 */
struct schc_rule_t* get_schc_rule_by_reliability_mode(
		struct schc_rule_t* schc_rule, reliability_mode mode,
		uint32_t device_id) {
	struct schc_device *device = get_device_by_id(device_id);

	if (device == NULL) {
		printf(
				"get_schc_rule(): no device was found for the id: %d\r\n", device_id);
		return NULL;
	}

	int i;
	for (i = 0; i < device->rule_count; i++) {
		const struct schc_rule_t* curr_rule = (*device->context)[i];
		if ((schc_rule->compression_rule == curr_rule->compression_rule)
				&& (curr_rule->mode == mode)) {
			return (struct schc_rule_t*) (curr_rule);
		}
	}

	return NULL;
}

/*
 * Combine the different layers to find the SCHC rule entry
 *
 * @param 	ip_rule_id 		the rule id for the IP layer
 * @param 	udp_rule_id		the rule id for the UDP layer
 * @param 	coap_rule_id	the rule id for the CoAP layer
 * @param 	device_id		the device to find a rule for
 * @param 	mode			the mode for which a rule should be found
 *
 * @return 	schc_rule		the rule that was found
 * 			NULL			if no rule was found
 *
 */
static struct schc_rule_t* get_schc_rule_by_layer_ids(uint8_t ip_rule_id,
		uint8_t udp_rule_id, uint8_t coap_rule_id, uint32_t device_id,
		reliability_mode mode) {
	int i;
	struct schc_device *device = get_device_by_id(device_id);

	if (device == NULL) {
		printf(
				"get_schc_rule(): no device was found for this id \r\n");
		return NULL;
	}

	for (i = 0; i < device->rule_count; i++) {
		const struct schc_rule_t* curr_rule = (*device->context)[i];

#if USE_IPv6
		if (curr_rule->compression_rule->ipv6_rule->rule_id == ip_rule_id) {
#if USE_UDP
			if (curr_rule->compression_rule->udp_rule->rule_id == udp_rule_id) {
				if (curr_rule->mode == mode) {
					return (struct schc_rule_t*) (curr_rule);
				}
			}
#else
			if (curr_rule->mode == mode) {
				return (struct schc_rule_t*) (curr_rule);
			}
#endif
		}
#endif
	}

	return NULL;
}

/*
 * Find a SCHC rule entry for a device
 *
 * @param 	rule_arr 		the rule id in uint8_t array
 * @param 	device_id		the device to find a rule for
 *
 * @return 	schc_rule		the rule that was found
 * 			NULL			if no rule was found
 *
 */
struct schc_rule_t* get_schc_rule_by_rule_id(uint8_t* rule_arr, uint32_t device_id) {
	int i;
	struct schc_device *device = get_device_by_id(device_id);

	if (device == NULL) {
		printf("get_schc_rule(): no device was found for this id \r\n");
		return NULL;
	}

	for (i = 0; i < device->rule_count; i++) {
		struct schc_rule_t* curr_rule = (struct schc_rule_t*) (*device->context)[i];
		uint8_t curr_rule_pos = get_position_in_first_byte(RULE_SIZE_BITS);
		if( compare_bits_aligned(curr_rule->id, curr_rule_pos, rule_arr, 0, RULE_SIZE_BITS)) {
			printf("get_schc_rule(): curr rule %p \r\n", curr_rule);
			return curr_rule;
		}
	}

	return NULL;
}


/**
 * The compression mechanism
 *
 * @param dst_arr	 			the bit array in which to copy the contents to
 * @param src_arr 				the original header
 * @param rule 					the rule to match the compression with
 *
 * @return the length 			length of the compressed header
 *
 */
static uint8_t compress(schc_bitarray_t* dst, schc_bitarray_t* src,
		const struct schc_layer_rule_t *rule, direction DI) {
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t field_length;
	uint8_t json_result;

	for (i = 0; i < rule->length; i++) {
		// exclude fields in other direction
		if (((rule->content[i].dir) == BI) || ((rule->content[i].dir) == DI)) {
			field_length = rule->content[i].field_length;

			switch (rule->content[i].action) {
			case NOTSENT: { // do nothing
			}
				break;
			case VALUESENT: {
				uint8_t src_pos = get_position_in_first_byte((field_length + src->offset));
				copy_bits(dst->ptr, dst->offset, src->ptr, src->offset, field_length);
				dst->offset += field_length;
			}
				break;
			case MAPPINGSENT: {
				json_result = 0;

				// if result is 0,
				if (json_result == 0) { // formatted as a normal unsigned char array
					uint8_t list_len = get_required_number_of_bits( (rule->content[i].MO_param_length - 1) ); // start from index 0
					for (j = 0; j < rule->content[i].MO_param_length; j++) {
						uint8_t src_pos = get_number_of_bytes_from_bits(src->offset);

						uint8_t ptr = j;
						if (! (field_length % 8) ) // only support byte aligned matchmap
							ptr = j * get_number_of_bytes_from_bits(field_length); // for multiple byte entry

						if (compare_bits_BIG_END(
								(uint8_t*) (src->ptr + src_pos),
								(uint8_t*) (rule->content[i].target_value + ptr),
								field_length)) { // match which index to send
							uint8_t ind[1] = { j }; // room for 255 indices
							uint8_t src_pos = get_position_in_first_byte(list_len);
							copy_bits(dst->ptr, dst->offset, ind, src_pos, list_len);
							dst->offset += list_len;
						}
					}

				} else {
				}
			}
				break;
			case LSB: {
				uint16_t lsb_len = rule->content[i].field_length - rule->content[i].MO_param_length;
				copy_bits(dst->ptr, dst->offset, (uint8_t*) (src->ptr),
						rule->content[i].MO_param_length + src->offset, lsb_len);
				dst->offset += lsb_len;
			}
				break;
			case COMPLENGTH:
			case COMPCHK: {
				// do nothing
			}
				break;
			case DEVIID: {
				// ToDo
			}
				break;
			case APPIID: {
				// ToDo
			}
				break;
			}
			src->offset += field_length;
		}
	}

	return 1;
}


/**
 * The decompression mechanism
 *
 * @param rule 			pointer to the rule to use during the decompression
 * @param src			the received SCHC bit buffer
 * @param dst			the buffer to store the decompressed, original packet
 *
 * @return the length of the decompressed header
 *
 */
static uint8_t decompress(struct schc_layer_rule_t* rule, schc_bitarray_t* src,
		schc_bitarray_t* dst, direction DI) {
	uint8_t i = 0; uint8_t j;
	uint8_t field_length; int8_t json_result = -1;

	if(rule == NULL)
		return 0;

	for (i = 0; i < rule->length; i++) {
		// exclude fields in other direction
		if (((rule->content[i].dir) == BI) || ((rule->content[i].dir) == DI)) {
			field_length = rule->content[i].field_length;
			switch (rule->content[i].action) {
			case NOTSENT: {
				// use value stored in context
				uint8_t src_pos = get_position_in_first_byte(field_length);
				copy_bits(dst->ptr, dst->offset, rule->content[i].target_value, src_pos, field_length);

			} break;
			case VALUESENT: {
				// build from received value
				copy_bits(dst->ptr, dst->offset, src->ptr, src->offset, field_length);
				src->offset += field_length;
			} break;
			case MAPPINGSENT: {
				// parse the json string
				json_result = 0; // todo

				// if result is 0,
				if (json_result == 0) { // formatted as a normal unsigned uint8_t array
					uint32_t list_len = get_required_number_of_bits( (rule->content[i].MO_param_length - 1) ); // start from index 0
					uint8_t src_pos = get_position_in_first_byte(list_len);

					uint8_t index[1] = { 0 };
					copy_bits((uint8_t*) (index), src_pos, src->ptr, src->offset, list_len); // copy the index from the received header
					if( ! (field_length % 8) ) // multiply with byte alligned field length
						index[0] = index[0] * get_number_of_bytes_from_bits(field_length);


					copy_bits(dst->ptr, dst->offset, (uint8_t*) (rule->content[i].target_value + index[0]), 0, field_length);
					src->offset += list_len;
				}
			} break;
			case LSB: {
				uint8_t msb_len = rule->content[i].MO_param_length;
				uint8_t lsb_len = rule->content[i].field_length - msb_len;
				// build partially from rule
				copy_bits(dst->ptr, dst->offset, rule->content[i].target_value, 0, msb_len);

				// .. and from received value
				copy_bits(dst->ptr, dst->offset + msb_len, src->ptr, src->offset, lsb_len);
				src->offset += lsb_len;
			} break;
			case COMPLENGTH:
			case COMPCHK: {
				clear_bits(dst->ptr, dst->offset, field_length); // set to 0, to indicate that it will be calculated after decompression
			} break;
			case DEVIID: {
//				if (!strcmp(rule->content[i].field, "src iid")) {
//
//					schc_ipaddr_t node_ip;
//					get_node_ip(node_ip);
//
//					unsigned char ip_addr[8] = {
//							(node_ip[4] & 0xFF),
//							(node_ip[4] & 0xFF00) >> 8,
//							(node_ip[5] & 0xFF),
//							(node_ip[5] & 0xFF00) >> 8,
//							(node_ip[6] & 0xFF),
//							(node_ip[6] & 0xFF00) >> 8,
//							(node_ip[7] & 0xFF),
//							(node_ip[7] & 0xFF00) >> 8,
//					};
//
//					for (j = 0; j < field_length; j++) {
//						schc_header[index + j] = ip_addr[j];
//					}
//				}
			} break;
			case APPIID: {
				// build iid from L2 server address
			} break;
			}

			dst->offset += field_length;
		}
	}

	return 1;
}

/**
 * Find a matching rule for a layer
 *
 * @param schc_bitarray the bit array as received from the network
 * 						note: a conversion is required for CoAP to decode the options
 *
 * @param device_id		the device to find an IP rule for
 * @param schc_layer	the layer for which to find a rule for
 *
 * @return the rule
 *         NULL if no rule is found
 */
static struct schc_layer_rule_t* schc_find_rule_from_header(
		schc_bitarray_t* src, uint32_t device_id, schc_layer_t layer, direction DI) {
	uint8_t i = 0;
	// set to 0 when a rule doesn't match
	uint8_t rule_is_found = 1; uint8_t max_layer_fields = 0; uint32_t prev_offset = src->offset;

	struct schc_device *device = get_device_by_id(device_id);
	if (device == NULL) {
		printf(
				"schc_find_rule_from_header(): no device was found for this id");
		return 0;
	}

	for (i = 0; i < device->rule_count; i++) {
		struct schc_layer_rule_t* curr_rule = NULL;
		if(layer == SCHC_IPV6) {
			max_layer_fields = IPV6_FIELDS;
			curr_rule = (struct schc_layer_rule_t*) (*device->context)[i]->compression_rule->ipv6_rule;
		} 
#if USE_UDP
		else if(layer == SCHC_UDP) {
			max_layer_fields = UDP_FIELDS;
			curr_rule = (struct schc_layer_rule_t*) (*device->context)[i]->compression_rule->udp_rule;
		}
#endif
		else {
			printf(
					"schc_find_rule_from_header(): no layer specified \r\n");
			return NULL;
		}

		uint8_t j = 0; uint8_t k = 0;
		uint8_t dir_length = (DI == UP) ? curr_rule->up : curr_rule->down;

		while (j < dir_length) {
			// exclude fields in other direction
			if ((curr_rule->content[k].dir == BI) || (curr_rule->content[k].dir == DI)) {
				uint8_t src_pos = 0;
				if(src->offset >= 8)
					src_pos = get_number_of_bytes_from_bits(src->offset);
				if (!curr_rule->content[k].MO(&curr_rule->content[k],
						(uint8_t*) (src->ptr + src_pos), (src->offset % 8))) { // compare header field and rule field using the matching operator
					rule_is_found = 0;
					printf(
							"schc_find_rule_from_header(): skipped rule %d due to %s \r\n", (*device->context)[i]->id[0], curr_rule->content[k].field);
					src->offset = prev_offset; // reset offset
					break;
				} else {
					rule_is_found = 1;
					src->offset += curr_rule->content[k].field_length;
				}
				j++;
			}
			k++; // increment to skip other directions
			if(k > max_layer_fields) { // todo coap <-> ipv6
				printf("schc_find_rule_from_header(): more fields present than LAYER_FIELDS \r\n");
				return NULL;
			}
		}

		if (rule_is_found) {
			return (struct schc_layer_rule_t*) (curr_rule);
		}
	}

	return NULL;
}

#if USE_IPv6

/**
 * Swaps the IPv6 source and destination
 *
 * @param ptr	pointer to the ipv6 header
 *
 */
static void swap_ipv6_source_and_destination(uint8_t* ptr) {
	schc_ip6addr_t tmp_addr;
	memcpy(tmp_addr, (uint8_t*) (ptr + 8), 16);
	memmove((uint8_t*) (ptr + 8), (uint8_t*) (ptr + 24), 16);
	memcpy((uint8_t*) (ptr + 24), tmp_addr, 16);
}
#endif

/**
 * The equal matching operator
 *
 * @param target_field 	the field from the rule
 * @param field_value 	the value from the header to compare with the rule value
 * @param field_offset	the offset (in bits), starting from the field value pointer
 *
 * @return 1 if the target field matches the field value
 *         0 if the target field doesn't match the field value
 *
 */
static uint8_t equal(struct schc_field* target_field, unsigned char* field_value, uint16_t field_offset) {
	uint8_t bit_pos = get_position_in_first_byte(target_field->field_length);

	return compare_bits_aligned((uint8_t*) (target_field->target_value), bit_pos,
			(uint8_t*) (field_value), field_offset, target_field->field_length);
}

/**
 * The ignore matching operator
 *
 * @param target_field the field from the rule
 * @param field_value the value from the header to compare with the rule value
 *
 * @return 1
 *
 */
static uint8_t ignore(struct schc_field* target_field, unsigned char* field_value, uint16_t field_offset){
	// ignore, always true
	return 1;
}

/**
 * The MSB matching operator
 * MSB(x): 	a match is obtained if the most significant (leftmost) x
 *    		bits of the packet header field value are equal to the TV in the
 *			Rule.  The x parameter of the MSB MO indicates how many bits are
 * 	 	 	involved in the comparison.  If the FL is described as variable,
 *     	  	the x parameter must be a multiple of the FL unit.  For example, x
 *			must be multiple of 8 if the unit of the variable length is bytes.
 *
 * @param target_field the field from the rule
 * @param field_value the value from the header to compare with the rule value
 *
 * @return 1 if the MSB of the target field matches the MSB of the field value
 *         0 if the MSB of the target field doesn't match the MSB of the field value
 *
 */
static uint8_t MSB(struct schc_field* target_field, unsigned char* field_value, uint16_t field_offset){
	if(compare_bits(target_field->target_value, field_value, target_field->MO_param_length)) {
		return 1; // left x bits match the target value
	}

	return 0;
}


/**
 * The match-map matching operator
 * match-mapping: 	With match-mapping, the Target Value is a list of
 * 					values.  Each value of the list is identified by an index.
 *					Compression is achieved by sending the index instead of the
 *					original header field value.
 *
 * @param target_field the field from the rule
 * @param field_value the value from the header to compare with the rule value
 *
 * @return 1 if the the field value is equal to one of the values found in the mapping array
 *         0 if no matching value is found in the mapping array
 *
 */
static uint8_t matchmap(struct schc_field* target_field, unsigned char* field_value, uint16_t field_offset){
	uint8_t i;

	uint8_t result; uint8_t match_counter = 0;
	result = 0;

	// if result is 0,
	if (result == 0) {
		uint16_t list_len = get_required_number_of_bits(
				target_field->MO_param_length);
		for (i = 0; i < target_field->MO_param_length; i++) {
			uint8_t ptr = i;
			if (! (target_field->field_length % 8) ) // only support byte aligned matchmap
				ptr = i * get_number_of_bytes_from_bits(target_field->field_length);

			if (compare_bits_BIG_END(field_value,
					(uint8_t*) (target_field->target_value + ptr),
					target_field->field_length)) {
				return 1;
			}
		}
	} else {
	}

	// target value doesn't match with any field value
	return 0;
}

/**
 * Notifies the compressor about the node its IP address
 *
 * @param node_ip pointer to the ip address array
 *
 * @return 0
 *
 */
static void set_node_ip(schc_ipaddr_t *node_ip) {
	memcpy(node_ip_6, node_ip, sizeof(schc_ipaddr_t));
}

////////////////////////////////////////////////////////////////////////////////////
//                               GLOBAL FUNCIONS                                  //
////////////////////////////////////////////////////////////////////////////////////


/**
 * Initializes the SCHC compressor
 *
 * @param node_ip 		a pointer to the source it's ip address
 *
 * @return error 		error codes on error
 *
 */
uint8_t schc_compressor_init(uint8_t src[16]) {
	set_node_ip(src);

	return 1;
}

/**
 * Compresses a CoAP/UDP/IP packet
 *
 * @param 	data 			pointer to the original packet
 * @param 	total_length 	the length of the packet
 * @param 	dst				pointer to the bit array object, where the compressed packet will
 * 							be stored. Can later be passed to fragmenter
 * @param 	device_id		the device id to find a rule for
 * @param 	direction		the direction of the flow
 * 							UP: LPWAN to IPv6 or DOWN: IPv6 to LPWAN
 * @param	device_type		the device type: NETWORK_GATEWAY or DEVICE
 * @param	schc_rule		a pointer to a schc rule struct to return the rule that was found
 *
 * @return 	length			the length of the compressed packet
 *         	-1 				on a memory overflow
 *
 * @note 	the compressor will only look for rules configured with the
 * 			NOT_FRAGMENTED reliability mode
 */

struct schc_rule_t* schc_compress(uint8_t *data, uint16_t total_length,
		schc_bitarray_t* dst, uint32_t device_id, direction dir) {
	struct schc_rule_t* schc_rule;
	uint16_t coap_length = 0; uint16_t src_hdr_length = 0;
	uint8_t coap_rule_id = 0; uint8_t udp_rule_id = 0; uint8_t ipv6_rule_id = 0;

	dst->offset = RULE_SIZE_BITS;
	memset(dst->ptr, 0, total_length + RULE_SIZE_BYTES); // buf should at least be packet length + RULE_SIZE_BYTES
	schc_bitarray_t src; src.ptr = data; src.offset = 0; // use bit array for comparison

	/*
	 * **** look for a matching rule ****
	*/

	LOG_DBG_("+---------------------------------+\r\n");
	LOG_DBG_("|    SCHC uncompressed Packet     |\r\n");
	LOG_DBG_("+---------------------------------+\r\n");

	int ind;
	for(ind = 0; ind <  total_length; ind++) {
		LOG_DBG_("%02X ", data[ind]);
		if(!((ind + 1) % 12)) {
			LOG_DBG_("\r\n");
		}
	}
	LOG_DBG_("\r\n");

#if USE_IPv6
	struct schc_ipv6_rule_t *ipv6_rule;
	if(dir == DOWN) {
		swap_ipv6_source_and_destination(src.ptr);
	}
	ipv6_rule = (struct schc_ipv6_rule_t*) schc_find_rule_from_header(&src, device_id, SCHC_IPV6, dir);
	if(ipv6_rule != NULL) {
		ipv6_rule_id = ipv6_rule->rule_id;
	}
#endif
#if USE_UDP
	struct schc_udp_rule_t *udp_rule;
	udp_rule = (struct schc_udp_rule_t*) schc_find_rule_from_header(&src, device_id, SCHC_UDP, dir);
	if(udp_rule != NULL) {
		udp_rule_id = udp_rule->rule_id;
	}
#endif

	printf("schc_compress(): IPv6 rule: %d, UDP rule: %d, CoAP rule: %d \r\n", ipv6_rule_id, udp_rule_id, coap_rule_id);
	src.offset = 0; // reset the bit arrays offset and start compressing

	// find the rule for this device by combining the available id's // todo pointers?
	uint8_t id_pos = get_position_in_first_byte(RULE_SIZE_BITS);
	schc_rule = get_schc_rule_by_layer_ids(ipv6_rule_id,
			udp_rule_id, coap_rule_id, device_id, NOT_FRAGMENTED);

	set_rule_id(schc_rule, dst->ptr); // set rule id

	if(schc_rule == NULL) { // no rule was found
		printf("schc_compress(): no rule was found \r\n");
#if USE_IPv6
		copy_bits(dst->ptr, dst->offset, data, 0, BYTES_TO_BITS(IP6_HLEN));
		dst->offset += BYTES_TO_BITS(IP6_HLEN);
		src_hdr_length += IP6_HLEN;
#endif
#if USE_UDP
		copy_bits(dst->ptr, dst->offset, data, BYTES_TO_BITS(IP6_HLEN), BYTES_TO_BITS(UDP_HLEN));
		dst->offset += BYTES_TO_BITS(UDP_HLEN);
		src_hdr_length += UDP_HLEN;
#endif
	}
	else { // a rule was found - compress
#if USE_IPv6
		compress(dst, &src, (const struct schc_layer_rule_t*) ipv6_rule, dir);
		src_hdr_length += IP6_HLEN;
#endif
#if USE_UDP
		compress(dst, &src, (const struct schc_layer_rule_t*) udp_rule, dir);
		src_hdr_length += UDP_HLEN;
#endif
	}

	uint16_t payload_len = (total_length - src_hdr_length - coap_length); // copy the payload
	const uint8_t* payload_ptr = (data + src_hdr_length + coap_length);

	copy_bits(dst->ptr, dst->offset, payload_ptr, 0, BYTES_TO_BITS(payload_len));
    uint16_t new_pkt_length = (BITS_TO_BYTES(dst->offset) + payload_len);
    dst->padding = padded(dst);


	LOG_DBG_("\r\n");
	LOG_DBG_(
			"schc_compress(): compressed header length: %d bits (%dB), payload length: %d (total length: %d) / %d b\r\n",
			dst->offset, BITS_TO_BYTES(dst->offset), payload_len, new_pkt_length, dst->offset + BYTES_TO_BITS(payload_len) + dst->padding);
	LOG_DBG_("+---------------------------------+\r\n");
	LOG_DBG_("|          SCHC Packet            |\r\n");
	LOG_DBG_("+---------------------------------+\r\n");

	int i;
	for(i = 0; i <  new_pkt_length; i++) {
		LOG_DBG_("%02X ", dst->ptr[i]);
		if(!((i + 1) % 12)) {
			LOG_DBG_("\r\n");
		}
	}
	LOG_DBG_("\r\n");

	dst->len = new_pkt_length; // set the compressed packet length

	return schc_rule; // return the schc rule
}

/**
 * Set the packet length for the UDP and IP headers
 *
 * @param data 			pointer to the data packet
 * @param data_len 		the length of the total packet
 *
 * @return 0
 *
 */
static uint16_t compute_length(unsigned char *data, uint16_t data_len) {
	// if the length fields are set to 0
	// the length must be calculated
	uint8_t* packet_ptr = (uint8_t*) data;
#if USE_IPv6
	if(packet_ptr[4] == 0 && packet_ptr[5] == 0) {
		// ip length
		packet_ptr[4] = (((data_len - IP6_HLEN) & 0xFF00) >> 8);
		packet_ptr[5] = ((data_len - IP6_HLEN) & 0xFF);
	}
#endif
#if USE_UDP
	if(packet_ptr[44] == 0 && packet_ptr[45] == 0) {
		// udp length
		packet_ptr[44] = (((data_len - IP6_HLEN) & 0xFF00) >> 8);
		packet_ptr[45] = ((data_len - IP6_HLEN) & 0xFF);
	}
#endif

	return 0;
}

static uint16_t chksum(uint16_t sum, const uint8_t *data, uint16_t len) {
	uint16_t t;
	const uint8_t *dataptr;
	const uint8_t *last_byte;

	dataptr = data;
	last_byte = data + len - 1;

	while (dataptr < last_byte) { /* At least two more bytes */
		t = (dataptr[0] << 8) + dataptr[1];
		sum += t;
		if (sum < t) {
			sum++; /* carry */
		}
		dataptr += 2;
	}

	if (dataptr == last_byte) {
		t = (dataptr[0] << 8) + 0;
		sum += t;
		if (sum < t) {
			sum++; // carry
		}
	}

	// return sum in host byte order
	return sum;
}

/**
 * Calculates the UDP checksum and sets the appropriate header fields
 *
 * @param data pointer to the data packet
 *
 * @return checksum the computed checksum
 *
 */
uint16_t compute_checksum(unsigned char *data) {
	// if the checksum fields are set to 0
	// the checksum must be calculated
#if USE_UDP
	if(data[46] == 0 && data[47] == 0) {
		uint16_t upper_layer_len; uint16_t sum; uint16_t result;

		upper_layer_len = (((uint16_t)(data[44]) << 8) + data[45]);

		// protocol (17 for UDP) and length fields. This addition cannot carry.
		uint8_t proto = data[6];
		sum = upper_layer_len + proto;

		// sum IP source and destination
		sum = chksum(sum, (uint8_t *)&data[8], 2 * sizeof(schc_ipaddr_t));

		// sum upper layer headers and data
		sum = chksum(sum, &data[IP6_HLEN], upper_layer_len);

		result = (~sum);

		data[46] = (uint8_t) ((result & 0xFF00) >> 8);
		data[47] = (uint8_t) (result & 0xFF);

		return 1;
	}
#endif

	return 0;
}

/**
 * Construct the header from the layered set of rules
 *
 * @param 	data 				pointer to the received data
 * @param 	buf	 				pointer where to save the decompressed packet
 * @param 	device_id 			the device its id
 * @param 	total_length 		the total length of the received data
 * @param 	direction			the direction of the flow (UP: LPWAN to IPv6, DOWN: IPv6 to LPWAN)
 *
 * @return 	length 				length of the newly constructed packet
 * 			0 					the rule was not found
 */
uint16_t schc_decompress(schc_bitarray_t* bit_arr, uint8_t *buf,
		uint32_t device_id, uint16_t total_length, direction dir) {
	uint8_t coap_rule_id = 0; uint8_t udp_rule_id = 0; uint8_t ipv6_rule_id = 0;
	uint8_t buf_hdr_length = 0;

	// prepare output buffer
	bit_arr->offset = RULE_SIZE_BITS;

	struct schc_rule_t *rule = get_schc_rule_by_rule_id(bit_arr->ptr, device_id);
	if(rule != NULL) {
#if USE_UDP
		udp_rule_id = rule->compression_rule->udp_rule->rule_id;
#endif
#if USE_IPv6
		ipv6_rule_id = rule->compression_rule->ipv6_rule->rule_id;
#endif
	}

	printf("\r\n");
	printf(
			"schc_decompress(): IPv6 rule: %d, UDP rule id: %d, CoAP rule id: %d \r\n", ipv6_rule_id, udp_rule_id, coap_rule_id);

	LOG_DBG_("\r\n");
	LOG_DBG_("+---------------------------------+\r\n");
	LOG_DBG_("|        Recv Packet          |\r\n");
	LOG_DBG_("+---------------------------------+\r\n");

	int ind;
	for (ind = 0; ind < total_length; ind++) {
		LOG_DBG_("%02X ", bit_arr->ptr[ind]);
		if (!((ind + 1) % 12)) {
			LOG_DBG_("\r\n");
		}
	}
	LOG_DBG_("\r\n");

	uint8_t ret = 0;
	uint8_t coap_offset = 0;

	if (compare_bits(bit_arr->ptr, UNCOMPRESSED_ID, RULE_SIZE_BITS)) { // uncompressed packet, copy uncompressed headers
#if USE_IPv6
		copy_bits(buf, 0, bit_arr->ptr, RULE_SIZE_BITS, BYTES_TO_BITS(IP6_HLEN));
		bit_arr->offset += BYTES_TO_BITS(IP6_HLEN);
		buf_hdr_length += IP6_HLEN;
#endif
#if USE_UDP
		copy_bits(buf, BYTES_TO_BITS(IP6_HLEN), bit_arr->ptr, bit_arr->offset, BYTES_TO_BITS(UDP_HLEN));
		bit_arr->offset += BYTES_TO_BITS(UDP_HLEN);
		buf_hdr_length += UDP_HLEN;
#endif
	} else { // compressed packet, decompress with residue and rule
		bit_arr->offset = RULE_SIZE_BITS;

		schc_bitarray_t dst_arr;
		dst_arr.ptr = buf;
		dst_arr.offset = 0;

#if USE_IPv6
		if (ipv6_rule_id != 0) {
			ret = decompress((struct schc_layer_rule_t *) rule->compression_rule->ipv6_rule, bit_arr, &dst_arr, dir);
			if (ret == 0) {
				return 0; // no rule was found
			}
			buf_hdr_length += IP6_HLEN;
		}

#endif
#if USE_UDP
		if (udp_rule_id != 0) {
			ret = decompress((struct schc_layer_rule_t *) (rule->compression_rule->udp_rule), bit_arr, &dst_arr, dir);
			if (ret == 0) {
				return 0; // no rule was found
			}
			buf_hdr_length += UDP_HLEN;
		}
#endif
	}

	uint8_t new_header_length = buf_hdr_length + coap_offset;
	uint16_t payload_bit_length = BYTES_TO_BITS(total_length) - bit_arr->offset; // the schc header minus the total length is the payload length

	copy_bits(buf, BYTES_TO_BITS(new_header_length), bit_arr->ptr, bit_arr->offset, payload_bit_length);
	uint16_t payload_length = get_number_of_bytes_from_bits(payload_bit_length);

#if USE_IPv6
	if (dir == DOWN) {
		swap_ipv6_source_and_destination(buf);
	}
#endif

	if(padded(bit_arr)) { // remove padding
		payload_length--;
	}

	// compute_length(buf, (payload_length + new_header_length)); // set udp and ipv6 length
	// compute_checksum(buf);

	LOG_DBG("schc_decompress(): header length: %d, payload length %d \r\n", new_header_length, payload_length);

	LOG_DBG_("\r\n");
	LOG_DBG_("+---------------------------------+\r\n");
	LOG_DBG_("|        Original Packet          |\r\n");
	LOG_DBG_("+---------------------------------+\r\n");

	int i;
	for (i = 0; i < new_header_length + payload_length; i++) {
		LOG_DBG_("%02X ", buf[i]);
		if (!((i + 1) % 12)) {
			LOG_DBG_("\r\n");
		}
	}

	LOG_DBG_("\n\r\n");

	return new_header_length + payload_length;
}

