/*
 * (c) 2018 - idlab - UGent - imec
 *
 * Bart Moons
 *
 * This file is part of the SCHC stack implementation
 *
 * Adjust this file to taste to include rules
 *
 */

#include "rules.h"

const struct schc_rule_t schc_rule_1 = { { 0x01 }, &compression_rule_1, NOT_FRAGMENTED, 1, 0, 0, 0 };
const struct schc_rule_t schc_rule_2 = { { 0x02 }, &compression_rule_1, NO_ACK, 1, 0, 0, 0 };
const struct schc_rule_t schc_rule_3 = { { 0x03 }, &compression_rule_1, ACK_ON_ERROR, 3, 6, 1, 0 };
const struct schc_rule_t schc_rule_4 = { { 0x04 }, &compression_rule_1, ACK_ALWAYS, 2, 2, 1, 0 };
//note: FCN_SIZE must larger than 1

// todo #define
const uint8_t UNCOMPRESSED_ID[RULE_SIZE_BYTES] = { 0x00 }; // the rule id for an uncompressed packet
// todo
// const uint8_t UNCOMPRESSED_NO_ACK_ID[RULE_SIZE_BYTES] = { 0 };
// const uint8_t UNCOMPRESSED_ACK_ON_ERR[RULE_SIZE_BYTES] = { 0 };
// const uint8_t UNCOMPRESSED_ACK_ALWAYS[RULE_SIZE_BYTES] = { 0 };

/* save rules in flash */
const struct schc_rule_t* node1_schc_rules[] = { &schc_rule_1, &schc_rule_2,
		&schc_rule_3, &schc_rule_4 };

/* rules for a particular device */
const struct schc_device node1 = { 1, 4, &node1_schc_rules };
const struct schc_device node2 = { 2, 4, &node1_schc_rules };
#define DEVICE_COUNT 2

/* server keeps track of multiple devices: add devices to device list */
const struct schc_device* devices[DEVICE_COUNT] = { &node1, &node2 };
