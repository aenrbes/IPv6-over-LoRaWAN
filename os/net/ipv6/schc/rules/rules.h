#include "../schc.h"

#if USE_IPv6
const static struct schc_ipv6_rule_t ipv6_rule1 = {
	//	id, up, down, length
		1, 10, 10, 10,
		{
			//	field, 			   MO, len,	 pos,dir, 	val,			MO,			CDA
				{ "version", 		0, 4,	1, BI, 		{6},			&ignore, 	VALUESENT },
				{ "traffic class", 	0, 8,	1, BI, 		{0},			&ignore, 	VALUESENT },
				{ "flow label", 	0, 20,	1, BI, 		{0, 0, 0},		&ignore, 	VALUESENT },
				{ "length", 		0, 16,	1, BI, 		{0, 0},			&ignore, 	VALUESENT },
				{ "next header", 	0, 8, 	1, BI, 		{6, 17, 58},	&ignore, 	VALUESENT },
				{ "hop limit", 		0, 8, 	1, BI, 		{64}, 			&ignore, 	VALUESENT },
				{ "src prefix",		0, 64,	1, BI,		{0xFD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						&ignore, 	VALUESENT },
				{ "src iid",		0, 64,	1, BI, 		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
						&ignore, 	VALUESENT },
				{ "dest prefix",	0, 64,	1, BI,		{0xFD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						&ignore, 	VALUESENT },
				{ "dest iid",		0, 64,	1, BI, 		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						&ignore, 	VALUESENT },
		}
};
#endif

#if USE_UDP
const static struct schc_udp_rule_t udp_rule1 = {
		1, 4, 4, 4,
		{
				{ "src port", 		2,	16, 	 1, BI, 	{0x33, 0x16, 0x33, 0x17},
						&matchmap,	MAPPINGSENT }, // 5683 or 5684
				{ "dest port", 		2,	16, 	 1, BI, 	{0x33, 0x16, 0x33, 0x17},
						&matchmap,	MAPPINGSENT },
						// set field length to 16 to indicate 16 bit values
						// MO param to 2 to indicate 2 indices
				{ "length", 		0,	16, 	 1, BI, 	{0, 0},		 		&ignore,	COMPLENGTH },
				{ "checksum", 		0,	16, 	 1, BI, 	{0, 0},				&ignore,	COMPCHK },
		}
};
#endif

#if USE_COAP
// it is important to use strings, identical to the ones
// defined in coap.h for the options

// GET usage
const static struct schc_coap_rule_t coap_rule1 = {
		1, 9, 7, 9,
		{
				{ "version",		0,	2,	 1, BI,		{COAP_V1},		&equal,			NOTSENT },
				{ "type",			4,	2,	 1, BI,		{CT_CON, CT_NON, CT_ACK, CT_RST},
						&matchmap,	MAPPINGSENT	}, // todo: non word-aligned matchmap
				{ "token length",	0,	4,	 1, BI,		{4},			&equal,			NOTSENT },
				{ "code",			0,	8,	 1, BI,		{CC_PUT},		&equal,			NOTSENT },
				{ "message ID",		0,	16,	 1, BI,		{0x23, 0xBB},	&equal,			NOTSENT },
				{ "token",			24,	32,	 1, BI,		{0x21, 0xFA, 0x01, 0x00},
						&MSB,		LSB },
				{ "uri-path", 		0,	40,	 1, BI,		"usage", 		&equal,			NOTSENT },
				{ "no-response", 	0,	8,	 1, BI,		{0x1A}, 		&equal,			NOTSENT },
				{ "payload marker",	0,	8,   1, BI, 	{0xFF},			&equal,			NOTSENT }

		}
};
#endif

const struct schc_compression_rule_t compression_rule_1 = {
#if USE_IPv6
		&ipv6_rule1,
#endif
#if USE_UDP
		&udp_rule1,
#endif
#if USE_COAP
		&coap_rule1,
#endif
};
