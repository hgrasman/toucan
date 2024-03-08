/*
Fuctions implementing packet counter update functions and CRC as defined by Valeo Documentation

For EV Cartz, Kettering University

CONFIDENTIAL? It's all according to open standards but best not to share it.

-Henry Grasman

*/

#include "arduino.h"
#include "Valeo_ext.h"


uint8_t crcTable[256] = {
(0x00), (0x1d), (0x3a), (0x27), (0x74),
(0x69), (0x4e), (0x53), (0xe8), (0xf5),
(0xd2), (0xcf), (0x9c), (0x81), (0xa6),
(0xbb), (0xcd), (0xd0), (0xf7), (0xea),
(0xb9), (0xa4), (0x83), (0x9e), (0x25),
(0x38), (0x1f), (0x02), (0x51), (0x4c),
(0x6b), (0x76), (0x87), (0x9a), (0xbd),
(0xa0), (0xf3), (0xee), (0xc9), (0xd4),
(0x6f), (0x72), (0x55), (0x48), (0x1b),
(0x06), (0x21), (0x3c), (0x4a), (0x57),
(0x70), (0x6d), (0x3e), (0x23), (0x04),
(0x19), (0xa2), (0xbf), (0x98), (0x85),
(0xd6), (0xcb), (0xec), (0xf1), (0x13),
(0x0e), (0x29), (0x34), (0x67), (0x7a),
(0x5d), (0x40), (0xfb), (0xe6), (0xc1),
(0xdc), (0x8f), (0x92), (0xb5), (0xa8),
(0xde), (0xc3), (0xe4), (0xf9), (0xaa),
(0xb7), (0x90), (0x8d), (0x36), (0x2b),
(0x0c), (0x11), (0x42), (0x5f), (0x78),
(0x65), (0x94), (0x89), (0xae), (0xb3),
(0xe0), (0xfd), (0xda), (0xc7), (0x7c),
(0x61), (0x46), (0x5b), (0x08), (0x15),
(0x32), (0x2f), (0x59), (0x44), (0x63),
(0x7e), (0x2d), (0x30), (0x17), (0x0a),
(0xb1), (0xac), (0x8b), (0x96), (0xc5),
(0xd8), (0xff), (0xe2), (0x26), (0x3b),
(0x1c), (0x01), (0x52), (0x4f), (0x68),
(0x75), (0xce), (0xd3), (0xf4), (0xe9),
(0xba), (0xa7), (0x80), (0x9d), (0xeb),
(0xf6), (0xd1), (0xcc), (0x9f), (0x82),
(0xa5), (0xb8), (0x03), (0x1e), (0x39),
(0x24), (0x77), (0x6a), (0x4d), (0x50),
(0xa1), (0xbc), (0x9b), (0x86), (0xd5),
(0xc8), (0xef), (0xf2), (0x49), (0x54),
(0x73), (0x6e), (0x3d), (0x20), (0x07),
(0x1a), (0x6c), (0x71), (0x56), (0x4b),
(0x18), (0x05), (0x22), (0x3f), (0x84),
(0x99), (0xbe), (0xa3), (0xf0), (0xed),
(0xca), (0xd7), (0x35), (0x28), (0x0f),
(0x12), (0x41), (0x5c), (0x7b), (0x66),
(0xdd), (0xc0), (0xe7), (0xfa), (0xa9),
(0xb4), (0x93), (0x8e), (0xf8), (0xe5),
(0xc2), (0xdf), (0x8c), (0x91), (0xb6),
(0xab), (0x10), (0x0d), (0x2a), (0x37),
(0x64), (0x79), (0x5e), (0x43), (0xb2),
(0xaf), (0x88), (0x95), (0xc6), (0xdb),
(0xfc), (0xe1), (0x5a), (0x47), (0x60),
(0x7d), (0x2e), (0x33), (0x14), (0x09),
(0x7f), (0x62), (0x45), (0x58), (0x0b),
(0x16), (0x31), (0x2c), (0x97), (0x8a),
(0xad), (0xb0), (0xe3), (0xfe), (0xd9),
(0xc4)
};


// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

#define CNT_MAX_VALUE 0x0F

/* Compute counter to send in frame */
uint8_t ComputeCounter(uint8_t Counter)
{
	if (Counter < CNT_MAX_VALUE) {
		Counter++;
	}
	else {
		Counter = 0;
	}
	return (Counter);
}


/* Check counter to rx from frame */
uint8_t CheckCounter(uint8_t u8CounterPrev, uint8_t u8CounterToCks)
{
	uint8_t bCksResultLoc;
	uint8_t u8CounterLoc;
	uint8_t u8ResultLoc;
	u8CounterLoc = u8CounterPrev;
	if (u8CounterPrev != u8CounterToCks)
	{
		if (u8CounterPrev < CNT_MAX_VALUE)
		{
			u8CounterLoc++;
		}
		else
		{
			u8CounterLoc = 0;
		}
		bCksResultLoc = 1; // TRUE
	}
	else
	{
		bCksResultLoc = 0; // FALSE
	}
	/* new counter in 4 bits and result check counter in 1 bit */
	u8ResultLoc = ((u8CounterLoc & 0x0F) |
		((bCksResultLoc & 0x01) << 4));
	return u8ResultLoc;
}


#define CRC8_INITIAL_VALUE 0xFF
#define CRC8_RESULT_WIDTH 8
/*
* @brief : Calculation of SAE-J1850 CRC8
* P(x) = x^4 + x^3 + x^2 + 1 -> 0x1D
* AUTOSAR interface for CRC Calculation of SAE-J1850 CRC8
*/
uint32_t ComputeCrc(uint8_t* MsgData, uint8_t CrcLength)
{
	uint8_t charData;
	uint8_t Idx;
	uint32_t crcValue; /* Init value - dword : uint32 */
	/*
	* Divide the message by the polynomial, a char at a time.
	*/
	crcValue = CRC8_INITIAL_VALUE;
	for (charData = 0; charData < CrcLength; ++charData)
	{
		Idx = (MsgData[charData] ^ crcValue) & (unsigned long)0x00FF;
		crcValue = crcTable[Idx] ^ (crcValue << CRC8_RESULT_WIDTH);
	}
	/*
	* The final remainder is the CRC.
	*/
	return ~(crcValue);
}


uint16_t ArrangeFHybrid(
	uint8_t* dst_p,
	struct x8578_can_db_client_pcm_pmz_f_hybrid_t* src_p,
	uint16_t size)
{
	if (size < 7) {
		return(-1);
	}

	//arrange according to crc documentation
	dst_p[0] = ((src_p->em_torque_request_ext) >> 7) & 0xFF;
	dst_p[1] = ((src_p->em_speed_request_ext) << 1) & 0xFE;
	dst_p[2] = ((src_p->em_speed_request_ext) >> 7) & 0xFF;
	dst_p[3] = ((src_p->em_torque_request_ext) << 1) & 0xFE;
	dst_p[4] = ((src_p->em_voltage_dc_link_req_ext) >> 2) & 0xFF;
	dst_p[5] = (((src_p->em_voltage_dc_link_req_ext) << 8) & 0xA0) | ((src_p->em_operating_mode_req_ext) & 0x0F);
	dst_p[6] = ((src_p->vscem_req_gp_counter) << 4) & 0xF0;

	return (7);


}


uint16_t ArrangeWMHEV(
	uint8_t* dst_p,
	struct x8578_can_db_client_pcm_pmz_w_mhev_t* src_p,
	uint16_t size) {

	if (size < 3) {
		return(-1);
	}

	dst_p[0] = (((src_p->em_vol_dc_link_motor_limit >> 7) & 0x07) | ((src_p->em_motor_limit_grp_counter & 0x0F) << 3) | ((src_p->em_cur_dc_link_motor_limit & 0x01) << 7));
	dst_p[1] = ((src_p->em_cur_dc_link_motor_limit >> 1) & 0xFF);
	dst_p[2] = (src_p->em_vol_dc_link_motor_limit & 0x7F) << 1;

	return (3);
}

uint16_t ArrangeTMHEV(
	uint8_t* dst_p,
	struct x8578_can_db_client_pcm_pmz_t_mhev_t* src_p,
	uint16_t size) {

	if (size < 3) {
		return(-1);
	}

	dst_p[0] = (((src_p->em_vol_dc_link_gen_limit >> 7) & 0x07) | ((src_p->em_gen_limit_grp_counter & 0x0F) << 3) | ((src_p->em_cur_dc_link_gen_limit & 0x01) << 7));
	dst_p[1] = ((src_p->em_cur_dc_link_gen_limit >> 1) & 0xFF);
	dst_p[2] = ((src_p->em_vol_dc_link_gen_limit & 0x7F) << 1);

	return (3);
}

uint16_t ArrangeUMHEV(
	uint8_t* dst_p,
	struct x8578_can_db_client_pcm_pmz_u_mhev_t* src_p,
	uint16_t size) {

	if (size < 6) {
		return(-1);
	}

	dst_p[0] = (src_p->em_torque_min_limit & 0x00FF);
	dst_p[1] = (((src_p->em_torque_min_limit >> 8) & 0x03) | ((src_p->vscem_torq_lim_gp_counter & 0x0F) << 2) |((src_p->em_torque_max_limit & 0x03) << 6));
	dst_p[2] = ((src_p->em_torque_max_limit >> 2) & 0x00FF);
	dst_p[3] = ((src_p->drvline_damp_torq_max_lim >> 2) & 0x00FF);
	dst_p[4] = (((src_p->drvline_damp_torq_max_lim & 0x03) << 6) | ((src_p->drvline_damp_torq_min_lim >> 8) & 0x03));
	dst_p[5] = (src_p->drvline_damp_torq_min_lim & 0x00FF);

	return (6);
}

