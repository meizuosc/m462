/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

struct ccinethdr{
	__be16 iplen;
	__u8 reserved;
	__u8 offset_len;
	__u32 cid;
} __attribute__((packed));


int data_rx(const unsigned char* packet, int len, unsigned long cid);
/*
 * calculate padding size
 *
 */

#define DATA_ALIGN_SIZE 8
#define MAX_CID_NUM    11
#define TX_BUFFER_LEN (16*1000)

static inline unsigned tx_padding_size(unsigned len)
{
	return (~len + 1) & (DATA_ALIGN_SIZE - 1);
	//return DATA_ALIGN_SIZE - (len & (DATA_ALIGN_SIZE - 1));
}

static inline unsigned rx_padding_size(unsigned len)
{
	return (~len + 1) & (DATA_ALIGN_SIZE - 1);
	// return DATA_ALIGN_SIZE - (len & (DATA_ALIGN_SIZE - 1));
}



