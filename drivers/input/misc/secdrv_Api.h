/*
 * Copyright (c) 2013 TRUSTONIC LIMITED
 * All rights reserved
 *
 * The present software is the confidential and proprietary information of
 * TRUSTONIC LIMITED. You shall not disclose the present software and shall
 * use it only in accordance with the terms of the license agreement you
 * entered into with TRUSTONIC LIMITED. This software may be subject to
 * export or import laws in certain countries.
 */

/**
 * @file   secdrv_api.h
 * @brief  Contains DCI command definitions and data structures
 *
 */

#ifndef __SECDRVAPI_H__
#define __SECDRVAPI_H__

#include "dci.h"


/**
 * Command ID's
  */
#define SECDEV_CMD_TESTCASE		1
#define SECDEV_CMD_READ_CHIP_ID	2
#define SECDEV_CMD_RW_ADDR		3
#define SECDEV_CMD_FORWARD_INT	4
#define SECDEV_CMD_SWITCH_SPI  5
#define SECDEV_CMD_RESET_SAMPLE  6
/*... add more command ids when needed */

/**
 * Maximum data length.
 */
 //the longest fp data each time?, need change later according the command
#define MAX_DATA_LEN (2048+5 + 4 /*GF66XX_RDATA_OFFSET*/)
//#define MAX_DATA_LEN (249) /*GF66XX_CFG_LEN*/;

/**
 * command message.
 *
 * @param len Lenght of the data to process.
 * @param data Data to be processed
 */
typedef struct {
    dciCommandHeader_t  header;     /**< Command header */
    uint32_t            len;        /**< Length of data to process */
} cmd_t;


/**
 * Response structure
 */
typedef struct {
    dciResponseHeader_t header;     /**< Response header */
    uint32_t            len;
} rsp_t;


/**
 * SECDEV_CMD_TESTCASE data structure
 */
typedef struct {
    uint32_t len;
    uint8_t data[64];
} testcase_t;


/**
 * SECDEV_CMD_READ_CHIP_ID data structure
 */
typedef struct {
    uint32_t len;
    uint16_t addr;
    uint8_t chipid[10];
} readchipID_t;


/**
 * SECDEV_CMD_RW_ADDR data structure
 */
typedef struct {
    uint32_t len;
    uint16_t direction;  //0 for read, 1 for write
    uint16_t addr;
    uint8_t data[MAX_DATA_LEN];
} rw_data_t;

/**
 * SECDEV_CMD_RW_ADDR data structure
 */
typedef struct {
    uint8_t event_type;   //0-invalid, 1-keydown, 2-keyon, 3-fp data ready
    uint8_t mode;
    uint8_t status;
    uint8_t notify_application;    //whether notify this interrupt to high layer
} forward_int_t;

/**
 * SECDEV_CMD_RW_ADDR data structure
 */
typedef struct {
    uint8_t to_secure;   //0-to non_secure world, 1-to secure world
} switch_spi_t;

/**
 * SECDEV_CMD_RESET_SAMPLE data structure
 */
typedef struct {
    uint8_t do_reset_sample;   //0-to non_secure world, 1-to secure world
} reset_sample_t;

/**
 * DCI message data, 1MB maximum.
 * ONLY USED BY ONE CLIENT
 */
typedef struct {
    union {
        cmd_t     command;
        rsp_t     response;
    };

    //TODO: union struct for different command, test temporary
    union {
        testcase_t  testcase;
        readchipID_t  readchipid;
        rw_data_t rw_data;
        forward_int_t forward_int;
        switch_spi_t switch_spi;
        reset_sample_t reset_sample;
    };
} dciMessage_t, *dciMessage_ptr;

/**
 * Trustlet UUID
 */
//#define TL_SECSPI_UUID  { { 7, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }
#define SECDRV_SPI_UUID  { { 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }

#endif // __SECDRVAPI_H__
