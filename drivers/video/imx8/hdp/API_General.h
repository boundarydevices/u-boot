/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. THE SOFTWARE IS PROVIDED "AS IS",
 * WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Copyright 2017-2018 NXP
 *
 ******************************************************************************
 *
 * API_General.h
 *
 ******************************************************************************
 */

#ifndef API_GENERAL_H_
#define API_GENERAL_H_

#ifndef __UBOOT__
#include <stdint.h>
#else
#include <common.h>
#endif

#define GENERAL_TEST_ECHO_MAX_PAYLOAD 100
#define GENERAL_TEST_ECHO_MIN_PAYLOAD 1

/**
 * \addtogroup GENERAL_API
 * \{
 */
/** status code returned by API calls */
typedef enum {
    /** operation succedded */
	CDN_OK = 0,
    /** CEC operation succedded */
	CDN_CEC_ERR_NONE = 0,
    /** mailbox is currently sending or receiving data */
	CDN_BSY,
    /** message set up and ready to be sent, no data sent yet */
	CDN_STARTED,
    /** error encountered while reading/writing APB */
	CDN_ERR,
    /** reply returned with bad opcode */
	CDN_BAD_OPCODE,
    /** reply returned with bad module */
	CDN_BAD_MODULE,
    /** reply not supported mode */
	CDN_ERROR_NOT_SUPPORTED,
    /** Invalid argument passed to CEC API function */
	CDN_CEC_ERR_INVALID_ARG,
    /**
     * TX Buffer for CEC Messages is full. This is applicable only
     * when TX Buffers for CEC Messages are implemented in the HW.
     */
	CDN_CEC_ERR_TX_BUFF_FULL,
    /** No Messages in the RX Buffers are present. */
	CDN_CEC_ERR_RX_BUFF_EMPTY,
    /** Timeout during TX operation */
	CDN_CEC_ERR_TX_TIMEOUT,
    /** Timeout during RX operation */
	CDN_CEC_ERR_RX_TIMEOUT,
    /** Data transmision fail. */
	CDN_CEC_ERR_TX_FAILED,
    /** Data reception fail. */
	CDN_CEC_ERR_RX_FAILED,
    /** Operation aborted. */
	CDN_CEC_ERR_ABORT,
    /** All Logical Addresses are in use. */
	CDN_CEC_ERR_ALL_LA_IN_USE,
} CDN_API_STATUS;

typedef enum {
	CDN_BUS_TYPE_APB = 0,
	CDN_BUS_TYPE_SAPB = 1
} CDN_BUS_TYPE;

/**
 * GENERAL_Read_Register response struct
 */
typedef struct {
	unsigned int addr;
	unsigned int val;
} GENERAL_READ_REGISTER_RESPONSE;

/**
 * \brief set up API, must be called before any other API call
 */
void cdn_api_init(void);

/**
 * \brief Loads firmware
 *
 * \param iMem - pointer to instruction memory
 * \param imemSize - size of instruction memory buffer
 * \param dMem - pointer to data memory
 * \param dmemSize - size of data memory buffer
 * \return 0 if success, 1 if apb error encountered, 2 if CPU
 *         isn't alive after loading firmware
 *
 * This function does not require initialisation by #CDN_API_Init
 */

CDN_API_STATUS cdn_api_loadfirmware(unsigned char *imem,
				    int imemsize,
				    unsigned char *dmem, int dmemsize);
/**
 * \brief debug echo command for APB
 * \param val - value to echo
 * \return status
 *
 * will return #CDN_ERROR if reply message doesn't match request
 */
CDN_API_STATUS cdn_api_general_test_echo(unsigned int val,
					 CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_General_Test_Echo
 */
CDN_API_STATUS cdn_api_general_test_echo_blocking(unsigned int val,
						  CDN_BUS_TYPE bus_type);

/**
 * \brief Extended Echo test for mailbox.
 *
 * This test will send msg buffer to firmware's mailbox and
 * receive it back to the resp buffer. Received data will be
 * check against data sent and status will be returned as well
 * as received data.
 *
 * \param msg - Pointer to a buffer to send.
 * \param resp - Pointer to buffer for receiving msg payload back.
 * \param num_bytes - Number of bytes to send and receive.
 * \param bus_type Bus type.
 * \return status
 *
 * will return #CDN_ERROR if reply message doesn't match request or if
 *  arguments are invalid.
 */
CDN_API_STATUS cdn_api_general_test_echo_ext(uint8_t const *msg, uint8_t *resp,
					     uint16_t num_bytes,
					     CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_General_Test_Echo_Ext
 */
CDN_API_STATUS cdn_api_general_test_echo_ext_blocking(uint8_t const *msg,
						      uint8_t *resp,
						      uint16_t num_bytes,
						      CDN_BUS_TYPE bus_type);

/**
 * \brief get current version
 * \param [out] ver - fw version
 * \param [out] libver - lib version
 * \return status
 *
 * this fucntion does not require #CDN_API_Init
 */
CDN_API_STATUS cdn_api_general_getcurversion(unsigned short *ver,
					     unsigned short *verlib);

/**
 * \brief read event value
 * \param [out] event - pointer to store 32-bit events value
 * \return status
 *
 * this function does not require #CDN_API_Init
 */
CDN_API_STATUS cdn_api_get_event(uint32_t *events);

/**
 * \brief read debug register value
 * \param [out] val - pointer to store 16-bit debug reg value
 * \return status
 *
 * this function does not require #CDN_API_Init
 */
CDN_API_STATUS cdn_api_get_debug_reg_val(uint16_t *val);

/**
 * \brief check if KEEP_ALIVE register changed
 * \return #CDN_BSY if KEEP_ALIVE not changed, #CDN_OK if changed and #CDN_ERR
 * if error occured while reading
 */
CDN_API_STATUS cdn_api_checkalive(void);

/**
 * \breif blocking version of #CDN_API_CheckAlive
 * blocks untill KEEP_ALIVE register changes or error occurs while reading
 */
CDN_API_STATUS cdn_api_checkalive_blocking(void);

/**
 * \brief set cpu to standby or active
 * \param [in] state - 1 for active, 0 for standby
 * \return status
 */
CDN_API_STATUS cdn_api_maincontrol(unsigned char mode, unsigned char *resp);

/**
 * \breif blocking version of #CDN_API_MainControl
 */
CDN_API_STATUS cdn_api_maincontrol_blocking(unsigned char mode,
					    unsigned char *resp);

/**
 * \brief settings for APB
 *
 * Sends GENERAL_APB_CONF Command via regular Mailbox.
 * @param dpcd_bus_sel Set DPCD to use selected bus (0 for APB or 1 for SAPB)
 * @param dpcd_bus_lock Lock bus type. Aftern that bus type cannot be changed
 * by using this function.
 * @param hdcp_bus_sel Same meaning as for DPCD but for HDCP.
 * @param hdcp_bus_lock Same meaning as for DPCD but for HDCP.
 * @param capb_bus_sel Same meaning as for DPCD but for Cipher APB.
 * @param capb_bus_lock Same meaning as for DPCD but for Cipher APB.
 * @param dpcd_resp [out] Status of the operation.
 * If set to zero then DPCD bus type was successfuly changed.
 * If not then error occurred, most likely due to locked DPCD bus.
 * @param hdcp_resp [out] Same as for DPCD but for HDCP.
 * @param capb_resp [out] Same as for DPCD but for Cipher APB.
 *
 * \return status
 */
CDN_API_STATUS cdn_api_apbconf(uint8_t dpcd_bus_sel, uint8_t dpcd_bus_lock,
			       uint8_t hdcp_bus_sel, uint8_t hdcp_bus_lock,
			       uint8_t capb_bus_sel, uint8_t capb_bus_lock,
			       uint8_t *dpcd_resp, uint8_t *hdcp_resp,
			       uint8_t *capb_resp);

/**
 * blocking version of #CDN_API_MainControl
 */
CDN_API_STATUS cdn_api_apbconf_blocking(uint8_t dpcd_bus_sel,
					uint8_t dpcd_bus_lock,
					uint8_t hdcp_bus_sel,
					uint8_t hdcp_bus_lock,
					uint8_t capb_bus_sel,
					uint8_t capb_bus_lock,
					uint8_t *dpcd_resp,
					uint8_t *hdcp_resp,
					uint8_t *capb_resp);

/**
 * \brief set the  xtensa clk, write this api before turn on the cpu
 */
CDN_API_STATUS cdn_api_setclock(unsigned char mhz);

CDN_API_STATUS cdn_api_general_read_register(unsigned int addr,
					     GENERAL_READ_REGISTER_RESPONSE
					     *resp);
CDN_API_STATUS
cdn_api_general_read_register_blocking(unsigned int addr,
				       GENERAL_READ_REGISTER_RESPONSE *resp);
CDN_API_STATUS cdn_api_general_write_register(unsigned int addr,
					      unsigned int val);
CDN_API_STATUS cdn_api_general_write_register_blocking(unsigned int addr,
						       unsigned int val);
CDN_API_STATUS cdn_api_general_write_field(unsigned int addr,
					   unsigned char startbit,
					   unsigned char bitsno,
					   unsigned int val);
CDN_API_STATUS cdn_api_general_write_field_blocking(unsigned int addr,
						    unsigned char startbit,
						    unsigned char bitsno,
						    unsigned int val);
CDN_API_STATUS cdn_api_general_phy_test_access(uint8_t *resp);
CDN_API_STATUS cdn_api_general_phy_test_access_blocking(uint8_t *resp);
CDN_API_STATUS hdp_rx_loadfirmware(unsigned char *imem,
				    int imemsize,
				    unsigned char *dmem, int dmemsize);

#endif
