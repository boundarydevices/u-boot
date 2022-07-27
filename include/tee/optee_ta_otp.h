/*
 * Copyright (c) 2017, Linaro Limited
 * Copyright (c) 2022, BayLibre
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __OTP_TA_H__
#define __OTP_TA_H__

/* UUID of the trusted application */
#define TA_OTP_UUID \
		{ 0x3712bdda, 0x569f, 0x4940, \
			{ 0xb7, 0x49, 0xfb, 0x3b, 0x06, 0xa5, 0xfd, 0x86 } }
/*
 * TA_OTP_CMD_READ_RAW - Create and fill a secure storage file
 * param[0] (memref) ID used the identify the persistent object
 * param[1] (memref) Raw data dumped from the persistent object
 * param[2] unused
 * param[3] unused
 */
#define TA_OTP_CMD_READ_RAW		0

/*
 * TA_OTP_CMD_WRITE_RAW - Create and fill a secure storage file
 * param[0] (memref) ID used the identify the persistent object
 * param[1] (memref) Raw data to be writen in the persistent object
 * param[2] unused
 * param[3] unused
 */
#define TA_OTP_CMD_WRITE_RAW		1

#ifdef CONFIG_OPTEE_TA_OTP
int optee_otp_readp_value(const char *name,
                          size_t buffer_size,
                          u8 *out_buffer,
                          size_t *out_num_bytes_read);
int optee_otp_read_serial(void);
int optee_otp_read_mac(const char *name);
int optee_otp_read_mac_fdt(void *blob, const char *node,
                           const char *attribute,
                           const char *otp_name);
void optee_otp_ta_close_session(void);
#else /* CONFIG_OPTEE_TA_OTP */
static inline int optee_otp_readp_value(const char *name,
                                        size_t buffer_size,
                                        u8 *out_buffer,
                                        size_t *out_num_bytes_read)
{
        return -ENODEV;
}

static inline int optee_otp_read_serial(void)
{
        return -ENODEV;
}
static inline int optee_otp_read_mac(const char *name)
{
        return -ENODEV;
}
static inline int optee_otp_read_mac_fdt(void *blob, const char *node,
                                         const char *attribute,
                                         const char *otp_name)
{
        return -ENODEV;
}
static inline void optee_otp_ta_close_session(void) {}
#endif /* CONFIG_OPTEE_TA_OTP */

#endif /* __OTP_TA_H__ */
