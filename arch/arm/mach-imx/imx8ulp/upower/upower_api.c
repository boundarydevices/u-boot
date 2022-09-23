/* SPDX-License-Identifier: BSD-3-Clause */
/* +FHDR----------------------------------------------------------------------
 * Copyright 2019-2021 NXP
 * ---------------------------------------------------------------------------
 * FILE NAME      : upower_api.c
 * DEPARTMENT     : BSTC - Campinas, Brazil
 * AUTHOR         : Celso Brites
 * AUTHOR'S EMAIL : celso.brites@nxp.com
 * ---------------------------------------------------------------------------
 * RELEASE HISTORY
 * VERSION DATE        AUTHOR                  DESCRIPTION
 *
 * $Log: upower_api.c.rca $
 *
 *  Revision: 1.216 Fri May 28 06:27:06 2021 nxa55768
 *  powersys_fw_048.011.012.006
 *
 *
 *  Revision: 1.215 Fri Apr 30 06:27:06 2021 nxa10721
 *  powersys_fw_048.011.012.006
 *
 *  Revision: 1.62 Tue Apr 27 12:43:01 2021 nxa11511
 *  Spec review fixes.
 *  Fixes uninitialized variable in upwr_pwm_power_on, upwr_pwm_power_off, upwr_pwm_chng_switch_mem.
 *  Adds new service upwr_pwm_reg_config.
 *
 *  Revision: 1.54 Fri Oct 23 13:19:58 2020 nxa11511
 *  Deleted the GPL license statements, leaving only BSD, as it is compatible with Linux and good for closed ROM/firmware code.
 *
 *  Revision: 1.53 Thu Sep 24 16:42:38 2020 nxa11511
 *  Fixes API buffer base setting.
 *
 *  Revision: 1.51 Wed Sep 16 15:58:44 2020 nxa11511
 *  Fixes the APD/RTD API buffer overlap issue using new #defines and tupedefs in upower_soc_defs.h
 *
 *  Revision: 1.39 Sun Jun 21 14:34:54 2020 nxa11511
 *  Fixes compilation outside block-level testbench.
 *
 *  Revision: 1.25 Tue Mar 10 06:23:23 2020 nxa11511
 *  Fixes copying of argument structs to shared memory.
 *  Fixes shared memory argument buffer size, fixing TKT0534585.
 *
 *  Revision: 1.24 Mon Mar  2 12:25:32 2020 nxa11511
 *  Updates comments to the new version 20200222.
 *  upwr_start callback now checks start error, doesn't change API state in case.
 *  Adds same-domain checks in calls with domain argument.
 *  Updates upwr_start to the new definition.
 *
 *  Revision: 1.17 Mon Jan 27 20:18:07 2020 nxa10721
 *  powersys_fw_021.002.000.004
 *
 *  Revision: 1.19 Mon Jan 27 15:29:03 2020 nxa11511
 *  Adds memcpy, under #ifdef NO_MEMCPY.
 *
 *  Revision: 1.13 Fri Sep 27 10:54:30 2019 nxa13158
 *  added if NULL condition to check if user_callback exists
 *
 *  Revision: 1.12 Mon Sep  9 23:54:18 2019 nxa16953
 *  Updated based on latest register structs.
 *
 *  Revision: 1.11 Fri Aug 23 17:51:07 2019 nxa11511
 *  Adds macros to simplify service request implementation.
 *  Adds shutdown support.
 *  Adds some message field asserts.
 *  Bug fixes.
 *  Changes ok/ko response to an error code.
 *  Adds Exception service requests.
 *  Adds bias setting functions and Diagnostic mode function.
 *  full implementation of the API spec in shared review (version 20190818).
 *
 *  Revision: 1.8 Wed Aug 14 13:45:13 2019 nxa11511
 *  Fixes compilation for simulation.
 *
 *  Revision: 1.7 Wed Aug 14 07:10:53 2019 nxa11511
 *  Partial editing only, not compiled, not tested:
 *  - updates upwr_init to the latest spec, including new argument waitinit.
 *  - bug fixes.
 *  - Rx and Tx interrupts treated in the same ISR.
 *
 *  Revision: 1.3 Sat Aug 10 09:04:32 2019 nxa11511
 *  No longer gets UPWR_NAMESPACE.
 *  Fixes some strict compiling errors.
 *
 *  Revision: 1.1 Fri Aug  2 14:28:58 2019 nxa11511
 *  mvfile on Fri Aug  2 14:28:58 2019 by user nxa11511.
 *  Originated from sync://sync-15088:15088/Projects/common_blocks/da_ip_ahb_upower_subsys_ln28fdsoi_tb/vectors/firmware/stimulus/src/drvapi/vers0/upower_api.cc;1.8
 *
 *  Revision: 1.8 Fri Aug  2 14:28:40 2019 nxa11511
 *  Adapted to Linux coding guidelines.
 *  Converted to C (no longer using C++ features).
 *
 *  Revision: 1.7 Wed Jun 12 15:44:39 2019 nxa11511
 *  Number of MUs now #defined by UPWR_MU_INSTANCES.
 *  Adds optional namespace definition with #define UPWR_NAMESPACE.
 *  No longer disables interrupt at the ISR start (automatically done by CAPI).
 *
 *  Revision: 1.5 Wed Apr 10 14:43:35 2019 nxa11511
 *  Several bug fixes.
 *
 * -----------------------------------------------------------------------------
 * KEYWORDS: micro-power uPower driver API
 * -----------------------------------------------------------------------------
 * PURPOSE: uPower driver API
 * -----------------------------------------------------------------------------
 * PARAMETERS:
 * PARAM NAME RANGE:DESCRIPTION:       DEFAULTS:                           UNITS
 * -----------------------------------------------------------------------------
 * REUSE ISSUES: no reuse issues
 * -FHDR------------------------------------------------------------------------
 */

#include "upower_soc_defs.h"
#include "upower_api.h"

#ifdef  UPWR_BLOCK_LEVEL

#include "capi_wrapper.h"
#include "CAPI_RAM.hh"

#define UPWR_API_ASSERT(c) do { if (!(c)) { ERROR("upower_api.c", "assert failed @line " UPWR_API_STRING(__LINE__)); }} while (0)

#else

#define UPWR_API_ASSERT(c) do {} while (0)

#ifdef NO_MEMCPY

typedef unsigned int size_t;

void *memcpy(void *dest, const void *src, size_t n)
{
	uint32_t *ldest = (uint32_t*) dest;
	uint32_t *lsrc  = (uint32_t*) src;

	while (n > 3) {
		*(ldest++) = *(lsrc++);
		n -= 4;
	}

	if (n > 0) {
		uint8_t  *bdest = (uint8_t*) ldest;
		uint8_t  *bsrc  = (uint8_t*) lsrc;

		while (n > 0) {
			*(bdest++) = *(bsrc++);
			n--;
		}
	}

	return dest;
}
#else
#include <string.h>
#endif /* NO_MEMCPY */

#endif /* not block-level code */

#ifndef __UPWR_API_CODE__
#define __UPWR_API_CODE__
#endif

#ifndef __UPWR_API_DATA__
#define __UPWR_API_DATA__
#endif

#ifndef __UPWR_API_SHARED_RAM__
#define __UPWR_API_SHARED_RAM__
#endif

#define UPWR_API_STR(s)     #s
#define UPWR_API_STRING(s)     UPWR_API_STR(s)
#define UPWR_API_CNCT(a,b)   a##b
#define UPWR_API_CONCAT(a,b) UPWR_API_CNCT(a,b)


/* ---------------------------------------------------------------
 * Common Macros
 * ---------------------------------------------------------------
 */

/* tests Service Group busy */
#define UPWR_SG_BUSY(sg) (sg_busy & (1 << sg))

/* installs a user callback for the Service Group */
#define UPWR_USR_CALLB(sg, cb) do { user_callback[sg] = cb; } while (0)

/* fills up common message header info */
#define UPWR_MSG_HDR(hdr, sg, fn)           \
	hdr.domain   = (uint32_t)pwr_domain;\
	hdr.srvgrp   = sg;                  \
	hdr.function = fn;

/* ---------------------------------------------------------------
 * Common Data Structures
 * ---------------------------------------------------------------
 */

soc_domain_t __UPWR_API_DATA__ pwr_domain; /* CPU/power domain */

upwr_code_vers_t __UPWR_API_DATA__ fw_rom_version;
upwr_code_vers_t __UPWR_API_DATA__ fw_ram_version;
uint32_t         __UPWR_API_DATA__ fw_launch_option;

/* shared memory buffers */

#define UPWR_API_BUFFER_SIZE (MAX_SG_EXCEPT_MEM_SIZE + MAX_SG_PWRMGMT_MEM_SIZE + MAX_SG_VOLTM_MEM_SIZE)

void* __UPWR_API_DATA__  sh_buffer[UPWR_SG_COUNT]; /* service group shared mem
                                                      buffer pointers */

/* Callbacks registered for each service group :
 *
 * NULL means no callback is registered;
 * for sgrp_callback, it also means the service group is free to
 * receive a new request.
 */

upwr_callb            __UPWR_API_DATA__ user_callback[UPWR_SG_COUNT];/* user */
UPWR_RX_CALLB_FUNC_T  __UPWR_API_DATA__ sgrp_callback[UPWR_SG_COUNT];/* API  */

/* request data structures for each service group */
                                                   /* message waiting for TX */
upwr_down_max_msg __UPWR_API_DATA__ sg_req_msg[UPWR_SG_COUNT];
                                                     /* waiting message size */
unsigned int      __UPWR_API_DATA__ sg_req_siz[UPWR_SG_COUNT];
                                                            /* response msg  */
upwr_up_max_msg   __UPWR_API_DATA__ sg_rsp_msg[UPWR_SG_COUNT];
                                                        /* response msg size */
unsigned int      __UPWR_API_DATA__ sg_rsp_siz[UPWR_SG_COUNT];

/* tx pending status for each (1 bit per service group) */
volatile uint32_t  __UPWR_API_DATA__ sg_tx_pend;
                                 /* serv.group of current ongoing Tx, if any */
volatile upwr_sg_t __UPWR_API_DATA__ sg_tx_curr;

/* service group busy status, only for this domain (MU index 0) */
                               /* SG bit = 1 if group is busy with a request */
volatile uint32_t  __UPWR_API_DATA__   sg_busy;

                                  /* OS-dependent memory allocation function */
upwr_malloc_ptr_t  __UPWR_API_DATA__ os_malloc;
               /* OS-dependent pointer->physical address conversion function */
upwr_phyadr_ptr_t  __UPWR_API_DATA__ os_ptr2phy;
                              /* OS-dependent function to lock critical code */
upwr_lock_ptr_t    __UPWR_API_DATA__ os_lock;

struct MU_tag*     __UPWR_API_DATA__ mu      ; /* pointer to MU structure */

                                        /* maps id -> MU Tx interrupt vector */
unsigned int       __UPWR_API_DATA__ mu_txvec;
                                        /* maps id -> MU Rx interrupt vector */
unsigned int       __UPWR_API_DATA__ mu_rxvec;

int __UPWR_API_DATA__ mu_tx_pend;  /* indicates that a transmission was done
                                    * and is pending; this bit is necessary
                                    * because the Tx and Rx interrupts are ORed
                                    * together, and there is no way of telling
                                    * if only Rx interrupt or both occurred just
                                    * by looking at the MU status registers */
UPWR_TX_CALLB_FUNC_T __UPWR_API_DATA__ mu_tx_callb; /* Tx callback */
UPWR_RX_CALLB_FUNC_T __UPWR_API_DATA__ mu_rx_callb; /* Rx callback */

typedef enum {
	UPWR_API_INIT_WAIT,        /* waiting for ROM firmware initialization */
	UPWR_API_INITLZED,         /* ROM firmware initialized */
	UPWR_API_START_WAIT,       /* waiting for start services */
	UPWR_API_SHUTDOWN_WAIT,    /* waiting for shutdown */
	UPWR_API_READY             /* ready to receive service requests */
} upwr_api_state_t;

volatile upwr_api_state_t __UPWR_API_DATA__ api_state;

#ifdef  __cplusplus
#ifndef UPWR_NAMESPACE /* extern "C" 'cancels' the effect of namespace */
extern "C" {
#endif
#endif

/* default pointer->physical address conversion, returns the same address */

static void* ptr2phys(const void* ptr) {
	return (void*)ptr;
}

/* ---------------------------------------------------------------
 * SHARED MEMORY MANAGEMENT
 * --------------------------------------------------------------
 */

/* upwr_ptr2offset() - converts a pointer (casted to uint64_t) to an
 *                     address offset from the  shared memory start.
 *                     If it does not point to a shared memory location,
 *                     the structure pointed is copied to a buffer in the
 *                     shared memory,  and the buffer offset is returned.
 *                     The 2nd argument is the service group to which the
 *                     buffer belongs;
 *                     The 3rd argument is the size of structure to be copied.
 *                     The 4th argument is an offset to apply to the copy
 *                     destination address.
 *                     The 5th argument is ptr before the conversion to physical
 *                     address.
 *                     2nd, 3rd. 4th and 5th arguments are not used if the
 *                     1st one points to a location inside the shared memory.
 */

static uint32_t upwr_ptr2offset(unsigned long         ptr,
				upwr_sg_t        sg,
				size_t           siz,
				size_t           offset,
				const void*      vptr)
{
	if ((ptr >= UPWR_DRAM_SHARED_BASE_ADDR) &&
	    ((ptr - UPWR_DRAM_SHARED_BASE_ADDR) < UPWR_DRAM_SHARED_SIZE)) {
		return (uint32_t)(ptr - UPWR_DRAM_SHARED_BASE_ADDR);
	}

	/* pointer is outside the shared memory, copy the struct to buffer */
	memcpy(offset + (char*)sh_buffer[sg], (void*)vptr, siz);
	return (uint32_t)((unsigned long)sh_buffer[sg] + offset - UPWR_DRAM_SHARED_BASE_ADDR);
}

/* ---------------------------------------------------------------
 * INTERRUPTS AND CALLBACKS
 *   Service-group specific callbacks are in their own sections
 * --------------------------------------------------------------
 */

/* upwr_lock()- locks (lock=1) or unlocks (lock=0) a critical code section;
 *              for now it only needs to protect a portion of the code from
 *              being interrupted by the MU.
 */

static void upwr_lock(int lock)
{
  if (os_lock != NULL) os_lock(lock);
}

/* upwr_exp_isr()- handles the exception interrupt from uPower */

void __UPWR_API_CODE__ upwr_exp_isr(void)
{
	/* TBD - what do do here */
}

/* upwr_copy2tr prototype; function definition in auxiliary function section */
void upwr_copy2tr(struct MU_tag* mu, const uint32_t* msg, unsigned int size);

#define UPWR_MU_TSR_EMPTY ((uint32_t)((1 << UPWR_MU_MSG_SIZE) -1))

/* upwr_txrx_isr()- handles both the Tx and Rx MU interrupts */

void __UPWR_API_CODE__ upwr_txrx_isr(void)
{
	if (mu_tx_pend &&                              /* Tx pending and ... */
           (mu->TSR.R == UPWR_MU_TSR_EMPTY)) {    /* ... Tx registers empty: */
	                                                  /* Tx ISR occurred */
		mu_tx_pend   = 0;
		mu->TCR.R    = 0; /* disable the tx interrupts */
		mu->FCR.B.F0 = 0; /* urgency flag off, in case it was set */
		if (mu_tx_callb != NULL) mu_tx_callb();
	}

	if (mu->RSR.R != (uint32_t)0) {                   /* Rx ISR occurred */

		mu->RCR.R = 0;   /* disable the interrupt until data is read */

		if (mu_rx_callb != NULL) mu_rx_callb();
	}
}

/**
 * upwr_next_req() - sends the next pending service request message, if any.
 *
 * Called upon MU Tx interrupts, it checks if there is any service request
 * pending amongst the service groups, and sends the request if needed.
 *
 * Context: no sleep, no locks taken/released.
 * Return: none (void).
 */

void upwr_next_req(void)
{
	upwr_sg_t sg = (upwr_sg_t)0;

	/* no lock needed here, this is called from an MU ISR */
	sg_tx_pend &= ~(1 << sg_tx_curr); /* no longer pending */

	if (sg_tx_pend == 0) return; /* no other pending */

	/* find the next one pending */
	for (uint32_t mask = 1; mask < (1 << UPWR_SG_COUNT); mask = mask << 1) {
		if (sg_tx_pend & mask) break;
		sg = (upwr_sg_t)((int)sg + 1);
	}

	sg_tx_curr = sg;
	if (upwr_tx((uint32_t*)&sg_req_msg[sg],
		     sg_req_siz[sg],
		     upwr_next_req) < 0) {
		UPWR_API_ASSERT(0);
		return; /* leave the Tx pending */
	}
}

/**
 * upwr_mu_int_callback() - general MU interrupt callback.
 *
 * Called upon MU Rx interrupts, it calls the Service Group-specific callback,
 * if any registered, based on the service group field in the received message.
 * Otherwise, calls the user callback, if any registered.
 *
 * Context: no sleep, no locks taken/released.
 * Return: none (void).
 */

void upwr_mu_int_callback(void)
{
	upwr_sg_t              sg;       /* service group number */
	UPWR_RX_CALLB_FUNC_T   sg_callb; /* service group callback */
	upwr_up_max_msg        rxmsg = {0};
	unsigned int           size;     /* in words */

	if (upwr_rx((char *)&rxmsg, &size) < 0) {
		UPWR_API_ASSERT(0);
		return;
	}

	sg = (upwr_sg_t)rxmsg.hdr.srvgrp;

	/* copy msg to the service group buffer */
	msg_copy((char *)&sg_rsp_msg[sg], (char *)&rxmsg, size);
	sg_rsp_siz[sg] = size;

	/* clear the service group busy status */
	sg_busy &= ~(1 << sg);	/* no lock needed here, we're in the MU ISR */

	if ((sg_callb = sgrp_callback[sg]) == NULL) {
		upwr_callb user_callb = user_callback[sg];

		/* no service group callback; call the user callback if any */

		if (user_callb == NULL) goto done; /* no user callback */

		/* make the user callback */
		user_callb(sg,
			   rxmsg.hdr.function,
			   (upwr_resp_t)rxmsg.hdr.errcode,
			   (int)(size == 2)? rxmsg.word2:rxmsg.hdr.ret);
		goto done;
	}

	sg_callb();                    /* finally make the group callback */
	/* don't uninstall the group callback, it's permanent */

done:
	if (rxmsg.hdr.errcode == UPWR_RESP_SHUTDOWN) { /* shutdown error: */
		api_state = UPWR_API_INITLZED;
					 /* change the API state automatically
					  * so new requests are rejected by
					  * the API immediately */
	}
}

/**
 * upwr_srv_req() - sends a service request message.
 * @sg: message service group.
 * @msg: pointer to the message
 * @size: message size in 32-bit words.
 *
 * The message is sent right away if possible, or gets pending to be sent later.
 * If pending, the message is stored in sg_req_msg and will be sent when the
 * MU tranmission buffer is clear and there are no other pending messages
 * from higher priority service groups.
 *
 * This is an auxiliary function used by the rest of the API calls.
 * It is normally not called by the driver code, unless maybe for test purposes.
 *
 * Context: no sleep, no locks taken/released.
 * Return: none (void)
 */

void upwr_srv_req(upwr_sg_t     sg,
		  uint32_t*     msg,
		  unsigned int  size)
{
	int rc;

	upwr_lock(1);
	sg_busy |= 1 << sg;
	upwr_lock(0);

	rc = upwr_tx(msg, size, upwr_next_req);
	if (rc  < 0) {
		UPWR_API_ASSERT(rc == -1);
		/* queue full, make the transmission pending */
		msg_copy((char *)&sg_req_msg[sg], (char *)msg, size);
		sg_req_siz[sg] = size;
		upwr_lock(1);
		sg_tx_curr  = sg;
		sg_tx_pend |= 1 << sg;
		upwr_lock(0);
		return;
	}
}

/**---------------------------------------------------------------
 * INITIALIZATION, CONFIGURATION
 *
 * A reference uPower initialization sequence goes as follows:
 *
 * 1. host CPU calls upwr_init.
 * 2. (optional) host checks the ROM version and SoC code calling upwr_vers(...)
 *    and optionally performs any configuration or workaround accordingly.
 * 3. host CPU calls upwr_start to start the uPower services, passing a
 *    service option number.
 *    If no RAM code is loaded or it has no service options, the launch option
 *    number passed must be 0, which will start the services available in ROM.
 *    upwr_start also receives a pointer to a callback called by the API
 *    when the firmware is ready to receive service requests.
 *    The callback may be replaced by polling, calling upwr_req_status in a loop
 *    or upwr_poll_req_status; in this case the callback pointer may be NULL.
 *    A host may call upwr_start even if the services were already started by
 *    any host: if the launch option is the same, the response will be ok,
 *    but will indicate error if the services were already started with a
 *    different launch option.
 * 4. host waits for the callback calling, or polling finishing;
 *    if no error is returned, it can start making service calls using the API.
 *
 * Variations on that reference sequence are possible:
 *  - the uPower services can be started using the ROM code only, which includes
 *    the basic Power Management services, among others, with launch option
 *    number = 0.
 *    The code RAM can be loaded while these services are running and,
 *    when the loading is done, the services can be re-started with these 2
 *    requests executed in order: upwr_xcp_shutdown and upwr_start,
 *    using the newly loaded RAM code (launch option > 0).
 *
 * NOTE: the initialization call upwr_init is not effective and
 *       returns error when called after the uPower services are started.
 */

/**
 * upwr_start_callb() - internal callback for the Rx message from uPower
 * that indicates the firmware is ready to receive the start commands.
 * It calls the user callbacks registered in the upwr_start_boot and upwr_start
 * call.
 */

void upwr_start_callb(void)
{
	switch (api_state) {
		case UPWR_API_START_WAIT:
		{
			upwr_rdy_callb start_callb = (upwr_rdy_callb)
						  user_callback[UPWR_SG_EXCEPT];

			upwr_ready_msg* msg =
				(upwr_ready_msg*)&sg_rsp_msg[UPWR_SG_EXCEPT];

			/* message sanity check */
			UPWR_API_ASSERT(msg->hdr.srvgrp   == UPWR_SG_EXCEPT);
			UPWR_API_ASSERT(msg->hdr.function == UPWR_XCP_START);
			UPWR_API_ASSERT(msg->hdr.errcode  == UPWR_RESP_OK);

			fw_ram_version.soc_id = fw_rom_version.soc_id;
			fw_ram_version.vmajor = msg->args.vmajor;
			fw_ram_version.vminor = msg->args.vminor;
			fw_ram_version.vfixes = msg->args.vfixes;

			/* vmajor == vminor == vfixes == 0 indicates start error
			   in this case, go back to the INITLZED state */

			if ((fw_ram_version.vmajor != 0) ||
			    (fw_ram_version.vminor != 0) ||
			    (fw_ram_version.vfixes != 0)) {

				api_state = UPWR_API_READY;

				/* initialization is over:
				   uninstall the user callback just in case */
				UPWR_USR_CALLB(UPWR_SG_EXCEPT, NULL);

				if (fw_launch_option == 0) {
					/* launched ROM firmware:
					 * RAM fw versions must be all 0s */
					fw_ram_version.vmajor =
					fw_ram_version.vminor =
					fw_ram_version.vfixes = 0;
				}
			}
			else api_state = UPWR_API_INITLZED;

			start_callb(msg->args.vmajor,
			            msg->args.vminor,
				    msg->args.vfixes);
		}
		break;

		case UPWR_API_SHUTDOWN_WAIT:
		{
			upwr_callb user_callb = (upwr_callb)
						  user_callback[UPWR_SG_EXCEPT];

			upwr_shutdown_msg* msg =
				(upwr_shutdown_msg*)&sg_rsp_msg[UPWR_SG_EXCEPT];

			/* message sanity check */
			UPWR_API_ASSERT(msg->hdr.srvgrp   == UPWR_SG_EXCEPT);
			UPWR_API_ASSERT(msg->hdr.function == UPWR_XCP_SHUTDOWN);
			UPWR_API_ASSERT(msg->hdr.errcode  == UPWR_RESP_OK);

			if ((upwr_resp_t)msg->hdr.errcode == UPWR_RESP_OK)
				api_state = UPWR_API_INITLZED;

			if (user_callb != NULL) user_callb(UPWR_SG_EXCEPT,
			                                   UPWR_XCP_SHUTDOWN,
				                           (upwr_resp_t)
							     msg->hdr.errcode,
			                                   0);
		}
		break;

		case UPWR_API_READY:
		{
			upwr_callb user_callb = (upwr_callb)
						  user_callback[UPWR_SG_EXCEPT];

			upwr_up_max_msg* msg =
				    (upwr_up_max_msg*)&sg_rsp_msg[UPWR_SG_EXCEPT];

			/* message sanity check */
			UPWR_API_ASSERT(msg->hdr.srvgrp   == UPWR_SG_EXCEPT);

			if (user_callb != NULL) user_callb(UPWR_SG_EXCEPT,
			                                   msg->hdr.function,
				                           (upwr_resp_t)
							     msg->hdr.errcode,
			                                   (int)(sg_rsp_siz[UPWR_SG_EXCEPT] == 2) ? msg->word2 : msg->hdr.ret);
		}
		break;

		default:
			UPWR_API_ASSERT(0);
			break;
	}
}

/**
 * upwr_init() - API initialization; must be the first API call after reset.
 * @domain: SoC-dependent CPU domain id; identifier used by the firmware in
 * many services. Defined by SoC-dependent type soc_domain_t found in
 * upower_soc_defs.h.
 * @muptr: pointer to the MU instance.
 * @mallocptr: pointer to the memory allocation function
 * @physaddrptr: pointer to the function to convert pointers to
 * physical addresses. If NULL, no conversion is made (pointer=physical address)
 * @isrinstptr: pointer to the function to install the uPower ISR callbacks;
 * the function receives the pointers to the MU tx/rx and Exception ISRs
 * callbacks, which must be called from the actual system ISRs.
 * The function pointed by isrinstptr must also enable the interrupt at the
 * core/interrupt controller, but must not enable the interrupt at the MU IP.
 * The system ISRs are responsible for dealing with the interrupt controller,
 * performing any other context save/restore, and any other housekeeping.
 * @lockptr: pointer to a function that prevents MU interrupts (if argrument=1)
 * or allows it (if argument=0). The API calls this function to make small
 * specific code portions thread safe. Only MU interrupts must be avoided,
 * the code may be suspended for other reasons.
 * If no MU interrupts can happen during the execution of an API call or
 * callback, even if enabled, for some other reason (e.g. interrupt priority),
 * then this argument may be NULL.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if failed to allocate memory, or use some other resource.
 *        -2 if any argument is invalid.
 *        -3 if failed to send the ping message.
 *        -4 if failed to receive the initialization message, or was invalid
 */

int upwr_init(	soc_domain_t               domain,
		struct MU_tag*             muptr,
		const  upwr_malloc_ptr_t   mallocptr,
		const  upwr_phyadr_ptr_t   phyadrptr,
		const  upwr_inst_isr_ptr_t isrinstptr,
		const  upwr_lock_ptr_t     lockptr)
{
	int j;

	upwr_sg_t           sg;       /* service group number */
	unsigned int        size;     /* in words */
	unsigned long        dom_buffer_base = (domain == RTD_DOMAIN)?
							   UPWR_API_BUFFER_BASE:
                           ((UPWR_API_BUFFER_ENDPLUS + UPWR_API_BUFFER_BASE)/2);

	upwr_init_msg*      msg = (upwr_init_msg*)&sg_rsp_msg[UPWR_SG_EXCEPT];

        mu = muptr;
	mu->TCR.R = mu->RCR.R = 0; /* disable tx and rx interrupts, in case
				      not called 1st time after reset   */

	os_malloc   = mallocptr;
	os_ptr2phy  = (phyadrptr == (upwr_phyadr_ptr_t)NULL)? ptr2phys :
                                                              phyadrptr;
        os_lock     = lockptr;
	api_state   = UPWR_API_INIT_WAIT;
        sg_busy     = 0;
	pwr_domain  = domain;

	/* initialize the versions, in case they are polled */
	fw_rom_version.soc_id =
	fw_rom_version.vmajor =
	fw_rom_version.vminor =
	fw_rom_version.vfixes = 0;

	fw_ram_version.soc_id =
	fw_ram_version.vmajor =
	fw_ram_version.vminor =
	fw_ram_version.vfixes = 0;

	mu_tx_pend     = 0;
	sg_tx_pend     = 0;
	sg_tx_curr     = UPWR_SG_COUNT; /* means none here */

	/* must have enough headroom for the shared RAM API buffers */
	UPWR_API_ASSERT((UPWR_DRAM_SHARED_ENDPLUS - dom_buffer_base) >=
			 UPWR_API_BUFFER_SIZE);

	/* UPWR_API_BUFFER_BASE must let enough buffer for both domains */
	UPWR_API_ASSERT((UPWR_DRAM_SHARED_ENDPLUS - UPWR_API_BUFFER_BASE) >=
			(UPWR_API_BUFFER_SIZE*2));

	sh_buffer[UPWR_SG_EXCEPT ] = (void*)(unsigned long)dom_buffer_base;
	sh_buffer[UPWR_SG_PWRMGMT] = (void*)(unsigned long)(dom_buffer_base + MAX_SG_EXCEPT_MEM_SIZE);
	sh_buffer[UPWR_SG_DELAYM ] = NULL;
	sh_buffer[UPWR_SG_VOLTM  ] = (void*)(unsigned long)(dom_buffer_base + MAX_SG_EXCEPT_MEM_SIZE + MAX_SG_PWRMGMT_MEM_SIZE);
	sh_buffer[UPWR_SG_CURRM  ] = NULL;
	sh_buffer[UPWR_SG_TEMPM  ] = NULL;
	sh_buffer[UPWR_SG_DIAG   ] = NULL;
	/* (no buffers service groups other than xcp and pwm for now) */

	for (j = 0; j < UPWR_SG_COUNT; j++)
	{
		user_callback[j] = NULL;
	        /* service group Exception gets the initialization callbacks */
		sgrp_callback[j] = (j == UPWR_SG_EXCEPT)? upwr_start_callb:NULL;

		/* response messages with an initial consistent content */
		sg_rsp_msg[j].hdr.errcode = UPWR_RESP_SHUTDOWN;
	}

	if (mu->FSR.B.F0) {            /* init message already received:
		                          assume tasks are running on uPower */

		/* send a ping message down to get the ROM version back */
		upwr_xcp_ping_msg ping_msg = {0};

		ping_msg.hdr.domain   = pwr_domain;
		ping_msg.hdr.srvgrp   = UPWR_SG_EXCEPT;
		ping_msg.hdr.function = UPWR_XCP_PING;

		if (mu->RSR.B.RF0) { /* first clean any Rx message left over */
			upwr_rx((char *)msg, &size);
		}

		while (mu->TSR.R != UPWR_MU_TSR_EMPTY) {    /* wait any Tx ...
						    ... left over to be sent */
			#ifdef UPWR_BLOCK_LEVEL
			WAIT_CLK(1000); /* waiting loop must advance clock */
			#endif
		};

		/* now send the ping message;
		   do not use upwr_tx, which needs API initilized;
		   just write to the MU TR register(s)
		 */
		mu->FCR.B.F0 = 1; /* flag urgency status */
		upwr_copy2tr(mu, (uint32_t*)&ping_msg, sizeof(ping_msg)/4);
	}

	do {
		/* poll for the MU Rx status: wait for an init message, either
		 * 1st sent from uPower after reset or as a response to a ping
		 */
		while (mu->RSR.B.RF0 == (uint32_t)0)	{
			#ifdef UPWR_BLOCK_LEVEL
			WAIT_CLK(1000); /* waiting loop must advance clock */
			#endif
		};

		mu->FCR.B.F0 = 0; /* urgency status off, in case it was set */

		if (upwr_rx((char *)msg, &size) < 0) {
			return -4;
		}

		if (size != (sizeof(upwr_init_msg)/4)) {
			if (mu->FSR.B.F0) continue; /* discard left over msg */
				     else return -4;
		}

		sg = (upwr_sg_t)msg->hdr.srvgrp;
		if (sg != UPWR_SG_EXCEPT) {
			if (mu->FSR.B.F0) continue; /* discard left over msg */
				     else return -4;
		}

		if ((upwr_xcp_f_t)msg->hdr.function   != UPWR_XCP_INIT) {
			if (mu->FSR.B.F0) continue; /* discard left over msg */
				     else return -4;
		}

		break;
	} while (1);

	fw_rom_version.soc_id = msg->args.soc;
	fw_rom_version.vmajor = msg->args.vmajor;
	fw_rom_version.vminor = msg->args.vminor;
	fw_rom_version.vfixes = msg->args.vfixes;

        if (upwr_rx_callback(upwr_mu_int_callback) < 0) {
		/* catastrophic error, but is it possible to happen? */
		UPWR_API_ASSERT(0);
		return -1;
	}

	mu_tx_callb    = NULL; /* assigned on upwr_tx */

	/* install the ISRs and enable the interrupts */

        isrinstptr(upwr_txrx_isr, upwr_exp_isr);

	mu->RCR.R = 1;    /* enable only RR[0] receive interrupt */

        api_state = UPWR_API_INITLZED;
	return 0;
} /* upwr_init */

/**
 * upwr_start() - Starts the uPower services.
 * @launchopt: a number to select between multiple launch options,
 * that may define, among other things, which services will be started,
 * or which services implementations, features etc.
 * launchopt = 0 selects a subset of services implemented in ROM;
 * any other number selects service sets implemented in RAM, launched
 * by the firmware function ram_launch; if an invalid launchopt value is passed,
 * no services are started, and the callback returns error (see below).
 * @rdycallb: pointer to the callback to be called when the uPower is ready
 * to receive service requests. NULL if no callback needed.
 * The callback receives as arguments the RAM firmware version numbers.
 * If all 3 numbers (vmajor, vminor, vfixes) are 0, that means the
 * service launching failed.
 * Firmware version numbers will be the same as ROM if launchopt = 0,
 * selecting the ROM services.
 *
 * upwr_start can be called by any domain even if the services are already
 * started: it has no effect, returning success, if the launch option is the
 * same as the one that actually started the service, and returns error if
 * called with a different option.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded or
 * not.
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_EXCEPT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if a resource failed,
 *        -2 if the domain passed is the same as the caller,
 *        -3 if called in an invalid API state
 */

int upwr_start(	uint32_t                launchopt,
		const upwr_rdy_callb    rdycallb)
{
	upwr_start_msg txmsg = {0};

	if (api_state != UPWR_API_INITLZED) return -3;

	UPWR_USR_CALLB(UPWR_SG_EXCEPT, (upwr_callb)rdycallb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_EXCEPT, UPWR_XCP_START);

	txmsg.hdr.arg = fw_launch_option = launchopt;

	if (upwr_tx((uint32_t*)&txmsg, sizeof(txmsg)/4, NULL) < 0) {
		/* catastrophic error, but is it possible to happen? */
		UPWR_API_ASSERT(0);
		return -1;
	}

	api_state = UPWR_API_START_WAIT;
	return 0;
} /* upwr_start */

/**---------------------------------------------------------------
 * EXCEPTION SERVICE GROUP
 */

/**
 * upwr_xcp_config() - Applies general uPower configurations.
 * @config: pointer to the uPower SoC-dependent configuration struct
 * upwr_xcp_config_t defined in upower_soc_defs.h. NULL may be passed, meaning
 * a request to read the configuration, in which case it appears in the callback
 * argument ret, or can be pointed by argument retptr in the upwr_req_status and
 * upwr_poll_req_status calls, casted to upwr_xcp_config_t.
 * @callb: pointer to the callback to be called when the uPower has finished
 * the configuration, or NULL if no callback needed (polling used instead).
 *
 * Some configurations are targeted for a specific domain (see the struct
 * upwr_xcp_config_t definition in upower_soc_defs.h); this call has implicit
 * domain target (the same domain from which is called).
 *
 * The return value is always the current configuration value, either in a
 * read-only request (config = NULL) or after setting a new cnfiguration
 * (non-NULL config).
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded or
 * not.
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_EXCEPT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 */

int upwr_xcp_config(const upwr_xcp_config_t* config, const upwr_callb callb)
{
	upwr_xcp_config_msg txmsg = {0};

	if (api_state != UPWR_API_READY)  return -3;
        if (UPWR_SG_BUSY(UPWR_SG_EXCEPT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_EXCEPT, callb);

	if (config == NULL) {
		txmsg.hdr.arg = 1;         /* 1= read, txmsg.word2 ignored */
	}
	else {
		txmsg.hdr.arg = 0;         /* 1= write */
		txmsg.word2   = config->R;
	}
	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_EXCEPT, UPWR_XCP_CONFIG);

	upwr_srv_req(UPWR_SG_EXCEPT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_xcp_sw_alarm() - Makes uPower issue an alarm interrupt to given domain.
 * @domain: identifier of the domain to alarm. Defined by SoC-dependent type
 * soc_domain_t found in upower_soc_defs.h.
 * @code: alarm code. Defined by SoC-dependent type upwr_alarm_t found in
 * upower_soc_defs.h.
 * @callb: pointer to the callback to be called when the uPower has finished
 * the alarm, or NULL if no callback needed (polling used instead).
 *
 * The function requests the uPower to issue an alarm of the given code as if
 * it had originated internally. This service is useful mainly to test the
 * system response to such alarms, or to make the system handle a similar alarm
 * situation detected externally to uPower.
 *
 * The system ISR/code handling the alarm may retrieve the alarm code by calling
 * the auxiliary function upwr_alarm_code.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded or
 * not.
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_EXCEPT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 */

int upwr_xcp_sw_alarm(soc_domain_t     domain,
		      upwr_alarm_t     code,
		      const upwr_callb callb)
{
	upwr_xcp_swalarm_msg txmsg = {0};

	if (api_state != UPWR_API_READY)  return -3;
	if (UPWR_SG_BUSY(UPWR_SG_EXCEPT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_EXCEPT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_EXCEPT, UPWR_XCP_SW_ALARM);
	txmsg.hdr.domain = (uint32_t)domain;
	txmsg.hdr.arg    = (uint32_t)code;

	upwr_srv_req(UPWR_SG_EXCEPT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_xcp_set_ddr_retention() - M33/A35 can use this API to set/clear ddr retention
 * @domain: identifier of the caller domain.
 * soc_domain_t found in upower_soc_defs.h.
 * @enable: true, means that set ddr retention, false clear ddr retention.
 * @callb: NULL
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_EXCEPT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 */

int upwr_xcp_set_ddr_retention(soc_domain_t     domain,
                        uint32_t enable,
                        const upwr_callb callb)
{
	upwr_xcp_ddr_retn_msg txmsg = {0};

	if (api_state != UPWR_API_READY)  return -3;
	if (UPWR_SG_BUSY(UPWR_SG_EXCEPT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_EXCEPT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_EXCEPT, UPWR_XCP_SET_DDR_RETN);
	txmsg.hdr.domain = (uint32_t)domain;
	txmsg.hdr.arg    = (uint32_t)enable;

	upwr_srv_req(UPWR_SG_EXCEPT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_xcp_set_rtd_use_ddr() - M33 call this API to inform uPower, M33 is using ddr
 * @domain: identifier of the caller domain.
 * soc_domain_t found in upower_soc_defs.h.
 * @enable: not 0, true, means that RTD is using ddr. 0, false, means that, RTD is not using ddr.
 * @callb: NULL
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_EXCEPT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 */

int upwr_xcp_set_rtd_use_ddr(soc_domain_t     domain,
                        uint32_t is_use_ddr,
                        const upwr_callb callb)
{
	upwr_xcp_rtd_use_ddr_msg txmsg = {0};

	if (api_state != UPWR_API_READY)  return -3;
	if (UPWR_SG_BUSY(UPWR_SG_EXCEPT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_EXCEPT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_EXCEPT, UPWR_XCP_SET_RTD_USE_DDR);
	txmsg.hdr.domain = (uint32_t)domain;
	txmsg.hdr.arg    = (uint32_t)is_use_ddr;

	upwr_srv_req(UPWR_SG_EXCEPT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_xcp_set_rtd_apd_llwu() - M33/A35 can use this API to set/clear rtd_llwu apd_llwu
 * @domain: set which domain (RTD_DOMAIN, APD_DOMAIN) LLWU.
 * soc_domain_t found in upower_soc_defs.h.
 * @enable: true, means that set rtd_llwu or apd_llwu, false clear rtd_llwu or apd_llwu.
 * @callb: NULL
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_EXCEPT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 */

int upwr_xcp_set_rtd_apd_llwu(soc_domain_t     domain,
                        uint32_t enable,
                        const upwr_callb callb)
{
	upwr_xcp_rtd_apd_llwu_msg txmsg = {0};

	if (api_state != UPWR_API_READY)  return -3;
	if (UPWR_SG_BUSY(UPWR_SG_EXCEPT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_EXCEPT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_EXCEPT, UPWR_XCP_SET_RTD_APD_LLWU);
	txmsg.hdr.domain = (uint32_t)domain;
	txmsg.hdr.arg    = (uint32_t)enable;

	upwr_srv_req(UPWR_SG_EXCEPT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_xcp_shutdown() - Shuts down all uPower services and power mode tasks.
 * @callb: pointer to the callback to be called when the uPower has finished
 * the shutdown, or NULL if no callback needed
 * (polling used instead).
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded or
 * not.
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_EXCEPT as the service group argument.
 *
 * At the callback the uPower/API is back to initialization/start-up phase,
 * so service request calls return error.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 */

int upwr_xcp_shutdown(const upwr_callb callb)
{
	upwr_xcp_shutdown_msg txmsg = {0};

	if (api_state != UPWR_API_READY)  return -3;
    if (UPWR_SG_BUSY(UPWR_SG_EXCEPT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_EXCEPT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_EXCEPT, UPWR_XCP_SHUTDOWN);

	upwr_srv_req(UPWR_SG_EXCEPT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	api_state = UPWR_API_SHUTDOWN_WAIT;

	return 0;
}

/**
 * upwr_xcp_i2c_access() - Performs an access through the uPower I2C interface.
 * @addr: I2C slave address, up to 10 bits.
 * @data_size: determines the access direction and data size in bytes, up to 4;
 * negetive data_size determines a read  access with size -data_size;
 * positive data_size determines a write access with size  data_size;
 * data_size=0 is invalid, making the service return error UPWR_RESP_BAD_REQ.
 * @subaddr_size: size of the sub-address in bytes, up to 4; if subaddr_size=0,
 * no subaddress is used.
 * @subaddr: sub-address, only used if subaddr_size > 0.
 * @wdata: write data, up to 4 bytes; ignored if data_size < 0 (read)
 * @callb: pointer to the callback to be called when the uPower has finished
 * the access, or NULL if no callback needed
 * (polling used instead).
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded or
 * not.
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_EXCEPT as the service group argument.
 *
 * The service performs a read (data_size < 0) or a write (data_size > 0) of
 * up to 4 bytes on the uPower I2C interface. The data read from I2C comes via
 * the callback argument ret, or written to the variable pointed by retptr,
 * if polling is used (calls upwr_req_status or upwr_poll_req_status).
 * ret (or *retptr) also returns the data written on writes.
 *
 * Sub-addressing is supported, with sub-address size determined by the argument
 * subaddr_size, up to 4 bytes. Sub-addressing is not used if subaddr_size=0.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 */

#ifdef UPWR_BLOCK_LEVEL
/* emulated struct equivalent to upwr_i2c_access in upower_defs.h
   (simulation only)
 */

struct upwr_i2c_access_emul {
	CapiRamHalf      addr;
	CapiRamByte      data_size;
	CapiRamByte      subaddr_size;
	CapiRamWord      subaddr;
	CapiRamWord      data;

	upwr_i2c_access_emul(unsigned int address)
	{
		addr.setAddress        (address);
		data_size.setAddress   (address+2);
		subaddr_size.setAddress(address+3);
		subaddr.setAddress     (address+4);
		data.setAddress        (address+8);
	}
};
#endif

int upwr_xcp_i2c_access(uint16_t         addr,
			int8_t           data_size,
			uint8_t          subaddr_size,
			uint32_t         subaddr,
			uint32_t         wdata,
			const upwr_callb callb)
{
	unsigned long ptrval = (unsigned long)sh_buffer[UPWR_SG_EXCEPT];
	#ifdef UPWR_BLOCK_LEVEL
	upwr_i2c_access_emul  i2c_acc((unsigned int)ptrval);
	upwr_i2c_access_emul* i2c_acc_ptr = &i2c_acc;
	#else
	upwr_i2c_access*      i2c_acc_ptr = (upwr_i2c_access*)ptrval;
	#endif
	upwr_pwm_pmiccfg_msg  txmsg = {0};

	if (api_state != UPWR_API_READY)  return -3;
	if (UPWR_SG_BUSY(UPWR_SG_EXCEPT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_EXCEPT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_EXCEPT, UPWR_XCP_I2C);

	i2c_acc_ptr->addr         = addr       ;
	i2c_acc_ptr->subaddr      = subaddr    ;
	i2c_acc_ptr->subaddr_size = subaddr_size;
	i2c_acc_ptr->data         = wdata      ;
	i2c_acc_ptr->data_size    = data_size  ;

	txmsg.ptr = upwr_ptr2offset(ptrval,
				    UPWR_SG_EXCEPT,
				    (size_t)sizeof(upwr_i2c_access),
				    0,
				    i2c_acc_ptr);

	upwr_srv_req(UPWR_SG_EXCEPT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**---------------------------------------------------------------
 * VOLTAGE MANAGERMENT SERVICE GROUP
 */

/**
 * upwr_vtm_pmic_cold_reset() -request cold reset the pmic
 * pmic will power cycle all the regulators
 * @callb: response callback pointer; NULL if no callback needed.
 *
 * The function requests uPower to cold reset the pmic.
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_VOLTM as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */
int upwr_vtm_pmic_cold_reset(upwr_callb callb)
{
	upwr_volt_pmic_cold_reset_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_PMIC_COLD_RESET);

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**
 * upwr_vtm_set_pmic_mode() -request uPower set pmic mode
 * @pmic_mode: the target mode need to be set
 * @callb: response callback pointer; NULL if no callback needed.
 *
 * The function requests uPower to set pmic mode
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_VOLTM as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */
int upwr_vtm_set_pmic_mode(uint32_t pmic_mode, upwr_callb callb)
{
	upwr_volt_pmic_set_mode_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_SET_PMIC_MODE);

    txmsg.hdr.arg = pmic_mode;

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**
 * upwr_vtm_chng_pmic_voltage() - Changes the voltage of a given rail.
 * @rail: pmic rail id.
 * @volt: the target voltage of the given rail, accurate to uV
 * If pass volt value 0, means that power off this rail.
 * @callb: response callback pointer; NULL if no callback needed.
 *
 * The function requests uPower to change the voltage of the given rail.
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_VOLTM as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_vtm_chng_pmic_voltage(uint32_t rail, uint32_t volt, upwr_callb callb)
{
	upwr_volt_pmic_set_volt_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_CHNG_PMIC_RAIL_VOLT);

	txmsg.args.rail = rail;

    txmsg.args.volt = (volt + PMIC_VOLTAGE_MIN_STEP - 1) / PMIC_VOLTAGE_MIN_STEP;

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**
 * upwr_vtm_get_pmic_voltage() - Get the voltage of a given rail.
 * @rail: pmic rail id.
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to get the voltage of the given rail.
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_VOLTM as the service group argument.
 *
 * The voltage data read from uPower via
 * the callback argument ret, or written to the variable pointed by retptr,
 * if polling is used (calls upwr_req_status or upwr_poll_req_status).
 * ret (or *retptr) also returns the data written on writes.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_vtm_get_pmic_voltage(uint32_t rail, upwr_callb callb)
{
	upwr_volt_pmic_get_volt_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_GET_PMIC_RAIL_VOLT);

	txmsg.args.rail = rail;

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**
 * upwr_vtm_dump_dva_info() - Dump dva information to M33/A35
 * @dump_addr: uPower dump dva information to the given address
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to dump dva information to the given address
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_VOLTM as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_vtm_dump_dva_info(uint32_t dump_addr, upwr_callb callb)
{
	upwr_volt_dva_dump_info_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	if ((dump_addr < UPWR_DRAM_SHARED_BASE_ADDR) ||
	    ((dump_addr - UPWR_DRAM_SHARED_BASE_ADDR) >= UPWR_DRAM_SHARED_SIZE)) {
		return -2;
    }

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_DVA_DUMP_INFO);

	txmsg.args.addr_offset = dump_addr - UPWR_DRAM_SHARED_BASE_ADDR;

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**
 * upwr_vtm_dva_request() - request uPower to dva an array IDs
 * @id: id of soc components, such as A35, M33, GPU, SDHC and etc, it is a bit group, extending to two u32 types.
 * @mode: OD (over drive) mode, ND (normal drive)  mode, LD (lower drive) mode,
 * type: enum work_mode, defined in upower_defs.h
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to dva an array IDs to the give work mode.
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_VOLTM as the service group argument.
 *
 * M33/A35 need to check the execute result
 * 0 means success, 1 means hold on(A35 request LD, but GPU request OD), negative value means failure.
 * upower fw needs support cocurrent request from M33 and A35.
 * upower fw needs support multiple masters requesting different modes.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_vtm_dva_request(const uint32_t id[], enum work_mode mode, upwr_callb callb)
{
	unsigned long ptrval = (unsigned long)sh_buffer[UPWR_SG_VOLTM];
    upwr_dva_id_struct *dva_id_struct_ptr = (upwr_dva_id_struct *)ptrval;
	upwr_volt_dva_req_id_msg  txmsg = {0};

	if (api_state != UPWR_API_READY)  return -3;
	if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_DVA_REQ_ID);

    memcpy(&(dva_id_struct_ptr->id_word0), &id[0], 4);
    memcpy(&(dva_id_struct_ptr->id_word1), &id[1], 4);
    dva_id_struct_ptr->mode = mode;

	txmsg.ptr = upwr_ptr2offset(ptrval,
				    UPWR_SG_VOLTM,
				    (size_t)sizeof(upwr_dva_id_struct),
				    0,
				    dva_id_struct_ptr);

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_vtm_dva_request_soc() - request uPower to dva the whole SOC to the given work mode
 * @mode: OD (over drive) mode, ND (normal drive)  mode, LD (lower drive) mode,
 * type: enum work_mode, defined in upower_defs.h
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to switch whole SOC to the given work mode.
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_VOLTM as the service group argument.
 *
 * M33/A35 need to check the execute result
 * 0 means success, 1 means hold on(A35 request LD, but GPU request OD), negative value means failure.
 * upower fw needs support cocurrent request from M33 and A35.
 * upower fw needs support multiple masters requesting different modes.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */
int upwr_vtm_dva_request_soc(enum work_mode mode, upwr_callb callb)
{
	upwr_volt_dva_req_soc_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_DVA_REQ_SOC);

	txmsg.args.mode = mode;

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}


/**
 * upwr_vtm_dva_domain_request() - request uPower to dva one domain to the given work mode
 * @domain_id: RTD, APD, LPAV, defined in upower_defs.h
 * @mode: OD (over drive) mode, ND (normal drive)  mode, LD (lower drive) mode,
 * type: enum work_mode, defined in upower_defs.h
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to switch one domain to the given work mode.
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_VOLTM as the service group argument.
 *
 * M33/A35 need to check the execute result
 * 0 means success, 1 means hold on(A35 request LD, but GPU request OD), negative value means failure.
 * upower fw needs support cocurrent request from M33 and A35.
 * upower fw needs support multiple masters requesting different modes.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_vtm_dva_domain_request(uint32_t domain_id, enum work_mode mode, upwr_callb callb)
{
	upwr_volt_dva_req_domain_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_DVA_REQ_DOMAIN);

	txmsg.args.mode = mode;
	txmsg.args.domain = domain_id;

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**
 * upwr_vtm_power_measure() - request uPower to measure power consumption
 * @ssel: This field determines which power switches will have their currents sampled to be accounted for a
current/power measurement. Support 0~7
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to measure power consumption
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_VOLTM as the service group argument.
 *
 * The power consumption data read from uPower via
 * the callback argument ret, or written to the variable pointed by retptr,
 * if polling is used (calls upwr_req_status or upwr_poll_req_status).
 * ret (or *retptr) also returns the data written on writes.
 * upower fw needs support cocurrent request from M33 and A35.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */
int upwr_vtm_power_measure(uint32_t ssel, upwr_callb callb)
{
	upwr_volt_pmeter_meas_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_PMETER_MEAS);

	txmsg.hdr.arg = ssel;

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**
 * upwr_vtm_vmeter_measure() - request uPower to measure voltage
 * @vdetsel: Voltage Detector Selector, support 0~3
 * 00b - RTD sense point
   01b - LDO output
   10b - APD domain sense point
   11b - AVD domain sense point
   Refer to upower_defs.h
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to use vmeter to measure voltage
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_VOLTM as the service group argument.
 *
 * The voltage data read from uPower via
 * the callback argument ret, or written to the variable pointed by retptr,
 * if polling is used (calls upwr_req_status or upwr_poll_req_status).
 * ret (or *retptr) also returns the data written on writes.
 * upower fw needs support cocurrent request from M33 and A35.
 *
 * Refer to RM COREREGVL (Core Regulator Voltage Level)
 * uPower return VDETLVL to user, user can calculate the real voltage:
 *
0b000000(0x00) - 0.595833V
0b100110(0x26) - 1.007498V
<value> - 0.595833V + <value>x10.8333mV
0b110010(0x32) - 1.138V
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */
int upwr_vtm_vmeter_measure(uint32_t vdetsel, upwr_callb callb)
{
	upwr_volt_vmeter_meas_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_VMETER_MEAS);

	txmsg.hdr.arg = vdetsel;

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**
 * upwr_vtm_pmic_config() - Configures the SoC PMIC (Power Management IC).
 * @config: pointer to a PMIC-dependent struct defining the PMIC configuration.
 * @size:   size of the struct pointed by config, in bytes.
 * @callb: pointer to the callback called when configurations are applied.
 * NULL if no callback is required.
 *
 * The function requests uPower to change/define the PMIC configuration.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok, -1 if service group is busy,
 *        -2 if the pointer conversion to physical address failed,
 *        -3 if called in an invalid API state.
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 *
 * Sample code:

// The tag value is fixed 0x706D6963, used by uPower PMIC driver to judge if the config data are valid.
#define PMIC_CONFIG_TAG 0x706D6963

// used to define reg_addr_data_arry, user can modify this value
// or you can use variable-length array
// or zero-length array
// or other C language technology skills
#define PMIC_CONFIG_REG_ARRAY_SIZE  8

struct pmic_reg_addr_data
{
    uint32_t reg;       // the target configured register of PMIC IC
    uint32_t data;      // the value of the target configured register
};

struct pmic_config_struct
{
    uint32_t cfg_tag;       // cfg_tag = PMIC_CONFIG_TAG, used to judge if the config data are valid
    uint32_t cfg_reg_size;  // how many registers shall be configured
    struct pmic_reg_addr_data reg_addr_data_array[PMIC_CONFIG_REG_ARRAY_SIZE];
};


    struct pmic_config_struct pmic_config_struct_data;
    pmic_config_struct_data.cfg_tag = PMIC_CONFIG_TAG;
    pmic_config_struct_data.cfg_reg_size = 3;

    pmic_config_struct_data.reg_addr_data_array[0].reg = 0x31 ;
    pmic_config_struct_data.reg_addr_data_array[0].data = 0x83;
    pmic_config_struct_data.reg_addr_data_array[1].reg = 0x36;
    pmic_config_struct_data.reg_addr_data_array[1].data = 0x03;
    pmic_config_struct_data.reg_addr_data_array[2].reg = 0x38;
    pmic_config_struct_data.reg_addr_data_array[2].data = 0x03;

    int size = sizeof(pmic_config_struct_data.cfg_tag) +
                sizeof(pmic_config_struct_data.cfg_reg_size) +
                pmic_config_struct_data.cfg_reg_size *  (sizeof(uint32_t) + sizeof(uint32_t));

    upower_pwm_chng_pmic_config((void *)&pmic_config_struct_data, size);



 *
 * Please must notice that, it will take very long time to finish,
 * beause it will send many I2C commands to pmic chip.
 */

int upwr_vtm_pmic_config(const void* config, uint32_t size, upwr_callb callb)
{
	upwr_pwm_pmiccfg_msg txmsg = {0};
	unsigned long ptrval = 0UL; /* needed for X86, ARM64 */

	if (api_state != UPWR_API_READY)   return -3;
	if (UPWR_SG_BUSY(UPWR_SG_VOLTM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_VOLTM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_VOLTM, UPWR_VTM_PMIC_CONFIG);

	if ((ptrval = (unsigned long)os_ptr2phy(config)) == 0)
		return -2; /* pointer conversion failed */

	txmsg.ptr = upwr_ptr2offset(ptrval,
				    UPWR_SG_VOLTM,
				    (size_t)size,
				    0,
				    config);

	upwr_srv_req(UPWR_SG_VOLTM, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**---------------------------------------------------------------
 * TEMPERATURE MANAGEMENT SERVICE GROUP
 */

/**
 * upwr_tpm_get_temperature() - request uPower to get temperature of one temperature sensor
 * @sensor_id: temperature sensor ID, support 0~2
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to measure temperature
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_TEMPM as the service group argument.
 *
 * The temperature data read from uPower via
 * the callback argument ret, or written to the variable pointed by retptr,
 * if polling is used (calls upwr_req_status or upwr_poll_req_status).
 * ret (or *retptr) also returns the data written on writes.
 *
 * uPower return TSEL to the caller (M33 or A35), caller calculate the real temperature
 * Tsh = 0.000002673049*TSEL[7:0]^3 + 0.0003734262*TSEL[7:0]^2 +
0.4487042*TSEL[7:0] - 46.98694
 *
 * upower fw needs support cocurrent request from M33 and A35.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */
int upwr_tpm_get_temperature(uint32_t sensor_id, upwr_callb callb)
{
	upwr_temp_get_cur_temp_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_TEMPM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_TEMPM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_TEMPM, UPWR_TEMP_GET_CUR_TEMP);

	txmsg.args.sensor_id = sensor_id;

	upwr_srv_req(UPWR_SG_TEMPM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**---------------------------------------------------------------
 * DELAY MANAGEMENT SERVICE GROUP
 */

/**
 * upwr_dlm_get_delay_margin() - request uPower to get delay margin
 * @path: The critical path
 * @index: Use whitch delay meter
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to get delay margin
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_DELAYM as the service group argument.
 *
 * The delay margin data read from uPower via
 * the callback argument ret, or written to the variable pointed by retptr,
 * if polling is used (calls upwr_req_status or upwr_poll_req_status).
 * ret (or *retptr) also returns the data written on writes.
 * upower fw needs support cocurrent request from M33 and A35.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */
int upwr_dlm_get_delay_margin(uint32_t path, uint32_t index, upwr_callb callb)
{
	upwr_dmeter_get_delay_margin_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_DELAYM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_DELAYM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_DELAYM, UPWR_DMETER_GET_DELAY_MARGIN);

	txmsg.args.path = path;
	txmsg.args.index = index;

	upwr_srv_req(UPWR_SG_DELAYM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**
 * upwr_dlm_set_delay_margin() - request uPower to set delay margin
 * @path: The critical path
 * @index: Use whitch delay meter
 * @delay_margin: the value of delay margin
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to set delay margin
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_DELAYM as the service group argument.
 *
 * The result of the corresponding critical path,  failed or not  read from uPower via
 * the callback argument ret, or written to the variable pointed by retptr,
 * if polling is used (calls upwr_req_status or upwr_poll_req_status).
 * ret (or *retptr) also returns the data written on writes.
 * upower fw needs support cocurrent request from M33 and A35.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */
int upwr_dlm_set_delay_margin(uint32_t path, uint32_t index, uint32_t delay_margin, upwr_callb callb)
{
	upwr_dmeter_set_delay_margin_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_DELAYM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_DELAYM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_DELAYM, UPWR_DMETER_SET_DELAY_MARGIN);

	txmsg.args.path = path;
	txmsg.args.index = index;
	txmsg.args.dm = delay_margin;

	upwr_srv_req(UPWR_SG_DELAYM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**
 * upwr_dlm_process_monitor() - request uPower to do process monitor
 * @chain_sel: Chain Cell Type Selection
 * Select the chain to be used for the clock signal generation.
 * Support two types chain cell, 0~1
0b - P4 type delay cells selected
1b - P16 type delay cells selected
 * @callb: response callback pointer; NULL if no callback needed.
 * (polling used instead)
 *
 * The function requests uPower to do process monitor
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_DELAYM as the service group argument.
 *
 * The result of process monitor,  failed or not  read from uPower via
 * the callback argument ret, or written to the variable pointed by retptr,
 * if polling is used (calls upwr_req_status or upwr_poll_req_status).
 * ret (or *retptr) also returns the data written on writes.
 * upower fw needs support cocurrent request from M33 and A35.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */
int upwr_dlm_process_monitor(uint32_t chain_sel, upwr_callb callb)
{
	upwr_pmon_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_DELAYM)) return -1;

	UPWR_USR_CALLB(UPWR_SG_DELAYM, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_DELAYM, UPWR_PMON_REQ);

	txmsg.args.chain_sel = chain_sel;

	upwr_srv_req(UPWR_SG_DELAYM, (uint32_t*)&txmsg, sizeof(txmsg) / 4);

	return 0;
}

/**---------------------------------------------------------------
 * POWER MANAGEMENT SERVICE GROUP
 */

/**
 * upwr_pwm_dom_power_on() - Commands uPower to power on the platform of other
 * domain (not necessarily its core(s)); does not release the core reset.
 * @domain: identifier of the domain to power on. Defined by SoC-dependent type
 * soc_domain_t found in upower_soc_defs.h.
 * @boot_start: must be 1 to start the domain core(s) boot(s), releasing
 * its (their) resets, or 0 otherwise.
 * @pwroncallb: pointer to the callback to be called when the uPower has
 * finished the power on procedure, or NULL if no callback needed
 * (polling used instead).
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded or
 * not.
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -2 if the domain passed is the same as the caller,
 *        -3 if called in an invalid API state
 */

int upwr_pwm_dom_power_on(soc_domain_t      domain,
                          int               boot_start,
                          const upwr_callb  pwroncallb)
{
	upwr_pwm_dom_pwron_msg txmsg = {0};

	if (pwr_domain == domain)          return -2;
	if (api_state != UPWR_API_READY)   return -3;
	if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, (upwr_callb)pwroncallb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_DOM_PWRON);
	txmsg.hdr.domain = (uint32_t)domain;
	txmsg.hdr.arg    = boot_start;

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_boot_start() - Commands uPower to release the reset of other CPU(s),
 * starting their boots.
 * @domain: identifier of the domain to release the reset. Defined by
 * SoC-dependent type soc_domain_t found in upower_soc_defs.h.
 * @bootcallb: pointer to the callback to be called when the uPower has finished
 * the boot start procedure, or NULL if no callback needed
 * (polling used instead).
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded or
 * not.
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * The callback calling doesn't mean the CPUs boots have finished:
 * it only indicates that uPower released the CPUs resets, and can receive
 * other power management service group requests.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -2 if the domain passed is the same as the caller,
 *        -3 if called in an invalid API state
 */

int upwr_pwm_boot_start(soc_domain_t domain, const upwr_callb  bootcallb)
{
	upwr_pwm_boot_start_msg txmsg = {0};

	if (pwr_domain == domain)          return -2;
	if (api_state != UPWR_API_READY)   return -3;
	if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, (upwr_callb)bootcallb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_BOOT);
	txmsg.hdr.domain = (uint32_t)domain;

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_param() - Changes Power Management parameters.
 * @param: pointer to a parameter structure upwr_pwm_param_t, SoC-dependent,
 * defined in upwr_soc_defines.h. NULL may be passed, meaning
 * a request to read the parameter set, in which case it appears in the callback
 * argument ret, or can be pointed by argument retptr in the upwr_req_status and
 * upwr_poll_req_status calls, casted to upwr_pwm_param_t.
 * @callb: response callback pointer; NULL if no callback needed.
 *
 * The return value is always the current parameter set value, either in a
 * read-only request (param = NULL) or after setting a new parameter
 * (non-NULL param).
 *
 * Some parameters may be targeted for a specific domain (see the struct
 * upwr_pwm_param_t definition in upower_soc_defs.h); this call has implicit
 * domain target (the same domain from which is called).
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded or
 * not.
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 */

int upwr_pwm_param(upwr_pwm_param_t* param, const upwr_callb  callb)
{
	upwr_pwm_param_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
        if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_PARAM);

	if (param == NULL) {
		txmsg.hdr.arg = 1;        /* 1= read, txmsg.word2 ignored */
	}
	else {
		txmsg.hdr.arg = 0;        /* 1= write */
		txmsg.word2   = param->R; /* just 1 word, so that's ok */
	}

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_chng_reg_voltage() - Changes the voltage at a given regulator.
 * @reg: regulator id.
 * @volt: voltage value; value unit is SoC-dependent, converted from mV by the
 * macro UPWR_VTM_MILIV, or from micro-Volts by the macro UPWR_VTM_MICROV,
 * both macros in upower_soc_defs.h
 * @callb: response callback pointer; NULL if no callback needed.
 *
 * The function requests uPower to change the voltage of the given regulator.
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate voltage value for the given domain process,
 * temperature and frequency.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_pwm_chng_reg_voltage(uint32_t reg, uint32_t volt, upwr_callb callb)
{
	upwr_pwm_volt_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_VOLT);

	txmsg.args.reg = reg;
	txmsg.args.volt = volt;

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_freq_setup() - Determines the next frequency target for a given
 *                         domain and current frequency.
 * @domain: identifier of the domain to change frequency. Defined by
 * SoC-dependent type soc_domain_t found in upower_soc_defs.h.
 * @rail: the pmic regulator number for the target domain.
 * @target_freq: the target adjust frequency, accurate to MHz
 *
 * refer to upower_defs.h structure definition upwr_pwm_freq_msg
 *
 * In some SoC implementations this may not be needed (argument is not used),
 * if uPower can measure the current frequency by itself.
 * @callb: response callback pointer; NULL if no callback needed.
 *
 * The function informs uPower that the given domain frequency has changed or
 * will change to the given value. uPower firmware will then adjust voltage and
 * bias to cope with the new frequency (if decreasing) or prepare for it
 * (if increasing). The function must be called after decreasing the frequency,
 * and before increasing it. The actual increase in frequency must not occur
 * before the service returns its response.
 *
 * The request is executed if arguments are within range.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_pwm_freq_setup(soc_domain_t domain, uint32_t rail, uint32_t target_freq,
			upwr_callb   callb)
{
	upwr_pwm_freq_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_FREQ);

	txmsg.hdr.domain   = (uint32_t)domain;
	txmsg.args.rail  = rail;
	txmsg.args.target_freq  = target_freq;

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_power_on()- Powers on (not off) one or more switches and ROM/RAMs.
 * @swton: pointer to an array of words that tells which power switches to
 *  turn on. Each word in the array has 1 bit for each switch.
 *  A bit=1 means the respective switch must be turned on,
 *  bit = 0 means it will stay unchanged (on or off).
 *  The pointer may be set to NULL, in which case no switch will be changed,
 *  unless a memory that it feeds must be turned on.
 *  WARNING: swton must not point to the first shared memory address.
 * @memon: pointer to an array of words that tells which memories to turn on.
 *  Each word in the array has 1 bit for each switch.
 *  A bit=1 means the respective memory must be turned on, both array and
 *  periphery logic;
 *  bit = 0 means it will stay unchanged (on or off).
 *  The pointer may be set to NULL, in which case no memory will be changed.
 *  WARNING: memon must not point to the first shared memory address.
 * @callb: pointer to the callback called when configurations are applyed.
 * NULL if no callback is required.
 *
 * The function requests uPower to turn on the PMC and memory array/peripheral
 * switches that control their power, as specified above.
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate memory power state related to overall system state.
 *
 * If a memory is requested to turn on, but the power switch that feeds that
 * memory is not, the power switch will be turned on anyway, if the pwron
 * array is not provided (that is, if pwron is NULL).
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Callback or polling may return error if the service contends for a resource
 * already being used by a power mode transition or an ongoing service in
 * another domain.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok, -1 if service group is busy,
 *        -2 if a pointer conversion to physical address failed,
 *        -3 if called in an invalid API state.
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_pwm_power_on(const uint32_t swton[],
		      const uint32_t memon[],
		      upwr_callb     callb)
{
	upwr_pwm_pwron_msg         txmsg = {0};
	unsigned long              ptrval = 0UL; /* needed for X86, ARM64 */
	size_t                     stsize = 0;

	if (api_state != UPWR_API_READY)   return -3;
        if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_PWR_ON);

	if (swton == NULL) txmsg.ptrs.ptr0 = 0; /* NULL pointer -> 0 offset */
	else if ((ptrval = (unsigned long)os_ptr2phy((void*)swton)) == 0)
		return -2; /* pointer conversion failed */
	else txmsg.ptrs.ptr0 = upwr_ptr2offset(ptrval,
					       UPWR_SG_PWRMGMT,
					       (stsize = UPWR_PMC_SWT_WORDS*4),
					       0,
					       swton);

	if (memon == NULL) txmsg.ptrs.ptr1 = 0; /* NULL pointer -> 0 offset */
	else if ((ptrval = (unsigned long)os_ptr2phy((void*)memon)) == 0)
		return -2; /* pointer conversion failed */
	else txmsg.ptrs.ptr1 = upwr_ptr2offset(ptrval,
					       UPWR_SG_PWRMGMT,
					       UPWR_PMC_MEM_WORDS*4,
					       stsize,
					       memon);

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_power_off()- Powers off (not on) one or more switches and ROM/RAMs.
 * @swtoff: pointer to an array of words that tells which power switches to
 *  turn off. Each word in the array has 1 bit for each switch.
 *  A bit=1 means the respective switch must be turned off,
 *  bit = 0 means it will stay unchanged (on or off).
 *  The pointer may be set to NULL, in which case no switch will be changed.
 *  WARNING: swtoff must not point to the first shared memory address.
 * @memoff: pointer to an array of words that tells which memories to turn off.
 *  Each word in the array has 1 bit for each switch.
 *  A bit=1 means the respective memory must be turned off, both array and
 *  periphery logic;
 *  bit = 0 means it will stay unchanged (on or off).
 *  The pointer may be set to NULL, in which case no memory will be changed,
 *  but notice it may be turned off if the switch that feeds it is powered off.
 *  WARNING: memoff must not point to the first shared memory address.
 * @callb: pointer to the callback called when configurations are applyed.
 * NULL if no callback is required.
 *
 * The function requests uPower to turn off the PMC and memory array/peripheral
 * switches that control their power, as specified above.
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate memory power state related to overall system state.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Callback or polling may return error if the service contends for a resource
 * already being used by a power mode transition or an ongoing service in
 * another domain.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok, -1 if service group is busy,
 *        -2 if a pointer conversion to physical address failed,
 *        -3 if called in an invalid API state.
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_pwm_power_off(const uint32_t swtoff[],
		       const uint32_t memoff[],
		       upwr_callb     callb)
{
	upwr_pwm_pwroff_msg        txmsg = {0};
	unsigned long              ptrval = 0UL; /* needed for X86, ARM64 */
	size_t                     stsize = 0;

	if (api_state != UPWR_API_READY)   return -3;
    if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_PWR_OFF);

	if (swtoff == NULL) txmsg.ptrs.ptr0 = 0; /* NULL pointer -> 0 offset */
	else if ((ptrval = (unsigned long)os_ptr2phy((void*)swtoff)) == 0)
		return -2; /* pointer conversion failed */
	else txmsg.ptrs.ptr0 = upwr_ptr2offset(ptrval,
					       UPWR_SG_PWRMGMT,
					       (stsize = UPWR_PMC_SWT_WORDS*4),
					       0,
					       swtoff);

	if (memoff == NULL) txmsg.ptrs.ptr1 = 0; /* NULL pointer -> 0 offset */
	else if ((ptrval = (unsigned long)os_ptr2phy((void*)memoff)) == 0)
		return -2; /* pointer conversion failed */
	else txmsg.ptrs.ptr1 = upwr_ptr2offset(ptrval,
					       UPWR_SG_PWRMGMT,
					       UPWR_PMC_MEM_WORDS*4,
					       stsize,
					       memoff);

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_mem_retain()- Configures one or more memory power switches to
 * retain its contents, having the power array on, while its peripheral logic
 * is turned off.
 * @mem: pointer to an array of words that tells which memories to put in a
 *  retention state. Each word in the array has 1 bit for each memory.
 *  A bit=1 means the respective memory must be put in retention state,
 *  bit = 0 means it will stay unchanged (retention, fully on or off).
 * @callb: pointer to the callback called when configurations are applyed.
 * NULL if no callback is required.
 *
 * The function requests uPower to turn off the memory peripheral and leave
 * its array on, as specified above.
 * The request is executed if arguments are within range.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Callback or polling may return error if the service contends for a resource
 * already being used by a power mode transition or an ongoing service in
 * another domain.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok, -1 if service group is busy,
 *        -2 if a pointer conversion to physical address failed,
 *        -3 if called in an invalid API state.
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_pwm_mem_retain(const uint32_t mem[], upwr_callb callb)
{
	upwr_pwm_retain_msg        txmsg = {0};
	unsigned long              ptrval = 0UL; /* needed for X86, ARM64 */

	if (api_state != UPWR_API_READY)   return -3;
        if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_RETAIN);

	if ((ptrval = (unsigned long)os_ptr2phy((void*)mem)) == 0)
		return -2; /* pointer conversion failed */

	txmsg.ptr = upwr_ptr2offset(ptrval,
				    UPWR_SG_PWRMGMT,
				    UPWR_PMC_MEM_WORDS*4,
				    0,
				    mem);
	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_chng_switch_mem() - Turns on/off power on one or more PMC switches
 * and memories, including their array and peripheral logic.
 * @swt: pointer to a list of PMC switches to be opened/closed.
 *  The list is structured as an array of struct upwr_switch_board_t
 *  (see upower_defs.h), each one containing a word for up to 32 switches,
 *  one per bit. A bit = 1 means switch closed, bit = 0 means switch open.
 *  struct upwr_switch_board_t also specifies a mask with 1 bit for each
 *  respective switch: mask bit = 1 means the open/close action is applied,
 *  mask bit = 0 means the switch stays unchanged.
 *  The pointer may be set to NULL, in which case no switch will be changed,
 *  unless a memory that it feeds must be turned on.
 *  WARNING: swt must not point to the first shared memory address.
 * @mem: pointer to a list of switches to be turned on/off.
 *  The list is structured as an array of struct upwr_mem_switches_t
 *  (see upower_defs.h), each one containing 2 word for up to 32 switches,
 *  one per bit, one word for the RAM array power switch, other for the
 *  RAM peripheral logic power switch. A bit = 1 means switch closed,
 *  bit = 0 means switch open.
 *  struct upwr_mem_switches_t also specifies a mask with 1 bit for each
 *  respective switch: mask bit = 1 means the open/close action is applied,
 *  mask bit = 0 means the switch stays unchanged.
 *  The pointer may be set to NULL, in which case no memory switch will be
 *  changed, but notice it may be turned off if the switch that feeds it is
 *  powered off.
 *  WARNING: mem must not point to the first shared memory address.
 * @callb: pointer to the callback called when the configurations are applied.
 * NULL if no callback is required.
 *
 * The function requests uPower to change the PMC switches and/or memory power
 * as specified above.
 * The request is executed if arguments are within range, with no protections
 * regarding the adequate switch combinations and overall system state.
 *
 * If a memory is requested to turn on, but the power switch that feeds that
 * memory is not, the power switch will be turned on anyway, if the swt
 * array is not provided (that is, if swt is NULL).
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Callback or polling may return error if the service contends for a resource
 * already being used by a power mode transition or an ongoing service in
 * another domain.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok, -1 if service group is busy.
 *        -2 if a pointer conversion to physical address failed,
 *        -3 if called in an invalid API state.
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_pwm_chng_switch_mem(const struct upwr_switch_board_t  swt[],
			     const struct upwr_mem_switches_t  mem[],
			     upwr_callb                        callb)
{
	upwr_pwm_switch_msg        txmsg = {0};
	unsigned long              ptrval = 0UL; /* needed for X86, ARM64 */
	size_t                     stsize = 0;

	if (api_state != UPWR_API_READY)   return -3;
        if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_SWITCH);

	if (swt == NULL) txmsg.ptrs.ptr0 = 0; /* NULL pointer -> 0 offset */
	else if ((ptrval = (unsigned long)os_ptr2phy((void*)swt)) == 0)
		return -2; /* pointer conversion failed */
	else txmsg.ptrs.ptr0 = upwr_ptr2offset(ptrval,
					       UPWR_SG_PWRMGMT,
					       (stsize = UPWR_PMC_SWT_WORDS*
					                 sizeof(struct
						         upwr_switch_board_t)),
					       0,
					       swt);

	if (mem == NULL) txmsg.ptrs.ptr1 = 0; /* NULL pointer -> 0 offset */
	else if ((ptrval = (unsigned long)os_ptr2phy((void*)mem)) == 0)
		return -2; /* pointer conversion failed */
	else txmsg.ptrs.ptr1 = upwr_ptr2offset(ptrval,
					       UPWR_SG_PWRMGMT,
					       UPWR_PMC_MEM_WORDS*
					       sizeof(struct
							  upwr_mem_switches_t),
					       stsize,
					       mem);

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_pmode_config() - Configures a given power mode in a given domain.
 * @domain: identifier of the domain to which the power mode belongs.
 * Defined by SoC-dependent type soc_domain_t found in upower_soc_defs.h.
 * @pmode: SoC-dependent power mode identifier defined by type abs_pwr_mode_t
 * found in upower_soc_defs.h.
 * @config: pointer to an SoC-dependent struct defining the power mode
 * configuration, found in upower_soc_defs.h.
 * @callb: pointer to the callback called when configurations are applied.
 * NULL if no callback is required.
 *
 * The function requests uPower to change the power mode configuration as
 * specified above. The request is executed if arguments are within range,
 * and complies with SoC-dependent restrictions on value combinations.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok, -1 if service group is busy,
 *        -2 if the pointer conversion to physical address failed,
 *        -3 if called in an invalid API state.
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_pwm_pmode_config(soc_domain_t   domain,
			  abs_pwr_mode_t pmode,
			  const void*    config,
			  upwr_callb     callb)
{
	upwr_pwm_pmode_cfg_msg     txmsg = {0};
	unsigned long              ptrval = 0UL; /* needed for X86, ARM64 */

	if (api_state != UPWR_API_READY)   return -3;
	if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_CONFIG);
	txmsg.hdr.domain = (uint32_t)domain;
	txmsg.hdr.arg    = pmode;

	if ((ptrval = (unsigned long)os_ptr2phy(config)) == 0)
		return -2; /* pointer conversion failed */

	/* upwr_pwm_pmode_config is an exception:
           use the pointer (physical addr) as is */

	txmsg.ptr = (uint32_t)ptrval;

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_reg_config() - Configures the uPower internal regulators.
 * @config: pointer to the struct defining the regulator configuration;
 * the struct upwr_reg_config_t is defined in the file upower_defs.h.
 * @callb: pointer to the callback called when configurations are applied.
 * NULL if no callback is required.
 *
 * The function requests uPower to change/define the configurations of the
 * internal regulators.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * The service may fail with error UPWR_RESP_RESOURCE if a power mode transition
 * or the same service (called from another domain) is executing simultaneously.
 * This error should be interpreted as a "try later" response, as the service
 * will succeed once those concurrent executions are done, and no other is
 * started.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok, -1 if service group is busy,
 *        -2 if the pointer conversion to physical address failed,
 *        -3 if called in an invalid API state.
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_pwm_reg_config(const struct upwr_reg_config_t* config,
			upwr_callb   callb)
{
	upwr_pwm_regcfg_msg txmsg = {0};
	unsigned long ptrval = 0UL; /* needed for X86, ARM64 */

	if (api_state != UPWR_API_READY)   return -3;
	if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_REGCFG);

	if ((ptrval = (unsigned long)os_ptr2phy(config)) == 0)
		return -2; /* pointer conversion failed */

	txmsg.ptr = upwr_ptr2offset(ptrval,
				    UPWR_SG_PWRMGMT,
				    sizeof(struct upwr_reg_config_t),
				    0,
				    config);

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_chng_dom_bias() - Changes the domain bias.
 * @bias: pointer to a domain bias configuration struct (see upower_soc_defs.h).
 * @callb: pointer to the callback called when configurations are applied.
 * NULL if no callback is required.
 *
 * The function requests uPower to change the domain bias configuration as
 * specified above. The request is executed if arguments are within range,
 * with no protections regarding the adequate value combinations and
 * overall system state.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok, -1 if service group is busy,
 *        -3 if called in an invalid API state.
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_pwm_chng_dom_bias(const struct upwr_dom_bias_cfg_t* bias,
			   upwr_callb                        callb)
{
	upwr_pwm_dom_bias_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
	if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_DOM_BIAS);

	/* SoC-dependent argument filling, defined in upower_soc_defs.h */
	UPWR_FILL_DOMBIAS_ARGS(txmsg.hdr.domain, bias, txmsg.args);

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**
 * upwr_pwm_chng_mem_bias()- Changes a ROM/RAM power bias.
 * @domain: identifier of the domain upon which the bias is applied.
 * Defined by SoC-dependent type soc_domain_t found in upower_soc_defs.h.
 * @bias: pointer to a memory bias configuration struct (see upower_soc_defs.h).
 * @callb: pointer to the callback called when configurations are applied.
 * NULL if no callback is required.
 *
 * The function requests uPower to change the memory bias configuration as
 * specified above. The request is executed if arguments are within range,
 * with no protections regarding the adequate value combinations and
 * overall system state.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check the response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok, -1 if service group is busy,
 *        -3 if called in an invalid API state.
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_pwm_chng_mem_bias(soc_domain_t                      domain,
			   const struct upwr_mem_bias_cfg_t* bias,
			   upwr_callb                        callb)
{
	upwr_pwm_mem_bias_msg txmsg = {0};

	if (api_state != UPWR_API_READY)   return -3;
	if (UPWR_SG_BUSY(UPWR_SG_PWRMGMT)) return -1;

	UPWR_USR_CALLB(UPWR_SG_PWRMGMT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_PWRMGMT, UPWR_PWM_MEM_BIAS);

	txmsg.hdr.domain = (uint32_t)domain;

	/* SoC-dependent argument filling, defined in upower_soc_defs.h */
	UPWR_FILL_MEMBIAS_ARGS(bias, txmsg.args);

	upwr_srv_req(UPWR_SG_PWRMGMT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

#ifdef UPWR_VERIFICATION

#include "upower_api_verif.h"

/* VERIFICATION ONLY: NOT TO MAKE INTO THE API SPEC, NOT EVEN INTO upower_api.h
 * upwr_xcp_reg_access()- accesses (read or write) a register inside uPower.
 * @access: pointer to the access specification struct (see upower_soc_defs.h).
 *          access->mask determines the bits of access->data written (if any)
 *          at access->addr; if access->mask = 0, the service performs a read.
 * @callb: pointer to the callback called when configurations are applied.
 * NULL if no callback is required.
 *
 * A callback can be optionally registered, and will be called upon the arrival
 * of the request response from the uPower firmware, telling if it succeeded
 * or not.
 *
 * A callback may not be registered (NULL pointer), in which case polling has
 * to be used to check a service group response, by calling upwr_req_status or
 * upwr_poll_req_status, using UPWR_SG_PWRMGMT as the service group argument.
 *
 * This service has as return value the final register value (read value on a
 * read or the updated value on a write), which is obtained from the callback
 * argument ret or *retptr in case of polling using the functions
 * upwr_req_status or upwr_poll_req_status.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok, -1 if service group is busy,
 *        -2 if the pointer conversion to physical address failed,
 *        -3 if called in an invalid API state.
 * Note that this is not the error response from the request itself:
 * it only tells if the request was successfully sent to the uPower.
 */

int upwr_xcp_reg_access(const struct upwr_reg_access_t* access,
			upwr_callb                      callb)
{
	upwr_xcp_access_msg txmsg = {0};
	unsigned long ptrval = 0UL;; /* needed for X86, ARM64 */

	if (api_state != UPWR_API_READY)           return -3;
	if (UPWR_SG_BUSY(UPWR_SG_EXCEPT))          return -1;

	UPWR_USR_CALLB(UPWR_SG_EXCEPT, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_EXCEPT, UPWR_XCP_SPARE_15);

	if ((ptrval = (unsigned long)os_ptr2phy(access)) == 0)
		return -2; /* pointer conversion failed */

	txmsg.ptr = UPWR_DRAM_SHARED_BASE_ADDR +
		    upwr_ptr2offset(ptrval,
                                    UPWR_SG_EXCEPT,
                                    sizeof(struct upwr_reg_access_t),
                                    0,
				    access);

	upwr_srv_req(UPWR_SG_EXCEPT, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}
#endif /* ifdef UPWR_VERIFICATION */

/**---------------------------------------------------------------
 * DIAGNOSE SERVICE GROUP
 */

/**
 * upwr_dgn_mode() - Sets the diagnostic mode.
 * @mode:  diagnostic mode, which can be:
 *  - UPWR_DGN_NONE:   no diagnostic recorded
 *  - UPWR_DGN_TRACE:  warnings, errors, service, internal activity recorded
 *  - UPWR_DGN_SRVREQ: warnings, errors, service activity recorded
 *  - UPWR_DGN_WARN:   warnings and errors recorded
 *  - UPWR_DGN_ALL:    trace, service, warnings, errors, task state recorded
 *  - UPWR_DGN_ERROR:  only errors recorded
 *  - UPWR_DGN_ALL2ERR: record all until an error occurs,
 *    freeze recording on error
 *  - UPWR_DGN_ALL2HLT: record all until an error occurs,
 *    executes an ebreak on error, which halts the core if enabled through
 *    the debug interface
 * @callb: pointer to the callback called when mode is changed.
 * NULL if no callback is required.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok,
 *        -1 if service group is busy,
 *        -3 if called in an invalid API state
 */

int upwr_dgn_mode(upwr_dgn_mode_t mode, const upwr_callb callb)
{
	upwr_dgn_mode_msg txmsg = {0};

    if (UPWR_SG_BUSY(UPWR_SG_DIAG)) return -1;

	UPWR_USR_CALLB(UPWR_SG_DIAG, callb);

	UPWR_MSG_HDR(txmsg.hdr, UPWR_SG_DIAG, UPWR_DGN_MODE);

	txmsg.hdr.arg = mode;

	upwr_srv_req(UPWR_SG_DIAG, (uint32_t*)&txmsg, sizeof(txmsg)/4);

	return 0;
}

/**---------------------------------------------------------------
 * AUXILIARY CALLS
 */

/**
 * upwr_rom_version() - informs the ROM firwmware version.
 * @vmajor: pointer to the variable to get the firmware major version number.
 * @vminor: pointer to the variable to get the firmware minor version number.
 * @vfixes: pointer to the variable to get the firmware fixes number.
 *
 * Context: no sleep, no locks taken/released.
 * Return: SoC id.
 */

uint32_t upwr_rom_version(uint32_t *vmajor, uint32_t *vminor, uint32_t *vfixes)
{
	uint32_t soc;

	upwr_lock(1);
	soc     = fw_rom_version.soc_id;
	*vmajor = fw_rom_version.vmajor;
	*vminor = fw_rom_version.vminor;
	*vfixes = fw_rom_version.vfixes;
	upwr_lock(0);
	return soc;
}

/**
 * upwr_ram_version() - informs the RAM firwmware version.
 * @vminor: pointer to the variable to get the firmware minor version number.
 * @vfixes: pointer to the variable to get the firmware fixes number.
 *
 * The 3 values returned are 0 if no RAM firmwmare was loaded and initialized.
 *
 * Context: no sleep, no locks taken/released.
 * Return: firmware major version number.
 */

uint32_t upwr_ram_version(uint32_t* vminor, uint32_t* vfixes)
{
	uint32_t vmajor;

	upwr_lock(1);
	vmajor  = fw_ram_version.vmajor;
	*vminor = fw_ram_version.vminor;
	*vfixes = fw_ram_version.vfixes;
	upwr_lock(0);
	return vmajor;
}

/**
 * upwr_req_status() - tells the status of the service group request, and
 *                     returns a request return value, if any.
 * @sg: service group of the request
 * @sgfptr: pointer to the variable that will hold the function id of
 * the last request completed; can be NULL, in which case it is not used.
 * @errptr: pointer to the variable that will hold the error code;
 * can be NULL, in which case it is not used.
 * @retptr: pointer to the variable that will hold the value returned
 * by the last request completed (invalid if the last request completed didn't
 * return any value); can be NULL, in which case it is not used.
 * Note that a request may return a value even if service error is returned
 * (*errptr != UPWR_RESP_OK): that is dependent on the specific service.
 *
 * This call can be used in a poll loop of a service request completion in case
 * a callback was not registered.
 *
 * Context: no sleep, no locks taken/released.
 * Return: service request status: succeeded, failed, or ongoing (busy)
 */

upwr_req_status_t upwr_req_status(upwr_sg_t     sg,
				  uint32_t*     sgfptr,
				  upwr_resp_t*  errptr,
				  int*          retptr)
{
	upwr_req_status_t status;

	upwr_lock(1);
	if (sgfptr != NULL) *sgfptr =(uint32_t)    sg_rsp_msg[sg].hdr.function;
	if (errptr != NULL) *errptr =(upwr_resp_t) sg_rsp_msg[sg].hdr.errcode;
	if (retptr != NULL) *retptr =(int) (sg_rsp_siz[sg] == 2)?
							sg_rsp_msg[sg].word2:
							sg_rsp_msg[sg].hdr.ret;

	status = (sg_busy & (1 << sg))?                        UPWR_REQ_BUSY :
	         (sg_rsp_msg[sg].hdr.errcode == UPWR_RESP_OK)? UPWR_REQ_OK   :
							       UPWR_REQ_ERR  ;
	upwr_lock(0);
	return status;
}

/**
 * upwr_poll_req_status() - polls the status of the service group request, and
 *                          returns a request return value, if any.
 * @sg: service group of the request
 * @sgfptr: pointer to the variable that will hold the function id of
 * the last request completed; can be NULL, in which case it is not used.
 * @errptr: pointer to the variable that will hold the error code;
 * can be NULL, in which case it is not used.
 * @retptr: pointer to the variable that will hold the value returned
 * by the last request completed (invalid if the last request completed didn't
 * return any value); can be NULL, in which case it is not used.
 * Note that a request may return a value even if service error is returned
 * (*errptr != UPWR_RESP_OK): that is dependent on the specific service.
 * @attempts: maximum number of polling attempts; if attempts > 0 and is
 * reached with no service response received, upwr_poll_req_status returns
 * UPWR_REQ_BUSY and variables pointed by sgfptr, retptr and errptr are not
 * updated; if attempts = 0, upwr_poll_req_status waits "forever".
 *
 * This call can be used to poll a service request completion in case a
 * callback was not registered.
 *
 * Context: no sleep, no locks taken/released.
 * Return: service request status: succeeded, failed, or ongoing (busy)
 */

upwr_req_status_t upwr_poll_req_status(upwr_sg_t     sg,
				       uint32_t*     sgfptr,
				       upwr_resp_t*  errptr,
				       int*          retptr,
				       uint32_t      attempts)
{
	uint32_t          i;
	upwr_req_status_t ret;

	if (attempts == 0)
	{
		while ((ret = upwr_req_status(sg, sgfptr, errptr, retptr)) ==
		       UPWR_REQ_BUSY)
		{
			#ifdef UPWR_BLOCK_LEVEL
			WAIT_CLK(10); /* waiting loop must advance clock */
			#endif
		}
		return ret;
	}

	for (i = 0; i < attempts; i++)
	{
		if ((ret = upwr_req_status(sg, sgfptr, errptr, retptr)) !=
		    UPWR_REQ_BUSY) break;

		#ifdef UPWR_BLOCK_LEVEL
		WAIT_CLK(10); /* waiting loop must advance clock */
		#endif
	}
	return ret;
}

/**
 * upwr_alarm_code() - returns the alarm code of the last alarm occurrence.
 *
 * The value returned is not meaningful if no alarm was issued by uPower.
 *
 * Context: no sleep, no locks taken/released.
 * Return: alarm code, as defined by the type upwr_alarm_t in upwr_soc_defines.h
 */

upwr_alarm_t upwr_alarm_code()
{
	return (upwr_alarm_t)(3 & (mu->FSR.R >> 1)); /* FSR[2:1] */
}

/**---------------------------------------------------------------
 * TRANSMIT/RECEIVE PRIMITIVES
 * ---------------------------------------------------------------
 */

/*
 * upwr_copy2tr() - copies a message to the MU TR registers;
 * fill the TR registers before writing TIEN to avoid early interrupts;
 * also, fill them from the higher index to the lowest, so the receive
 * interrupt flag RF[0] will be the last to set, regardless of message size;
 */

void upwr_copy2tr(struct MU_tag* local_mu, const uint32_t* msg, unsigned int size)
{
	for (int i = size - 1; i > -1; i--) local_mu->TR[i].R = msg[i];
}

/**
 * upwr_tx() - queues a message for transmission.
 * @msg : pointer to the message sent.
 * @size: message size in 32-bit words
 * @callback: pointer to a function to be called when transmission done;
 *            can be NULL, in which case no callback is done.
 *
 * This is an auxiliary function used by the rest of the API calls.
 * It is normally not called by the driver code, unless maybe for test purposes.
 *
 * Context: no sleep, no locks taken/released.
 * Return: number of vacant positions left in the trasmission queue, or
 *         -1 if the queue was already full when upwr_tx was called, or
 *         -2 if any argument is invalid (like size off-range)
 */

int upwr_tx(const uint32_t*         msg,
            unsigned int            size,
            UPWR_TX_CALLB_FUNC_T    callback)
{
	if (size >  UPWR_MU_MSG_SIZE) return -2;
	if (size ==                0) return -2;

	if (mu->TSR.R != UPWR_MU_TSR_EMPTY)
		return -1;  /* not all TE bits in 1: some data to send still */

	mu_tx_callb = callback;

	upwr_copy2tr(mu, msg, size);
	mu->TCR.R = 1 << (size - 1);

	mu_tx_pend = 1;

	return 0;
}

/**
 * upwr_rx() - unqueues a received message from the reception queue.
 * @msg: pointer to the message destination buffer.
 * @size: pointer to variable to hold message size in 32-bit words.
 *
 * This is an auxiliary function used by the rest of the API calls.
 * It is normally not called by the driver code, unless maybe for test purposes.
 *
 * Context: no sleep, no locks taken/released.
 * Return: number of messages remaining in the reception queue, or
 *         -1 if the queue was already empty when upwr_rx was called, or
 *         -2 if any argument is invalid (like mu off-range)
 */

int upwr_rx(char *msg, unsigned int *size)
{
	unsigned int len = mu->RSR.R;

	len = (len == 0x0)? 0:
	      (len == 0x1)? 1:
	      #if UPWR_MU_MSG_SIZE > 1
	      (len == 0x3)? 2:
	      #if UPWR_MU_MSG_SIZE > 2
	      (len == 0x7)? 3:
	      #if UPWR_MU_MSG_SIZE > 3
	      (len == 0xF)? 4:
	      #endif
	      #endif
	      #endif
	                    0xFFFFFFFF; /* something wrong */

	if (len  == 0xFFFFFFFF) return -3;
	if ((*size = len) == 0) return -1;

	/* copy the received message to the rx queue,
         * so the interrupts are cleared;*/
    msg_copy(msg, (char *)&mu->RR[0], len);

	mu->RCR.R = 1; /* enable only RR[0] receive interrupt */

	return 0;
}

/**
 * upwr_rx_callback() - sets up a callback for a message receiving event.
 * @callback: pointer to a function to be called when a message arrives;
 *            can be NULL, in which case no callback is done.
 *
 * This is an auxiliary function used by the rest of the API calls.
 * It is normally not called by the driver code, unless maybe for test purposes.
 *
 * Context: no sleep, no locks taken/released.
 * Return: 0 if ok; -2 if any argument is invalid (mu off-range).
 */

int upwr_rx_callback(UPWR_RX_CALLB_FUNC_T    callback)
{
	mu_rx_callb = callback;

	return 0;
}

/**
 * msg_copy() - copies a message.
 * @dest: pointer to the destination message.
 * @src : pointer to the source message.
 * @size: message size in words.
 *
 * This is an auxiliary function used by the rest of the API calls.
 * It is normally not called by the driver code, unless maybe for test purposes.
 *
 * Context: no sleep, no locks taken/released.
 * Return: none (void)
 */

void msg_copy(char *dest, char *src, unsigned int size)
{
    for (uint32_t i = 0; i < size * sizeof(uint32_t); i++)
    {
        dest[i] = src[i];
    }
}

#ifdef  __cplusplus
#ifndef UPWR_NAMESPACE /* extern "C" 'cancels' the effect of namespace */
} /* extern "C" */
#endif
#endif

