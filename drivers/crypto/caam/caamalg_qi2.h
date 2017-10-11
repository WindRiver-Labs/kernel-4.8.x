/*
 * Copyright 2015-2016 Freescale Semiconductor Inc.
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the names of the above-listed copyright holders nor the
 *	 names of any contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _CAAMALG_QI2_H_
#define _CAAMALG_QI2_H_

#include "../../../drivers/staging/fsl-mc/include/fsl_dpaa2_io.h"
#include "../../../drivers/staging/fsl-mc/include/fsl_dpaa2_fd.h"
#include <linux/threads.h>
#include "dpseci.h"
#include "desc_constr.h"

#define DPAA2_CAAM_STORE_SIZE	16
/* NAPI weight *must* be a multiple of the store size. */
#define DPAA2_CAAM_NAPI_WEIGHT	64

/* The congestion entrance threshold was chosen so that on LS2088
 * we support the maximum throughput for the available memory
 */
#define DPAA2_SEC_CONG_ENTRY_THRESH	(128 * 1024 * 1024)
#define DPAA2_SEC_CONG_EXIT_THRESH	(DPAA2_SEC_CONG_ENTRY_THRESH * 9 / 10)

/**
 * dpaa2_caam_priv - driver private data
 * @dpseci_id: DPSECI object unique ID
 * @major_ver: DPSECI major version
 * @minor_ver: DPSECI minor version
 * @dpseci_attr: DPSECI attributes
 * @rx_queue_attr: array of Rx queue attributes
 * @tx_queue_attr: array of Tx queue attributes
 * @cscn_mem: pointer to memory region containing the
 *	dpaa2_cscn struct; it's size is larger than
 *	sizeof(struct dpaa2_cscn) to accommodate alignment
 * @cscn_mem_aligned: pointer to struct dpaa2_cscn; it is computed
 *	as PTR_ALIGN(cscn_mem, DPAA2_CSCN_ALIGN)
 * @cscn_dma: dma address used by the QMAN to write CSCN messages
 * @dev: device associated with the DPSECI object
 * @mc_io: pointer to MC portal's I/O object
 * @ppriv: per CPU pointers to privata data
 */
struct dpaa2_caam_priv {
	int dpsec_id;

	u16 major_ver;
	u16 minor_ver;

	struct dpseci_attr dpseci_attr;
	struct dpseci_rx_queue_attr rx_queue_attr[DPSECI_PRIO_NUM];
	struct dpseci_tx_queue_attr tx_queue_attr[DPSECI_PRIO_NUM];
	int num_pairs;

	/* congestion */
	void *cscn_mem;
	void *cscn_mem_aligned;
	dma_addr_t cscn_dma;

	struct device *dev;
	struct fsl_mc_io *mc_io;

	struct dpaa2_caam_priv_per_cpu __percpu *ppriv;
};

/**
 * dpaa2_caam_priv_per_cpu - per CPU private data
 * @napi: napi structure
 * @net_dev: netdev used by napi
 * @req_fqid: (virtual) request (Tx / enqueue) FQID
 * @rsp_fqid: (virtual) response (Rx / dequeue) FQID
 * @prio: internal queue number - index for dpaa2_caam_priv.*_queue_attr
 * @nctx: notification context of response FQ
 * @store: where dequeued frames are stored
 * @priv: backpointer to dpaa2_caam_priv
 */
struct dpaa2_caam_priv_per_cpu {
	struct napi_struct napi;
	struct net_device net_dev;
	int req_fqid;
	int rsp_fqid;
	int prio;
	struct dpaa2_io_notification_ctx nctx;
	struct dpaa2_io_store *store;
	struct dpaa2_caam_priv *priv;
};

/*
 * The CAAM QI hardware constructs a job descriptor which points
 * to shared descriptor (as pointed by context_a of FQ to CAAM).
 * When the job descriptor is executed by deco, the whole job
 * descriptor together with shared descriptor gets loaded in
 * deco buffer which is 64 words long (each 32-bit).
 *
 * The job descriptor constructed by QI hardware has layout:
 *
 *	HEADER		(1 word)
 *	Shdesc ptr	(1 or 2 words)
 *	SEQ_OUT_PTR	(1 word)
 *	Out ptr		(1 or 2 words)
 *	Out length	(1 word)
 *	SEQ_IN_PTR	(1 word)
 *	In ptr		(1 or 2 words)
 *	In length	(1 word)
 *
 * The shdesc ptr is used to fetch shared descriptor contents
 * into deco buffer.
 *
 * Apart from shdesc contents, the total number of words that
 * get loaded in deco buffer are '8' or '11'. The remaining words
 * in deco buffer can be used for storing shared descriptor.
 */
#define MAX_SDLEN	((CAAM_DESC_BYTES_MAX - DESC_JOB_IO_LEN) / CAAM_CMD_SZ)

/*
 * aead_edesc - s/w-extended aead descriptor
 * @src_nents: number of segments in input scatterlist
 * @dst_nents: number of segments in output scatterlist
 * @iv_dma: dma address of iv for checking continuity and link table
 * @qm_sg_bytes: length of dma mapped h/w link table
 * @qm_sg_dma: bus physical mapped address of h/w link table
 * @assoclen_dma: bus physical mapped address of req->assoclen
 * @sgt: the h/w link table
 */
struct aead_edesc {
	int src_nents;
	int dst_nents;
	dma_addr_t iv_dma;
	int qm_sg_bytes;
	dma_addr_t qm_sg_dma;
	dma_addr_t assoclen_dma;
	struct dpaa2_sg_entry sgt[0];
};

/*
 * ablkcipher_edesc - s/w-extended ablkcipher descriptor
 * @src_nents: number of segments in input scatterlist
 * @dst_nents: number of segments in output scatterlist
 * @iv_dma: dma address of iv for checking continuity and link table
 * @qm_sg_bytes: length of dma mapped qm_sg space
 * @qm_sg_dma: I/O virtual address of h/w link table
 * @sgt: the h/w link table
 */
struct ablkcipher_edesc {
	int src_nents;
	int dst_nents;
	dma_addr_t iv_dma;
	int qm_sg_bytes;
	dma_addr_t qm_sg_dma;
	struct dpaa2_sg_entry sgt[0];
};

/**
 * caam_flc - Flow Context (FLC)
 * @flc: Flow Context options
 * @sh_desc: Shared Descriptor
 */
struct caam_flc {
	u32 flc[16];
	u32 sh_desc[MAX_SDLEN];
} ____cacheline_aligned;

enum optype {
	ENCRYPT = 0,
	DECRYPT,
	GIVENCRYPT,
	NUM_OP
};

/**
 * caam_request - the request structure the driver application should fill while
 *                submitting a job to driver.
 * @fd_flt: Frame list table defining input and output
 *          fd_flt[0] - FLE pointing to output buffer
 *          fd_flt[1] - FLE pointing to input buffer
 * @fd_flt_dma: DMA address for the frame list table
 * @flc: Flow Context
 * @flc_dma: DMA address of Flow Context
 * @op_type: operation type
 * @cbk: Callback function to invoke when job is completed
 * @ctx: arbit context attached with request by the application
 * @edesc: extended descriptor; points to one of {ablkcipher,aead}_edesc
 */
struct caam_request {
	struct dpaa2_fl_entry fd_flt[2];
	dma_addr_t fd_flt_dma;
	struct caam_flc *flc;
	dma_addr_t flc_dma;
	enum optype op_type;
	void (*cbk)(void *ctx, u32 err);
	void *ctx;
	void *edesc;
} ____cacheline_aligned;

/**
 * dpaa2_caam_enqueue() - enqueue a crypto request
 * @dev: device associated with the DPSECI object
 * @req: pointer to caam_request
 */
int dpaa2_caam_enqueue(struct device *dev, struct caam_request *req);

#endif	/* _CAAMALG_QI2_H_ */
