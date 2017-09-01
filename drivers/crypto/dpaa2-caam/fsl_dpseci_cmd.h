/* Copyright 2013-2016 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the above-listed copyright holders nor the
 * names of any contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
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
#ifndef _FSL_DPSECI_CMD_H
#define _FSL_DPSECI_CMD_H

/* DPSECI Version */
#define DPSECI_VER_MAJOR				5
#define DPSECI_VER_MINOR				1

/* Command IDs */

#define DPSECI_CMDID_CLOSE                              0x8001
#define DPSECI_CMDID_OPEN                               0x8091
#define DPSECI_CMDID_CREATE                             0x9092
#define DPSECI_CMDID_DESTROY                            0x9891
#define DPSECI_CMDID_GET_API_VERSION                    0xa091

#define DPSECI_CMDID_ENABLE                             0x0021
#define DPSECI_CMDID_DISABLE                            0x0031
#define DPSECI_CMDID_GET_ATTR                           0x0041
#define DPSECI_CMDID_RESET                              0x0051
#define DPSECI_CMDID_IS_ENABLED                         0x0061

#define DPSECI_CMDID_SET_IRQ_ENABLE                     0x0121
#define DPSECI_CMDID_GET_IRQ_ENABLE                     0x0131
#define DPSECI_CMDID_SET_IRQ_MASK                       0x0141
#define DPSECI_CMDID_GET_IRQ_MASK                       0x0151
#define DPSECI_CMDID_GET_IRQ_STATUS                     0x0161
#define DPSECI_CMDID_CLEAR_IRQ_STATUS                   0x0171

#define DPSECI_CMDID_SET_RX_QUEUE                       0x1941
#define DPSECI_CMDID_GET_RX_QUEUE                       0x1961
#define DPSECI_CMDID_GET_TX_QUEUE                       0x1971
#define DPSECI_CMDID_GET_SEC_ATTR                       0x1981
#define DPSECI_CMDID_GET_SEC_COUNTERS                   0x1991
#define DPSECI_CMDID_SET_OPR				0x19A1
#define DPSECI_CMDID_GET_OPR				0x19B1

#define DPSECI_CMDID_SET_CONGESTION_NOTIFICATION	0x1701
#define DPSECI_CMDID_GET_CONGESTION_NOTIFICATION	0x1711

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_OPEN(cmd, dpseci_id) \
	MC_CMD_OP(cmd, 0, 0,  32, int,      dpseci_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_CREATE(cmd, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  8,  uint8_t,  cfg->priorities[0]);\
	MC_CMD_OP(cmd, 0, 8,  8,  uint8_t,  cfg->priorities[1]);\
	MC_CMD_OP(cmd, 0, 16, 8,  uint8_t,  cfg->priorities[2]);\
	MC_CMD_OP(cmd, 0, 24, 8,  uint8_t,  cfg->priorities[3]);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  cfg->priorities[4]);\
	MC_CMD_OP(cmd, 0, 40, 8,  uint8_t,  cfg->priorities[5]);\
	MC_CMD_OP(cmd, 0, 48, 8,  uint8_t,  cfg->priorities[6]);\
	MC_CMD_OP(cmd, 0, 56, 8,  uint8_t,  cfg->priorities[7]);\
	MC_CMD_OP(cmd, 1, 0,  8,  uint8_t,  cfg->num_tx_queues);\
	MC_CMD_OP(cmd, 1, 8,  8,  uint8_t,  cfg->num_rx_queues);\
	MC_CMD_OP(cmd, 2, 0,  32, uint32_t, cfg->options);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_RSP_IS_ENABLED(cmd, en) \
	MC_RSP_OP(cmd, 0, 0,  1,  int,	    en)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_SET_IRQ_ENABLE(cmd, irq_index, enable_state) \
do { \
	MC_CMD_OP(cmd, 0, 0,  8,  uint8_t,  enable_state); \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_GET_IRQ_ENABLE(cmd, irq_index) \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_RSP_GET_IRQ_ENABLE(cmd, enable_state) \
	MC_RSP_OP(cmd, 0, 0,  8,  uint8_t,  enable_state)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_SET_IRQ_MASK(cmd, irq_index, mask) \
do { \
	MC_CMD_OP(cmd, 0, 0,  32, uint32_t, mask); \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_GET_IRQ_MASK(cmd, irq_index) \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_RSP_GET_IRQ_MASK(cmd, mask) \
	MC_RSP_OP(cmd, 0, 0,  32, uint32_t, mask)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_GET_IRQ_STATUS(cmd, irq_index, status) \
do { \
	MC_CMD_OP(cmd, 0, 0,  32, uint32_t, status);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_RSP_GET_IRQ_STATUS(cmd, status) \
	MC_RSP_OP(cmd, 0, 0,  32, uint32_t,  status)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_CLEAR_IRQ_STATUS(cmd, irq_index, status) \
do { \
	MC_CMD_OP(cmd, 0, 0,  32, uint32_t, status); \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_RSP_GET_ATTRIBUTES(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 0, 0, 32, int,     (attr)->id); \
	MC_RSP_OP(cmd, 1, 0,  8, uint8_t, (attr)->num_tx_queues); \
	MC_RSP_OP(cmd, 1, 8,  8, uint8_t, (attr)->num_rx_queues); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_SET_RX_QUEUE(cmd, queue, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  32, int,      cfg->dest_cfg.dest_id); \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  cfg->dest_cfg.priority); \
	MC_CMD_OP(cmd, 0, 40, 8,  uint8_t,  queue); \
	MC_CMD_OP(cmd, 0, 48, 4,  enum dpseci_dest, cfg->dest_cfg.dest_type); \
	MC_CMD_OP(cmd, 1, 0,  64, uint64_t, cfg->user_ctx); \
	MC_CMD_OP(cmd, 2, 0,  32, uint32_t, cfg->options);\
	MC_CMD_OP(cmd, 2, 32, 1,  int,		cfg->order_preservation_en);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_GET_RX_QUEUE(cmd, queue) \
	MC_CMD_OP(cmd, 0, 40, 8,  uint8_t,  queue)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_RSP_GET_RX_QUEUE(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 0, 0,  32, int,      attr->dest_cfg.dest_id);\
	MC_RSP_OP(cmd, 0, 32, 8,  uint8_t,  attr->dest_cfg.priority);\
	MC_RSP_OP(cmd, 0, 48, 4,  enum dpseci_dest, attr->dest_cfg.dest_type);\
	MC_RSP_OP(cmd, 1, 0,  8,  uint64_t,  attr->user_ctx);\
	MC_RSP_OP(cmd, 2, 0,  32, uint32_t,  attr->fqid);\
	MC_RSP_OP(cmd, 2, 32, 1,  int,		 attr->order_preservation_en);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_GET_TX_QUEUE(cmd, queue) \
	MC_CMD_OP(cmd, 0, 40, 8,  uint8_t,  queue)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_RSP_GET_TX_QUEUE(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 0, 32, 32, uint32_t,  attr->fqid);\
	MC_RSP_OP(cmd, 1, 0,  8,  uint8_t,   attr->priority);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_RSP_GET_SEC_ATTR(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 0,  0, 16, uint16_t,  attr->ip_id);\
	MC_RSP_OP(cmd, 0, 16,  8,  uint8_t,  attr->major_rev);\
	MC_RSP_OP(cmd, 0, 24,  8,  uint8_t,  attr->minor_rev);\
	MC_RSP_OP(cmd, 0, 32,  8,  uint8_t,  attr->era);\
	MC_RSP_OP(cmd, 1,  0,  8,  uint8_t,  attr->deco_num);\
	MC_RSP_OP(cmd, 1,  8,  8,  uint8_t,  attr->zuc_auth_acc_num);\
	MC_RSP_OP(cmd, 1, 16,  8,  uint8_t,  attr->zuc_enc_acc_num);\
	MC_RSP_OP(cmd, 1, 32,  8,  uint8_t,  attr->snow_f8_acc_num);\
	MC_RSP_OP(cmd, 1, 40,  8,  uint8_t,  attr->snow_f9_acc_num);\
	MC_RSP_OP(cmd, 1, 48,  8,  uint8_t,  attr->crc_acc_num);\
	MC_RSP_OP(cmd, 2,  0,  8,  uint8_t,  attr->pk_acc_num);\
	MC_RSP_OP(cmd, 2,  8,  8,  uint8_t,  attr->kasumi_acc_num);\
	MC_RSP_OP(cmd, 2, 16,  8,  uint8_t,  attr->rng_acc_num);\
	MC_RSP_OP(cmd, 2, 32,  8,  uint8_t,  attr->md_acc_num);\
	MC_RSP_OP(cmd, 2, 40,  8,  uint8_t,  attr->arc4_acc_num);\
	MC_RSP_OP(cmd, 2, 48,  8,  uint8_t,  attr->des_acc_num);\
	MC_RSP_OP(cmd, 2, 56,  8,  uint8_t,  attr->aes_acc_num);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_RSP_GET_SEC_COUNTERS(cmd, counters) \
do { \
	MC_RSP_OP(cmd, 0,  0, 64, uint64_t,  counters->dequeued_requests);\
	MC_RSP_OP(cmd, 1,  0, 64, uint64_t,  counters->ob_enc_requests);\
	MC_RSP_OP(cmd, 2,  0, 64, uint64_t,  counters->ib_dec_requests);\
	MC_RSP_OP(cmd, 3,  0, 64, uint64_t,  counters->ob_enc_bytes);\
	MC_RSP_OP(cmd, 4,  0, 64, uint64_t,  counters->ob_prot_bytes);\
	MC_RSP_OP(cmd, 5,  0, 64, uint64_t,  counters->ib_dec_bytes);\
	MC_RSP_OP(cmd, 6,  0, 64, uint64_t,  counters->ib_valid_bytes);\
} while (0)

/*                cmd, param, offset, width, type,      arg_name */
#define DPSECI_RSP_GET_API_VERSION(cmd, major, minor) \
do { \
	MC_RSP_OP(cmd, 0, 0,  16, uint16_t, major);\
	MC_RSP_OP(cmd, 0, 16, 16, uint16_t, minor);\
} while (0)

/*            cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_SET_OPR(cmd, index, options, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 16, 8, uint8_t, index); \
	MC_CMD_OP(cmd, 0, 24, 8, uint8_t, options); \
	MC_CMD_OP(cmd, 1, 24, 8, uint8_t, (cfg)->oloe);\
	MC_CMD_OP(cmd, 1, 32, 8, uint8_t, (cfg)->oeane); \
	MC_CMD_OP(cmd, 1, 40, 8, uint8_t, (cfg)->olws); \
	MC_CMD_OP(cmd, 1, 48, 8, uint8_t, (cfg)->oa); \
	MC_CMD_OP(cmd, 1, 56, 8, uint8_t, (cfg)->oprrws); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSECI_CMD_GET_OPR(cmd, index) \
	MC_CMD_OP(cmd, 0, 16, 8, uint8_t, index)

/*            cmd, param, offset, width, type, arg_name */
#define DPSECI_RSP_GET_OPR(cmd, cfg, qry) \
do { \
	MC_RSP_OP(cmd, 1,  0,  1, char, (qry)->rip); \
	MC_RSP_OP(cmd, 1,  1,  1, char, (qry)->enable); \
	MC_RSP_OP(cmd, 1, 24, 8, uint8_t, (cfg)->oloe);\
	MC_RSP_OP(cmd, 1, 32, 8, uint8_t, (cfg)->oeane); \
	MC_RSP_OP(cmd, 1, 40, 8, uint8_t, (cfg)->olws); \
	MC_RSP_OP(cmd, 1, 48, 8, uint8_t, (cfg)->oa); \
	MC_RSP_OP(cmd, 1, 56, 8, uint8_t, (cfg)->oprrws); \
	MC_RSP_OP(cmd, 2, 0, 16, uint16_t, (qry)->nesn); \
	MC_RSP_OP(cmd, 2, 32, 16, uint16_t, (qry)->ndsn); \
	MC_RSP_OP(cmd, 3, 0, 16, uint16_t, (qry)->ea_tseq); \
	MC_RSP_OP(cmd, 3, 16, 1, char, (qry)->tseq_nlis); \
	MC_RSP_OP(cmd, 3, 32, 16, uint16_t, (qry)->ea_hseq); \
	MC_RSP_OP(cmd, 3, 48, 1, char, (qry)->hseq_nlis); \
	MC_RSP_OP(cmd, 4, 0, 16, uint16_t, (qry)->ea_hptr); \
	MC_RSP_OP(cmd, 4, 32, 16, uint16_t, (qry)->ea_tptr); \
	MC_RSP_OP(cmd, 5, 0, 16, uint16_t, (qry)->opr_vid); \
	MC_RSP_OP(cmd, 5, 32, 16, uint16_t, (qry)->opr_id); \
} while (0)

#define DPSECI_CMD_SET_CONGESTION_NOTIFICATION(cmd, cfg) \
do { \
	MC_CMD_OP(cmd, 0,  0, 32, uint32_t, (cfg)->dest_cfg.dest_id); \
	MC_CMD_OP(cmd, 0,  0, 16, uint16_t, (cfg)->notification_mode); \
	MC_CMD_OP(cmd, 0, 48,  8, uint8_t, (cfg)->dest_cfg.priority); \
	MC_CMD_OP(cmd, 0, 56,  4, enum dpseci_dest, (cfg)->dest_cfg.dest_type);\
	MC_CMD_OP(cmd, 0, 60,  2, enum dpseci_congestion_unit, (cfg)->units); \
	MC_CMD_OP(cmd, 1,  0, 64, uint64_t, (cfg)->message_iova); \
	MC_CMD_OP(cmd, 2,  0, 64, uint64_t, (cfg)->message_ctx); \
	MC_CMD_OP(cmd, 3,  0, 32, uint32_t, (cfg)->threshold_entry); \
	MC_CMD_OP(cmd, 3, 32, 32, uint32_t, (cfg)->threshold_exit); \
} while (0)

#define DPSECI_RSP_GET_CONGESTION_NOTIFICATION(cmd, cfg) \
do { \
	MC_RSP_OP(cmd, 1,  0, 32, uint32_t, (cfg)->dest_cfg.dest_id); \
	MC_RSP_OP(cmd, 1,  0, 16, uint16_t, (cfg)->notification_mode); \
	MC_RSP_OP(cmd, 1, 48,  8, uint8_t, (cfg)->dest_cfg.priority); \
	MC_RSP_OP(cmd, 1, 56,  4, enum dpseci_dest, (cfg)->dest_cfg.dest_type);\
	MC_RSP_OP(cmd, 1, 60,  2, enum dpseci_congestion_unit, (cfg)->units); \
	MC_RSP_OP(cmd, 2,  0, 64, uint64_t, (cfg)->message_iova); \
	MC_RSP_OP(cmd, 3,  0, 64, uint64_t, (cfg)->message_ctx); \
	MC_RSP_OP(cmd, 4,  0, 32, uint32_t, (cfg)->threshold_entry); \
	MC_RSP_OP(cmd, 4, 32, 32, uint32_t, (cfg)->threshold_exit); \
} while (0)

#endif /* _FSL_DPSECI_CMD_H */
