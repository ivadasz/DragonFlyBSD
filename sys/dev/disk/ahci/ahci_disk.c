/*
 * (MPSAFE)
 *
 * Copyright (c) 2009 The DragonFly Project.  All rights reserved.
 *
 * This code is derived from software contributed to The DragonFly Project
 * by Matthew Dillon <dillon@backplane.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of The DragonFly Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific, prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 * Copyright (c) 2007 David Gwynne <dlg@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * $OpenBSD: atascsi.c,v 1.64 2009/02/16 21:19:06 miod Exp $
 */
/*
 * Implement each SATA port as its own SCSI bus on CAM.  This way we can
 * implement future port multiplier features as individual devices on the
 * bus.
 *
 * Much of the cdb<->xa conversion code was taken from OpenBSD, the rest
 * was written natively for DragonFly.
 *
 * NOTE-1: I was temporarily unlocking the port while making the CCB
 *	   callback, to reduce the chance of a deadlock and to improve
 *	   performance by allowing new commands to be queued.
 *
 *	   However, this also creates an opening where another AHCI
 *	   interrupt can come in and execute the ahci_port_intr()
 *	   function, creating a huge mess in the sequencing of the
 *	   chipset.
 *
 *	   So for now we don't do this. XXX
 */

#include "ahci.h"

static void ahci_ata_complete_disk_rw(struct ata_xfer *xa);
static void ahci_ata_complete_disk_synchronize_cache(struct ata_xfer *xa);
static void ahci_submit_task(void *context, int pending);
#if 0
static void ahci_atapi_complete_cmd(struct ata_xfer *xa);
static void ahci_ata_dummy_sense(struct scsi_sense_data *sense_data);
static void ahci_ata_atapi_sense(struct ata_fis_d2h *rfis,
		     struct scsi_sense_data *sense_data);
#endif

static int ahci_cam_probe_disk(struct ahci_port *ap, struct ata_port *at);
static int ahci_cam_probe_atapi(struct ahci_port *ap, struct ata_port *at);
static int ahci_set_xfer(struct ahci_port *ap, struct ata_port *atx);
static void ahci_ata_dummy_done(struct ata_xfer *xa);
static void ata_fix_identify(struct ata_identify *id);
static void ahci_strip_string(const char **basep, int *lenp);

/* disk routines */
static d_open_t ahcid_open;
static d_close_t ahcid_close;
static d_strategy_t ahcid_strategy;
static d_dump_t ahcid_dump;

static struct dev_ops ahcid_ops = {
	{ "ahcid", 0, D_DISK | D_MPSAFE },
	.d_open = ahcid_open,
	.d_close = ahcid_close,
	.d_strategy = ahcid_strategy,
	.d_dump = ahcid_dump,
};

/*
 * Setting the transfer mode is irrelevant for the SATA transport
 * but some (atapi) devices seem to need it anyway.  In addition
 * if we are running through a SATA->PATA converter for some reason
 * beyond my comprehension we might have to set the mode.
 *
 * We only support DMA modes for SATA attached devices, so don't bother
 * with legacy modes.
 */
static int
ahci_set_xfer(struct ahci_port *ap, struct ata_port *atx)
{
	struct ata_port *at;
	struct ata_xfer	*xa;
	u_int16_t mode;
	u_int16_t mask;

	at = atx ? atx : ap->ap_ata[0];

	/*
	 * Figure out the supported UDMA mode.  Ignore other legacy modes.
	 */
	mask = le16toh(at->at_identify.ultradma);
	if ((mask & 0xFF) == 0 || mask == 0xFFFF)
		return(0);
	mask &= 0xFF;
	mode = 0x4F;
	while ((mask & 0x8000) == 0) {
		mask <<= 1;
		--mode;
	}

	/*
	 * SATA atapi devices often still report a dma mode, even though
	 * it is irrelevant for SATA transport.  It is also possible that
	 * we are running through a SATA->PATA converter and seeing the
	 * PATA dma mode.
	 *
	 * In this case the device may require a (dummy) SETXFER to be
	 * sent before it will work properly.
	 */
	xa = ahci_ata_get_xfer(ap, atx);
	xa->complete = ahci_ata_dummy_done;
	xa->fis->command = ATA_C_SET_FEATURES;
	xa->fis->features = ATA_SF_SETXFER;
	xa->fis->flags = ATA_H2D_FLAGS_CMD | at->at_target;
	xa->fis->sector_count = mode;
	xa->flags = ATA_F_PIO | ATA_F_POLL;
	xa->timeout = 1000;
	xa->datalen = 0;
	if (ahci_ata_cmd(xa) != ATA_S_COMPLETE) {
		kprintf("%s: Unable to set dummy xfer mode \n",
			ATANAME(ap, atx));
	} else if (bootverbose) {
		kprintf("%s: Set dummy xfer mode to %02x\n",
			ATANAME(ap, atx), mode);
	}
	ahci_ata_put_xfer(xa);
	return(0);
}

/*
 * Fix byte ordering so buffers can be accessed as
 * strings.
 */
static void
ata_fix_identify(struct ata_identify *id)
{
	u_int16_t	*swap;
	int		i;

	swap = (u_int16_t *)id->serial;
	for (i = 0; i < sizeof(id->serial) / sizeof(u_int16_t); i++)
		swap[i] = bswap16(swap[i]);

	swap = (u_int16_t *)id->firmware;
	for (i = 0; i < sizeof(id->firmware) / sizeof(u_int16_t); i++)
		swap[i] = bswap16(swap[i]);

	swap = (u_int16_t *)id->model;
	for (i = 0; i < sizeof(id->model) / sizeof(u_int16_t); i++)
		swap[i] = bswap16(swap[i]);
}

/*
 * Dummy done callback for xa.
 */
static void
ahci_ata_dummy_done(struct ata_xfer *xa)
{
}

static void
ahci_schedule_next(struct ahci_port *ap)
{
	lwkt_serialize_enter(&ap->ap_slz);
	ap->ap_curcmds--;
	if (ap->ap_curcmds == ap->ap_maxcmds - 1 &&
	    bioq_first(&ap->ap_bioq) != NULL) {
		lwkt_serialize_exit(&ap->ap_slz);
		taskqueue_enqueue(taskqueue_thread[mycpuid], &ap->ap_task);
	} else {
		lwkt_serialize_exit(&ap->ap_slz);
	}
}

/*
 * Completion function for ATA_PORT_T_DISK cache synchronization.
 */
static
void
ahci_ata_complete_disk_synchronize_cache(struct ata_xfer *xa)
{
	struct bio *bio = xa->atascsi_private;
	struct buf *bp = bio->bio_buf;
	struct ahci_port *ap = bio->bio_driver_info;

	switch(xa->state) {
	case ATA_S_COMPLETE:
		bp->b_resid = 0;
		break;
	case ATA_S_ERROR:
		kprintf("%s: synchronize_cache: error\n",
			ATANAME(ap, xa->at));
		bp->b_error = EIO;
		bp->b_resid = bp->b_bcount;
		bp->b_flags |= B_ERROR;
		break;
	case ATA_S_TIMEOUT:
		kprintf("%s: synchronize_cache: timeout\n",
			ATANAME(ap, xa->at));
		bp->b_error = EIO;
		bp->b_resid = bp->b_bcount;
		bp->b_flags |= B_ERROR;
		break;
	default:
		kprintf("%s: synchronize_cache: unknown state %d\n",
			ATANAME(ap, xa->at), xa->state);
		panic("%s: Unknown state", ATANAME(ap, xa->at));
		break;
	}
	ahci_ata_put_xfer(xa);
	biodone(bio);
	ahci_schedule_next(ap);
}

/*
 * Completion function for ATA_PORT_T_DISK I/O
 */
static
void
ahci_ata_complete_disk_rw(struct ata_xfer *xa)
{
	struct bio *bio = xa->atascsi_private;
	struct buf *bp = bio->bio_buf;
	struct ahci_port *ap = bio->bio_driver_info;
	struct ata_fis_h2d *fis;

#if 0
	kprintf("%s: complete: block=%lu count=%u\n",
	    __func__, xa->lba, bp->b_bcount / 512);
#endif

	switch(xa->state) {
	case ATA_S_COMPLETE:
		bp->b_resid = 0;
		break;
	case ATA_S_ERROR:
		fis = xa->fis;
		kprintf("%s: disk_rw: error fiscmd=0x%02x @off=0x%016jx, %zu\n",
			ATANAME(ap, xa->at),
			fis->command,
			(intmax_t)xa->lba * 512,
			xa->datalen);
		bp->b_error = EIO;
		bp->b_resid = bp->b_bcount;
		bp->b_flags |= B_ERROR;
		break;
	case ATA_S_TIMEOUT:
		kprintf("%s: disk_rw: timeout\n", ATANAME(ap, xa->at));
		bp->b_error = EIO;
		bp->b_resid = bp->b_bcount;
		bp->b_flags |= B_ERROR;
		break;
	default:
		kprintf("%s: disk_rw: unknown state %d\n",
			ATANAME(ap, xa->at), xa->state);
		panic("%s: Unknown state", ATANAME(ap, xa->at));
		break;
	}
	ahci_ata_put_xfer(xa);
	devstat_end_transaction_buf(&ap->ap_device_stats, bp);
	biodone(bio);
	ahci_schedule_next(ap);
}

#if 0
/*
 * Completion function for ATA_PORT_T_ATAPI I/O
 *
 * Sense data is returned in the rfis.
 */
static
void
ahci_atapi_complete_cmd(struct ata_xfer *xa)
{
	union ccb *ccb = xa->atascsi_private;
	struct ccb_hdr *ccbh = &ccb->ccb_h;
	struct ahci_port *ap = ccb->ccb_h.sim_priv.entries[0].ptr;
	scsi_cdb_t cdb;

	cdb = (void *)((ccb->ccb_h.flags & CAM_CDB_POINTER) ?
			ccb->csio.cdb_io.cdb_ptr : ccb->csio.cdb_io.cdb_bytes);

	switch(xa->state) {
	case ATA_S_COMPLETE:
		ccbh->status = CAM_REQ_CMP;
		ccb->csio.scsi_status = SCSI_STATUS_OK;
		break;
	case ATA_S_ERROR:
		ccbh->status = CAM_SCSI_STATUS_ERROR;
		ccb->csio.scsi_status = SCSI_STATUS_CHECK_COND;
		ahci_ata_atapi_sense(&xa->rfis, &ccb->csio.sense_data);
		break;
	case ATA_S_TIMEOUT:
		kprintf("%s: cmd %d: timeout\n",
			PORTNAME(ap), cdb->generic.opcode);
		ccbh->status = CAM_CMD_TIMEOUT;
		ccb->csio.scsi_status = SCSI_STATUS_CHECK_COND;
		ahci_ata_dummy_sense(&ccb->csio.sense_data);
		break;
	default:
		kprintf("%s: cmd %d: unknown state %d\n",
			PORTNAME(ap), cdb->generic.opcode, xa->state);
		panic("%s: Unknown state", PORTNAME(ap));
		ccbh->status = CAM_REQ_CMP_ERR;
		break;
	}
	ccb->csio.resid = xa->resid;
	xa->atascsi_private = NULL;
	ahci_ata_put_xfer(xa);
	/*ahci_os_unlock_port(ap); ILLEGAL SEE NOTE-1 AT TOP */
	xpt_done(ccb);
	/*ahci_os_lock_port(ap);*/
}

/*
 * Construct dummy sense data for errors on DISKs
 */
static
void
ahci_ata_dummy_sense(struct scsi_sense_data *sense_data)
{
	sense_data->error_code = SSD_ERRCODE_VALID | SSD_CURRENT_ERROR;
	sense_data->segment = 0;
	sense_data->flags = SSD_KEY_MEDIUM_ERROR;
	sense_data->info[0] = 0;
	sense_data->info[1] = 0;
	sense_data->info[2] = 0;
	sense_data->info[3] = 0;
	sense_data->extra_len = 0;
}

/*
 * Construct atapi sense data for errors on ATAPI
 *
 * The ATAPI sense data is stored in the passed rfis and must be converted
 * to SCSI sense data.
 */
static
void
ahci_ata_atapi_sense(struct ata_fis_d2h *rfis,
		     struct scsi_sense_data *sense_data)
{
	sense_data->error_code = SSD_ERRCODE_VALID | SSD_CURRENT_ERROR;
	sense_data->segment = 0;
	sense_data->flags = (rfis->error & 0xF0) >> 4;
	if (rfis->error & 0x04)
		sense_data->flags |= SSD_KEY_ILLEGAL_REQUEST;
	if (rfis->error & 0x02)
		sense_data->flags |= SSD_EOM;
	if (rfis->error & 0x01)
		sense_data->flags |= SSD_ILI;
	sense_data->info[0] = 0;
	sense_data->info[1] = 0;
	sense_data->info[2] = 0;
	sense_data->info[3] = 0;
	sense_data->extra_len = 0;
}
#endif

static
void
ahci_strip_string(const char **basep, int *lenp)
{
	const char *base = *basep;
	int len = *lenp;

	while (len && (*base == 0 || *base == ' ')) {
		--len;
		++base;
	}
	while (len && (base[len-1] == 0 || base[len-1] == ' '))
		--len;
	*basep = base;
	*lenp = len;
}

int
ahci_disks_attach(struct ahci_port *ap)
{
	int error;

	ap->ap_flags |= AP_F_BUS_REGISTERED;
	bioq_init(&ap->ap_bioq);

	if (ap->ap_probe == ATA_PROBE_NEED_IDENT)
		error = ahci_cam_probe(ap, NULL);
	else
		error = 0;
	if (error) {
		ahci_disks_detach(ap);
		return (EIO);
	}
	ap->ap_flags |= AP_F_CAM_ATTACHED;

	return(0);
}

void
ahci_disks_detach(struct ahci_port *ap)
{
	/* XXX */
}

/*
 * Once the AHCI port has been attached we need to probe for a device or
 * devices on the port and setup various options.
 *
 * If at is NULL we are probing the direct-attached device on the port,
 * which may or may not be a port multiplier.
 */
int
ahci_cam_probe(struct ahci_port *ap, struct ata_port *atx)
{
	struct ata_port	*at;
	struct ata_xfer	*xa;
	u_int64_t	capacity;
	u_int64_t	capacity_bytes;
	int		model_len;
	int		firmware_len;
	int		serial_len;
	int		error;
	int		devncqdepth;
	int		i;
	const char	*model_id;
	const char	*firmware_id;
	const char	*serial_id;
	const char	*wcstr;
	const char	*rastr;
	const char	*scstr;
	const char	*type;

	error = EIO;

	/*
	 * A NULL atx indicates a probe of the directly connected device.
	 * A non-NULL atx indicates a device connected via a port multiplier.
	 * We need to preserve atx for calls to ahci_ata_get_xfer().
	 *
	 * at is always non-NULL.  For directly connected devices we supply
	 * an (at) pointing to target 0.
	 */
	if (atx == NULL) {
		at = ap->ap_ata[0];	/* direct attached - device 0 */
		if (ap->ap_type == ATA_PORT_T_PM) {
			kprintf("%s: Found Port Multiplier\n",
				ATANAME(ap, atx));
			return (0);
		}
		at->at_type = ap->ap_type;
	} else {
		at = atx;
		if (atx->at_type == ATA_PORT_T_PM) {
			kprintf("%s: Bogus device, reducing port count to %d\n",
				ATANAME(ap, atx), atx->at_target);
			if (ap->ap_pmcount > atx->at_target)
				ap->ap_pmcount = atx->at_target;
			goto err;
		}
	}
	if (ap->ap_type == ATA_PORT_T_NONE)
		goto err;
	if (at->at_type == ATA_PORT_T_NONE)
		goto err;

	/*
	 * Issue identify, saving the result
	 */
	xa = ahci_ata_get_xfer(ap, atx);
	xa->complete = ahci_ata_dummy_done;
	xa->data = &at->at_identify;
	xa->datalen = sizeof(at->at_identify);
	xa->flags = ATA_F_READ | ATA_F_PIO | ATA_F_POLL;
	xa->fis->flags = ATA_H2D_FLAGS_CMD | at->at_target;

	switch(at->at_type) {
	case ATA_PORT_T_DISK:
		xa->fis->command = ATA_C_IDENTIFY;
		type = "DISK";
		break;
	case ATA_PORT_T_ATAPI:
		xa->fis->command = ATA_C_ATAPI_IDENTIFY;
		xa->flags |= ATA_F_AUTOSENSE;
		type = "ATAPI";
		break;
	default:
		xa->fis->command = ATA_C_ATAPI_IDENTIFY;
		type = "UNKNOWN(ATAPI?)";
		break;
	}
	xa->fis->features = 0;
	xa->fis->device = 0;
	xa->timeout = 1000;

	if (ahci_ata_cmd(xa) != ATA_S_COMPLETE) {
		kprintf("%s: Detected %s device but unable to IDENTIFY\n",
			ATANAME(ap, atx), type);
		ahci_ata_put_xfer(xa);
		goto err;
	}
	ahci_ata_put_xfer(xa);

	ata_fix_identify(&at->at_identify);

	if (at->at_type == ATA_PORT_T_DISK && at->at_identify.nomrota_rate == 1)
		type = "SSD";

	/*
	 * Read capacity using SATA probe info.
	 */
	if (le16toh(at->at_identify.cmdset83) & 0x0400) {
		/* LBA48 feature set supported */
		capacity = 0;
		for (i = 3; i >= 0; --i) {
			capacity <<= 16;
			capacity +=
			    le16toh(at->at_identify.addrsecxt[i]);
		}
	} else {
		capacity = le16toh(at->at_identify.addrsec[1]);
		capacity <<= 16;
		capacity += le16toh(at->at_identify.addrsec[0]);
	}
	if (capacity == 0)
		capacity = 1024 * 1024 / 512;
	at->at_capacity = capacity;
	if (atx == NULL)
		ap->ap_probe = ATA_PROBE_GOOD;

	capacity_bytes = capacity * 512;
	ap->ap_curcmds = 0;
	lwkt_serialize_init(&ap->ap_slz);
	TASK_INIT(&ap->ap_task, 0, ahci_submit_task, ap);

	if (ap->ap_sc->sc_ncmds > 1)
		ap->ap_maxcmds = ap->ap_sc->sc_ncmds - 1;
	else
		ap->ap_maxcmds = ap->ap_sc->sc_ncmds;
	/*
	 * Negotiate NCQ, throw away any ata_xfer's beyond the negotiated
	 * number of slots and limit the number of CAM ccb's to one less
	 * so we always have a slot available for recovery.
	 *
	 * NCQ is not used if ap_ncqdepth is 1 or the host controller does
	 * not support it, and in that case the driver can handle extra
	 * ccb's.
	 *
	 * NCQ is currently used only with direct-attached disks.  It is
	 * not used with port multipliers or direct-attached ATAPI devices.
	 *
	 * Remember at least one extra CCB needs to be reserved for the
	 * error ccb.
	 */
	if ((ap->ap_sc->sc_cap & AHCI_REG_CAP_SNCQ) &&
	    ap->ap_type == ATA_PORT_T_DISK &&
	    (le16toh(at->at_identify.satacap) & (1 << 8))) {
		at->at_ncqdepth = (le16toh(at->at_identify.qdepth) & 0x1F) + 1;
		devncqdepth = at->at_ncqdepth;
		if (at->at_ncqdepth > ap->ap_sc->sc_ncmds)
			at->at_ncqdepth = ap->ap_sc->sc_ncmds;
		if (at->at_ncqdepth > 1) {
			for (i = 0; i < ap->ap_sc->sc_ncmds; ++i) {
				xa = ahci_ata_get_xfer(ap, atx);
				if (xa->tag < at->at_ncqdepth) {
					xa->state = ATA_S_COMPLETE;
					ahci_ata_put_xfer(xa);
				}
			}
#if 0
			if (at->at_ncqdepth >= ap->ap_sc->sc_ncmds) {
				cam_sim_set_max_tags(ap->ap_sim,
						     at->at_ncqdepth - 1);
			}
#else
			ap->ap_maxcmds = at->at_ncqdepth - 1;
#endif
		}
	} else {
		devncqdepth = 0;
	}

	model_len = sizeof(at->at_identify.model);
	model_id = at->at_identify.model;
	ahci_strip_string(&model_id, &model_len);

	firmware_len = sizeof(at->at_identify.firmware);
	firmware_id = at->at_identify.firmware;
	ahci_strip_string(&firmware_id, &firmware_len);

	serial_len = sizeof(at->at_identify.serial);
	serial_id = at->at_identify.serial;
	ahci_strip_string(&serial_id, &serial_len);

	/*
	 * Generate informative strings.
	 *
	 * NOTE: We do not automatically set write caching, lookahead,
	 *	 or the security state for ATAPI devices.
	 */
	if (at->at_identify.cmdset82 & ATA_IDENTIFY_WRITECACHE) {
		if (at->at_identify.features85 & ATA_IDENTIFY_WRITECACHE)
			wcstr = "enabled";
		else if (at->at_type == ATA_PORT_T_ATAPI)
			wcstr = "disabled";
		else
			wcstr = "enabling";
	} else {
		    wcstr = "notsupp";
	}

	if (at->at_identify.cmdset82 & ATA_IDENTIFY_LOOKAHEAD) {
		if (at->at_identify.features85 & ATA_IDENTIFY_LOOKAHEAD)
			rastr = "enabled";
		else if (at->at_type == ATA_PORT_T_ATAPI)
			rastr = "disabled";
		else
			rastr = "enabling";
	} else {
		    rastr = "notsupp";
	}

	if (at->at_identify.cmdset82 & ATA_IDENTIFY_SECURITY) {
		if (at->at_identify.securestatus & ATA_SECURE_FROZEN)
			scstr = "frozen";
		else if (at->at_type == ATA_PORT_T_ATAPI)
			scstr = "unfrozen";
		else
			scstr = "freezing";
	} else {
		    scstr = "notsupp";
	}

	kprintf("%s: Found %s \"%*.*s %*.*s\" serial=\"%*.*s\"\n"
		"%s: tags=%d/%d satacap=%04x satacap2=%04x satafea=%04x NCQ=%s "
		"capacity=%lld.%02dMB\n",

		ATANAME(ap, atx),
		type,
		model_len, model_len, model_id,
		firmware_len, firmware_len, firmware_id,
		serial_len, serial_len, serial_id,

		ATANAME(ap, atx),
		devncqdepth, ap->ap_sc->sc_ncmds,
		at->at_identify.satacap,
		at->at_identify.satacap2,
		at->at_identify.satafsup,
		(at->at_ncqdepth > 1 ? "YES" : "NO"),
		(long long)capacity_bytes / (1024 * 1024),
		(int)(capacity_bytes % (1024 * 1024)) * 100 / (1024 * 1024)
	);
	kprintf("%s: f85=%04x f86=%04x f87=%04x WC=%s RA=%s SEC=%s\n",
		ATANAME(ap, atx),
		at->at_identify.features85,
		at->at_identify.features86,
		at->at_identify.features87,
		wcstr,
		rastr,
		scstr
	);

	/*
	 * Additional type-specific probing
	 */
	switch(at->at_type) {
	case ATA_PORT_T_DISK:
		error = ahci_cam_probe_disk(ap, atx);
		break;
	case ATA_PORT_T_ATAPI:
		error = ahci_cam_probe_atapi(ap, atx);
		break;
	default:
		error = EIO;
		break;
	}
	if (at->at_type == ATA_PORT_T_DISK) {
		struct disk_info info;
		char serialno[21];

		memcpy(serialno, serial_id, serial_len);
		serialno[serial_len] = '\0';
		bzero(&info, sizeof(info));
		info.d_media_blksize = 512;
		info.d_media_blocks = capacity;
		/* XXX Correctly determine disk geometry. */
		info.d_secpertrack = 1024;
		info.d_nheads = 1;
		info.d_secpercyl = info.d_secpertrack * info.d_nheads;
		info.d_ncylinders =  (u_int)(capacity / info.d_secpercyl);
		info.d_type = DTYPE_SCSI;
		info.d_serialno = &serialno[0];

		device_printf(ap->ap_sc->sc_dev, "Using geometry secpertrac=%u heads=%u secpercyl=%u ncylinders=%u\n", info.d_secpertrack, info.d_nheads, info.d_secpercyl, info.d_ncylinders);

		/* XXX Fix device unit selection. */
		devstat_add_entry(&ap->ap_device_stats, "ahcid",
		    device_get_unit(ap->ap_sc->sc_dev), 512,
		    DEVSTAT_NO_ORDERED_TAGS,
		    DEVSTAT_TYPE_DIRECT | DEVSTAT_TYPE_IF_OTHER,
		    DEVSTAT_PRIORITY_DISK);
		ap->ap_cdev = disk_create(device_get_unit(ap->ap_sc->sc_dev), &ap->ap_disk,
		    &ahcid_ops);
		ap->ap_cdev->si_drv1 = ap;
		ap->ap_cdev->si_iosize_max = MAXPHYS;

		disk_setdiskinfo(&ap->ap_disk, &info);
	}
err:
	if (error) {
		at->at_probe = ATA_PROBE_FAILED;
		if (atx == NULL)
			ap->ap_probe = at->at_probe;
	} else {
		at->at_probe = ATA_PROBE_GOOD;
		if (atx == NULL)
			ap->ap_probe = at->at_probe;
	}
	return (error);
}

void
ahci_cam_changed(struct ahci_port *ap, struct ata_port *atx, int found)
{
}

/*
 * DISK-specific probe after initial ident
 */
static int
ahci_cam_probe_disk(struct ahci_port *ap, struct ata_port *atx)
{
	struct ata_port *at;
	struct ata_xfer	*xa;

	at = atx ? atx : ap->ap_ata[0];

	/*
	 * Set dummy xfer mode
	 */
	ahci_set_xfer(ap, atx);

	/*
	 * Enable write cache if supported
	 *
	 * NOTE: "WD My Book" external disk devices have a very poor
	 *	 daughter board between the the ESATA and the HD.  Sending
	 *	 any ATA_C_SET_FEATURES commands will break the hardware port
	 *	 with a fatal protocol error.  However, this device also
	 *	 indicates that WRITECACHE is already on and READAHEAD is
	 *	 not supported so we avoid the issue.
	 */
	if ((at->at_identify.cmdset82 & ATA_IDENTIFY_WRITECACHE) &&
	    (at->at_identify.features85 & ATA_IDENTIFY_WRITECACHE) == 0) {
		xa = ahci_ata_get_xfer(ap, atx);
		xa->complete = ahci_ata_dummy_done;
		xa->fis->command = ATA_C_SET_FEATURES;
		xa->fis->features = ATA_SF_WRITECACHE_EN;
		/* xa->fis->features = ATA_SF_LOOKAHEAD_EN; */
		xa->fis->flags = ATA_H2D_FLAGS_CMD | at->at_target;
		xa->fis->device = 0;
		xa->flags = ATA_F_PIO | ATA_F_POLL;
		xa->timeout = 1000;
		xa->datalen = 0;
		if (ahci_ata_cmd(xa) == ATA_S_COMPLETE)
			at->at_features |= ATA_PORT_F_WCACHE;
		else
			kprintf("%s: Unable to enable write-caching\n",
				ATANAME(ap, atx));
		ahci_ata_put_xfer(xa);
	}

	/*
	 * Enable readahead if supported
	 */
	if ((at->at_identify.cmdset82 & ATA_IDENTIFY_LOOKAHEAD) &&
	    (at->at_identify.features85 & ATA_IDENTIFY_LOOKAHEAD) == 0) {
		xa = ahci_ata_get_xfer(ap, atx);
		xa->complete = ahci_ata_dummy_done;
		xa->fis->command = ATA_C_SET_FEATURES;
		xa->fis->features = ATA_SF_LOOKAHEAD_EN;
		xa->fis->flags = ATA_H2D_FLAGS_CMD | at->at_target;
		xa->fis->device = 0;
		xa->flags = ATA_F_PIO | ATA_F_POLL;
		xa->timeout = 1000;
		xa->datalen = 0;
		if (ahci_ata_cmd(xa) == ATA_S_COMPLETE)
			at->at_features |= ATA_PORT_F_RAHEAD;
		else
			kprintf("%s: Unable to enable read-ahead\n",
				ATANAME(ap, atx));
		ahci_ata_put_xfer(xa);
	}

	/*
	 * FREEZE LOCK the device so malicious users can't lock it on us.
	 * As there is no harm in issuing this to devices that don't
	 * support the security feature set we just send it, and don't bother
	 * checking if the device sends a command abort to tell us it doesn't
	 * support it
	 */
	if ((at->at_identify.cmdset82 & ATA_IDENTIFY_SECURITY) &&
	    (at->at_identify.securestatus & ATA_SECURE_FROZEN) == 0 &&
	    (AhciNoFeatures & (1 << ap->ap_num)) == 0) {
		xa = ahci_ata_get_xfer(ap, atx);
		xa->complete = ahci_ata_dummy_done;
		xa->fis->command = ATA_C_SEC_FREEZE_LOCK;
		xa->fis->flags = ATA_H2D_FLAGS_CMD | at->at_target;
		xa->flags = ATA_F_PIO | ATA_F_POLL;
		xa->timeout = 1000;
		xa->datalen = 0;
		if (ahci_ata_cmd(xa) == ATA_S_COMPLETE)
			at->at_features |= ATA_PORT_F_FRZLCK;
		else
			kprintf("%s: Unable to set security freeze\n",
				ATANAME(ap, atx));
		ahci_ata_put_xfer(xa);
	}

	return (0);
}

/*
 * ATAPI-specific probe after initial ident
 */
static int
ahci_cam_probe_atapi(struct ahci_port *ap, struct ata_port *atx)
{
	ahci_set_xfer(ap, atx);
	return(0);
}

static int
ahcid_open(struct dev_open_args *ap __unused)
{
	return (0);
}

static int
ahcid_close(struct dev_close_args *ap __unused)
{
	return (0);
}

static void
ahcid_submit_disk_io(struct ahci_port *ap, struct bio *bio)
{
	struct ata_port *at = ap->ap_ata[0];
	struct buf *bp = bio->bio_buf;
	struct ata_xfer *xa;
	struct ata_fis_h2d *fis;
	off_t block;

	/* XXX Not passing NULL for direct attached disk! Doesn't seem to matter... */
	xa = ahci_ata_get_xfer(ap, at);
	fis = xa->fis;

	if (bp->b_cmd == BUF_CMD_FLUSH) {
#if 0
		device_printf(port->ap_sc->sc_dev, "%s: cmd=BUF_CMD_FLUSH\n",
		    __func__);
#endif
		fis->flags = ATA_H2D_FLAGS_CMD;
		fis->command = ATA_C_FLUSH_CACHE;
		fis->device = 0;
		xa->timeout = 45000;
		xa->datalen = 0;
		xa->flags = 0;
		xa->complete = ahci_ata_complete_disk_synchronize_cache;
		goto out;
	}

	devstat_start_transaction(&ap->ap_device_stats);
	block = bio->bio_offset / 512;
#if 0
	device_printf(port->ap_sc->sc_dev, "%s: cmd=%s block=%lu count=%u\n",
	    __func__, bp->b_cmd == BUF_CMD_READ ? "read" : "write", block,
	    bp->b_bcount / 512);
#endif
	fis->flags = ATA_H2D_FLAGS_CMD;
	fis->lba_low = (uint8_t)block;
	fis->lba_mid = (uint8_t)(block >> 8);
	fis->lba_high = (uint8_t)(block >> 16);
	fis->device = ATA_H2D_DEVICE_LBA;

	xa->flags = (bp->b_cmd == BUF_CMD_READ) ? ATA_F_READ : ATA_F_WRITE;

	/*
	 * NCQ only for direct-attached disks, do not currently
	 * try to use NCQ with port multipliers.
	 */
	if (at->at_ncqdepth > 1 &&
	    ap->ap_type == ATA_PORT_T_DISK &&
	    (ap->ap_sc->sc_cap & AHCI_REG_CAP_SNCQ)) {
		/*
		 * Use NCQ - always uses 48 bit addressing
		 */
		xa->flags |= ATA_F_NCQ;
		fis->command = (xa->flags & ATA_F_WRITE) ?
				ATA_C_WRITE_FPDMA : ATA_C_READ_FPDMA;
		fis->lba_low_exp = (u_int8_t)(block >> 24);
		fis->lba_mid_exp = (u_int8_t)(block >> 32);
		fis->lba_high_exp = (u_int8_t)(block >> 40);
		fis->sector_count = xa->tag << 3;
		fis->features = (u_int8_t)(bp->b_bcount / 512);
		fis->features_exp = (u_int8_t)((bp->b_bcount / 512) >> 8);
	} else if (bp->b_bcount / 512 > 0x100 || block > 0x0FFFFFFFU) {
		/*
		 * Use LBA48
		 */
		fis->command = (bp->b_cmd == BUF_CMD_READ) ?
		    ATA_C_READDMA_EXT : ATA_C_WRITEDMA_EXT;
		fis->lba_low_exp = (u_int8_t)(block >> 24);
		fis->lba_mid_exp = (u_int8_t)(block >> 32);
		fis->lba_high_exp = (u_int8_t)(block >> 40);
		fis->sector_count = (u_int8_t)(bp->b_bcount / 512);
		fis->sector_count_exp = (u_int8_t)((bp->b_bcount / 512) >> 8);
	} else {
		/*
		 * Use LBA
		 *
		 * NOTE: 256 sectors is supported, stored as 0.
		 */
		fis->command = (bp->b_cmd == BUF_CMD_READ) ?
		    ATA_C_READDMA: ATA_C_WRITEDMA;
		fis->device |= (uint8_t)(block >> 24) & 0x0F;
		fis->sector_count = (u_int8_t)(bp->b_bcount / 512);
	}

	xa->lba = block;
	xa->data = bp->b_data;
	xa->datalen = bp->b_bcount;
	xa->complete = ahci_ata_complete_disk_rw;
	/* XXX The default timeout in cam is 60 seconds. */
	xa->timeout = 60000;

out:
	bio->bio_driver_info = ap;
	xa->atascsi_private = bio;
	ahci_os_lock_port(ap);
	xa->fis->flags |= at->at_target;
	ahci_ata_cmd(xa);
	ahci_os_unlock_port(ap);
}

static void
ahci_submit_task(void *context, int pending __unused)
{
	struct ahci_port *ap = context;
	struct bio *bio;
	int done = 0;

	do {
		lwkt_serialize_enter(&ap->ap_slz);
		if (ap->ap_curcmds < ap->ap_maxcmds) {
			bio = bioq_takefirst(&ap->ap_bioq);
			if (bio == NULL) {
				lwkt_serialize_exit(&ap->ap_slz);
				done = 1;
			} else {
				ap->ap_curcmds++;
				if (ap->ap_curcmds == ap->ap_maxcmds)
					done = 1;
				lwkt_serialize_exit(&ap->ap_slz);
				ahcid_submit_disk_io(ap, bio);
			}
		} else {
			lwkt_serialize_exit(&ap->ap_slz);
			done = 1;
		}
	} while (!done);
}

static int
ahcid_strategy(struct dev_strategy_args *ap)
{
	struct ahci_port *port = (void *)ap->a_head.a_dev->si_drv1;
	struct bio *bio = ap->a_bio;
	struct buf *bp = bio->bio_buf;
	int done = 0;

	if (bp->b_cmd != BUF_CMD_READ && bp->b_cmd != BUF_CMD_WRITE &&
	    bp->b_cmd != BUF_CMD_FLUSH) {
		bp->b_error = EIO;
		bp->b_resid = bp->b_bcount;
		bp->b_flags |= B_ERROR;
		biodone(bio);
		return 0;
	}

	lwkt_serialize_enter(&port->ap_slz);
	bioqdisksort(&port->ap_bioq, bio);
	do {
		if (port->ap_curcmds < port->ap_maxcmds) {
			bio = bioq_takefirst(&port->ap_bioq);
			if (bio == NULL) {
				lwkt_serialize_exit(&port->ap_slz);
				done = 1;
			} else {
				port->ap_curcmds++;
				if (port->ap_curcmds == port->ap_maxcmds)
					done = 1;
				lwkt_serialize_exit(&port->ap_slz);
				ahcid_submit_disk_io(port, bio);
			}
		} else {
			lwkt_serialize_exit(&port->ap_slz);
			done = 1;
		}
		if (!done)
			lwkt_serialize_enter(&port->ap_slz);
	} while (!done);

	return 0;
}

static int
ahcid_dump(struct dev_dump_args *ap)
{
	return EIO;
}
