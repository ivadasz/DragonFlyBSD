#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/serialize.h>
#include <sys/thread2.h>

#include <machine/md_var.h>
#include <machine/specialreg.h>

#include <bus/pci/pcivar.h>
#include "pcidevs.h"

#include "iosf_mbi.h"

struct iosf_softc {
	struct device		*sc_dev;
};

static void	iosf_identify(driver_t *, struct device *);
static int	iosf_probe(struct device *);
static int	iosf_attach(struct device *);
static int	iosf_detach(struct device *);

static device_method_t iosf_methods[] = {
	DEVMETHOD(device_identify,	iosf_identify),
	DEVMETHOD(device_probe,		iosf_probe),
	DEVMETHOD(device_attach,	iosf_attach),
	DEVMETHOD(device_detach,	iosf_detach),

	DEVMETHOD_END
};

static driver_t iosf_driver = {
	"iosf",
	iosf_methods,
	sizeof(struct iosf_softc)
};

static devclass_t iosf_devclass;
static struct lwkt_serialize iosf_slz = LWKT_SERIALIZE_INITIALIZER;

DRIVER_MODULE(iosf, hostb, iosf_driver, iosf_devclass, NULL, NULL);
MODULE_VERSION(iosf, 1);

device_t iosf_dev = NULL;

static uint32_t
iosf_mbi_form_mcr(uint8_t op, uint8_t port, uint8_t offset)
{
	return (op << 24) | (port << 16) | (offset << 8) | MBI_ENABLE;
}

static int
iosf_mbi_pci_read_mdr(uint32_t mcrx, uint32_t mcr, uint32_t *mdr)
{
	if (mcrx) {
		pci_write_config(iosf_dev, MBI_MCRX_OFFSET, mcrx, 4);
	}

	pci_write_config(iosf_dev, MBI_MCR_OFFSET, mcr, 4);

	*mdr = pci_read_config(iosf_dev, MBI_MDR_OFFSET, 4);

	return (0);
}

static int
iosf_mbi_pci_write_mdr(uint32_t mcrx, uint32_t mcr, uint32_t mdr)
{
	pci_write_config(iosf_dev, MBI_MDR_OFFSET, mdr, 4);

	if (mcrx) {
		pci_write_config(iosf_dev, MBI_MCRX_OFFSET, mcrx, 4);
	}

	pci_write_config(iosf_dev, MBI_MCR_OFFSET, mcr, 4);

	return (0);
}

int
iosf_mbi_read(uint8_t port, uint8_t opcode, uint32_t offset, uint32_t *mdr)
{
	uint32_t mcr, mcrx;
	int ret;

	lwkt_serialize_enter(&iosf_slz);
	if (iosf_dev == NULL) {
		ret = -ENODEV;
		goto done;
	}

	if (port == BT_MBI_UNIT_GFX) {
		device_printf(iosf_dev,
		    "%s: Access to GFX unit is handled by GPU code\n",
		    __func__);
		ret = -EPERM;
		goto done;
	}

	mcr = iosf_mbi_form_mcr(opcode, port, offset & MBI_MASK_LO);
	mcrx = offset & MBI_MASK_HI;

	crit_enter();
	ret = iosf_mbi_pci_read_mdr(mcrx, mcr, mdr);
	crit_exit();

done:
	lwkt_serialize_exit(&iosf_slz);
	return (ret);
}

int
iosf_mbi_write(uint8_t port, uint8_t opcode, uint32_t offset, uint32_t mdr)
{
	uint32_t mcr, mcrx;
	int ret;

	lwkt_serialize_enter(&iosf_slz);
	if (iosf_dev == NULL) {
		ret = -ENODEV;
		goto done;
	}

	if (port == BT_MBI_UNIT_GFX) {
		device_printf(iosf_dev,
		    "%s: Access to GFX unit is handled by GPU code\n",
		    __func__);
		ret = -EPERM;
		goto done;
	}

	mcr = iosf_mbi_form_mcr(opcode, port, offset & MBI_MASK_LO);
	mcrx = offset & MBI_MASK_HI;

	crit_enter();
	ret = iosf_mbi_pci_write_mdr(mcrx, mcr, mdr);
	crit_exit();

done:
	lwkt_serialize_exit(&iosf_slz);
	return (ret);
}

int
iosf_mbi_modify(uint8_t port, uint8_t opcode, uint32_t offset, uint32_t mdr,
    uint32_t mask)
{
	uint32_t mcr, mcrx;
	uint32_t value;
	int ret;

	lwkt_serialize_enter(&iosf_slz);
	if (iosf_dev == NULL) {
		ret = -ENODEV;
		goto done;
	}

	if (port == BT_MBI_UNIT_GFX) {
		device_printf(iosf_dev,
		    "%s: Access to GFX unit is handled by GPU code\n",
		    __func__);
		ret = -EPERM;
		goto done;
	}

	mcr = iosf_mbi_form_mcr(opcode, port, offset & MBI_MASK_LO);
	mcrx = offset & MBI_MASK_HI;

	crit_enter();

	/* Read current mdr value */
	ret = iosf_mbi_pci_read_mdr(mcrx, mcr & MBI_RD_MASK, &value);
	if (ret < 0) {
		crit_exit();
		goto done;
	}

	/* Apply mask */
	value &= ~mask;
	mdr &= mask;
	value |= mdr;

	/* Write back */
	ret = iosf_mbi_pci_write_mdr(mcrx, mcr | MBI_WR_MASK, value);

	crit_exit();

done:
	lwkt_serialize_exit(&iosf_slz);
	return (ret);
}

static void
iosf_identify(driver_t *driver, struct device *parent)
{
	if (iosf_probe(parent) == ENXIO)
		return;
	if (device_find_child(parent, driver->name, -1) != NULL)
		return;
	device_add_child(parent, driver->name, -1);
}

static int
iosf_probe(struct device *dev)
{
	char *desc;

	if (device_get_unit(dev) != 0)
		return (ENXIO);

	if (pci_get_vendor(dev) != PCI_VENDOR_INTEL)
		return (ENXIO);

	switch (pci_get_device(dev)) {
	case 0x2280:
		desc = "Braswell IOSF sideband interface";
		break;
	default:
		return (ENXIO);
	}

	if (device_get_desc(dev) == NULL)
		device_set_desc(dev, desc);
	return (0);
}

static int
iosf_attach(struct device *dev)
{
	struct iosf_softc	*sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	lwkt_serialize_enter(&iosf_slz);
	iosf_dev = dev;
	lwkt_serialize_exit(&iosf_slz);

	return (0);
}

static int
iosf_detach(struct device *dev)
{
	 /* struct iosf_softc *sc = device_get_softc(dev); */

	lwkt_serialize_enter(&iosf_slz);
	iosf_dev = NULL;
	lwkt_serialize_exit(&iosf_slz);

	return (0);
}
