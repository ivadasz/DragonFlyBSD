/*
 * acpi_lpat.c - LPAT table processing functions
 *
 * Copyright (C) 2015 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>

#include "opt_acpi.h"
#include "acpi.h"
#include <dev/acpica/acpivar.h>

#include "acpi_lpat.h"

/**
 * acpi_lpat_raw_to_temp(): Return temperature from raw value through
 * LPAT conversion table
 *
 * @lpat_table: the temperature_raw mapping table structure
 * @raw: the raw value, used as a key to get the temerature from the
 *       above mapping table
 *
 * A positive converted temperarure value will be returned on success,
 * a negative errno will be returned in error cases.
 */
int
acpi_lpat_raw_to_temp(struct acpi_lpat_conversion_table *lpat_table, int raw)
{
	int i, delta_temp, delta_raw, temp;
	struct acpi_lpat *lpat = lpat_table->lpat;

	for (i = 0; i < lpat_table->lpat_count - 1; i++) {
		if ((raw >= lpat[i].raw && raw <= lpat[i+1].raw) ||
		    (raw <= lpat[i].raw && raw >= lpat[i+1].raw))
			break;
	}

	if (i == lpat_table->lpat_count - 1)
		return -ENOENT;

	delta_temp = lpat[i+1].temp - lpat[i].temp;
	delta_raw = lpat[i+1].raw - lpat[i].raw;
	temp = lpat[i].temp + (raw - lpat[i].raw) * delta_temp / delta_raw;

	return temp;
}

/**
 * acpi_lpat_temp_to_raw(): Return raw value from temperature through
 * LPAT conversion table
 *
 * @lpat: the temperature_raw mapping table
 * @temp: the temperature, used as a key to get the raw value from the
 *        above mapping table
 *
 * A positive converted temperature value will be returned on success,
 * a negative errno will be returned in error cases.
 */
int
acpi_lpat_temp_to_raw(struct acpi_lpat_conversion_table *lpat_table, int temp)
{
	int i, delta_temp, delta_raw, raw;
	struct acpi_lpat *lpat = lpat_table->lpat;

	for (i = 0; i < lpat_table->lpat_count - 1; i++) {
		if (temp >= lpat[i].temp && temp <= lpat[i+1].temp)
			break;
	}

	if (i ==  lpat_table->lpat_count - 1)
		return -ENOENT;

	delta_temp = lpat[i+1].temp - lpat[i].temp;
	delta_raw = lpat[i+1].raw - lpat[i].raw;
	raw = lpat[i].raw + (temp - lpat[i].temp) * delta_raw / delta_temp;

	return raw;
}

/**
 * acpi_lpat_get_conversion_table(): Parse ACPI LPAT table if present.
 *
 * @handle: Handle to acpi device
 *
 * Parse LPAT table to a struct of type acpi_lpat_table. On success
 * it returns a pointer to newly allocated table. This table must
 * be freed by the caller when finished processing, using a call to
 * acpi_lpat_free_conversion_table.
 */
struct acpi_lpat_conversion_table *
acpi_lpat_get_conversion_table(ACPI_HANDLE handle)
{
	struct acpi_lpat_conversion_table *lpat_table = NULL;
	ACPI_BUFFER buffer;
	ACPI_OBJECT*obj_p, *obj_e;
	int *lpat, i;
	ACPI_STATUS status;

	buffer.Length = ACPI_ALLOCATE_BUFFER;
	status = AcpiEvaluateObject(handle, "LPAT", NULL, &buffer);
	if (ACPI_FAILURE(status))
		return NULL;

	obj_p = buffer.Pointer;
	if (obj_p == NULL || (obj_p->Type != ACPI_TYPE_PACKAGE) ||
	    (obj_p->Package.Count % 2) || (obj_p->Package.Count < 4))
		goto out;

	lpat = kmalloc(obj_p->Package.Count * sizeof(int), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	for (i = 0; i < obj_p->Package.Count; i++) {
		obj_e = &obj_p->Package.Elements[i];
		if (obj_e->Type != ACPI_TYPE_INTEGER) {
			kfree(lpat, M_DEVBUF);
			goto out;
		}
		lpat[i] = (int64_t)obj_e->Integer.Value;
	}

	lpat_table = kmalloc(sizeof(*lpat_table), M_DEVBUF, M_WAITOK | M_ZERO);
	if (!lpat_table) {
		kfree(lpat, M_DEVBUF);
		goto out;
	}

	lpat_table->lpat = (struct acpi_lpat *)lpat;
	lpat_table->lpat_count = obj_p->Package.Count / 2;

out:
	AcpiOsFree(buffer.Pointer);
	return lpat_table;
}

/**
 * acpi_lpat_free_conversion_table(): Free LPAT table.
 *
 * @lpat_table: the temperature_raw mapping table structure
 *
 * Frees the LPAT table previously allocated by a call to
 * acpi_lpat_get_conversion_table.
 */
void
acpi_lpat_free_conversion_table(struct acpi_lpat_conversion_table *lpat_table)
{
	if (lpat_table != NULL) {
		kfree(lpat_table->lpat, M_DEVBUF);
		kfree(lpat_table, M_DEVBUF);
	}
}
