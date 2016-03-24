/*
 * Copyright (c) 2016 The DragonFly Project.  All rights reserved.
 *
 * This code is derived from software contributed to The DragonFly Project
 * by Imre Vadász <imre@vdsz.com>
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
 */

#include "opt_acpi.h"
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/queue.h>
#include <sys/bus.h>

#include "acpi.h"
#include <dev/acpica/acpivar.h>

/* Hooks for the ACPICA debugging infrastructure */
#define _COMPONENT	ACPI_BUS
ACPI_MODULE_NAME("DEP_ORDER")

static TAILQ_HEAD(dep_list, dep_node) nodes, stack;
TAILQ_HEAD(child_list, child_node);

struct dep_node {
	ACPI_HANDLE handle;
	int level;
	int onwork;
	struct child_list children;
	TAILQ_ENTRY(dep_node) link, worklink;
};

struct child_node {
	struct dep_node *node;
	TAILQ_ENTRY(child_node) link;
};

static struct dep_list	*acpi_handle_get_deps(ACPI_HANDLE h);
static BOOLEAN		acpi_is_device_handle(ACPI_HANDLE handle);
static ACPI_STATUS	acpi_dep_probe_child(ACPI_HANDLE handle, UINT32 level,
			    void *context, void **status);

static struct dep_list *
acpi_handle_get_deps(ACPI_HANDLE h)
{
	ACPI_STATUS s;
	ACPI_BUFFER b;
	ACPI_OBJECT *bp, *o;
	struct dep_list *ls = NULL;
	struct dep_node *n;
	int i;

	b.Length = ACPI_ALLOCATE_BUFFER;
	s = AcpiEvaluateObjectTyped(h, "_DEP", NULL, &b, ACPI_TYPE_PACKAGE);
	if (ACPI_FAILURE(s))
		return NULL;

	bp = b.Pointer;
	o = bp->Package.Elements;

	if (bp->Package.Count > 0) {
		ls = kmalloc(sizeof(*ls), M_TEMP, M_WAITOK | M_ZERO);
		TAILQ_INIT(ls);
		for (i = 0; i < bp->Package.Count; i++) {
			n = kmalloc(sizeof(*n), M_TEMP, M_WAITOK | M_ZERO);
			n->handle = acpi_GetReference(h, &o[i]);
			n->level = 0;
			TAILQ_INSERT_TAIL(ls, n, link);
		}
	}
	AcpiOsFree(b.Pointer);

	return ls;
}

static BOOLEAN
acpi_is_device_handle(ACPI_HANDLE handle)
{
	ACPI_OBJECT_TYPE type;
	char *handle_str;

	if (acpi_disabled("children"))
		return (FALSE);

	/* Skip this device if we think we'll have trouble with it. */
	if (acpi_avoid(handle))
		return (FALSE);

	if (ACPI_SUCCESS(AcpiGetType(handle, &type))) {
		handle_str = acpi_name(handle);
		switch (type) {
		case ACPI_TYPE_DEVICE:
			/*
			 * Since we scan from \, be sure to skip system scope
			 * objects.  \_SB_ and \_TZ_ are defined in ACPICA as
			 * devices to work around BIOS bugs.  For example,
			 * \_SB_ is to allow * \_SB_._INI to be run during the
			 * intialization and \_TZ_ is to support Notify() on it.
			 */
			if (strcmp(handle_str, "\\_SB_") == 0 ||
			    strcmp(handle_str, "\\_TZ_") == 0)
				break;

			/* FALLTHROUGH */
		case ACPI_TYPE_PROCESSOR:
		case ACPI_TYPE_THERMAL:
		case ACPI_TYPE_POWER:
			return (TRUE);
		}
	}

	return (FALSE);
}

/*
 * Scan all of the ACPI namespace.
 *
 * We should only expect to find devices in the \_PR, \_TZ, \_SI, and
 * \_SB scopes, and \_PR and \_TZ became obsolete in the ACPI 2.0 spec.
 * However, in violation of the spec, some systems place their PCI link
 * devices in \, so we have to walk the whole namespace.  We check the
 * type of namespace nodes, so this should be ok.
 */
static ACPI_STATUS
acpi_dep_probe_child(ACPI_HANDLE handle, UINT32 level, void *context,
    void **status)
{
	struct dep_node *n;
	char *handle_str;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);

	if (acpi_disabled("children"))
		return_ACPI_STATUS (AE_OK);

	/* Skip this device if we think we'll have trouble with it. */
	if (acpi_avoid(handle))
		return_ACPI_STATUS (AE_OK);

	if (acpi_is_device_handle(handle)) {
		handle_str = acpi_name(handle);
		ACPI_DEBUG_PRINT((ACPI_DB_OBJECTS, "scanning '%s'\n",
		    handle_str));
		n = kmalloc(sizeof(*n), M_TEMP, M_WAITOK | M_ZERO);
		n->handle = handle;
		n->level = level;
		n->onwork = 0;
		TAILQ_INIT(&n->children);
		TAILQ_INSERT_TAIL(&nodes, n, link);
		TAILQ_INSERT_TAIL(&stack, n, worklink);
	}

	return_ACPI_STATUS (AE_OK);
}

static ACPI_STATUS
acpi_dep_ascend(ACPI_HANDLE handle, UINT32 level, void *context,
    void **status)
{
	struct dep_node *n;
	struct child_node *c;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);

	if (acpi_disabled("children"))
		return_ACPI_STATUS (AE_OK);

	/* Skip this device if we think we'll have trouble with it. */
	if (acpi_avoid(handle))
		return_ACPI_STATUS (AE_OK);

	if (acpi_is_device_handle(handle)) {
		n = TAILQ_LAST(&stack, dep_list);
		TAILQ_REMOVE(&stack, n, worklink);
		c = kmalloc(sizeof(*c), M_TEMP, M_WAITOK | M_ZERO);
		c->node = n;
		TAILQ_INSERT_TAIL(&TAILQ_LAST(&stack, dep_list)->children, c,
		    link);
	}

	return_ACPI_STATUS (AE_OK);
}

void
acpi_dep_foreach_device(device_t bus, ACPI_HANDLE StartObject, UINT32 MaxDepth,
    void *Context,
    void(*DeviceCallback)(ACPI_HANDLE handle, UINT32 level, void *context))
{
	struct dep_node *m, *n, *o, root;
	struct child_node *c;
	struct dep_list work, *deps;

	TAILQ_INIT(&nodes);
	TAILQ_INIT(&stack);
	memset(&root, 0, sizeof(root));
	root.handle = NULL;
	root.level = 0;
	TAILQ_INIT(&root.children);
	TAILQ_INSERT_TAIL(&stack, &root, worklink);

	ACPI_DEBUG_PRINT((ACPI_DB_OBJECTS, "namespace scan\n"));
	AcpiWalkNamespace(ACPI_TYPE_ANY, StartObject, MaxDepth,
	    acpi_dep_probe_child, acpi_dep_ascend, bus, NULL);
	TAILQ_REMOVE(&stack, &root, worklink);

	/* Add child relation according to _DEP return values */
	TAILQ_FOREACH(n, &nodes, link) {
		deps = acpi_handle_get_deps(n->handle);
		if (deps == NULL)
			continue;
		TAILQ_FOREACH(m, deps, link) {
			TAILQ_FOREACH(o, &nodes, link) {
				if (o->handle == m->handle) {
					c = kmalloc(sizeof(*c), M_TEMP,
					    M_WAITOK | M_ZERO);
					c->node = n;
					TAILQ_INSERT_TAIL(&o->children, c,
					    link);
					break;
				}
			}
		}
		while (!TAILQ_EMPTY(deps)) {
			m = TAILQ_FIRST(deps);
			TAILQ_REMOVE(deps, m, link);
			kfree(m, M_TEMP);
		}
		kfree(deps, M_TEMP);
	}

	/* Compute topological sorting */
	TAILQ_INIT(&work);
	TAILQ_FOREACH(c, &root.children, link) {
		n = c->node;
		TAILQ_INSERT_TAIL(&work, n, worklink);
		n->onwork = 1;
	}
	while (!TAILQ_EMPTY(&work)) {
		n = TAILQ_FIRST(&work);
		TAILQ_REMOVE(&work, n, worklink);
		n->onwork = 0;
		TAILQ_FOREACH(c, &n->children, link) {
			m = c->node;
			if (m->level < n->level + 1) {
				m->level = n->level + 1;
				/* m->level >= 10000 would be absurd */
				KKASSERT(m->level < 10000);
			}
			if (!m->onwork) {
				TAILQ_INSERT_TAIL(&work, m, worklink);
				m->onwork = 1;
			}
		}
	}

	/* Execute callback */
	TAILQ_FOREACH(n, &nodes, link) {
		DeviceCallback(n->handle, n->level, Context);
	}

	/* Clean up */
	while (!TAILQ_EMPTY(&nodes)) {
		n = TAILQ_FIRST(&nodes);
		TAILQ_REMOVE(&nodes, n, link);
		while (!TAILQ_EMPTY(&n->children)) {
			c = TAILQ_FIRST(&n->children);
			TAILQ_REMOVE(&n->children, c, link);
			kfree(c, M_TEMP);
		}
		kfree(n, M_TEMP);
	}
}
