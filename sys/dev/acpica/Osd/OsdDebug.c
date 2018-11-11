/*-
 * Copyright (c) 2000 Michael Smith
 * Copyright (c) 2000 BSDi
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD: head/sys/dev/acpica/Osd/OsdDebug.c 222544 2011-05-31 19:45:58Z jkim $
 */

/*
 * Debugging Support
 */

#include "opt_ddb.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <ddb/ddb.h>
#include <ddb/db_output.h>

#include "acpi.h"
#include "accommon.h"
#include <dev/acpica/acpivar.h>
#include "acdebug.h"

ACPI_MODULE_NAME("DEBUG")

ACPI_STATUS
AcpiOsGetLine(char *Buffer, UINT32 BufferLength, UINT32 *BytesRead)
{
#ifdef DDB
	char *cp;

	cp = Buffer;
	if (db_readline(Buffer, BufferLength) > 0)
		while (*cp != '\0' && *cp != '\n' && *cp != '\r')
			cp++;
	*cp = '\0';
	if (BytesRead != NULL)
		*BytesRead = cp - Buffer;
	return (AE_OK);
#else
	kprintf("AcpiOsGetLine called but no input support");
	return (AE_NOT_EXIST);
#endif /* DDB */
}

ACPI_STATUS
AcpiOsSignal(UINT32 Function, void *Info)
{
    ACPI_SIGNAL_FATAL_INFO	*fatal;
    
    switch (Function) {
    case ACPI_SIGNAL_FATAL:
	fatal = (ACPI_SIGNAL_FATAL_INFO *)Info;
	kprintf("ACPI fatal signal, type 0x%x code 0x%x argument 0x%x",
	      fatal->Type, fatal->Code, fatal->Argument);
#ifdef ACPI_DEBUG
	Debugger("AcpiOsSignal");
#endif
	break;
	
    case ACPI_SIGNAL_BREAKPOINT:
#ifdef ACPI_DEBUG
	Debugger((char *)Info);
#endif
	break;

    default:
	return (AE_BAD_PARAMETER);
    }

    return (AE_OK);
}

#ifdef ACPI_DEBUGGER
ACPI_STATUS
AcpiOsInitializeDebugger(void)
{
	return (AE_OK);
}

void
AcpiOsTerminateDebugger(void)
{
}

ACPI_STATUS
AcpiOsWaitCommandReady(void)
{
	ACPI_STATUS Status;

	/* Force output to console until a command is entered */

	AcpiDbSetOutputDestination(ACPI_DB_CONSOLE_OUTPUT);

	/* Different prompt if method is executing */

	if (!AcpiGbl_MethodExecuting)
		AcpiOsPrintf("%1c ", ACPI_DEBUGGER_COMMAND_PROMPT);
	else
		AcpiOsPrintf("%1c ", ACPI_DEBUGGER_EXECUTE_PROMPT);

	/* Get the user input line */

	Status = AcpiOsGetLine(AcpiGbl_DbLineBuf,
	    ACPI_DB_LINE_BUFFER_SIZE, NULL);

	if (ACPI_FAILURE (Status) && Status != AE_CTRL_TERMINATE)
		ACPI_EXCEPTION ((AE_INFO, Status,
			"While parsing/handling command line"));
	return (Status);
}

ACPI_STATUS
AcpiOsNotifyCommandComplete(void)
{
	return (AE_OK);
}

DB_COMMAND(acpidb, db_cmd_acpidb)
{
    kprintf("Entering ACPICA debugger...\n");
    while (!AcpiGbl_DbTerminateLoop) {
	if (ACPI_FAILURE(AcpiOsWaitCommandReady()))
	    break;
	AcpiDbCommandDispatch(AcpiGbl_DbLineBuf, NULL, NULL);
    }
    AcpiGbl_DbTerminateLoop = FALSE;
    kprintf("Leaving ACPICA debugger...\n");
}
#endif /* ACPI_DEBUGGER */
