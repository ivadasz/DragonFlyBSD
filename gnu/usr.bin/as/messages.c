/* messages.c - error reporter -
   Copyright (C) 1987, 1991, 1992 Free Software Foundation, Inc.

   This file is part of GAS, the GNU Assembler.

   GAS is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   GAS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with GAS; see the file COPYING.  If not, write to
   the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA. */

/*
 * $FreeBSD: src/gnu/usr.bin/as/messages.c,v 1.8 1999/08/27 23:34:19 peter Exp $
 * $DragonFly: src/gnu/usr.bin/as/Attic/messages.c,v 1.3 2004/01/23 20:53:09 joerg Exp $
 */
#include <stdio.h>
#include <errno.h>

#include "as.h"

#include <stdarg.h>

extern char *strerror ();

static void as_show_where PARAMS ((void));
static void as_warn_internal PARAMS ((char *, unsigned int, char *));
static void as_bad_internal PARAMS ((char *, unsigned int, char *));

/*
 * Despite the rest of the comments in this file, (FIXME-SOON),
 * here is the current scheme for error messages etc:
 *
 * as_fatal() is used when gas is quite confused and
 * continuing the assembly is pointless.  In this case we
 * exit immediately with error status.
 *
 * as_bad() is used to mark errors that result in what we
 * presume to be a useless object file.  Say, we ignored
 * something that might have been vital.  If we see any of
 * these, assembly will continue to the end of the source,
 * no object file will be produced, and we will terminate
 * with error status.  The new option, -Z, tells us to
 * produce an object file anyway but we still exit with
 * error status.  The assumption here is that you don't want
 * this object file but we could be wrong.
 *
 * as_warn() is used when we have an error from which we
 * have a plausible error recovery.  eg, masking the top
 * bits of a constant that is longer than will fit in the
 * destination.  In this case we will continue to assemble
 * the source, although we may have made a bad assumption,
 * and we will produce an object file and return normal exit
 * status (ie, no error).  The new option -X tells us to
 * treat all as_warn() errors as as_bad() errors.  That is,
 * no object file will be produced and we will exit with
 * error status.  The idea here is that we don't kill an
 * entire make because of an error that we knew how to
 * correct.  On the other hand, sometimes you might want to
 * stop the make at these points.
 *
 * as_tsktsk() is used when we see a minor error for which
 * our error recovery action is almost certainly correct.
 * In this case, we print a message and then assembly
 * continues as though no error occurred.
 */

static void
identify(char *file)
{
  static int identified;
  if (identified)
    return;
  identified++;

  if (!file)
    {
      unsigned int x;
      as_where (&file, &x);
    }

  fprintf (stderr, "%s: Assembler messages:\n", file);
}

static int warning_count;	/* Count of number of warnings issued */

int
had_warnings ()
{
  return (warning_count);
}				/* had_err() */

/* Nonzero if we've hit a 'bad error', and should not write an obj file,
   and exit with a nonzero error code */

static int error_count;

int
had_errors ()
{
  return (error_count);
}				/* had_errors() */


/* Print the current location to stderr.  */

static void
as_show_where ()
{
  char *file;
  unsigned int line;

  as_where (&file, &line);
  identify (file);
  fprintf (stderr, "%s:%u: ", file, line);
}

/*
 *			a s _ p e r r o r
 *
 * Like perror(3), but with more info.
 */

void
as_perror (gripe, filename)
     const char *gripe;		/* Unpunctuated error theme. */
     const char *filename;
{
  const char *errtxt;

  as_show_where ();
  fprintf (stderr, gripe, filename);
#ifdef BFD_ASSEMBLER
  errtxt = bfd_errmsg (bfd_get_error ());
#else
  errtxt = strerror (errno);
#endif
  fprintf (stderr, ": %s\n", errtxt);
  errno = 0;
#ifdef BFD_ASSEMBLER
  bfd_set_error (bfd_error_no_error);
#endif
}

/*
 *			a s _ t s k t s k ()
 *
 * Send to stderr a string as a warning, and locate warning
 * in input file(s).
 * Please only use this for when we have some recovery action.
 * Please explain in string (which may have '\n's) what recovery was done.
 */

void
as_tsktsk(const char *format, ...)
{
  va_list args;

  as_show_where ();
  va_start (args, format);
  vfprintf (stderr, format, args);
  va_end (args);
  (void) putc ('\n', stderr);
}				/* as_tsktsk() */

/* The common portion of as_warn and as_warn_where.  */

static void
as_warn_internal (file, line, buffer)
     char *file;
     unsigned int line;
     char *buffer;
{
  ++warning_count;

  if (file == NULL)
    as_where (&file, &line);

  identify (file);
  fprintf (stderr, "%s:%u: Warning: ", file, line);
  fputs (buffer, stderr);
  (void) putc ('\n', stderr);
#ifndef NO_LISTING
  listing_warning (buffer);
#endif
}

/*
 *			a s _ w a r n ()
 *
 * Send to stderr a string as a warning, and locate warning
 * in input file(s).
 * Please only use this for when we have some recovery action.
 * Please explain in string (which may have '\n's) what recovery was done.
 */

#if 1
#define flag_no_warnings	(flagseen['W'])
#endif

void
as_warn (const char *format,...)
{
  va_list args;
  char buffer[200];

  if (!flag_no_warnings)
    {
      va_start (args, format);
      vsprintf (buffer, format, args);
      va_end (args);
      as_warn_internal ((char *) NULL, 0, buffer);
    }
}				/* as_warn() */

/* as_warn_where, like as_bad but the file name and line number are
   passed in.  Unfortunately, we have to repeat the function in order
   to handle the varargs correctly and portably.  */

void
as_warn_where(char *file, unsigned int line, const char *format, ...)
{
  va_list args;
  char buffer[200];

  if (!flag_no_warnings)
    {
      va_start (args, format);
      vsprintf (buffer, format, args);
      va_end (args);
      as_warn_internal (file, line, buffer);
    }
}				/* as_warn() */

/* The common portion of as_bad and as_bad_where.  */

static void
as_bad_internal (file, line, buffer)
     char *file;
     unsigned int line;
     char *buffer;
{
  ++error_count;

  if (file == NULL)
    as_where (&file, &line);

  identify (file);
  fprintf (stderr, "%s:%u: Error: ", file, line);
  fputs (buffer, stderr);
  (void) putc ('\n', stderr);
#ifndef NO_LISTING
  listing_error (buffer);
#endif
}

/*
 *			a s _ b a d ()
 *
 * Send to stderr a string as a warning, and locate warning in input file(s).
 * Please us when there is no recovery, but we want to continue processing
 * but not produce an object file.
 * Please explain in string (which may have '\n's) what recovery was done.
 */

void
as_bad (const char *format,...)
{
  va_list args;
  char buffer[200];

  va_start (args, format);
  vsprintf (buffer, format, args);
  va_end (args);

  as_bad_internal ((char *) NULL, 0, buffer);
}

/* as_bad_where, like as_bad but the file name and line number are
   passed in.  Unfortunately, we have to repeat the function in order
   to handle the varargs correctly and portably.  */

void
as_bad_where (char *file, unsigned int line, const char *format,...)
{
  va_list args;
  char buffer[200];

  va_start (args, format);
  vsprintf (buffer, format, args);
  va_end (args);

  as_bad_internal (file, line, buffer);
}

/*
 *			a s _ f a t a l ()
 *
 * Send to stderr a string as a fatal message, and print location of error in
 * input file(s).
 * Please only use this for when we DON'T have some recovery action.
 * It exit()s with a warning status.
 */

void
as_fatal (const char *format,...)
{
  va_list args;

  as_show_where ();
  va_start (args, format);
  fprintf (stderr, "Fatal error:");
  vfprintf (stderr, format, args);
  (void) putc ('\n', stderr);
  va_end (args);
  exit (33);
}				/* as_fatal() */

void
fprint_value (file, val)
     FILE *file;
     valueT val;
{
  if (sizeof (val) <= sizeof (long))
    {
      fprintf (file, "%ld", (long) val);
      return;
    }
#ifdef BFD_ASSEMBLER
  if (sizeof (val) <= sizeof (bfd_vma))
    {
      fprintf_vma (file, val);
      return;
    }
#endif
  abort ();
}

void
sprint_value (buf, val)
     char *buf;
     valueT val;
{
  if (sizeof (val) <= sizeof (long))
    {
      sprintf (buf, "%ld", (long) val);
      return;
    }
#ifdef BFD_ASSEMBLER
  if (sizeof (val) <= sizeof (bfd_vma))
    {
      sprintf_vma (buf, val);
      return;
    }
#endif
  abort ();
}

/* end of messages.c */
