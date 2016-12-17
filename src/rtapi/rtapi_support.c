/********************************************************************
* Description:  rtapi_support.c
*               This file, 'rtapi_support.c', implements the messaging
*               functions for both kernel and userland thread
*               systems.  See rtapi.h for more info.
*
*               Other than the rest of RTAPI, these functions are linked
*               into the instance module which is loaded before rtapi.so/ko
*               so they are available and message level set before
*               RTAPI starts up
*
*     Copyright 2006-2013 Various Authors
* 
*     This program is free software; you can redistribute it and/or modify
*     it under the terms of the GNU General Public License as published by
*     the Free Software Foundation; either version 2 of the License, or
*     (at your option) any later version.
* 
*     This program is distributed in the hope that it will be useful,
*     but WITHOUT ANY WARRANTY; without even the implied warranty of
*     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*     GNU General Public License for more details.
* 
*     You should have received a copy of the GNU General Public License
*     along with this program; if not, write to the Free Software
*     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
********************************************************************/


#include "config.h"
#include "rtapi.h"
#include "shmdrv.h"
#include "ring.h"
#if defined(BUILD_SYS_USER_DSO) || defined(ULAPI)
#include "syslog_async.h"
#ifndef SYSLOG_FACILITY
#define SYSLOG_FACILITY LOG_LOCAL1  // where all rtapi/ulapi logging goes
#endif
#endif
#define RTPRINTBUFFERLEN 256

#ifdef MODULE
#include "rtapi_app.h"

#include <stdarg.h>		/* va_* */
#include <linux/kernel.h>	/* kernel's vsnprintf */

#define MSG_ORIGIN MSG_KERNEL

#else  /* user land */

#include <stdio.h>		/* libc's vsnprintf() */
#include <sys/types.h>
#include <unistd.h>

#ifdef RTAPI
#define MSG_ORIGIN MSG_RTUSER
#else
#define MSG_ORIGIN MSG_ULAPI
#endif
#endif

static int get_msg_level(void);

// these message levels are used in RTAPI and ULAPI
// respectively until the global segment is attached;
// thereafter switch to using the message levels from there.
#ifdef RTAPI
static int rt_msg_level = RTAPI_MSG_INFO;    // RTAPI (u+k)
#else
static int ulapi_msg_level = RTAPI_MSG_INFO; // ULAPI
#endif

#ifdef ULAPI
ringbuffer_t rtapi_message_buffer;   // rtapi_message ring access strcuture
# else
extern ringbuffer_t rtapi_message_buffer;
#endif

static char logtag[TAGSIZE];
static const char *origins[] = { "kernel", "rt", "user", "*invalid*" };

typedef struct {
    rtapi_msgheader_t hdr;
    char buf[RTPRINTBUFFERLEN];
} rtapi_msg_t;

int vs_ringlogfv(const msg_level_t level,
		 const pid_t pid,
		 const msg_origin_t origin,
		 const char *tag,
		 const char *format,
		 va_list ap)
{
    int n;
    rtapi_msg_t msg;

    if (get_msg_level() == RTAPI_MSG_NONE)
	return 0;
    if (level >= get_msg_level())
	return 0;

    msg.hdr.origin = origin;
    msg.hdr.pid = pid;
    msg.hdr.level = level;
    strncpy(msg.hdr.tag, tag, sizeof(msg.hdr.tag));

    // do format outside critical section
    n = vsnprintf(msg.buf, RTPRINTBUFFERLEN, format, ap);

    if (rtapi_message_buffer.header != NULL) {
	if (rtapi_message_buffer.header->use_wmutex &&
	    rtapi_mutex_try(&rtapi_message_buffer.header->wmutex)) {
	    global_data->error_ring_locked++;
	    return -EBUSY;
	}
	// use copying writer to shorten criticial section
	record_write(&rtapi_message_buffer, (void *) &msg,
		     sizeof(rtapi_msgheader_t) + n + 1); // trailing zero
	if (rtapi_message_buffer.header->use_wmutex)
	    rtapi_mutex_give(&rtapi_message_buffer.header->wmutex);
    } else {
	// early startup, global_data & log ring not yet initialized
	// depending on context, log the message in an appropriate way:

#if defined(BUILD_SYS_USER_DSO) || defined(ULAPI)
	static int log_opened;
	if (!log_opened) {
	    log_opened = async_log_open();
	    if (!log_opened) {
		openlog_async("startup", LOG_NDELAY , SYSLOG_FACILITY);
		log_opened = 1;
	    }
	}
#ifdef USE_STDERR
	if (!strchr(msg.buf, '\n'))
	    strcat(msg.buf,"\n");
	fprintf(stderr,
#else
        syslog_async(rtapi2syslog(level),
#endif
#endif
#if defined(RTAPI) && defined(BUILD_SYS_KBUILD)
	printk(
#endif
	       "%d:%s:%d:%s %s",
	       level,
	       tag,
	       pid,
	       origins[origin & 3],
	       msg.buf);
    }
    return n;
}

void default_rtapi_msg_handler(msg_level_t level, const char *fmt,
			       va_list ap)
{
    static pid_t rtapi_pid;
#if !defined(BUILD_SYS_KBUILD) && !defined(MODULE)
    if (rtapi_pid == 0)
	rtapi_pid = getpid();
#endif
    vs_ringlogfv(level, rtapi_pid, MSG_ORIGIN, logtag, fmt, ap);
}

static rtapi_msg_handler_t rtapi_msg_handler = default_rtapi_msg_handler;

rtapi_msg_handler_t rtapi_get_msg_handler(void) {
    return rtapi_msg_handler;
}

void rtapi_set_msg_handler(rtapi_msg_handler_t handler) {
    if (handler == NULL)
	rtapi_msg_handler = default_rtapi_msg_handler;
    else
	rtapi_msg_handler = handler;
}

// rtapi_get_msg_level and rtapi_set_msg_level moved here
// since they access the global segment 
// which might not exist during first use
// assure we can use message levels before global_data is set up

static int get_msg_level(void)
{
#if RTAPI
    if (global_data == 0)
	return rt_msg_level;
    else
	return global_data->rt_msg_level;
#else
    if (global_data == 0)
	return ulapi_msg_level;
    else
	return global_data->user_msg_level;
#endif
}

static int set_msg_level(int new_level)
{
    int old_level;

#if RTAPI
    if (global_data) {
	old_level = global_data->rt_msg_level;
	global_data->rt_msg_level = new_level;
    } else {
	old_level = rt_msg_level;
	rt_msg_level = new_level;
    }
    return old_level;
#else
    old_level = ulapi_msg_level;
    ulapi_msg_level = new_level;
    return old_level;
#endif
}

int rtapi_set_msg_level(int level) {
    int oldlevel;
    if ((level < RTAPI_MSG_NONE) || (level > RTAPI_MSG_ALL)) {
	return -EINVAL;
    }
    oldlevel = set_msg_level(level);
    return oldlevel;
}

int rtapi_get_msg_level() {
    return get_msg_level();
}

void rtapi_print(const char *fmt, ...) {
    va_list args;

    va_start(args, fmt);
    rtapi_msg_handler(RTAPI_MSG_ALL, fmt, args);
    va_end(args);
}

void rtapi_print_msg(int level, const char *fmt, ...) {
    va_list args;

    if ((level <= rtapi_get_msg_level()) && 
	(rtapi_get_msg_level() != RTAPI_MSG_NONE)) {
	va_start(args, fmt);
	rtapi_msg_handler(level, fmt, args);
	va_end(args);
    }
}
#define RTAPIPRINTBUFFERLEN 256
static char _rtapi_logmsg[RTAPIPRINTBUFFERLEN];

const char *rtapi_last_msg(void) {
    return _rtapi_logmsg;
}
void rtapi_print_loc(const int level,
		     const char *func,
		     const int line,
		     const char *topic,
		     const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    const char *pfmt = "%s:%d %s ";
    rtapi_snprintf(_rtapi_logmsg, RTAPIPRINTBUFFERLEN, pfmt,
		   func  == NULL ? "(nil)" : func,
		   line,
		   topic == NULL ? "" : topic);
    int n = strlen(_rtapi_logmsg);

    vsnprintf(_rtapi_logmsg + n, RTAPIPRINTBUFFERLEN - n, fmt, args);
    rtapi_print_msg(level, _rtapi_logmsg);
    va_end(args);
}

int rtapi_snprintf(char *buf, unsigned long int size,
		   const char *fmt, ...) {
    va_list args;
    int result;

    va_start(args, fmt);
    result = vsnprintf(buf, size, fmt, args);
    va_end(args);
    return result;
}

int rtapi_vsnprintf(char *buf, unsigned long int size, const char *fmt,
		    va_list ap) {
    return vsnprintf(buf, size, fmt, ap);
}

int rtapi_set_logtag(const char *fmt, ...) {
    va_list args;
    int result;

    va_start(args, fmt);
    result = vsnprintf(logtag, sizeof(logtag), fmt, args);
    va_end(args);
    return result;
}

const char *rtapi_get_logtag(void) {
    return logtag;
}

/* $Id: argvize.c,v 1.1.1.1 2006/06/29 06:43:26 cpu Exp $ */

/*
 * Copyright (c) 2000-2002 Opsycon AB  (www.opsycon.se)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Opsycon AB.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

// see rtapi.h for usage information
int rtapi_argvize(int avsize, char **av, char *s)
{
	char **pav = av, c;
	int ac;

	for (ac = 0; ac < avsize - 1; ac++) {
		/* step over cntrls and spaces */
		while (*s && *s <= ' ')
			s++;

		if (!*s)
			break;

		c = *s;
		/* if it's a quote skip forward */
		if (c == '\'' || c == '"') {
			if (pav)
				*pav++ = ++s;
			while (*s && *s != c)
				s++;
			if (*s)
				*s++ = 0;
		} else {		/* find end of word */
			if (pav)
				*pav++ = s;
			while (' ' < *s)
				s++;
		}

		if (*s)
			*s++ = 0;
	}
	*pav = NULL;
	return (ac);
}


#ifdef RTAPI
EXPORT_SYMBOL(rtapi_get_msg_handler);
EXPORT_SYMBOL(rtapi_set_msg_handler);
EXPORT_SYMBOL(rtapi_print_msg);
EXPORT_SYMBOL(rtapi_print);
EXPORT_SYMBOL(rtapi_snprintf);
EXPORT_SYMBOL(rtapi_vsnprintf);
EXPORT_SYMBOL(rtapi_argvize);
EXPORT_SYMBOL(rtapi_set_msg_level);
EXPORT_SYMBOL(rtapi_get_msg_level);
EXPORT_SYMBOL(rtapi_set_logtag);
EXPORT_SYMBOL(rtapi_get_logtag);
EXPORT_SYMBOL(rtapi_print_loc);
#endif
