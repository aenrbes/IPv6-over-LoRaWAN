/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *         Slip configuration
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <err.h>
#include "contiki.h"

const char *slip_config_ipaddr;
const char *slip_config_appid = NULL;
char slip_config_tundev[32] = { "" };

/*---------------------------------------------------------------------------*/
int
slip_config_handle_arguments(int argc, char **argv)
{
  const char *prog;
  int c;

  prog = argv[0];
  while((c = getopt(argc, argv, "t:a:h")) != -1) {
    switch(c) {
    case 't':
      if(strncmp("/dev/", optarg, 5) == 0) {
        strncpy(slip_config_tundev, optarg + 5, sizeof(slip_config_tundev));
      } else {
        strncpy(slip_config_tundev, optarg, sizeof(slip_config_tundev));
      }
      break;

    case 'a':
      slip_config_appid = optarg;
      break;

    case '?':
    case 'h':
    default:
      fprintf(stderr, "usage:  %s [options] ipaddress\n", prog);
      fprintf(stderr, "example: border-router.native -a 123 fd00::1/64\n");
      fprintf(stderr, "Options are:\n");
      fprintf(stderr, " -a appID       chirpstack applicationID\n");
      fprintf(stderr, " -t tundev      Name of interface (default tun0)\n");
      exit(1);
      break;
    }
  }
  argc -= optind - 1;
  argv += optind - 1;

  if(argc < 2) {
    err(1, "usage: %s [-t tundev] -a appID ipaddress", prog);
  }
  slip_config_ipaddr = argv[1];

  if(*slip_config_tundev == '\0') {
    /* Use default. */
    strcpy(slip_config_tundev, "tun0");
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
