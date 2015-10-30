
/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */


#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__

    /*** USER DEFINES:  ***/
    //#define FAILURE_HANDLING
    //#define SERIAL_DEBUG

    /**********************/
    #define rf24_max(a,b) (a>b?a:b)
    #define rf24_min(a,b) (a<b?a:b)

    #define _BV(x) (1<<(x))

    #define PRIPSTR "%s"
    #define pgm_read_word(p) (*(p))
    #ifdef SERIAL_DEBUG
    	#define IF_SERIAL_DEBUG(x) ({x;})
    #else
    	#define IF_SERIAL_DEBUG(x)
    #endif


#endif // __RF24_CONFIG_H__
