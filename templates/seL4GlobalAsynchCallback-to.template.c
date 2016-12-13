/*#
 *# Copyright 2014, NICTA
 *#
 *# This software may be distributed and modified according to the terms of
 *# the BSD 2-Clause license. Note that NO WARRANTY is provided.
 *# See "LICENSE_BSD2.txt" for details.
 *#
 *# @TAG(NICTA_BSD)
 #*/

#include <sel4/sel4.h>

/*? macros.show_includes(me.instance.type.includes) ?*/

/*- set is_reader = True -*/
/*- set instance = me.instance.name -*/
/*- set interface = me.interface.name -*/
/*- include 'global-endpoint.template.c' -*/
/*- set notification = pop('notification') -*/

void /*? me.interface.name ?*/_callback(seL4_Word badge);

void /*? me.interface.name ?*/__run(void) {
    while(1) {
        seL4_Word badge;
        seL4_Wait(/*? notification ?*/, &badge);
        /*? me.interface.name ?*/_callback(badge);
    }
}

seL4_CPtr /*? me.interface.name ?*/_notification(void) {
    return /*? notification ?*/;
}
