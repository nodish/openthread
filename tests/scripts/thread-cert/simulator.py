#!/usr/bin/env python
#
#  Copyright (c) 2018, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

import binascii
import bisect
import cmd
import exceptions
import os
import socket
import struct
import time
import sys
import traceback

import io
import config
import message
import pcap

class RealTime:

    def __init__(self):
        self._sniffer = config.create_default_thread_sniffer()
        self._sniffer.start()

    def set_lowpan_context(self, cid, prefix):
        self._sniffer.set_lowpan_context(cid, prefix)

    def get_messages_sent_by(self, nodeid):
        return self._sniffer.get_messages_sent_by(nodeid)

    def go(self, duration):
        time.sleep(duration)

    def stop(self):
        pass

class VirtualTime:

    OT_SIM_EVENT_ALARM_FIRED    = 0
    OT_SIM_EVENT_RADIO_RECEIVED = 1
    OT_SIM_EVENT_UART_INPUT  = 2
    OT_SIM_EVENT_UART_OUTPUT      = 3
    OT_SIM_EVENT_UART_ACK            = 4
    OT_SIM_EVENT_PRECMD      = 5
    OT_SIM_EVENT_POSTCMD      = 6

    EVENT_TIME = 0
    EVENT_SEQUENCE = 1
    EVENT_ADDR = 2
    EVENT_TYPE = 3
    EVENT_DATA_LENGTH = 4
    EVENT_DATA = 5

    BASE_PORT = 9000
    MAX_NODES = 34
    MAX_MESSAGE = 1536
    END_OF_TIME = 0x7fffffff
    PORT_OFFSET = int(os.getenv('PORT_OFFSET', '0'))

    BLOCK_TIMEOUT = 8

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        ip = '127.0.0.1'
        self.port = self.BASE_PORT + (self.PORT_OFFSET * self.MAX_NODES)
        self.addr = (ip, self.port)
        self.sock.bind(self.addr)

        self.devices = {}
        self.event_queue = []
        self.event_sequence = 0 # there could be events scheduled at exactly the same time
        self.current_time = 0
        self.current_event = None

        self._pcap = pcap.PcapCodec(os.getenv('TEST_NAME', 'current'))
        self._commander = None
        self._pause_time = 0

        self._message_factory = config.create_default_thread_message_factory()

    def __del__(self):
        if self.sock:
            self.stop()

    def stop(self):
        self.sock.close()
        self.sock = None

    def _add_message(self, nodeid, message):
        addr = ('127.0.0.1', self.port + nodeid)

        # Ignore any exceptions
        try:
            msg = self._message_factory.create(io.BytesIO(message))

            if msg is not None:
                self.devices[addr]['msgs'].append(msg)

        except Exception as e:
            traceback.print_exc()

    def set_lowpan_context(self, cid, prefix):
        self._message_factory.set_lowpan_context(cid, prefix)

    def get_messages_sent_by(self, nodeid):
        """ Get sniffed messages.

        Note! This method flushes the message queue so calling this method again will return only the newly logged messages.

        Args:
            nodeid (int): node id

        Returns:
            MessagesSet: a set with received messages.
        """
        addr = ('127.0.0.1', self.port + nodeid)

        messages = self.devices[addr]['msgs']
        self.devices[addr]['msgs'] = []

        return message.MessagesSet(messages)

    def _next_event_time(self):
        if len(self.event_queue) == 0:
            return self.END_OF_TIME
        else:
            return self.event_queue[0][0]

    def receive_events(self, block=False):
        """ Receive events until no more event are possible.

        Condition to process next event:
            1. all devices have send sleep event, so that no event will come from ncp devices.
            2. current event is done.
            3. zero delay event occurred. this will only happen when simulator is paused.
        Condition to block:
            1. There is a pending event, which means there is a running device.
            2. We are waiting for at least one event, for example, waiting for cli input event.
            3. CLI command is not finished, we are waiting for additional events.
        """
        while True:
            if self.current_event or block or (self._next_event_time() > self._pause_time and self._commander):
                self.sock.settimeout(self.BLOCK_TIMEOUT)
                try:
                    msg, addr = self.sock.recvfrom(self.MAX_MESSAGE)
                except socket.error:
                    # fail
                    print self.current_time, self.current_event
                    raise
                block = False
            else:
                self.sock.settimeout(0)
                try:
                    msg, addr = self.sock.recvfrom(self.MAX_MESSAGE)
                except socket.error:
                    break

            #print '%u: awake %u paused %u next_event_time %u pause_time %u' % (self.current_time, len(self._awake), self._paused.is_set(), self._next_event_time(), self._pause_time)
            #print self.current_event

            if addr[1] > self.BASE_PORT * 2:
                sender = addr
                addr = (addr[0], addr[1] - self.BASE_PORT)

            if addr != self.addr and addr not in self.devices:
                self.devices[addr] = {}
                self.devices[addr]['alarm'] = None
                self.devices[addr]['msgs'] = []
                self.devices[addr]['time'] = self.current_time
                #print "New device:", addr, self.devices

            delay, type, datalen = struct.unpack('=QBH', msg[:11])
            data = msg[11:]

            event_time = self.current_time + delay

            #print "New event:", type, addr

            if type == self.OT_SIM_EVENT_ALARM_FIRED:
                # remove any existing alarm event for device
                if self.devices[addr]['alarm']:
                    self.event_queue.remove(self.devices[addr]['alarm'])
                    #print "-- Remove\t", self.devices[addr]['alarm']


                # add alarm event to event queue
                event = (event_time, self.event_sequence, addr, type, datalen)
                self.event_sequence += 1
                #print "-- Enqueue\t", event, delay, self.current_time
                bisect.insort(self.event_queue, event)
                self.devices[addr]['alarm'] = event

                if self.current_event is not None and self.current_event[self.EVENT_ADDR] == addr:
                    #print "Done\t", self.current_event
                    self.current_event = None

            elif type == self.OT_SIM_EVENT_RADIO_RECEIVED:
                # add radio receive events event queue
                for device in self.devices:
                    if device != addr:
                        event = (event_time, self.event_sequence, device, type, datalen, data)
                        self.event_sequence += 1
                        #print "-- Enqueue\t", event
                        bisect.insort(self.event_queue, event)

                self._pcap.append(data, (event_time // 1000000, event_time % 1000000))
                self._add_message(addr[1] - self.port, data)

                # add radio transmit done events to event queue
                event = (event_time, self.event_sequence, addr, type, datalen, data)
                self.event_sequence += 1
                bisect.insort(self.event_queue, event)

            elif type == self.OT_SIM_EVENT_UART_INPUT:
                event = (event_time, self.event_sequence, addr, type, datalen, data)
                self.event_sequence += 1
                bisect.insort(self.event_queue, event)

                self._ack(sender)

            elif type == self.OT_SIM_EVENT_UART_OUTPUT:
                event = (event_time, self.event_sequence, addr, type, datalen, data)
                self.event_sequence += 1
                bisect.insort(self.event_queue, event)

            elif type == self.OT_SIM_EVENT_UART_ACK:
                if self.current_event is not None and self.current_event[self.EVENT_ADDR] == addr and self.current_event[self.EVENT_TYPE] == self.OT_SIM_EVENT_UART_OUTPUT:
                    self.current_event = None

            elif type == self.OT_SIM_EVENT_PRECMD:
                assert self._commander == None
                self._commander = addr

            elif type == self.OT_SIM_EVENT_POSTCMD:
                assert self._commander == addr
                self._commander = None

    def _send_message(self, message, addr):
        while True:
            try:
                sent = self.sock.sendto(message, addr)
            except socket.error:
                traceback.print_exc()
                time.sleep(0)
            else:
                break
        assert sent == len(message)

    def process_next_event(self):
        assert self.current_event is None
        assert self._next_event_time() != self.END_OF_TIME

        # process next event
        event = self.event_queue.pop(0)

        #print "Pop\t", event

        if len(event) == 5:
            event_time, sequence, addr, type, datalen = event
        else:
            event_time, sequence, addr, type, datalen, data = event

        self.current_event = event

        assert(event_time >= self.current_time)
        self.current_time = event_time

        elapsed = event_time - self.devices[addr]['time']
        self.devices[addr]['time'] = event_time

        message = struct.pack('=QBH', elapsed, type, datalen)

        if type == self.OT_SIM_EVENT_ALARM_FIRED:
            self.devices[addr]['alarm'] = None
            self._send_message(message, addr)
        elif type == self.OT_SIM_EVENT_RADIO_RECEIVED:
            message += data
            self._send_message(message, addr)
        elif type == self.OT_SIM_EVENT_UART_INPUT:
            message += data
            self._send_message(message, addr)
        elif type == self.OT_SIM_EVENT_UART_OUTPUT:
            message += data
            self._send_message(message, (addr[0], addr[1] + self.BASE_PORT))

    def _ack(self, addr):
        message = struct.pack('=QBH', 0, self.OT_SIM_EVENT_UART_ACK, 0)
        self._send_message(message, addr)

    def sync_devices(self):
        for addr in self.devices:
            elapsed = self.current_time - self.devices[addr]['time']
            if elapsed == 0:
                continue
            self.devices[addr]['time'] = self.current_time
            message = struct.pack('=QBH', elapsed, self.OT_SIM_EVENT_ALARM_FIRED, 0)
            self._send_message(message, addr)
            self.receive_events(True)

    def go(self, duration):
        assert self.current_time == self._pause_time
        duration = int(duration) * 1000000
        print "running for %d us" % duration
        self._pause_time += duration
        self.receive_events(duration == 0)
        while self._next_event_time() <= self._pause_time:
            self.process_next_event()
            self.receive_events()
        self.current_time = self._pause_time
        if duration > 0:
            self.sync_devices()
        print "current time %d us" % self.current_time


if __name__ == '__main__':
    simulator = VirtualTime()
    while True:
        simulator.go(0)
        #simulator.go(5)
        time.sleep(1)
