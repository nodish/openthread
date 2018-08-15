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
import threading

import io
import config
import message
import pcap

OT_SIM_EVENT_ALARM_FIRED    = 0
OT_SIM_EVENT_RADIO_RECEIVED = 1
OT_SIM_EVENT_UART_RECEIVED  = 2
OT_SIM_EVENT_UART_SENT      = 3
OT_SIM_EVENT_UART_DONE      = 4
OT_SIM_EVENT_GO             = 5

EVENT_TIME = 0
EVENT_SEQUENCE = 1
EVENT_ADDR = 2
EVENT_DATA_LENGTH = 3
EVENT_DATA = 4

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

    BASE_PORT = 9000
    MAX_NODES = 34
    END_OF_TIME = 0x7fffffff
    PORT_OFFSET = int(os.getenv('PORT_OFFSET', '0'))

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        ip = '127.0.0.1'
        self.port = self.BASE_PORT + (self.PORT_OFFSET * self.MAX_NODES)
        self.addr = (ip, self.port)
        self.sock.bind(self.addr)

        self.devices = {}
        self.event_queue = []
        self.event_count = 0
        self.event_sequence = 0 # there could be events scheduled at exactly the same time
        self.current_time = 0
        self.current_event = None

        self._pcap = pcap.PcapCodec(os.getenv('TEST_NAME', 'current'))
        self._pause_time = 0
        self._paused = threading.Event()

        self._coordinator_alive = False
        self._coordinator_thread = threading.Thread(target=self._coordinator_main_loop)
        self._coordinator_thread.daemon = True
        self._coordinator_thread.start()

        self._message_factory = config.create_default_thread_message_factory()

    def __del__(self):
        if self.sock:
            self.stop()

    def stop(self):
        self._coordinator_alive = False
        self._go(0)
        self.sock.close()
        self.sock = None
        self._coordinator_thread.join()
        self._pcap.save_to_file(os.getenv('TEST_NAME', 'current'))

    def _add_message(self, nodeid, message):
        addr = ('127.0.0.1', self.port + nodeid)

        # Ignore any exceptions
        try:
            msg = self._message_factory.create(io.BytesIO(message))

            if msg is not None:
                self.devices[addr]['msgs'].append(msg)

        except Exception as e:
            # Just print the exception to the console
            print("EXCEPTION: %s" % e)

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

    def recvfrom(self, blocking=False):
        if blocking:
            self.sock.setblocking(1)
        else:
            self.sock.setblocking(0)

        try:
            return self.sock.recvfrom(1024)
        except socket.error:
            pass


    def receive_events(self):
        while True:
            packet = self.recvfrom(self.current_event)
            if not packet:
                # we are waiting for response of an event
                if self.current_event:
                    break

                # we should process all events ahead of pause time
                if self._next_event_time() <= self._pause_time:
                    break

                # no events indicates nothing happened yet
                if self._next_event_time() < self.END_OF_TIME and not self._paused.is_set():
                    self.current_time = self._pause_time
                    self.sync_devices()
                    self._paused.set()

                packet = self.recvfrom(True)
                if not packet:
                    break

            msg, addr = packet

            if addr[1] > self.BASE_PORT * 2:
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

            if type == OT_SIM_EVENT_ALARM_FIRED:
                # remove any existing alarm event for device
                try:
                    self.event_queue.remove(self.devices[addr]['alarm'])
                    #print "-- Remove\t", self.devices[addr]['alarm']
                    self.devices[addr]['alarm'] = None
                except ValueError:
                    pass

                # add alarm event to event queue
                event = (event_time, self.event_sequence, addr, type, datalen)
                self.event_sequence += 1
                #print "-- Enqueue\t", event, delay, self.current_time
                bisect.insort(self.event_queue, event)
                self.devices[addr]['alarm'] = event

                if self.current_event is not None and self.current_event[EVENT_ADDR] == addr:
                    #print "Done\t", self.current_event
                    self.current_event = None
                    return

            elif type == OT_SIM_EVENT_RADIO_RECEIVED:
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

            elif type == OT_SIM_EVENT_UART_RECEIVED:
                event = (event_time, self.event_sequence, addr, type, datalen, data)
                self.event_sequence += 1
                bisect.insort(self.event_queue, event)

            elif type == OT_SIM_EVENT_UART_SENT:
                event = (event_time, self.event_sequence, addr, type, datalen, data)
                self.event_sequence += 1
                bisect.insort(self.event_queue, event)

            elif type == OT_SIM_EVENT_UART_DONE:
                if self.current_event is not None and self.current_event[EVENT_ADDR] == addr:
                    self.current_event = None
                    return

            elif type == OT_SIM_EVENT_GO:
                assert(addr[1] == self.port)
                self._pause_time = event_time
                return

    def process_next_event(self):

        if self.current_event != None:
            return

        if self._next_event_time() > self._pause_time:
            return

        #print "Events", len(self.event_queue)
        count = 0
        for event in self.event_queue:
            #print count, event
            count += 1

        # process next event
        try:
            event = self.event_queue.pop(0)
        except IndexError:
            return

        #print "Pop\t", event

        if len(event) == 5:
            event_time, sequence, addr, type, datalen = event
        else:
            event_time, sequence, addr, type, datalen, data = event

        if event_time == self.END_OF_TIME:
            return

        self.event_count += 1
        self.current_event = event

        assert(event_time >= self.current_time)
        self.current_time = event_time

        elapsed = event_time - self.devices[addr]['time']
        self.devices[addr]['time'] = event_time

        message = struct.pack('=QBH', elapsed, type, datalen)

        self.sock.setblocking(1)

        if type == OT_SIM_EVENT_ALARM_FIRED:
            self.devices[addr]['alarm'] = None
            sent = self.sock.sendto(message, addr)
        elif type == OT_SIM_EVENT_RADIO_RECEIVED:
            message += data
            sent = self.sock.sendto(message, addr)
        elif type == OT_SIM_EVENT_UART_RECEIVED:
            message += data
            sent = self.sock.sendto(message, addr)
        elif type == OT_SIM_EVENT_UART_SENT:
            message += data
            sent = self.sock.sendto(message, (addr[0], addr[1] + self.BASE_PORT))

    def sync_devices(self):
        for addr in self.devices:
            elapsed = self.current_time - self.devices[addr]['time']
            self.devices[addr]['time'] = self.current_time
            message = struct.pack('=QBH', elapsed, 0, 0)
            self.sock.sendto(message, addr)

    def _coordinator_main_loop(self):
        self._coordinator_alive = True
        try:
            while self._coordinator_alive:
                self.process_next_event()
                self.receive_events()
        except socket.error as e:
            print('EXCEPTION:' % e)

    def _go(self, duration):
        message = struct.pack('=QBH', duration, OT_SIM_EVENT_GO, 0)
        self.sock.sendto(message, self.addr)

    def go(self, duration):
        self._paused.clear()
        duration = int(duration) * 1000000
        print "running for %d us" % duration
        self._go(duration)
        while not self._paused.wait(1):
            pass


if __name__ == '__main__':
    simulator = VirtualTime()
    simulator.go(100)
