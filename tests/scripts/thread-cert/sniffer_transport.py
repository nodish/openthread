#!/usr/bin/env python
#
#  Copyright (c) 2016, The OpenThread Authors.
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

import ctypes
import os
import socket
import sys
import inotify_simple

HDLC_FLAG = 0x7e
HDLC_ESCAPE = 0x7d

# RFC 1662 Appendix C

HDLC_FCS_INIT = 0xFFFF
HDLC_FCS_POLY = 0x8408
HDLC_FCS_GOOD = 0xF0B8


class FileStream(object):
    def __init__(self, fname):
        open(fname, 'w+b').close()
        self.stream = open(fname, 'rb')
        self.inotify = inotify_simple.INotify()
        self.inotify.add_watch(fname, inotify_simple.flags.MODIFY)

    def read(self):
        while 1:
            byte = self.stream.read(1)
            if byte != '':
                return ord(byte)
            else:
                self.inotify.read()

class Hdlc(object):
    """ Utility class for HDLC encoding and decoding. """

    def __init__(self, fname):
        self.stream = FileStream(fname)
        self.fcstab = self.mkfcstab()

    @classmethod
    def mkfcstab(cls):
        """ Make a static lookup table for byte value to FCS16 result. """
        polynomial = HDLC_FCS_POLY

        def valiter():
            """ Helper to yield FCS16 table entries for each byte value. """
            for byte in range(256):
                fcs = byte
                i = 8
                while i:
                    fcs = (fcs >> 1) ^ polynomial if fcs & 1 else fcs >> 1
                    i -= 1

                yield fcs & 0xFFFF

        return tuple(valiter())

    def fcs16(self, byte, fcs):
        """
        Return the next iteration of an fcs16 calculation
        given the next data byte and current fcs accumulator.
        """
        fcs = (fcs >> 8) ^ self.fcstab[(fcs ^ byte) & 0xff]
        return fcs

    def collect(self):
        """ Return the next valid packet to pass HDLC decoding on the stream. """
        fcs = HDLC_FCS_INIT
        packet = []

        # Synchronize
        while 1:
            byte = self.stream.read()
            if byte == HDLC_FLAG:
                break

        # Read packet, updating fcs, and escaping bytes as needed
        while 1:
            byte = self.stream.read()
            if byte == HDLC_FLAG:
                if len(packet) != 0:
                    break
                else:
                    # If multiple FLAG bytes in a row, keep looking for data.
                    continue
            if byte == HDLC_ESCAPE:
                byte = self.stream.read()
                byte ^= 0x20
            packet.append(byte)
            fcs = self.fcs16(byte, fcs)

        if fcs != HDLC_FCS_GOOD:
            packet = None
        else:
            packet = packet[:-2]        # remove FCS16 from end

        return packet

    @classmethod
    def encode_byte(cls, byte, packet=[]):
        """ HDLC encode and append a single byte to the given packet. """
        if (byte == HDLC_ESCAPE) or (byte == HDLC_FLAG):
            packet.append(HDLC_ESCAPE)
            packet.append(byte ^ 0x20)
        else:
            packet.append(byte)
        return packet

    def encode(self, payload=""):
        """ Return the HDLC encoding of the given packet. """
        fcs = HDLC_FCS_INIT
        packet = []
        packet.append(HDLC_FLAG)
        for byte in payload:
            byte = ord(byte)
            fcs = self.fcs16(byte, fcs)
            packet = self.encode_byte(byte, packet)

        fcs ^= 0xffff
        byte = fcs & 0xFF
        packet = self.encode_byte(byte, packet)
        byte = fcs >> 8
        packet = self.encode_byte(byte, packet)
        packet.append(HDLC_FLAG)
        packet = pack("%dB" % len(packet), *packet)

        if CONFIG.DEBUG_HDLC:
            logging.debug("TX Hdlc: " + hexify_bytes(packet))
        return packet

    def write(self, data):
        """ HDLC encode and write the given data to this stream. """
        pkt = self.encode(data)
        self.stream.write(pkt)

    def read(self, _size=None):
        """ Read and HDLC decode the next packet from this stream. """
        pkt = self.collect()
        return pkt

class SnifferTransport(object):
    """ Interface for transport that allows eavesdrop other nodes. """

    def open(self):
        """ Open transport.

        Raises:
            RuntimeError: when transport is already opened or when transport opening failed.
        """
        raise NotImplementedError

    def close(self):
        """ Close transport.

        Raises:
            RuntimeError: when transport is already closed.
        """
        raise NotImplementedError

    @property
    def is_opened(self):
        """ Check if transport is opened.

        Returns:
            bool: True if the transport is opened, False in otherwise
        """
        raise NotImplementedError

    def send(self, data, nodeid):
        """ Send data to the node with nodeid.

        Args:
            data (bytearray): outcoming data.
            nodeid (int): node id

        Returns:
            int: number of sent bytes
        """
        raise NotImplementedError

    def recv(self, bufsize):
        """ Receive data sent by other node.

        Args:
            bufsize (int): size of buffer for incoming data.

        Returns:
            A tuple contains data and node id.

            For example:
            (bytearray([0x00, 0x01...], 1)
        """
        raise NotImplementedError


class SnifferSocketTransport(SnifferTransport):
    """ Socket based implementation of sniffer transport. """

    PORT_OFFSET = os.getenv('PORT_OFFSET', "0")

    def __init__(self, nodeid):
        self._nodeid = nodeid
        self._socket = None

    def __del__(self):
        if not self.is_opened:
            return

        self.close()

    def open(self):
        if self.is_opened:
            raise RuntimeError("Transport is already opened.")

        try:
            os.mkdir('tmp')
        except:
            pass

        fname = 'tmp/%s.radio' % self.PORT_OFFSET
        self._socket = Hdlc(fname)

        if not self.is_opened:
            raise RuntimeError("Transport opening failed.")

    def close(self):
        if not self.is_opened:
            raise RuntimeError("Transport is closed.")

        self._socket.close()
        self._socket = None

    @property
    def is_opened(self):
        return bool(self._socket is not None)

    def send2(self, data, nodeid):
        address = self._nodeid_to_address(nodeid)

        return self._socket.sendto(data, address)

    def recv(self, bufsize):
        data = self._socket.read()

        nodeid = data[0]
        data = data[1:]

        return bytearray(data), nodeid


class MacFrame(ctypes.Structure):
    _fields_ = [("buffer", ctypes.c_ubyte * 128),
                ("length", ctypes.c_ubyte),
                ("nodeid", ctypes.c_uint)]

class SnifferVirtualTransport(SnifferTransport):
    """ Virtual interface based implementation of sniffer transport. """

    def __init__(self, nodeid):
        self.Handle = None

        # Load the DLL
        self.Api = ctypes.WinDLL("otnodeapi.dll")
        if self.Api == None:
            raise OSError("Failed to load otnodeapi.dll!")

        # Define the functions
        self.Api.otListenerInit.argtypes = [ctypes.c_uint]
        self.Api.otListenerInit.restype = ctypes.c_void_p

        self.Api.otListenerFinalize.argtypes = [ctypes.c_void_p]

        self.Api.otListenerRead.argtypes = [ctypes.c_void_p, ctypes.POINTER(MacFrame)]

    def __del__(self):
        if not self.is_opened:
            return

        self.close()

    def open(self):
        if self.is_opened:
            raise RuntimeError("Transport is already opened.")

        # Initialize a listener
        self.Handle = self.Api.otListenerInit(0)

        if not self.is_opened:
            raise RuntimeError("Transport opening failed.")

    def close(self):
        if not self.is_opened:
            raise RuntimeError("Transport is closed.")

        self.Api.otListenerFinalize(self.Handle);
        self.Handle = None

    @property
    def is_opened(self):
        return bool(self.Handle is not None)

    def recv(self, bufsize):
        frame = MacFrame()
        pFrame = ctypes.pointer(frame);

        self.Api.otListenerRead(self.Handle, pFrame)

        return bytearray(frame.buffer)[:frame.length], frame.nodeid


class SnifferTransportFactory(object):

    def create_transport(self, nodeid):
        if sys.platform != "win32":
            return SnifferSocketTransport(nodeid)

        else:
            return SnifferVirtualTransport(nodeid)
