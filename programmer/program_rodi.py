#!/usr/bin/env python

# Copyright (C) 2014 SISTEMAS O.R.P.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import binascii
import struct
import select
import socket

MAX_TIMEOUT = 500
SUCCESS = "success"
FAILED = "failed"


class Data(object):
    '''
    Class containing the data from non-contiguous memory allocations
    '''
    def __init__(self, begin, data):
        self.begin = begin
        self.data = data
        self.count = len(data)

    def get_data(self):
        '''
        Method gor getting the data
        '''
        return self.data

    def get_count(self):
        '''
        Method gor getting the count
        '''
        return self.count


def parse_line(line):
    '''
    Parameters:
        line: The line to parse
    Returns:
        The size, address, type of data. The line checksum.
        True if the checksum is correct, otherwise False.
    Description:
        It parses a line from the .hex file.
    '''
    status_ok = False
    size = int(line[1:3], 16)
    address = int(line[3:7], 16)
    data_type = int(line[7:9], 16)
    next_index = (9 + size * 2)
    data = binascii.a2b_hex(line[9:next_index])
    checksum = int(line[next_index:], 16)

    # checking if checksum is correct
    test_checksum = size + (address >> 8) + (address & 0xFF) + data_type
    for byte in data:
        test_checksum += ord(byte)

    if (~(test_checksum & 0xFF) + 1) & 0xFF == checksum:
        status_ok = True

    return (size, address, data_type, data, checksum, status_ok)
# -----------------------------------------------------------------------------


def read_hex_file(chunks, path):
    '''
    Parameters:
        chunks: An array with different chunks of data.
        path: The path to the .hex file to read
    Returns:
        True if the reading was successfully, otherwise False.
    Description:
        It reads a .hex file and stores the data in memory.
    '''
    try:
        hex_file = open(path, 'r')
    except IOError:
        print "Hex file not loaded"
        return False
    line = hex_file.readline()
    if line[0] != ':':
        print "The file seems to be a not valid .hex file"
        hex_file.close()
        return False

    size, address, _, data, _, status_ok = parse_line(line.strip())
    if not status_ok:
        print "The checksum in line 1 is wrong"
        hex_file.close()
        return False

    chunks.append(Data(address, data))

    # Read the other lines
    index = 0
    count = 2
    for line in hex_file:
        size, address, _, data, _, status_ok = parse_line(line.strip())
        if not status_ok:
            print "The checksum in line", count, "is wrong"
            hex_file.close()
            return False

        if chunks[index].begin + chunks[index].count == address:
            chunks[index].count += size
            for code in data:
                chunks[index].data += code
        else:
            chunks.append(Data(address, data))
            index += 1
        count += 1

    return True


def init_client(address, port):
    '''
    Parameters:
        address: The address of the server
        port: The port to connect to
    Returns:
        The client socket
    Description:
        It connects to a socket server at the specified port.
    '''
    client = socket.socket()
    client.connect((address, int(port)))
    client.settimeout(0.5)
    return client


def wait_for(cli, response, timeout):
    '''
    Parameters:
        cli: The client socket
        response: The search string
        timeout: The maximum time in milliseconds the function can be running
        before a time out.
    Returns:
        True if the string was found, otherwise False. The received string.
    Description:
        It waits for the expected string.
    '''
    inputs = [cli]
    received = ""
    milliseconds = 0
    while milliseconds < timeout:
        rlist, _, _ = select.select(inputs, [], [], 0.001)
        if len(rlist) > 0:
            received += cli.recv(1)
            if response in received:
                return True, received
        milliseconds += 1

    return False, received


def return_data(cli, timeout, length=1):
    '''
    Parameters:
        cli: The client socket
        timeout: The maximum time in milliseconds the function can be running
        before a time out.
        length: The number of bytes to receive.
    Returns:
        True if the string has the required length, otherwise False.
        The received string.
    Description:
        It waits for the required length of bytes.
    '''
    inputs = [cli]
    received = ""
    milliseconds = 0
    while milliseconds < timeout:
        rlist, _, _ = select.select(inputs, [], [], 0.001)
        if len(rlist) > 0:
            received = cli.recv(length)
            return True, received
        milliseconds += 1

    return False, received


def acknowledge(cli):
    '''
    Parameters:
        cli: The client socket
    Returns:
        True if the string was found, otherwise False
    Description:
        It waits for the acknowledge string.
    '''
    if wait_for(cli, "\x14\x10", MAX_TIMEOUT)[0]:  # STK_INSYNC, STK_OK
        print SUCCESS
        return True
    else:
        print FAILED
        return False


def program_process(chunks, cli):
    '''
    Parameters:
        chunks: An array with different chunks of data.
        cli: The client socket
    Returns:
        Nothing
    Description:
        It starts the STK500 protocol to program the data at their respective
        memory address.
    '''
    print "Connection to Arduino bootloader:",

    cli.send("\x30\x20")  # STK_GET_SYNCH, SYNC_CRC_EOP
    if not acknowledge(cli):
        return

    print "Enter in programming mode:",
    cli.send("\x50\x20")  # STK_ENTER_PROGMODE, SYNC_CRC_EOP
    if not acknowledge(cli):
        return

    print "Read device signature:",
    cli.send("\x75\x20")  # STK_READ_SIGN, SYNC_CRC_EOP
    if wait_for(cli, "\x14", MAX_TIMEOUT)[0]:  # STK_INSYNC
        _, received = return_data(cli, MAX_TIMEOUT, 3)
        print binascii.b2a_hex(received)
        if not wait_for(cli, "\x10", MAX_TIMEOUT)[0]:  # STK_INSYNC
            print FAILED
            return
    else:
        print FAILED
        return

    for chunk in chunks:
        total = chunk.count
        if total > 0:  # avoid the last block (the last line of .hex file)
            current_page = chunk.begin
            pages = total / 0x80
            index = 0

            for _ in range(pages):
                print "Load memory address", current_page, ":",
                # STK_LOAD_ADDRESS, address, SYNC_CRC_EOP
                cli.send(struct.pack("<BHB", 0x55, current_page, 0x20))
                if not acknowledge(cli):
                    return

                print "Program memory address:",
                # STK_PROGRAM_PAGE, page size, flash memory, data, SYNC_CRC_EOP
                cli.send("\x64\x00\x80\x46" + chunk.data[index:index + 0x80] +
                         "\x20")
                if not acknowledge(cli):
                    return
                current_page += 0x40
                total -= 0x80
                index += 0x80

            if total > 0:
                print "Load memory address", current_page, ":",
                # STK_LOAD_ADDRESS, address, SYNC_CRC_EOP
                cli.send(struct.pack("<BHB", 0x55, current_page, 0x20))
                if not acknowledge(cli):
                    return

                print "Program memory address:",
                # STK_PROGRAM_PAGE, page size, flash memory, data, SYNC_CRC_EOP
                cli.send(struct.pack(">BHB", 0x64, total, 0x20) +
                         chunk.data[index:index + total] + "\x20")
                if not acknowledge(cli):
                    return

    print "Leave programming mode:",
    cli.send("\x51\x20")  # STK_LEAVE_PROGMODE, SYNC_CRC_EOP
    acknowledge(cli)


def main():
    '''
    The maimain entry point
    '''
    print "RoDI WiFi programmer 2014 (c) Gary Servin"
    print "Based on http://www.sistemasorp.es/2014/11/11/\
programando-un-arduino-remotamente-con-el-modulo-esp8266/"
    print

    address = "192.168.4.1"
    port = 1234

    received = ""
    chunks = []

    try:
        received = sys.argv[1]
    except IndexError:
        print "You need to specify an hex file"
        sys.exit(1)

    print "Connecting to RoDI..."
    try:
        sock = init_client(address, port)
    except socket.error as msg:
        print 'something\'s wrong with %s:%d. \
               Exception type is %s' % (address, port, repr(msg))
        sys.exit(1)

    print
    print "Reading hex file..."
    if read_hex_file(chunks, received.strip()):
        print
        print "Resetting RoDI"
        print
        sock.send("bootloader")
        if wait_for(sock, "\x00", 5000)[0]:
            program_process(chunks, sock)
        else:
            print "Couldn't communicate with the bootloader"

    sock.close()

if __name__ == "__main__":
    main()
