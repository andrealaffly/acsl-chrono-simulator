#!/usr/bin/env python3

import struct
import zlib

# Field Types
INT8 = 0
INT16 = 1
INT32 = 2
INT64 = 3
UINT8 = 4
UINT16 = 5
UINT32 = 6
UINT64 = 7
HALF = 8
FLOAT = 9
DOUBLE = 10
UNKNOWN = 11

# Compression Types
NONE = 0
ZLIB = 1


def get_version(file):
	return get_header_v1(file)["version"]


def get_header_v1(file):
	# The first 16 bytes of the header
	HEADER_FORMAT_BASE = "<IHHII"
	HEADER_SIZE_BASE = struct.calcsize(HEADER_FORMAT_BASE)

	file.seek(0)

	# Read in the first part of the header
	buffer = file.read(HEADER_SIZE_BASE)
	magic, version, reserved, size, num_types = struct.unpack(HEADER_FORMAT_BASE, buffer)

	assert magic == 0x46506843, "ChPF Magic Number is invalid!"

	# Create a format string for the correct number of type fields
	type_string = "<" + str(num_types) + "H"
	
	# Read in the array of type fields
	buffer = file.read(struct.calcsize(type_string))
	types = struct.unpack(type_string, buffer)

	return { "magic": magic, "version": version, "__RESERVED__": reserved, "header_size": size, "num_types": num_types, "types": list(types) }


def get_header_v2(file):

	# NOTE: get_header_v1 is expected to leave the file stream exactly where it
	# finished reading data. If that behavior changes, the behavior of this 
	# function must change accordingly.
	header_data = get_header_v1(file)

	COMPRESSION_FORMAT = "<H"
	COMPRESSION_FORMAT_SIZE = struct.calcsize(COMPRESSION_FORMAT)

	buffer = file.read(COMPRESSION_FORMAT_SIZE)
	compression = struct.unpack(COMPRESSION_FORMAT, buffer)

	header_data["compression"] = compression[0]

	return header_data


def get_rows(buffer, types):
	
	def type_to_fmt(t):
		if t == INT8:
			return "b"
		elif t == INT16:
			return "h"
		elif t == INT32:
			return "i"
		elif t == INT64:
			return "q"
		elif t == UINT8:
			return "B"
		elif t == UINT16:
			return "H"
		elif t == UINT32:
			return "I"
		elif t == UINT64:
			return "Q"
		elif t == HALF:
			return "e"
		elif t == FLOAT:
			return "f"
		elif t == DOUBLE:
			return "d"
		else:
			raise TypeError("Unrecognized data type!", t)

	row_format = "".join(type_to_fmt(t) for t in types)
	return struct.iter_unpack("<"+row_format, buffer)


def read(filename):

	f = open(filename, "rb")
	
	vers = get_version(f)

	if vers == 1:
		header_data = get_header_v1(f)
	elif vers == 2:
		header_data = get_header_v2(f)
	else:
		raise ValueError("Unrecognized ChPF header version!")
	
	# Jump to the end of the header
	f.seek(header_data["header_size"])

	if "compression" in header_data:
		compression = header_data["compression"]
	else:
		compression = NONE

	if compression == ZLIB:
		return [row for row in get_rows(zlib.decompress(f.read(), 0), header_data["types"])]
	else:
		return [row for row in get_rows(f.read(), header_data["types"])]
	

def readlines(filename):
	pass


if __name__ == "__main__":
	print("This module should not be executed directly!")
