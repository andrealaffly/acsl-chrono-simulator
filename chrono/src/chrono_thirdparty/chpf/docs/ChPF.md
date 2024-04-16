# Chrono Particle Format Specification

## About
**Chrono Particle Format** (ChPF) is a tabular binary storage format for simulation output using Project Chrono. This format is designed to with built-in versioning in order to allow the creation of extensions to the initial format, `ChPFv1`.

## File Format

A ChPF file is laid out with the following structure:

- ChPF Header (up to `2^16 - 1` bytes)
- Optional padding
- Binary Data Table

### Header Format

A ChPF Header contains six or more components:

> Note: All integers are little-endian unless otherwise specified.

1. A magic number
 - One of:
  - The 4 byte UTF-8 sequence `'C'` `'h'` `'P'` `'F'`
  - The 4-byte integer `0x46506843`

2. A 2-byte integer indicating the specification version

3. Two reserved bytes, ignored

4. A 4-byte integer indicating the size of the header, including padding, in bytes

5. A 4-byte integer indicating the number of columns in the table

6. An array of 2-byte integers containing the enumerated data types of each column [see Data Types](#Data-Types)


#### Example C++ Implementation:
```cpp
struct {
	const uint8_t magic {0x43, 0x68, 0x50, 0x46};
	uint16_t version;
	uint16_t __RESERVED;
	uint32_t header_size;
	uint32_t num_types;
	uint16_t types[];
};
```

### Table Format

The data table is a series of repeating rows of binary-encoded data elements. The elements are stored sequentially, with no padding between successive elements or rows.

### Data Types

> Note: All data types are stored in little-endian order unless otherwise specified

| **Type** | **Value** | **Description** |
| ---      | ---       | ---             |
| INT8     |         0 | 8-bit signed integer |
| INT16    |         1 | 16-bit signed integer |
| INT32    |         2 | 32-bit signed integer |
| INT64    |         3 | 64-bit signed integer |
| UINT8    |         4 | 8-bit unsigned integer[¹](#footnote-1) |
| UINT16   |         5 | 16-bit unsigned integer |
| UINT32   |         6 | 32-bit unsigned integer |
| UINT64   |         7 | 64-bit unsigned integer |
| HALF     |         8 | 16-bit IEEE 754 floating point |
| FLOAT    |         9 | 32-bit IEEE 754 floating point |
| DOUBLE   |        10 | 64-bit IEEE 754 floating point |
| UNKNOWN  |      > 10 | Unsupported data type |


> <a id="footnote-1">1</a>: May also represent an 8-bit character type (i.e. `char`)