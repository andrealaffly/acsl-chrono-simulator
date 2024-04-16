/*
Copyright (c) 2021, University of Wisconsin - Madison
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions 
are met:

 - Redistributions of source code must retain the above copyright notice, this 
   list of conditions and the following disclaimer. 
 - Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation 
   and/or other materials provided with the distribution. 
 - Neither the name of the nor the names of its contributors may be used to 
   endorse or promote products derived from this software without specific 
   prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CHPF_WRITER_HPP
#define CHPF_WRITER_HPP 1

#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include <type_traits>
#include <functional>

#ifndef FLOAT16_TYPE_AVAILABLE
typedef void half;
#endif

#ifdef CHPF_USE_ZLIB
#include <zlib.h>
#endif

class ParticleFormatWriter {
public:

	static const uint16_t CHPF_DEFAULT_VERSION = 2;
	static const uint16_t CHPF_OPTIMIZED_BLOCK_SIZE = 512;
	static const size_t CHPF_ZLIB_BLOCK_SIZE = 16384;

	enum class FieldType {
		INT8 = 0,
		INT16,
		INT32,
		INT64,
		UINT8,
		UINT16,
		UINT32,
		UINT64,
		HALF,
		FLOAT,
		DOUBLE,
		UNKNOWN
	};

	enum class CompressionType {
		NONE = 0,
		#ifdef CHPF_USE_ZLIB
		ZLIB
		#endif
	};

	template <typename... Ts>
	struct ParticleFormatHeader {
		const uint8_t magic[4] = {0x43, 0x68, 0x50, 0x46}; // UTF-8 letters 'C' 'h' 'P' 'F' 0x46506843
		uint16_t version = 1;
		uint16_t reserved = 0;
		uint32_t header_size = 0;
		uint32_t num_types = 0;
		uint16_t types[sizeof...(Ts)] = {0};
		uint16_t compression = 0;
	};

	template <typename... Ts>
	constexpr struct ParticleFormatHeader<Ts...> buildHeaderV1() {
		struct ParticleFormatHeader<Ts...> pfh;
		pfh.version = 1;
		pfh.header_size = sizeof(ParticleFormatHeader<Ts...>);
		pfh.num_types = sizeof...(Ts);
		
		// Do some templated type detection in the weirdest way possible
		std::array<bool, sizeof...(Ts)> int8s { std::is_same<int8_t, typename std::remove_reference<Ts>::type::value_type>::value... };
		std::array<bool, sizeof...(Ts)> int16s { std::is_same<int16_t, typename std::remove_reference<Ts>::type::value_type>::value... };
		std::array<bool, sizeof...(Ts)> int32s { std::is_same<int32_t, typename std::remove_reference<Ts>::type::value_type>::value... };
		std::array<bool, sizeof...(Ts)> int64s { std::is_same<int64_t, typename std::remove_reference<Ts>::type::value_type>::value... };
		std::array<bool, sizeof...(Ts)> uint8s { std::is_same<uint8_t, typename std::remove_reference<Ts>::type::value_type>::value... };
		std::array<bool, sizeof...(Ts)> uint16s { std::is_same<uint16_t, typename std::remove_reference<Ts>::type::value_type>::value... };
		std::array<bool, sizeof...(Ts)> uint32s { std::is_same<uint32_t, typename std::remove_reference<Ts>::type::value_type>::value... };
		std::array<bool, sizeof...(Ts)> uint64s { std::is_same<uint64_t, typename std::remove_reference<Ts>::type::value_type>::value... };
		std::array<bool, sizeof...(Ts)> halfs { std::is_same<half, typename std::remove_reference<Ts>::type::value_type>::value... };
		std::array<bool, sizeof...(Ts)> floats { std::is_same<float, typename std::remove_reference<Ts>::type::value_type>::value... };
		std::array<bool, sizeof...(Ts)> doubles { std::is_same<double, typename std::remove_reference<Ts>::type::value_type>::value... };

		std::array<bool, sizeof...(Ts)> chars { std::is_same<char, typename std::remove_reference<Ts>::type::value_type>::value... };

		// Store the detected types into their enum counterparts
		for (size_t i = 0; i < sizeof...(Ts); i++) {
			if (int8s[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::INT8);
			}
			else if (int16s[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::INT16);
			}
			else if (int32s[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::INT32);
			}
			else if (int64s[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::INT64);
			}
			else if (uint8s[i] || chars[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::UINT8);
			}
			else if (uint16s[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::UINT16);
			}
			else if (uint32s[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::UINT32);
			}
			else if (uint64s[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::UINT64);
			}
			else if (halfs[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::HALF);
			}
			else if (floats[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::FLOAT);
			}
			else if (doubles[i]) {
				pfh.types[i] = static_cast<uint16_t>(FieldType::DOUBLE);
			}
			else {
				pfh.types[i] = static_cast<uint16_t>(FieldType::UNKNOWN);
			}
		}

		return pfh;
	}

	template <typename... Ts>
	constexpr struct ParticleFormatHeader<Ts...> buildHeaderV2(CompressionType compression) {
		auto pfh = buildHeaderV1<Ts...>();
		pfh.version = 2;
		pfh.compression = static_cast<uint16_t>(compression);
		return pfh;
	}

	// template <typename... Ts>
	// auto getHeaderGeneratorByVersion(uint16_t version) {
	// 	switch(version) {
	// 		case 1:
	// 			return &buildHeaderV1;
	// 		#ifdef CHPF_USE_ZLIB
	// 		case 2:
	// 			return &buildHeaderV2;
	// 		#endif
	// 		default:
	// 			std::cerr << "Version not supported!";
	// 			return std::function<decltype(buildHeaderV1<Ts...>)>();
	// 	}
	// }

	template <typename... Ts>
	constexpr size_t getRowSize() {
		// Create an array using a parameter expansion
		std::array<size_t, sizeof...(Ts)> sizes { sizeof(typename std::remove_reference<Ts>::type::value_type)... };

		size_t accumulator = 0;
		for (size_t i = 0; i < sizeof...(Ts); i++) {
			accumulator += sizes[i];
		}
		
		return accumulator;
	}

	template <typename T, typename Allocator>
	inline void repackParams(std::vector<char>& streambuf, size_t offset, const size_t idx, const std::vector<T, Allocator>& v)
	{
		for (size_t byte = 0; byte < sizeof(T); byte++) {
			streambuf[offset + byte] = *(reinterpret_cast<const char*>(&v[idx]) + byte);
		}
	}

	template <typename T, typename Allocator, typename... Ts>
	inline void repackParams(std::vector<char>& streambuf, size_t offset, const size_t idx, const std::vector<T, Allocator>& v, Ts&& ... args) {
		
		for (size_t byte = 0; byte < sizeof(T); byte++) {
			streambuf[offset + byte] = *(reinterpret_cast<const char*>(&v[idx]) + byte);
		}

		offset += sizeof(T);
		repackParams(streambuf, offset, idx, std::forward<Ts>(args)...);
	}

	template <typename T, typename Allocator = std::allocator<T>, typename... Ts>
	void write(std::ostream& stream, CompressionType compression, const std::vector<T, Allocator>& v, Ts&&... args) {
		
		// Generate a Chrono Particle Format Header and write it to a file, 
		// then pad it to some multiple of CHPF_OPTIMIZED_BLOCK_SIZE bytes.
		auto header = buildHeaderV2<decltype(v), Ts...>(compression);
		uint16_t header_size_orig = header.header_size;
		uint16_t header_size_padding = CHPF_OPTIMIZED_BLOCK_SIZE - header.header_size % CHPF_OPTIMIZED_BLOCK_SIZE;
		
		// Set the size saved in the file to include the padding
		header.header_size += header_size_padding;
		
		// Exit before writing anything if the stream isn't sane
		if (!stream.good()) {
			return;
		}

		// write the header (struct only)
		stream.write(reinterpret_cast<char*>(&header), header_size_orig);
		
		// write the padding
		std::vector<char> padding(header_size_padding);
		stream.write(padding.data(), padding.size());

		switch (compression) {
			case CompressionType::NONE: {
				// Initialize a buffer large enough to hold one row of the file
				std::vector<char> buffer(getRowSize<decltype(v), Ts...>());
				
				// Unpack each row into a buffer and write the buffer to the file
				for (size_t i = 0; i < v.size(); i++) {
					// Horizontally repack one row of the vector parameters into a buffer
					repackParams(buffer, 0, i, v, std::forward<Ts>(args)...);
					
					// Validate that the stream is still sane
					if (!stream.good()) {
						return;
					}

					// Write the row to the output stream
					stream.write(buffer.data(), buffer.size());
				}
			} 
			break;
			#ifdef CHPF_USE_ZLIB
			case CompressionType::ZLIB: {

				// Initialize a buffer large enough to hold one line of the file
				std::vector<char> buffer(getRowSize<decltype(v), Ts...>());

				// Initialize zlib state
				z_stream zstr;
				zstr.zalloc = Z_NULL;
				zstr.zfree = Z_NULL;
				zstr.opaque = Z_NULL;
				zstr.data_type = Z_BINARY;
				if (deflateInit(&zstr, Z_DEFAULT_COMPRESSION) != Z_OK) {
					std::cerr << "Unable to initialize Zlib context!\n";
					return;
				}

				// Create a buffer large enough to hold compressed data
				std::vector<char> compressed_buffer(CHPF_ZLIB_BLOCK_SIZE);
				
				// Unpack each row into a buffer and write the buffer to the file
				for (size_t i = 0; i < v.size(); i++) {
					// Horizontally repack one row of the vector parameters into a buffer
					repackParams(buffer, 0, i, v, std::forward<Ts>(args)...);
					
					// Inform the zlib context how much (and which) data we want to compress
					zstr.avail_in = buffer.size();
					zstr.next_in = reinterpret_cast<Bytef*>(buffer.data());

					do {
						// Inform the zlib context how much room there is for output
						zstr.avail_out = compressed_buffer.size();
						zstr.next_out = reinterpret_cast<Bytef*>(&compressed_buffer[0]);

						// Attempt to read and/or output data
						int res = deflate(&zstr, Z_NO_FLUSH);
						
						if (res == Z_STREAM_ERROR) {
							std::cout << "Internal Zlib error!\n";
							break;
						}

						// If any data is ready for output, write it to the output stream
						if (zstr.avail_out != compressed_buffer.size()) {
							// Validate that the stream is still sane
							if (!stream.good()) {
								return;
							}
							// Write the row to the output stream
							stream.write(compressed_buffer.data(), compressed_buffer.size() - zstr.avail_out);
						}
					}
					while(zstr.avail_in != 0);
					
				}

				// Flush the output buffer until the stream is empty
				int deflate_res = Z_OK;
				do {
					zstr.avail_out = compressed_buffer.size();
					zstr.next_out = reinterpret_cast<Bytef*>(&compressed_buffer[0]);

					deflate_res = deflate(&zstr, Z_FINISH);

					if (zstr.avail_out != compressed_buffer.size()) {
						// Validate that the stream is still sane
						if (!stream.good()) {
							return;
						}
						// Write the row to the output stream
						stream.write(compressed_buffer.data(), compressed_buffer.size() - zstr.avail_out);
					}
				}
				while(deflate_res != Z_STREAM_END);

				// Clean up the zlib state
				deflateEnd(&zstr);
			}
			break;
			#endif
			default: {
				std::cerr << "Unsupported compression type!\n";
				return;
			}
		}
		
	}

};

#endif
