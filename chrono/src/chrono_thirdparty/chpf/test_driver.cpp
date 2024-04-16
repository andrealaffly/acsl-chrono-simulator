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

#include <fstream>
#include <vector>

// Compile with -DCHPF_USE_ZLIB to enable Zlib compression and tests
#include "particle_writer.hpp"

int main(int argc, char** argv) {

	std::vector<char> c {'a', 'b', 'c', 'd'};
	std::vector<int>  i {5, 6, 7, 8};
	std::vector<long> l {9l, 10l, 11l, 12l};

	std::ofstream output_file("test.chpf", std::ios_base::binary);
	ParticleFormatWriter pw;

	pw.write(output_file, ParticleFormatWriter::CompressionType::NONE, c, i, l);
	
	#ifdef CHPF_USE_ZLIB
	std::ofstream output_file2("test2.chpf", std::ios_base::binary);
	pw.write(output_file2, ParticleFormatWriter::CompressionType::ZLIB, c, i, l);
	#endif

	return 0;
}