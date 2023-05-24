#ifndef SOBEL_FILTER_H_
#define SOBEL_FILTER_H_
#include <cmath>
#include <iomanip>
#include <systemc>
using namespace sc_core;

#include <tlm_utils/simple_target_socket.h>

#include <tlm>

#include "filter_def.h"

struct SobelFilter : public sc_module {
	tlm_utils::simple_target_socket<SobelFilter> tsock;

	sc_fifo<unsigned char> i_r;
	sc_fifo<unsigned char> i_g;
	sc_fifo<unsigned char> i_b;
	sc_fifo<unsigned int> o_result;

	SC_HAS_PROCESS(SobelFilter);

	SobelFilter(sc_module_name n) : sc_module(n), tsock("t_skt"), base_offset(0) {
		tsock.register_b_transport(this, &SobelFilter::blocking_transport);
		SC_THREAD(do_filter);
	}

	~SobelFilter() {}

	unsigned int base_offset;

	sc_dt::sc_uint<8> avg_r;
	sc_dt::sc_uint<8> avg_g;
	sc_dt::sc_uint<8> avg_b;

	sc_dt::sc_uint<12> temp_r[9];
	sc_dt::sc_uint<12> temp_g[9];
	sc_dt::sc_uint<12> temp_b[9];

	void do_filter() {
		{ wait(CLOCK_PERIOD, SC_NS); }
		while (true) {
			///////////////////

			avg_r = 0;
			avg_g = 0;
			avg_b = 0;

			for (unsigned int v = 0; v < MASK_Y; ++v) {
				for (unsigned int u = 0; u < MASK_X; ++u) {
					temp_r[u + 3 * v] = i_r.read();
					temp_g[u + 3 * v] = i_g.read();
					temp_b[u + 3 * v] = i_b.read();

					if (u == 1 && v == 1) {
						avg_r = avg_r;
						avg_g = avg_g;
						avg_b = avg_b;
					} else {
						avg_r = temp_r[u + 3 * v] + avg_r;
						avg_g = temp_g[u + 3 * v] + avg_g;
						avg_b = temp_b[u + 3 * v] + avg_b;
					}
				}
			}

			std::sort(begin(temp_r), begin(temp_r) + 9);
			std::sort(begin(temp_g), begin(temp_g) + 9);
			std::sort(begin(temp_b), begin(temp_b) + 9);

			avg_r = (avg_r + 2 * temp_r[4]) / 10;
			avg_g = (avg_g + 2 * temp_g[4]) / 10;
			avg_b = (avg_b + 2 * temp_b[4]) / 10;

			sc_dt::sc_uint<32> result = (0, avg_b, avg_g, avg_r);
			o_result.write(result);

			/////////////////////////////
		}
	}

	void blocking_transport(tlm::tlm_generic_payload &payload, sc_core::sc_time &delay) {
		wait(delay);
		// unsigned char *mask_ptr = payload.get_byte_enable_ptr();
		// auto len = payload.get_data_length();
		tlm::tlm_command cmd = payload.get_command();
		sc_dt::uint64 addr = payload.get_address();
		unsigned char *data_ptr = payload.get_data_ptr();

		addr -= base_offset;

		// cout << (int)data_ptr[0] << endl;
		// cout << (int)data_ptr[1] << endl;
		// cout << (int)data_ptr[2] << endl;
		word buffer;

		switch (cmd) {
			case tlm::TLM_READ_COMMAND:
				// cout << "READ" << endl;
				switch (addr) {
					case SOBEL_FILTER_RESULT_ADDR:
						buffer.uint = o_result.read();
						break;
					default:
						std::cerr << "READ Error! SobelFilter::blocking_transport: address 0x" << std::setfill('0')
						          << std::setw(8) << std::hex << addr << std::dec << " is not valid" << std::endl;
				}
				data_ptr[0] = buffer.uc[0];
				data_ptr[1] = buffer.uc[1];
				data_ptr[2] = buffer.uc[2];
				data_ptr[3] = buffer.uc[3];
				break;
			case tlm::TLM_WRITE_COMMAND:
				// cout << "WRITE" << endl;
				switch (addr) {
					case SOBEL_FILTER_R_ADDR:
						i_r.write(data_ptr[3]);
						i_g.write(data_ptr[2]);
						i_b.write(data_ptr[1]);
						break;
					default:
						std::cerr << "WRITE Error! SobelFilter::blocking_transport: address 0x" << std::setfill('0')
						          << std::setw(8) << std::hex << addr << std::dec << " is not valid" << std::endl;
				}
				break;
			case tlm::TLM_IGNORE_COMMAND:
				payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
				return;
			default:
				payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
				return;
		}
		payload.set_response_status(tlm::TLM_OK_RESPONSE);  // Always OK
	}
};
#endif
