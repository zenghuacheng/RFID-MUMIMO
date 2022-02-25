/* -*- c++ -*- */
/*
 * Copyright 2021 Huacheng Zeng.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "rx_dec_impl.h"

namespace gr {
namespace rfidmimo {
	
	rx_dec::sptr
	rx_dec::make(int num_ant, float samp_rate) {
		return gnuradio::get_initial_sptr(new rx_dec_impl(num_ant, samp_rate));
	}

 
	rx_dec_impl::rx_dec_impl(int num_ant_, float samp_rate_): gr::block("rx_dec",
		gr::io_signature::make(2, 2, sizeof(gr_complex)),
		gr::io_signature::make(2, 2, sizeof(gr_complex))) {



		num_ant = num_ant_;
		samp_rate = samp_rate_;
		
		d_samp = 1.0e6 / samp_rate; // time duration of a sample (us)

		num_samp_per_bit = D_TAG_BIT / d_samp;
		num_samp_per_halfbit = num_samp_per_bit/2.0;
		num_samp_pw = D_PW / d_samp;		
		num_samp_rn16_data = D_RN16 / d_samp;
		num_samp_rn16_preamble = D_TAG_PREAMBLE / d_samp;
		num_samp_epc_data = D_EPC / d_samp;
		
		
		
		num_tag_preamble_halfbit = 2 * NUM_TAG_PREAMBLE_BITS;
		num_tag_rn16_halfbit = 2 * NUM_RN16_BITS;
		num_tag_total_halfbit = 2 * (NUM_TAG_PREAMBLE_BITS + NUM_RN16_BITS);
		samp_shift = int(num_samp_per_halfbit/2.0);


		rx_tag_halfbit_ampl = vec_2d_f(num_ant, vec_1d_f(num_tag_total_halfbit)); // include preamble
		rx_tag_halfbit_cmpl = vec_2d_c(num_ant, vec_1d_c(num_tag_total_halfbit)); // include preamble
		
		rx_tag_transit_left = vec_2d_c(num_ant, vec_1d_c(num_tag_total_halfbit+1)); // include preamble
		rx_tag_transit_rigt = vec_2d_c(num_ant, vec_1d_c(num_tag_total_halfbit+1)); // include preamble
		
		
		
		tag_halfbit = vec_2d_i(num_ant, vec_1d_i(num_tag_total_halfbit));
		tag_bits = vec_2d_i(num_ant, vec_1d_i(num_tag_total_halfbit / 2));
		halfbit_level_index = vec_1d_i(num_tag_total_halfbit);
		halfbit_level_clstr = vec_1d_f(num_tag_total_halfbit + 6);
		
		interf.rx_dec_state = RX_DEC_STATE_IDLE;
		

		
		for (int n = 0; n < MAX_NUM_ANT; n++) {
			h0[n] = complex_f(0, 0);
			h1[n] = complex_f(0, 0);
			h0ph1[n] = complex_f(0, 0);
			h0nh1[n] = complex_f(0, 0);
			h0_count[n] = 0;
			h1_count[n] = 0;
			h0ph1_count[n] = 0;
			h0nh1_count[n] = 0;
		}
		

		
		fout4.open("f4.m");  
		fout5.open("f5.m");  
		fout6.open("f6.m");  
		fout7.open("f7.m");  
		
		halfbit_count = 0;
		pkt_count = 0;
		//cout<<"rx dec's interf addr: "<<&interf<<endl;		
		cout<<"completed rx_dec block"<<endl;
		
		
	}
 
 
	rx_dec_impl::~rx_dec_impl() {}


	void
	rx_dec_impl::forecast(int noutput_items, gr_vector_int & ninput_items_required) {
		ninput_items_required[0] = 1;
	}



	int
	rx_dec_impl::general_work(int noutput_items,
		gr_vector_int & ninput_items,
		gr_vector_const_void_star & input_items,
		gr_vector_void_star & output_items) {
		
		
		int num_in = ninput_items[0];
		int consume_len = num_in;
		for (int n = 0; n < num_ant; n++) in[n] = (gr_complex* ) input_items[n];
		int num_tag_found[MAX_NUM_ANT];

		switch (interf.rx_dec_state) {			
			case RX_DEC_STATE_RN16_START:
				samp_count = 0;
				halfbit_count = 0;
				interf.rx_dec_state = RX_DEC_STATE_RN16;
			case RX_DEC_STATE_RN16: 
				for (int i = 0; i < num_in; i++) {
					
					if (samp_count%num_samp_per_halfbit == samp_shift) {
						for (int n = 0; n < num_ant; n++) {
							rx_tag_halfbit_ampl[n][halfbit_count] = abs(in[n][i]) - interf.dc_ampl[n];
							rx_tag_halfbit_cmpl[n][halfbit_count] = in[n][i] - interf.dc_comp[n];
						}
						halfbit_count++;
					}
					
					// estimate channel
					if (samp_count%num_samp_per_halfbit == 4) {
						for (int n = 0; n < num_ant; n++) rx_tag_transit_left[n][halfbit_count] = in[n][i];
					}
					if (samp_count%num_samp_per_halfbit == 21) {
						for (int n = 0; n < num_ant; n++) rx_tag_transit_rigt[n][halfbit_count] = in[n][i];
					}
					
					//fout6<<real(in[0][i])<<"+1i*("<<imag(in[0][i])<<")  ";
					//fout7<<real(in[1][i])<<"+1i*("<<imag(in[1][i])<<")  ";
					
					samp_count++;	
					if (samp_count >= num_samp_per_bit*(NUM_TAG_PREAMBLE_BITS+NUM_RN16_BITS)) {
						pkt_count++;

						for (int n = 0; n < num_ant; n++) {
							num_tag_found[n] = rx_decoding_tag_rn16(n);	
							// last RN16 bit is dummy
							for (int m = 0; m < num_tag_found[n]; m++) {
								for (int j = 0; j < NUM_RN16_BITS - 1; j++) {
									interf.rn16_bits[n][m][j] = tag_bits[m][j + NUM_TAG_PREAMBLE_BITS];
								}
							}
							
						}
						interf.num_tag = (num_tag_found[0] == num_tag_found[1] ? num_tag_found[0] : 0);
						
						interf.rx_dec_state = RX_DEC_STATE_IDLE;
						if (interf.num_tag == 2) // || interf.num_tag == 1)  
							interf.tx_send_state = TX_SEND_STATE_ACK;
						else
							interf.tx_send_state = TX_SEND_STATE_QUERY;

						// for (int k = 0; k < 2*NUM_SAMP_CALIB; k++) {
							// fout6<<real(interf.d_tmp[0][k])<<"+1i*("<<imag(interf.d_tmp[0][k])<<")  ";
							// fout7<<real(interf.d_tmp[1][k])<<"+1i*("<<imag(interf.d_tmp[1][k])<<")  ";
						// }
						
						cout << "** num_level0 = " << num_level[0];
						cout << "** num_tag0 = " << num_tag_found[0];
						cout << "** num_level1 = " << num_level[1];
						cout << "** num_tag1 = " << num_tag_found[1];
						cout << "** pkt_count = " << pkt_count;
						cout << "** tx state = "<<interf.tx_send_state <<endl<<endl;
						
						// if (interf.num_tag == 2 || interf.num_tag == 1) {
							// for (int m = 0; m < interf.num_tag; m++) {
								// for (int n = 0; n < num_ant; n++) {
									// for (int j = 0; j < NUM_RN16_BITS - 1; j++) {
										// cout<<interf.rn16_bits[n][m][j]<<"  ";
									// }
									// cout<<endl;
								// }
								// cout<<endl;
							// }
						// }
						
						// fout6<<endl;
						// fout7<<endl;
						consume_len = i;
						samp_count = 0;						
					}						
				}
				break;

			case RX_DEC_STATE_EPC_START:
				halfbit_count = 0;
				interf.rx_dec_state = RX_DEC_STATE_EPC;
			case RX_DEC_STATE_EPC:
				//interf.num_tag = rx_decoding_tag_epc(in, num_in);
				if (interf.num_tag >= 0) {
					interf.rx_dec_state = RX_DEC_STATE_IDLE;
				}
				break;				
			default: ;
		}

 
		consume_each(consume_len);
		return 0;
	}


 
 

	int
	rx_dec_impl::rx_decoding_tag_rn16(const int n) {
		int num_dec_tag = -1;
		float adapt_thre;
				
				
		halfbit_level_clstr[0] = (rx_tag_halfbit_ampl[n][2] + rx_tag_halfbit_ampl[n][4] 
								+ rx_tag_halfbit_ampl[n][5] + rx_tag_halfbit_ampl[n][7] 
								+ rx_tag_halfbit_ampl[n][8] + rx_tag_halfbit_ampl[n][9]) 
								* float(1.0 / 6); // (0, 0)
		halfbit_level_clstr[1] = (rx_tag_halfbit_ampl[n][0] + rx_tag_halfbit_ampl[n][1] 
								+ rx_tag_halfbit_ampl[n][3] + rx_tag_halfbit_ampl[n][6] 
								+ rx_tag_halfbit_ampl[n][10] + rx_tag_halfbit_ampl[n][11]) 
								* float(1.0 / 6); // (1, 1)
		adapt_thre = (0.125) * abs(halfbit_level_clstr[1] - halfbit_level_clstr[0]);
		level_count = 2;
		for (int j = 0; j < num_tag_total_halfbit; j++) {
			int k;
			for (k = 0; k < level_count; k++) {
				if (abs(rx_tag_halfbit_ampl[n][j] - halfbit_level_clstr[k]) < adapt_thre) {
					halfbit_level_index[j] = k;
					break;
				}
			}
			if (k == level_count) {
				halfbit_level_clstr[level_count] = rx_tag_halfbit_ampl[n][j];
				halfbit_level_index[j] = level_count;
				level_count++;
			}
		}
		// make sure the signel level is consistent
		if (level_count == 4 && halfbit_level_clstr[2] < halfbit_level_clstr[3]) {
			for (int j = 0; j < num_tag_total_halfbit; j++) {
				if (halfbit_level_index[j] == 2)
					halfbit_level_index[j] = 3;
				else if (halfbit_level_index[j] == 3)
					halfbit_level_index[j] = 2;
			}
		}
		// post processing
		num_dec_tag = level_count == 2 ? 1 : (level_count == 4 ? 2 : 0);
		num_level[n] = level_count;
		
		if (num_dec_tag == 1) {
			for (int j = 0; j < num_tag_total_halfbit; j++)	
				tag_halfbit[0][j] = halfbit_level_index[j];
			// convert half bits to bits 
			for (int j = 0; j < num_tag_total_halfbit / 2; j++)	
				tag_bits[0][j] = tag_halfbit[0][2*j] == tag_halfbit[0][2*j+1] ? 1 : 0;
			// last bit is dummy
			//~ for (int j = 0; j < NUM_RN16_BITS-1; j++) 
				//~ interf.rn16_bits[0][0][j] = tag_bits[0][j + NUM_TAG_PREAMBLE_BITS];
		}
		else if (num_dec_tag == 2) {
			for (int j = 0; j < num_tag_total_halfbit; j++) {
				switch (halfbit_level_index[j]) {
					case 0:
						tag_halfbit[0][j] = 0;
						tag_halfbit[1][j] = 0;
						break;
					case 1:
						tag_halfbit[0][j] = 1;
						tag_halfbit[1][j] = 1;
						break;
					case 2:
						tag_halfbit[0][j] = 1;
						tag_halfbit[1][j] = 0;
						break;
					case 3:
						tag_halfbit[0][j] = 0;
						tag_halfbit[1][j] = 1;
						break;
					default:
						cout << "tag_halfbit error!" << endl;
				}
			}
			// convert half bits to bits 
			for (int j = 0; j < num_tag_total_halfbit / 2; j++) {
				tag_bits[0][j] = tag_halfbit[0][2*j] == tag_halfbit[0][2*j+1] ? 1 : 0;
				tag_bits[1][j] = tag_halfbit[1][2*j] == tag_halfbit[1][2*j+1] ? 1 : 0;
			}
		}

		// h0[n] = complex_f(0, 0);
		// h1[n] = complex_f(0, 0);
		// h0ph1[n] = complex_f(0, 0);
		// h0nh1[n] = complex_f(0, 0);
		// h0_count[n] = 0;
		// h1_count[n] = 0;
		// h0ph1_count[n] = 0;
		// h0nh1_count[n] = 0;
	
				
		// ******************* estimate channel ********************
		if (num_dec_tag != 2) return num_dec_tag;

		int a, b;
		complex_f h_trans;
		for (int i = 1; i < num_tag_total_halfbit/2 ; i++) {
			a = halfbit_level_index[i-1];
			b = halfbit_level_index[i];
			if (a != b) {
				
				h_trans = rx_tag_transit_left[n][i] - rx_tag_transit_rigt[n][i];
					
				if (a == 0 && b == 1) {  // h0 + h1
					// h0ph1[n] = (1-c)*h0ph1[n] + c*h_trans;
					h0ph1[n] += h_trans; // h0 + h1
					h0ph1_count[n]++;
				}
				else if (a == 1 && b == 0) {   // -(h0 + h1)
					// h0ph1[n] = (1-c)*h0ph1[n] - c*h_trans; // h0 + h1				
					h0ph1[n] -= h_trans;
					h0ph1_count[n]++;
				}
				else if ((a == 0 && b == 2) || (a == 3 && b == 1)) { // h0
					// h0[n] = (1-c)*h0[n] + c*h_trans; // h0 + h1
					h0[n] += h_trans; 
					h0_count[n]++;
				}
				else if ((a == 2 && b == 0) || (a == 1 && b == 3)) { // -h0
					// h0[n] = (1-c)*h0[n] - c*h_trans; // h0 + h1
					h0[n] -= h_trans;
					h0_count[n]++;
				}
				else if ((a == 0 && b == 3) || (a == 2 && b == 1)) { // h1
					// h1[n] = (1-c)*h1[n] + c*h_trans; // h0 + h1
					h1[n] += h_trans;
					h1_count[n]++;
				}
				else if ((a == 3 && b == 0) || (a == 1 && b == 2)) { // -h1
					// h1[n] = (1-c)*h1[n] - c*h_trans; // h0 + h1
					h1[n] -= h_trans;
					h1_count[n]++;
				}
				else if (a == 3 && b == 2) { // h0-h1
					// h0nh1[n] = (1-c)*h0nh1[n] + c*h_trans; // h0 + h1
					h0nh1[n] += h_trans;
					h0nh1_count[n]++;
				}
				else if (a == 2 && b == 3) { // -(h0 - h1)
					// h0nh1[n] = (1-c)*h0nh1[n] - c*h_trans; // h0 + h1
					h0nh1[n] -= h_trans;
					h0nh1_count[n]++;
				}
			}			
		}
		if (DEBUG) cout<<"end of channel estimate"<<endl;

		float eps = 0.00000001;
		//h_ul[n][0] = h0ph1[n]/(h0ph1_count[n] + eps) + h0[n]/(h0_count[n]+eps) + h0nh1[n]/(h0nh1_count[n] + eps); // h0
		//h_ul[n][1] = h0ph1[n]/(h0ph1_count[n] + eps) + h1[n]/(h1_count[n]+eps) - h0nh1[n]/(h0nh1_count[n] + eps); // h0		

		// h_ul[n][0] = h0[n]/(h0_count[n]+eps); // h0
		// h_ul[n][1] = h1[n]/(h1_count[n]+eps); // h1		


		complex_f h0_tmp = h0[n]/(h0_count[n]+eps); // h0
		complex_f h1_tmp = h1[n]/(h1_count[n]+eps); // h1		
		complex_f h0ph1_tmp = h0ph1[n]/(h0ph1_count[n] + eps);
		complex_f h0nh1_tmp = h0nh1[n]/(h0nh1_count[n] + eps);
		
		interf.h[n][0] = float(0.5)*(h0ph1_tmp + h0nh1_tmp);
		interf.h[n][1] = float(0.5)*(h0ph1_tmp - h0nh1_tmp);
		
		
		
		// if (n == 0) {
			// fout4 <<real(h0_tmp)<<"+1i*("<<imag(h0_tmp)<<") ";
			// fout4 <<real(h1_tmp)<<"+1i*("<<imag(h1_tmp)<<") ";
			// fout4 <<real(h0ph1_tmp)<<"+1i*("<<imag(h0ph1_tmp)<<") ";
			// fout4 <<real(h0nh1_tmp)<<"+1i*("<<imag(h0nh1_tmp)<<") ";
			// fout4 <<endl;
		// }
		// if (n == 1) {
			// fout5 <<real(h0_tmp)<<"+1i*("<<imag(h0_tmp)<<") ";
			// fout5 <<real(h1_tmp)<<"+1i*("<<imag(h1_tmp)<<") ";
			// fout5 <<real(h0ph1_tmp)<<"+1i*("<<imag(h0ph1_tmp)<<") ";
			// fout5 <<real(h0nh1_tmp)<<"+1i*("<<imag(h0nh1_tmp)<<") ";
			// fout5 <<endl;
		// }


		return num_dec_tag;

	}
 



	int
	rx_dec_impl::rx_decoding_tag_epc() {
		// cout<<"to be done"<<endl;
		return 0;
	}

	int
	rx_dec_impl::rx_check_crc(char * bits, int num_bit) {
		register unsigned short i, j;
		register unsigned short crc_16, rcvd_crc;
		unsigned char * data;
		int num_bytes = num_bit / 8;
		data = (unsigned char * ) malloc(num_bytes);
		int mask;

		for (i = 0; i < num_bytes; i++) {
			mask = 0x80;
			data[i] = 0;
			for (j = 0; j < 8; j++) {
				if (bits[(i * 8) + j] == '1') {
					data[i] = data[i] | mask;
				}
				mask = mask >> 1;
			}
		}
		rcvd_crc = (data[num_bytes - 2] << 8) + data[num_bytes - 1];

		crc_16 = 0xFFFF;
		for (i = 0; i < num_bytes - 2; i++) {
			crc_16 ^= data[i] << 8;
			for (j = 0; j < 8; j++) {
				if (crc_16 & 0x8000) {
					crc_16 <<= 1;
					crc_16 ^= 0x1021;
				} else
					crc_16 <<= 1;
			}
		}
		crc_16 = ~crc_16;

		if (rcvd_crc != crc_16)
			return -1;
		else
			return 1;
	}


} /* namespace rfidmimo */
} /* namespace gr */

