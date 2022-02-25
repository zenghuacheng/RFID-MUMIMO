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

#ifndef INCLUDED_RFIDMIMO_RX_DEC_IMPL_H
#define INCLUDED_RFIDMIMO_RX_DEC_IMPL_H

#include <rfidmimo/rx_dec.h>
#include "rfid_para.h"

namespace gr {
namespace rfidmimo {

	extern CONTROL_INTERFACE interf;


	class rx_dec_impl: public rx_dec {
		private:

			int   num_ant;	
			float samp_rate;
			
			int samp_shift;
			float d_samp;

			int samp_count;
			int pkt_count;
			int halfbit_count;
			int level_count;
			int num_level[MAX_NUM_ANT];

			// rx samples
			int num_samp_per_bit;
			int num_samp_per_halfbit;
			int num_samp_T1;		
			int num_samp_pw;			
			int num_samp_rn16_preamble;	
			int num_samp_rn16_data;
			int num_samp_epc_data;	
			
			int num_tag_preamble_halfbit;
			int num_tag_rn16_halfbit;
			int num_tag_total_halfbit;

			vec_2d_f rx_tag_halfbit_ampl; // include preamble
			vec_2d_c rx_tag_halfbit_cmpl; // include preamble
			
			vec_2d_c rx_tag_transit_left; // include preamble
			vec_2d_c rx_tag_transit_rigt; // include preamble
			
			
			complex_f h0[MAX_NUM_ANT]; // h0
			complex_f h1[MAX_NUM_ANT]; // h1		
			complex_f h0ph1[MAX_NUM_ANT];
			complex_f h0nh1[MAX_NUM_ANT];
			int h0_count[MAX_NUM_ANT];
			int h1_count[MAX_NUM_ANT];
			int h0ph1_count[MAX_NUM_ANT];
			int h0nh1_count[MAX_NUM_ANT];
			
			
			vec_2d_i tag_halfbit;
			vec_2d_i tag_bits;
			vec_1d_i halfbit_level_index;
			vec_1d_f halfbit_level_clstr;
		
			gr_complex* in[MAX_NUM_ANT];
			
			std::ofstream fout4;
			std::ofstream fout5;
			std::ofstream fout6;
			std::ofstream fout7;	

		public: 
			rx_dec_impl(int num_ant, float samp_rate);
			~rx_dec_impl();

			// Where all the action really happens
			void forecast(int noutput_items, gr_vector_int & ninput_items_required);

			int general_work(int noutput_items,
				gr_vector_int & ninput_items,
				gr_vector_const_void_star & input_items,
				gr_vector_void_star & output_items);

			int rx_decoding_tag_rn16(const int n);				
			int rx_channel_estimate(const int n);				
			int rx_decoding_tag_epc();				
			int rx_check_crc(char * bits, int num_bit);
	};
} // namespace rfidmimo
} // namespace gr

#endif /* INCLUDED_RFIDMIMO_RX_DEC_IMPL_H */
