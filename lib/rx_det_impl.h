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

#ifndef INCLUDED_RFIDMIMO_RX_DET_IMPL_H
#define INCLUDED_RFIDMIMO_RX_DET_IMPL_H

#include <rfidmimo/rx_det.h>
#include "rfid_para.h"
#include "string.h"
#include "stdio.h"
#include <fstream>


namespace gr {
namespace rfidmimo {


	extern CONTROL_INTERFACE interf;


	class rx_det_impl: public rx_det {
		private: 
		
			SIGNAL_STATE signal_state;
			float samp_ampl_arr[100];
			
			int offset;
			
			int   num_ant;		
			float samp_rate;
			float d_samp;

			int g_count;
			int samp_count;
			int pulse_count;
			float samp_ampl;	
			float samp_ampl_pre;
			float coarse_dc_ampl;	
			
			// tag data 
			int num_samp_per_bit;
			int num_samp_per_halfbit;
			int num_samp_T1;		
			int num_samp_pw;	
			int num_samp_rn16_preamble;				
			int num_samp_rn16_data;
			int num_samp_epc_data;	

			complex_f pre_calib_coeff, cur_calib_coeff;
			complex_f rf_calib[MAX_NUM_ANT];

			std::ofstream fout0;
			std::ofstream fout1;
			std::ofstream fout2;
			std::ofstream fout3;
	

		
 
		public: 
			rx_det_impl(int num_ant, float samp_rate);
			~rx_det_impl();

			// Where all the action really happens
			void forecast(int noutput_items, gr_vector_int & ninput_items_required);

			int general_work(int noutput_items,
				gr_vector_int & ninput_items,
				gr_vector_const_void_star & input_items,
				gr_vector_void_star & output_items);

	};

 

} // namespace rfidmimo
} // namespace gr

#endif /* INCLUDED_RFIDMIMO_RX_DET_IMPL_H */

