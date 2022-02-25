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

#ifndef INCLUDED_RFIDMIMO_TX_SEND_IMPL_H
#define INCLUDED_RFIDMIMO_TX_SEND_IMPL_H

#include <rfidmimo/tx_send.h>
#include "rfid_para.h"



namespace gr {
namespace rfidmimo {


	CONTROL_INTERFACE interf;


    class tx_send_impl : public tx_send
    {
     private:
		vec_2d_c tx_buf;
		int tx_sta_pos;
		int tx_end_pos;
		int g_count;
		int pkt_count;
		
		//******************************************
		// system parameters for both tx and rx 
		//******************************************
		int   num_ant;		
		float samp_rate;
		int wait_count;
		
		
		
		float d_samp;
		int   num_samp_T1;
		int   num_samp_per_bit;
		int   num_samp_per_halfbit;

		// number of samples 
		int num_samp_data0;
		int num_samp_data1;
		int num_samp_cw;
		int num_samp_pw;
		int num_samp_delim;
		int num_samp_trcal;
		
		// tx samples
		int num_samp_cwquery;
		int num_samp_cwack;
		int num_samp_pow_down;		
		int num_samp_query_cw;
		
		// command components
		vec_1d_f pow_down;
		vec_1d_f data0;
		vec_1d_f data1;
		vec_1d_f cw;
		vec_1d_f cw_query;
		vec_1d_f cw_ack;
		vec_1d_f delim;
		vec_1d_f rtcal;
		vec_1d_f trcal;		

		// command samples
		vec_1d_f preamble;
		vec_1d_f frame_sync;
		vec_1d_f query;
		vec_1d_f query_rep;
		vec_1d_f query_adjust;
		vec_1d_f nak;
		
		
		// command bits 
		vec_1d_f query_bits;
		vec_1d_f query_adjust_bits;
		vec_2d_f ack_bits;
		vec_2d_c ack_precoded;
		vec_2d_c ack;


		vec_1d_c pow_down_c;
		vec_1d_c cw_c;
		vec_1d_c cw_query_c;
		vec_1d_c cw_ack_c;
		vec_1d_c preamble_c;
		vec_1d_c frame_sync_c;
		vec_1d_c query_c;
		vec_1d_c query_rep_c;
		vec_1d_c query_adjust_c;
		vec_1d_c nak_c;
		vec_1d_c ack_c;

		// frames
		vec_1d_c frame_query_cw;
		vec_2d_c frame_query_cw_2ant_calib;
 
 
 		complex_f ack_c_precoded[MAX_NUM_ANT][5000];
		complex_f p[MAX_NUM_ANT][MAX_NUM_ANT];


		// ???
		int q_change; // 0-> increment, 1-> unchanged, 2-> decrement
 
 		
     public:
      tx_send_impl(int num_ant, float samp_rate);
      ~tx_send_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
   
		void tx_crc_append(vec_1d_f & q);
		int  tx_send_query_1ant();
		int  tx_send_query_2ant_calib();
		int  tx_send_ack_1ant();
		int  tx_send_ack_2ant_precoding();
		void tx_bits_to_samples(vec_1d_f & b, vec_1d_f & s);
		int  tx_generate_ack_signal(int m);
		int  tx_generate_precoded_ack_signal();
    };

} // namespace rfidmimo
} // namespace gr

#endif /* INCLUDED_RFIDMIMO_TX_SEND_IMPL_H */

