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

#include "rx_det_impl.h"

#include "rfid_para.h"


namespace gr {
namespace rfidmimo {

	rx_det::sptr
	rx_det::make(int num_ant, float samp_rate) {
		return gnuradio::get_initial_sptr(new rx_det_impl(num_ant, samp_rate));
	}


	rx_det_impl::rx_det_impl(int num_ant_, float samp_rate_): gr::block("rx_det",
		gr::io_signature::make(2, 2, sizeof(gr_complex)),
		gr::io_signature::make(2, 2, sizeof(gr_complex))) {



	
		num_ant = num_ant_;
		samp_rate = samp_rate_;
		
		interf.num_tag = 1; // default setting, change by rx signal
		d_samp = 1.0e6 / samp_rate; // time duration of a sample (us)


		num_samp_per_bit = D_TAG_BIT / d_samp;
		num_samp_per_halfbit = num_samp_per_bit/2.0;

		num_samp_T1 = D_T1 / d_samp;		
		num_samp_pw = D_PW / d_samp;		
		num_samp_rn16_data = D_RN16 / d_samp;
		num_samp_rn16_preamble = D_TAG_PREAMBLE / d_samp;
		num_samp_epc_data = D_EPC / d_samp;
		signal_state = NEG_LEVEL;

		interf.rx_det_state = RX_DET_STATE_SEARCH_FOR_COMMAND;
		

    
		fout0.open("f0.m");  
		fout1.open("f1.m");  
		fout2.open("f2.m");  
		fout3.open("f3.m");  
				
		//cout<<"rx det's interf addr: "<<&interf<<endl;		
		cout<<"completed rx_det block"<<endl;
		
		g_count = 0;
	
	}


	rx_det_impl::~rx_det_impl() {
	}


	void
	rx_det_impl::forecast(int noutput_items, gr_vector_int & ninput_items_required) {
		ninput_items_required[0] = 1;
	}

	int
	rx_det_impl::general_work(int noutput_items,
			gr_vector_int & ninput_items,
			gr_vector_const_void_star & input_items,
			gr_vector_void_star & output_items) {
			

		
		const int num_in = ninput_items[0];
		int num_out = noutput_items;
		vector<gr_complex*> in = vector<gr_complex*>(num_ant);
		vector<gr_complex*> out = vector<gr_complex*>(num_ant);
		for (int n = 0; n < num_ant; n++) {
			in[n] = (gr_complex*) input_items[n];
			out[n] = (gr_complex*) output_items[n];
		}
		
		gr_complex *rx_signal = (gr_complex*) input_items[0];
		float coarse_dc_thre;
		int produce_len = 0;
		int consume_len = num_in;		
	

		
		for (int i = 0; i < num_in; i++) {
			switch(interf.rx_det_state) {
				case RX_DET_STATE_SEARCH_FOR_COMMAND_START:		
					samp_count = 0;
					pulse_count = 0;
					signal_state = NEG_LEVEL;
					interf.rx_det_state = RX_DET_STATE_SEARCH_FOR_COMMAND;
				case RX_DET_STATE_SEARCH_FOR_COMMAND:
					// Tracking coarse dc amplitude      
					samp_ampl = abs(rx_signal[i]);			
					if(samp_ampl > 0.1) coarse_dc_ampl += 0.1*(samp_ampl - coarse_dc_ampl);
					if (coarse_dc_ampl < 0.1) coarse_dc_ampl = 0.1;
					coarse_dc_thre = 0.75*coarse_dc_ampl;
					// Positive edge -> Negative edge
					if (signal_state == POS_LEVEL && samp_ampl < coarse_dc_thre) {
						if (samp_count > 0.8*num_samp_pw && samp_count < 2*num_samp_pw)
							pulse_count++;
						else
							pulse_count = 0;
						signal_state = NEG_LEVEL;
						samp_count = 0;
					}
					// Negative edge -> Positive edge
					if (signal_state == NEG_LEVEL && samp_ampl > coarse_dc_thre) {
						signal_state = POS_LEVEL;
						samp_count = 0;
					}
					if (signal_state == POS_LEVEL && samp_ampl > coarse_dc_thre) samp_count++;
					if (pulse_count > NUM_PULSES_COMMAND) { interf.rx_det_state = RX_DET_STATE_SEARCH_FOR_DC; samp_count = 0; pulse_count = 0; signal_state = NEG_LEVEL;}
					break;
					
				case RX_DET_STATE_SEARCH_FOR_DC:	
					samp_ampl_pre = samp_ampl;
					samp_ampl = abs(rx_signal[i]);
					if (abs(samp_ampl - samp_ampl_pre) < 0.2 * samp_ampl)
						samp_count++;
					else
						samp_count = 0;
					if (samp_count >= num_samp_T1 + 16) {interf.rx_det_state = RX_DET_STATE_SEARCH_FOR_TAG_SIGNAL; samp_count = 0;}
					//if (i + num_samp_per_halfbit >=  num_samp_per_halfbit) {consume_each(i); return 0;}			
					break;
					
				case RX_DET_STATE_SEARCH_FOR_TAG_SIGNAL:
					samp_ampl_pre = samp_ampl;
					samp_ampl = abs(rx_signal[i]);	
					samp_ampl_arr[samp_count] = abs(samp_ampl - samp_ampl_pre);
					samp_count++;		
					if (samp_count >= NUM_SEARCH_SAMP) {
						int max_pos = 0;
						float max_val = 0;
						for (int j = 0; j < samp_count; j++) {
							if (samp_ampl_arr[j] > max_val) {
								max_val = samp_ampl_arr[j];
								max_pos = j;
							}
						}
						i -= (samp_count - max_pos);
						if (i < 0) i = 0; //cout<<"num_in = "<<num_in<<"  "; cout<<"error!!! i = "<<i<<endl; 
						
						offset = samp_count - max_pos;
						interf.rx_det_state = RX_DET_STATE_CAPTURE_TAG_SIGNAL; 
						interf.rx_dec_state = RX_DEC_STATE_RN16_START;						
						samp_count = 0;	
					}
					break;
				case RX_DET_STATE_CAPTURE_TAG_SIGNAL:	
					for (int n = 0; n < num_ant; n++) out[n][produce_len] = in[n][i];
					produce_len++;
					samp_count++;
					if (produce_len > num_out) cout<<"Warning from RX_DET_STATE_SEARCH_FOR_TAG_SIGNAL..."<<endl;				
					if (samp_count >= num_samp_per_bit*(NUM_TAG_PREAMBLE_BITS+NUM_RN16_BITS)) {
						samp_count = 0;
						rf_calib[0] = complex_f(0, 0);
						rf_calib[1] = complex_f(0, 0);	
						consume_len = i;
						interf.rx_det_state = RX_DET_STATE_ESTIMATE_RF_CALIB_COEFF; 
						g_count++;
						cout<<"offset = "<<offset<<endl;
						//if (g_count > 5) exit(0);
					}
					break;
				case RX_DET_STATE_ESTIMATE_RF_CALIB_COEFF:	
					// fout1<<interf.tx_send_state<<"  ";
					// fout1<<real(in[0][i])<<"+1i*("<<imag(in[0][i])<<")   ";
					// fout1<<real(in[1][i])<<"+1i*("<<imag(in[1][i])<<")   "<<endl;
					// interf.d_tmp[0][samp_count] = in[0][i];
					// interf.d_tmp[1][samp_count] = in[1][i];
					
					if (10 < samp_count && samp_count < NUM_SAMP_CALIB - 10) 
						rf_calib[1] += in[1][i];
					if (NUM_SAMP_CALIB+10 < samp_count && samp_count < 2*NUM_SAMP_CALIB - 10)
						rf_calib[0] += in[0][i];
					samp_count++;
					if (samp_count >= 2*NUM_SAMP_CALIB) {
						cur_calib_coeff = rf_calib[1]/rf_calib[0];
						if (abs(cur_calib_coeff - pre_calib_coeff) < 0.05) {
							interf.calib_coeff = cur_calib_coeff;
							interf.rf_calib[0] = rf_calib[0];
							interf.rf_calib[1] = rf_calib[1];
						}
						pre_calib_coeff = cur_calib_coeff;

						// cout<<"  **** amp = "<<abs(cur_calib_coeff)<<"    angle="<<arg(cur_calib_coeff);
						// cout<<"  >>>> amp = "<<abs(interf.calib_coeff)<<"    angle="<<arg(interf.calib_coeff)<<endl;
						interf.rx_det_state = RX_DET_STATE_SEARCH_FOR_COMMAND_START; 
						samp_count = 0;
						
						 // fout0<<real(cur_calib_coeff)<<"+1i*("<<imag(cur_calib_coeff)<<")   ";
						 // fout1<<real(interf.calib_coeff)<<"+1i*("<<imag(interf.calib_coeff)<<")   ";
					}
					break;
				case RX_DET_STATE_IDLE:		
					samp_count = 0;	
					pulse_count = 0;
					break;
				default: 
					samp_count = 0;					
			}
		}

 
		
		consume_each(consume_len);
		return produce_len;
	}


	 



} /* namespace rfidmimo */
} /* namespace gr */
