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
#include "tx_send_impl.h"

namespace gr {
namespace rfidmimo {

	tx_send::sptr
	tx_send::make(int num_ant, float samp_rate) {
		return gnuradio::get_initial_sptr(new tx_send_impl(num_ant, samp_rate));
	}

	tx_send_impl::tx_send_impl(int num_ant_, float samp_rate_): gr::block("tx_send",
		gr::io_signature::make(2, 2, sizeof(gr_complex)),
		gr::io_signature::make(2, 2, sizeof(gr_complex))) {
			
		num_ant = num_ant_;
		samp_rate = samp_rate_;
		
		d_samp = 1.0e6 / samp_rate; // time duration of a sample (us)


		num_samp_T1 = D_T1 / d_samp;	
		num_samp_per_bit = D_TAG_BIT / d_samp;
		num_samp_per_halfbit = num_samp_per_bit/2.0;

		// tx parameters
		num_samp_pw = D_PW / d_samp;
		num_samp_data0 = 2 * D_PW / d_samp;
		num_samp_data1 = 4 * D_PW / d_samp;
		num_samp_delim = D_DELIM / d_samp;
		num_samp_trcal = D_TRCAL / d_samp;

		num_samp_cw = D_CW / d_samp;    
		num_samp_cwquery = (D_T1 + D_T2 + D_RN16) / d_samp;  // for tag's RN16 response
		num_samp_cwack = (3 * D_T1 + D_T2 + D_EPC) / d_samp; // for tag's EPS response
		

		

		// Construct vectors (resize(0) default initialization is zero)
		data0.resize(num_samp_data0);
		data1.resize(num_samp_data1);
		delim.resize(num_samp_delim);
		rtcal.resize(num_samp_data0 + num_samp_data1);
		trcal.resize(num_samp_trcal);

		// Fill vectors with data
		std::fill_n(data0.begin(), data0.size() / 2, 1);
		std::fill_n(data1.begin(), 3 * data1.size() / 4, 1);
		std::fill_n(rtcal.begin(), rtcal.size() - num_samp_pw, 1); // RTcal
		std::fill_n(trcal.begin(), trcal.size() - num_samp_pw, 1); // TRcal

		// CW waveforms of different sizes
		cw.resize(num_samp_cw);
		cw_query.resize(num_samp_cwquery); // Sent after query/query rep
		cw_ack.resize(num_samp_cwack); // Sent after ack
		std::fill_n(cw.begin(), cw.size(), 1);
		std::fill_n(cw_query.begin(), cw_query.size(), 1);
		std::fill_n(cw_ack.begin(), cw_ack.size(), 1);

		// power down
		num_samp_pow_down = (D_POW_DOWN) / d_samp;
		pow_down.resize(num_samp_pow_down); // Power down samples

		//generate query bits
		query_bits.resize(0);
		query_bits.insert(query_bits.end(), &QUERY_CODE[0], &QUERY_CODE[4]);
		query_bits.push_back(DIVIDE_RATIO);
		query_bits.insert(query_bits.end(), &M[0], &M[2]);
		query_bits.push_back(TREXT);
		query_bits.insert(query_bits.end(), &SEL[0], &SEL[2]);
		query_bits.insert(query_bits.end(), &SESSION[0], &SESSION[2]);
		query_bits.push_back(TARGET);
		query_bits.insert(query_bits.end(), &Q_VALUE[FIXED_Q][0], &Q_VALUE[FIXED_Q][4]);
		tx_crc_append(query_bits);
		tx_bits_to_samples(query_bits, query);

		// create preamble
		preamble.insert(preamble.end(), delim.begin(), delim.end());
		preamble.insert(preamble.end(), data0.begin(), data0.end());
		preamble.insert(preamble.end(), rtcal.begin(), rtcal.end());
		preamble.insert(preamble.end(), trcal.begin(), trcal.end());

		// create framesync
		frame_sync.insert(frame_sync.end(), delim.begin(), delim.end());
		frame_sync.insert(frame_sync.end(), data0.begin(), data0.end());
		frame_sync.insert(frame_sync.end(), rtcal.begin(), rtcal.end());

		// create query rep
		query_rep.insert(query_rep.end(), frame_sync.begin(), frame_sync.end());
		query_rep.insert(query_rep.end(), data0.begin(), data0.end());
		query_rep.insert(query_rep.end(), data0.begin(), data0.end());
		query_rep.insert(query_rep.end(), data0.begin(), data0.end());
		query_rep.insert(query_rep.end(), data0.begin(), data0.end());

		// generate query adjust bits
		query_adjust_bits.resize(0);
		query_adjust_bits.insert(query_adjust_bits.end(), &QADJ_CODE[0], &QADJ_CODE[4]);
		query_adjust_bits.insert(query_adjust_bits.end(), &SESSION[0], &SESSION[2]);
		query_adjust_bits.insert(query_adjust_bits.end(), &Q_UPDN[1][0], &Q_UPDN[1][3]);
		tx_bits_to_samples(query_adjust_bits, query_adjust);

		// create nak
		nak.insert(nak.end(), frame_sync.begin(), frame_sync.end());
		nak.insert(nak.end(), data1.begin(), data1.end());
		nak.insert(nak.end(), data1.begin(), data1.end());
		nak.insert(nak.end(), data0.begin(), data0.end());
		nak.insert(nak.end(), data0.begin(), data0.end());
		nak.insert(nak.end(), data0.begin(), data0.end());
		nak.insert(nak.end(), data0.begin(), data0.end());
		nak.insert(nak.end(), data0.begin(), data0.end());
		nak.insert(nak.end(), data0.begin(), data0.end());


		// convert float to complex<float>
		pow_down_c.resize(0); 
		for (int i = 0; i < pow_down.size(); i++) pow_down_c.push_back(complex_f(pow_down[i], 0));
		cw_c.resize(0); 
		for (int i = 0; i < cw.size(); i++) cw_c.push_back(complex_f(cw[i], 0));
		cw_query_c.resize(0); 
		for (int i = 0; i < cw_query.size(); i++) cw_query_c.push_back(complex_f(cw_query[i], 0));
		cw_ack_c.resize(0); 
		for (int i = 0; i < cw_ack.size(); i++) cw_ack_c.push_back(complex_f(cw_ack[i], 0));
		preamble_c.resize(0); 
		for (int i = 0; i < preamble.size(); i++) preamble_c.push_back(complex_f(preamble[i], 0));
		frame_sync_c.resize(0); 
		for (int i = 0; i < frame_sync.size(); i++) frame_sync_c.push_back(complex_f(frame_sync[i], 0));
		query_c.resize(0); 
		for (int i = 0; i < query.size(); i++) query_c.push_back(complex_f(query[i], 0));
		query_rep_c.resize(0); 
		for (int i = 0; i < query_rep.size(); i++) query_rep_c.push_back(complex_f(query_rep[i], 0));
		query_adjust_c.resize(0); 
		for (int i = 0; i < query_adjust.size(); i++) query_adjust_c.push_back(complex_f(query_adjust[i], 0));
		nak_c.resize(0); 
		for (int i = 0; i < nak.size(); i++) nak_c.push_back(complex_f(nak[i], 0));


		/*
		// create query signal frame
		frame_query_cw.resize(0);
		for (int i = 0; i < preamble.size(); i++)
			frame_query_cw.push_back(complex_f(preamble[i], 0.0));
		for (int i = 0; i < query.size(); i++)
			frame_query_cw.push_back(complex_f(query[i], 0.0));
		for (int i = 0; i < cw_query.size(); i++)
			frame_query_cw.push_back(complex_f(cw_query[i], 0.0));

		// create query signal frame with calibration for 2 antennas
		frame_query_cw_2ant_calib = vec_2d_c(2, vec_1d_c(0));
		for (int i = 0; i < preamble.size(); i++) {
			frame_query_cw_2ant_calib[0].push_back(complex_f(preamble[i], 0));
			frame_query_cw_2ant_calib[1].push_back(complex_f(0, 0));
		}
		for (int i = 0; i < query.size(); i++) {
			frame_query_cw_2ant_calib[0].push_back(complex_f(query[i], 0));
			frame_query_cw_2ant_calib[1].push_back(complex_f(0, 0));
		}
		for (int i = 0; i < cw_query.size(); i++) {
			//if (i > num_samp_T1/4) { // to be determined
			if (NUM_SAMP_CALIB < i &&i < 2*NUM_SAMP_CALIB) { // to be determined
				frame_query_cw_2ant_calib[0].push_back(complex_f(0, 0));
				frame_query_cw_2ant_calib[1].push_back(complex_f(cw_query[i], 0));
			} else {
				frame_query_cw_2ant_calib[0].push_back(complex_f(cw_query[i], 0));
				frame_query_cw_2ant_calib[1].push_back(complex_f(0, 0));
			}
		}
		*/
		
		interf.tx_send_state = TX_SEND_STATE_TAG_INIT;    
		interf.num_tag = 1; // default setting, change by rx signal
		tx_buf = vec_2d_c(num_ant, vec_1d_c(20000));
		tx_sta_pos = 0; 
		tx_end_pos = 0;	
		g_count = 0;
		pkt_count = 0;
		for (int n = 0; n < num_ant; n++) interf.rf_calib[n] = complex_f(float(n), 0);

		//cout<<"tx send's interf addr: "<<&interf<<endl;				
		cout<<"completed tx_send block"<<endl;

	}

	tx_send_impl::~tx_send_impl() {}

	void
	tx_send_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required) {
		ninput_items_required[0] = 0;
	}

	int
	tx_send_impl::general_work(int noutput_items,
			gr_vector_int &ninput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items) {
		
		
		int beg_pos, end_pos, add_len;
		float max_val;
		complex_f ack_tmp0, ack_tmp1;
		
		
		int num_in = ninput_items[0];
		if (num_ant > 1) num_in = ninput_items[0] < ninput_items[1] ?  ninput_items[0] : ninput_items[1];
		int num_out = noutput_items;
		vector<gr_complex*> in = vector<gr_complex*>(num_ant);
		vector<gr_complex*> out = vector<gr_complex*>(num_ant);
		for (int n = 0; n < num_ant; n++) {
			in[n] = (gr_complex*) input_items[n];
			out[n] = (gr_complex*) output_items[n];
		}
		
		if (tx_end_pos < 5000) {
			
			switch (interf.tx_send_state) {

				case TX_SEND_STATE_TAG_INIT:
					memcpy(&tx_buf[0][tx_end_pos], &pow_down_c[0], sizeof(gr_complex)*pow_down_c.size() );
					for (int n = 1; n < num_ant; n++) {
						memset(&tx_buf[n][tx_end_pos], 0, sizeof(gr_complex)*pow_down_c.size() );
					}
					tx_end_pos += pow_down_c.size();
					interf.tx_send_state = TX_SEND_STATE_START;  				
					break;
			  
				case TX_SEND_STATE_START:
					memcpy(&tx_buf[0][tx_end_pos], &cw_ack_c[0], sizeof(gr_complex)*cw_ack_c.size() );
					for (int n = 1; n < num_ant; n++)
						memset(&tx_buf[n][tx_end_pos], 0, sizeof(gr_complex)*cw_ack_c.size() );
					tx_end_pos += cw_ack_c.size();
					interf.tx_send_state = TX_SEND_STATE_QUERY;    
					break;
					
				case TX_SEND_STATE_QUERY:
					// preamble
					memcpy(&tx_buf[0][tx_end_pos], &preamble_c[0], sizeof(gr_complex)*preamble_c.size() );
					for (int n = 1; n < num_ant; n++)
						memset(&tx_buf[n][tx_end_pos], 0, sizeof(gr_complex)*preamble_c.size() );
					tx_end_pos += preamble_c.size();
					interf.tx_send_state = TX_SEND_STATE_QUERY1;    
					break;
					
				case TX_SEND_STATE_QUERY1:
					// query
					memcpy(&tx_buf[0][tx_end_pos], &query_c[0], sizeof(gr_complex)*query_c.size() );
					for (int n = 1; n < num_ant; n++)
						memset(&tx_buf[n][tx_end_pos], 0, sizeof(gr_complex)*query_c.size() );
					tx_end_pos += query_c.size();
					interf.tx_send_state = TX_SEND_STATE_QUERY2;    
					break;

				case TX_SEND_STATE_QUERY2:
					// cw
					memcpy(&tx_buf[0][tx_end_pos], &cw_query_c[0], sizeof(gr_complex)*cw_query_c.size() );
					for (int n = 1; n < num_ant; n++)
						memset(&tx_buf[n][tx_end_pos], 0, sizeof(gr_complex)*cw_query_c.size() );
					tx_end_pos += cw_query_c.size();
					
					//swap signal for RF calibration
					beg_pos = tx_end_pos - cw_query_c.size() + 
									num_samp_T1 + num_samp_per_halfbit + 
									num_samp_per_bit*(NUM_TAG_PREAMBLE_BITS+NUM_RN16_BITS) + NUM_SAMP_CALIB;
					end_pos = beg_pos + NUM_SAMP_CALIB;
					for (int i = beg_pos; i < end_pos; i++) {
						tx_buf[1][i] = tx_buf[0][i];
						tx_buf[0][i] = complex_f(0, 0);
					}
					
					// cw
					add_len = cw_query_c.size()/6;
					memcpy(&tx_buf[0][tx_end_pos], &cw_query_c[0], sizeof(gr_complex)*add_len);
					for (int n = 1; n < num_ant; n++)
						memset(&tx_buf[n][tx_end_pos], 0, sizeof(gr_complex)*add_len );
					tx_end_pos += add_len;

					interf.tx_send_state = TX_SEND_STATE_QUERY_WAIT; 
					wait_count = 0;
					pkt_count++;
					//~ cout<<">>>>>>>>>>>send packet "<<pkt_count<<endl;
					
					break;
					
				case TX_SEND_STATE_ACK:
					//tx_generate_ack_signal();
					// frame_sync
					//cout<<"frame_sync_c = "<<frame_sync_c.size()<<endl;
					memcpy(&tx_buf[0][tx_end_pos], &frame_sync_c[0], sizeof(gr_complex)*frame_sync_c.size() );
					for (int n = 1; n < num_ant; n++)
						memset(&tx_buf[n][tx_end_pos], 0, sizeof(gr_complex)*frame_sync_c.size() );
					tx_end_pos += frame_sync_c.size();
					interf.tx_send_state = TX_SEND_STATE_ACK1; 
					break;
					
				case TX_SEND_STATE_ACK1:
					// ack
					//cout<<"ack_c = "<<ack_c.size()<<endl;
					if (false){//interf.num_tag == 1) {
						int num_ack_c = tx_generate_ack_signal(1);
						memcpy(&tx_buf[0][tx_end_pos], &ack_c[0], sizeof(gr_complex)*num_ack_c);			
						for (int n = 1; n < num_ant; n++)
							memset(&tx_buf[n][tx_end_pos], 0, sizeof(gr_complex)*num_ack_c);
						tx_end_pos += num_ack_c;
					}
					else {
						int num_ack_c = tx_generate_precoded_ack_signal();
						for (int n = 0; n < num_ant; n++)
							memcpy(&tx_buf[n][tx_end_pos], &ack_c_precoded[n], sizeof(gr_complex)*num_ack_c);			
						tx_end_pos += num_ack_c;
					}
					interf.tx_send_state = TX_SEND_STATE_CW_ACK;   
					break;
					
				case TX_SEND_STATE_CW_ACK:			
					// cw for ack
					//cout<<"cw_ack_c = "<<cw_ack_c.size()<<endl;	
					memset(&tx_buf[0][tx_end_pos], 0, sizeof(gr_complex)*cw_ack_c.size() );
					memcpy(&tx_buf[1][tx_end_pos], &cw_ack_c[0], sizeof(gr_complex)*cw_ack_c.size() );	
					tx_end_pos += cw_ack_c.size();
					
					// ack_tmp0 = p[0][0] + p[0][1];
					// ack_tmp1 = p[1][0] + p[1][1];
					// max_val = real(ack_tmp0) > imag(ack_tmp0) ? real(ack_tmp0) : imag(ack_tmp0);
					// if (max_val < real(ack_tmp1)) max_val = real(ack_tmp1);
					// if (max_val < imag(ack_tmp1)) max_val = imag(ack_tmp1);
					// ack_tmp0 /= max_val;
					// ack_tmp1 /= max_val;
					// for (int i = 0; i < cw_ack_c.size(); i++) {
						// tx_buf[0][tx_end_pos] = ack_tmp0;
						// tx_buf[1][tx_end_pos] = ack_tmp1;
						// tx_end_pos++;
					// }
				

					interf.tx_send_state = TX_SEND_STATE_QUERY; //TX_SEND_STATE_ACK_WAIT; //TX_SEND_STATE_QUERY;  
					wait_count = 0;
					break;

				case TX_SEND_STATE_QUERY_REP:
					// inform rx detector to search for command
					interf.rx_det_state = RX_DET_STATE_SEARCH_FOR_COMMAND_START;
					// query repeat
					for (int n = 0; n < num_ant; n++)
						memcpy(&tx_buf[n][tx_end_pos], &query_rep_c[0], sizeof(gr_complex)*query_rep_c.size() );
					tx_end_pos += query_rep_c.size();
					// cw
					for (int n = 0; n < num_ant; n++)
						memcpy(&tx_buf[n][tx_end_pos], &cw_query_c[0], sizeof(gr_complex)*cw_query_c.size() );
					tx_end_pos += cw_query_c.size();
					interf.tx_send_state = TX_SEND_STATE_WAIT;   
					wait_count = 0;
					break;
					
				case TX_SEND_STATE_QUERY_ADJUST:
					// inform rx detector to search for command
					interf.rx_det_state = RX_DET_STATE_SEARCH_FOR_COMMAND_START;			
					// frame sync
					for (int n = 0; n < num_ant; n++)
						memcpy(&tx_buf[n][tx_end_pos], &frame_sync_c[0], sizeof(gr_complex)*frame_sync_c.size() );
					tx_end_pos += frame_sync_c.size();
					// query adjust
					for (int n = 0; n < num_ant; n++)
						memcpy(&tx_buf[n][tx_end_pos], &query_adjust_c[0], sizeof(gr_complex)*query_adjust_c.size() );
					tx_end_pos += query_adjust_c.size();
					// cw
					for (int n = 0; n < num_ant; n++)
						memcpy(&tx_buf[n][tx_end_pos], &cw_query_c[0], sizeof(gr_complex)*cw_query_c.size() );
					tx_end_pos += cw_query_c.size();
					interf.tx_send_state = TX_SEND_STATE_WAIT;   
					break;

				case TX_SEND_STATE_NAK_QR:
					// inform rx detector to search for command
					interf.rx_det_state = RX_DET_STATE_SEARCH_FOR_COMMAND_START;
					// nak
					for (int n = 0; n < num_ant; n++)
						memcpy(&tx_buf[n][tx_end_pos], &nak_c[0], sizeof(gr_complex)*nak_c.size() );
					tx_end_pos += nak_c.size();
					// cw
					for (int n = 0; n < num_ant; n++)
						memcpy(&tx_buf[n][tx_end_pos], &cw_c[0], sizeof(gr_complex)*cw_c.size() );
					tx_end_pos += cw_c.size();
					interf.tx_send_state = TX_SEND_STATE_QUERY_REP;   				
					break;
					
				case TX_SEND_STATE_NAK_Q:
					// inform rx detector to search for command
					interf.rx_det_state = RX_DET_STATE_SEARCH_FOR_COMMAND_START;
					// nak
					for (int n = 0; n < num_ant; n++)
						memcpy(&tx_buf[n][tx_end_pos], &nak_c[0], sizeof(gr_complex)*nak_c.size() );
					tx_end_pos += nak_c.size();
					// cw
					for (int n = 0; n < num_ant; n++)
						memcpy(&tx_buf[n][tx_end_pos], &cw_c[0], sizeof(gr_complex)*cw_c.size() );
					tx_end_pos += cw_c.size();
					interf.tx_send_state = TX_SEND_STATE_QUERY;   				
					break;
				case TX_SEND_STATE_WAIT:
					wait_count++;
					//if (wait_count > 100) {cout<<"Error occurred!"<<endl;	exit(0);} 
				default: ;
			}

			
		}
		
		int produce_len = tx_end_pos - tx_sta_pos;
		if (produce_len > num_out) produce_len = num_out;
		//if (produce_len > 300) produce_len = 300;
		for (int n = 0; n < num_ant; n++) memcpy(&out[n][0], &tx_buf[n][tx_sta_pos], produce_len*sizeof(gr_complex));
		tx_sta_pos += produce_len;
		if (tx_sta_pos == tx_end_pos) {tx_sta_pos = 0; tx_end_pos = 0;}
		if (tx_end_pos > 15000) {cout<<"tx buf error!"<<endl; tx_sta_pos = 0; tx_end_pos = 0;}
		
		//if (produce_len > 0) 		cout<<"produce_len = "<<produce_len<<endl;


		consume_each(num_in);		
		return produce_len;
	}

	int tx_send_impl::tx_generate_ack_signal(int m) {
		//*************** compute ack bits for n tags ***************
		vec_1d_f ack_bit;
		vec_1d_f ack_tmp;
				
		ack_bit.resize(0);
		ack_bit.insert(ack_bit.end(), &ACK_CODE[0], &ACK_CODE[2]);
		ack_bit.insert(ack_bit.end(), &interf.rn16_bits[0][m][0], &interf.rn16_bits[0][m][16]);
		tx_bits_to_samples(ack_bit, ack_tmp);
		
		ack_c.resize(0);
		for (int i = 0; i < ack_tmp.size(); i++) 
			ack_c.push_back(complex_f(ack_tmp[i], 0));
		
		return ack_c.size();
	}
	
	
	int tx_send_impl::tx_generate_precoded_ack_signal() {
		
		//*************** compute ack bits for n tags ***************
		vec_2d_f ack_samp = vec_2d_f(interf.num_tag, vec_1d_f(0));
		vec_1d_f ack_bit;
		for (int m = 0; m < interf.num_tag; m++) {
			ack_bit.resize(0);
			ack_bit.insert(ack_bit.end(), &ACK_CODE[0], &ACK_CODE[2]);
			ack_bit.insert(ack_bit.end(), &interf.rn16_bits[0][m][0], &interf.rn16_bits[0][m][16]);
			tx_bits_to_samples(ack_bit, ack_samp[m]);
		}
		
		
 
		// *************** calculate downlink channel ***************
		for (int n = 0; n < num_ant; n++)
			for (int m = 0; m < interf.num_tag; m++)
				interf.h[n][m] = interf.h[n][m] / interf.rf_calib[n];
 
		// matrix inverse (2x2 only)
		complex_f a, b, c, d, det;
		a = interf.h[0][0];
		b = interf.h[0][1];
		c = interf.h[1][0];
		d = interf.h[1][1];
		det = a * d - b * c;
		// det +=  float(0.0000000001);
		
		//~ cout<<a<<"  ";
		//~ cout<<b<<"  ";
		//~ cout<<c<<"  ";
		//~ cout<<d<<"  ";
		//~ cout<<endl;
		
		
		p[0][0] = d / det;
		p[0][1] = float(-1) * b / det;
		p[1][0] = float(-1) * c / det;
		p[1][1] = a / det;



		// cout<<p[0][0]<<"  ";
		// cout<<p[0][1]<<"  ";
		// cout<<p[1][0]<<"  ";
		// cout<<p[1][1]<<"  ";
		// cout<<endl;
		
		
		int len0 = ack_samp[0].size();
		int len1 = ack_samp[1].size();
		const int num_ack_c = (len0 > len1 ? len0 : len1);
		if (len1 > len0)
			for (int i = 0; i < len1 - len0; i++) ack_samp[0].push_back(1);
		else
			for (int i = 0; i < len0 - len1; i++) ack_samp[1].push_back(1);
		

		
		//for (int i = 0; i < ack_samp[0].size(); i++) ack_samp[0][i] = 0;

		for (int i = 0; i < num_ack_c; i++) {
			for (int n = 0; n < num_ant; n++) {
				ack_c_precoded[n][i] = complex_f(0.0, 0.0);
				for (int m = 0; m < interf.num_tag; m++) {
					ack_c_precoded[n][i] += p[n][m] * ack_samp[m][i];
				}
			} 
		}

			
		// for (int i = 0; i < num_ack_c; i++) {
			// ack_c_precoded[1][i] = ack_samp[1][i];
			// ack_c_precoded[0][i] = complex_f(0, 0);
		// }
		
		
		// scale the transmit signal to keep in ADC dynamic range
		float max_val = 0;
		float tmp_val = 0;
		for (int n = 0; n < num_ant; n++) {
			for (int i = 0; i < num_ack_c; i++) {
				tmp_val = abs(real(ack_c_precoded[n][i]));
				if (max_val < tmp_val) max_val = tmp_val;
				tmp_val = abs(imag(ack_c_precoded[n][i]));				
				if (max_val < tmp_val) max_val = tmp_val;
			}
		}
		
		//cout << "max_val = " << max_val << endl;
		float coeff = 0.7/max_val;
		for (int n = 0; n < num_ant; n++) {
			for (int i = 0; i < num_ack_c; i++) {
				ack_c_precoded[n][i] *= coeff;
			}
		}
		
		return num_ack_c;
	}
 


	void tx_send_impl::tx_bits_to_samples(vec_1d_f &b, vec_1d_f &s) {
		s.resize(0);
		for (int i = 0; i < b.size(); i++) {
			if (b[i] == 1)
				s.insert(s.end(), data1.begin(), data1.end());
			else
				s.insert(s.end(), data0.begin(), data0.end());
		}
	}


	int tx_send_impl::tx_send_query_1ant() {
 	// if (usrp.tx_len != 0) return 0;
		// memcpy( &usrp.tx_buf[0][0], &frame_query_cw[0], frame_query_cw.size() * sizeof(complex_f));
		// for (int m = 1; m < num_ant; m++)
			// memset( &usrp.tx_buf[m][0], 0, frame_query_cw.size() * sizeof(complex_f));
		// usrp.tx_len = frame_query_cw.size();
		// return usrp.tx_len;
		return 0;
	}

	int tx_send_impl::tx_send_query_2ant_calib() {
		// if (usrp.tx_len != 0) return 0;
			// for (int m = 0; m < num_ant; m++)
				// memcpy( &usrp.tx_buf[m][0], &frame_query_cw_2ant_calib[m][0], frame_query_cw_2ant_calib[m].size() * sizeof(complex_f));
			// usrp.tx_len = frame_query_cw_2ant_calib[0].size();
		// return usrp.tx_len;
		return 0;
	}


	/* Function adapted from https://www.cgran.org/wiki/Gen2 */
	void tx_send_impl::tx_crc_append(std::vector < float > &q) {
		int crc[] = { 1, 0, 0, 1, 0 };

		for (int i = 0; i < 17; i++) {
			int tmp[] = { 0, 0, 0, 0, 0 };
			tmp[4] = crc[3];
			if (crc[4] == 1) {
				if (q[i] == 1) {
					tmp[0] = 0;
					tmp[1] = crc[0];
					tmp[2] = crc[1];
					tmp[3] = crc[2];
				} else {
					tmp[0] = 1;
					tmp[1] = crc[0];
					tmp[2] = crc[1];
					if (crc[2] == 1) {
						tmp[3] = 0;
					} else {
						tmp[3] = 1;
					}
				}
			} else {
				if (q[i] == 1) {
					tmp[0] = 1;
					tmp[1] = crc[0];
					tmp[2] = crc[1];
					if (crc[2] == 1) {
						tmp[3] = 0;
					} else {
						tmp[3] = 1;
					}
				} else {
					tmp[0] = 0;
					tmp[1] = crc[0];
					tmp[2] = crc[1];
					tmp[3] = crc[2];
				}
			}
			memcpy(crc, tmp, 5 * sizeof(float));
		}
		for (int i = 4; i >= 0; i--)
			q.push_back(crc[i]);
	}

} /* namespace rfidmimo */
} /* namespace gr */
