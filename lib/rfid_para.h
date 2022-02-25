#ifndef _RFID_PARA_H_
#define _RFID_PARA_H_


#include <iostream>
#include <iterator>
#include <algorithm>
#include <vector>
#include <complex>
using namespace std;




#define div_cf(x, y)  complex<float>(x.real()/y, x.imag()/y)
#define mul_cf(x, y)  complex<float>(x.real()*y, x.imag()*y)

#define complex_f complex<float> 


// 3-dimentional vector
#define vec_3d_c  vector<vector<vector<complex_f > > >
#define vec_3d_f  vector<vector<vector<float> >>
#define vec_3d_i  vector<vector<vector<int> >>

// 2-dimentional vector
#define vec_2d_c  vector<vector<complex_f > >
#define vec_2d_f  vector<vector<float> >
#define vec_2d_i  vector<vector<int> >

// 1-dimentional vector
#define vec_1d_c  vector<complex_f >
#define vec_1d_f  vector<float>
#define vec_1d_i  vector<int>

// 3-dimentional vector
#define new_vec_3d_c(p1, p2, p3)  vec_3d_c(p1, vec_2d_c(p2, vec_1d_c(p3)))
#define new_vec_3d_f(p1, p2, p3)  vec_3d_f(p1, vec_2d_f(p2, vec_1d_f(p3)))
#define new_vec_3d_i(p1, p2, p3)  vec_3d_i(p1, vec_2d_i(p2, vec_1d_i(p3)))

// 2-dimentional vector
#define new_vec_2d_c(p1, p2)  vec_2d_c(p1, vec_1d_c(p2))
#define new_vec_2d_f(p1, p2)  vec_2d_f(p1, vec_1d_f(p2))
#define new_vec_2d_i(p1, p2)  vec_2d_i(p1, vec_1d_i(p2))

// 1-dimentional vector
#define new_vec_1d_c(p1)  vec_1d_c(p1)
#define new_vec_1d_f(p1)  vec_1d_f(p1)
#define new_vec_1d_i(p1)  vec_1d_i(p1)


const bool DEBUG = false;

//=================================================
// RFID system parameters
//=================================================

// Fixed number of slots (2^(FIXED_Q))  
const int FIXED_Q             = 0;

// Termination criteria
// const int MAX_INVENTORY_ROUND = 50;
const int MAX_NUM_QUERIES     = 1000;     // Stop after MAX_NUM_QUERIES have been sent

// valid values for Q
const int Q_VALUE [16][4] =  
{
	{0,0,0,0}, {0,0,0,1}, {0,0,1,0}, {0,0,1,1}, 
	{0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1}, 
	{1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1},
	{1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1}
};  

const bool P_DOWN = false;

const int T_READER_FREQ = 40e3;       // BLF = 40kHz

// Number of bits
const int NUM_PILOT_TONE          = 12;   // Optional
const int NUM_TAG_PREAMBLE_BITS   = 6;    // Number of preamble bits
const int NUM_RN16_BITS           = 17;   // Dummy bit at the end
const int NUM_EPC_BITS            = 129;  // PC + EPC + CRC16 + Dummy = 6 + 16 + 96 + 16 + 1 = 135
const int NUM_QUERY_BITS          = 22;   // Query length in bits


// Duration in us ('D' means 'Time Duration')
const float D_TAG_BIT  = 1.0/T_READER_FREQ * pow(10,6); // Duration in us
const int D_RN16       = (NUM_RN16_BITS + NUM_TAG_PREAMBLE_BITS) * D_TAG_BIT;
const int D_EPC        = (NUM_EPC_BITS  + NUM_TAG_PREAMBLE_BITS) * D_TAG_BIT;
const int D_TAG_PREAMBLE = NUM_TAG_PREAMBLE_BITS * D_TAG_BIT;

const int D_CW         = 250;    // Carrier wave
const int D_POW_DOWN   = 2000;   // power down
const int D_T1         = 240;    // Time from usrprogator transmission to Tag response (250 us)
const int D_MIN_T1     = 200;    // Time from usrprogator transmission to Tag response (250 us)
const int D_MAX_T1     = 260;    // Time from usrprogator transmission to Tag response (250 us)
const int D_T2         = 480;    // Time from Tag response to usrprogator transmission. Max value = 20.0 * T_tag = 500us 
const int D_PW         = 12;     // Half Tari 
const int D_DELIM      = 12;     // A preamble shall comprise a fixed-length start delimiter 12.5us +/-5%
const int D_TRCAL      = 200;    // BLF = DIVIDE_RATIO/TRCAL => 40e3 = 8/TRCAL => TRCAL = 200us
const int D_RTCAL      = 72;     // 6*PW = 72us
 

const int NUM_PULSES_COMMAND = 15;       // Number of pulses to detect a reader command
const int NUM_UNIQUE_TAGS    = 100;     // Stop after NUM_UNIQUE_TAGS have been read 




// Query command 
const int QUERY_CODE[4]  = {1,0,0,0};
const int M[2]           = {0,0};
const int SEL[2]         = {0,0};
const int SESSION[2]     = {0,0};
const int TARGET         = 0;
const int TREXT          = 0;
const int DIVIDE_RATIO   = 0;


const int NAK_CODE[8]   = {1,1,0,0,0,0,0,0};

// ACK command
const int ACK_CODE[2]   = {0,1};

// QueryAdjust command
const int QADJ_CODE[4]   = {1,0,0,1};

// 110 Increment by 1, 000 unchanged, 011 decrement by 1
const int Q_UPDN[3][3]  = { {1,1,0}, {0,0,0}, {0,1,1} };

// FM0 encoding preamble sequences
const int TAG_PREAMBLE[] = {1,1,0,1,0,0,1,0,0,0,1,1};

   

// Duration in which dc offset is estimated (D_T1 is 250)
const int D_WIN_SIZE         = 250; 
const int D_DC_SIZE          = 120;





//=================================================
// RFID implementation parameters
//=================================================
const int NUM_SEARCH_SAMP = 10;

const int MAX_NUM_ANT          = 5;
const int MAX_NUM_TAG          = 5;
const int RN16_SIGNAL_TYPE     = 1;
const int EPC_SIGNAL_TYPE      = 2;
const int NUM_SAMP_CALIB       = 50;


const float THRESH_FRACTION  = 0.7;  
const int   MAX_SEARCH_TIMES = 10;
const float RX_SIGNAL_AMP_THRESH = 0.10;
const float DC_VARI_PERCENT = 0.10;

enum SIGNAL_STATE {POS_LEVEL, NEG_LEVEL};

 
enum TX_SEND_STATE {
	TX_SEND_STATE_TAG_INIT = 0,
	TX_SEND_STATE_START = 1,
	TX_SEND_STATE_QUERY = 2,
	TX_SEND_STATE_QUERY1 = 4,
	TX_SEND_STATE_QUERY2 = 5,
	TX_SEND_STATE_ACK = 6,
	TX_SEND_STATE_ACK1 = 7,	
	TX_SEND_STATE_CW_ACK = 8,
	TX_SEND_STATE_QUERY_REP = 9,
	TX_SEND_STATE_QUERY_ADJUST = 10,
	TX_SEND_STATE_NAK_QR = 11,
	TX_SEND_STATE_NAK_Q = 12,
	TX_SEND_STATE_WAIT = 13,
	TX_SEND_STATE_QUERY_WAIT = 14,
	TX_SEND_STATE_ACK_WAIT  = 15

};

enum RX_DET_STATE {
	RX_DET_STATE_SEARCH_FOR_COMMAND_START,
	RX_DET_STATE_SEARCH_FOR_COMMAND,
	RX_DET_STATE_SEARCH_FOR_DC,
	RX_DET_STATE_SEARCH_FOR_TAG_PREAMBLE,
	RX_DET_STATE_SEARCH_FOR_TAG_SIGNAL,
	RX_DET_STATE_CAPTURE_TAG_SIGNAL,
	RX_DET_STATE_ESTIMATE_RF_CALIB_COEFF,
	RX_DET_STATE_IDLE
};

enum RX_DEC_STATE {
	RX_DEC_STATE_IDLE,	
	RX_DEC_STATE_RN16_START,	
	RX_DEC_STATE_RN16,
	RX_DEC_STATE_RN16_DECODING,
	RX_DEC_STATE_RN16_ONE_TAG,
	RX_DEC_STATE_RN16_TWO_TAG,
	RX_DEC_STATE_EPC_START,
	RX_DEC_STATE_EPC
	
};



struct CONTROL_INTERFACE {
	RX_DET_STATE  rx_det_state;
	RX_DEC_STATE  rx_dec_state;
	TX_SEND_STATE tx_send_state;
	
	int num_tag;
	float dc_ampl[MAX_NUM_ANT]; // support at most 5 antennas	
	complex_f dc_comp[MAX_NUM_ANT]; // support at most 5 antennas
	complex_f rf_calib[MAX_NUM_ANT];  // support at most 5 antennas
	complex_f h[MAX_NUM_ANT][MAX_NUM_TAG];      // support at most 5 antennas
	complex_f calib_coeff;
	
	int rn16_bits[MAX_NUM_ANT][MAX_NUM_TAG][NUM_RN16_BITS];
	
	complex_f d_tmp[MAX_NUM_ANT][1000];
};

 


 

#endif  
