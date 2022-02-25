/* -*- c++ -*- */

#define RFIDMIMO_API

%include "gnuradio.i"           // the common stuff

//load generated python docstrings
%include "rfidmimo_swig_doc.i"

%{
#include "rfidmimo/rx_det.h"
#include "rfidmimo/rx_dec.h"
#include "rfidmimo/tx_send.h"
%}

%include "rfidmimo/rx_det.h"
GR_SWIG_BLOCK_MAGIC2(rfidmimo, rx_det);
%include "rfidmimo/rx_dec.h"
GR_SWIG_BLOCK_MAGIC2(rfidmimo, rx_dec);
%include "rfidmimo/tx_send.h"
GR_SWIG_BLOCK_MAGIC2(rfidmimo, tx_send);
