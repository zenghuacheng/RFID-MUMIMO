#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: v3.8.2.0-113-g729d5a98

from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
import rfidmimo


class rfid(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet")

        ##################################################
        # Variables
        ##################################################
        self.tx_gain = tx_gain = 12
        self.samp_rate = samp_rate = 2000000
        self.rx_gain = rx_gain = 0
        self.num_ant = num_ant = 2
        self.center_freq = center_freq = 915e6

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_0 = uhd.usrp_source(
            ",".join(("addr0=192.168.1.177,addr1=192.168.1.183", "")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,2)),
            ),
        )
        self.uhd_usrp_source_0.set_time_source('mimo', 0)
        self.uhd_usrp_source_0.set_clock_source('mimo', 0)
        self.uhd_usrp_source_0.set_center_freq(center_freq, 0)
        self.uhd_usrp_source_0.set_gain(rx_gain, 0)
        self.uhd_usrp_source_0.set_antenna('RX2', 0)
        self.uhd_usrp_source_0.set_bandwidth(samp_rate, 0)
        self.uhd_usrp_source_0.set_center_freq(center_freq, 1)
        self.uhd_usrp_source_0.set_gain(rx_gain, 1)
        self.uhd_usrp_source_0.set_antenna('RX2', 1)
        self.uhd_usrp_source_0.set_bandwidth(samp_rate, 1)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)

        self.uhd_usrp_source_0.set_auto_dc_offset(False, 0)
        self.uhd_usrp_source_0.set_auto_dc_offset(False, 1)
                
        
        # No synchronization enforced.
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join(("addr0=192.168.1.177,addr1=192.168.1.183", "")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,2)),
            ),
            '',
        )
        self.uhd_usrp_sink_0.set_time_source('mimo', 0)
        self.uhd_usrp_sink_0.set_clock_source('mimo', 0)
        self.uhd_usrp_sink_0.set_center_freq(center_freq, 0)
        self.uhd_usrp_sink_0.set_gain(tx_gain, 0)
        self.uhd_usrp_sink_0.set_antenna('TX/RX', 0)
        self.uhd_usrp_sink_0.set_bandwidth(samp_rate, 0)
        self.uhd_usrp_sink_0.set_center_freq(center_freq, 1)
        self.uhd_usrp_sink_0.set_gain(tx_gain, 1)
        self.uhd_usrp_sink_0.set_antenna('TX/RX', 1)
        self.uhd_usrp_sink_0.set_bandwidth(samp_rate, 1)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        # No synchronization enforced.
        self.rfidmimo_tx_send_0 = rfidmimo.tx_send(num_ant, samp_rate)
        self.rfidmimo_rx_det_0 = rfidmimo.rx_det(num_ant, samp_rate)
        self.rfidmimo_rx_dec_0 = rfidmimo.rx_dec(num_ant, samp_rate)
        self.blocks_multiply_const_vxx_1 = blocks.multiply_const_cc(0.5)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_cc(0.5)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.uhd_usrp_sink_0, 1))
        self.connect((self.blocks_multiply_const_vxx_1, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.rfidmimo_rx_dec_0, 1), (self.rfidmimo_tx_send_0, 1))
        self.connect((self.rfidmimo_rx_dec_0, 0), (self.rfidmimo_tx_send_0, 0))
        self.connect((self.rfidmimo_rx_det_0, 1), (self.rfidmimo_rx_dec_0, 1))
        self.connect((self.rfidmimo_rx_det_0, 0), (self.rfidmimo_rx_dec_0, 0))
        self.connect((self.rfidmimo_tx_send_0, 1), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.rfidmimo_tx_send_0, 0), (self.blocks_multiply_const_vxx_1, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.rfidmimo_rx_det_0, 0))
        self.connect((self.uhd_usrp_source_0, 1), (self.rfidmimo_rx_det_0, 1))


    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.uhd_usrp_sink_0.set_gain(self.tx_gain, 0)
        self.uhd_usrp_sink_0.set_gain(self.tx_gain, 1)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_sink_0.set_bandwidth(self.samp_rate, 0)
        self.uhd_usrp_sink_0.set_bandwidth(self.samp_rate, 1)
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source_0.set_bandwidth(self.samp_rate, 0)
        self.uhd_usrp_source_0.set_bandwidth(self.samp_rate, 1)

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.uhd_usrp_source_0.set_gain(self.rx_gain, 0)
        self.uhd_usrp_source_0.set_gain(self.rx_gain, 1)

    def get_num_ant(self):
        return self.num_ant

    def set_num_ant(self, num_ant):
        self.num_ant = num_ant

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.uhd_usrp_sink_0.set_center_freq(self.center_freq, 0)
        self.uhd_usrp_sink_0.set_center_freq(self.center_freq, 1)
        self.uhd_usrp_source_0.set_center_freq(self.center_freq, 0)
        self.uhd_usrp_source_0.set_center_freq(self.center_freq, 1)





def main(top_block_cls=rfid, options=None):
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print("Error: failed to enable real-time scheduling.")
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
