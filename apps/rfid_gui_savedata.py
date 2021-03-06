#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: v3.8.2.0-113-g729d5a98

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio.filter import firdes
import sip
from gnuradio import blocks
from gnuradio import gr
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
import rfidmimo

from gnuradio import qtgui

class rfid(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Not titled yet")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "rfid")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables
        ##################################################
        self.tx_gain = tx_gain = 10
        self.samp_rate = samp_rate = 2000000
        self.rx_gain = rx_gain =0
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
        self.qtgui_sink_x_1_0 = qtgui.sink_f(
            1024*8*4, #fftsize
            firdes.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "", #name
            False, #plotfreq
            False, #plotwaterfall
            True, #plottime
            False #plotconst
        )
        self.qtgui_sink_x_1_0.set_update_time(1.0)
        self._qtgui_sink_x_1_0_win = sip.wrapinstance(self.qtgui_sink_x_1_0.pyqwidget(), Qt.QWidget)

        self.qtgui_sink_x_1_0.enable_rf_freq(False)

        self.top_layout.addWidget(self._qtgui_sink_x_1_0_win)
        self.qtgui_sink_x_1 = qtgui.sink_f(
            1024*8*4, #fftsize
            firdes.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "", #name
            False, #plotfreq
            False, #plotwaterfall
            True, #plottime
            False #plotconst
        )
        self.qtgui_sink_x_1.set_update_time(1.0)
        self._qtgui_sink_x_1_win = sip.wrapinstance(self.qtgui_sink_x_1.pyqwidget(), Qt.QWidget)

        self.qtgui_sink_x_1.enable_rf_freq(False)

        self.top_layout.addWidget(self._qtgui_sink_x_1_win)
        self.blocks_null_sink_0_0 = blocks.null_sink(gr.sizeof_float*1)
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_float*1)
        self.blocks_multiply_const_vxx_1 = blocks.multiply_const_cc(0.7)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_cc(0.7)
        self.blocks_file_sink_1 = blocks.file_sink(gr.sizeof_gr_complex*1, '/home/hz/workspace/gr-rfid-mimo/apps/ant1.bin', False)
        self.blocks_file_sink_1.set_unbuffered(False)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_gr_complex*1, '/home/hz/workspace/gr-rfid-mimo/apps/ant0.bin', False)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.blocks_complex_to_magphase_0_0 = blocks.complex_to_magphase(1)
        self.blocks_complex_to_magphase_0 = blocks.complex_to_magphase(1)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_complex_to_magphase_0, 1), (self.blocks_null_sink_0, 0))
        self.connect((self.blocks_complex_to_magphase_0, 0), (self.qtgui_sink_x_1, 0))
        self.connect((self.blocks_complex_to_magphase_0_0, 1), (self.blocks_null_sink_0_0, 0))
        self.connect((self.blocks_complex_to_magphase_0_0, 0), (self.qtgui_sink_x_1_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.uhd_usrp_sink_0, 1))
        self.connect((self.blocks_multiply_const_vxx_1, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.rfidmimo_rx_dec_0, 1), (self.rfidmimo_tx_send_0, 1))
        self.connect((self.rfidmimo_rx_dec_0, 0), (self.rfidmimo_tx_send_0, 0))
        self.connect((self.rfidmimo_rx_det_0, 0), (self.rfidmimo_rx_dec_0, 0))
        self.connect((self.rfidmimo_rx_det_0, 1), (self.rfidmimo_rx_dec_0, 1))
        self.connect((self.rfidmimo_tx_send_0, 1), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.rfidmimo_tx_send_0, 0), (self.blocks_multiply_const_vxx_1, 0))
        self.connect((self.uhd_usrp_source_0, 1), (self.blocks_complex_to_magphase_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.blocks_complex_to_magphase_0_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.uhd_usrp_source_0, 1), (self.blocks_file_sink_1, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.rfidmimo_rx_det_0, 0))
        self.connect((self.uhd_usrp_source_0, 1), (self.rfidmimo_rx_det_0, 1))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "rfid")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

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
        self.qtgui_sink_x_1.set_frequency_range(0, self.samp_rate)
        self.qtgui_sink_x_1_0.set_frequency_range(0, self.samp_rate)
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

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    def quitting():
        tb.stop()
        tb.wait()

    qapp.aboutToQuit.connect(quitting)
    qapp.exec_()

if __name__ == '__main__':
    main()
