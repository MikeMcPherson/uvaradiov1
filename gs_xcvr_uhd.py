#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: VCC Libertas N210 Transceiver
# Author: Zach Leffke, KJ4QLP and Mike McPherson, KQ9P
# Description: Development receiver/tramsmitter for testing Lithium Radio
# GNU Radio version: 3.7.13.5
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt4 import Qt
from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import qtgui
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.qtgui import Range, RangeWidget
from optparse import OptionParser
import gpredict
import math
import pmt
import satellites
import sip
import sys
import time
from gnuradio import qtgui


class gs_xcvr_uhd(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "VCC Libertas N210 Transceiver")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("VCC Libertas N210 Transceiver")
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

        self.settings = Qt.QSettings("GNU Radio", "gs_xcvr_uhd")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())


        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = float(250e3)
        self.rx_interp = rx_interp = 24
        self.rx_decim = rx_decim = 25*5
        self.libertas_freq = libertas_freq = 401.040e6
        self.gpredict_freq = gpredict_freq = 401.040e6
        self.baud = baud = 9600
        self.variable_tag_object_0 = variable_tag_object_0 = gr.tag_utils.python_to_tag((0, pmt.intern("key"), pmt.intern("value"), pmt.intern("src")))
        self.tx_samps_per_symb = tx_samps_per_symb = 240e3/baud
        self.tx_offset = tx_offset = samp_rate / 2.0
        self.tx_interp = tx_interp = 25
        self.tx_gain = tx_gain = 21.5
        self.tx_decim = tx_decim = 24
        self.rx_samps_per_symb = rx_samps_per_symb = (samp_rate/rx_decim*rx_interp)/baud
        self.rx_offset = rx_offset = samp_rate / 2.0
        self.rx_gain = rx_gain = 31.5
        self.lpf_trans = lpf_trans = 1e3
        self.lpf_cutoff = lpf_cutoff = 12.5e3
        self.fsk_dev = fsk_dev = 10000
        self.doppler_offset = doppler_offset = gpredict_freq - libertas_freq
        self.bb_gain = bb_gain = 0.75
        self.alpha = alpha = .5

        ##################################################
        # Blocks
        ##################################################
        self._tx_gain_range = Range(0, 21.5, 0.5, 21.5, 200)
        self._tx_gain_win = RangeWidget(self._tx_gain_range, self.set_tx_gain, 'TX Gain', "counter_slider", float)
        self.top_grid_layout.addWidget(self._tx_gain_win, 9, 0, 1, 4)
        for r in range(9, 10):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._rx_gain_range = Range(0, 31.5, 1, 31.5, 200)
        self._rx_gain_win = RangeWidget(self._rx_gain_range, self.set_rx_gain, 'RX Gain', "counter_slider", float)
        self.top_grid_layout.addWidget(self._rx_gain_win, 8, 0, 1, 4)
        for r in range(8, 9):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join(("", "addr=192.168.10.2")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_source_0.set_clock_source('gpsdo', 0)
        self.uhd_usrp_source_0.set_time_source('gpsdo', 0)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(uhd.tune_request(libertas_freq, rx_offset), 0)
        self.uhd_usrp_source_0.set_gain(rx_gain, 0)
        self.uhd_usrp_source_0.set_antenna('TX/RX', 0)
        self.uhd_usrp_source_0.set_auto_dc_offset(True, 0)
        self.uhd_usrp_source_0.set_auto_iq_balance(True, 0)
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
        	",".join(("", "addr=192.168.10.2")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_sink_0.set_clock_source('gpsdo', 0)
        self.uhd_usrp_sink_0.set_time_source('gpsdo', 0)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_center_freq(uhd.tune_request(libertas_freq, tx_offset), 0)
        self.uhd_usrp_sink_0.set_gain(tx_gain, 0)
        self.uhd_usrp_sink_0.set_antenna('TX/RX', 0)
        self.satellites_pdu_to_kiss_0 = satellites.pdu_to_kiss()
        self.satellites_nrzi_encode_0 = satellites.nrzi_encode()
        self.satellites_nrzi_decode_0 = satellites.nrzi_decode()
        self.satellites_kiss_to_pdu_0 = satellites.kiss_to_pdu(True)
        self.satellites_hdlc_framer_0 = satellites.hdlc_framer(preamble_bytes=64, postamble_bytes=64)
        self.satellites_hdlc_deframer_0 = satellites.hdlc_deframer(check_fcs=True, max_length=300)
        self.rational_resampler_xxx_0_0 = filter.rational_resampler_ccc(
                interpolation=tx_interp,
                decimation=tx_decim,
                taps=None,
                fractional_bw=None,
        )
        self.rational_resampler_xxx_0 = filter.rational_resampler_ccc(
                interpolation=rx_interp,
                decimation=rx_decim,
                taps=None,
                fractional_bw=None,
        )
        self.qtgui_waterfall_sink_x_0 = qtgui.waterfall_sink_c(
        	1024, #size
        	firdes.WIN_BLACKMAN_hARRIS, #wintype
        	libertas_freq, #fc
        	samp_rate, #bw
        	"", #name
                1 #number of inputs
        )
        self.qtgui_waterfall_sink_x_0.set_update_time(0.20)
        self.qtgui_waterfall_sink_x_0.enable_grid(False)
        self.qtgui_waterfall_sink_x_0.enable_axis_labels(True)

        if not True:
          self.qtgui_waterfall_sink_x_0.disable_legend()

        if "complex" == "float" or "complex" == "msg_float":
          self.qtgui_waterfall_sink_x_0.set_plot_pos_half(not True)

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        colors = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_waterfall_sink_x_0.set_color_map(i, colors[i])
            self.qtgui_waterfall_sink_x_0.set_line_alpha(i, alphas[i])

        self.qtgui_waterfall_sink_x_0.set_intensity_range(-140, 10)

        self._qtgui_waterfall_sink_x_0_win = sip.wrapinstance(self.qtgui_waterfall_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_waterfall_sink_x_0_win, 4, 0, 4, 4)
        for r in range(4, 8):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.qtgui_freq_sink_x_1_0 = qtgui.freq_sink_c(
        	2048, #size
        	firdes.WIN_BLACKMAN_hARRIS, #wintype
        	libertas_freq, #fc
        	samp_rate / rx_decim*rx_interp, #bw
        	"RX Spectrum", #name
        	1 #number of inputs
        )
        self.qtgui_freq_sink_x_1_0.set_update_time(0.20)
        self.qtgui_freq_sink_x_1_0.set_y_axis(-150, 0)
        self.qtgui_freq_sink_x_1_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_1_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_1_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_1_0.enable_grid(True)
        self.qtgui_freq_sink_x_1_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_1_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_1_0.enable_control_panel(False)

        if not False:
          self.qtgui_freq_sink_x_1_0.disable_legend()

        if "complex" == "float" or "complex" == "msg_float":
          self.qtgui_freq_sink_x_1_0.set_plot_pos_half(not True)

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_1_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_1_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_1_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_1_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_1_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_1_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_1_0.pyqwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_freq_sink_x_1_0_win, 0, 0, 4, 4)
        for r in range(0, 4):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.low_pass_filter_0 = filter.fir_filter_ccf(1, firdes.low_pass(
        	1, samp_rate / rx_decim *rx_interp, lpf_cutoff, lpf_trans, firdes.WIN_HAMMING, 6.76))
        self.gpredict_doppler_0 = gpredict.doppler("localhost", 7356, False)
        self.gpredict_MsgPairToVar_0 = gpredict.MsgPairToVar(self.set_gpredict_freq)
        self.digital_scrambler_bb_0 = digital.scrambler_bb(0x21, 0x0, 16)
        self.digital_gmsk_mod_0 = digital.gmsk_mod(
        	samples_per_symbol=int(tx_samps_per_symb),
        	bt=alpha,
        	verbose=False,
        	log=False,
        )
        self.digital_descrambler_bb_0 = digital.descrambler_bb(0x21, 0, 16)
        self.digital_clock_recovery_mm_xx_0 = digital.clock_recovery_mm_ff(rx_samps_per_symb*(1+0.0), 0.25*0.175*0.175, 0.25, 0.175, 0.005)
        self.digital_binary_slicer_fb_0 = digital.binary_slicer_fb()
        self.blocks_tag_gate_0 = blocks.tag_gate(gr.sizeof_gr_complex * 1, False)
        self.blocks_tag_gate_0.set_single_key("")
        self.blocks_socket_pdu_1 = blocks.socket_pdu("TCP_SERVER", '', '18500', 1024, False)
        self.blocks_pdu_to_tagged_stream_1 = blocks.pdu_to_tagged_stream(blocks.byte_t, 'packet_len')
        self.blocks_pdu_to_tagged_stream_0_0 = blocks.pdu_to_tagged_stream(blocks.byte_t, 'packet_len')
        self.blocks_pack_k_bits_bb_0 = blocks.pack_k_bits_bb(8)
        self.blocks_multiply_xx_1 = blocks.multiply_vcc(1)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        self.blocks_multiply_const_vxx_0_0 = blocks.multiply_const_vcc((bb_gain, ))
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_gr_complex*1, '/zfs0/iqfiles/pass_iq.tmp', True)
        self.blocks_file_sink_0.set_unbuffered(True)
        self.analog_sig_source_x_1 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, doppler_offset, 1, 0)
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, -1 * doppler_offset, 1, 0)
        self.analog_quadrature_demod_cf_1 = analog.quadrature_demod_cf((samp_rate/rx_decim*rx_interp)/(2*math.pi*fsk_dev/8.0))
        self.analog_agc2_xx_0 = analog.agc2_cc(1e-1, 1e-2, 1.0, 1.0)
        self.analog_agc2_xx_0.set_max_gain(65536)



        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_socket_pdu_1, 'pdus'), (self.blocks_pdu_to_tagged_stream_1, 'pdus'))
        self.msg_connect((self.gpredict_doppler_0, 'freq'), (self.gpredict_MsgPairToVar_0, 'inpair'))
        self.msg_connect((self.satellites_hdlc_deframer_0, 'out'), (self.satellites_pdu_to_kiss_0, 'in'))
        self.msg_connect((self.satellites_hdlc_framer_0, 'out'), (self.blocks_pdu_to_tagged_stream_0_0, 'pdus'))
        self.msg_connect((self.satellites_kiss_to_pdu_0, 'out'), (self.satellites_hdlc_framer_0, 'in'))
        self.msg_connect((self.satellites_pdu_to_kiss_0, 'out'), (self.blocks_socket_pdu_1, 'pdus'))
        self.connect((self.analog_agc2_xx_0, 0), (self.low_pass_filter_0, 0))
        self.connect((self.analog_quadrature_demod_cf_1, 0), (self.digital_clock_recovery_mm_xx_0, 0))
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0, 1))
        self.connect((self.analog_sig_source_x_1, 0), (self.blocks_multiply_xx_1, 1))
        self.connect((self.blocks_multiply_const_vxx_0_0, 0), (self.rational_resampler_xxx_0_0, 0))
        self.connect((self.blocks_multiply_xx_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.blocks_multiply_xx_1, 0), (self.rational_resampler_xxx_0, 0))
        self.connect((self.blocks_pack_k_bits_bb_0, 0), (self.digital_gmsk_mod_0, 0))
        self.connect((self.blocks_pdu_to_tagged_stream_0_0, 0), (self.digital_scrambler_bb_0, 0))
        self.connect((self.blocks_pdu_to_tagged_stream_1, 0), (self.satellites_kiss_to_pdu_0, 0))
        self.connect((self.blocks_tag_gate_0, 0), (self.blocks_multiply_const_vxx_0_0, 0))
        self.connect((self.digital_binary_slicer_fb_0, 0), (self.digital_descrambler_bb_0, 0))
        self.connect((self.digital_clock_recovery_mm_xx_0, 0), (self.digital_binary_slicer_fb_0, 0))
        self.connect((self.digital_descrambler_bb_0, 0), (self.satellites_nrzi_decode_0, 0))
        self.connect((self.digital_gmsk_mod_0, 0), (self.blocks_tag_gate_0, 0))
        self.connect((self.digital_scrambler_bb_0, 0), (self.satellites_nrzi_encode_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.analog_quadrature_demod_cf_1, 0))
        self.connect((self.rational_resampler_xxx_0, 0), (self.analog_agc2_xx_0, 0))
        self.connect((self.rational_resampler_xxx_0_0, 0), (self.blocks_multiply_xx_0, 0))
        self.connect((self.satellites_nrzi_decode_0, 0), (self.satellites_hdlc_deframer_0, 0))
        self.connect((self.satellites_nrzi_encode_0, 0), (self.blocks_pack_k_bits_bb_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.blocks_multiply_xx_1, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.qtgui_freq_sink_x_1_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.qtgui_waterfall_sink_x_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "gs_xcvr_uhd")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_tx_offset(self.samp_rate / 2.0)
        self.set_rx_samps_per_symb((self.samp_rate/self.rx_decim*self.rx_interp)/self.baud)
        self.set_rx_offset(self.samp_rate / 2.0)
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)
        self.qtgui_waterfall_sink_x_0.set_frequency_range(self.libertas_freq, self.samp_rate)
        self.qtgui_freq_sink_x_1_0.set_frequency_range(self.libertas_freq, self.samp_rate / self.rx_decim*self.rx_interp)
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate / self.rx_decim *self.rx_interp, self.lpf_cutoff, self.lpf_trans, firdes.WIN_HAMMING, 6.76))
        self.analog_sig_source_x_1.set_sampling_freq(self.samp_rate)
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)
        self.analog_quadrature_demod_cf_1.set_gain((self.samp_rate/self.rx_decim*self.rx_interp)/(2*math.pi*self.fsk_dev/8.0))

    def get_rx_interp(self):
        return self.rx_interp

    def set_rx_interp(self, rx_interp):
        self.rx_interp = rx_interp
        self.set_rx_samps_per_symb((self.samp_rate/self.rx_decim*self.rx_interp)/self.baud)
        self.qtgui_freq_sink_x_1_0.set_frequency_range(self.libertas_freq, self.samp_rate / self.rx_decim*self.rx_interp)
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate / self.rx_decim *self.rx_interp, self.lpf_cutoff, self.lpf_trans, firdes.WIN_HAMMING, 6.76))
        self.analog_quadrature_demod_cf_1.set_gain((self.samp_rate/self.rx_decim*self.rx_interp)/(2*math.pi*self.fsk_dev/8.0))

    def get_rx_decim(self):
        return self.rx_decim

    def set_rx_decim(self, rx_decim):
        self.rx_decim = rx_decim
        self.set_rx_samps_per_symb((self.samp_rate/self.rx_decim*self.rx_interp)/self.baud)
        self.qtgui_freq_sink_x_1_0.set_frequency_range(self.libertas_freq, self.samp_rate / self.rx_decim*self.rx_interp)
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate / self.rx_decim *self.rx_interp, self.lpf_cutoff, self.lpf_trans, firdes.WIN_HAMMING, 6.76))
        self.analog_quadrature_demod_cf_1.set_gain((self.samp_rate/self.rx_decim*self.rx_interp)/(2*math.pi*self.fsk_dev/8.0))

    def get_libertas_freq(self):
        return self.libertas_freq

    def set_libertas_freq(self, libertas_freq):
        self.libertas_freq = libertas_freq
        self.set_doppler_offset(self.gpredict_freq - self.libertas_freq)
        self.uhd_usrp_source_0.set_center_freq(uhd.tune_request(self.libertas_freq, self.rx_offset), 0)
        self.uhd_usrp_sink_0.set_center_freq(uhd.tune_request(self.libertas_freq, self.tx_offset), 0)
        self.qtgui_waterfall_sink_x_0.set_frequency_range(self.libertas_freq, self.samp_rate)
        self.qtgui_freq_sink_x_1_0.set_frequency_range(self.libertas_freq, self.samp_rate / self.rx_decim*self.rx_interp)

    def get_gpredict_freq(self):
        return self.gpredict_freq

    def set_gpredict_freq(self, gpredict_freq):
        self.gpredict_freq = gpredict_freq
        self.set_doppler_offset(self.gpredict_freq - self.libertas_freq)

    def get_baud(self):
        return self.baud

    def set_baud(self, baud):
        self.baud = baud
        self.set_tx_samps_per_symb(240e3/self.baud)
        self.set_rx_samps_per_symb((self.samp_rate/self.rx_decim*self.rx_interp)/self.baud)

    def get_variable_tag_object_0(self):
        return self.variable_tag_object_0

    def set_variable_tag_object_0(self, variable_tag_object_0):
        self.variable_tag_object_0 = variable_tag_object_0

    def get_tx_samps_per_symb(self):
        return self.tx_samps_per_symb

    def set_tx_samps_per_symb(self, tx_samps_per_symb):
        self.tx_samps_per_symb = tx_samps_per_symb

    def get_tx_offset(self):
        return self.tx_offset

    def set_tx_offset(self, tx_offset):
        self.tx_offset = tx_offset
        self.uhd_usrp_sink_0.set_center_freq(uhd.tune_request(self.libertas_freq, self.tx_offset), 0)

    def get_tx_interp(self):
        return self.tx_interp

    def set_tx_interp(self, tx_interp):
        self.tx_interp = tx_interp

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.uhd_usrp_sink_0.set_gain(self.tx_gain, 0)


    def get_tx_decim(self):
        return self.tx_decim

    def set_tx_decim(self, tx_decim):
        self.tx_decim = tx_decim

    def get_rx_samps_per_symb(self):
        return self.rx_samps_per_symb

    def set_rx_samps_per_symb(self, rx_samps_per_symb):
        self.rx_samps_per_symb = rx_samps_per_symb
        self.digital_clock_recovery_mm_xx_0.set_omega(self.rx_samps_per_symb*(1+0.0))

    def get_rx_offset(self):
        return self.rx_offset

    def set_rx_offset(self, rx_offset):
        self.rx_offset = rx_offset
        self.uhd_usrp_source_0.set_center_freq(uhd.tune_request(self.libertas_freq, self.rx_offset), 0)

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.uhd_usrp_source_0.set_gain(self.rx_gain, 0)


    def get_lpf_trans(self):
        return self.lpf_trans

    def set_lpf_trans(self, lpf_trans):
        self.lpf_trans = lpf_trans
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate / self.rx_decim *self.rx_interp, self.lpf_cutoff, self.lpf_trans, firdes.WIN_HAMMING, 6.76))

    def get_lpf_cutoff(self):
        return self.lpf_cutoff

    def set_lpf_cutoff(self, lpf_cutoff):
        self.lpf_cutoff = lpf_cutoff
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate / self.rx_decim *self.rx_interp, self.lpf_cutoff, self.lpf_trans, firdes.WIN_HAMMING, 6.76))

    def get_fsk_dev(self):
        return self.fsk_dev

    def set_fsk_dev(self, fsk_dev):
        self.fsk_dev = fsk_dev
        self.analog_quadrature_demod_cf_1.set_gain((self.samp_rate/self.rx_decim*self.rx_interp)/(2*math.pi*self.fsk_dev/8.0))

    def get_doppler_offset(self):
        return self.doppler_offset

    def set_doppler_offset(self, doppler_offset):
        self.doppler_offset = doppler_offset
        self.analog_sig_source_x_1.set_frequency(self.doppler_offset)
        self.analog_sig_source_x_0.set_frequency(-1 * self.doppler_offset)

    def get_bb_gain(self):
        return self.bb_gain

    def set_bb_gain(self, bb_gain):
        self.bb_gain = bb_gain
        self.blocks_multiply_const_vxx_0_0.set_k((self.bb_gain, ))

    def get_alpha(self):
        return self.alpha

    def set_alpha(self, alpha):
        self.alpha = alpha


def main(top_block_cls=gs_xcvr_uhd, options=None):

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
