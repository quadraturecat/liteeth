#!/usr/bin/env python3

#
# This file is part of LiteEth.
#
# Copyright (c) 2026 Scott Torborg <scott@quadraturecat.com>
# SPDX-License-Identifier: BSD-2-Clause

"""10GBASE-R bench on the Xilinx KC705."""

import argparse

from migen import *

from litex.gen import *

from litex.build.io import DifferentialInput, DifferentialOutput
from litex_boards.platforms import xilinx_kc705

from litex.soc.cores.clock import *
from litex.soc.integration.soc import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.interconnect.csr import CSRStorage, CSRStatus, CSRField
from migen.genlib.cdc import MultiReg

from litescope import LiteScopeAnalyzer

from liteiclink.serdes.gtx_7series import GTXQuadPLL

from liteeth.phy.k7_gtx_10g_baser import K7_GTX_10G_BASER, K7_GTX_5G_BASER

ETH_ETHERBONE_BUFFER_DEPTH = 160

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()

        # # #

        clk200    = platform.request("clk200")
        clk200_se = Signal()
        self.specials += DifferentialInput(clk200.p, clk200.n, clk200_se)

        self.pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200_se, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

        # No Ethernet reference PLL: the GTX reference is a real MGTREFCLK input from the Si5324.

# Bench SoC ----------------------------------------------------------------------------------------

class BenchSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(125e6), eth_ip="10.1.1.1", eth_phy="10gbaser",
                 refclk_freq=156.25e6, rx_polarity=0, tx_polarity=0):
        platform = xilinx_kc705.Platform()

        # SoC --------------------------------------------------------------------------------------
        # CPU included for clock init
        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type             = "vexriscv",
            integrated_rom_size  = 0x8000,
            integrated_sram_size = 0x2000,
            with_uart            = True,
            ident                = "LiteEth %s bench on KC705" % eth_phy,
            ident_version        = True,
        )

        self.add_jtagbone()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # GTX reference clock ----------------------------------------------------------------------
        self.comb += platform.request("si5324").rst_n.eq(1)
        refclk_pads = platform.request("si5324_clkout")
        refclk      = Signal()
        self.specials += Instance("IBUFDS_GTE2",
            i_CEB = 0,
            i_I   = refclk_pads.p,
            i_IB  = refclk_pads.n,
            o_O   = refclk,
        )
        platform.add_period_constraint(refclk, 1e9/refclk_freq)

        clk156    = platform.request("clk156")
        clk156_se = Signal()
        self.specials += DifferentialInput(clk156.p, clk156.n, clk156_se)
        si5324_clkin = platform.request("si5324_clkin")
        self.specials += DifferentialOutput(clk156_se, si5324_clkin.p, si5324_clkin.n)

        # Enable the SFP transmitter (active low disable).
        self.comb += platform.request("sfp_tx_disable_n").eq(1)

        # PHY --------------------------------------------------------------------------------------
        phy_cls = {"10gbaser": K7_GTX_10G_BASER, "5000baser": K7_GTX_5G_BASER}[eth_phy]

        # Shared QPLL: same 10.3125GHz VCO for both rates, different divider
        self.qpll = qpll = GTXQuadPLL(refclk, refclk_freq, phy_cls.linerate)

        self.ethphy = phy_cls(
            qpll         = qpll,
            data_pads    = platform.request("sfp", 0),
            sys_clk_freq = sys_clk_freq,
            platform     = platform,
            rx_polarity  = rx_polarity,
            tx_polarity  = tx_polarity,
        )
        platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.cd_eth_rx.clk,
            self.ethphy.cd_eth_tx.clk,
        )
        platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.txoutclk,
            self.ethphy.rxoutclk,
        )

        self.add_etherbone(
            phy          = self.ethphy,
            ip_address   = eth_ip,
            data_width   = 64,
            arp_entries  = 4,
            buffer_depth = ETH_ETHERBONE_BUFFER_DEPTH,
        )
        self.ethphy.add_timing_constraints(platform)

        self.comb += platform.request("i2c_mux_reset").eq(1)
        self.sfp_i2c = I2CMaster(platform.request("i2c", 0))
        self.add_si5324_init(refclk_freq)

        # Leds -------------------------------------------------------------------------------------
        self.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq,
        )

    # Si5324 init ----------------------------------------------------------------------------------

    # PCA9548 mux at 0x74; the Si5324 sits on channel 7, the SFP cage on channel 4.
    I2C_MUX      = 0x74
    I2C_MUX_CH   = 7
    SI5324_ADDR  = 0x68

    def add_si5324_init(self, refclk_freq):
        """Program the Si5324 for a 1:1 jitter-clean of CKIN1, from the BIOS at boot."""
        if abs(refclk_freq - 156.25e6) > 1:
            raise ValueError(
                "The Si5324 init table is a 1:1 pass-through of the 156.25MHz Si570, but "
                "refclk_freq is %.6fMHz. Recompute the dividers before changing it."
                % (refclk_freq/1e6))

        # Select the mux channel first
        self.sfp_i2c.add_init(addr=self.I2C_MUX, init=[
            (1 << self.I2C_MUX_CH, 1 << self.I2C_MUX_CH),
        ])

        self.sfp_i2c.add_init(addr=self.SI5324_ADDR, init=[
            (25, 0x00),                             # N1_HS  = 4    (bits[7:5] = N1_HS-4)
            (31, 0x00), (32, 0x00), (33, 0x07),     # NC1_LS = 8 -> N1 = 32, fosc = fin*32 = 5GHz
            (40, 0x80),                             # N2_HS  = 8    (bits[7:5]=4), N2_LS[19:16]=0
            (41, 0x0f), (42, 0x9f),                 # N2_LS  = 4000 -> N2 = 32000
            (43, 0x00), (44, 0x03), (45, 0xe7),     # N31    = 1000 -> f3 = fin/N31 = 156.25kHz
            (46, 0x00), (47, 0x03), (48, 0xe7),     # N32    = 1000 (CKIN2, unused, kept sane)
            # Clear the sticky alarm flags (129/130 are the flags; 131/132 are the masks), then
            # trigger an internal calibration so the new dividers take effect.
            (129, 0x00), (130, 0x00),
            (136, 0x40),                            # ICAL
        ])

    def add_eth_phy_probe(self, depth=4096, probe="rx"):
        """Add 5GBASE-R status CSRs and a receive-side LiteScope probe."""
        assert hasattr(self, "ethphy")
        if not getattr(self.ethphy, "baser", False):
            raise ValueError("The Ethernet PHY probe currently targets the BASE-R datapath.")

        control = self.ethphy._baser_control = CSRStorage(name="baser_control", fields=[
            CSRField("tx_prbs31", size=1, offset=0, description="Transmit BASE-R PRBS31 blocks."),
            CSRField("rx_prbs31", size=1, offset=1, description="Check BASE-R PRBS31 blocks."),
            CSRField("loopback", size=3, offset=2,
                description="GTP LOOPBACK control (010 selects near-end PMA loopback)."),
        ], description="5GBASE-R PCS diagnostics.")
        tx_prbs31 = Signal()
        rx_prbs31 = Signal()
        loopback  = Signal(3)
        self.specials += [
            MultiReg(control.fields.tx_prbs31, tx_prbs31, "eth_tx"),
            MultiReg(control.fields.rx_prbs31, rx_prbs31, "eth_rx"),
            MultiReg(control.fields.loopback,  loopback,  "eth_tx"),
        ]
        # TX driver sweep controls.
        drv = self.ethphy._baser_drive = CSRStorage(name="baser_drive", fields=[
            CSRField("diffctrl",   size=4, offset=0,  reset=0b1000, description="TXDIFFCTRL."),
            CSRField("precursor",  size=5, offset=4,  reset=0,      description="TXPRECURSOR."),
            CSRField("postcursor", size=5, offset=9,  reset=0,      description="TXPOSTCURSOR."),
        ], description="5GBASE-R TX driver settings.")
        self.specials += [
            MultiReg(drv.fields.diffctrl,   self.ethphy.tx_diffctrl,   "eth_tx"),
            MultiReg(drv.fields.precursor,  self.ethphy.tx_precursor,  "eth_tx"),
            MultiReg(drv.fields.postcursor, self.ethphy.tx_postcursor, "eth_tx"),
        ]

        self.comb += [
            self.ethphy.pcs.tx_prbs31_enable.eq(tx_prbs31),
            self.ethphy.pcs.rx_prbs31_enable.eq(rx_prbs31),
            self.ethphy.loopback.eq(loopback),
        ]

        status_signals = [Signal() for _ in range(8)]
        self.specials += [
            MultiReg(self.ethphy.qpll_lock,              status_signals[0]),
            MultiReg(self.ethphy.tx_reset_done,          status_signals[1]),
            MultiReg(self.ethphy.rx_reset_done,          status_signals[2]),
            MultiReg(self.ethphy.pcs.rx_block_lock,      status_signals[4]),
            MultiReg(self.ethphy.pcs.rx_high_ber,        status_signals[5]),
            MultiReg(self.ethphy.pcs.rx_status,          status_signals[6]),
            MultiReg(self.ethphy.pcs.rx_sequence_error,  status_signals[7]),
        ]
        self.ethphy._baser_status = CSRStatus(name="baser_status", fields=[
            CSRField("qpll_lock",         size=1, offset=0),
            CSRField("tx_reset_done",     size=1, offset=1),
            CSRField("rx_reset_done",     size=1, offset=2),
            CSRField("rx_block_lock",     size=1, offset=4),
            CSRField("rx_high_ber",       size=1, offset=5),
            CSRField("rx_status",         size=1, offset=6),
            CSRField("rx_sequence_error", size=1, offset=7),
            CSRField("tx_mmcm_locked",    size=1, offset=8),
            CSRField("rx_mmcm_locked",    size=1, offset=9),
            CSRField("tx_rst_held",       size=1, offset=10),
            CSRField("rx_rst_held",       size=1, offset=11),
        ], description="BASE-R GTX and PCS status.")
        self.comb += [
            self.ethphy._baser_status.fields.qpll_lock.eq(status_signals[0]),
            self.ethphy._baser_status.fields.tx_reset_done.eq(status_signals[1]),
            self.ethphy._baser_status.fields.rx_reset_done.eq(status_signals[2]),
            self.ethphy._baser_status.fields.rx_block_lock.eq(status_signals[4]),
            self.ethphy._baser_status.fields.rx_high_ber.eq(status_signals[5]),
            self.ethphy._baser_status.fields.rx_status.eq(status_signals[6]),
            self.ethphy._baser_status.fields.rx_sequence_error.eq(status_signals[7]),
        ]
        self.specials += [
            MultiReg(self.ethphy.tx_mmcm_locked, self.ethphy._baser_status.fields.tx_mmcm_locked),
            MultiReg(self.ethphy.rx_mmcm_locked, self.ethphy._baser_status.fields.rx_mmcm_locked),
            MultiReg(ResetSignal("eth_tx"), self.ethphy._baser_status.fields.tx_rst_held),
            MultiReg(ResetSignal("eth_rx"), self.ethphy._baser_status.fields.rx_rst_held),
        ]

        # eth_rx frequency counter.
        self.ethphy._rx_freq_start = CSRStorage(name="rx_freq_start",
            description="Write to start an eth_rx frequency measurement.")
        self.ethphy._rx_freq_busy  = CSRStatus(1,  name="rx_freq_busy")
        self.ethphy._rx_freq_count = CSRStatus(32, name="rx_freq_count")
        _win   = int(self.clk_freq // 10)          # 100 ms
        _cnt_s = Signal(max=_win + 1)
        _run_s = Signal()
        self.sync += [
            If(self.ethphy._rx_freq_start.re,
                _run_s.eq(1), _cnt_s.eq(0),
            ).Elif(_run_s,
                _cnt_s.eq(_cnt_s + 1),
                If(_cnt_s == (_win - 1), _run_s.eq(0)),
            ),
        ]
        _run_rx   = Signal()
        _run_rx_d = Signal()
        _cnt_rx   = Signal(32)
        self.specials += MultiReg(_run_s, _run_rx, "eth_rx")
        self.sync.eth_rx += [
            _run_rx_d.eq(_run_rx),
            If(_run_rx & ~_run_rx_d, _cnt_rx.eq(0)).Elif(_run_rx, _cnt_rx.eq(_cnt_rx + 1)),
        ]
        self.specials += [
            MultiReg(_run_s,  self.ethphy._rx_freq_busy.status),
            MultiReg(_cnt_rx, self.ethphy._rx_freq_count.status),
        ]

        self.ethphy._baser_rx_error_count = CSRStatus(7,
            name        = "baser_rx_error_count",
            description = "BASE-R PRBS31 error count from the most recent PCS interval.",
        )
        self.specials += MultiReg(
            self.ethphy.pcs.rx_error_count,
            self.ethphy._baser_rx_error_count.status,
        )

        if probe == "tx":
            analyzer_signals = [
                self.ethphy.tx_sequence,
                self.ethphy.tx_gearbox_ready,
                self.ethphy.tx_header,
                self.ethphy.pcs.serdes_tx_hdr,
                self.ethphy.pcs.serdes_tx_data,
                self.ethphy.pcs.tx_bad_block,
            ]
            self.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = depth,
                samplerate   = self.ethphy.tx_clk_freq,
                clock_domain = "eth_tx",
                register     = True,
                csr_csv      = "analyzer.csv")
            return

        analyzer_signals = [
            self.ethphy.rx_data_valid,
            self.ethphy.rx_header_valid,
            self.ethphy.rx_block_ce,
            self.ethphy.pcs.serdes_rx_slip,
            self.ethphy.pcs.serdes_rx_hdr,
            self.ethphy.rx_header,
            self.ethphy.rx_slip_pulse,
            self.ethphy.pcs.serdes_rx_data,
            self.ethphy.pcs.rx_block_lock,
            self.ethphy.pcs.rx_high_ber,
            self.ethphy.pcs.rx_bad_block,
            self.ethphy.pcs.rx_sequence_error,
            self.ethphy.pcs.rx_status,
            self.ethphy.source.valid,
            self.ethphy.source.last,
            self.ethphy.source.error,
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = depth,
            samplerate   = self.ethphy.rx_clk_freq,
            clock_domain = "eth_rx",
            register     = True,
            csr_csv      = "analyzer.csv"
        )

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteEth 10GBASE-R Bench on KC705.")
    parser.add_argument("--build",        action="store_true", help="Build bitstream.")
    parser.add_argument("--load",         action="store_true", help="Load bitstream.")
    parser.add_argument("--sys-clk-freq", default=125e6, type=float, help="System clock frequency.")
    parser.add_argument("--eth-ip",       default="10.1.1.1", help="Etherbone IP address.")
    parser.add_argument("--eth-phy",      default="10gbaser", choices=["10gbaser", "5000baser"],
        help="PHY rate; both use the same 156.25MHz reference and QPLL VCO.")
    parser.add_argument("--refclk-freq",  default=156.25e6, type=float,
        help="GTX reference clock frequency from the Si5324.")
    parser.add_argument("--rx-polarity",  default=0, type=int, choices=[0, 1],
        help="Invert the receive pair (SFP pairs are inverted prior to KC705 HW rev 1.1).")
    parser.add_argument("--tx-polarity",  default=0, type=int, choices=[0, 1],
        help="Invert the transmit pair.")
    parser.add_argument("--with-eth-phy-probe", action="store_true",
        help="Enable the BASE-R status CSRs and Ethernet PHY probe.")
    args = parser.parse_args()

    soc = BenchSoC(
        sys_clk_freq = int(args.sys_clk_freq),
        eth_ip       = args.eth_ip,
        eth_phy      = args.eth_phy,
        refclk_freq  = args.refclk_freq,
        rx_polarity  = args.rx_polarity,
        tx_polarity  = args.tx_polarity,
    )

    if args.with_eth_phy_probe:
        soc.add_eth_phy_probe()

    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
