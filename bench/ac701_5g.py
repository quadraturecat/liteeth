#!/usr/bin/env python3

#
# This file is part of LiteEth.
#
# Copyright (c) 2026 Scott Torborg <scott@quadraturecat.com>
# SPDX-License-Identifier: BSD-2-Clause

"""5GBASE-R bench on the Xilinx AC701. Theoretically better clock tree than the Acorn."""

import argparse

from migen import *

from litex.gen import *

from litex.build.generic_platform import Pins, Subsignal, IOStandard
from litex.build.io import DifferentialInput, DifferentialOutput
from litex_boards.platforms import xilinx_ac701

from litex.soc.cores.clock import *
from litex.soc.integration.soc import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStorage
from migen.genlib.cdc import MultiReg
from liteeth.common import eth_phy_description
from liteeth.phy.a7_gtp import QPLLSettings, QPLL
from liteeth.phy.a7_1000basex import A7_5000BASER

# Buffered PHY ---------------------------------------------------------------------------------------

class _PacketFIFO(LiteXModule):
    """Store-and-forward FIFO: a packet is released once complete (or once the FIFO is full)."""
    def __init__(self, layout, depth):
        self.sink   = sink   = stream.Endpoint(layout)
        self.source = source = stream.Endpoint(layout)

        # # #

        self.fifo = fifo = stream.SyncFIFO(layout, depth, buffered=True)
        packets = Signal(max=depth + 1)
        inc     = Signal()
        dec     = Signal()
        release = Signal()
        self.comb += [
            sink.connect(fifo.sink),
            inc.eq(sink.valid & sink.ready & sink.last),
            dec.eq(source.valid & source.ready & source.last),
            release.eq((packets != 0) | ~fifo.sink.ready),
            fifo.source.connect(source, omit={"valid", "ready"}),
            source.valid.eq(fifo.source.valid & release),
            fifo.source.ready.eq(source.ready & release),
        ]
        self.sync += packets.eq(packets + inc - dec)


class BufferedPHY(LiteXModule):
    """A BASE-R PHY with store-and-forward TX and burst RX buffering on its MAC side.

    XGMII cannot pause inside a frame, so the PHY needs every word of a frame on consecutive
    block slots. With LiteX's 32-bit sys-datapath Etherbone at sys_clk 125MHz the MAC sources
    4Gb/s into a 5.156Gb/s line, and this xc7a200t-2 does not close the sys datapath at the
    161MHz+ that would remove the deficit; so buffer whole frames here instead. Presents the
    attributes add_etherbone() reads from a PHY.
    """
    def __init__(self, phy, tx_depth=256, rx_depth=128):
        self.phy    = phy
        self.sink   = stream.Endpoint(eth_phy_description(phy.dw))
        self.source = stream.Endpoint(eth_phy_description(phy.dw))

        # # #

        self.tx_fifo = tx_fifo = ClockDomainsRenamer("eth_tx")(
            _PacketFIFO(eth_phy_description(phy.dw), tx_depth))
        self.rx_fifo = rx_fifo = ClockDomainsRenamer("eth_rx")(
            stream.SyncFIFO(eth_phy_description(phy.dw), rx_depth, buffered=True))
        self.comb += [
            self.sink.connect(tx_fifo.sink),
            tx_fifo.source.connect(phy.sink),
            phy.source.connect(rx_fifo.sink),
            rx_fifo.source.connect(self.source),
        ]

    # Forwarded as properties (not attributes) so Migen does not collect the PHY's clock
    # domains a second time.
    dw                      = property(lambda self: self.phy.dw)
    cd_eth_tx               = property(lambda self: self.phy.cd_eth_tx)
    cd_eth_rx               = property(lambda self: self.phy.cd_eth_rx)
    tx_clk_freq             = property(lambda self: self.phy.tx_clk_freq)
    rx_clk_freq             = property(lambda self: self.phy.rx_clk_freq)
    integrated_ifg_inserter = property(lambda self: self.phy.integrated_ifg_inserter)
    link_up                 = property(lambda self: self.phy.link_up)
    reset                   = property(lambda self: self.phy.reset)
    txoutclk                = property(lambda self: self.phy.txoutclk)
    rxoutclk                = property(lambda self: self.phy.rxoutclk)

# 5GBASE-R is 5.15625Gb/s and the GTP PLL VCO must be half of it, 2578.125MHz, since the line
# rate is VCO*2/OUT_DIV with OUT_DIV=1. Only a few reference frequencies divide it with the
# integer GTP dividers (fbdiv_45 in {4,5}, fbdiv in 1..5).
REFCLK_SETTINGS = {
    103.125e6 : dict(fbdiv_45=5, fbdiv=5),   # x25, as used by litex_m2sdr for "5000baser"
    128.90625e6: dict(fbdiv_45=5, fbdiv=4),  # x20
    161.1328125e6: dict(fbdiv_45=4, fbdiv=4),# x16
    171.875e6 : dict(fbdiv_45=5, fbdiv=3),   # x15
}

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst        = Signal()
        self.cd_sys     = ClockDomain()
        self.cd_sys_eth = ClockDomain()

        # # #

        clk200    = platform.request("clk200")
        clk200_se = Signal()
        self.specials += DifferentialInput(clk200.p, clk200.n, clk200_se)

        self.pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200_se, 200e6)
        pll.create_clkout(self.cd_sys,     sys_clk_freq)
        pll.create_clkout(self.cd_sys_eth, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

        # No Ethernet reference PLL here: unlike the Acorn, the GTP reference is a real
        # MGTREFCLK input rather than something synthesised in the fabric.


# Si570 / Si5324 -------------------------------------------------------------------------------------

# The AC701 Si570 powers up at 156.25MHz, which is NOT one of the references a GTP PLL can turn
# into the 2578.125MHz VCO that 5GBASE-R needs. So unlike the KC705 (where the Si5324 was a 1:1
# jitter clean) this one also does the ratio, 156.25 -> 103.125MHz = 33/50:
#
#   f3   = fin/N31            = 156.25MHz / 1000    = 156.250kHz  (phase detector, 2kHz-2MHz)
#   fosc = fin*(N2_HS*N2_LS)/N31 = 156.25MHz*31680/1000 = 4.950000GHz  (must be 4.85-5.67GHz)
#   fout = fosc/(N1_HS*NC1_LS)   = 4.95GHz/48         = 103.125000MHz
#
# N31=1000 is deliberate: it puts f3 at the same 156.25kHz that is proven to lock on the KC705.
# A solver-minimal N31=80 also satisfies the arithmetic but sits at f3=1.953MHz, a hair under the
# 2MHz ceiling and with a correspondingly wide loop bandwidth - the opposite of what a jitter
# attenuator is for.
SI570_FREQ = 156.25e6

SI5324_INIT = [
    (25, 0x00),                             # N1_HS  = 4
    (31, 0x00), (32, 0x00), (33, 0x0b),     # NC1_LS = 12    -> N1 = 48
    (40, 0x00),                             # N2_HS  = 4,  N2_LS[19:16] = 0
    (41, 0x1e), (42, 0xef),                 # N2_LS  = 7920  -> N2 = 31680
    (43, 0x00), (44, 0x03), (45, 0xe7),     # N31    = 1000  -> f3 = 156.25kHz
    (46, 0x00), (47, 0x03), (48, 0xe7),     # N32    = 1000  (CKIN2, unused, kept sane)
    (129, 0x00), (130, 0x00),               # clear sticky alarms (131/132 are the masks)
    (136, 0x40),                            # ICAL
]

# Platform extension --------------------------------------------------------------------------------

# The AC701 platform definition carries clk156 (M21/M22, the Si570) and gtp_refclk 0
# (AA13/AB13 = MGTREFCLK0_213, fed from the SFP MGT clock mux), but nothing for the Si5324's
# clock *input*. D23/D24 is IO_L24P/N_T3_16 - a true differential pair in bank 16, the same 2.5V
# bank as sfp_mgt_clk_sel0/1 (which the platform declares LVCMOS25), hence LVDS_25 here.
_si5324_io = [
    # Reset is active-low and NOT in the litex-boards AC701 platform, so nothing drove it and the
    # part sat mute on the I2C bus: mux ch7 selects correctly (control reg reads back 0x80) but
    # 0x68 NAKs in both directions until this is released. B24 is IO_L23N_T3_16, bank 16, the
    # same 2.5V bank as sfp_mgt_clk_sel0/1.
    ("si5324", 0,
        Subsignal("rst_n", Pins("B24"), IOStandard("LVCMOS25")),
    ),
    ("si5324_clkin", 0,
        Subsignal("p", Pins("D23"), IOStandard("LVDS_25")),
        Subsignal("n", Pins("D24"), IOStandard("LVDS_25")),
    ),
]

# Bench SoC ----------------------------------------------------------------------------------------

class BenchSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(125e6), eth_ip="10.3.1.1",
                 refclk_freq=103.125e6, refclk_index=0, mgt_clk_sel=0b01,
                 rx_polarity=0, tx_polarity=0):
        platform = xilinx_ac701.Platform()
        platform.add_extension(_si5324_io)

        # SoC --------------------------------------------------------------------------------------
        # CPU included for clock init.
        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type             = "vexriscv",
            integrated_rom_size  = 0x10000,
            integrated_sram_size = 0x2000,
            with_uart            = False,
            ident                = "LiteEth 5GBASE-R bench on AC701",
            ident_version        = True,
        )
        self.add_uart(name="uart", uart_name="crossover", fifo_depth=4096)
        self.add_jtagbone()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # Si570 -> Si5324 clock input --------------------------------------------------------------
        # The reference chain is: Si570 (clk156, M21/M22) -> FPGA -> si5324_clkin (D23/D24) ->
        # Si5324 jitter attenuator -> its OUT0 -> SFP MGT clock mux -> gtp_refclk 0 (AA13/AB13)
        # -> GTP QPLL.
        self.comb += platform.request("si5324").rst_n.eq(1)

        clk156    = platform.request("clk156")
        clk156_se = Signal()
        self.specials += DifferentialInput(clk156.p, clk156.n, clk156_se)
        si5324_clkin = platform.request("si5324_clkin")
        self.specials += DifferentialOutput(clk156_se, si5324_clkin.p, si5324_clkin.n)
        platform.add_period_constraint(clk156_se, 1e9/SI570_FREQ)

        # GTP reference clock ----------------------------------------------------------------------
        if refclk_freq not in REFCLK_SETTINGS:
            raise ValueError("refclk_freq %s cannot reach a 2578.125MHz VCO with integer GTP "
                             "dividers; pick one of %s"
                             % (refclk_freq, sorted(REFCLK_SETTINGS)))
        divs = REFCLK_SETTINGS[refclk_freq]

        refclk_pads = platform.request("gtp_refclk", refclk_index)
        refclk      = Signal()
        self.specials += Instance("IBUFDS_GTE2",
            i_CEB = 0,
            i_I   = refclk_pads.p,
            i_IB  = refclk_pads.n,
            o_O   = refclk,
        )
        platform.add_period_constraint(refclk, 1e9/refclk_freq)

        # SFP cage MGT clock source select (sfp_mgt_clk_sel0/1).
        self.comb += [
            platform.request("sfp_mgt_clk_sel0").eq((mgt_clk_sel >> 0) & 1),
            platform.request("sfp_mgt_clk_sel1").eq((mgt_clk_sel >> 1) & 1),
        ]
        self.comb += platform.request("sfp_tx_disable_n").eq(1)

        # Ethernet QPLL ----------------------------------------------------------------------------
        qpll_eth_settings = QPLLSettings(
            refclksel  = 0b001,   # GTREFCLK0
            fbdiv      = divs["fbdiv"],
            fbdiv_45   = divs["fbdiv_45"],
            refclk_div = 1,
        )
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")
        self.qpll = qpll = QPLL(
            gtrefclk0     = refclk,
            qpllsettings0 = qpll_eth_settings,
            qpllsettings1 = None,
        )

        # PHY --------------------------------------------------------------------------------------
        self.ethphy = BufferedPHY(A7_5000BASER(
            qpll_channel = qpll.channels[0],
            data_pads    = platform.request("sfp", 0),
            sys_clk_freq = sys_clk_freq,
            platform     = platform,
            rx_polarity  = rx_polarity,
            tx_polarity  = tx_polarity,
            with_csr     = False, # Reset is driven by add_phy_watchdog().
        ))
        self.add_phy_watchdog(self.ethphy)
        # CDC between sys_eth and the PHY clocks.
        platform.add_false_path_constraints(
            self.crg.cd_sys_eth.clk,
            self.ethphy.cd_eth_rx.clk,
            self.ethphy.cd_eth_tx.clk,
        )
        platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.txoutclk,
            self.ethphy.rxoutclk,
        )

        # 32-bit Etherbone (sys datapath)
        # PHY's TX packet FIFO covers the 4Gb/s-vs-5.156Gb/s rate difference so
        # that it can send packets without them being mangled.
        self.add_etherbone(
            phy          = self.ethphy,
            ip_address   = eth_ip,
            data_width   = 32,
            arp_entries  = 4,
            buffer_depth = 160,
        )

        self.comb += platform.request("i2c_mux_reset").eq(1)
        self.sfp_i2c = I2CMaster(platform.request("i2c", 0))
        self.add_si5324_init()
        # RollBall SFP module host-mode init by the BIOS (libliteeth/sfp_rollball.c): the cage is
        # on PCA9548 channel 4 of the same bus.
        self.add_config("SFP_ROLLBALL_I2C",         "sfp_i2c")
        self.add_config("SFP_ROLLBALL_MUX_ADDR",    self.I2C_MUX)
        self.add_config("SFP_ROLLBALL_MUX_CHANNEL", 4)
        self.add_config("SFP_ROLLBALL_MACTYPE",     4)

        # Leds -------------------------------------------------------------------------------------
        self.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq,
        )

    # PHY watchdog -----------------------------------------------------------------------------

    def add_phy_watchdog(self, phy, timeout=0.5):
        """Reset the PHY whenever the link has been down for `timeout` seconds."""
        self.phy_reset = CSRStorage(description="PHY reset.")
        link_up = Signal()
        self.specials += MultiReg(phy.link_up, link_up)
        wd_max    = int(timeout*self.sys_clk_freq)
        wd        = Signal(max=wd_max + 1)
        pulse_max = int(10e-6*self.sys_clk_freq)
        pulse     = Signal(max=pulse_max + 1)
        self.sync += [
            If(pulse != 0, pulse.eq(pulse - 1)),
            If(link_up,
                wd.eq(0),
            ).Elif(wd == wd_max,
                wd.eq(0),
                pulse.eq(pulse_max),
            ).Else(
                wd.eq(wd + 1),
            ),
        ]
        self.comb += phy.reset.eq(self.phy_reset.storage | (pulse != 0))

    # Si5324 init ----------------------------------------------------------------------------------

    I2C_MUX     = 0x74      # PCA9548 on the bare bus
    I2C_MUX_CH  = 7         # Si5324 sits here; ch0 is the Si570, ch4 the SFP cage
    SI5324_ADDR = 0x68

    def add_si5324_init(self):
        """Program the Si5324 from the BIOS at boot (see SI5324_INIT for the divider maths).

        The Si570 is left at its 156.25MHz factory default and the Si5324 does the 33/50 ratio.
        """
        # The PCA9548 has no sub-address, but the BIOS init path always emits <reg><data>.
        # Writing the channel mask as both bytes puts the same value in the mux's single control
        # register twice, which is harmless and leaves ch7 selected.
        self.sfp_i2c.add_init(addr=self.I2C_MUX, init=[
            (1 << self.I2C_MUX_CH, 1 << self.I2C_MUX_CH),
        ])
        self.sfp_i2c.add_init(addr=self.SI5324_ADDR, init=SI5324_INIT)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteEth 5GBASE-R Bench on AC701.")
    parser.add_argument("--build",        action="store_true", help="Build bitstream.")
    parser.add_argument("--load",         action="store_true", help="Load bitstream.")
    parser.add_argument("--sys-clk-freq", default=125e6, type=float, help="System clock frequency.")
    parser.add_argument("--eth-ip",       default="10.3.1.1", help="Etherbone IP address.")
    parser.add_argument("--refclk-freq",  default=103.125e6, type=float,
        help="GTP reference clock frequency; must reach a 2578.125MHz VCO (%s)."
             % ", ".join("%.6fMHz" % (f/1e6) for f in sorted(REFCLK_SETTINGS)))
    parser.add_argument("--refclk-index", default=0, type=int, choices=[0, 1],
        help="Which gtp_refclk pin pair to use.")
    parser.add_argument("--mgt-clk-sel",  default=0b01, type=int,
        help="SFP MGT clock source select driven onto sfp_mgt_clk_sel0/1 (see UG952).")
    parser.add_argument("--rx-polarity",  default=0, type=int, choices=[0, 1],
        help="Invert the receive pair.")
    parser.add_argument("--tx-polarity",  default=0, type=int, choices=[0, 1],
        help="Invert the transmit pair.")
    args = parser.parse_args()

    soc = BenchSoC(
        sys_clk_freq = int(args.sys_clk_freq),
        eth_ip       = args.eth_ip,
        refclk_freq  = args.refclk_freq,
        refclk_index = args.refclk_index,
        mgt_clk_sel  = args.mgt_clk_sel,
        rx_polarity  = args.rx_polarity,
        tx_polarity  = args.tx_polarity,
    )

    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
