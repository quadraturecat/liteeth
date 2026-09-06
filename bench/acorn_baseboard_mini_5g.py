#!/usr/bin/env python3

#
# This file is part of LiteEth.
#
# Copyright (c) 2026 Scott Torborg <scott@quadraturecat.com>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *

from litex.gen import *

from litex.build.io import DifferentialInput
from litex_boards.platforms import sqrl_acorn

from litex.soc.cores.clock import *
from litex.soc.integration.soc import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.interconnect.csr import CSRStorage, CSRStatus, CSRField
from migen.genlib.cdc import MultiReg

from liteeth.phy.a7_gtp import QPLLSettings, QPLL
from liteeth.phy.a7_1000basex import A7_5000BASER

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, eth_ref_freq=171.875e6):
        self.rst        = Signal()
        self.cd_sys     = ClockDomain()
        self.cd_sys_eth = ClockDomain()
        self.cd_eth_ref = ClockDomain()

        # # #

        # Clk/Rst.
        clk200    = platform.request("clk200")
        clk200_se = Signal()
        self.specials += DifferentialInput(clk200.p, clk200.n, clk200_se)

        # System PLL.
        self.pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200_se, 200e6)
        pll.create_clkout(self.cd_sys,     sys_clk_freq)
        pll.create_clkout(self.cd_sys_eth, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

        # Ethernet reference clock PLL. The Acorn has no MGTREFCLK source that divides into the
        # 2578.125MHz QPLL VCO, so the reference is synthesised here and fed to GTGREFCLK, despite
        # UG842 advising against this. Configuration attempts to minimize phase noise.
        self.eth_pll = eth_pll = S7PLL()
        self.comb += eth_pll.reset.eq(self.rst)
        eth_pll.register_clkin(clk200_se, 200e6)
        eth_pll.create_clkout(self.cd_eth_ref, eth_ref_freq, margin=0)
        platform.add_false_path_constraints(self.cd_sys.clk, eth_pll.clkin)

# Bench SoC ----------------------------------------------------------------------------------------

class BenchSoC(SoCCore):
    def __init__(self, variant="cle-215+", sys_clk_freq=int(166.67e6), sfp=0, eth_ip="10.3.1.1",
                 eth_ref_freq=171.875e6, sfp_mactype=4):
        platform = sqrl_acorn.Platform(variant=variant)
        platform.add_extension(sqrl_acorn._litex_acorn_baseboard_mini_io, prepend=True)

        # SoC --------------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type             = "vexriscv",
            integrated_rom_size  = 0x10000,
            integrated_sram_size = 0x2000,
            with_uart            = False,
            ident                = "LiteEth 5GBASE-R bench on Acorn Baseboard Mini",
            ident_version        = True,
        )
        self.add_uart(name="uart", uart_name="crossover", fifo_depth=4096)
        if sfp_mactype is not None:
            self.add_config("SFP_ROLLBALL_MACTYPE", sfp_mactype)
            self.add_config("SFP_ROLLBALL_I2C",     "sfp_i2c")

        self.add_jtagbone()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, eth_ref_freq)

        # Ethernet QPLL ----------------------------------------------------------------------------
        # refclksel 0b111 selects GTGREFCLK (the fabric reference). The VCO must be 2578.125MHz
        # (5.15625Gb/s = VCO*2/OUT_DIV, OUT_DIV=1); only a few references reach it with the
        # integer dividers (fbdiv_45 in {4,5} x fbdiv in 1..5).
        QPLL_DIVS = {
            103.125e6 : dict(fbdiv_45=5, fbdiv=5),   # x25, litex_m2sdr's choice
            171.875e6 : dict(fbdiv_45=5, fbdiv=3),   # x15
        }
        if eth_ref_freq not in QPLL_DIVS:
            raise ValueError("eth_ref_freq must be one of %s" % sorted(QPLL_DIVS))
        qpll_eth_settings = QPLLSettings(
            refclksel  = 0b111,
            fbdiv      = QPLL_DIVS[eth_ref_freq]["fbdiv"],
            fbdiv_45   = QPLL_DIVS[eth_ref_freq]["fbdiv_45"],
            refclk_div = 1,
        )
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")
        self.qpll = qpll = QPLL(
            gtgrefclk0    = self.crg.cd_eth_ref.clk,
            qpllsettings0 = qpll_eth_settings,
            gtgrefclk1    = Open(),
            qpllsettings1 = None,
        )

        self.ethphy = A7_5000BASER(
            qpll_channel = qpll.channels[0],
            data_pads    = self.platform.request("sfp", sfp),
            sys_clk_freq = sys_clk_freq,
            platform     = platform,
            rx_polarity  = 1, # Inverted on Acorn.
            tx_polarity  = 0, # Inverted on Acorn and on baseboard.
            refclk_freq  = eth_ref_freq,
            with_csr     = False, # Reset is driven by add_phy_watchdog().
        )
        self.add_phy_watchdog(self.ethphy)
        self.add_phy_status(self.ethphy)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys_eth.clk,
            self.ethphy.cd_eth_rx.clk,
            self.ethphy.cd_eth_tx.clk,
        )
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.txoutclk,
            self.ethphy.rxoutclk,
        )
        self.add_etherbone(
            phy          = self.ethphy,
            ip_address   = eth_ip,
            data_width   = 32,
            arp_entries  = 4,
            buffer_depth = 160,
        )

        # SFP management interface, so the module's host mode can be set from the host.
        self.sfp_i2c = I2CMaster(platform.request("sfp_i2c", 0))

        # Leds -------------------------------------------------------------------------------------
        self.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq,
        )

    # PHY watchdog -----------------------------------------------------------------------------

    def add_phy_watchdog(self, phy, timeout=0.5):
        # Reset the PHY whenever the link has been down for `timeout` seconds.
        # This may be necessary for late-configuration of a 5GBASE-R SFP module, and
        # could probably be avoided if the module has a fixed config.
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

    # PHY status -------------------------------------------------------------------------------

    def add_phy_status(self, phy):
        """Expose the BASE-R PCS receive state for bring-up against unknown modules."""
        self.phy_status = CSRStatus(fields=[
            CSRField("link_up",    size=1, description="PCS receive status (link up)."),
            CSRField("block_lock", size=1, description="64B/66B block lock."),
            CSRField("high_ber",   size=1, description="High bit-error-rate."),
        ])
        self.specials += [
            MultiReg(phy.link_up,           self.phy_status.fields.link_up),
            MultiReg(phy.pcs.rx_block_lock, self.phy_status.fields.block_lock),
            MultiReg(phy.pcs.rx_high_ber,   self.phy_status.fields.high_ber),
        ]
        # Event counters (rising edges seen from sys_clk; PCS pulses last >= 2 sys cycles).
        for name, sig, invert in [
            ("slips",     phy.pcs.serdes_rx_slip, False),
            ("lock_lost", phy.pcs.rx_block_lock,  True),
            ("bad_block", phy.pcs.rx_bad_block,   False),
            ("reset_req", phy.pcs.rx_reset_req,   False),
        ]:
            csr  = CSRStatus(32, name="phy_" + name)
            setattr(self, "phy_" + name, csr)
            sync = Signal(); prev = Signal()
            self.specials += MultiReg(~sig if invert else sig, sync)
            self.sync += [
                prev.eq(sync),
                If(sync & ~prev, csr.status.eq(csr.status + 1)),
            ]

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteEth 5GBASE-R Bench on Acorn Baseboard Mini.")
    parser.add_argument("--build",        action="store_true", help="Build bitstream.")
    parser.add_argument("--load",         action="store_true", help="Load bitstream.")
    parser.add_argument("--variant",      default="cle-215+", choices=["cle-101", "cle-215", "cle-215+"],
        help="Acorn board variant.")
    parser.add_argument("--programmer",   default="openfpgaloader", choices=["openocd", "openfpgaloader"],
        help="Programmer to use for loading.")
    parser.add_argument("--sys-clk-freq", default=166.67e6, type=float, help="System clock frequency.")
    parser.add_argument("--sfp",          default=0, type=int, choices=[0, 1], help="SFP port to use.")
    parser.add_argument("--eth-ip",       default="10.3.1.1", help="Etherbone IP address.")
    args = parser.parse_args()

    soc = BenchSoC(
        variant      = args.variant,
        sys_clk_freq = int(args.sys_clk_freq),
        sfp          = args.sfp,
        eth_ip       = args.eth_ip,
    )
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer(args.programmer)
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
