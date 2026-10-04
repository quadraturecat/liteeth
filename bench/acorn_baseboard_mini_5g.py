#!/usr/bin/env python3

#
# This file is part of LiteEth.
#
# Copyright (c) 2026 Scott Torborg <scott@quadraturecat.com>
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""5GBASE-R / 5000BASE-X Etherbone on Acorn Baseboard Mini SFP0; see doc/a7_5g.md."""

import argparse

from migen import ClockDomain, Signal

from litex.gen import LiteXModule
from litex.build.xilinx.vivado import vivado_build_args, vivado_build_argdict
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.clock import S7PLL
from litex.soc.cores.led import LedChaser
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder, builder_args, builder_argdict

from litex_boards.platforms import sqrl_acorn

from liteeth.phy.serial.gtp_7series import QPLL, QPLLSettings
from liteeth.phy.serial.baser.wrappers.a7_gtp import A7_GTP_5G_BASER
from liteeth.phy.serial.basex.wrappers.a7_gtp import A7_5000BASEX

# Clocking -----------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, refclk_freq):
        self.rst        = Signal()
        self.cd_sys     = ClockDomain()
        self.cd_eth_ref = ClockDomain()

        # # #

        clk200 = platform.request("clk200")
        self.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

        # This board has no suitable dedicated reference. GTGREFCLK needs board-level validation.
        self.eth_pll = eth_pll = S7PLL(speedgrade=-2)
        self.comb += eth_pll.reset.eq(self.rst)
        eth_pll.register_clkin(pll.clkin, 200e6)
        eth_pll.create_clkout(self.cd_eth_ref, refclk_freq, margin=0)

# Bench SoC ----------------------------------------------------------------------------------------

class BenchSoC(SoCCore):
    def __init__(self, variant="cle-215+", sys_clk_freq=int(100e6), eth_ip="192.168.1.50",
        eth_phy="5GBASER", sfp_host_mode=None):
        platform = sqrl_acorn.Platform(variant=variant)
        platform.add_extension(sqrl_acorn._litex_acorn_baseboard_mini_io, prepend=True)
        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type             = "vexriscv",
            integrated_rom_size  = 0x20000,
            integrated_sram_size = 0x4000,
            uart_name            = "crossover",
            uart_fifo_depth      = 4096, # Let BIOS finish SFP setup before a host opens the console.
            ident                = "LiteEth %s bench on Acorn Baseboard Mini" % eth_phy,
        )

        refclk_freq, qpll_fbdiv = {
            "5GBASER"   : (171.875e6, 3),
            "5000BASEX" : (125e6,     5),
        }[eth_phy]
        self.crg = _CRG(platform, sys_clk_freq, refclk_freq)
        self.add_jtagbone()
        jtag_clk = self.jtagbone.phy.cd_jtag.clk
        platform.add_period_constraint(jtag_clk, 1e9/20e6)
        platform.add_false_path_constraints(self.crg.cd_sys.clk, jtag_clk)

        self.qpll = QPLL(gtgrefclk0=self.crg.cd_eth_ref.clk, qpllsettings0=QPLLSettings(
            refclksel  = 0b111,
            fbdiv      = qpll_fbdiv,
            fbdiv_45   = 5,
            refclk_div = 1,
        ))
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # PHY / Etherbone. Keep the standard reset, loopback, status and PRBS CSRs.
        if eth_phy == "5GBASER":
            self.ethphy = A7_GTP_5G_BASER(self.qpll.channels[0], platform.request("sfp", 0),
                sys_clk_freq = sys_clk_freq,
                refclk_freq  = refclk_freq,
                rx_polarity  = 1, # Inverted on Acorn.
                tx_polarity  = 0, # Two inversions: Acorn and baseboard.
            )
        elif eth_phy == "5000BASEX":
            self.ethphy = A7_5000BASEX(self.qpll.channels[0], platform.request("sfp", 0),
                sys_clk_freq = sys_clk_freq,
                with_csr     = True,
                pcs_kwargs   = {
                    "with_csr"     : True,
                    "sys_clk_freq" : sys_clk_freq,
                    # Fixed-rate link: the BCM84891 sends no Clause 37 config words in 5000BASE-X.
                    "with_autoneg" : False,
                    "sgmii"        : False,
                },
                rx_polarity  = 1, # Inverted on Acorn.
                tx_polarity  = 0, # Two inversions: Acorn and baseboard.
            )
        else:
            raise ValueError("unknown eth PHY: %s" % eth_phy)

        self.ethphy.add_timing_constraints(platform)
        self.add_etherbone(phy=self.ethphy, ip_address=eth_ip, data_width=32, buffer_depth=256,
            with_timing_constraints=False)
        platform.add_false_path_constraints(
            self.crg.cd_sys.clk, self.ethphy.txoutclk, self.ethphy.rxoutclk)

        if sfp_host_mode is None:
            sfp_host_mode = eth_phy

        self.sfp_i2c = I2CMaster(platform.request("sfp_i2c", 0))
        self.add_config("SFP_0_I2C",       "sfp_i2c")
        self.add_config("SFP_0_HOST_MODE", sfp_host_mode)
        self.leds = LedChaser(platform.request_all("user_led"), sys_clk_freq)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    # Build/load.
    parser.add_argument("--build",         action="store_true", help="Build bitstream.")
    parser.add_argument("--load",          action="store_true", help="Load bitstream.")

    # Board/Ethernet.
    parser.add_argument("--variant",       choices=["cle-101", "cle-215", "cle-215+"], default="cle-215+")
    parser.add_argument("--sys-clk-freq",  type=float, default=100e6)
    parser.add_argument("--eth-ip",        default="192.168.1.50")
    parser.add_argument("--eth-sfp", "--eth-phy", dest="eth_phy", choices=["5000BASEX", "5GBASER"], default="5GBASER",
        help="SFP0 PHY / line encoding.")
    parser.add_argument("--sfp-host-mode", choices=["5000BASEX", "5GBASER", "AUTO"],
        help="Module host mode set by the BIOS (default: same as --eth-sfp).")

    builder_args(parser)
    vivado_build_args(parser)
    args = parser.parse_args()

    soc = BenchSoC(
        variant       = args.variant,
        sys_clk_freq  = int(args.sys_clk_freq),
        eth_ip        = args.eth_ip,
        eth_phy       = args.eth_phy,
        sfp_host_mode = args.sfp_host_mode,
    )
    builder = Builder(soc, **builder_argdict(args))
    builder.build(run=args.build, **vivado_build_argdict(args))
    if args.load:
        soc.platform.create_programmer().load_bitstream(builder.get_bitstream_filename(mode="sram"))


if __name__ == "__main__":
    main()
