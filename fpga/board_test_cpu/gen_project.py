#!/usr/bin/env python3
# Writes board_test_cpu.xml from ../board_test/board_test.xml: the same sources and
# settings, with board_test_top.vhd taken from board_test, the Sapphire IP and
# board_test_cpu_top.vhd added, and this project's top, interface and timing files.
from pathlib import Path

here = Path(__file__).resolve().parent
xml = (here.parent / "board_test" / "board_test.xml").read_text()


def replace(old, new):
    global xml
    if xml.count(old) != 1:
        raise SystemExit(f"board_test.xml: expected one {old!r}, found {xml.count(old)}")
    xml = xml.replace(old, new)


replace('name="board_test" description=""', 'name="board_test_cpu" description=""')
replace('<efx:top_module name="board_test_top" />', '<efx:top_module name="board_test_cpu_top" />')
replace('<efx:design_file name="board_test_top.vhd" version="default" library="default" />',
        '<efx:design_file name="../board_test/board_test_top.vhd" version="default" library="default" />\n'
        '        <efx:design_file name="ip/sapphire/sapphire.v" version="default" library="default" />\n'
        '        <efx:design_file name="board_test_cpu_top.vhd" version="default" library="default" />')
replace('<efx:default_peri_file name="board_test.peri.xml" />',
        '<efx:default_peri_file name="board_test_cpu.peri.xml" />')
replace('<efx:peri_file name="board_test.peri.xml" />', '<efx:peri_file name="board_test_cpu.peri.xml" />')
replace('<efx:sdc_file name="board_test.sdc" />', '<efx:sdc_file name="board_test_cpu.sdc" />')

(here / "board_test_cpu.xml").write_text(xml)
