# MDIO access through the TSEMAC registers, for OpenOCD with the CPU halted.
proc mdio_wait {} { while {[read_memory 0xf8100114 32 1] & 2} {} }
proc mdio_rd {phy reg} {
    mdio_wait; mww 0xf8100108 [expr {($phy << 8) | $reg}]; mww 0xf8100104 1; sleep 2; mdio_wait
    return [expr {[read_memory 0xf8100110 32 1] & 0xffff}]
}
proc mdio_wr {phy reg val} {
    mdio_wait; mww 0xf8100108 [expr {($phy << 8) | $reg}]; mww 0xf810010c $val; mww 0xf8100104 2; sleep 2; mdio_wait
}
