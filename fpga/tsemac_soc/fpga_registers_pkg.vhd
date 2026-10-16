-- The register bus's address map: every block of registers the CPU reaches (word addresses on
-- the APB bridge, the CPU's 0xF8130000 + 4 * address), its first address and how many it takes.
-- The single place to look an address up or find a free one: tsemac_soc_top takes its blocks'
-- base addresses from here, and fpga_registers_tb checks that no two blocks overlap. Each
-- block's register map is in the comment of the file named.
--
--   address      registers  block                       its register map
--   600..622        23      the scope                   fpga_scope/source/scope_capture.vhd
--   700..716        17      board ADC, secondary side   source/adc/board_adc.vhd
--   720..736        17      board ADC, primary side     source/adc/board_adc.vhd
--   740..759        20      sigma-delta currents        source/adc/sdm_adc.vhd
--   760..765         6      gate driver supplies        tsemac_soc/gate_power.vhd
--   780..788         9      the DHB's bursts and runs   tsemac_soc/dhb_test.vhd
--   800..802         3      remote update               tsemac_soc/remote_update/remote_update.vhd
--   900..995        96      measurement scaler          source/measurements/measurement_scaler.vhd
--   1100..1111      12      DHB voltage control         source/control/dhb_voltage_controller.vhd
--   1120..1126       7      DHB protection              source/control/dhb_protection.vhd
--   1140..1153      14      output bridge               tsemac_soc/output_bridge_test.vhd
--   1160..1177      18      output observer             source/control/output_control_processor.vhd
--   1180..1198      19      output voltage control      source/control/output_control_processor.vhd
--   1200..1206       7      output waveform             source/inverter/waveform_player.vhd
--   1260..1279      20      input boost                 source/control/input_boost_control.vhd
--   1300..1313      14      LCD (button 0: STOP)        source/lcd/lcd_display.vhd
--   1320..1331      12      touch screen                source/lcd/touch_reader.vhd
--   1335..1339       5      version, HyperRAM status    tsemac_soc/tsemac_soc_top.vhd
--   1340..1343       4      supply sequencer            tsemac_soc/supply_sequencer.vhd
--   1360..1371      12      LCD buttons 1 and 2         source/lcd/lcd_display.vhd
--
-- The firmware has the same numbers (firmware/apps/scope/src/capture_hw.c, the scope's base in
-- the board's device tree).
package fpga_registers_pkg is

    type register_block is record
        base  : natural;
        count : natural;
    end record;
    type register_block_array is array (natural range <>) of register_block;

    constant scope_registers            : register_block := (600, 23);
    constant secondary_adc_registers    : register_block := (700, 17);
    constant primary_adc_registers      : register_block := (720, 17);
    constant sdm_adc_registers          : register_block := (740, 20);
    constant gate_power_registers       : register_block := (760, 6);
    constant dhb_test_registers         : register_block := (780, 9);
    constant remote_update_registers    : register_block := (800, 3);
    constant scaler_registers           : register_block := (900, 96);
    constant dhb_control_registers      : register_block := (1100, 12);
    constant dhb_protection_registers   : register_block := (1120, 7);
    constant output_bridge_registers    : register_block := (1140, 14);
    constant observer_registers         : register_block := (1160, 18);
    constant output_control_registers   : register_block := (1180, 19);
    constant waveform_registers         : register_block := (1200, 7);
    constant input_boost_registers      : register_block := (1260, 20);
    constant lcd_registers              : register_block := (1300, 14);
    constant touch_registers            : register_block := (1320, 12);
    constant version_registers          : register_block := (1335, 5);
    constant sequencer_registers        : register_block := (1340, 4);
    constant lcd_button_registers       : register_block := (1360, 12);

    -- the version block's: the image's commit, the HyperRAM controller's calibration status, its
    -- read timeouts, its debug word (low half, high half)
    constant version_register          : natural := version_registers.base;
    constant hyperram_status_register  : natural := version_registers.base + 1;
    constant hyperram_timeout_register : natural := version_registers.base + 2;
    constant hyperram_debug_register   : natural := version_registers.base + 3;

    constant all_register_blocks : register_block_array := (
        scope_registers, secondary_adc_registers, primary_adc_registers, sdm_adc_registers,
        gate_power_registers, dhb_test_registers, remote_update_registers, scaler_registers,
        dhb_control_registers, dhb_protection_registers, output_bridge_registers,
        observer_registers, output_control_registers, waveform_registers, input_boost_registers,
        lcd_registers, touch_registers, version_registers, sequencer_registers,
        lcd_button_registers);

    -- the blocks' names, in all_register_blocks' order, for messages
    function register_block_name(index : natural) return string;

end package fpga_registers_pkg;

package body fpga_registers_pkg is

    function register_block_name(index : natural) return string is
    begin
        case index is
            when 0  => return "scope";
            when 1  => return "secondary ADC";
            when 2  => return "primary ADC";
            when 3  => return "sigma-delta ADC";
            when 4  => return "gate power";
            when 5  => return "DHB test";
            when 6  => return "remote update";
            when 7  => return "measurement scaler";
            when 8  => return "DHB voltage control";
            when 9  => return "DHB protection";
            when 10 => return "output bridge";
            when 11 => return "output observer";
            when 12 => return "output control";
            when 13 => return "output waveform";
            when 14 => return "input boost";
            when 15 => return "LCD";
            when 16 => return "touch screen";
            when 17 => return "version";
            when 18 => return "supply sequencer";
            when 19 => return "LCD buttons";
            when others => return "block " & integer'image(index);
        end case;
    end function;

end package body fpga_registers_pkg;
