library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- Behavioural ADS7056: samples value at the CS falling edge and shifts it out in the next
-- frame (one frame pipeline delay): two zeros and the 14 bits MSB first, the first
-- g_output_delay after the CS edge and each next one g_output_delay after an SCLK rising edge,
-- like the ADC behaves on the control board. Counts the SCLK rising edges of each frame:
-- a frame of 64 or more after the power up dummy frame (24 or more) calibrates the offset.
entity ads7056_model is
    generic(
        g_output_delay : time := 10 ns
    );
    port(
        sclk          : in std_logic;
        cs_n          : in std_logic;
        sdo           : out std_logic;
        value         : in natural range 0 to 16383;
        is_calibrated : out boolean;
        frames        : out natural    -- conversion frames
    );
end entity ads7056_model;

architecture behavioural of ads7056_model is
begin

    process
        variable bits        : std_logic_vector(17 downto 0);
        variable clocks      : natural;
        variable had_dummy   : boolean := false;
        variable calibrated  : boolean := false;
        variable conversions : natural := 0;
        variable sampled     : natural range 0 to 16383 := 0;
    begin
        sdo <= 'Z';
        is_calibrated <= false;
        frames <= 0;
        loop
            wait until falling_edge(cs_n);
            bits := "00" & std_logic_vector(to_unsigned(sampled, 14)) & "00";
            sampled := value;
            sdo <= bits(17) after g_output_delay;
            clocks := 0;
            -- bit k goes out after the k:th SCLK rising edge
            loop
                wait until rising_edge(sclk) or rising_edge(cs_n);
                exit when cs_n = '1';
                clocks := clocks + 1;
                if clocks < 18 then
                    sdo <= bits(17 - clocks) after g_output_delay;
                else
                    sdo <= '0' after g_output_delay;
                end if;
            end loop;
            sdo <= 'Z' after g_output_delay;
            if not had_dummy then
                assert clocks >= 24 report "ads7056 model: power up dummy frame too short" severity error;
                had_dummy := true;
            elsif not calibrated then
                assert clocks >= 64 report "ads7056 model: calibration frame too short" severity error;
                calibrated := true;
            elsif clocks >= 64 then
                calibrated := true;          -- calibrated again
            elsif clocks >= 24 then
                calibrated := false;         -- the dummy frame of a new calibration
            else
                assert clocks = 18 report "ads7056 model: conversion frame of " & integer'image(clocks) & " SCLKs" severity error;
                conversions := conversions + 1;
            end if;
            is_calibrated <= calibrated;
            frames <= conversions;
        end loop;
    end process;

end architecture behavioural;
