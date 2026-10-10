-- The measurement scaler's channels: the measurements the controls and protections use, picked
-- from the ADC subsystem's (ad_subsystem's measurements, numbered there). Only these are scaled;
-- the scope takes every measurement as counts directly.
package scaler_channels_pkg is

    constant scaler_channels : natural := 6;

    -- the scaler's channel numbers
    constant scaled_vdc_out         : natural := 0;   -- the secondary DC link
    constant scaled_vfilt_out       : natural := 1;   -- the output filter's capacitor
    constant scaled_vdc_in          : natural := 2;   -- the primary DC link
    constant scaled_iac_out         : natural := 3;   -- the output current, filtered
    constant scaled_i_dab           : natural := 4;   -- the DHB's current, filtered
    constant scaled_iac_out_sampled : natural := 5;   -- the output current's period mean at the
                                                      -- modulator's pulses

    -- each one's measurement in ad_subsystem's numbering
    type measurement_index_array is array (0 to scaler_channels - 1) of natural;
    constant measurement_of : measurement_index_array := (
        scaled_vdc_out         => 0,    -- sec mux 0
        scaled_vfilt_out       => 2,    -- sec mux 2
        scaled_vdc_in          => 9,    -- pri mux 1
        scaled_iac_out         => 17,   -- sdm 1
        scaled_i_dab           => 18,   -- sdm 2
        scaled_iac_out_sampled => 23);  -- sdm 1's sampled period mean

end package scaler_channels_pkg;
