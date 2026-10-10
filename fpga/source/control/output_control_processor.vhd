library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;
    use work.output_observer_pkg.all;
    use work.output_voltage_control_pkg.all;
    use work.output_bridge_pkg.all;
    use work.output_control_program_pkg.all;
    use work.multi_port_ram_pkg.all;
    use work.dual_port_ram_pkg.all;
    use work.microinstruction_pkg.all;
    use work.microprogram_interface_pkg.all;
    use work.execution_unit_pkg.all;
    use work.scope_pkg.all;

-- The output filter's observer and the output voltage's control on one hVHDL_microprogram_processor
-- core (output_control_program_pkg's programs): a fixed point multiply-add, its data and program
-- in block RAM.
--
-- The observer steps at each arrival of the output current's period mean sampled at the
-- modulator's pulse (g_current_channel: twice a switching period), with the voltage reference
-- the bridge applies now (u) and the mean of the capacitor voltage's readings since the step
-- before (g_voltage_channel). It steps whether the bridge runs or not: with run false the bridge
-- off mode (output_observer_pkg). Disabled it holds no current, the PIs cleared, the voltage a
-- low pass of the readings.
--
-- The control steps once a switching period, after the observer's step that follows the
-- period's start, while it is enabled and closed_loop; its bridge reference (u_ref, with a
-- clock's ready) goes to the modulator, which applies it from the next period. Stopped, each
-- observer step resets it: its integrals cleared, du's filter at the observer's du.
--
-- The inputs and the bus's coefficient writes go to the data RAM through the core's external
-- write port, each held until the core's write stream shows it written; the results are taken
-- from that stream.
--
-- registers, as output_observer's and output_voltage_control's were (the coefficients write
-- only); coefficients with 24 fractional bits, the rest fixed point with 20
--   observer, from g_observer_address
--   0  RW bit0 enable
--   1..4   W phi11, phi12, phi21, phi22    the filter's step (zero order hold)
--   5..8   W gamma11, gamma12, gamma21, gamma22
--   9  W kp_i       10 W ki_i * dt      the current's PI
--   11 W kp_v       12 W ki_v * dt      the voltage's PI
--   13 R  the current estimate, A       14 R  the voltage estimate, V
--   15 R  du, V                         16 R  iload, A
--   17 R  steps since power up
--   control, from g_control_address
--   0  RW bit0 enable
--   1  W kp_v * b       2 W kp_v        3 W ki_v * period    the voltage's PI, A/V
--   4  W kp_i           5 W ki_i * period                    the current's PI, V/A
--   6  W R, ohm         7 W du's low pass: its share of a period's difference
--   8  W the current limit, A    9 W the inductor voltage's limit, V   10 W the bridge's, V
--   11 R  i_ref, A       12 R  vl_ref, V   13 R  u_ref, V   14 R  the predicted voltage, V
--   15 R  bit0 the current limited, bit1 the inductor voltage, bit2 the bridge's
--   16 R  steps since power up
--   17 W the reference's prefilter: each stage's share of a period's difference (1.0: none)
--   18 W the feedforward of the filtered reference's slope, A/V: C alpha / period
-- The defaults: the observer's for the fitted filter (0.83 mH, 18 uF, 0.51 ohm), current 1 kHz,
-- voltage 100 Hz; the control's current 3 kHz, voltage 1000 Hz with its zero at 0.1 of it, b 1,
-- du 20 Hz, 5 A, 60 V, 89 V, the reference's prefilter 1 kHz with its slope fed forward.
entity output_control_processor is
    generic(
        g_observer_address : natural := 1160;
        g_control_address  : natural := 1180;
        g_current_channel  : natural := 5;   -- the measurement scaler's iac_out sampled
        g_voltage_channel  : natural := 1    -- vfilt_out
    );
    port(
        clock          : in std_logic;
        bus_in         : in fpga_interconnect_record;
        bus_out        : out fpga_interconnect_record;
        -- from measurement_scaler
        scaled         : in fixed_array;
        scaled_ready   : in boolean;
        scaled_channel : in natural;
        -- the bridge running, the voltage reference it applies now
        run            : in boolean;
        u              : in fixed_number;
        -- the closed loop running, its reference, the period's start (the modulator's)
        closed_loop    : in boolean;
        v_ref          : in fixed_number;
        period_start   : in boolean;
        -- the bridge's reference, for the modulator
        u_ref          : out fixed_number;
        ready          : out boolean;
        -- scope probes: the voltages at 64 counts a volt (+-512 V), the currents at 256 an
        -- ampere (+-128 A): the observer's 0 i, 1 v, 2 du, 3 iload; the control's 0 i_ref, 1 u_ref
        observer_probes : out scope_sample_array(0 to 3);
        control_probes  : out scope_sample_array(0 to 1)
    );
end entity output_control_processor;

architecture rtl of output_control_processor is

    constant observer_defaults : observer_coefficients_record :=
        observer_coefficients(observer_design(0.83e-3, 18.0e-6, 0.51, output_step, 1000.0, 100.0));
    constant control_defaults : voltage_control_design_record :=
        voltage_control_design(output_period, 0.83e-3, 18.0e-6, 0.51, 3000.0, 1000.0, 5.0, 60.0, 0.95 * 94.0, 1.0, 20.0,
                               voltage_zero => 0.1, prefilter_hz => 1000.0);

    constant program : work.dual_port_ram_pkg.ram_array := control_program;
    constant data    : work.dual_port_ram_pkg.ram_array := control_data(observer_defaults, control_defaults);

    constant data_address_width : natural := address_width(data_words);
    constant program_address_width : natural := address_width(program_words);

    constant ref_subtype : subtype_ref_record :=
        create_ref_subtypes(readports => 3, datawidth => program_data_width, addresswidth => data_address_width);
    constant instr_ref_subtype : subtype_ref_record :=
        create_ref_subtypes(readports => 1, datawidth => program_config.instruction_width,
                            addresswidth => program_address_width);

    constant unit_in_ref : execution_unit_in_record := (
        instr_ram_read_out => instr_ref_subtype.ram_read_out
        ,data_read_out     => ref_subtype.ram_read_out
        ,instr_pipeline    => (0 to 12 => encode(mi(nop), program_config.instruction_width)));
    constant unit_out_ref : execution_unit_out_record := (
        data_read_in  => ref_subtype.ram_read_in
        ,ram_write_in => ref_subtype.ram_write_in);

    signal unit_in  : unit_in_ref'subtype  := unit_in_ref;
    signal unit_out : unit_out_ref'subtype := unit_out_ref;

    signal mproc_in    : microprogram_processor_in_record := (processor_requested => false, start_address => 0);
    signal mproc_out   : microprogram_processor_out_record;
    signal mc_output   : ref_subtype.ram_write_in'subtype;
    signal mc_write_in : ref_subtype.ram_write_in'subtype := ref_subtype.ram_write_in;

    signal observer_enable, control_enable : std_logic_vector(31 downto 0) := (others => '0');

    -- the bus's coefficient writes queued (firmware writes them back to back), then each held
    -- until it is in the data ram
    constant queue_subtype : dpram_ref_record :=
        create_ref_subtypes(datawidth => data_address_width + program_data_width, addresswidth => 5);
    constant queue_zeros : work.dual_port_ram_pkg.ram_array(0 to 31)(data_address_width + program_data_width - 1 downto 0)
        := (others => (others => '0'));
    signal queue_write_in, queue_read_in : queue_subtype.ram_in'subtype;
    signal queue_write_out, queue_read_out : queue_subtype.ram_out'subtype;
    signal queue_head, queue_tail : unsigned(4 downto 0) := (others => '0');
    signal queue_fetching : boolean := false;
    signal bus_write_pending : boolean := false;
    signal bus_write_address : natural range 0 to data_words-1 := 0;
    signal bus_write_data    : program_word := (others => '0');

    -- the voltage's readings since the last step
    signal reading_sum   : fixed_number := fixed_zero;
    signal reading_count : natural range 0 to max_readings := 0;

    -- a step's arrival, until the core is free
    signal step_pending : boolean := false;
    signal step_current, step_sum, step_u : fixed_number := fixed_zero;
    signal step_count   : natural range 0 to max_readings := 0;
    signal step_run     : boolean := false;

    -- a step's inputs, written in turn, then its program
    type input_array is array (natural range <>) of program_word;
    type address_array is array (natural range <>) of natural range 0 to data_words-1;
    constant observer_inputs : address_array(0 to 5) := (a_i_meas, a_v_sum, a_recip, a_has, a_u, a_on);
    constant control_inputs  : address_array(0 to 1) := (a_v_ref, a_u_app);
    signal inputs       : input_array(0 to 5) := (others => (others => '0'));
    signal input_count  : natural range 0 to 6 := 0;
    signal input_index  : natural range 0 to 6 := 0;
    signal writing_input : boolean := false;
    signal is_control   : boolean := false;

    type stage_type is (idle, write_inputs, start, running);
    signal stage     : stage_type := idle;
    signal start_at     : natural range 0 to program_words-1 := 0;
    signal pending_control : boolean := false;
    signal next_is_control : boolean := false;

    -- the results
    signal i_estimate, v_estimate, du, iload : fixed_number := fixed_zero;
    signal i_ref, vl_ref, u_ref_r, v_predicted : fixed_number := fixed_zero;
    signal limited : std_logic_vector(2 downto 0) := (others => '0');
    signal observer_steps, control_steps : unsigned(31 downto 0) := (others => '0');
    signal ready_r : boolean := false;

    type reciprocal_array is array (0 to max_readings) of program_word;
    function reciprocals return reciprocal_array is
        variable retval : reciprocal_array := (others => (others => '0'));
    begin
        for n in 1 to max_readings loop
            retval(n) := to_signed(integer(2.0**program_radix / real(n)), program_data_width);
        end loop;
        return retval;
    end function;
    constant reciprocal : reciprocal_array := reciprocals;
    constant program_one : program_word := shift_left(to_signed(1, program_data_width), program_radix);

    function to_probe_scaled(value : fixed_number; shift : natural) return scope_sample is
        variable v : fixed_number := shift_right(value, shift);
    begin
        if v > 32767 then
            return to_probe(32767);
        elsif v < -32768 then
            return to_probe(-32768);
        end if;
        return to_probe(v(15 downto 0));
    end function;

    function one_if(condition : boolean) return program_word is
    begin
        if condition then
            return program_one;
        end if;
        return (others => '0');
    end function;

begin

    u_ref <= u_ref_r;
    ready <= ready_r;
    observer_probes <= (to_probe_scaled(i_estimate, fixed_radix - 8), to_probe_scaled(v_estimate, fixed_radix - 6),
                        to_probe_scaled(du, fixed_radix - 6), to_probe_scaled(iload, fixed_radix - 8));
    control_probes  <= (to_probe_scaled(i_ref, fixed_radix - 8), to_probe_scaled(u_ref_r, fixed_radix - 6));

    u_microprogram_core : entity work.microprogram_core
    generic map(g_program => program, g_data => data)
    port map(
        clock        => clock
        ,mproc_in    => mproc_in
        ,mproc_out   => mproc_out
        ,mc_output   => mc_output
        ,mc_write_in => mc_write_in
        ,to_unit     => unit_in
        ,from_unit   => unit_out);

    u_queue : entity work.dual_port_ram
    generic map(g_dpram_subtype => queue_subtype, g_ram_init_values => queue_zeros, g_output_register => false)
    port map(clock => clock, ram_a_in => queue_write_in, ram_a_out => queue_write_out,
             ram_b_in => queue_read_in, ram_b_out => queue_read_out);

    u_fixed_mult_add : entity work.execution_unit(fixed_mult_add)
    generic map(g_radix => program_radix, g_pre_add_register => true, g_product_register => true,
                g_round_result => true)
    port map(clock, unit_in, unit_out);

    process(clock)
        variable result  : fixed_number;
        variable queued  : std_logic_vector(data_address_width + program_data_width - 1 downto 0);
        variable observer_active, control_active : boolean;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_data_to_address(bus_in, bus_out, g_observer_address + 0, observer_enable);
            connect_read_only_data_to_address(bus_in, bus_out, g_observer_address + 13, std_logic_vector(i_estimate));
            connect_read_only_data_to_address(bus_in, bus_out, g_observer_address + 14, std_logic_vector(v_estimate));
            connect_read_only_data_to_address(bus_in, bus_out, g_observer_address + 15, std_logic_vector(du));
            connect_read_only_data_to_address(bus_in, bus_out, g_observer_address + 16, std_logic_vector(iload));
            connect_read_only_data_to_address(bus_in, bus_out, g_observer_address + 17, std_logic_vector(observer_steps));
            connect_data_to_address(bus_in, bus_out, g_control_address + 0, control_enable);
            connect_read_only_data_to_address(bus_in, bus_out, g_control_address + 11, std_logic_vector(i_ref));
            connect_read_only_data_to_address(bus_in, bus_out, g_control_address + 12, std_logic_vector(vl_ref));
            connect_read_only_data_to_address(bus_in, bus_out, g_control_address + 13, std_logic_vector(u_ref_r));
            connect_read_only_data_to_address(bus_in, bus_out, g_control_address + 14, std_logic_vector(v_predicted));
            connect_read_only_data_to_address(bus_in, bus_out, g_control_address + 15, (31 downto 3 => '0') & limited);
            connect_read_only_data_to_address(bus_in, bus_out, g_control_address + 16, std_logic_vector(control_steps));

            init_mproc(mproc_in);
            ready_r <= false;
            observer_active := observer_enable(0) = '1';
            control_active  := control_enable(0) = '1' and closed_loop;

            -- the external writes, one at a time, each held until the core's write stream has it:
            -- the bus's coefficients first, a step's inputs between them (the address and the data
            -- in the stream: only these writes go to these addresses)
            if mc_write_in.write_requested = '1' and mc_output.write_requested = '1'
                and mc_output.address = mc_write_in.address and mc_output.data = mc_write_in.data
            then
                init_mp_write(mc_write_in);
                if writing_input then
                    writing_input <= false;
                    input_index   <= input_index + 1;
                else
                    bus_write_pending <= false;
                end if;
            elsif mc_write_in.write_requested = '0' then
                if bus_write_pending then
                    write_data_to_ram(mc_write_in, bus_write_address, std_logic_vector(bus_write_data));
                elsif stage = write_inputs and input_index < input_count then
                    writing_input <= true;
                    if is_control then
                        write_data_to_ram(mc_write_in, control_inputs(input_index), std_logic_vector(inputs(input_index)));
                    else
                        write_data_to_ram(mc_write_in, observer_inputs(input_index), std_logic_vector(inputs(input_index)));
                    end if;
                end if;
            end if;

            -- a coefficient written on the bus, to the queue; from it one at a time to be written
            init_ram(queue_write_in);
            init_ram(queue_read_in);
            for n in 1 to 12 loop
                if write_is_requested_to_address(bus_in, g_observer_address + n) then
                    write_data_to_ram(queue_write_in, to_integer(queue_tail), std_logic_vector(
                        to_unsigned(a_observer_k + n - 1, data_address_width)) &
                        std_logic_vector(coefficient_to_program(signed(get_slv_data(bus_in)))));
                    queue_tail <= queue_tail + 1;
                end if;
            end loop;
            for n in 1 to 10 loop
                if write_is_requested_to_address(bus_in, g_control_address + n) then
                    if n >= 8 then
                        -- the limits, values
                        write_data_to_ram(queue_write_in, to_integer(queue_tail), std_logic_vector(
                            to_unsigned(a_control_k + n - 1, data_address_width)) &
                            std_logic_vector(to_program(signed(get_slv_data(bus_in)))));
                    else
                        write_data_to_ram(queue_write_in, to_integer(queue_tail), std_logic_vector(
                            to_unsigned(a_control_k + n - 1, data_address_width)) &
                            std_logic_vector(coefficient_to_program(signed(get_slv_data(bus_in)))));
                    end if;
                    queue_tail <= queue_tail + 1;
                end if;
            end loop;
            for n in 17 to 18 loop
                if write_is_requested_to_address(bus_in, g_control_address + n) then
                    write_data_to_ram(queue_write_in, to_integer(queue_tail), std_logic_vector(
                        to_unsigned(a_c_prefilter + n - 17, data_address_width)) &
                        std_logic_vector(coefficient_to_program(signed(get_slv_data(bus_in)))));
                    queue_tail <= queue_tail + 1;
                end if;
            end loop;
            if not bus_write_pending and not queue_fetching and queue_head /= queue_tail then
                request_data_from_ram(queue_read_in, to_integer(queue_head));
                queue_head     <= queue_head + 1;
                queue_fetching <= true;
            end if;
            if ram_read_is_ready(queue_read_out) then
                queue_fetching    <= false;
                bus_write_pending <= true;
                -- (a function's result is not sliced: Efinity cannot)
                queued := get_ram_data(queue_read_out);
                bus_write_address <= to_integer(unsigned(queued(queued'left downto program_data_width)));
                bus_write_data    <= signed(queued(program_data_width - 1 downto 0));
            end if;

            -- the readings, summed for the step; the step at the sampled current's arrival, its
            -- inputs kept until the core is free
            if scaled_ready and scaled_channel = g_voltage_channel then
                if reading_count < max_readings then
                    reading_sum   <= reading_sum + scaled(g_voltage_channel);
                    reading_count <= reading_count + 1;
                end if;
            end if;
            if scaled_ready and scaled_channel = g_current_channel then
                step_pending  <= true;
                step_current  <= scaled(g_current_channel);
                step_sum      <= reading_sum;
                step_count    <= reading_count;
                step_u        <= u;
                step_run      <= run;
                reading_sum   <= fixed_zero;
                reading_count <= 0;
            end if;

            if control_active then
                if period_start then
                    pending_control <= true;
                end if;
            else
                pending_control <= false;
            end if;

            case stage is
                when idle =>
                    -- a step
                    if step_pending then
                        step_pending <= false;
                        inputs(0) <= to_program(step_current);
                        inputs(1) <= to_program(step_sum);
                        inputs(2) <= reciprocal(step_count);
                        inputs(3) <= one_if(step_count > 0);
                        inputs(4) <= to_program(step_u);
                        inputs(5) <= one_if(step_run);
                        input_count <= 6;
                        input_index <= 0;
                        is_control  <= false;
                        if observer_active then
                            start_at <= observer_start;
                        else
                            start_at <= disabled_start;
                        end if;
                        -- the control after the step: its own, or stopped its reset
                        next_is_control <= true;
                        stage <= write_inputs;
                    end if;

                when write_inputs =>
                    if input_index = input_count then
                        stage <= start;
                    end if;

                when start =>
                    if mc_write_in.write_requested = '0' then
                        calculate(mproc_in, start_at);
                        stage <= running;
                    end if;

                when running =>
                    if is_ready(mproc_out) then
                        if not is_control then
                            observer_steps <= observer_steps + 1;
                        elsif start_at = control_start then
                            control_steps <= control_steps + 1;
                            ready_r <= true;
                        end if;
                        stage <= idle;
                        if next_is_control then
                            next_is_control <= false;
                            is_control  <= true;
                            input_index <= 0;
                            if control_active and pending_control then
                                pending_control <= false;
                                inputs(0)   <= to_program(v_ref);
                                inputs(1)   <= to_program(u);
                                input_count <= 2;
                                start_at    <= control_start;
                                stage    <= write_inputs;
                            elsif not control_active then
                                input_count <= 0;
                                start_at    <= reset_start;
                                stage    <= write_inputs;
                            end if;
                        end if;
                    end if;
            end case;

            -- the results, from the core's writes
            if mc_output.write_requested = '1' then
                result := from_program(mc_output.data);
                case to_integer(mc_output.address) is
                    when a_i      => i_estimate <= result;
                    when a_v      => v_estimate <= result;
                    when a_du     => du         <= result;
                    when a_iload  => iload      <= result;
                    when a_i_ref  => i_ref      <= result;
                    when a_vl_ref => vl_ref     <= result;
                    when a_u_ref  => u_ref_r    <= result;
                    when a_v_pred => v_predicted <= result;
                    when a_excess_i  => limited(0) <= or mc_output.data;
                    when a_excess_vl => limited(1) <= or mc_output.data;
                    when a_excess_u  => limited(2) <= or mc_output.data;
                    when others =>
                end case;
            end if;
        end if;
    end process;

end architecture rtl;
