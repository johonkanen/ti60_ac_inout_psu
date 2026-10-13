library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_registers_pkg.all;

-- fpga_registers_pkg's address map: every block has registers, and no two blocks share an
-- address (an overlap makes both answer a read, their words or'd on the bus).
entity fpga_registers_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of fpga_registers_tb is
begin

    main : process
        constant blocks : register_block_array := all_register_blocks;
        variable a, b : register_block;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("no_two_blocks_overlap") then
                for i in blocks'range loop
                    a := blocks(i);
                    check(a.count > 0, register_block_name(i) & ": no registers");
                    for j in blocks'range loop
                        b := blocks(j);
                        if j > i then
                            check(a.base + a.count <= b.base or b.base + b.count <= a.base,
                                  register_block_name(i) & " (" & integer'image(a.base) & ".." &
                                  integer'image(a.base + a.count - 1) & ") overlaps " & register_block_name(j) &
                                  " (" & integer'image(b.base) & ".." & integer'image(b.base + b.count - 1) & ")");
                        end if;
                    end loop;
                end loop;
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

end vunit_simulation;
