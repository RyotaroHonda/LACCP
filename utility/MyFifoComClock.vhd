-- FIFO descriptions
-- Since the algorithm generating empty and full flags are too simple,
-- this FIFO can store data upto kFifoDepth-1.
-- The full flag goes high at the almost full flag position that for the FIFO generated by IP

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library mylib;

entity MyFifoComClock is
  generic(
    kFifoDepth   : integer;
    kWidthData    : integer;
    kProgFullTh   : integer;
    kInferBRAM    : boolean;
    kRegisterOut  : boolean
    );
  port (
    rst       : in std_logic;
    clk       : in std_logic;
    wrEn      : in std_logic;
    rdEn      : in std_logic;
    rdValid   : out std_logic;
    emptyOut  : out std_logic;
    fullOut   : out std_logic;
    pfullOut  : out std_logic;
    dIn       : in std_logic_vector(kWidthData-1 downto 0);
    dOut      : out std_logic_vector(kWidthData-1 downto 0)
    );
end MyFifoComClock;

architecture syn of MyFifoComClock is
  constant kWidthAddr         : integer:= integer(ceil(log2(real(kFifoDepth))));

  signal ptr_diff             : integer range 0 to kFifoDepth;
  signal write_ptr, read_ptr  : std_logic_vector(kWidthAddr-1 downto 0);
  signal write_en             : std_logic;
  signal read_en              : std_logic;
  signal pre_valid            : std_logic;
  signal empty, full, pfull  : std_logic;
  signal din_ram              : std_logic_vector(dOut'range);
  signal dout_ram             : std_logic_vector(dOut'range);

begin
  -- ================================ body ===============================

  emptyOut  <= empty;
  fullOut   <= full;
  pfullOut  <= pfull;

  ptr_diff  <= to_integer(unsigned(read_ptr)- unsigned(write_ptr));

  empty     <= '1' when (ptr_diff = 0) else '0';
  full      <= '1' when (ptr_diff = 1) else '0';
  pfull     <= '1' when (ptr_diff > 0 and ptr_diff <= kFifoDepth - kProgFullTh) else '0';

  write_en  <= '1' when (wrEn = '1' and full = '0') else '0';
  read_en   <= write_en or (rdEn and not empty);
  din_ram   <= dIn;

  process(clk, rst)
  begin
    if(clk'event and clk = '1') then
      if(rst = '1') then
        write_ptr <= (others => '0');
        read_ptr  <= (others => '0');
      else
        -- Pointer increment --
        if(wrEn = '1' and full = '0') then
          write_ptr   <= std_logic_vector(unsigned(write_ptr) +1);
        end if;

        if(rdEn = '1' and empty = '0') then
          read_ptr    <= std_logic_vector(unsigned(read_ptr) +1);
        end if;

      end if;
    end if;
  end process;

  gen_srrt : if kInferBRAM = false generate
  begin
    u_ram : entity mylib.MyDPRamSRRT
      generic map(
        kWidthAddr  => kWidthAddr,
        kWidthData  => kWidthData
        )
      port map(
        clk   => clk,
        we    => write_en,
        a     => write_ptr,
        dpra  => read_ptr,
        di    => din_ram,
        spo   => open,
        dpo   => dout_ram
        );

  end generate;

  gen_de : if kInferBRAM = true generate
  begin
    u_ram : entity mylib.MyDPRamDE
      generic map(
        kWidthAddr  => kWidthAddr,
        kWidthData  => kWidthData
        )
      port map(
        clk   => clk,
        ena   => '1',
        enb   => read_en,
        wea   => write_en,
        addra => write_ptr,
        addrb => read_ptr,
        dia   => din_ram,
        doa   => open,
        dob   => dout_ram
        );

  end generate;

  gen_noreg : if kRegisterOut = false generate
  begin

    dOut    <= dout_ram when pre_valid = '1' else (others=>'0');
    rdValid <= pre_valid;

    u_buf : process(clk, rst)
    begin
      if(rst = '1') then
        pre_valid <= '0';
      elsif(clk'event and clk = '1') then
        if(rdEn = '1' and empty = '0') then
          pre_valid <= '1';
        else
          pre_valid <= '0';
        end if;
      end if;
    end process;
  end generate;

  gen_reg : if kRegisterOut = true generate
  begin

    u_buf : process(clk, rst)
    begin
      if(rst = '1') then
        dOut      <= (others => '0');
        pre_valid <= '0';
        rdValid   <= '0';
      elsif(clk'event and clk = '1') then
        if(pre_valid = '1') then
          dOut      <= dout_ram;
        end if;
        rdValid     <= pre_valid;

        if(rdEn = '1' and empty = '0') then
          pre_valid <= '1';
        else
          pre_valid <= '0';
        end if;
      end if;
    end process;
  end generate;

end syn;