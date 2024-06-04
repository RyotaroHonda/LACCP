--
-- Common Clock Dual-Port RAM with Synchronous Read (Read Through)
--
-- This descriptions are directly mappable onto block RAM and also distributed RAM.
-- Descriptions are based on Xilinx UG627
--
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity MyDPRamSRRT is
  generic(
    kWidthAddr  : integer;
    kWidthData  : integer
    );
  port (
    clk   : in std_logic;
    we    : in std_logic;
    a     : in std_logic_vector(kWidthAddr-1 downto 0);
    dpra  : in std_logic_vector(kWidthAddr-1 downto 0);
    di    : in std_logic_vector(kWidthData-1 downto 0);
    spo   : out std_logic_vector(kWidthData-1 downto 0);
    dpo   : out std_logic_vector(kWidthData-1 downto 0)
    );
end MyDPRamSRRT;

architecture syn of MyDPRamSRRT is
  constant kLength  : integer:= integer(2**real(kWidthAddr));

  type ram_type is array (kLength-1 downto 0) of std_logic_vector(di'range);
  signal RAM : ram_type;
  signal read_a     : std_logic_vector(a'range):= (others=>'0');
  signal read_dpra  : std_logic_vector(a'range):= (others=>'0');

  attribute ram_style : string;
  attribute ram_style of RAM : signal is "distributed";

begin

  process (clk)
  begin
    if clk'event and clk = '1' then
      if we = '1' then
        ram(to_integer(unsigned(a))) <= di;
      end if;

      read_a    <= a;
      read_dpra <= dpra;
    end if;
  end process;

  spo <= ram(to_integer(unsigned(read_a)));
  dpo <= ram(to_integer(unsigned(read_dpra)));

end syn;
