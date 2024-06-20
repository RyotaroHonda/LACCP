--
-- Common Clock Dual-Port RAM with Asynchronous Read (Read Through)
--
-- This descriptions are directly mappable onto distributed RAM.
-- Descriptions are based on Xilinx UG901
--
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity MyDPRamARRT is
  generic(
    kWidthAddr  : integer;
    kWidthData  : integer
    );
  port (
    clk   : in std_logic;
    we    : in std_logic;
    addra : in std_logic_vector(kWidthAddr-1 downto 0);
    addrb : in std_logic_vector(kWidthAddr-1 downto 0);
    di    : in std_logic_vector(kWidthData-1 downto 0);
    doa   : out std_logic_vector(kWidthData-1 downto 0);
    dob   : out std_logic_vector(kWidthData-1 downto 0)
    );
end MyDPRamARRT;

architecture syn of MyDPRamARRT is
  constant kLength  : integer:= integer(2**real(kWidthAddr));

  type ram_type is array (kLength-1 downto 0) of std_logic_vector(di'range);
  signal RAM : ram_type;
  signal read_a     : std_logic_vector(addra'range):= (others=>'0');
  signal read_b  : std_logic_vector(addra'range):= (others=>'0');

  attribute ram_style : string;
  attribute ram_style of RAM : signal is "distributed";

begin

  process (clk)
  begin
    if clk'event and clk = '1' then
      if we = '1' then
        ram(to_integer(unsigned(addra))) <= di;
      end if;
    end if;
  end process;

  read_a <= addra;
  read_b <= addrb;

  doa <= ram(to_integer(unsigned(read_a)));
  dob <= ram(to_integer(unsigned(read_b)));

end syn;
