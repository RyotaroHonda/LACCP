--
-- Common clock dual port RAM Write-First Mode
--
-- This descriptions are directly mappable onto block RAM.
-- Descriptions are based on Xilinx UG627
--
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity MyDPRamDE is
  generic(
    kWidthAddr  : integer;
    kWidthData  : integer
    );
  port (
    clk   : in std_logic;
    ena   : in std_logic;
    enb   : in std_logic;
    wea   : in std_logic;
    addra : in std_logic_vector(kWidthAddr-1 downto 0);
    addrb : in std_logic_vector(kWidthAddr-1 downto 0);
    dia   : in std_logic_vector(kWidthData-1 downto 0);
    doa   : out std_logic_vector(kWidthData-1 downto 0);
    dob   : out std_logic_vector(kWidthData-1 downto 0)
    );
end MyDPRamDE;

architecture syn of MyDPRamDE is
  constant kLength  : integer:= integer(2**real(kWidthAddr));

  type ram_type is array (kLength-1 downto 0) of std_logic_vector(dia'range);
  signal RAM : ram_type;
  signal read_addra: std_logic_vector(addra'range):= (others=>'0');
  signal read_addrb: std_logic_vector(addra'range):= (others=>'0');

begin

  process (clk)
  begin
    if clk'event and clk = '1' then
      if ena = '1' then
        if wea = '1' then
          ram(to_integer(unsigned(addra))) <= dia;
        end if;
        read_addra <= addra;
      end if;

      if enb = '1' then
        read_addrb <= addrb;
      end if;

    end if;
  end process;

  doa <= ram(to_integer(unsigned(read_addra)));
  dob <= ram(to_integer(unsigned(read_addrb)));

end syn;
