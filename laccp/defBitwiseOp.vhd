library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_misc.all;
use ieee.numeric_std.all;

library mylib;

package defBitwiseOp is

  function GetBitIndex(bit_vect : std_logic_vector) return integer;
  function IsolateRMHB(bit_vect : std_logic_vector) return std_logic_vector;


end package defBitwiseOp;
-- ----------------------------------------------------------------------------------
-- Package body
-- ----------------------------------------------------------------------------------
package body defBitwiseOp is

  -- GetBitIndex --------------------------------------------------------
  -- *** Input vector has only one high bit ***
  function GetBitIndex(bit_vect : std_logic_vector) return integer is
    variable result   : integer range 0 to bit_vect'length +1 :=0;
  begin
    for i in 0 to bit_vect'length -1 loop
      if(bit_vect(i) = '1') then
        result  := result + i;
      else
        result  := result + 0;
      end if;
    end loop;

    return result;
  end function;

  -- IsorateRMHB ---------------------------------------------------------
  -- Isorate rightmost high bit
  function IsolateRMHB(bit_vect : std_logic_vector) return std_logic_vector is
  begin
    return bit_vect and std_logic_vector(unsigned((not bit_vect)) + 1 );
  end IsolateRMHB;

end package body defBitwiseOp;
