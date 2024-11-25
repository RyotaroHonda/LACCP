library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_misc.all;
use ieee.numeric_std.all;

library mylib;
use mylib.defLaccp.all;

package defHeartBeatUnit is

  -- Heartbeat --
  constant kWidthHbCount    : integer:= 16;
  constant kWidthHbfNum     : integer:= 24;

  -- Local Address --
  constant kAddrFrameInfo   : std_logic_vector(kPosDestLocalAddr'length-1 downto 0):= X"0";

  -- DAQ --
  type HbfStateType is (kActiveFrame, kIdleFrame);
  function EncodeHbfState(state : HbfStateType) return std_logic_vector;
  function DecodeHbfState(vect  : std_logic_vector) return HbfStateType;

  constant kWidthFrameFlag      : integer := 2;

end package defHeartBeatUnit;
-- ----------------------------------------------------------------------------------
-- Package body
-- ----------------------------------------------------------------------------------
package body defHeartBeatUnit is

  function EncodeHbfState(state : HbfStateType) return std_logic_vector is
  begin
    if(state = kActiveFrame) then
      return X"0";
    elsif(state = kIdleFrame) then
      return X"1";
    else
      return X"F";
    end if;
  end EncodeHbfState;

  function DecodeHbfState(vect  : std_logic_vector) return HbfStateType is
  begin
    case vect is
      when X"0" => return kActiveFrame;
      when X"1" => return kIdleFrame;
      when others => return kIdleFrame;
    end case;
  end DecodeHbfState;


end package body defHeartBeatUnit;