library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_misc.all;
use ieee.numeric_std.all;

library mylib;

package defLaccp is

  -- LACCP Frame structure ----------------------------------------------------------
  -- - 1-byte frame preamble
  -- - 1-byte destination address
  -- - 1-byte source address
  -- - 1-byte command bits
  -- - 1-byte reserve
  -- - 4-byte register value

  constant kLaccpFrameLength    : integer:= 9; -- Byte
  constant kFramePreamble       : std_logic_vector(7 downto 0):= X"FA";

  -- Command bits --
  constant kCmdPassagePermission  : integer:= 0; -- Allow to pass the packet through switch
  constant kCmdDepature           : integer:= 1; -- Request to tramismit packet from MIKUMARI
  constant kCmdWrite              : integer:= 2;
  constant kCmdRead               : integer:= 3;
  constant kCmdReply              : integer:= 4;
  constant kCmdReplyRequest       : integer:= 5;
  function GenCmdVect(index : integer) return std_logic_vector;
  function isWrite(cmd : std_logic_vector) return boolean;
  function isRead(cmd : std_logic_vector) return boolean;
  function isReply(cmd : std_logic_vector) return boolean;

  subtype LaccpFrameBobyType is std_logic_vector(8*(kLaccpFrameLength-1)-1 downto 0);
  constant kPosDestModAddr        : std_logic_vector(63 downto 60):= X"0";
  constant kPosDestLocalAddr      : std_logic_vector(59 downto 56):= X"0";
  constant kPosSrcModAddr         : std_logic_vector(55 downto 52):= X"0";
  constant kPosSrcLocalAddr       : std_logic_vector(51 downto 48):= X"0";
  constant kPosCmd                : std_logic_vector(47 downto 40):= X"00";
  constant kPosRsv                : std_logic_vector(39 downto 32):= X"00";
  constant kPosRegister           : std_logic_vector(31 downto 00):= X"00000000";

  -- Laccp Bus ----------------------------------------------------------------------
  constant kPortMikumari          : integer:= 0;

  constant kNumIntraPort          : integer:= 3;
  constant kPortRLIGP             : integer:= 1; -- Internal
  constant kPortRCAP              : integer:= 2; -- Internal
  constant kPortHBU               : integer:= 3; -- First external intra port
  constant kPortDummy             : integer:= -1;

  function GetIntraIndex(address : std_logic_vector) return integer;
  constant kBroadCast             : std_logic_vector(kPosDestModAddr'length -1 downto 0):= X"0";
  constant kAddrRLIGP             : std_logic_vector(kPosDestModAddr'length -1 downto 0):= X"1";
  constant kAddrRCAP              : std_logic_vector(kPosDestModAddr'length -1 downto 0):= X"2";
  constant kAddrHBU               : std_logic_vector(kPosDestModAddr'length -1 downto 0):= X"3";

  function CheckInterPortRange(index : integer) return boolean;

  -- Local address --
  constant kAddrPrimaryReset      : std_logic_vector(kPosDestLocalAddr'length -1 downto 0):= X"F";

  -- Bus structure ------------------------------------------------------------------
  constant kMaxNumInterconnect    : integer:= 32;
  constant kMaxBusPort            : integer:= kNumIntraPort + kMaxNumInterconnect + 1;
  type LaccpBusDataType is array(kMaxBusPort-1 downto 0) of LaccpFrameBobyType;
  subtype LaccpBusCtrlType is std_logic_vector(kMaxBusPort-1 downto 0);

  -- External port --
  constant kNumExtIntraPort     : integer:= kNumIntraPort -kPortHBU+1; -- RLIGP and RACP are internal
  type ExtIntraType is array(kNumExtIntraPort-1 downto 0)    of LaccpFrameBobyType;
  type ExtInterType is array(kMaxNumInterconnect-1 downto 0) of LaccpFrameBobyType;
  function GetExtIntraIndex(index : integer) return integer;

  -- MIKUMARI pulse -----------------------------------------------------------------
  constant kNumLaccpPulse     : integer:= 8;

  -- LACCP pulses --
  -- Down pulses --
  constant kDownPulseTrigger  : integer:= 0;
  constant kDownPulseCntRst   : integer:= 1;
  constant kDownPulseSysRst   : integer:= 2;
  constant kDownPulseRSV3     : integer:= 3;
  constant kDownPulseRSV4     : integer:= 4;
  constant kDownPulseRSV5     : integer:= 5;
  constant kDownPulseRSV6     : integer:= 6;
  constant kDownPulseRSV7     : integer:= 7;

  -- Up pulses --
  constant kUpPulseRSV0       : integer:= 0;
  constant kUpPulseRSV1       : integer:= 1;
  constant kUpPulseRSV2       : integer:= 2;
  constant kUpPulseRSV3       : integer:= 3;
  constant kUpPulseRSV4       : integer:= 4;
  constant kUpPulseRSV5       : integer:= 5;
  constant kUpPulseRSV6       : integer:= 6;
  constant kUpPulseRSV7       : integer:= 7;

  -- Pulse definition overtaken by RCAP --
  constant kPulseHeartbeat    : integer:= 0;
  constant kPulseRcapProbe    : integer:= 1;
  constant kPulseNC2          : integer:= 2;
  constant kPulseNC3          : integer:= 3;
  constant kPulseNC4          : integer:= 4;
  constant kPulseNC5          : integer:= 5;
  constant kPulseNC6          : integer:= 6;
  constant kPulseNC7          : integer:= 7;


  -- RCAP ---------------------------------------------------------------------------
  constant kWidthLaccpFineOffset  : integer:= 16;

  constant kFullCycle   : integer:= 8;
  constant kHalfCycle   : integer:= 4;
  constant kCalibDelayWidth  : integer:= 12;
  function GetFastClockPeriod(fast_clock_freq : real) return integer;
  function CalcFineLantency(idelay_tap : signed; serdes_latency : signed; fast_clock_period : integer)  return signed;

end package defLaccp;
-- ----------------------------------------------------------------------------------
-- Package body
-- ----------------------------------------------------------------------------------
package body defLaccp is

  function isWrite(cmd : std_logic_vector) return boolean is
  begin
    if(cmd(cmd'low + kCmdWrite) = '1') then
      return true;
    else
      return false;
    end if;
  end isWrite;

  function isRead(cmd : std_logic_vector) return boolean is
  begin
    if(cmd(cmd'low + kCmdRead) = '1') then
      return true;
    else
      return false;
    end if;
  end isRead;

  function isReply(cmd : std_logic_vector) return boolean is
  begin
    if(cmd(cmd'low + kCmdReply) = '1') then
      return true;
    else
      return false;
    end if;
  end isReply;

  function GenCmdVect(index : integer) return std_logic_vector is
  begin
    case index is
      when 0 => return "00000001";
      when 1 => return "00000010";
      when 2 => return "00000100";
      when 3 => return "00001000";
      when 4 => return "00010000";
      when 5 => return "00100000";
      when 6 => return "01000000";
      when 7 => return "10000000";
      when others => return "00000000";
    end case;
  end GenCmdVect;


  function GetIntraIndex(address : std_logic_vector) return integer is
  begin
    case address is
      when kAddrRLIGP => return kPortRLIGP;
      when kAddrRCAP  => return kPortRCAP;
      when kAddrHBU   => return kPortHBU;
      when others     => return kPortDummy;
    end case;
  end GetIntraIndex;

  function CheckInterPortRange(index : integer) return boolean is
  begin
    if(kNumIntraPort+1 <= index) then
      return true;
    else
      return false;
    end if;
  end CheckInterPortRange;

  function GetExtIntraIndex(index : integer) return integer is
  begin
    return (index - (kNumIntraPort-kNumExtIntraPort+1));
  end GetExtIntraIndex;

  -- RCAP --------------------------------------------------------------------------
  function GetFastClockPeriod(fast_clock_freq : real) return integer is
    -- fast_clock_freq: unit is MHz
    variable period   : integer;
  begin
    -- 1.0/(2.0*fast_clock_period)*1000*1000/(2000.0/2048.0);
    period  := integer(500.0*1024.0/fast_clock_freq);
    return period;
  end GetFastClockPeriod;

  function CalcFineLantency(idelay_tap : signed; serdes_latency : signed; fast_clock_period : integer)
  return signed is
    variable kTapDelay    : signed(9 downto 0):= to_signed(80, 10); -- 78ps
    variable kPeriod      : signed(11 downto 0):= to_signed(fast_clock_period, 12);
    variable idelay_val   : signed(15 downto 0);
    variable serdes_val   : signed(15 downto 0);
  begin
    idelay_val  := idelay_tap*kTapDelay;
    serdes_val  := serdes_latency*kPeriod;
    return idelay_val + serdes_val;
  end CalcFineLantency;

end package body defLaccp;
