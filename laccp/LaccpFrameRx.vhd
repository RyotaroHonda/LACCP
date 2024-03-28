library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library mylib;
use mylib.defLaccp.all;
use mylib.defMikumari.all;
use mylib.defCDCM.all;

entity LaccpFrameRx is
  generic
    (
      enDebug         : boolean:= false
    );
  port
    (
      -- System --
      syncReset       : in std_logic; -- Active high
      clk             : in std_logic;

      -- LACCP --
      frameDataOut    : out LaccpFrameBobyType;
      frameValid      : out std_logic;
      frameInValid    : out std_logic;

      -- MIKUMARI-Link --
      dataRx          : in CbtUDataType;
      validRx         : in std_logic;
      frameLastRx     : in std_logic;
      checkSumErrRx   : in std_logic;
      frameBrokenRx   : in std_logic;
      recvTermndRx    : in std_logic

      --pulseRx         : in std_logic;
      --pulseTypeRx     : in MikumariPulseType

    );
end LaccpFrameRx;

architecture RTL of LaccpFrameRx is
  attribute mark_debug  : boolean;

  -- System --

  -- Internal signal decralation --
  -- Buffer --
  signal valid_rx             : std_logic;
  signal frame_last_rx        : std_logic;
  signal checksum_error       : std_logic;
  signal frame_broken         : std_logic;
  signal recv_terminated      : std_logic;
  signal data_rx              : CbtUDataType;

  signal unexpected_preamble  : std_logic;
  signal invalid_frame_length : std_logic;
  signal frame_valid          : std_logic;
  signal frame_invalid        : std_logic;
  signal frame_body           : LaccpFrameBobyType;

  type FrameParseProcessType is  (
    WaitPreamble, ReceiveFrameBody, DoneReceive, ReceiveError
  );
  signal state_rx             : FrameParseProcessType;

  -- Debug --
  attribute mark_debug of unexpected_preamble  : signal is enDebug;
  attribute mark_debug of invalid_frame_length : signal is enDebug;
  attribute mark_debug of frame_valid          : signal is enDebug;
  attribute mark_debug of frame_invalid        : signal is enDebug;
  attribute mark_debug of state_rx             : signal is enDebug;

begin
  -- =================================================================
  --                           Body
  -- =================================================================

  frameDataOut  <= frame_body;
  frameValid    <= frame_valid;
  frameInValid  <= frame_invalid;

  u_buf : process(clk)
  begin
    if(clk'event and clk = '1') then
      data_rx         <= dataRx;
      valid_rx        <= validRx;
      frame_last_rx   <= frameLastRx;
      checksum_error  <= checkSumErrRx;
      frame_broken    <= frameBrokenRx;
      recv_terminated <= recvTermndRx;
    end if;
  end process;

  -- Frame Rx --------------------------------------------------------
  u_frame_rx : process(clk, syncReset)
    variable index : integer range -1 to kLaccpFrameLength;
  begin
    if(syncReset = '1') then
      index                 := kLaccpFrameLength-1;
      unexpected_preamble   <= '0';
      invalid_frame_length  <= '0';
      frame_valid           <= '0';
      frame_invalid         <= '0';
      state_rx              <= WaitPreamble;

    elsif(clk'event and clk = '1') then
    case state_rx is
      when WaitPreamble =>
        frame_valid           <= '0';
        frame_invalid         <= '0';
        invalid_frame_length  <= '0';

        if(valid_rx = '1') then
          if(data_rx = kFramePreamble) then
            state_rx      <= ReceiveFrameBody;
            index         := kLaccpFrameLength-2;
            unexpected_preamble <= '0';
          else
            -- LACCP frame does not start from pre-amble
            unexpected_preamble <= '1';
          end if;
        end if;

      when ReceiveFrameBody =>
        if(valid_rx = '1') then
          frame_body((index+1)*data_rx'length-1 downto index*data_rx'length)   <= data_rx;

          if(index = 0) then
            if(frame_last_rx = '1' and checksum_error = '0') then
              state_rx  <= DoneReceive;
            else
              -- Frame Last does not come --
              invalid_frame_length  <= '1';
              state_rx  <= ReceiveError;
            end if;
          else
            if(frame_last_rx = '1') then
              -- Frame length is shorter than expected --
              invalid_frame_length  <= '1';
              state_rx  <= ReceiveError;
            end if;
          end if;

          index   := index -1;
        end if;

      when DoneReceive =>
        frame_valid     <= '1';
        frame_invalid   <= '0';
        state_rx        <= WaitPreamble;

      when ReceiveError =>
        frame_valid     <= '0';
        frame_invalid   <= '1';
        state_rx        <= WaitPreamble;

      when others =>
        state_rx      <= WaitPreamble;

    end case;

    end if;
  end process;

end RTL;