library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library mylib;
use mylib.defLaccp.all;
use mylib.defHeartBeatUnit.all;

-- Primary-HeartBeat Unit --
entity PrimaryHeartBeatUnit is
  generic
    (
      enDebug         : boolean:= false
    );
  port
    (
      -- System --
      rst               : in std_logic; -- Asynchronous, Active high
      primaryRst        : in std_logic; -- Synchronous reset, Active high
      clk               : in std_logic;

      -- Sync I/F --

      -- HeartBeat I/F --
      heartbeatOut      : out std_logic;
      heartbeatCount    : out std_logic_vector(kWidthHbCount-1 downto 0);
      hbfNumber         : out std_logic_vector(kWidthHbfNum-1 downto 0);

      -- DAQ I/F --
      hbfCtrlGateIn     : in std_logic;
      forceOn           : in std_logic;
      frameState        : out HbfStateType;

      hbfFlagsIn        : in std_logic_vector(kWidthFrameFlag-1 downto 0);
      frameFlags        : out std_logic_vector(kWidthFrameFlag-1 downto 0);

      -- LACCP Bus --
      dataBusIn         : in LaccpFrameBobyType;
      validBusIn        : in std_logic;
      dataBusOut        : out LaccpFrameBobyType;
      validBusOut       : out std_logic;
      isReadyOut        : out std_logic

    );
end PrimaryHeartBeatUnit;

architecture RTL of PrimaryHeartBeatUnit is
  attribute mark_debug  : boolean;

  -- System --
  signal sync_reset           : std_logic;
  constant kWidthResetSync    : integer:= 16;
  signal reset_shiftreg       : std_logic_vector(kWidthResetSync-1 downto 0);

  attribute async_reg : string;
  attribute async_reg of u_sync_reset : label is "true";

  -- Internal signal decralation --
  signal heartbeat_signal, backbeat_signal  : std_logic;
  constant kMaxCount          : std_logic_vector(heartbeatCount'range):= (others => '1');
  constant kHalfCount         : std_logic_vector(heartbeatCount'range):= (heartbeatCount'high => '1', others => '0');
  signal hb_counter           : std_logic_vector(heartbeatCount'range);
  signal local_hbf_number     : std_logic_vector(hbfNumber'range);


  signal frame_state                        : HbfStateType;
  signal frame_flags                        : std_logic_vector(hbfFlagsIn'range);

  -- FSMs --
  type TxProcessType is (TxIdle, SetHbfInfo, SetPrimaryReset, SendFrame);
  signal state_tx       : TxProcessType;

  -- Tx --
  signal reg_frame_tx   : LaccpFrameBobyType;

  -- Debug --
  attribute mark_debug of heartbeat_signal  : signal is enDebug;
  attribute mark_debug of backbeat_signal   : signal is enDebug;
  attribute mark_debug of hb_counter        : signal is enDebug;
  attribute mark_debug of local_hbf_number  : signal is enDebug;
  attribute mark_debug of frame_state       : signal is enDebug;
  attribute mark_debug of frame_flags       : signal is enDebug;


begin
  -- =================================================================
  --                           Body
  -- =================================================================
  isReadyOut      <= '1';

  heartbeatOut    <= heartbeat_signal;
  heartbeatCount  <= hb_counter;
  hbfNumber       <= local_hbf_number;
  frameState      <= frame_state;
  frameFlags      <= frame_flags;

  -- HeartBeat -----------------------------------------------------------
  u_counter : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(sync_reset = '1') then
        hb_counter  <= (others => '0');
      else
        if(primaryRst = '1' and frame_state = kIdleFrame) then
          hb_counter  <= (others => '0');
        else
          hb_counter  <= std_logic_vector(unsigned(hb_counter) +1);
        end if;

      end if;
    end if;
  end process;

  u_heartbeat : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(hb_counter = kMaxCount) then
        heartbeat_signal  <= '1';
      elsif(hb_counter = kHalfCount) then
        backbeat_signal   <= '1';
      else
        heartbeat_signal  <= '0';
        backbeat_signal   <= '0';
      end if;
    end if;
  end process;

  u_hbf : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(sync_reset = '1') then
        local_hbf_number   <= (others => '0');
      else
        if(primaryRst = '1' and frame_state = kIdleFrame) then
          local_hbf_number   <= (others => '0');
        elsif(heartbeat_signal = '1') then
          local_hbf_number   <= std_logic_vector(unsigned(local_hbf_number) +1);
        else
          null;
        end if;
      end if;
    end if;
  end process;

  -- DAQ state -----------------------------------------------------------
  u_framestate : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(sync_reset = '1') then
        frame_state   <= kIdleFrame;
        frame_flags   <= (others => '0');
      else
        if(forceOn = '1') then
          frame_state <= kActiveFrame;
        else
          if(backbeat_signal = '1' and hbfCtrlGateIn = '1') then
            frame_state   <= kActiveFrame;
          elsif(backbeat_signal = '1' and hbfCtrlGateIn = '0') then
            frame_state   <= kIdleFrame;
          end if;
        end if;

        if(backbeat_signal = '1') then
          frame_flags <= hbfFlagsIn;
        end if;
      end if;
    end if;
  end process;



  -- Rx Process ----------------------------------------------------------
  -- Nothing to do

  -- Tx Process ----------------------------------------------------------
  u_txfsm : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(sync_reset = '1') then
        validBusOut <= '0';
        state_tx    <= TxIdle;
      else
      case state_tx is
        when TxIdle =>
          validBusOut <= '0';
          if(backbeat_signal = '1') then
            state_tx          <= SetHbfInfo;
          elsif(primaryRst = '1' and frame_state = kIdleFrame) then
            state_tx          <= SetPrimaryReset;
          end if;

        when SetHbfInfo =>
          reg_frame_tx(kPosDestModAddr'range)   <= kAddrHBU;
          reg_frame_tx(kPosDestLocalAddr'range) <= kAddrFrameInfo;
          reg_frame_tx(kPosSrcModAddr'range)    <= kAddrHBU;
          reg_frame_tx(kPosSrcLocalAddr'range)  <= kAddrFrameInfo;
          reg_frame_tx(kPosCmd'range)           <= GenCmdVect(kCmdPassagePermission) or
                                                  GenCmdVect(kCmdDepature) or
                                                  GenCmdVect(kCmdWrite);
          reg_frame_tx(kPosRsv'range)           <= (others => '0');
          reg_frame_tx(kPosRegister'range)      <= EncodeHbfState(frame_state) & frame_flags & local_hbf_number;
          state_tx                              <= SendFrame;

        when SetPrimaryReset =>
          reg_frame_tx(kPosDestModAddr'range)   <= kBroadCast;
          reg_frame_tx(kPosDestLocalAddr'range) <= kAddrPrimaryReset;
          reg_frame_tx(kPosSrcModAddr'range)    <= kAddrHBU;
          reg_frame_tx(kPosSrcLocalAddr'range)  <= kAddrPrimaryReset;
          reg_frame_tx(kPosCmd'range)           <= GenCmdVect(kCmdPassagePermission) or
                                                  GenCmdVect(kCmdDepature) or
                                                  GenCmdVect(kCmdWrite);
          reg_frame_tx(kPosRsv'range)           <= (others => '0');
          reg_frame_tx(kPosRegister'range)      <= (others => '0');
          state_tx                              <= SendFrame;


        when SendFrame =>
          dataBusOut                          <= reg_frame_tx;
          validBusOut                         <= '1';
          state_tx                            <= TxIdle;

        when others =>
          state_tx  <= TxIdle;

      end case;
      end if;
    end if;
  end process;



  -- Reset sequence --
  sync_reset  <= reset_shiftreg(kWidthResetSync-1);
  u_sync_reset : process(clk)
  begin
    if(clk'event and clk = '1') then
      reset_shiftreg  <= reset_shiftreg(kWidthResetSync-2 downto 0) & rst;
    end if;
  end process;

end RTL;