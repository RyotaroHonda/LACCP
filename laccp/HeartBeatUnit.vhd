library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library mylib;
use mylib.defLaccp.all;
use mylib.defHeartBeatUnit.all;

-- Primary-HeartBeat Unit --
entity HeartBeatUnit is
  generic
    (
      enDebug         : boolean:= false
    );
  port
    (
      -- System --
      rst               : in std_logic; -- Asynchronous, Active high
      clk               : in std_logic;
      enStandAlone      : in std_logic;
      keepLocalHbfNum   : in std_logic;

      -- Sync I/F --
      syncPulseIn       : in std_logic;  -- Invalid in stand-alone mode
      hbcOffsetIn       : in std_logic_vector(kWidthHbCount-1 downto 0); -- Invalid in stand-alone mode
      validOffsetIn     : in std_logic; -- Invalid in stand-alone mode
      isSynchronized    : out std_logic; -- Invalid in stand-alone mode

      -- HeartBeat I/F --
      heartbeatOut      : out std_logic;
      heartbeatCount    : out std_logic_vector(kWidthHbCount-1 downto 0);
      hbfNumber         : out std_logic_vector(kWidthHbfNum-1 downto 0);
      hbfNumMismatch    : out std_logic; -- Invalid in stand-alone mode

      -- DAQ I/F --
      hbfCtrlGateIn     : in std_logic; -- Valid only in stand-alone mode
      forceOn           : in std_logic; -- Valid only in stand-alone mode
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
end HeartBeatUnit;

architecture RTL of HeartBeatUnit is
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
  signal hb_counter                         : std_logic_vector(heartbeatCount'range);
  signal local_hbf_number                   : std_logic_vector(hbfNumber'range);
  signal global_hbf_number                  : std_logic_vector(hbfNumber'range);

  signal frame_state                        : HbfStateType;
  signal global_frame_state                 : HbfStateType;

  signal frame_flags                        : std_logic_vector(frameFlags'range);
  signal global_frame_flags                 : std_logic_vector(frameFlags'range);

  signal grst_from_primary                  : std_logic;

  -- Synchronization --
  signal hbc_is_synced, ghbf_is_valid, hbf_is_synced  : std_logic;
  signal reg_hbfnum_mismatch                          : std_logic;
  signal comp_hbfnum                                  : std_logic;

  -- FIFO --
  signal re_rx_fifo, rd_valid_rx_fifo, empty_rx_fifo  : std_logic;
  signal dout_rx_fifo         : LaccpFrameBobyType;

  -- FSMs --
  type RxProcessType is (WaitRxIn, ParseFrame);
  signal state_rx       : RxProcessType;

  -- Debug --
  attribute mark_debug of hbc_is_synced         : signal is enDebug;
  attribute mark_debug of ghbf_is_valid         : signal is enDebug;
  attribute mark_debug of hbf_is_synced         : signal is enDebug;
  attribute mark_debug of comp_hbfnum           : signal is enDebug;
  attribute mark_debug of reg_hbfnum_mismatch   : signal is enDebug;
  attribute mark_debug of heartbeat_signal      : signal is enDebug;
  attribute mark_debug of backbeat_signal       : signal is enDebug;
  attribute mark_debug of hb_counter            : signal is enDebug;
  attribute mark_debug of local_hbf_number      : signal is enDebug;
  attribute mark_debug of global_hbf_number     : signal is enDebug;
  attribute mark_debug of frame_state           : signal is enDebug;
  attribute mark_debug of global_frame_state    : signal is enDebug;


begin
  -- =================================================================
  --                           Body
  -- =================================================================
  isReadyOut      <= '1';
  validBusOut     <= '0';
  dataBusOut      <= (others => '0');

  isSynchronized  <= hbf_is_synced;

  heartbeatOut    <= heartbeat_signal;
  heartbeatCount  <= hb_counter;
  hbfNumber       <= local_hbf_number;
  frameState      <= frame_state;
  hbfNumMismatch  <= reg_hbfnum_mismatch;
  frameFlags      <= frame_flags;

  -- HeartBeat -----------------------------------------------------------
  u_counter : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(sync_reset = '1') then
        hbc_is_synced   <= '0';
        hb_counter      <= (others => '0');
      else
        if(grst_from_primary = '1') then
          hbc_is_synced   <= '0';
        elsif(validOffsetIn = '1' and syncPulseIn = '1') then
          hbc_is_synced   <= '1';
          hb_counter      <= std_logic_vector(unsigned(hbcOffsetIn)+1);
        else
          hb_counter  <= std_logic_vector(unsigned(hb_counter) +1);
        end if;
      end if;

    end if;
  end process;

  u_heartbeat : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(hb_counter = kMaxCount and (hbc_is_synced = '1' or enStandAlone = '1')) then
        heartbeat_signal  <= '1';
      elsif(hb_counter = kHalfCount and (hbc_is_synced = '1' and enStandAlone = '1')) then
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
        hbf_is_synced         <= '0';
        reg_hbfnum_mismatch   <= '0';
        local_hbf_number      <= (others => '0');
      else
        if(grst_from_primary = '1') then
          hbf_is_synced         <= '0';
          reg_hbfnum_mismatch   <= '0';
        elsif(enStandAlone = '1') then
          -- Stand alone mode --
          if(heartbeat_signal = '1') then
            -- Heartbeat timing --
            local_hbf_number   <= std_logic_vector(unsigned(local_hbf_number) +1);
          end if;
        elsif(hbc_is_synced = '1' and ghbf_is_valid = '1') then
          if(hbf_is_synced = '0') then
            -- First HBF number update --
            hbf_is_synced     <= '1';
            local_hbf_number  <= global_hbf_number;
          else
            -- Normal process --
            if(comp_hbfnum = '1') then
              -- Global HBF number update timing --
              if(local_hbf_number /= global_hbf_number) then
                reg_hbfnum_mismatch   <= '1';
                if(keepLocalHbfNum = '0') then
                  local_hbf_number  <= global_hbf_number;
                end if;
              else
                reg_hbfnum_mismatch   <= '0';
              end if;
            elsif(heartbeat_signal = '1') then
              -- Heartbeat timing --
              local_hbf_number   <= std_logic_vector(unsigned(local_hbf_number) +1);
            else
              null;
            end if;
          end if;

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
        frame_flags   <= (others => "0");
      else
        if(enStandAlone = '1') then
          -- Stand-alone mode --
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
        else
          -- Synchronized mode --
          if(comp_hbfnum = '1') then
            frame_state <= global_frame_state;
            frame_flags <= global_frame_flags;
          end if;
        end if;
      end if;
    end if;
  end process;



  -- Rx Process ----------------------------------------------------------
  u_rx_fifo : entity mylib.MyFifoComClock
    generic map(
      kFifoDepth    => 16,
      kWidthData    => dataBusIn'length,
      kProgFullTh   => 14,
      kInferBRAM    => false,
      kRegisterOut  => false
      )
    port map(
      rst       => sync_reset,
      clk       => clk,
      wrEn      => validBusIn,
      rdEn      => re_rx_fifo,
      rdValid   => rd_valid_rx_fifo,
      emptyOut  => empty_rx_fifo,
      fullOut   => open,
      pfullOut  => open,
      dIn       => dataBusIn,
      dOut      => dout_rx_fifo
      );


  u_rxfsm : process(clk, sync_reset)
  begin
    if(clk'event and clk = '1') then
      if(sync_reset = '1') then
        re_rx_fifo          <= '0';
        grst_from_primary   <= '0';
        comp_hbfnum         <= '0';
        ghbf_is_valid       <= '0';
        global_hbf_number   <= (others => '0');
        global_frame_state  <= kIdleFrame;
        global_frame_flags  <= (others => '0');
        state_rx            <= WaitRxIn;
      else
      case state_rx is
        when WaitRxIn =>
          grst_from_primary   <= '0';
          comp_hbfnum         <= '0';
          if(empty_rx_fifo = '0') then
            re_rx_fifo  <= '1';
            state_rx    <= ParseFrame;
          end if;

        when ParseFrame =>
          re_rx_fifo  <= '0';

          if(rd_valid_rx_fifo = '1') then
            if(isWrite(dout_rx_fifo(kPosCmd'range))) then
              if(dout_rx_fifo(kPosDestLocalAddr'range) = kAddrPrimaryReset) then
                grst_from_primary   <= '1';
              elsif(dout_rx_fifo(kPosDestLocalAddr'range) = kAddrFrameInfo) then
                ghbf_is_valid       <= hbc_is_synced;
                global_hbf_number   <= dout_rx_fifo(kWidthHbfNum-1 downto 0);
                global_frame_flags  <= dout_rx_fifo(kWidthFrameFlag-1 + kWidthHbfNum downto kWidthHbfNum);
                global_frame_state  <= DecodeHbfState(dout_rx_fifo(kPosRegister'high downto kWidthFrameFlag+kWidthHbfNum));
                comp_hbfnum         <= '1';
              else
                -- This frame is not for me --
                null;
              end if;
            else
              -- This frame is not for me --
              null;
            end if;

            state_rx  <= WaitRxIn;
          end if;

        when others =>
          state_rx  <= WaitRxIn;


      end case;
      end if;
    end if;
  end process;


  -- Tx Process ----------------------------------------------------------
  -- Nothing to do

  -- Reset sequence --
  sync_reset  <= reset_shiftreg(kWidthResetSync-1);
  u_sync_reset : process(clk)
  begin
    if(clk'event and clk = '1') then
      reset_shiftreg  <= reset_shiftreg(kWidthResetSync-2 downto 0) & rst;
    end if;
  end process;

end RTL;