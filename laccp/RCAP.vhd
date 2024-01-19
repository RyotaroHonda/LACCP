library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library mylib;
use mylib.defLaccp.all;

-- Remote Clock Adjustment Protocol --
entity RCAP is
  generic
    (
      kWidthOffset    : integer:= 16;
      kPrimaryMode    : boolean:= false;
      enDebug         : boolean:= false
    );
  port
    (
      -- System --
      rst               : in std_logic; -- Asynchronous, Active high
      clk               : in std_logic;

      -- User Interface --
      isDone            : out std_logic;
      clockIsSyncedIn   : in std_logic;

      validOffset       : out std_logic;
      hbcOffset         : out std_logic_vector(kWidthOffset-1 downto 0);

      -- LACCP Bus --
      pulseTakeOver     : out std_logic;
      probePulseIn      : in std_logic;
      probePulseOut     : out std_logic;

      dataBusIn         : in LaccpFrameBobyType;
      validBusIn        : in std_logic;

      dataBusOut        : out LaccpFrameBobyType;
      validBusOut       : out std_logic;
      isReadyOut        : out std_logic

    );
end RCAP;

architecture RTL of RCAP is
  attribute mark_debug  : boolean;

  -- System --
  signal sync_reset           : std_logic;
  signal grst_from_primary    : std_logic;
  constant kWidthResetSync    : integer:= 16;
  signal reset_shiftreg       : std_logic_vector(kWidthResetSync-1 downto 0);

  -- Internal signal decralation --
  constant kWidthResend : integer:= 28;

  signal pulse_takeover : std_logic;
  signal probe_pulse, returned_pulse  : std_logic;

  constant kMaxOffset         : std_logic_vector(hbcOffset'range):= (others => '1');
  signal reg_round_trip_time  : std_logic_vector(hbcOffset'range);
  signal reg_hbc_offset       : std_logic_vector(hbcOffset'range);
  signal offset_counter       : std_logic_vector(hbcOffset'range):= (others => '0');
  signal reg_valid_offset     : std_logic;

  signal is_done              : std_logic;
  signal req_send_offset      : std_logic;
  type RcapStateType is (Init, AdjustClock, Finalize, Done);
  signal state_rcap           : RcapStateType;

  signal offset_is_received   : std_logic;
  signal reg_offset_rx        : std_logic_vector(kPosRegister'range);

  -- Local Address --
  constant kAddrOffset   : std_logic_vector(kPosDestLocalAddr'length-1 downto 0):= X"0";

  -- FIFO --
  signal re_rx_fifo, rd_valid_rx_fifo, empty_rx_fifo  : std_logic;
  signal dout_rx_fifo         : LaccpFrameBobyType;

  -- FSMs --
  type RxProcessType is (WaitRxIn, ParseFrame, CheckReplyReq, ReplyProcess, WaitInternalAck, ParseReply);
  type SwitchProcessType is (WaitReq, SendFrame);
  type TxProcessType is (WaitSendReq, SendFrame, WaitInternalAck, WaitReply, Done);

  signal state_rx       : RxProcessType;
  signal state_switch   : SwitchProcessType;
  signal state_tx       : TxProcessType;

  -- Tx --
  constant kNumTxPath   : integer:= 2;
  constant kWrite       : integer:= 0;
  constant kReply       : integer:= 1;
  type TxBufferType is array (kNumTxPath-1 downto 0) of LaccpFrameBobyType;
  signal reg_frame_tx   : TxBufferType;
  signal reg_tx_req     : std_logic_vector(kNumTxPath-1 downto 0);
  signal reg_tx_ack     : std_logic_vector(kNumTxPath-1 downto 0);

  -- Reply --
  constant kNumReplyPath  : integer:= 1;
  constant kOffset        : integer:= 0;
  signal reg_frame_rx     : LaccpFrameBobyType;
  signal got_reply        : std_logic_vector(kNumReplyPath-1 downto 0);

  -- Debug --
  attribute mark_debug  of offset_is_received   : signal is enDebug;
  attribute mark_debug  of reg_round_trip_time  : signal is enDebug;
  attribute mark_debug  of reg_hbc_offset       : signal is enDebug;
  attribute mark_debug  of reg_valid_offset     : signal is enDebug;
  attribute mark_debug  of is_done              : signal is enDebug;


begin
  -- =================================================================
  --                           Body
  -- =================================================================

  isDone          <= is_done;

  pulseTakeOver   <= pulse_takeover;
  isReadyOut      <= '1';

  -- Primary mode ----------------------------------------------------
  gen_prim : if kPrimaryMode = true generate
  begin
    probePulseOut       <= probePulseIn;

    reg_round_trip_time <= (others => '0');
    hbcOffset           <= reg_offset_rx(kWidthOffset-1 downto 0);
    validOffset         <= offset_is_received;

    u_rcapstate : process(clk, sync_reset)
    begin
      if(sync_reset = '1') then
        is_done           <= '0';
        pulse_takeover    <= '1';
      elsif(clk'event and clk = '1') then
        if(grst_from_primary = '1') then
          is_done           <= '0';
          pulse_takeover    <= '1';
        elsif(offset_is_received = '1') then
          is_done         <= '1';
          pulse_takeover  <= '0';
        else
          null;
        end if;
      end if;
    end process;

  end generate;

  -- Secondary mode --------------------------------------------------
  gen_secnd : if kPrimaryMode = false generate
  begin

    returned_pulse    <= probePulseIn;
    probePulseOut     <= probe_pulse;

    hbcOffset         <= reg_hbc_offset;
    validOffset       <= reg_valid_offset;

    u_offset : process(clk, sync_reset)
    begin
      if(sync_reset = '1') then
        reg_hbc_offset      <= (others => '0');
        reg_round_trip_time <= (others => '0');
        reg_valid_offset    <= '0';
      elsif(clk'event and clk = '1') then
        if(grst_from_primary = '1') then
          reg_valid_offset     <= '0';
        elsif(is_done = '0') then
          offset_counter  <= std_logic_vector(unsigned(offset_counter) +1);

          if(offset_counter = kMaxOffset) then
            probe_pulse   <= '1';
          else
            probe_pulse   <= '0';
          end if;

          if(returned_pulse = '1' and reg_valid_offset = '0') then
            reg_round_trip_time  <= offset_counter;
            reg_hbc_offset       <= '0' & offset_counter(kWidthOffset-1 downto 1);
            reg_valid_offset     <= '1';
          end if;
        end if;
      end if;
    end process;

    u_rcapstate : process(clk, sync_reset)
    begin
      if(sync_reset = '1') then
        pulse_takeover  <= '1';
        is_done         <= '0';
        req_send_offset <= '0';
        state_rcap      <= Init;
      elsif(clk'event and clk = '1') then
        case state_rcap is
          when Init =>
            pulse_takeover  <= '1';
            is_done         <= '0';
            req_send_offset <= '0';
            state_rcap      <= AdjustClock;

          when AdjustClock =>
            pulse_takeover  <= '1';

            if(grst_from_primary = '1') then
              state_rcap  <= Init;
            elsif(clockIsSyncedIn = '1') then
              req_send_offset <= '1';
              state_rcap      <= Finalize;
            end if;

          when Finalize =>
            req_send_offset <= '0';
            if(grst_from_primary = '1') then
              state_rcap      <= Init;
            elsif(got_reply(kOffset) = '1') then
              state_rcap      <= Done;
            end if;

          when Done =>
            is_done         <= '1';
            pulse_takeover  <= '0';
            if(grst_from_primary = '1') then
              state_rcap      <= Init;
            end if;

          when others => null;

        end case;
      end if;
    end process;

  end generate;

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
    if(sync_reset = '1') then
      offset_is_received  <= '0';
      re_rx_fifo          <= '0';
      reg_tx_req(kReply)  <= '0';
      got_reply           <= (others => '0');
      state_rx            <= WaitRxIn;
    elsif(clk'event and clk = '1') then
    case state_rx is
      when WaitRxIn =>
        reg_tx_req(kReply)  <= '0';
        if(empty_rx_fifo = '0') then
          re_rx_fifo  <= '1';
          state_rx          <= ParseFrame;
        end if;

      when ParseFrame =>
        re_rx_fifo  <= '0';

        if(rd_valid_rx_fifo = '1') then
          if(isWrite(dout_rx_fifo(kPosCmd'range))) then
            if(dout_rx_fifo(kPosDestLocalAddr'range) = kAddrOffset) then
              reg_frame_rx      <= dout_rx_fifo;
              reg_offset_rx     <= dout_rx_fifo(kPosRegister'range);
              offset_is_received  <= '1';
              state_rx          <= CheckReplyReq;
            else
              -- This frame is not for me --
              state_rx  <= WaitRxIn;
            end if;
          elsif(isReply(dout_rx_fifo(kPosCmd'range))) then
            reg_frame_rx  <= dout_rx_fifo;
            state_rx      <= ParseReply;
          else
            -- This frame is not for me --
            state_rx  <= WaitRxIn;
          end if;
        end if;

      -- From Write Command --
      when CheckReplyReq =>
        if('1' = reg_frame_rx(kPosCmd'low + kCmdReplyRequest)) then
          state_rx  <= ReplyProcess;
        else
          state_rx  <= WaitRxIn;
        end if;

      when ReplyProcess =>
        if(reg_frame_rx(kPosDestModAddr'range)   = kAddrRCAP and
           reg_frame_rx(kPosDestLocalAddr'range) = kAddrOffset) then
          reg_frame_tx(kReply)(kPosDestModAddr'range)   <= kAddrRCAP;
          reg_frame_tx(kReply)(kPosDestLocalAddr'range) <= kAddrOffset;
          reg_frame_tx(kReply)(kPosSrcModAddr'range)    <= kAddrRCAP;
          reg_frame_tx(kReply)(kPosSrcLocalAddr'range)  <= kAddrOffset;
          reg_frame_tx(kReply)(kPosCmd'range)           <= GenCmdVect(kCmdDepature) or
                                                           GenCmdVect(kCmdReply);
          reg_frame_tx(kReply)(kPosRsv'range)           <= (others => '0');
          reg_frame_tx(kReply)(kPosRegister'range)      <= (others => '0');
          reg_tx_req(kReply)                            <= '1';
          state_rx                                      <= WaitInternalAck;
        end if;

      when WaitInternalAck =>
        if(reg_tx_ack(kReply) = '1') then
          reg_tx_req(kReply)  <= '0';
          state_rx            <= WaitRxIn;
        end if;

      -- From Reply Command --
      when ParseReply =>
        if(reg_frame_rx(kPosSrcModAddr'range)   = kAddrRCAP and
           reg_frame_rx(kPosSrcLocalAddr'range) = kAddrOffset) then
            got_reply(kOffset)  <= '1';
        end if;
        state_rx  <= WaitRxIn;

      when others =>
        state_rx  <= WaitRxIn;


      end case;
    end if;
  end process;


  -- Tx Process ----------------------------------------------------------
  u_intswith : process(clk, sync_reset)
  begin
    if(sync_reset = '1') then
      validBusOut   <= '0';
      reg_tx_ack    <= (others => '0');
      state_switch  <= WaitReq;
    elsif(clk'event and clk = '1') then
    case state_switch is
      when WaitReq =>
        validBusOut <= '0';
        if(reg_tx_req(kWrite) = '1') then
          reg_tx_ack(kWrite)  <= '1';
          dataBusOut          <= reg_frame_tx(kWrite);
          state_switch        <= SendFrame;
        elsif(reg_tx_req(kReply) = '1') then
          reg_tx_ack(kReply)  <= '1';
          dataBusOut          <= reg_frame_tx(kReply);
          state_switch        <= SendFrame;
        end if;

      when SendFrame =>
        validBusOut   <= '1';
        state_switch  <= WaitReq;

      when others =>
        state_switch  <= WaitReq;

    end case;
    end if;
  end process;


  u_txfsm : process(clk, sync_reset)
    variable resend_counter : std_logic_vector(kWidthResend-1 downto 0);
  begin
    if(sync_reset = '1') then
      reg_tx_req(kWrite)  <= '0';
      resend_counter      := (others => '1');
      state_tx            <= WaitSendReq;
    elsif(clk'event and clk = '1') then
    case state_tx is
      when WaitSendReq =>
        if(req_send_offset = '1') then
          state_tx          <= SendFrame;
        end if;

      when SendFrame =>
        reg_frame_tx(kWrite)(kPosDestModAddr'range)   <= kAddrRCAP;
        reg_frame_tx(kWrite)(kPosDestLocalAddr'range) <= kAddrOffset;
        reg_frame_tx(kWrite)(kPosSrcModAddr'range)    <= kAddrRCAP;
        reg_frame_tx(kWrite)(kPosSrcLocalAddr'range)  <= kAddrOffset;
        reg_frame_tx(kWrite)(kPosCmd'range)           <= GenCmdVect(kCmdDepature) or
                                                         GenCmdVect(kCmdWrite) or
                                                         GenCmdVect(kCmdReplyRequest);
        reg_frame_tx(kWrite)(kPosRsv'range)           <= (others => '0');
        reg_frame_tx(kWrite)(kPosRegister'range)      <= std_logic_vector(to_unsigned(0, kPosRegister'length-kWidthOffset)) & reg_hbc_offset;
        reg_tx_req(kWrite)                            <= '1';
        state_tx                                      <= WaitInternalAck;

      when WaitInternalAck =>
        if(reg_tx_ack(kWrite) = '1') then
          resend_counter      := (others => '1');
          reg_tx_req(kWrite)  <= '0';
          state_tx            <= WaitReply;
        end if;

      when WaitReply =>
        if(got_reply(kOffset) = '1') then
          state_tx  <= Done;
        else
          if(to_integer(unsigned(resend_counter)) = 0) then
            state_tx  <= WaitSendReq;
          else
            resend_counter  := std_logic_vector(unsigned(resend_counter) -1);
          end if;
        end if;

      when Done =>
        null;

      when others =>
        state_tx  <= WaitSendReq;

    end case;
    end if;
  end process;



  -- Reset sequence --
  sync_reset  <= reset_shiftreg(kWidthResetSync-1);
  u_sync_reset : process(rst, clk)
  begin
    if(rst = '1') then
      reset_shiftreg  <= (others => '1');
    elsif(clk'event and clk = '1') then
      reset_shiftreg  <= reset_shiftreg(kWidthResetSync-2 downto 0) & '0';
    end if;
  end process;

end RTL;