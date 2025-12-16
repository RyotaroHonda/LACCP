library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library mylib;
use mylib.defCDCM.all;
use mylib.defLaccp.all;

-- Remote Clock Adjustment Protocol --
entity RCAP is
  generic
    (
      kWidthOffset    : integer:= 16;
      kFastClkFreq    : real:= 500.0; -- MHz
      kPrimaryMode    : boolean:= false;
      enDebug         : boolean:= false;
      enCalibDelay    : boolean:= false
    );
  port
    (
      -- System --
      syncReset         : in std_logic; -- Active high
      clk               : in std_logic;

      -- User Interface --
      isDone            : out std_logic;
      clockIsSyncedIn   : in std_logic;

      idelayTapIn       : in unsigned(kWidthTap-1 downto 0);
      serdesLantencyIn  : in signed(kWidthSerdesOffset-1 downto 0);
      cntValueInitIn    : in std_logic_vector(kCNTVALUEbit-1 downto 0);
      cntValueSlaveInitIn : in std_logic_vector(kCNTVALUEbit-1 downto 0);
      idelayTapOut      : out unsigned(kWidthTap-1 downto 0);
      serdesLantencyOut : out signed(kWidthSerdesOffset-1 downto 0);
      cntValueInitOut   : out unsigned(kCNTVALUEbit-1 downto 0);
      cntValueSlaveInitOut : out unsigned(kCNTVALUEbit-1 downto 0);

      calibDelay        : in signed(kCalibDelayWidth-1 downto 0);
      calibDelayOut     : out signed(kCalibDelayWidth-1 downto 0);

      roundTripTime     : out std_logic_vector(kWidthOffset-1 downto 0);

      upstreamOffset    : in signed(kWidthLaccpFineOffset-1 downto 0);
      validOffset       : out std_logic;
      hbcOffset         : out std_logic_vector(kWidthOffset-1 downto 0);
      fineOffset        : out signed(kWidthLaccpFineOffset-1 downto 0);
      fineOffsetLocal   : out signed(kWidthLaccpFineOffset-1 downto 0);

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

  -- Internal signal decralation --
  constant kWidthResend : integer:= 28;

  signal pulse_takeover : std_logic;
  signal probe_pulse, returned_pulse  : std_logic;

  constant kMaxOffset         : std_logic_vector(hbcOffset'range):= (others => '1');
  signal reg_round_trip_time  : std_logic_vector(hbcOffset'range);
  signal reg_hbc_offset       : std_logic_vector(hbcOffset'range);
  signal modified_hbc_offset  : std_logic_vector(hbcOffset'range);
  signal offset_counter       : std_logic_vector(hbcOffset'range):= (others => '0');
  signal reg_valid_offset     : std_logic;
  signal valid_modified_offset : std_logic;
  signal valid_accumulated_offset : std_logic;

  signal reg_fine_offset      : signed(fineOffset'range):= (others => '0');
  signal semi_fine_offset     : signed(fineOffset'range):= (others => '0');
  signal modified_fine_offset : signed(fineOffset'range):= (others => '0');
  signal accumulated_offset   : signed(fineOffset'range):= (others => '0');
  signal upstream_fine_offset : signed(fineOffset'range):= (others => '0');
  signal reg_valid_fineoffset : std_logic;

  signal is_done              : std_logic;
  signal req_send_done        : std_logic;
  signal prioffset_valid      : std_logic;
  type RcapStateType is (Init, AdjustClock, Finalize, Done);
  signal state_rcap           : RcapStateType;

  signal secondary_is_ready   : std_logic;
  signal reg_offset_rx        : std_logic_vector(kPosRegister'range);
  signal req_send_fineoffset  : std_logic;
  signal req_send_CNTVALUEOUT : std_logic;

  signal idelay_tap_rx        : unsigned(idelayTapIn'range);
  signal serdes_latency_rx    : signed(serdesLantencyIn'range);

  signal cntvalueout_init_rx  : unsigned(cntValueInitIn'range);
  signal cntvalueout_slaveinit_rx : unsigned(cntValueSlaveInitIn'range);
  signal calibdelay_rx       : signed(CalibDelay'range);
  signal calibdelay_result   : signed(kWidthLaccpFineOffset-1 downto 0);
  signal dT_sec_US           : signed(kWidthLaccpFineOffset-1 downto 0);
  signal dT_pri_US           : signed(kWidthLaccpFineOffset-1 downto 0);
  signal is_ultrascale_sec       : std_logic;
  signal is_ultrascale_pri       : std_logic;



  -- Local Address --
  constant kAddrDone        : std_logic_vector(kPosDestLocalAddr'length-1 downto 0):= X"0";
  constant kAddrFineOffset  : std_logic_vector(kPosDestLocalAddr'length-1 downto 0):= X"1";
  constant kAddrCNTVALUEOUT : std_logic_vector(kPosDestLocalAddr'length-1 downto 0):= X"2";

  -- FIFO --
  signal re_rx_fifo, rd_valid_rx_fifo, empty_rx_fifo  : std_logic;
  signal dout_rx_fifo         : LaccpFrameBobyType;

  -- FSMs --
  type RxProcessType is (WaitRxIn, ParseFrame, CheckReplyReq, ReplyProcess, WaitInternalAck, ParseReply);
  type SwitchProcessType is (WaitReq, SendFrame);
  type TxProcessType is (TxIdle, SetDoneMessange, SetFineOffset, SetCNTVALUEOUT, WaitInternalAck, WaitReply, Done);

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
  constant kNumReplyPath  : integer:= 2;
  constant kDone          : integer:= 0;
  constant kFineOffset    : integer:= 1;
  signal reg_frame_rx     : LaccpFrameBobyType;
  signal got_reply        : std_logic_vector(kNumReplyPath-1 downto 0);
  signal wait_reply       : std_logic_vector(kNumReplyPath-1 downto 0);

  -- Debug --
  attribute mark_debug  of secondary_is_ready       : signal is enDebug;
  attribute mark_debug  of reg_round_trip_time      : signal is enDebug;
  attribute mark_debug  of reg_hbc_offset           : signal is enDebug;
  attribute mark_debug  of reg_valid_offset         : signal is enDebug;
  attribute mark_debug  of valid_modified_offset    : signal is enDebug;
  attribute mark_debug  of valid_accumulated_offset : signal is enDebug;
  attribute mark_debug  of is_done                  : signal is enDebug;
  attribute mark_debug  of state_rx                 : signal is enDebug;
  attribute mark_debug  of state_tx                 : signal is enDebug;

    --add 2024/12/12
  attribute mark_debug  of pulse_takeover           : signal is enDebug;
  attribute mark_debug  of state_rcap               : signal is enDebug;
  attribute mark_debug  of req_send_fineoffset      : signal is enDebug;
  attribute mark_debug  of semi_fine_offset         : signal is enDebug;
  attribute mark_debug  of dout_rx_fifo             : signal is enDebug;
  attribute mark_debug  of validBusIn               : signal is enDebug;
  attribute mark_debug  of state_switch             : signal is enDebug;
  attribute mark_debug  of got_reply                : signal is enDebug;
  attribute mark_debug  of cntvalueout_init_rx      : signal is enDebug;
  attribute mark_debug  of cntvalueout_slaveinit_rx : signal is enDebug;
  attribute mark_debug  of calibdelay_rx            : signal is enDebug;

  attribute mark_debug  of dT_sec_US : signal is enDebug;
  attribute mark_debug  of dT_pri_US : signal is enDebug;
  attribute mark_debug  of is_ultrascale_sec : signal is enDebug;
  attribute mark_debug  of is_ultrascale_pri : signal is enDebug;



begin
  -- =================================================================
  --                           Body
  -- =================================================================

  isDone          <= is_done;

  pulseTakeOver   <= pulse_takeover;
  isReadyOut      <= '1';

  idelayTapOut      <= idelay_tap_rx;
  serdesLantencyOut <= serdes_latency_rx;

  cntValueInitOut   <= cntvalueout_init_rx;
  cntValueSlaveInitOut <= cntvalueout_slaveinit_rx;
  CalibDelayOut     <= calibdelay_rx;

  roundTripTime     <= reg_round_trip_time;

  -- Primary mode ----------------------------------------------------
  gen_prim : if kPrimaryMode = true generate
  begin
    probePulseOut       <= probePulseIn;

    reg_round_trip_time <= (others => '0');
    hbcOffset           <= reg_offset_rx(kWidthOffset-1 downto 0);
    fineOffset          <= to_signed(0, fineOffset'length);
    validOffset         <= secondary_is_ready;

    u_rcapstate : process(clk)
    begin
      if(clk'event and clk = '1') then
        if(syncReset = '1') then
          is_done             <= '0';
          pulse_takeover      <= '1';
          req_send_fineoffset <= '1';
        else
          if(secondary_is_ready = '1') then
            is_done             <= '1';
            pulse_takeover      <= '0';
          else
            null;
          end if;

          if(got_reply(kFineOffset) = '0') then
            req_send_fineoffset <= '1';
          else
            req_send_fineoffset <= '0';
          end if;
        end if;
      end if;
    end process;

  end generate;

  -- Secondary mode --------------------------------------------------
  gen_secnd : if kPrimaryMode = false generate
  begin

    returned_pulse    <= probePulseIn;
    probePulseOut     <= probe_pulse;

    hbcOffset         <= modified_hbc_offset;
    fineOffset        <= accumulated_offset;
    fineOffsetLocal   <= modified_fine_offset;
    validOffset       <= valid_accumulated_offset;

    --Since CalcFineLantency_US is written in Verilog, the library is 'work'
    u_CalcFineLantency_US_sec : entity work.CalcFineLantency_US
    generic map
      (
        kFastClkFreq    => kFastClkFreq
      )
        port map (
            CLK                 => clk,
            CNTVALUEOUTInit     => cntValueInitIn,
            CNTVALUEOUT_slaveInit => cntValueSlaveInitIn,
            idelay_tap         => idelayTapIn,
            serdes_latency     => serdesLantencyIn,
            result             => dT_sec_US,
            is_ultrascale      => is_ultrascale_sec
        );

    u_CalcFineLantency_US_pri : entity work.CalcFineLantency_US
    generic map
      (
        kFastClkFreq    => kFastClkFreq
      )
        port map (
            CLK                 => clk,
            CNTVALUEOUTInit     => cntvalueout_init_rx,
            CNTVALUEOUT_slaveInit => cntvalueout_slaveinit_rx,
            idelay_tap         => idelay_tap_rx,
            serdes_latency     => serdes_latency_rx,
            result             => dT_pri_US,
            is_ultrascale      => is_ultrascale_pri
        );

    u_CalcCalibDelay : entity work.CalcCalibDelay
    generic map
    (
        enCalibDelay          => enCalibDelay,
        kCalibDelayWidth      => kCalibDelayWidth,
        kWidthLaccpFineOffset => kWidthLaccpFineOffset,
        kFastClkFreq          => kFastClkFreq
    )
    port map (
        CLK                   => clk,
        is_ultrascale_sec     => is_ultrascale_sec,
        is_ultrascale_pri     => is_ultrascale_pri,
        CalibDelay            => CalibDelay,
        calibdelay_rx         => calibdelay_rx,
        result                => calibdelay_result

    );

    u_fine_carry : process(clk)
      variable kPlusCycle          : signed(fineOffset'range):= to_signed(kFullCycle*GetFastClockPeriod(kFastClkFreq), fineOffset'length);
      variable kMinusCycle         : signed(fineOffset'range):= to_signed(-kFullCycle*GetFastClockPeriod(kFastClkFreq), fineOffset'length);
      variable tmp_fine_offset     : signed(fineOffset'range);
      variable tmp2_fine_offset    : signed(fineOffset'range);
    begin
      if(clk'event and clk = '1') then
        if(syncReset = '1') then
          valid_modified_offset     <= '0';
          valid_accumulated_offset  <= '0';
        else
          if(reg_valid_fineoffset = '1' and reg_valid_offset = '1' and valid_modified_offset = '0') then
            tmp_fine_offset := reg_fine_offset + semi_fine_offset;
            if(tmp_fine_offset > kPlusCycle) then
              modified_fine_offset  <= tmp_fine_offset - kPlusCycle;
              modified_hbc_offset   <= std_logic_vector(unsigned(reg_hbc_offset) +1);
            else
              modified_fine_offset  <= tmp_fine_offset;
              modified_hbc_offset   <= reg_hbc_offset;
            end if;

            valid_modified_offset   <= '1';
          end if;

          if(valid_modified_offset = '1' and valid_accumulated_offset = '0') then
            tmp2_fine_offset  := modified_fine_offset + upstream_fine_offset;
            if(tmp2_fine_offset > kPlusCycle) then
              accumulated_offset    <= tmp2_fine_offset - kPlusCycle;
              modified_hbc_offset   <= std_logic_vector(unsigned(modified_hbc_offset) +1);
            elsif(tmp2_fine_offset < kMinusCycle) then
              accumulated_offset    <= tmp2_fine_offset + kPlusCycle;
              modified_hbc_offset   <= std_logic_vector(unsigned(modified_hbc_offset) -1);
            else
              accumulated_offset    <= tmp2_fine_offset;
            end if;

            valid_accumulated_offset  <= '1';
          end if;
        end if;
      end if;
    end process;

    u_offset : process(clk)
    begin
      if(clk'event and clk = '1') then
        if(syncReset = '1') then
          reg_hbc_offset      <= (others => '0');
          reg_round_trip_time <= (others => '0');
          reg_valid_offset    <= '0';
        else
          if(is_done = '0') then
            offset_counter  <= std_logic_vector(unsigned(offset_counter) +1);

            if(offset_counter = kMaxOffset) then
              probe_pulse   <= '1';
            else
              probe_pulse   <= '0';
            end if;

            if(returned_pulse = '1' and reg_valid_offset = '0') then
              reg_round_trip_time  <= offset_counter;
              reg_hbc_offset       <= '0' & offset_counter(kWidthOffset-1 downto 1);
              if(offset_counter(0) = '1') then
                semi_fine_offset   <= to_signed(kHalfCycle*GetFastClockPeriod(kFastClkFreq), semi_fine_offset'length);
              else
                semi_fine_offset   <= to_signed(0, semi_fine_offset'length);
              end if;

              reg_valid_offset     <= '1';
            end if;
          end if;
        end if;
      end if;
    end process;

    u_fineoffset : process(clk)
      variable dT_sec   : signed(reg_fine_offset'range):= (others => '0');
      variable dT_pri   : signed(reg_fine_offset'range):= (others => '0');

      variable pri_idelay_tap : signed(idelay_tap_rx'length downto 0):= (others => '0');
      variable sec_idelay_tap : signed(idelay_tap_rx'length downto 0):= (others => '0');
      variable result   : signed(reg_fine_offset'range):= (others => '0');
    begin
      if(clk'event and clk = '1') then
        if(syncReset = '1') then
          reg_fine_offset       <= (others => '0');
          reg_valid_fineoffset  <= '0';
        else
          if(is_done = '0' and prioffset_valid = '1') then
            pri_idelay_tap  := signed('0' & idelay_tap_rx);
            sec_idelay_tap  := signed('0' & idelayTapIn);

            if(is_ultrascale_pri = '1') then
              dT_pri    := dT_pri_US;
            else
              dT_pri    := CalcFineLantency(pri_idelay_tap, serdes_latency_rx, GetFastClockPeriod(kFastClkFreq));
            end if;

            if(is_ultrascale_sec = '1') then
              dT_sec := dT_sec_US;
            else
              dT_sec    := CalcFineLantency(sec_idelay_tap, serdesLantencyIn,  GetFastClockPeriod(kFastClkFreq));
            end if;

            result    := dT_sec - dT_pri + resize(calibdelay_result, kWidthLaccpFineOffset);
            reg_fine_offset <= result(result'high) & result(result'high downto 1);
            reg_valid_fineoffset     <= '1';
          end if;
        end if;
      end if;
    end process;


    u_rcapstate : process(clk)
    begin
      if(clk'event and clk = '1') then
        if(syncReset = '1') then
          pulse_takeover  <= '1';
          is_done         <= '0';
          req_send_done <= '0';
          state_rcap      <= Init;
        else
          case state_rcap is
            when Init =>
              pulse_takeover  <= '1';
              is_done         <= '0';
              req_send_done <= '0';
              state_rcap      <= AdjustClock;

            when AdjustClock =>
              pulse_takeover  <= '1';

              if(clockIsSyncedIn = '1') then
                req_send_done <= '1';
                state_rcap      <= Finalize;
              end if;

            when Finalize =>
              if(got_reply(kDone) = '1') then
                req_send_done <= '0';
                state_rcap      <= Done;
              end if;

            when Done =>
              is_done         <= '1';
              pulse_takeover  <= '0';

            when others => null;

          end case;
        end if;
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
      rst       => syncReset,
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


  u_rxfsm : process(clk)
    variable  rx_register : std_logic_vector(kPosRegister'range);
  begin
    if(clk'event and clk = '1') then
      if(syncReset = '1') then
        secondary_is_ready  <= '0';
        prioffset_valid     <= '0';
        re_rx_fifo          <= '0';
        reg_tx_req(kReply)  <= '0';
        got_reply           <= (others => '0');
        state_rx            <= WaitRxIn;
      else
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
              if(dout_rx_fifo(kPosDestLocalAddr'range) = kAddrDone) then
                reg_frame_rx      <= dout_rx_fifo;
                reg_offset_rx     <= dout_rx_fifo(kPosRegister'range);
                secondary_is_ready  <= '1';
                state_rx          <= CheckReplyReq;
              elsif(dout_rx_fifo(kPosDestLocalAddr'range) = kAddrFineOffset) then
                reg_frame_rx          <= dout_rx_fifo;
                rx_register           := dout_rx_fifo(kPosRegister'range);
                idelay_tap_rx         <= unsigned(rx_register(kWidthTap-1 downto 0));
                serdes_latency_rx     <= signed(rx_register(kWidthSerdesOffset-1 + kWidthTap downto kWidthTap));
                upstream_fine_offset  <= signed(rx_register(kWidthLaccpFineOffset-1 +kWidthSerdesOffset + kWidthTap downto kWidthSerdesOffset + kWidthTap));
                --prioffset_valid       <= '1';
                state_rx              <= CheckReplyReq;
              elsif(dout_rx_fifo(kPosDestLocalAddr'range) = kAddrCNTVALUEOUT) then
                reg_frame_rx          <= dout_rx_fifo;
                rx_register           := dout_rx_fifo(kPosRegister'range);
                cntvalueout_init_rx   <= unsigned(rx_register(kCNTVALUEbit-1 + kCNTVALUEbit + kCalibDelayWidth downto kCNTVALUEbit + kCalibDelayWidth));
                cntvalueout_slaveinit_rx      <= unsigned(rx_register(kCNTVALUEbit-1 + kCalibDelayWidth downto kCalibDelayWidth));
                calibdelay_rx         <= signed(rx_register(kCalibDelayWidth-1 downto 0));
                prioffset_valid       <= '1';
                state_rx              <= CheckReplyReq;
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
            reg_frame_rx(kPosDestLocalAddr'range) = kAddrDone) then
            reg_frame_tx(kReply)(kPosDestModAddr'range)   <= kAddrRCAP;
            reg_frame_tx(kReply)(kPosDestLocalAddr'range) <= kAddrDone;
            reg_frame_tx(kReply)(kPosSrcModAddr'range)    <= kAddrRCAP;
            reg_frame_tx(kReply)(kPosSrcLocalAddr'range)  <= kAddrDone;
            reg_frame_tx(kReply)(kPosCmd'range)           <= GenCmdVect(kCmdDepature) or
                                                            GenCmdVect(kCmdReply);
            reg_frame_tx(kReply)(kPosRsv'range)           <= (others => '0');
            reg_frame_tx(kReply)(kPosRegister'range)      <= (others => '0');
            reg_tx_req(kReply)                            <= '1';
            state_rx                                      <= WaitInternalAck;
          elsif(reg_frame_rx(kPosDestModAddr'range)   = kAddrRCAP and
                reg_frame_rx(kPosDestLocalAddr'range) = kAddrFineOffset) then
            reg_frame_tx(kReply)(kPosDestModAddr'range)   <= kAddrRCAP;
            reg_frame_tx(kReply)(kPosDestLocalAddr'range) <= kAddrFineOffset;
            reg_frame_tx(kReply)(kPosSrcModAddr'range)    <= kAddrRCAP;
            reg_frame_tx(kReply)(kPosSrcLocalAddr'range)  <= kAddrFineOffset;
            reg_frame_tx(kReply)(kPosCmd'range)           <= GenCmdVect(kCmdDepature) or
                                                            GenCmdVect(kCmdReply);
            reg_frame_tx(kReply)(kPosRsv'range)           <= (others => '0');
            reg_frame_tx(kReply)(kPosRegister'range)      <= std_logic_vector(to_unsigned(0, kPosRegister'length-kWidthTap-kWidthSerdesOffset)) & std_logic_vector(serdesLantencyIn) & std_logic_vector(idelayTapIn);
  --            kWidthTap-1 downto 0 => std_logic_vector(idelayTapIn),
  --            kWidthSerdesOffset-1 + kWidthTap downto kWidthTap => std_logic_vector(serdesLantencyIn),
  --            others => '0'
  --            );
            reg_tx_req(kReply)                            <= '1';
            state_rx                                      <= WaitInternalAck;
          elsif(reg_frame_rx(kPosDestModAddr'range)   = kAddrRCAP and
                reg_frame_rx(kPosDestLocalAddr'range) = kAddrCNTVALUEOUT) then
            reg_frame_tx(kReply)(kPosDestModAddr'range)   <= kAddrRCAP;
            reg_frame_tx(kReply)(kPosDestLocalAddr'range) <= kAddrCNTVALUEOUT;
            reg_frame_tx(kReply)(kPosSrcModAddr'range)    <= kAddrRCAP;
            reg_frame_tx(kReply)(kPosSrcLocalAddr'range)  <= kAddrCNTVALUEOUT;
            reg_frame_tx(kReply)(kPosCmd'range)           <= GenCmdVect(kCmdDepature) or
                                                            GenCmdVect(kCmdReply);
            reg_frame_tx(kReply)(kPosRsv'range)           <= (others => '0');
            reg_frame_tx(kReply)(kPosRegister'range)      <= std_logic_vector(to_unsigned(0, kPosRegister'length-kCNTVALUEbit-kCNTVALUEbit-kCalibDelayWidth)) & std_logic_vector(cntValueInitIn) & std_logic_vector(cntValueSlaveInitIn) & std_logic_vector(CalibDelay);
  --            kWidthTap-1 downto 0 => std_logic_vector(idelayTapIn),
  --            kWidthSerdesOffset-1 + kWidthTap downto kWidthTap => std_logic_vector(serdesLantencyIn),
  --            others => '0'
  --            );
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
            reg_frame_rx(kPosSrcLocalAddr'range) = kAddrDone) then
              got_reply(kDone)  <= '1';
          elsif(reg_frame_rx(kPosSrcModAddr'range)   = kAddrRCAP and
                reg_frame_rx(kPosSrcLocalAddr'range) = kAddrFineOffset) then
              rx_register       := reg_frame_rx(kPosRegister'range);
              idelay_tap_rx     <= unsigned(rx_register(kWidthTap-1 downto 0));
              serdes_latency_rx <= signed(rx_register(kWidthSerdesOffset-1 + kWidthTap downto kWidthTap));
              got_reply(kFineOffset)  <= '1';
          elsif(reg_frame_rx(kPosSrcModAddr'range)   = kAddrRCAP and
                reg_frame_rx(kPosSrcLocalAddr'range) = kAddrCNTVALUEOUT) then
              rx_register       := reg_frame_rx(kPosRegister'range);
                cntvalueout_init_rx   <= unsigned(rx_register(kCNTVALUEbit-1 + kCNTVALUEbit + kCalibDelayWidth downto kCNTVALUEbit + kCalibDelayWidth));
                cntvalueout_slaveinit_rx      <= unsigned(rx_register(kCNTVALUEbit-1 + kCalibDelayWidth downto kCalibDelayWidth));
                calibdelay_rx         <= signed(rx_register(kCalibDelayWidth-1 downto 0));
              got_reply(kFineOffset)  <= '1';
          end if;
          state_rx  <= WaitRxIn;

        when others =>
          state_rx  <= WaitRxIn;


        end case;
      end if;
    end if;
  end process;


  -- Tx Process ----------------------------------------------------------
  u_intswith : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(syncReset = '1') then
        validBusOut   <= '0';
        reg_tx_ack    <= (others => '0');
        state_switch  <= WaitReq;
      else
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
    end if;
  end process;


  u_txfsm : process(clk)
    variable resend_counter : std_logic_vector(kWidthResend-1 downto 0);
  begin
    if(clk'event and clk = '1') then
      if(syncReset = '1') then
        reg_tx_req(kWrite)  <= '0';
        resend_counter      := (others => '1');
        wait_reply          <= (others => '0');
        state_tx            <= TxIdle;
        req_send_CNTVALUEOUT <= '0';
      else
      case state_tx is
        when TxIdle =>
          if(req_send_done = '1') then
            state_tx          <= SetDoneMessange;
          elsif(req_send_fineoffset = '1') then
            state_tx          <= SetFineOffset;
          elsif(req_send_CNTVALUEOUT = '1') then
            state_tx          <= SetCNTVALUEOUT;
          end if;

        when SetDoneMessange =>
          reg_frame_tx(kWrite)(kPosDestModAddr'range)   <= kAddrRCAP;
          reg_frame_tx(kWrite)(kPosDestLocalAddr'range) <= kAddrDone;
          reg_frame_tx(kWrite)(kPosSrcModAddr'range)    <= kAddrRCAP;
          reg_frame_tx(kWrite)(kPosSrcLocalAddr'range)  <= kAddrDone;
          reg_frame_tx(kWrite)(kPosCmd'range)           <= GenCmdVect(kCmdDepature) or
                                                          GenCmdVect(kCmdWrite) or
                                                          GenCmdVect(kCmdReplyRequest);
          reg_frame_tx(kWrite)(kPosRsv'range)           <= (others => '0');
          reg_frame_tx(kWrite)(kPosRegister'range)      <= std_logic_vector(to_unsigned(0, kPosRegister'length-kWidthOffset)) & modified_hbc_offset;
          reg_tx_req(kWrite)                            <= '1';
          wait_reply(kDone)                             <= '1';
          state_tx                                      <= WaitInternalAck;

        when SetFineOffset =>
          reg_frame_tx(kWrite)(kPosDestModAddr'range)   <= kAddrRCAP;
          reg_frame_tx(kWrite)(kPosDestLocalAddr'range) <= kAddrFineOffset;
          reg_frame_tx(kWrite)(kPosSrcModAddr'range)    <= kAddrRCAP;
          reg_frame_tx(kWrite)(kPosSrcLocalAddr'range)  <= kAddrFineOffset;
          reg_frame_tx(kWrite)(kPosCmd'range)           <= GenCmdVect(kCmdDepature) or
                                                          GenCmdVect(kCmdWrite) or
                                                          GenCmdVect(kCmdReplyRequest);
          reg_frame_tx(kWrite)(kPosRsv'range)           <= (others => '0');
          reg_frame_tx(kWrite)(kPosRegister'range)      <= std_logic_vector(to_unsigned(0, kPosRegister'length-kWidthTap-kWidthSerdesOffset-kWidthLaccpFineOffset)) & std_logic_vector(upstreamOffset) & std_logic_vector(serdesLantencyIn) & std_logic_vector(idelayTapIn);
  --          kWidthTap-1 downto 0 => std_logic_vector(idelayTapIn),
  --          kWidthSerdesOffset-1 + kWidthTap downto kWidthTap => std_logic_vector(serdesLantencyIn),
  --          others => '0'
  --          );
          reg_tx_req(kWrite)                            <= '1';
          wait_reply(kFineOffset)                       <= '1';
          req_send_CNTVALUEOUT                          <= '1';
          state_tx                                      <= WaitInternalAck;

        when SetCNTVALUEOUT =>
          reg_frame_tx(kWrite)(kPosDestModAddr'range)   <= kAddrRCAP;
          reg_frame_tx(kWrite)(kPosDestLocalAddr'range) <= kAddrCNTVALUEOUT;
          reg_frame_tx(kWrite)(kPosSrcModAddr'range)    <= kAddrRCAP;
          reg_frame_tx(kWrite)(kPosSrcLocalAddr'range)  <= kAddrCNTVALUEOUT;
          reg_frame_tx(kWrite)(kPosCmd'range)           <= GenCmdVect(kCmdDepature) or
                                                          GenCmdVect(kCmdWrite) or
                                                          GenCmdVect(kCmdReplyRequest);
          reg_frame_tx(kWrite)(kPosRsv'range)           <= (others => '0');
          reg_frame_tx(kWrite)(kPosRegister'range)      <= std_logic_vector(to_unsigned(0, kPosRegister'length-kCNTVALUEbit-kCNTVALUEbit-kCalibDelayWidth)) & std_logic_vector(cntValueInitIn) & std_logic_vector(cntValueSlaveInitIn) & std_logic_vector(CalibDelay);


          reg_tx_req(kWrite)                            <= '1';
          wait_reply(kFineOffset)                       <= '1';
          req_send_CNTVALUEOUT                          <= '0';
          state_tx                                      <= WaitInternalAck;

        when WaitInternalAck =>
          if(reg_tx_ack(kWrite) = '1') then
            resend_counter      := (others => '1');
            reg_tx_req(kWrite)  <= '0';
            state_tx            <= WaitReply;
          end if;

        when WaitReply =>
          if(unsigned(got_reply xor wait_reply) /= 0) then
            if(to_integer(unsigned(resend_counter)) = 0) then
              state_tx  <= TxIdle;
            else
              resend_counter  := std_logic_vector(unsigned(resend_counter) -1);
            end if;
          elsif(req_send_CNTVALUEOUT = '1') then
            state_tx  <= TxIdle;
          end if;

        --when Done =>
        --  null;

        when others =>
          state_tx  <= TxIdle;

      end case;
      end if;
    end if;
  end process;

end RTL;
