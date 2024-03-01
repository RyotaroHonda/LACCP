library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library mylib;
use mylib.defLaccp.all;

-- Remote Link Information Getter Protocol --
entity RLIGP is
  generic
    (
      enDebug         : boolean:= false
    );
  port
    (
      -- System --
      rst               : in std_logic; -- Asynchronous, Active high
      clk               : in std_logic;

      -- User Interface --
      addrMyLink        : in std_logic_vector(kPosRegister'range);
      validMyLink       : in std_logic;
      addrPartnerLink   : out std_logic_vector(kPosRegister'range);
      validPartnerLink  : out std_logic;

      -- LACCP Bus --
      dataBusIn         : in LaccpFrameBobyType;
      validBusIn        : in std_logic;

      dataBusOut        : out LaccpFrameBobyType;
      validBusOut       : out std_logic;
      isReadyOut        : out std_logic

    );
end RLIGP;

architecture RTL of RLIGP is
  attribute mark_debug  : boolean;

  -- System --
  signal sync_reset           : std_logic;
  constant kWidthResetSync    : integer:= 16;
  signal reset_shiftreg       : std_logic_vector(kWidthResetSync-1 downto 0);

  -- Internal signal decralation --
  constant kWidthResend : integer:= 28;
  signal reg_addr_my_link     : std_logic_vector(addrMyLink'range);

  -- Local Address --
  constant kAddrLinkAddress   : std_logic_vector(kPosDestLocalAddr'length-1 downto 0):= X"0";

  -- FIFO --
  signal re_rx_fifo, rd_valid_rx_fifo, empty_rx_fifo  : std_logic;
  signal dout_rx_fifo         : LaccpFrameBobyType;

  -- FSMs --
  type RxProcessType is (WaitRxIn, ParseFrame, CheckReplyReq, ReplyProcess, WaitInternalAck, ParseReply);
  type SwitchProcessType is (TxIdle, SendFrame);
  type TxProcessType is (TxIdle, SetAddress, WaitInternalAck, WaitReply, Done);

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
  constant kMyAddr        : integer:= 0;
  signal reg_frame_rx     : LaccpFrameBobyType;
  signal got_reply        : std_logic_vector(kNumReplyPath-1 downto 0);

  -- Debug --


begin
  -- =================================================================
  --                           Body
  -- =================================================================

  isReadyOut  <= '1';

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
      validPartnerLink    <= '0';
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
            if(dout_rx_fifo(kPosDestLocalAddr'range) = kAddrLinkAddress) then
              reg_frame_rx      <= dout_rx_fifo;
              addrPartnerLink   <= dout_rx_fifo(kPosRegister'range);
              validPartnerLink  <= '1';
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
        if(reg_frame_rx(kPosDestModAddr'range)   = kAddrRLIGP and
           reg_frame_rx(kPosDestLocalAddr'range) = kAddrLinkAddress) then
          reg_frame_tx(kReply)(kPosDestModAddr'range)   <= kAddrRLIGP;
          reg_frame_tx(kReply)(kPosDestLocalAddr'range) <= kAddrLinkAddress;
          reg_frame_tx(kReply)(kPosSrcModAddr'range)    <= kAddrRLIGP;
          reg_frame_tx(kReply)(kPosSrcLocalAddr'range)  <= kAddrLinkAddress;
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
        if(reg_frame_rx(kPosSrcModAddr'range)   = kAddrRLIGP and
           reg_frame_rx(kPosSrcLocalAddr'range) = kAddrLinkAddress) then
            got_reply(kMyAddr)  <= '1';
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
      state_switch  <= TxIdle;
    elsif(clk'event and clk = '1') then
    case state_switch is
      when TxIdle =>
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
        state_switch  <= TxIdle;

      when others =>
        state_switch  <= TxIdle;

    end case;
    end if;
  end process;


  u_txfsm : process(clk, sync_reset)
    variable resend_counter : std_logic_vector(kWidthResend-1 downto 0);
  begin
    if(sync_reset = '1') then
      reg_tx_req(kWrite)  <= '0';
      resend_counter      := (others => '1');
      state_tx            <= TxIdle;
    elsif(clk'event and clk = '1') then
    case state_tx is
      when TxIdle =>
        if(validMyLink = '1') then
          reg_addr_my_link  <= addrMyLink;
          state_tx          <= SetAddress;
        end if;

      when SetAddress =>
        reg_frame_tx(kWrite)(kPosDestModAddr'range)   <= kAddrRLIGP;
        reg_frame_tx(kWrite)(kPosDestLocalAddr'range) <= kAddrLinkAddress;
        reg_frame_tx(kWrite)(kPosSrcModAddr'range)    <= kAddrRLIGP;
        reg_frame_tx(kWrite)(kPosSrcLocalAddr'range)  <= kAddrLinkAddress;
        reg_frame_tx(kWrite)(kPosCmd'range)           <= GenCmdVect(kCmdDepature) or
                                                         GenCmdVect(kCmdWrite) or
                                                         GenCmdVect(kCmdReplyRequest);
        reg_frame_tx(kWrite)(kPosRsv'range)           <= (others => '0');
        reg_frame_tx(kWrite)(kPosRegister'range)      <= reg_addr_my_link;
        reg_tx_req(kWrite)                            <= '1';
        state_tx                                      <= WaitInternalAck;

      when WaitInternalAck =>
        if(reg_tx_ack(kWrite) = '1') then
          resend_counter      := (others => '1');
          reg_tx_req(kWrite)  <= '0';
          state_tx            <= WaitReply;
        end if;

      when WaitReply =>
        if(got_reply(kMyAddr) = '1') then
          state_tx  <= Done;
        else
          if(to_integer(unsigned(resend_counter)) = 0) then
            state_tx  <= TxIdle;
          else
            resend_counter  := std_logic_vector(unsigned(resend_counter) -1);
          end if;
        end if;

      when Done =>
        null;

      when others =>
        state_tx  <= TxIdle;

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