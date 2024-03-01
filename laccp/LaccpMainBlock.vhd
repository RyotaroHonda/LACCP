library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library mylib;
use mylib.defLaccp.all;
use mylib.defHeartBeatUnit.all;
use mylib.defMikumari.all;
use mylib.defCDCM.all;

entity LaccpMainBlock is
  generic
    (
      kPrimaryMode      : boolean:= false;
      kNumInterconnect  : integer:= 16;
      kFastClkFreq      : real:= 500.0;
      enDebug           : boolean:= false
    );
  port
    (
      -- System --------------------------------------------------------
      rst               : in std_logic; -- Asynchronous, Active high
      clk               : in std_logic;

      -- User Interface ------------------------------------------------
      isReadyForDaq     : out std_logic;
      laccpPulsesIn     : in  std_logic_vector(kNumLaccpPulse-1 downto 0);
      laccpPulsesOut    : out std_logic_vector(kNumLaccpPulse-1 downto 0);
      pulseInRejected   : out std_logic;

      -- RLIGP --
      addrMyLink        : in std_logic_vector(kPosRegister'range);
      validMyLink       : in std_logic;
      addrPartnerLink   : out std_logic_vector(kPosRegister'range);
      validPartnerLink  : out std_logic;

      -- RCAP --
      idelayTapIn       : in unsigned(kWidthTap-1 downto 0);
      serdesLantencyIn  : in signed(kWidthSerdesOffset-1 downto 0);
      idelayTapOut      : out unsigned(kWidthTap-1 downto 0);
      serdesLantencyOut : out signed(kWidthSerdesOffset-1 downto 0);

      hbuIsSyncedIn     : in std_logic;
      syncPulseIn       : in std_logic;
      syncPulseOut      : out std_logic;

      upstreamOffset    : in signed(kWidthLaccpFineOffset-1 downto 0);
      validOffset       : out std_logic;
      hbcOffset         : out std_logic_vector(kWidthHbCount-1 downto 0);
      fineOffset        : out signed(kWidthLaccpFineOffset-1 downto 0);
      fineOffsetLocal   : out signed(kWidthLaccpFineOffset-1 downto 0);

      -- LACCP Bus Port ------------------------------------------------
      -- Intra-port--
      isReadyIntraIn    : in  std_logic_vector(kNumExtIntraPort-1 downto 0);
      dataIntraIn       : in  ExtIntraType;
      validIntraIn      : in  std_logic_vector(kNumExtIntraPort-1 downto 0);
      dataIntraOut      : out ExtIntraType;
      validIntraOut     : out std_logic_vector(kNumExtIntraPort-1 downto 0);

      -- Interconnect --
      isReadyInterIn    : in  std_logic_vector(kMaxNumInterconnect-1 downto 0);
      existInterOut     : out std_logic_vector(kMaxNumInterconnect-1 downto 0);
      dataInterIn       : in  ExtInterType;
      validInterIn      : in  std_logic_vector(kMaxNumInterconnect-1 downto 0);
      dataInterOut      : out ExtInterType;
      validInterOut     : out std_logic_vector(kMaxNumInterconnect-1 downto 0);

      -- MIKUMARI-Link -------------------------------------------------
      mikuLinkUpIn      : in std_logic;

      -- TX port --
      dataTx            : out CbtUDataType;
      validTx           : out std_logic;
      frameLastTx       : out std_logic;
      txAck             : in std_logic;

      pulseTx           : out std_logic;
      pulseTypeTx       : out MikumariPulseType;
      busyPulseTx       : in std_logic;

      -- RX port --
      dataRx            : in CbtUDataType;
      validRx           : in std_logic;
      frameLastRx       : in std_logic;
      checkSumErrRx     : in std_logic;
      frameBrokenRx     : in std_logic;
      recvTermndRx      : in std_logic;

      pulseRx           : in std_logic;
      pulseTypeRx       : in MikumariPulseType

    );
end LaccpMainBlock;

architecture RTL of LaccpMainBlock is
  attribute mark_debug  : boolean;

  -- System --
  signal laccp_reset    : std_logic;

  -- Internal signal decralation --
  -- MIKUMARI pulse --
  subtype InternalPulseType is std_logic_vector(3 downto 0);
  constant kMultiPulse    : InternalPulseType:= "1000";

  function EncodeMikuPulse(pusle_vector : std_logic_vector) return InternalPulseType is
  begin
    case pusle_vector is
      when "00000001" => return "0000";
      when "00000010" => return "0001";
      when "00000100" => return "0010";
      when "00001000" => return "0011";
      when "00010000" => return "0100";
      when "00100000" => return "0101";
      when "01000000" => return "0110";
      when "10000000" => return "0111";
      when others => return kMultiPulse;
    end case;
  end EncodeMikuPulse;

  function DecodeMikuPulse(pulse_type : MikumariPulseType) return std_logic_vector is
  begin
    case pulse_type is
      when "000"  => return "00000001";
      when "001"  => return "00000010";
      when "010"  => return "00000100";
      when "011"  => return "00001000";
      when "100"  => return "00010000";
      when "101"  => return "00100000";
      when "110"  => return "01000000";
      when "111"  => return "10000000";
      when others => return "00000000";
    end case;
  end DecodeMikuPulse;

  signal pulse_tx_rejected    : std_logic;
  signal reg_pulse_tx         : std_logic;
  signal type_vector_tx       : InternalPulseType;
  signal merged_pulse_tx      : std_logic_vector(kNumLaccpPulse-1 downto 0);
  signal busy_pulse_tx        : std_logic;

  signal reg_pulse_vector_rx  : std_logic_vector(kNumLaccpPulse-1 downto 0);

  -- RCAP --
  signal rcap_is_done         : std_logic;
  signal valid_offset         : std_logic;
  signal rcap_pulse_takeover  : std_logic;
  signal probe_pulse_in, probe_pulse_out  : std_logic;


  -- LACCP bus --
  constant kNumBusPorts       : integer:= kNumIntraPort + kNumInterconnect + 1;

  signal data_bus_in          : LaccpBusDataType;
  signal valid_bus_in         : LaccpBusCtrlType;
  signal bus_ready_in         : LaccpBusCtrlType;
  signal data_bus_out         : LaccpBusDataType;
  signal valid_bus_out        : LaccpBusCtrlType;

  signal frame_data_rx, frame_data_tx : LaccpFrameBobyType;
  signal valid_frame_rx, we_frame_data_tx : std_logic;

  -- Debug --

begin
  -- =================================================================
  --                           Body
  -- =================================================================

  isReadyForDaq <= rcap_is_done;

  validOffset   <= valid_offset;

  laccp_reset <= rst or (not mikuLinkUpIn);

  -- Protocols -------------------------------------------------------
  u_RLIGP : entity mylib.RLIGP
    generic map
      (
        enDebug         => enDebug
      )
    port map
      (
        -- System --
        rst               => laccp_reset,
        clk               => clk,

        -- User Interface --
        addrMyLink        => addrMyLink,
        validMyLink       => validMyLink,
        addrPartnerLink   => addrPartnerLink,
        validPartnerLink  => validPartnerLink,

        -- LACCP Bus --
        dataBusIn         => data_bus_out(kPortRLIGP),
        validBusIn        => valid_bus_out(kPortRLIGP),
        dataBusOut        => data_bus_in(kPortRLIGP),
        validBusOut       => valid_bus_in(kPortRLIGP),
        isReadyOut        => bus_ready_in(kPortRLIGP)

      );

  --
  u_RCAP : entity mylib.RCAP
    generic map
      (
        kWidthOffset    => kWidthHbCount,
        kFastClkFreq    => kFastClkFreq,
        kPrimaryMode    => kPrimaryMode,
        enDebug         => enDebug
      )
    port map
      (
        -- System --
        rst               => laccp_reset,
        clk               => clk,

        -- User Interface --
        idelayTapIn       => idelayTapIn,
        serdesLantencyIn  => serdesLantencyIn,
        idelayTapOut      => idelayTapOut,
        serdesLantencyOut => serdesLantencyOut,

        isDone            => rcap_is_done,
        clockIsSyncedIn   => hbuIsSyncedIn,

        upstreamOffset    => upstreamOffset,
        validOffset       => valid_offset,
        hbcOffset         => hbcOffset,
        fineOffset        => fineOffset,
        fineOffsetLocal   => fineOffsetLocal,

        -- LACCP Bus --
        pulseTakeOver     => rcap_pulse_takeover,
        probePulseIn      => probe_pulse_in,
        probePulseOut     => probe_pulse_out,

        dataBusIn         => data_bus_out(kPortRCAP),
        validBusIn        => valid_bus_out(kPortRCAP),
        dataBusOut        => data_bus_in(kPortRCAP),
        validBusOut       => valid_bus_in(kPortRCAP),
        isReadyOut        => bus_ready_in(kPortRCAP)

      );


  -- MIKUMARI interface ----------------------------------------------
  pulseInRejected <= pulse_tx_rejected;

  pulseTx         <= reg_pulse_tx;
  pulseTypeTx     <= type_vector_tx(pulseTypeTx'range);
  busy_pulse_tx   <= busyPulseTx;

  syncPulseOut    <= reg_pulse_vector_rx(kPulseHeartbeat) when(rcap_pulse_takeover = '1' and valid_offset = '1') else '0';
  probe_pulse_in  <= reg_pulse_vector_rx(kPulseRcapProbe) when(rcap_pulse_takeover = '1' and valid_offset = '0') else '0';
  laccpPulsesOut  <= (others => '0') when(rcap_pulse_takeover = '1') else reg_pulse_vector_rx;

  merged_pulse_tx(0)  <= syncPulseIn when(rcap_pulse_takeover = '1' and valid_offset = '0') else
                         '0'         when(rcap_pulse_takeover = '1' and valid_offset = '1') else
                         laccpPulsesIn(0);
  merged_pulse_tx(1)  <= probe_pulse_out when(rcap_pulse_takeover = '1' and valid_offset = '0') else
                         '0'             when(rcap_pulse_takeover = '1' and valid_offset = '1') else
                         laccpPulsesIn(1);
  merged_pulse_tx(kNumLaccpPulse-1 downto 2) <= (others => '0') when(rcap_pulse_takeover = '1') else laccpPulsesIn(kNumLaccpPulse-1 downto 2);

  u_pulse_buf : process(clk)
    variable type_vector_rx : std_logic_vector(kNumLaccpPulse-1 downto 0):= (others => '0');
  begin
    if(clk'event and clk = '1') then
      if(mikuLinkUpIn = '1') then
        -- Pulse RX --
        type_vector_rx      := DecodeMikuPulse(pulseTypeRx);
        for i in 0 to kNumLaccpPulse-1 loop
          reg_pulse_vector_rx(i)   <= type_vector_rx(i) and pulseRx;
        end loop;

        -- Pulse Tx --
        if(unsigned(merged_pulse_tx) /= 0 and busy_pulse_tx = '0' and reg_pulse_tx = '0') then
          reg_pulse_tx      <= '1';
          type_vector_tx    <= EncodeMikuPulse(merged_pulse_tx);
        else
          reg_pulse_tx      <= '0';
          type_vector_tx    <= (others => '0');
        end if;

        if(((unsigned(merged_pulse_tx) /= 0 and (busy_pulse_tx = '1' or reg_pulse_tx = '1'))) or type_vector_tx = kMultiPulse) then
          pulse_tx_rejected   <= '1';
        else
          pulse_tx_rejected   <= '0';
        end if;

      end if;
    end if;
  end process;

  u_rx : entity mylib.LaccpFrameRx
    generic map
      (
        enDebug         => enDebug
      )
    port map
      (
        -- System --
        rst             => laccp_reset,
        clk             => clk,

        -- LACCP --
        frameDataOut    => frame_data_rx,
        frameValid      => valid_frame_rx,
        frameInValid    => open,

        -- MIKUMARI-Link --
        dataRx          => dataRx,
        validRx         => validRx,
        frameLastRx     => frameLastRx,
        checkSumErrRx   => checkSumErrRx,
        frameBrokenRx   => frameBrokenRx,
        recvTermndRx    => recvTermndRx

      );

  u_tx : entity mylib.LaccpFrameTx
    generic map
      (
        enDebug         => enDebug
      )
    port map
      (
        -- System --
        rst             => laccp_reset,
        clk             => clk,

        -- LACCP --
        frameDataIn     => frame_data_tx,
        weFrameData     => we_frame_data_tx,

        -- MIKUMARI-Link --
        dataTx          => dataTx,
        validTx         => validTx,
        frameLastTx     => frameLastTx,
        txAck           => txAck

      );

  -- Bus Switch ------------------------------------------------------
  data_bus_in(kPortMikumari)  <= frame_data_rx;
  valid_bus_in(kPortMikumari) <= valid_frame_rx;
  bus_ready_in(kPortMikumari) <= mikuLinkUpIn;
  frame_data_tx     <= data_bus_out(kPortMikumari);
  we_frame_data_tx  <= valid_bus_out(kPortMikumari);

  -- Intra-port connection --
  gen_intra : for i in 0 to kNumExtIntraPort-1 generate
    data_bus_in( i+ kNumIntraPort-kNumExtIntraPort+1)   <= dataIntraIn(i);
    valid_bus_in(i+ kNumIntraPort-kNumExtIntraPort+1)   <= validIntraIn(i);
    bus_ready_in(i+ kNumIntraPort-kNumExtIntraPort+1)   <= isReadyIntraIn(i);
    dataIntraOut(i)    <= data_bus_out( i + kNumIntraPort-kNumExtIntraPort+1);
    validIntraOut(i)   <= valid_bus_out(i + kNumIntraPort-kNumExtIntraPort+1);
  end generate;

  -- Inter-connect --
  gen_inter : for i in 0 to kNumInterconnect-1 generate
    data_bus_in( i + kNumIntraPort+1)  <= dataInterIn(i);
    valid_bus_in(i + kNumIntraPort+1)  <= validInterIn(i);
    bus_ready_in(i + kNumIntraPort+1)  <= isReadyInterIn(i);
    dataInterOut(i)  <= data_bus_out( i + kNumIntraPort+1);
    validInterOut(i) <= valid_bus_out(i + kNumIntraPort+1);
    existInterOut(i) <= '1';
  end generate;

  u_sw : entity mylib.LaccpBusSwitch
    generic map
      (
        kNumInterconnect  => kNumInterconnect,
        enDebug           => false
      )
    port map
      (
        -- System --
        rst             => laccp_reset,
        clk             => clk,

        -- Bus ports --
        dataBusIn       => data_bus_in,
        validBusIn      => valid_bus_in,
        busReadyIn      => bus_ready_in,

        dataBusOut      => data_bus_out,
        validBusOut     => valid_bus_out

      );

end RTL;