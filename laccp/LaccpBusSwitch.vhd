library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library mylib;
use mylib.defBitwiseOp.all;
use mylib.defLaccp.all;

entity LaccpBusSwitch is
  generic
    (
      kNumInterconnect  : integer:= 16;
      enDebug           : boolean:= false
    );
  port
    (
      -- System --
      rst             : in std_logic; -- Asynchronous, Active high
      clk             : in std_logic;

      -- Bus ports --
      dataBusIn       : in LaccpBusDataType;
      validBusIn      : in LaccpBusCtrlType;
      busReadyIn      : in LaccpBusCtrlType;

      dataBusOut      : out LaccpBusDataType;
      validBusOut     : out LaccpBusCtrlType

    );
end LaccpBusSwitch;

architecture RTL of LaccpBusSwitch is
  attribute mark_debug  : boolean;

  -- System --
  signal sync_reset           : std_logic;
  constant kWidthResetSync    : integer:= 16;
  signal reset_shiftreg       : std_logic_vector(kWidthResetSync-1 downto 0);

  -- Internal signal decralation --
  constant kNumBusPorts       : integer:= kNumIntraPort + kNumInterconnect + 1;
  constant kPortInterBegin    : integer:= kNumIntraPort + 1;
  constant kPortInterEnd      : integer:= kNumBusPorts-1;

  -- RX-FIFO --
  signal re_rx_fifo, rd_valid_rx_fifo, empty_rx_fifo  : std_logic_vector(kNumBusPorts-1 downto 0);
  signal dout_rx_fifo         : LaccpBusDataType;

  -- BusSwitch --
  signal en_broadcast         : std_logic;
  signal intra_connect, inter_connect, miku_connect : std_logic;
  signal reg_bus_data         : LaccpFrameBobyType;

  type LaccpBusSwitchProcess is (
    WaitRxIn, ParseFrame, Connect
  );

  signal state_switch  : LaccpBusSwitchProcess;

  -- Debug --
  attribute mark_debug of intra_connect  : signal is enDebug;
  attribute mark_debug of inter_connect  : signal is enDebug;
  attribute mark_debug of miku_connect   : signal is enDebug;
  attribute mark_debug of rd_valid_rx_fifo   : signal is enDebug;
  attribute mark_debug of reg_bus_data   : signal is enDebug;
  attribute mark_debug of state_switch    : signal is enDebug;

begin
  -- =================================================================
  --                           Body
  -- =================================================================

  gen_rx_fifo : for i in 0 to kNumBusPorts -1 generate
  begin
    u_rx_fifo : entity mylib.MyFifoComClock
      generic map(
        kFifoDepth    => 16,
        kWidthData    => dataBusIn(i)'length,
        kProgFullTh   => 14,
        kInferBRAM    => false,
        kRegisterOut  => false
        )
      port map(
        rst       => sync_reset,
        clk       => clk,
        wrEn      => validBusIn(i),
        rdEn      => re_rx_fifo(i),
        rdValid   => rd_valid_rx_fifo(i),
        emptyOut  => empty_rx_fifo(i),
        fullOut   => open,
        pfullOut  => open,
        dIn       => dataBusIn(i),
        dOut      => dout_rx_fifo(i)
        );
  end generate;

  -- Bus Switcher ------------------------------------------------------
  u_switch : process(clk, sync_reset)
    variable index_rx     : integer range -1 to kNumBusPorts:= 0;
    variable index_intra  : integer range -1 to kNumBusPorts:= 0;
    variable tmp_vect     : std_logic_vector(re_rx_fifo'range);
  begin
    if(sync_reset = '1') then
      intra_connect   <= '0';
      en_broadcast    <= '0';
      inter_connect   <= '0';
      miku_connect    <= '0';
      validBusOut     <= (others => '0');
      re_rx_fifo      <= (others => '0');

      state_switch              <= WaitRxIn;

    elsif(clk'event and clk = '1') then
    case state_switch is
      when WaitRxIn =>
        intra_connect   <= '0';
        en_broadcast    <= '0';
        inter_connect   <= '0';
        miku_connect    <= '0';

        validBusOut     <= (others => '0');
        re_rx_fifo      <= (others => '0');

        if(to_integer(unsigned(not empty_rx_fifo)) /= 0) then
          re_rx_fifo    <= IsolateRMHB(not empty_rx_fifo);
          tmp_vect      := IsolateRMHB(not empty_rx_fifo);
          index_rx      := GetBitIndex(tmp_vect);
          state_switch  <= ParseFrame;
        end if;

      when ParseFrame =>
        re_rx_fifo    <= (others => '0');

        if(rd_valid_rx_fifo(index_rx) = '1') then
          reg_bus_data   <= dout_rx_fifo(index_rx);

          -- Resoluve intra port --
          if(kBroadCast = dout_rx_fifo(index_rx)(kPosDestModAddr'range)) then
            intra_connect   <= '1';
            en_broadcast    <= '1';
          elsif(index_rx /= GetIntraIndex(dout_rx_fifo(index_rx)(kPosDestModAddr'range))) then
            intra_connect   <= '1';
            index_intra     := GetIntraIndex(dout_rx_fifo(index_rx)(kPosDestModAddr'range));
          else
            intra_connect   <= '0';
          end if;

          -- Passage permission --
          if(false = CheckInterPortRange(index_rx) and
             '1' = dout_rx_fifo(index_rx)(kPosCmd'low + kCmdPassagePermission)) then
            inter_connect   <= '1';
          else
            inter_connect   <= '0';
          end if;

          -- Allow departure --
          if(index_rx /= kPortMikumari and
            '1' = dout_rx_fifo(index_rx)(kPosCmd'low + kCmdDepature)) then
              miku_connect  <= '1';
          else
              miku_connect  <= '0';
          end if;

          state_switch <= Connect;
        end if;

      when Connect =>
        if(intra_connect = '1') then
          if(en_broadcast = '1') then
            for i in 0 to kNumIntraPort-1 loop
              dataBusOut(i + kPortRLIGP)   <= reg_bus_data;
              validBusOut(i + kPortRLIGP)  <= busReadyIn(i + kPortRLIGP);
            end loop;
          else
            dataBusOut(index_intra)   <= reg_bus_data;
            validBusOut(index_intra)  <= busReadyIn(index_intra);
          end if;
        end if;

        if(inter_connect  = '1') then
          for i in kPortInterBegin to kPortInterEnd loop
            dataBusOut(i)   <= reg_bus_data;
            validBusOut(i)  <= busReadyIn(i);
          end loop;
        end if;

        if(miku_connect = '1') then
          dataBusOut(kPortMikumari)   <= reg_bus_data;
          validBusOut(kPortMikumari)  <= busReadyIn(kPortMikumari);
        end if;

        state_switch   <= WaitRxIn;

      when others =>
        state_switch      <= WaitRxIn;

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