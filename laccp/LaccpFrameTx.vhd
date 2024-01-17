library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library mylib;
use mylib.defLaccp.all;
use mylib.defMikumari.all;
use mylib.defCDCM.all;

entity LaccpFrameTx is
  generic
    (
      enDebug         : boolean:= false
    );
  port
    (
      -- System --
      rst             : in std_logic; -- Asynchronous, Active high
      clk             : in std_logic;

      -- LACCP --
      frameDataIn     : in LaccpFrameBobyType;
      weFrameData     : in std_logic;

      -- MIKUMARI-Link --
      dataTx          : out CbtUDataType;
      validTx         : out std_logic;
      frameLastTx     : out std_logic;
      txAck           : in std_logic

      --pulseRx         : in std_logic;
      --pulseTypeRx     : in MikumariPulseType

    );
end LaccpFrameTx;

architecture RTL of LaccpFrameTx is
  attribute mark_debug  : boolean;

  -- System --
  signal sync_reset           : std_logic;
  constant kWidthResetSync    : integer:= 16;
  signal reset_shiftreg       : std_logic_vector(kWidthResetSync-1 downto 0);

  -- Internal signal decralation --
  type LaccpFrameTxType is array (kLaccpFrameLength-1 downto 0) of std_logic_vector(kFramePreamble'range);
  signal frame_data           : LaccpFrameTxType;
  constant kRegLast           : std_logic_vector(kLaccpFrameLength-1 downto 0):= std_logic_vector(to_unsigned(1, kLaccpFrameLength));

  -- FIFO --
  signal re_tx_fifo, rd_valid_tx_fifo, empty_tx_fifo  : std_logic;
  signal dout_tx_fifo         : std_logic_vector(frameDataIn'range);

  type FrameTransProcessType is  (
    WaitTxIn, SetFrameData, SendFrameData
  );
  signal state_tx             : FrameTransProcessType;

  -- Debug --
  attribute mark_debug of state_tx             : signal is enDebug;

begin
  -- =================================================================
  --                           Body
  -- =================================================================


  u_tx_fifo : entity mylib.MyFifoComClock
  generic map(
    kFifoDepth    => 16,
    kWidthData    => frameDataIn'length,
    kProgFullTh   => 14,
    kInferBRAM    => false,
    kRegisterOut  => false
    )
  port map(
    rst       => sync_reset,
    clk       => clk,
    wrEn      => weFrameData,
    rdEn      => re_tx_fifo,
    rdValid   => rd_valid_tx_fifo,
    emptyOut  => empty_tx_fifo,
    fullOut   => open,
    pfullOut  => open,
    dIn       => frameDataIn,
    dOut      => dout_tx_fifo
    );

  -- Frame TX --------------------------------------------------------
  u_frame_tx : process(clk, sync_reset)
    variable index : integer range -1 to kLaccpFrameLength+1;
  begin
    if(sync_reset = '1') then
      re_tx_fifo    <= '0';
      validTx       <= '0';
      frameLastTx   <= '0';
      index         := kLaccpFrameLength-1;
      state_tx      <= WaitTxIn;

    elsif(clk'event and clk = '1') then
    case state_tx is
      when WaitTxIn =>
        validTx         <= '0';
        frameLastTx     <= '0';
        if(empty_tx_fifo = '0') then
          re_tx_fifo  <= '1';
          state_tx    <= SetFrameData;
        end if;

      when SetFrameData =>
        re_tx_fifo    <= '0';
        if(rd_valid_tx_fifo = '1') then
          frame_data(kLaccpFrameLength-1)  <= kFramePreamble;
          for i in 0 to kLaccpFrameLength-2 loop
            frame_data(i)   <= dout_tx_fifo((i+1)*dataTx'length-1 downto i*dataTx'length);
          end loop;

          index     := kLaccpFrameLength-1;
          state_tx  <= SendFrameData;
        end if;

      when SendFrameData =>
        if(txAck = '1') then
          if(index = 0) then
            validTx   <= '0';
            state_tx  <= WaitTxIn;
          else
            index   := index -1;
          end if;
        else
          validTx <= '1';
          frameLastTx   <= kRegLast(index);
          dataTx        <= frame_data(index);
        end if;

      when others =>
        state_tx      <= WaitTxIn;

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