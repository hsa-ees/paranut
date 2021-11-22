--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2013  Michael Seider, <michael.seider@hs-augsburg.de>
-- 		 Hochschule Augsburg, University of Applied Sciences
--
-- Redistribution and use in source and binary forms, with or without modification,
-- are permitted provided that the following conditions are met:
--
-- 1. Redistributions of source code must retain the above copyright notice, this 
--    list of conditions and the following disclaimer.
--
-- 2. Redistributions in binary form must reproduce the above copyright notice,
--    this list of conditions and the following disclaimer in the documentation and/or
--    other materials provided with the distribution.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
-- WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
-- DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
-- ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
-- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
-- LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
-- ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
-- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
-- SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-- Description:
--   This is the arbiter for the MEMU. See description below for details.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.paranut_lib.all;
use paranut.lfsr.all;
    

entity selector is 
    generic (
                DWIDTH         : integer := 1; -- Width of the data that gets arbitrated
                SEL_MAX        : integer := 1; -- Maximum value for sel (usually RPORTS+WPORTS)
                FAST_INDEX     : integer := 0  -- If prio < FAST_INDEX the fast inputs have priority else the slow inputs
            );
    
    port (
             clk            : in std_logic;
             reset          : in std_logic;
             f_dat_in       : in std_logic_vector(DWIDTH-1 downto 0);     
             f_sel_in       : in integer range 0 to SEL_MAX;          
             f_sel_valid_in : in std_logic;
             s_dat_in       : in std_logic_vector(DWIDTH-1 downto 0); 
             s_sel_in       : in integer range 0 to SEL_MAX;        
             s_sel_valid_in : in std_logic;
             prio           : in unsigned(MAX(0, CFG_NUT_CPU_CORES_LD-1) downto 0);
             --hold           : in std_logic;
             dat_out        : out std_logic_vector(DWIDTH-1 downto 0); 
             sel_out        : out integer range 0 to SEL_MAX;     
             sel_valid_out  : out std_logic   
         );

end selector;

architecture rtl of selector is
    type registers is record
        s_dat_in : std_logic_vector(DWIDTH-1 downto 0);
        s_sel_in : integer range 0 to SEL_MAX;  
        s_sel_valid_in : std_logic;
        --last_sel : std_logic;       
    end record;
    
    signal r, rin : registers;
begin
    -- Combinatorial process
    comb : process (reset, r, f_dat_in, f_sel_in, f_sel_valid_in, s_sel_in, s_sel_valid_in, prio) --, hold)
        variable v : registers;
        variable sel : std_logic;
        variable slow_sel_valid : std_logic; 
    begin
        v := r;
        
        -- Set defaults...
        --sel := r.last_sel;
        if (r.s_sel_in = s_sel_in) then
          slow_sel_valid := (r.s_sel_valid_in and s_sel_valid_in);  
        else 
          slow_sel_valid := '0';
        end if;
              
        
        --if (hold = '0') then
        v.s_dat_in := s_dat_in;
        v.s_sel_in := s_sel_in;
        v.s_sel_valid_in := s_sel_valid_in;
        
        if (prio <= FAST_INDEX) then
            -- Fast input has priority 
            if (f_sel_valid_in = '1') then
                 sel := '0';
            else 
                 sel := '1';
            end if;
        else 
            -- Slow input has priority
            if (slow_sel_valid = '1') then
                 sel := '1';
            else 
                 sel := '0';
            end if;
        end if;
        -- Save last selected output
        --v.last_sel := sel;
        --end if;

        -- Write results...  
        rin <= v;
        
        case sel is 
            when '0' => dat_out <= f_dat_in; sel_out <= f_sel_in; 
            when others => dat_out <= r.s_dat_in; sel_out <= r.s_sel_in;
        end case;
        
        sel_valid_out <= slow_sel_valid or f_sel_valid_in; 

    end process;
    
   
    -- Register process
    process (clk)
    begin
       if (clk'event and clk = '1') then
            if (reset = '1') then
                --r.last_sel <= '0';
                r.s_sel_valid_in <= '0';
            else
                r <= rin;
            end if;
       end if;
    end process;

end rtl;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.paranut_lib.all;
use paranut.lfsr.all;

entity marbiter is
    --generic (
    --            CFG_MEMU_CACHE_BANKS : integer := 1;
    --            RPORTS      : integer := 1;
    --            WPORTS      : integer := 1
    --        );
    port (
             clk            : in std_logic;
             reset          : in std_logic;
             ai             : in arbiter_in_type;
             ao             : out arbiter_out_type;
             bifai          : out busif_arbiter_in_type;
             bifao          : in busif_arbiter_out_type;
             rpai           : out readport_arbiter_in_vector(0 to RPORTS-1);
             rpao           : in readport_arbiter_out_vector(0 to RPORTS-1);
             wpai           : out writeport_arbiter_in_vector(0 to WPORTS-1);
             wpao           : in writeport_arbiter_out_vector(0 to WPORTS-1)
         );
end marbiter;

architecture rtl of marbiter is
    component selector is 
        generic (
                DWIDTH         : integer := 1; -- Width of the data that gets arbitrated
                SEL_MAX        : integer := 1; -- Maximum value for sel (usually RPORTS+WPORTS)
                FAST_INDEX     : integer := 0  -- If prio < FAST_INDEX the fast inputs have priority else the slow inputs
        );
    
        port (
             clk            : in std_logic;
             reset          : in std_logic;
             f_dat_in       : in std_logic_vector(DWIDTH-1 downto 0);     
             f_sel_in       : in integer range 0 to SEL_MAX;          
             f_sel_valid_in : in std_logic;
             s_dat_in       : in std_logic_vector(DWIDTH-1 downto 0); 
             s_sel_in       : in integer range 0 to SEL_MAX;        
             s_sel_valid_in : in std_logic;
             prio           : in unsigned(MAX(0, CFG_NUT_CPU_CORES_LD-1) downto 0);
             --hold           : in std_logic;
             dat_out        : out std_logic_vector(DWIDTH-1 downto 0); 
             sel_out        : out integer range 0 to SEL_MAX;     
             sel_valid_out  : out std_logic   
        );
    end component;
    
    -- Line lock...
    -- - Implements mutex for a single cache line (tag + all data banks).
    -- - Writers are supposed to acquire a line lock.
    -- - Readers do not acquire anything => Writers must perform their actions in a safe order.
    --  - if a line lock request is held, the associated address must not change
    --  - A line lock may be released during the clock cycle that the last bank/tag access is made,
    --    given that the bank/tag lock is still held. This allows faster write cycles without write port
    --    monopolizing a line lock (keeping it permanently up).

    -- Tag RAM...
    -- - Tag RAM must have CFG_NUT_CPU_CORES ports.
    -- - Each port is reserved for one CPU (three ports: RP #n, RP $CFG_NUT_CPU_CORES+n, WP #n).
    -- - BUSIF uses port #CFG_NUT_CPU_CORES-1.
    -- - Arbiter provides:
    --      a) Write-/Read-Lock (a write-lock excludes all readers)
    --      b) Port arbitration: Multiple readers are allowed, but only one per port (see above)
    --      c) Priority selection among a port: 0. BUSIF (if applicable), 1. Data read, 2. Insn Read, 3. Data Write
    --      EXAMINE: Would two ports/CPU bring speed improvement? Would a separate port for BUSIF bring speed improvement?
    --  - The tag RAM arbitration is also used to prevent writers from replacing/changing a cache line
    --      while it is read during a cache read hit. Hence, a reader must keep its 'tagr' lock until its bank access
    --      is completed, too. This is not necessary, if a line lock is held (i.e. for cache line reading during a write miss).

    -- Bank RAMs...
    -- - All ports of CPU n must be linked to port n % CFG_MEMU_BANK_RAM_PORTS of each bank RAM.
    -- - The BUSIF is linked to port #(CFG_MEMU_BANK_RAM_PORTS-1).
    -- - Multiple usually conflicting grant signals may be set, if the addresses match.
    -- - As long as a request signal is set, the address must not chage!
    -- - Among write ports and the BUSIF, only one grant will be given to avoid writing conflicts.

    -- BUSIF

    -- select/routing information can be derived from grant lines (only one is set at a time)

    -- Note on deadlock prevention:
    -- 1. Requests/grants must always be in the following order (-> break cyclic wait condition):
    --      busif < linelock < tagr/tagw < bank
    -- 2. (corollary to 1.) R/W ports must not request anything when waiting for BusIF (i.e. 'busif_busy')
    -- 3. tag/bank access may never be requested in a hold-and-wait manner: Either request simultaneously or use & complete serially.

    -- Mappings for the bank ports to the 'req_bank'/'gnt_bank' bus ...
    constant SINGLE_CPU : integer := 1 / CFG_NUT_CPU_CORES; -- TBD: is this really the only way to generate a 1 for CFG_NUT_CPU_CORES_LD == 0 
    constant NOT_SINGLE_CPU : integer := CONDITIONAL(CFG_NUT_CPU_CORES > 1, 1, 0);
    constant IDX_RP_OFF : integer := 0;             -- data read ports
    constant IDX_IP_OFF : integer := 1;             -- insn read ports
    constant IDX_WP_OFF : integer := 2;             -- write ports
    constant IDX_BUSIF  : integer := (RPORTS+WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS; -- BusIF port
    constant RAMPORT_BUSIF : integer := CFG_MEMU_BANK_RAM_PORTS-1;

    constant COUNTER_INIT : std_logic_vector(15 downto 0) := (others => '1');
    constant COUNTER_POLY : std_logic_vector(15 downto 0) := get_prime_poly(16, 0);

    subtype read_req_gnt_type is std_logic_vector(0 to RPORTS+WPORTS);
    type read_req_gnt_vector is array (natural range <>) of read_req_gnt_type;
    
    subtype bank_gnt_type is std_logic_vector(0 to (RPORTS+WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS);
    type bank_gnt_vector is array (natural range <>) of bank_gnt_type;
    type bank_gnt_vector_2 is array (natural range <>) of bank_gnt_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);

    type registers is record
        counter : unsigned(MAX(0, CFG_NUT_CPU_CORES_LD+CFG_MEMU_ARBITER_METHOD-1) downto 0);
        lfsr_counter : std_logic_vector(15 downto 0);
        linelock : std_logic_vector(0 to WPORTS);
        tagr : read_req_gnt_type;
        req_tagw : std_logic_vector(0 to WPORTS);
        tagw : std_logic_vector(0 to WPORTS);
        bank : bank_gnt_vector_2(0 to CFG_MEMU_CACHE_BANKS-1);
        busif : std_logic_vector(0 to RPORTS+WPORTS-1);
    end record;

    function "and" (op1 : bank_gnt_vector; op2 : bank_gnt_vector) return bank_gnt_vector is
        variable vec_out : bank_gnt_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);
    begin
        -- result is and of subvectors
        for i in 0 to CFG_MEMU_BANK_RAM_PORTS-1 loop
            vec_out(i) := op1(i) and op2(i);
        end loop;
        return (vec_out);
    end;

    function get_prio_cpu (r : registers) return integer is
        variable index : unsigned(MAX(0, CFG_NUT_CPU_CORES_LD-1) downto 0);
    begin
        if (CFG_NUT_CPU_CORES_LD > 0) then
            if (CFG_MEMU_ARBITER_METHOD >= 0) then
                index := r.counter(CFG_MEMU_ARBITER_METHOD+CFG_NUT_CPU_CORES_LD-1 downto CFG_MEMU_ARBITER_METHOD);
            else
                index := unsigned(r.lfsr_counter(CFG_NUT_CPU_CORES_LD-1 downto 0));
            end if;
        else
            index := "0";
        end if;
        return (conv_integer(index));
    end;
    
    function ram_port (i : integer) return integer is
        variable index : integer range 0 to CFG_MEMU_BANK_RAM_PORTS-1;
    begin
        index := CFG_MEMU_BANK_RAM_PORTS-1;
        if (i /= IDX_BUSIF) then
            index := (i mod CFG_NUT_CPU_CORES) mod CFG_MEMU_BANK_RAM_PORTS;
        end if;
        return (index);
    end;
    
  
    function ram_index (i : integer) return integer is
        variable index : integer range 0 to (RPORTS+WPORTS)/CFG_MEMU_BANK_RAM_PORTS;
        variable slv : std_logic_vector((log2x(RPORTS+WPORTS)+CFG_MEMU_BANK_RAM_PORTS-1)/CFG_MEMU_BANK_RAM_PORTS downto 0); -- (M+N-1)/N to ceil 
    begin
        if CFG_MEMU_BANK_RAM_PORTS = 2 then
            -- remove rightmost bit
            slv := std_logic_vector(to_unsigned(i, slv'length));
            index := to_integer(unsigned(slv(slv'left downto 1))); 
            -- index * 3 (2 RP + 1 WP) is final index
            index := index*3;
        else 
            index := i*3;
        end if;
        return (index);
    end;
        
    function cpu_index (i : integer) return integer is
        variable index : integer range 0 to (CFG_NUT_CPU_CORES/CFG_MEMU_BANK_RAM_PORTS)-1;
        variable slv : std_logic_vector((CFG_NUT_CPU_CORES_LD+CFG_MEMU_BANK_RAM_PORTS-1)/CFG_MEMU_BANK_RAM_PORTS downto 0);  -- (M+N-1)/N to ceil
    begin
        if CFG_MEMU_BANK_RAM_PORTS = 2 then
            -- remove rightmost bit
            slv := std_logic_vector(to_unsigned(i, slv'length));
            index := to_integer(unsigned(slv(slv'left downto 1))); -- (M+N-1)/N to ceil
        else 
            index := i;
        end if;
        return (index);
    end;
    
    signal r, rin : registers;
    
    -- Bank Arbitration Types and Signals
    -- --------------------------------------
    -- per Bank and Port inputs
    type bank_sel_3d_type is array (natural range <>, natural range <>, natural range <>) of integer range 0 to (RPORTS+WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS; 
    type bank_sel_valid_3d_type is array (natural range <>, natural range <>, natural range <>) of std_logic;
    type bank_sel_wiadr_3d_type is array (natural range <>, natural range <>, natural range <>) of way_index_addr_type; 
    signal bank_sel_in : bank_sel_3d_type (0 to CFG_MEMU_CACHE_BANKS-1, 0 to CFG_MEMU_BANK_RAM_PORTS-1, 0 to 1);     
    signal bank_sel_valid_in : bank_sel_valid_3d_type (0 to CFG_MEMU_CACHE_BANKS-1, 0 to CFG_MEMU_BANK_RAM_PORTS-1, 0 to 1);     
    signal bank_wiadr_in : bank_sel_wiadr_3d_type (0 to CFG_MEMU_CACHE_BANKS-1, 0 to CFG_MEMU_BANK_RAM_PORTS-1, 0 to 1);     
    
    -- per Bank and Port outputs
    type bank_sel_2d_type is array (natural range <>, natural range <>) of integer range 0 to (RPORTS+WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS; 
    type bank_sel_valid_2d_type is array (natural range <>, natural range <>) of std_logic;
    type bank_sel_wiadr_2d_type is array (natural range <>, natural range <>) of way_index_addr_type; 
    signal bank_sel_out : bank_sel_2d_type (0 to CFG_MEMU_CACHE_BANKS-1, 0 to CFG_MEMU_BANK_RAM_PORTS-1);     
    signal bank_sel_valid_out : bank_sel_valid_2d_type (0 to CFG_MEMU_CACHE_BANKS-1, 0 to CFG_MEMU_BANK_RAM_PORTS-1);     
    signal bank_wiadr_out : bank_sel_wiadr_2d_type (0 to CFG_MEMU_CACHE_BANKS-1, 0 to CFG_MEMU_BANK_RAM_PORTS-1);     
       
      
    -- BusIf Arbitration Types and Signals
    -- --------------------------------------
    -- per CPU select and select_valid (request) inputs...
    type busif_sel_type is array (natural range <>) of integer range 0 to (RPORTS+WPORTS); 
    type busif_sel_valid_type is array (natural range <>) of std_logic;
    signal busif_sel_in : busif_sel_type (0 to 1);    
    signal busif_sel_valid_in : busif_sel_valid_type (0 to 1); 
  
    -- per Arbitration step sel output
    signal busif_sel_out : integer range 0 to (RPORTS+WPORTS);
    signal busif_sel_valid_out : std_logic;
  
    
    -- Linelock Arbitration Types and Signals
    -- --------------------------------------
    -- per WPORT select and select_valid (request) inputs...
    type linelock_sel_type is array (natural range <>) of integer range 0 to (WPORTS-1); 
    type linelock_sel_valid_type is array (natural range <>) of std_logic;
    signal linelock_sel_in : linelock_sel_type (0 to 1);    
    signal linelock_sel_valid_in : linelock_sel_valid_type (0 to 1); 
  
    -- per Arbitration step sel output
    signal linelock_sel_out : integer range 0 to (WPORTS-1);
    signal linelock_sel_valid_out : std_logic;
  
    
    signal cpu_prio : unsigned(MAX(0, CFG_NUT_CPU_CORES_LD-1) downto 0);
    
    constant BANK_SEL_STEPS : natural := MAX(0, CFG_NUT_CPU_CORES/CFG_MEMU_BANK_RAM_PORTS-2);
begin
    
    -- Generate the Arbitration steps for the Bank Arbitration
    -- ------------------------------------------------------------
    Banks : for b in 0 to CFG_MEMU_CACHE_BANKS-1 generate
      BankBusIfSel: if (CFG_NUT_CPU_CORES > 1 or CFG_MEMU_BANK_RAM_PORTS = 1) generate -- Needed if we only have 1 BRAM port or more than 1 CPU 
          -- BusIf gets fastest grants
          BankBusIfSel : selector 
          generic map ( 
                       DWIDTH  => CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, -- way_index_addr_type
                       SEL_MAX => (RPORTS+WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS,
                       FAST_INDEX => CFG_NUT_CPU_CORES
                   )
          port map (
                    clk => clk,
                    reset => reset,
                    f_dat_in => bank_wiadr_in(b, RAMPORT_BUSIF, 0), 
                    f_sel_in => bank_sel_in(b, RAMPORT_BUSIF, 0),           
                    f_sel_valid_in => bank_sel_valid_in(b, RAMPORT_BUSIF, 0),
                    s_dat_in => bank_wiadr_in(b, RAMPORT_BUSIF, 1),     
                    s_sel_in => bank_sel_in(b, RAMPORT_BUSIF, 1),         
                    s_sel_valid_in => bank_sel_valid_in(b, RAMPORT_BUSIF, 1),      
                    prio => cpu_prio,
                    --hold => '0', -- No need to hold for bank arbitration (all accesses take 1 cycle)
                    dat_out => bank_wiadr_out(b, RAMPORT_BUSIF),  
                    sel_out => bank_sel_out(b, RAMPORT_BUSIF), 
                    sel_valid_out => bank_sel_valid_out(b, RAMPORT_BUSIF)        
                );
      end generate; -- BankBusIfSel
      -- If we have only 1 CPU with more than 1 BRAM port the BusIf gets exclusive access through the second BRAM port...
      BankBusIfPassthrough: if (CFG_NUT_CPU_CORES = 1 and CFG_MEMU_BANK_RAM_PORTS > 1) generate
             -- Passthrough has BusIf as input     
             bank_wiadr_out(b, RAMPORT_BUSIF) <= bank_wiadr_in(b, RAMPORT_BUSIF, 0);
             bank_sel_out(b, RAMPORT_BUSIF) <= bank_sel_in(b, RAMPORT_BUSIF, 0);
             bank_sel_valid_out(b, RAMPORT_BUSIF)   <= bank_sel_valid_in(b, RAMPORT_BUSIF, 0);
      end generate; -- BankBusIfPassthrough
      
      BankCPUSel: if (CFG_MEMU_BANK_RAM_PORTS > 1 and CFG_NUT_CPU_CORES >= 4) generate
        -- CePU gets fastest grants
        BankCPUSel : selector        
        generic map ( 
                     DWIDTH  => CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, -- way_index_addr_type
                     SEL_MAX => (RPORTS+WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS,
                     FAST_INDEX => 0
                 )
        port map (
                  clk => clk,
                  reset => reset,
                  f_dat_in => bank_wiadr_in(b, 0, 0), 
                  f_sel_in => bank_sel_in(b, 0, 0),           
                  f_sel_valid_in => bank_sel_valid_in(b, 0, 0),
                  s_dat_in => bank_wiadr_in(b, 0, 1),     
                  s_sel_in => bank_sel_in(b, 0, 1),         
                  s_sel_valid_in => bank_sel_valid_in(b, 0, 1),      
                  prio => cpu_prio,
                  --hold => '0', -- No need to hold for bank arbitration (all accesses take 1 cycle)
                  dat_out => bank_wiadr_out(b, 0),  
                  sel_out => bank_sel_out(b, 0), 
                  sel_valid_out => bank_sel_valid_out(b, 0)        
              ); 
      end generate; -- BankCPUSel
      -- If we have less than 4 CPUs with more than 1 BRAM port the CePU gets exclusive access through the first BRAM port...
      BankCPUPassthrough: if (CFG_MEMU_BANK_RAM_PORTS > 1 and CFG_NUT_CPU_CORES < 4) generate
             -- Passthrough has CePU as input     
             bank_wiadr_out(b, 0) <= bank_wiadr_in(b, 0, 0);
             bank_sel_out(b, 0) <= bank_sel_in(b, 0, 0);
             bank_sel_valid_out(b, 0)   <= bank_sel_valid_in(b, 0, 0);
      end generate; -- BankCPUPassthrough  
    end generate; -- Banks
  
    
    -- Generate the Arbitration steps for the BusIf Arbitration
    -- ------------------------------------------------------------
    BusIfSel : if (CFG_NUT_CPU_CORES >= 2) generate 
       -- The step has the CePU and all other CoPUs as input
       BusIfSel : selector 
       generic map ( 
                       DWIDTH  => 1,
                       SEL_MAX => (RPORTS+WPORTS+1),
                       FAST_INDEX => 0
                   )
       port map (
                    clk => clk,
                    reset => reset,
                    f_dat_in => "0", 
                    f_sel_in => busif_sel_in(0),          
                    f_sel_valid_in => busif_sel_valid_in(0),
                    s_dat_in => "0",
                    s_sel_in => busif_sel_in(1),      
                    s_sel_valid_in => busif_sel_valid_in(1),      
                    prio => cpu_prio,
                    --hold => busif_step_hold,
                    dat_out => open, 
                    sel_out => busif_sel_out, 
                    sel_valid_out => busif_sel_valid_out         
                );
    end generate; -- LastStep
    -- If there is only 1 CPU we do not need any arbitration, just passthrough input signals...
    BusIfPassthrough : if (CFG_NUT_CPU_CORES < 2) generate 
       -- Passthrough has 1 CPU as input     
       busif_sel_out <= busif_sel_in(0); 
       busif_sel_valid_out <= busif_sel_valid_in(0);
    end generate; -- LastStep
    
    
    -- Generate the Arbitration steps for the LineLock Arbitration
    -- ------------------------------------------------------------
    LineLockSel : if (CFG_NUT_CPU_CORES >= 2) generate 
        -- The step has the CePU WPORT and all other CoPUs WPORTs as input
        LineLockSel : selector 
        generic map ( 
                        DWIDTH  => 1,
                        SEL_MAX => WPORTS,
                        FAST_INDEX => 0
                    )
        port map (
                     clk => clk,
                     reset => reset,
                     f_dat_in => "0", 
                     f_sel_in => linelock_sel_in(0),          
                     f_sel_valid_in => linelock_sel_valid_in(0),
                     s_dat_in => "0",
                     s_sel_in => linelock_sel_in(1),      
                     s_sel_valid_in => linelock_sel_valid_in(1),      
                     prio => cpu_prio,
                     --hold => linelock_step_hold,
                     dat_out => open, 
                     sel_out => linelock_sel_out, 
                     sel_valid_out => linelock_sel_valid_out         
                 );
   end generate; -- LastStep
   -- If there is only 1 CPU we do not need any arbitration, just passthrough input signals...
   LineLockPassthrough : if (CFG_NUT_CPU_CORES < 2) generate 
        -- Passthrough has 1 CPU as input     
        linelock_sel_out <= linelock_sel_in(0);
        linelock_sel_valid_out <= linelock_sel_valid_in(0);
   end generate; -- LastStep

         
    comb : process (reset, r, bifao, rpao, wpao, ai, bank_wiadr_out, bank_sel_out, bank_sel_valid_out, busif_sel_out, busif_sel_valid_out, linelock_sel_out, linelock_sel_valid_out)
        variable v : registers;
        -- LineLockMethod
        variable req_linelock, gnt_linelock : std_logic_vector(0 to WPORTS);
        variable linelock_wport_sel : linelock_sel_type(0 to WPORTS-1-NOT_SINGLE_CPU); -- -2 because these are without CePU 
        variable linelock_wport_sel_valid : linelock_sel_valid_type(0 to WPORTS-1-NOT_SINGLE_CPU);
        -- TagMethod
        variable req_tagr, gnt_tagr : read_req_gnt_type;
        variable req_tagw, gnt_tagw : std_logic_vector(0 to WPORTS);
        -- BankMethod
        variable req_bank, gnt_bank : bank_gnt_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);
        variable sel_wiadr : way_index_addr_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);
        variable sel_cpu_wiadr : way_index_addr_type;
        constant BANK_CPU_NUM : integer := CFG_NUT_CPU_CORES/CFG_MEMU_BANK_RAM_PORTS - NOT_SINGLE_CPU;
--        variable wiadr : way_index_addr_2d_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1, 0 to (RPORTS+WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS);
        variable bank_cpu_wiadr_in : bank_sel_wiadr_2d_type(0 to CFG_MEMU_BANK_RAM_PORTS-1, 0 to BANK_CPU_NUM);
        variable bank_cpu_sel_in : bank_sel_2d_type(0 to CFG_MEMU_BANK_RAM_PORTS-1, 0 to BANK_CPU_NUM);
        variable bank_cpu_sel_valid_in : bank_sel_valid_2d_type(0 to CFG_MEMU_BANK_RAM_PORTS-1, 0 to BANK_CPU_NUM);
--        type sel_port_type is array (natural range <>) of integer range 0 to (RPORTS+WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS;
--        variable sel_port : sel_port_type (0 to CFG_NUT_CPU_CORES-1);         
        variable sel_port : integer range 0 to (RPORTS+WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS;         
--        variable sel_port_valid : std_logic_vector(0 to CFG_NUT_CPU_CORES-1); 
        variable sel_port_valid : std_logic; 
--        type sel_br_type is array (natural range <>) of integer range 0 to CFG_NUT_CPU_CORES-1;
--        variable sel_br   : sel_port_type(0 to CFG_MEMU_BANK_RAM_PORTS-1);
--        variable sel_br_valid   : std_logic_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);
        -- BusifMethod
        variable req_busif, gnt_busif : std_logic_vector(0 to RPORTS+WPORTS-1);
        variable busif_cpu_sel : busif_sel_type(0 to CFG_NUT_CPU_CORES-1-NOT_SINGLE_CPU); -- -2 because these are without CePU
        variable busif_cpu_sel_valid : busif_sel_valid_type(0 to CFG_NUT_CPU_CORES-1-NOT_SINGLE_CPU); 
        variable busif_sel : integer range 0 to (RPORTS+WPORTS);
        variable cpu_sel : integer range 0 to (RPORTS+WPORTS);
--        variable cpu_sel : busif_sel_type(0 to CFG_NUT_CPU_CORES-1);
        variable cpu_sel_valid : std_logic;
        variable i : integer range 0 to (RPORTS+WPORTS)/CFG_MEMU_BANK_RAM_PORTS;
        -- SnoopMethod
        variable write : boolean;
        variable writer : integer range 0 to WPORTS-1;
    begin

        v := r;
        
        cpu_prio <= to_unsigned(get_prio_cpu(r) mod CFG_NUT_CPU_CORES, cpu_prio'length);

        --------------------------------------------------------------------------------------
        -- LineLockMethod
        -- Current policy (to save area):
        -- - WPORT requests always exclude each other, independent of the index address
        -- - concurrent BUSIF and WPORT grants are possible, if they address different lines
        --------------------------------------------------------------------------------------

        -- Collect all request signals...
        req_linelock(WPORTS) := bifao.req_linelock;
        for n in 0 to WPORTS-1 loop
            req_linelock(n) := wpao(n).req_linelock;
        end loop;
        -- Determine existing & to-keep grants...
        gnt_linelock := r.linelock and req_linelock;
        
        -- Handle BUSIF request (highest priority)...
        if (req_linelock(WPORTS) = '1' and gnt_linelock(WPORTS) = '0') then
            gnt_linelock(WPORTS) := '1';
            for n in 0 to WPORTS-1 loop
                if (gnt_linelock(n) = '1' and ai.adr_wp(n)(INDEX_OF_ADDR_RANGE) = ai.wiadr_busif(INDEX_OF_WAY_INDEX_RANGE)) then
                    gnt_linelock(WPORTS) := '0';
                end if;
            end loop;
        end if;

        -- Set selector inputs for CePU...
        linelock_sel_in(0) <= 0;
        if (req_linelock(0) = '1' and (gnt_linelock(WPORTS) = '0' or ai.adr_wp(0)(INDEX_OF_ADDR_RANGE) /= ai.wiadr_busif(INDEX_OF_WAY_INDEX_RANGE))) then
            linelock_sel_valid_in(0) <= '1';
        else 
            linelock_sel_valid_in(0) <= '0';
        end if;
        -- Set local inputs for slow selector input
        for n in 1 to WPORTS-1 loop
            linelock_wport_sel(n-1) := n;
            -- Make sure to only request a grant if the cache line is different from the BusIf line...
            if (req_linelock(n) = '1' and (gnt_linelock(WPORTS) = '0' or ai.adr_wp(n)(INDEX_OF_ADDR_RANGE) /= ai.wiadr_busif(INDEX_OF_WAY_INDEX_RANGE))) then
                linelock_wport_sel_valid(n-1) := '1';
            else 
                linelock_wport_sel_valid(n-1) := '0';
            end if;
        end loop;
        
        -- Select slow input...
        linelock_sel_in(1) <= linelock_wport_sel(0);
        linelock_sel_valid_in(1) <= linelock_wport_sel_valid(0);
        for n in 0 to WPORTS-2 loop
--            i := (n + get_prio_cpu(r)) mod WPORTS-1;
            i := n;
            if (linelock_wport_sel_valid(i) = '1') then
                linelock_sel_in(1) <= linelock_wport_sel(i);
                linelock_sel_valid_in(1) <= '1';
                exit;
            end if;
        end loop;
        
        -- Handle result of write port requests...
        if (gnt_linelock(0 to WPORTS-1) = zero64(WPORTS-1 downto 0)) then
            -- New grant...
            if (linelock_sel_valid_out = '1') then
                gnt_linelock(linelock_sel_out) := '1';        
            end if;
        end if;
        
        -- Write results...
        v.linelock := gnt_linelock;

        bifai.gnt_linelock <= gnt_linelock(WPORTS);
        for n in 0 to WPORTS-1 loop
            wpai(n).gnt_linelock <= gnt_linelock(n);
        end loop;
        
        --------------------------------------------------------------------------------------
        -- TagMethod
        --  - Read access is granted per CPU
        --  - Write access is global
        --------------------------------------------------------------------------------------
        -- Collect all request signals...
        for n in 0 to RPORTS-1 loop req_tagr(n) := rpao(n).req_tagr; end loop;
        for n in 0 to WPORTS-1 loop
            req_tagr(n+RPORTS) := wpao(n).req_tagr;
            req_tagw(n) := wpao(n).req_tagw;
        end loop;
        req_tagr(RPORTS+WPORTS) := bifao.req_tagr;
        req_tagw(WPORTS) := bifao.req_tagw;
        
        -- Wait for tagram_ready...
        if (ai.tagram_ready = '0') then
            gnt_tagr := (others => '0');
            gnt_tagw := (others => '0');
        else
            -- Determine existing & to-keep grants...
            gnt_tagr := r.tagr and req_tagr;
            gnt_tagw := r.tagw and req_tagw;
            -- Handle read requests...
            if (unsigned(r.req_tagw) = 0) then -- Writer priority: only accept new reader if no write request waited for more than one cycle ...
                for n in 0 to CFG_NUT_CPU_CORES-1 loop -- Select highest priority acquired bit for each CPU
                    if ((gnt_tagr(n) or gnt_tagr(n+WPORTS) or gnt_tagr(n+2*WPORTS)) = '0'
                    and (n /= CFG_NUT_CPU_CORES-1 or gnt_tagr(RPORTS+WPORTS) = '0')) then
                        -- no existing grant...
                        if (n = CFG_NUT_CPU_CORES-1 and req_tagr(RPORTS+WPORTS) = '1') then
                            -- Prio 0: BUSIF (shares port with last CPU)
                            gnt_tagr(RPORTS+WPORTS) := '1';
                        elsif (req_tagr(n) = '1') then
                            -- Prio 1: Data read
                            gnt_tagr(n) := '1';
                        elsif (req_tagr(n+WPORTS) = '1') then
                            -- Prio 2: Insn read
                            gnt_tagr(n+WPORTS) := '1';
                        elsif (req_tagr(n+RPORTS) = '1') then
                            -- Prio 3: Data write
                            gnt_tagr(n+RPORTS) := '1';
                        end if;
                    end if;
                end loop;
            end if;

            -- Handle write requests...
            -- can only accept new writers if no other writer active
            if ( unsigned(gnt_tagw) = 0) then 
                if (req_tagw(WPORTS) = '1') then
                    gnt_tagw(WPORTS) := '1'; -- give BUSIF highest priority
                else
                    for n in 0 to WPORTS-1 loop
                        i := (n + get_prio_cpu(r)) mod WPORTS;
                        if (req_tagw(i) = '1') then
                            gnt_tagw(i) := '1';
                            exit;
                        end if;
                    end loop;
                end if;
            end if;
        end if;

        -- Write results...
        v.req_tagw := req_tagw;
        v.tagr := gnt_tagr;
        v.tagw := gnt_tagw;

        for n in 0 to RPORTS-1 loop
            rpai(n).gnt_tagr <= gnt_tagr(n);
        end loop;
        wpai(0).gnt_tagr <= gnt_tagr(0+RPORTS);
        wpai(0).gnt_tagw <= gnt_tagw(0);
        for n in 1 to WPORTS-1 loop
            wpai(n).gnt_tagr <= gnt_tagr(n+RPORTS);
            wpai(n).gnt_tagw <= r.tagw(n);
        end loop;
        bifai.gnt_tagr <= gnt_tagr(RPORTS+WPORTS);
        bifai.gnt_tagw <= gnt_tagw(WPORTS);

        --------------------------------------------------------------------------------------
        -- BankMethod
        --  - Arbitration per Bank and Port
        --  - CPUs are assigned to specific one specific port to reduce routing overhead
        --  - Each CPU decides which of its 3 Ports has priority, after that the CPU index
        --    determines priority
        --------------------------------------------------------------------------------------
        -- Collect all way & index addresses...
--        for n in 0 to CFG_NUT_CPU_CORES-1 loop
--            -- Sort input addresses into wiadr (optimized for 2 BRAM ports)
--            wiadr(ram_port(n), ram_index(n)+IDX_RP_OFF) := ai.wiadr_rp(n);
--            wiadr(ram_port(n), ram_index(n)+IDX_IP_OFF) := ai.wiadr_rp(n+CFG_NUT_CPU_CORES);
--            wiadr(ram_port(n), ram_index(n)+IDX_WP_OFF) := get_way_index_of_addr(ai.adr_wp(n), ai.way_wp(n));
--        end loop;
--        if SINGLE_CPU = 0 then wiadr(0, IDX_BUSIF) := (others => '-'); end if; -- Port 0, has one less input
--        wiadr(ram_port(IDX_BUSIF), IDX_BUSIF) := ai.wiadr_busif;

        for b in 0 to CFG_MEMU_CACHE_BANKS-1 loop
            -- Collect all request signals...
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                 -- Sort request signals same as wiadr (optimized for 2 BRAM ports)
                req_bank(ram_port(n))(ram_index(n)+IDX_RP_OFF) := rpao(n).req_bank(b);
                req_bank(ram_port(n))(ram_index(n)+IDX_IP_OFF) := rpao(n+CFG_NUT_CPU_CORES).req_bank(b);
                req_bank(ram_port(n))(ram_index(n)+IDX_WP_OFF) := wpao(n).req_bank(b);
            end loop;
            if SINGLE_CPU = 0 then req_bank(0)(IDX_BUSIF) := '-'; end if; -- Port 0, has one less input
            req_bank(ram_port(IDX_BUSIF))(IDX_BUSIF) := bifao.req_bank(b);
           
            -- Preset sel_port            
--            sel_port := (others => 0);
--            sel_port_valid := (others => '0');
                           
            -- Preset sel_br            
--            sel_br := (others => 0);
--            sel_br_valid := (others => '0');
            
            -- All bank accesses are done in 1 cycle, no need to keep track of already granted ports...
            gnt_bank := (others => (others => '0')); 
            
            -- Set selector input signals per CPU...   
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                i := ram_index(n); -- i is the ram_index of CPU n 
                sel_cpu_wiadr := (others => '-');
                sel_port := i+IDX_RP_OFF;
                sel_port_valid := req_bank(ram_port(n))(i+IDX_RP_OFF) or req_bank(ram_port(n))(i+IDX_IP_OFF) or req_bank(ram_port(n))(i+IDX_WP_OFF);
                if (req_bank(ram_port(n))(i+IDX_RP_OFF) = '1') then
                    -- Prio 1: Data read
                    sel_port := i+IDX_RP_OFF; 
                    sel_cpu_wiadr := ai.wiadr_rp(n);
                elsif (req_bank(ram_port(n))(i+IDX_IP_OFF) = '1') then
                    -- Prio 2: Insn read
                    sel_port := i+IDX_IP_OFF; 
                    sel_cpu_wiadr := ai.wiadr_rp(n+CFG_NUT_CPU_CORES);
                elsif (req_bank(ram_port(n))(i+IDX_WP_OFF) = '1') then
                    -- Prio 3: Data write
                    sel_port := i+IDX_WP_OFF; 
                    sel_cpu_wiadr := get_way_index_of_addr(ai.adr_wp(n), ai.way_wp(n));
                end if;

                -- Write back sel_ports
                if (n = 0 and CFG_MEMU_BANK_RAM_PORTS > 1) then
                  -- At 2 ports CePU gets its own selector...
                  bank_wiadr_in(b, 0, 0) <= sel_cpu_wiadr;
                  bank_sel_in(b, 0, 0) <= sel_port;
                  bank_sel_valid_in(b, 0, 0) <= sel_port_valid; 
                else 
                  bank_cpu_wiadr_in(ram_port(n), cpu_index(n) - CONDITIONAL(CFG_MEMU_BANK_RAM_PORTS > 1 and ram_port(n) = 0, 1, 0)) := sel_cpu_wiadr;
                  bank_cpu_sel_in(ram_port(n), cpu_index(n) - CONDITIONAL(CFG_MEMU_BANK_RAM_PORTS > 1 and ram_port(n) = 0, 1, 0)) := sel_port;
                  bank_cpu_sel_valid_in(ram_port(n), cpu_index(n) - CONDITIONAL(CFG_MEMU_BANK_RAM_PORTS > 1 and ram_port(n) = 0, 1, 0)) := sel_port_valid;
                end if;
            end loop;
            -- BusIf always gets fast input of BankBusIfSel...
            bank_wiadr_in(b, RAMPORT_BUSIF, 0) <= ai.wiadr_busif;
            bank_sel_in(b, RAMPORT_BUSIF, 0 ) <= IDX_BUSIF;
            bank_sel_valid_in(b, RAMPORT_BUSIF, 0) <= req_bank(RAMPORT_BUSIF)(IDX_BUSIF);
      
            -- Select slow input...
            for p in 0 to CFG_MEMU_BANK_RAM_PORTS-1 loop
              bank_wiadr_in(b, p, 1) <= bank_cpu_wiadr_in(p, BANK_CPU_NUM);
              bank_sel_in(b, p, 1) <= bank_cpu_sel_in(p, BANK_CPU_NUM);
              bank_sel_valid_in(b, p, 1) <= bank_cpu_sel_valid_in(p, BANK_CPU_NUM);
              for n in 0 to CFG_NUT_CPU_CORES/CFG_MEMU_BANK_RAM_PORTS - CONDITIONAL(CFG_MEMU_BANK_RAM_PORTS > 1 and p = 0, 1, 0) - 1 loop
--                  i := (n + get_prio_cpu(r)) mod CFG_NUT_CPU_CORES/CFG_MEMU_BANK_RAM_PORTS - CONDITIONAL(CFG_MEMU_BANK_RAM_PORTS > 1 and p = 0, 1, 0);
                  i := n;
                  if (bank_cpu_sel_valid_in(p, i) = '1') then
                       bank_wiadr_in(b, p, 1) <= bank_cpu_wiadr_in(p, i);
                       bank_sel_in(b, p, 1) <= bank_cpu_sel_in(p, i);
                       bank_sel_valid_in(b, p, 1) <= '1';
                      exit;
                  end if;
              end loop;
            end loop;
                        
            -- Find selected 'wiadr's & determine all possible grant lines...
            for p in 0 to CFG_MEMU_BANK_RAM_PORTS-1 loop
                sel_wiadr(p) := (others => '-'); 
                if (bank_sel_valid_out(b, p) = '1') then 
                    gnt_bank(p)(bank_sel_out(b, p)) := '1';
                    sel_wiadr(p) := bank_wiadr_out(b, p);
                end if;
            end loop;
            -- Deactivated parallel execution for now 
            --for n in 0 to RPORTS+WPORTS loop
                --if (sel_port_valid(ram_port(n)) = '1' and req_bank(n) = '1' and wiadr(n) = sel_wiadr(ram_port(n))) then
                    --gnt_bank(n) := '1';
                --end if;
            --end loop;

            -- Write results...
            v.bank(b) := gnt_bank;
            for n in 0 to CFG_NUT_CPU_CORES-1 loop  
                rpai(n).gnt_bank(b) <= gnt_bank(ram_port(n))(ram_index(n)+IDX_RP_OFF);
                rpai(n+CFG_NUT_CPU_CORES).gnt_bank(b) <= gnt_bank(ram_port(n))(ram_index(n)+IDX_IP_OFF);
                wpai(n).gnt_bank(b) <= gnt_bank(ram_port(n))(ram_index(n)+IDX_WP_OFF);
            end loop;
            bifai.gnt_bank(b) <= gnt_bank(RAMPORT_BUSIF)(IDX_BUSIF);

            for p in 0 to CFG_MEMU_BANK_RAM_PORTS-1 loop
                ao.wiadr_bank(b)(p) <= sel_wiadr(p);
            end loop;
        end loop; -- for b in 0 to CFG_MEMU_CACHE_BANKS-1 loop

        -------------------------------------------------------------------------------------- 
        -- BusifMethod
        --  - Each CPU decides which of its 3 Ports has priority, after that the CPU index
        --    determines priority
        --------------------------------------------------------------------------------------
        -- Collect all request signals...
        for n in 0 to RPORTS-1 loop req_busif(n) := rpao(n).req_busif; end loop;
        for n in 0 to WPORTS-1 loop req_busif(RPORTS+n) := wpao(n).req_busif; end loop;
        
        -- Determine existing & to-keep grants...
        gnt_busif := r.busif and req_busif;
        
        -- Set default select value...
        busif_sel := RPORTS+WPORTS;
        
        -- Pio 4: Default value               
        -- TBD: Still needed?  
        cpu_sel := RPORTS+WPORTS;
        cpu_sel_valid := '0';
        -- Prio 0: Granted ports
        if (gnt_busif(0) = '1') then 
           -- data read port
           busif_sel := 0;
           cpu_sel := 0;
           cpu_sel_valid := '1';
        elsif (gnt_busif(CFG_NUT_CPU_CORES) = '1') then 
           -- insn read port
           busif_sel := CFG_NUT_CPU_CORES;
           cpu_sel := CFG_NUT_CPU_CORES;
           cpu_sel_valid := '1';
        elsif (gnt_busif(RPORTS) = '1') then 
           -- data write port
           busif_sel := RPORTS;
           cpu_sel := RPORTS;
           cpu_sel_valid := '1';
        else      
           -- Handle new requests...
           if (req_busif(0) = '1') then 
               -- Prio 1: Data read
               cpu_sel := 0;
               cpu_sel_valid := '1';
           elsif (req_busif(CFG_NUT_CPU_CORES) = '1') then 
               -- Prio 2: Instruction read
               cpu_sel := CFG_NUT_CPU_CORES;
               cpu_sel_valid := '1';
           elsif (req_busif(RPORTS) = '1') then 
               -- Prio 3: Data write
               cpu_sel := RPORTS;
               cpu_sel_valid := '1';
           end if; 
        end if;
       
        busif_sel_in(0) <= cpu_sel;
        busif_sel_valid_in(0) <= cpu_sel_valid;
        
        -- Handle new requests...
        for n in 1 to CFG_NUT_CPU_CORES-1 loop
           -- Pio 4: Default value               
           -- TBD: Still needed?  
           busif_cpu_sel(n-1) := RPORTS+WPORTS;
           busif_cpu_sel_valid(n-1) := '0';
           -- Prio 0: Granted ports
           if (gnt_busif(n) = '1') then 
               -- data read port
               busif_sel := n;
               busif_cpu_sel(n-1) := n;
               busif_cpu_sel_valid(n-1) := '1';
           elsif (gnt_busif(CFG_NUT_CPU_CORES+n) = '1') then 
               -- insn read port
               busif_sel := CFG_NUT_CPU_CORES + n;
               busif_cpu_sel(n-1) := CFG_NUT_CPU_CORES + n;
               busif_cpu_sel_valid(n-1) := '1';
           elsif (gnt_busif(RPORTS+n) = '1') then 
               -- data write port
               busif_sel := RPORTS + n;
               busif_cpu_sel(n-1) := RPORTS + n;
               busif_cpu_sel_valid(n-1) := '1';
           else      
               -- Handle new requests...
               if (req_busif(n) = '1') then 
                   -- Prio 1: Data read
                   busif_cpu_sel(n-1) := n;
                   busif_cpu_sel_valid(n-1) := '1';
               elsif (req_busif(CFG_NUT_CPU_CORES+n) = '1') then 
                   -- Prio 2: Instruction read
                   busif_cpu_sel(n-1) := CFG_NUT_CPU_CORES + n;
                   busif_cpu_sel_valid(n-1) := '1';
               elsif (req_busif(RPORTS+n) = '1') then 
                   -- Prio 3: Data write
                   busif_cpu_sel(n-1) := RPORTS + n;
                   busif_cpu_sel_valid(n-1) := '1';
               end if; 
           end if;
        end loop;
         
        -- Select slow input...
        busif_sel_in(1) <= busif_cpu_sel(0);
        busif_sel_valid_in(1) <= busif_cpu_sel_valid(0);
        for n in 0 to CFG_NUT_CPU_CORES-2 loop
--            i := (n + get_prio_cpu(r)) mod CFG_NUT_CPU_CORES-1;
            i := n;
            if (busif_cpu_sel_valid(i) = '1') then
                busif_sel_in(1) <= busif_cpu_sel(i);
                busif_sel_valid_in(1) <= busif_cpu_sel_valid(i);
                exit;
            end if;
        end loop;
         
        -- Find selected port & grant...
        if (busif_sel = RPORTS+WPORTS)  then 
            -- New grant...
            if (busif_sel_valid_out = '1') then
                busif_sel := busif_sel_out;
                gnt_busif(busif_sel) := '1';
            end if;       
        end if;
        
        --Write results...
        v.busif := gnt_busif;
        ao.busif_sel <= busif_sel;
        for n in 0 to RPORTS-1 loop rpai(n).gnt_busif <= gnt_busif(n); end loop;
        for n in 0 to WPORTS-1 loop wpai(n).gnt_busif <= gnt_busif(n+RPORTS); end loop;


        --------------------------------------------------------------------------------------
        -- SnoopMethod
        --  - Determine a/the writer...
        --  - NOTE: only cached writes are supported for snooping (LL/SC)
        --------------------------------------------------------------------------------------
        write := false;
        writer := WPORTS-1;
        --if (v.linelock(0) = '1') then
                --write := true;
                --writer := 0;
        --end if;
        for n in 0 to WPORTS-1 loop
            if (r.linelock(n) = '1') then -- to catch a writer to the cache
                --pragma translate_off
                -- there should be only one!
                assert not write report "more than 1 writers" severity error;
                --pragma translate_on
                write := true;
                writer := n;
            end if;
        end loop;
        
        -- Generate output signals...
        if (write) then
            for n in 0 to WPORTS-1 loop
                rpai(n).snoop_stb <= '1'; -- signal to all CPUs...
                rpai(n).snoop_adr <= ai.adr_wp(writer);
            end loop;
            rpai(writer).snoop_stb <= '0'; -- ... except to the one that caused the write to avoid race condition
        else
            for n in 0 to WPORTS-1 loop
                rpai(n).snoop_stb <= '0';
                rpai(n).snoop_adr <= (others => '-'); -- don't care
            end loop;
        end if;

        if (CFG_MEMU_ARBITER_METHOD >= 0) then
            -- round robin...
            v.counter := r.counter + 1;
        else
            -- LFSR...
            v.lfsr_counter := get_next_lfsr_state(r.lfsr_counter, COUNTER_POLY);
        end if;

        if (reset = '1') then
            if (CFG_MEMU_ARBITER_METHOD >= 0) then
                v.counter := (others => '1');
            else
                v.lfsr_counter := (others => '1');
            end if;
            v.linelock := (others => '0');
        end if;

        rin <= v;

    end process;

    process (clk)
    begin
        if (clk'event and clk = '1') then
            r <= rin;
        end if;
    end process;

end rtl;
