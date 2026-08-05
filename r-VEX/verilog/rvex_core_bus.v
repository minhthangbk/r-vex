//======================================================================
// r-VEX SoC phase 1 | Bus-attached core variant (round 25)
//----------------------------------------------------------------------
// SCOPE / WHY A SEPARATE FILE, NOT AN EDIT TO rvex_core.v:
//   Every existing testbench in this directory (tb_core.v, tb_core_prog.v,
//   tb_core_carry.v, tb_dsp.v, tb_units.v) plus the automated 41-kernel
//   regression (toolchain/simauto.sh + tb_auto.v) reach straight into
//   rvex_core's internal `imem`/`dmem`/`gr`/`br` reg arrays via Verilog
//   hierarchical references (`dut.imem[a]=...`, `dut.gr[N]`, `$readmemh(
//   "...", dut.imem)`). Replacing those arrays with an external bus
//   in-place would silently break every one of those references and put
//   this project's entire "silicon-verified" kernel history at risk in a
//   single round -- exactly the invasive-blast-radius mistake this
//   project's own CLAUDE.md warns about for scheduler/packetizer changes,
//   here at the RTL level instead. rvex_core.v is therefore left
//   completely untouched; this is an ADDITIVE new core variant, verified
//   against the same kernels independently (see tb_soc_top.v), not a
//   replacement.
//
// WHAT THIS ADDS vs rvex_core.v (round-24 finding, docs item 22):
//   rvex_core.v is purely combinational for IMEM/DMEM (zero real latency),
//   which is exactly why round 24 found the compiler's load-use-latency=2
//   assumption (rvexSchedule.td: IILoadStore, InstrStage<2,[P0]>) could
//   never be exercised by RTL simulation -- there was no failure mode for
//   "used a loaded value too early". This variant gives DMEM (and IMEM) a
//   real registered SRAM (sram_sync.v, 1-cycle address-to-Q latency)
//   behind an AHB-Lite bus (ahb_lite_sram_ctrl.v) for DMEM, making that
//   assumption falsifiable for the FIRST time: see the "GATE DECISION:
//   non-interlocked" note below for why this only works if the hardware
//   does NOT auto-stall on a load-use hazard.
//
//   SCOPE OF THE FALSIFIABILITY CLAIM (Codex catch, round 25): "bundle-
//   distance 1 always reads stale, distance 2 always reads fresh" holds
//   for an ORDINARY (non-memory, non-XNOP) consumer bundle -- including a
//   consumer that is itself a taken branch. It does NOT hold universally
//   for every bundle shape the core supports: (a) if the distance-1
//   bundle is ITSELF a memory op, bundle_stall holds it until the load's
//   own transaction frees the bus, by which point the load has already
//   completed -- so it re-evaluates its operands from the now-updated
//   register file and sees the FRESH value, not stale; (b) a load
//   immediately followed by an XNOP can finish completing DURING the
//   XNOP's stall (the DMEM sub-FSM runs independently of stall_cnt), so
//   the bundle right after the XNOP -- nominally bundle-distance 1 from
//   the load, ignoring the XNOP -- can also see FRESH data. Neither is a
//   bug: a real compiler would never schedule a load's consumer at
//   distance 1 in the first place (that's the violation this RTL exists
//   to catch), and both cases only ever make an ALREADY-illegal schedule
//   accidentally work rather than making a LEGAL one fail. The precise
//   claim is "falsifiable for unstalled, non-XNOP-adjacent consumers,"
//   not "falsifiable for every syntactically possible bundle sequence" --
//   see reviews/rvex-round25-*.html section 7.2 for Codex's full analysis.
//
// GATE DECISION: non-interlocked load-use (grounded via NotebookLM
// digital_design, round 25 -- reviews/rvex-round25-*.html section 2,
// question 3): a real r-VEX / VEX ISA core is NON-interlocked -- the
// compiler is solely responsible for respecting the machine's latencies
// (the VEX "LEQ" model: hardware may complete faster than assumed, never
// slower, without the compiler's knowledge). If this wrapper made DMEM
// automatically stall the whole core until load data arrived (a
// HREADY-style CPU-side stall), a compiler bug that scheduled a load's
// consumer 1 cycle too early would be silently absorbed by the stall --
// masking exactly the class of bug this exists to catch. So: the DMEM
// bus stalls OTHER new memory transactions (a real structural hazard --
// see DMEM_BUSY below) but never stalls the core waiting on a specific
// load's DATA; the loaded value lands in the register file exactly
// load-use-latency cycles after issue, and a too-early read of that
// register sees the architecturally-correct-but-stale prior value, same
// as rvex_core.v's own existing "last write wins" GR semantics.
//
// GATE DECISION: IMEM is a private, non-bus, fixed-1-cycle-latency TCM
// (tightly-coupled memory) port straight to its own sram_sync macro, NOT
// routed through ahb_lite_sram_ctrl's HREADY handshake. This is a
// deliberate, documented deviation from a literal reading of "AXI4/AHB-
// Lite wrapper for everything": NotebookLM digital_design's own answer
// warned that keeping IMEM combinational while DMEM is synchronous is
// architecturally inconsistent and would require a real fetch pipeline
// stage regardless -- so this core DOES add that fetch pipeline stage
// (2-deep: address phase then data phase, see pc_f/pc_f_prev/packet_e
// below), achieving genuine 1-bundle/cycle steady-state fetch throughput.
// Routing IMEM through a second full AHB-Lite handshake on top of that
// would add area/complexity without changing any observable timing
// (fetch latency is fixed and known, unlike DMEM which will eventually
// share its bus with other masters/peripherals in later SoC phases) --
// this matches how real embedded/DSP cores commonly keep instruction TCM
// off the main system bus specifically for fetch-critical-path reasons.
//
// MICROARCHITECTURE:
//   F stage: pc_f drives imem_addr every cycle (free-running, 1 access/
//     cycle, no backpressure -- TCM is never busy). pc_f_prev/packet_e
//     form a 2-deep pipeline register so packet_e/pc_e are the
//     currently-executing bundle 2 cycles after its address was issued
//     (1 cycle is the SRAM's own registered latency; the 2nd is this
//     module's own capture register).
//   E stage: identical ALU/MUL/ctrl decode+writeback to rvex_core.v
//     (slots 0-2 unchanged -- diff this file against rvex_core.v to see
//     how small that part of the change actually is). Slot 3 (MEM) is
//     the only structurally new logic: it drives the DMEM AHB-Lite
//     master, with a small `dm_state` sub-FSM tracking an in-flight
//     transaction (DM_WAIT1/DM_WAIT2) independently of whether the E
//     stage itself is stalled, so a load's completion can still write
//     GR at the correct cycle even while later bundles are technically
//     blocked waiting for the bus to free up (DMEM_BUSY structural
//     stall -- back-to-back memory-accessing bundles cost 1 extra cycle
//     because this phase-1 bus supports exactly one outstanding
//     transaction; this is a genuine, hardware-enforced resource stall,
//     unrelated to and does not mask the load-use DATA hazard above).
//   STH/STB (sub-word stores) need a read-modify-write: the old word
//     must come back from DMEM before the merged word can be written,
//     so these cost 2 bus transactions (DM_WAIT1 = read old word +
//     compute merge via mem_unit, DM_WAIT2 = write merged word) instead
//     of STW's 1.
//   Branch/call/return misprediction: this core always "predicts"
//     sequential (pc_f free-runs at pc_f+1). A taken branch resolved in
//     E redirects pc_f to the target and asserts `squash` for 2 cycles,
//     forcing packet_e_eff to an all-zero (4-way NOP) bundle for however
//     many already-in-flight wrong-path fetches are still draining the
//     2-deep fetch pipeline -- a conventional 2-cycle branch penalty.
//   XNOP and the DMEM structural stall both freeze the ENTIRE F/E
//     pipeline (pc_f/pc_f_prev/packet_e/pc_e/squash all hold) rather
//     than letting F keep prefetching into a separate buffer -- simpler
//     and lower-risk than an elastic buffer, at the cost of not
//     prefetching ahead during a stall (acceptable: XNOP/back-to-back-
//     mem-access sequences are not this round's optimization target,
//     only their correctness is).
//
// ROUND 26 ADDITION: `imem_stall` input (see reviews/rvex-round26-*.html).
//   D-side caches need NO core change at all: the existing DMEM AHB-Lite
//   port already tolerates an arbitrarily long wait for d_hreadyout
//   (bundle_stall already freezes the whole core for however long that
//   takes), so a D-Cache miss "just" makes d_hreadyout arrive later --
//   exactly the LEQ-model "hardware slower than assumed" case this core
//   was already built to handle correctly. I-side is different: IMEM was
//   deliberately given NO ready/valid handshake in round 25 (a fixed,
//   always-1-cycle TCM, required for genuine 1-bundle/cycle steady-state
//   fetch throughput -- see the round-25 header above) so there was
//   nothing an I-Cache miss could assert to make the core wait. This
//   port is that missing signal: when high, the ENTIRE F/E pipeline
//   freezes for exactly as long as it's asserted (same "whole pipeline
//   holds, nothing advances" pattern as the existing XNOP/bundle_stall
//   freezes -- see fetch_advance below, now also gated on !imem_stall),
//   resuming the cycle it drops with imem_rdata now holding the correct
//   (freshly cached) data. Tied to 1'b0 in round 25's own rvex_soc_top.v
//   (unchanged, still exactly its original behavior/timing -- reverified
//   by rerunning that round's full regression after adding this port);
//   driven from an I-Cache's miss FSM in rvex_soc_top_cache.v.
//======================================================================
`include "rvex_defs.vh"

module rvex_core_bus #(
    // Reset-time squash duration (round 26): round 25's TCM (sram_sync
    // wired directly, no ready/valid handshake) has no way to say "not
    // ready yet" -- its internal addr_r register has NO reset, so it
    // sits at X until the first `en` pulse, and even after that, packet0's
    // REAL data doesn't reach packet_e until the 3rd normal-advance event
    // (1 cycle of X-settling + the pc_f_prev/packet_e 2-deep pipe fill).
    // squash=2'b11 at reset masks exactly that: found by hand-tracing
    // round 25's own timing, verified by that round's regression.
    //
    // A cache changes this: imem_stall (see that port's header note)
    // means normal-advance literally CANNOT happen until the I-Cache's
    // own miss-refill handshake says the data is legitimate -- so by the
    // time the FIRST normal-advance event fires, packet0's real data is
    // ALREADY correctly settled (found by simulation: rvex_soc_top_cache.v
    // was silently discarding a legitimately-arrived packet0 with the
    // round-25 default, because the extra 2-event mask now falls on the
    // FIRST already-valid capture instead of on genuine pre-settling
    // garbage). Overridden to 0 by rvex_soc_top_cache.v; left at the
    // round-25 default (and reverified via that round's own regression)
    // for rvex_soc_top.v. The BRANCH-flush use of squash (`squash<=2'b11`
    // on a taken branch, elsewhere in this file) is a separate, genuinely
    // cache-independent structural property -- the pc_f_prev/packet_e
    // pipe is always exactly 2 deep once in steady state -- and is NOT
    // parameterized.
    parameter [1:0] RESET_SQUASH = 2'b11,
    // pc_e labeling source (round 26 -- found by simulation, not hand
    // tracing: a real bug that shipped in an earlier draft and produced
    // wrong branch targets / misattributed writebacks for an entire
    // instruction). round 25's sram_sync gives imem_rdata a FIXED,
    // unconditional 1-RAW-CYCLE latency (addr_r<=addr every single
    // cycle, en tied to fetch_advance which always equals 1 event in
    // that design) -- since round 25 never stalls, "1 cycle behind" and
    // "1 EVENT behind" are the same thing, and pc_f_prev (which tracks
    // pc_f exactly 1 EVENT behind) correctly labels whatever imem_rdata
    // is showing. A cache breaks that equivalence: icache.v's idx_r
    // resolves to represent pc_f's CURRENT value once ready (0 events
    // behind -- pc_f is frozen at that same address for however many
    // raw cycles the miss takes, so by the time the miss resolves,
    // "pc_f now" and "pc_f when the miss started" are the SAME value),
    // not "1 event behind" like pc_f_prev. Pairing packet_e (cache data,
    // 0 events behind) with pc_e<=pc_f_prev (1 event behind) mislabels
    // every capture by exactly one packet -- observed as: branch targets
    // computed from the WRONG packet's decode, and an entire ALU bundle
    // (whichever one pc_e claimed instead of the one packet_e actually
    // held) silently never getting its writeback committed. Set to 1 by
    // rvex_soc_top_cache.v; left 0 (pc_f_prev, round-25-original) for
    // rvex_soc_top.v, reverified via that round's own regression.
    parameter PC_E_FROM_PC_F = 0,
    // Branch-flush squash duration (round 26, found by simulation as a
    // DIRECT CONSEQUENCE of PC_E_FROM_PC_F, not a separate hand-derived
    // decision): round 25's pipeline is genuinely 2-deep for addressing
    // purposes (pc_f -> pc_f_prev -> pc_e/packet_e), so a taken branch
    // always has exactly 2 already-in-flight wrong-path captures to
    // discard. Collapsing pc_e's source to pc_f (PC_E_FROM_PC_F=1)
    // collapses the EFFECTIVE addressing pipeline to 1-deep -- pc_f
    // itself is what freezes-until-resolved and gets captured directly,
    // with no intermediate stage -- so there is only ever ONE wrong-path
    // capture in flight at branch-resolution time (whatever's currently
    // sitting in packet_e, about to be overwritten). Using the round-25
    // value (2'b11) here squashed the branch TARGET's own packet, not
    // just the wrong-path one -- observed as the target bundle's
    // writeback silently never happening even though control flow
    // otherwise looked correct. Set to 2'b01 by rvex_soc_top_cache.v.
    parameter [1:0] BRANCH_SQUASH = 2'b11
) (
    input  wire clk,
    input  wire reset,
    input  wire start,
    output reg  done,
    output reg  [31:0] cycles,

    // ---- IMEM: private fixed-latency TCM port (see header) ----
    output wire [7:0]   imem_addr,
    output wire          imem_en,   // see fetch_advance below -- MUST gate real reads, not be tied high
    input  wire [127:0] imem_rdata,
    input  wire          imem_stall, // round 26: I-Cache miss -- whole-pipeline freeze, see header

    // ---- DMEM: AHB-Lite master ----
    output wire [7:0]   d_haddr,
    output wire          d_hwrite,
    output wire [1:0]    d_htrans,
    output wire [31:0]   d_hwdata,
    input  wire [31:0]   d_hrdata,
    input  wire          d_hreadyout
);
    // ---- architectural state ----
    reg [31:0] gr [0:63];
    reg        br [0:7];
    reg        running;
    integer    i;

    // ---- fetch pipeline (F stage, 2-deep: addr phase -> data phase) ----
    reg [7:0]   pc_f;        // address driven to imem THIS cycle
    reg [7:0]   pc_f_prev;   // pc_f as of the previous cycle
    reg [127:0] packet_e;    // captured imem_rdata (pairs with pc_e)
    reg [7:0]   pc_e;        // architectural PC of packet_e
    reg [1:0]   squash;      // branch-flush shift register (see header)

    assign imem_addr = pc_f;

    wire [127:0] packet_e_eff = squash[0] ? 128'd0 : packet_e;

    // ---- XNOP stall + DMEM structural-hazard stall ----
    reg [11:0] stall_cnt;

    // ---- DMEM transaction sub-FSM ----
    // DM_RMW_ISSUE is its own state (not folded into the DM_WAIT1->DM_WAIT2
    // edge) for a subtle reason found by simulation (tb_soc_top.v's STB
    // test): ahb_lite_sram_ctrl's OWN internal `state` only returns to IDLE
    // (able to latch a NEW address) the cycle AFTER its HREADYOUT went high
    // -- so combinationally asserting the RMW's write-phase HTRANS=NONSEQ
    // in the SAME cycle DM_WAIT1 sees HREADYOUT (an earlier draft did this)
    // presents the new address while the slave is still finishing its OWN
    // wait state; the slave silently ignores it (never latches), and this
    // core's dm_state marches on to DM_WAIT2 believing the write was
    // issued, where it then sees the slave's now-idle HREADYOUT=1 (nothing
    // in flight) and immediately calls the write "done" -- so the merged
    // word is silently never written. DM_RMW_ISSUE exists purely to give
    // the write-phase address ONE cycle where the slave is guaranteed IDLE
    // (mirroring exactly how DM_IDLE+issue1 issues phase 1), one full
    // cycle later than the naive version.
    localparam DM_IDLE = 2'd0, DM_WAIT1 = 2'd1, DM_RMW_ISSUE = 2'd2, DM_WAIT2 = 2'd3;
    reg [1:0]  dm_state;
    // Round 26: prevents a real race found by simulation (tb_soc_top_cache.v's
    // D-Cache eviction test) -- issue1's condition (dm_state==DM_IDLE &&
    // want_mem) becomes true again the INSTANT a bundle's own transaction
    // completes and dm_state cycles back to IDLE, using packet_e_eff's
    // STILL-current (not yet superseded) content. In round 25 this always
    // coincided with normal-advance ALSO being ready to fire that same
    // cycle (bundle_stall clearing was the only gate), so the bundle got
    // superseded before the spurious reissue could matter -- wasted a
    // cycle re-fetching the SAME address, invisible to round 25's tests
    // since the value doesn't change. Round 26 breaks that coincidence:
    // imem_stall can independently keep normal-advance blocked for
    // several MORE cycles after bundle_stall clears, during which
    // issue1 keeps re-evaluating true and re-issues the SAME bundle's
    // transaction again -- and because that spurious transaction makes
    // the bus busy again right as normal-advance FINALLY does fire, it
    // delays the NEXT bundle's own legitimate transaction. mem_issued
    // latches "this bundle already issued its transaction once"; issue1
    // is gated by !mem_issued; the flag resets to 0 the cycle normal-
    // advance actually captures a new bundle (see mem_issued's two write
    // sites below -- both live in the SAME always block as each other
    // and as load_complete_now, for the same multi-driver reason).
    reg        mem_issued;
    reg        pend_is_load, pend_is_rmw;
    reg [5:0]  pend_dst;
    reg [6:0]  pend_op3;
    reg [1:0]  pend_pos;
    reg [31:0] pend_regval;
    reg [7:0]  pend_addr;

    wire mem_busy = (dm_state != DM_IDLE);

    // ---- current bundle & syllables (identical field layout to rvex_core.v) ----
    wire [31:0] syl0 = packet_e_eff[31:0];
    wire [31:0] syl1 = packet_e_eff[63:32];
    wire [31:0] syl2 = packet_e_eff[95:64];
    wire [31:0] syl3 = packet_e_eff[127:96];

    wire [6:0] op0=syl0[31:25], op1=syl1[31:25], op2=syl2[31:25], op3=syl3[31:25];
    wire [1:0] imt0=syl0[24:23], imt1=syl1[24:23], imt2=syl2[24:23], imt3=syl3[24:23];
    wire [5:0] dst0=syl0[22:17], dst1=syl1[22:17], dst2=syl2[22:17], dst3=syl3[22:17];
    wire [5:0] r1a0=syl0[16:11], r1a1=syl1[16:11], r1a2=syl2[16:11], r1a3=syl3[16:11];
    wire [5:0] r2a0=syl0[10:5],  r2a1=syl1[10:5],  r2a2=syl2[10:5];
    wire       st3  = (op3==`MEM_STW) || (op3==`MEM_STH) || (op3==`MEM_STB);
    wire [5:0] r2a3 = st3 ? syl3[22:17] : syl3[10:5];
    wire [2:0] dbr0=syl0[4:2],   dbr1=syl1[4:2],   dbr2=syl2[4:2],   dbr3=syl3[4:2];
    wire [2:0] sbr0=syl0[27:25], sbr1=syl1[27:25], sbr2=syl2[27:25];
    wire        isret0 = (op0==`CTRL_RETURN) || (op0==`CTRL_RFI);
    wire [11:0] off0   = isret0 ? {3'b0, syl0[10:2]} : syl0[16:5];

    function is_cmp; input [6:0] op; begin
        is_cmp = (op >= `ALU_CMPEQ) && (op <= `ALU_ANDL); end
    endfunction
    function is_mul; input [6:0] op; begin
        is_mul = (op[6:4] == 3'b000) && (op != `OP_NOP); end
    endfunction
    function is_addcg_divs; input [6:0] op; begin
        is_addcg_divs = (op[6:3]==`ALU_ADDCG4) || (op[6:3]==`ALU_DIVS4); end
    endfunction

    wire [31:0] gr_r1_0 = gr[r1a0], gr_r2_0 = gr[r2a0];
    wire [31:0] gr_r1_1 = gr[r1a1], gr_r2_1 = gr[r2a1];
    wire [31:0] gr_r1_2 = gr[r1a2], gr_r2_2 = gr[r2a2];
    wire [31:0] gr_r1_3 = gr[r1a3], gr_r2_3 = gr[r2a3];

    function narrow_imm; input [6:0] op; input [5:0] dst; begin
        narrow_imm = (op >= `ALU_CMPEQ) && (op <= `ALU_CMPNE) && (dst == 6'd0); end
    endfunction

    function [31:0] op2sel;
        input [1:0] imt; input [31:0] regv; input [31:0] syl; input [31:0] ext;
        input narrow;
    begin
        case (imt)
            `SHORT_IMM : op2sel = narrow ? {{26{syl[10]}}, syl[10:5]}
                                         : {{23{syl[10]}}, syl[10:2]};
            `BRANCH_IMM: op2sel = {20'b0, syl[16:5]};
            `LONG_IMM  : op2sel = {ext[22:0],     syl[10:2]};
            default    : op2sel = regv;
        endcase
    end endfunction

    wire [31:0] a2_0 = op2sel(imt0, gr_r2_0, syl0, syl1,  narrow_imm(op0,dst0));
    wire [31:0] a2_1 = op2sel(imt1, gr_r2_1, syl1, syl2,  narrow_imm(op1,dst1));
    wire [31:0] a2_2 = op2sel(imt2, gr_r2_2, syl2, syl3,  narrow_imm(op2,dst2));
    wire [31:0] a2_3 = op2sel(imt3, gr_r2_3, syl3, 32'd0, narrow_imm(op3,dst3));

    wire ext0 = (op0 == `OP_SYLFOLW), ext1 = (op1 == `OP_SYLFOLW);
    wire ext2 = (op2 == `OP_SYLFOLW), ext3 = (op3 == `OP_SYLFOLW);

    wire cin0 = br[sbr0], cin1 = br[sbr1], cin2 = br[sbr2];
    wire cin3 = br[syl3[27:25]];

    wire [31:0] alu0_r,alu1_r,alu2_r,alu3_r;
    wire        alu0_c,alu1_c,alu2_c,alu3_c;
    wire        alu0_v,alu1_v,alu2_v,alu3_v;
    alu u_alu0(.aluop(op0),.src1(gr_r1_0),.src2(a2_0),.cin(cin0),.result(alu0_r),.cout(alu0_c),.out_valid(alu0_v));
    alu u_alu1(.aluop(op1),.src1(gr_r1_1),.src2(a2_1),.cin(cin1),.result(alu1_r),.cout(alu1_c),.out_valid(alu1_v));
    alu u_alu2(.aluop(op2),.src1(gr_r1_2),.src2(a2_2),.cin(cin2),.result(alu2_r),.cout(alu2_c),.out_valid(alu2_v));
    alu u_alu3(.aluop(op3),.src1(gr_r1_3),.src2(a2_3),.cin(cin3),.result(alu3_r),.cout(alu3_c),.out_valid(alu3_v));

    wire [31:0] mul1_r, mul2_r; wire mul1_v, mul2_v, mul1_o, mul2_o;
    mul u_mul1(.mulop(op1),.src1(gr_r1_1),.src2(a2_1),.result(mul1_r),.overflow(mul1_o),.out_valid(mul1_v));
    mul u_mul2(.mulop(op2),.src1(gr_r1_2),.src2(a2_2),.result(mul2_r),.overflow(mul2_o),.out_valid(mul2_v));

    // ---- slot-3 memory decode (address-issue time; opcode-only, no mem_val needed yet) ----
    wire [9:0] dmem_byte_addr = gr_r1_3[9:0] + {syl3[10], syl3[10:2]};
    wire [7:0] dmem_word_addr = dmem_byte_addr[9:2];
    wire [1:0] dmem_pos       = dmem_byte_addr[1:0];
    wire is_ld3  = (op3==`MEM_LDW)||(op3==`MEM_LDH)||(op3==`MEM_LDHU)||(op3==`MEM_LDB)||(op3==`MEM_LDBU);
    wire is_st3  = (op3==`MEM_STW)||(op3==`MEM_STH)||(op3==`MEM_STB);
    wire is_rmw3 = (op3==`MEM_STH)||(op3==`MEM_STB);
    wire want_mem = (is_ld3 || is_st3) && !ext3;

    // ---- mem_unit: used ONLY at DMEM completion time (mem_val=d_hrdata) ----
    wire [31:0] mem_load_c, mem_store_c;
    wire        mem_is_load_c, mem_is_store_c, mem_v_c;
    mem_unit u_mem (
        .opcode(pend_op3), .mem_val(d_hrdata), .reg_val(pend_regval), .pos(pend_pos),
        .load_data(mem_load_c), .store_data(mem_store_c),
        .is_load(mem_is_load_c), .is_store(mem_is_store_c), .out_valid(mem_v_c)
    );

    // ---- control unit (slot 0) ----
    wire [7:0]  c_pc_goto; wire [31:0] c_link; wire c_taken, c_wlink, c_xnop, c_rfi, c_v;
    wire [11:0] c_xnop_n;
    wire c_br_in = br[syl0[4:2]];
    ctrl_unit u_ctrl(.opcode(op0),.pc(pc_e),.lr(gr_r1_0),.sp(gr[1]),.offset(off0),.br(c_br_in),
                     .pc_goto(c_pc_goto),.link_val(c_link),.taken(c_taken),.writes_link(c_wlink),
                     .is_xnop(c_xnop),.xnop_n(c_xnop_n),.is_rfi(c_rfi),.out_valid(c_v));

    wire slot0_is_ctrl = (op0[6:4] == 3'b010);
    wire slot0_is_stop = (op0 == `OP_STOP);

    wire [31:0] res1 = is_mul(op1) ? mul1_r : alu1_r;
    wire [31:0] res2 = is_mul(op2) ? mul2_r : alu2_r;

    wire bundle_stall = want_mem && mem_busy; // structural: previous DMEM txn still in flight

    // A completed load's gr[] write MUST land in the SAME always block as
    // every other gr[] writer (Codex catch, round 25): this signal used to
    // drive `gr[pend_dst] <= mem_load_c` from ITS OWN always block (the
    // DMEM sub-FSM above), independently clocked from the main sequencer's
    // always block below, which also writes gr[] for slots 0-3. Two
    // separate always blocks both issuing a non-blocking write to the same
    // array on the same edge is well-defined in a given simulator but is
    // NOT guaranteed consistent across simulators, and many synthesis
    // flows reject or silently mis-resolve multi-process writes to the
    // same register/RAM. Fixed by moving the actual write into the main
    // sequencer below, keyed off this wire, so gr[] has exactly one
    // clocked writer. Placed BEFORE slots 0-3's writebacks there (program-
    // order priority: the currently-committing bundle's own write, if it
    // happens to collide on the same destination, is chronologically
    // later than the load and should win -- consistent with how slots 0-3
    // already resolve same-bundle collisions by later-slot-wins).
    wire load_complete_now = (dm_state == DM_WAIT1) && d_hreadyout
                              && pend_is_load && (pend_dst != 6'd0);

    // Exactly the condition that gates the "normal advance" branch below
    // (fetch-pipeline advance + bundle commit). imem_en MUST track this,
    // not be tied high: found by simulation (tb_soc_top.v's 3-deep
    // back-to-back-store test silently dropped the 3rd store's whole
    // bundle) that a constantly-enabled SRAM re-latches pc_f's address
    // EVERY cycle even while the core is frozen mid-stall -- and because
    // pc_f free-runs 2 cycles ahead of pc_e, by the time a stall is
    // detected pc_f has already raced past the very fetch that's still
    // in flight toward packet_e. With `en` tied high, freezing pc_f but
    // not the SRAM's own address latch let it keep re-latching pc_f's
    // now-frozen-but-too-advanced value every cycle, permanently
    // overwriting (never losing to X, just silently replacing) the
    // in-flight bundle before packet_e ever captured it -- so the bundle
    // between the stalled one and the frozen pc_f simply never executed.
    // Gating `en` by fetch_advance holds the SRAM's address register
    // stable for the whole freeze, so the in-flight fetch is exactly
    // where packet_e finds it once unfrozen.
    //
    // Deliberately NOT gated by `imem_stall` (round 26): imem_stall is an
    // INPUT computed by the I-Cache from imem_addr/imem_en themselves --
    // making imem_en depend on imem_stall would be a real combinational
    // loop (imem_en -> cache's hit/miss decision -> imem_stall -> imem_en),
    // not just a bad-practice one. Caught by re-deriving this by hand
    // before writing the cache, not by simulation. Instead: imem_en stays
    // high (re-presenting the SAME, frozen pc_f address) for the whole
    // duration of an I-Cache miss, which is safe as long as the cache
    // itself ignores repeated en-pulses for an address it's already
    // servicing (see icache.v's `miss_active` gating on `new_miss`) --
    // the actual freeze (packet_e/pc_f not advancing) is enforced by
    // imem_stall gating the main sequencer's "normal advance" branch
    // below instead, which has no such loop risk.
    wire fetch_advance = running && !done && (stall_cnt==0) && !slot0_is_stop && !bundle_stall;
    assign imem_en = fetch_advance;

    // ================= DMEM bus drive: COMBINATIONAL issue, registered completion =================
    // Timing is the crux of this round's falsifiability claim (see reviews/
    // rvex-round25-*.html section 3): the address phase for a bundle's own
    // memory op must be driven the SAME cycle that bundle commits (issue1
    // below), not one cycle later -- an earlier draft registered the issue
    // (drove d_haddr/d_htrans only at the NEXT edge), which added one whole
    // extra cycle of round-trip latency and silently turned the compiler's
    // load-use-latency=2 contract into an effective latency=3 in this RTL
    // (a 1-bubble-bundle schedule, which the compiler considers safe, would
    // have read stale data -- a false failure, not a caught bug). Re-derived
    // by hand cycle-by-cycle before trusting it; see tb_soc_top.v's
    // load-use directed test, which checks BOTH the 0-bubble (must read
    // stale) and 1-bubble (must read fresh) cases explicitly.
    wire issue1 = running && !done && stall_cnt==0 && !slot0_is_stop && !imem_stall
                  && want_mem && (dm_state == DM_IDLE) && !mem_issued;
    // RMW's write-phase address is driven for exactly the DM_RMW_ISSUE
    // cycle -- see that state's declaration above for why this can't just
    // be folded into the DM_WAIT1->DM_WAIT2 edge. d_hrdata is still valid
    // here (nothing else touched the SRAM between DM_WAIT1 completing and
    // this cycle), so mem_store_c's merge is still correctly computed from
    // the just-read old word.
    wire issue2 = (dm_state == DM_RMW_ISSUE);

    assign d_htrans = (issue1 || issue2) ? 2'b10 : 2'b00;
    assign d_haddr  = issue1 ? dmem_word_addr : (issue2 ? pend_addr : 8'd0);
    // issue1: LDx and the RMW-first-phase (STH/STB) both READ; only a plain
    // STW writes on this phase. issue2 is always the RMW's write-back phase.
    assign d_hwrite = issue1 ? (is_st3 && !is_rmw3) : issue2;
    assign d_hwdata = issue1 ? gr_r2_3 : (issue2 ? mem_store_c : 32'd0);

    always @(posedge clk) begin
        if (reset) begin
            dm_state <= DM_IDLE; pend_is_load <= 1'b0; pend_is_rmw <= 1'b0;
            pend_dst <= 6'd0; pend_op3 <= 7'd0; pend_pos <= 2'd0;
            pend_regval <= 32'd0; pend_addr <= 8'd0;
        end else begin
            case (dm_state)
                DM_IDLE: begin
                    if (issue1) begin
                        pend_op3 <= op3; pend_pos <= dmem_pos; pend_dst <= dst3;
                        pend_is_load <= is_ld3; pend_is_rmw <= is_rmw3;
                        pend_regval  <= gr_r2_3; pend_addr <= dmem_word_addr;
                        dm_state <= DM_WAIT1;
                    end
                end
                DM_WAIT1: begin
                    if (d_hreadyout) begin
                        // NOTE: the actual gr[] write for a completed load is
                        // NOT done here -- see load_complete_now below and
                        // its comment for why (Codex-caught multi-driver bug).
                        dm_state <= pend_is_rmw ? DM_RMW_ISSUE : DM_IDLE;
                    end
                end
                DM_RMW_ISSUE: begin
                    dm_state <= DM_WAIT2; // issue2 combinationally drove the write address this cycle
                end
                DM_WAIT2: begin
                    if (d_hreadyout) dm_state <= DM_IDLE;
                end
                default: dm_state <= DM_IDLE;
            endcase
        end
    end

    // ================= main sequencer: fetch advance + bundle commit =================
    always @(posedge clk) begin
        if (reset) begin
            pc_f <= 8'd0; pc_f_prev <= 8'd0; packet_e <= 128'd0; pc_e <= 8'd0;
            squash <= RESET_SQUASH; stall_cnt <= 12'd0; running <= 1'b0; done <= 1'b0; cycles <= 32'd0;
            mem_issued <= 1'b0;
            for (i=0;i<64;i=i+1) gr[i] <= 32'd0;
            for (i=0;i<8;i=i+1)  br[i] <= 1'b0;
        end else begin
            if (start) running <= 1'b1;
            // Fires independently of stall_cnt/bundle_stall/done: a load's
            // completion is on the DMEM sub-FSM's own clock, not gated by
            // whether THIS cycle's bundle (if any) is frozen -- and must
            // not be dropped even if a STOP bundle immediately following
            // the load already set `done` one cycle earlier (structurally
            // reachable: the load's own commit isn't stalled, so packet_e
            // advances to the STOP bundle right away, which can commit
            // before the load's 1-cycle-later completion). See
            // load_complete_now's declaration above for why this write
            // lives here instead of in the DMEM sub-FSM's always block.
            if (load_complete_now) gr[pend_dst] <= mem_load_c;
            // Set here (top-level, same pattern as load_complete_now above)
            // so it applies regardless of which branch below executes this
            // cycle; the normal-advance branch's own `mem_issued <= 1'b0`
            // (new-bundle reset) is textually LATER in this same always
            // block, so it correctly wins on any cycle both conditions
            // coincide (issue1 firing for the outgoing bundle at the exact
            // edge normal-advance also supersedes it with a new one).
            if (issue1) mem_issued <= 1'b1;
            if (running && !done) begin
                cycles <= cycles + 1;
                if (stall_cnt != 0) begin
                    stall_cnt <= stall_cnt - 1;               // XNOP bubble: whole pipeline frozen
                end else if (slot0_is_stop) begin
                    done <= 1'b1;
                end else if (bundle_stall) begin
                    ; // waiting for the DMEM bus to free up; F/E pipeline frozen, DM sub-FSM above still runs
                end else if (imem_stall) begin
                    ; // I-Cache miss (round 26): whole pipeline frozen, same shape as bundle_stall above;
                      // load_complete_now above is unaffected -- a D-side transaction may keep completing
                      // independently, since it's on a physically separate AXI port from the I-Cache miss
                end else begin
                    // ---- fetch pipeline advance (always runs a cycle ahead) ----
                    pc_f_prev <= pc_f;
                    packet_e  <= imem_rdata;
                    pc_e      <= PC_E_FROM_PC_F ? pc_f : pc_f_prev;
                    mem_issued <= 1'b0; // new bundle incoming, hasn't issued anything yet
                    if (squash != 2'b00) squash <= squash >> 1;

                    //--------------- writeback slot 0 ---------------
                    if (ext0) ;
                    else if (slot0_is_ctrl) begin
                        if (c_wlink && dst0 != 6'd0) gr[dst0] <= c_link;
                    end else if (op0 == `ALU_MTB) begin
                        br[dbr0] <= alu0_c;
                    end else if (is_addcg_divs(op0)) begin
                        if (dst0 != 6'd0) gr[dst0] <= alu0_r; br[dbr0] <= alu0_c;
                    end else if (is_cmp(op0) && dst0 == 6'd0) begin
                        br[dbr0] <= alu0_r[0];
                    end else if (op0 != `OP_NOP && alu0_v && dst0 != 6'd0) begin
                        gr[dst0] <= alu0_r;
                    end

                    //--------------- writeback slot 1 ---------------
                    if (ext1) ;
                    else if (op1 == `ALU_MTB) br[dbr1] <= alu1_c;
                    else if (is_addcg_divs(op1)) begin
                        if (dst1 != 6'd0) gr[dst1] <= alu1_r; br[dbr1] <= alu1_c;
                    end else if (is_cmp(op1) && dst1 == 6'd0) br[dbr1] <= alu1_r[0];
                    else if (op1 != `OP_NOP && dst1 != 6'd0) gr[dst1] <= res1;

                    //--------------- writeback slot 2 ---------------
                    if (ext2) ;
                    else if (op2 == `ALU_MTB) br[dbr2] <= alu2_c;
                    else if (is_addcg_divs(op2)) begin
                        if (dst2 != 6'd0) gr[dst2] <= alu2_r; br[dbr2] <= alu2_c;
                    end else if (is_cmp(op2) && dst2 == 6'd0) br[dbr2] <= alu2_r[0];
                    else if (op2 != `OP_NOP && dst2 != 6'd0) gr[dst2] <= res2;

                    //--------------- slot 3: ALU (non-mem) path, or handled by DM sub-FSM above ----
                    if (ext3) ;
                    else if (want_mem) ; // address issued by the DMEM sub-FSM this same cycle
                    else if (op3 == `ALU_MTB) br[dbr3] <= alu3_c;
                    else if (is_addcg_divs(op3)) begin
                        if (dst3 != 6'd0) gr[dst3] <= alu3_r; br[dbr3] <= alu3_c;
                    end else if (is_cmp(op3) && dst3 == 6'd0) br[dbr3] <= alu3_r[0];
                    else if (op3 != `OP_NOP && dst3 != 6'd0) gr[dst3] <= alu3_r;

                    //--------------- PC / branch / XNOP ---------------
                    if (slot0_is_ctrl && c_xnop) begin
                        stall_cnt <= c_xnop_n;
                        pc_f <= pc_f + 8'd1;
                    end else if (slot0_is_ctrl && c_taken) begin
                        pc_f   <= c_pc_goto;
                        squash <= BRANCH_SQUASH;                // flush already-in-flight wrong-path fetch(es)
                    end else begin
                        pc_f <= pc_f + 8'd1;
                    end
                end
            end
            gr[0] <= 32'd0; // r0 hardwired
        end
    end
endmodule
