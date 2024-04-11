/////////////////////////////////////////////////////////////////////
//   ,------.                    ,--.                ,--.          //
//   |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---.    //
//   |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--'    //
//   |  |\  \ ' '-' '\ '-'  |    |  '--.' '-' ' '-' ||  |\ `--.    //
//   `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---'    //
//                                             `---'               //
//   AHB Bus Protocol Checker                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//             Copyright (C) 2014-2024 ROA Logic BV                //
//             www.roalogic.com                                    //
//                                                                 //
//   This source file may be used and distributed without          //
//   restriction provided that this copyright statement is not     //
//   removed from the file and that any derivative work contains   //
//   the original copyright notice and the associated disclaimer.  //
//                                                                 //
//    This soure file is free software; you can redistribute it    //
//  and/or modify it under the terms of the GNU General Public     //
//  License as published by the Free Software Foundation,          //
//  either version 3 of the License, or (at your option) any later //
//  versions. The current text of the License can be found at:     //
//  http://www.gnu.org/licenses/gpl.html                           //
//                                                                 //
//    This source file is distributed in the hope that it will be  //
//  useful, but WITHOUT ANY WARRANTY; without even the implied     //
//  warranty of MERCHANTABILITY or FITTNESS FOR A PARTICULAR       //
//  PURPOSE. See the GNU General Public License for more details.  //
//                                                                 //
/////////////////////////////////////////////////////////////////////


module ahb_protocol_checker
import ahb3lite_pkg::*;
#(
  parameter HADDR_SIZE       = 32,       //HADDR bus width
  parameter HDATA_SIZE       = 32,       //HDATA bus width

  parameter CHECK_HPROT      = 1,        //1: check HPROT signal
                                         //0: do not check HPROT signal
  parameter WATCHDOG_TIMEOUT = 128,      //number of cycles before watchdog triggers
                                         //WATCHDOG_TIMEOUT==0 disables watchdog
  parameter STOP_ON_WATCHDOG = 1         //1: stop simulation if watchdog triggers
                                         //0: do not stop simulation if watchdog triggers
)
(
  //AHB Interface
  input                        HRESETn,
  input                        HCLK,

  input                        HSEL,
  input      [HTRANS_SIZE-1:0] HTRANS,
  input      [HSIZE_SIZE -1:0] HSIZE,
  input      [HBURST_SIZE-1:0] HBURST,
  input      [HPROT_SIZE -1:0] HPROT,
  input                        HWRITE,
  input                        HMASTLOCK,
  input      [HADDR_SIZE -1:0] HADDR,
  input      [HDATA_SIZE -1:0] HWDATA,
  input      [HDATA_SIZE -1:0] HRDATA,
  input                        HREADY,
  input                        HRESP
);
  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  logic                   is_burst,
                          last_burst_beat;
  logic                   dly_hsel;
  logic [HTRANS_SIZE-1:0] curr_htrans,
                          prev_htrans,
                          dly_htrans;
  logic [HSIZE_SIZE -1:0] prev_hsize,
                          dly_hsize;
  logic [HBURST_SIZE-1:0] prev_hburst,
                          dly_hburst;
  logic [HPROT_SIZE -1:0] dly_hprot;
  logic                   prev_hwrite,
                          dly_hwrite;
  logic                   dly_hmastlock;
  logic [HADDR_SIZE -1:0] prev_haddr,
                          dly_haddr;
  logic [HDATA_SIZE -1:0] dly_hwdata,
                          dly_hrdata;
  logic                   dly_hready,
                          dly_hresp;

  integer                 burst_cnt,
                          watchdog_cnt;

  integer                 errors   = 0;
  integer                 warnings = 0;


  //////////////////////////////////////////////////////////////////
  //
  // Functions
  //
  function automatic [HADDR_SIZE-1:0] hsize2adrmask (input [HSIZE_SIZE-1:0] hsize);
    case (hsize)
      HSIZE_B8   : hsize2adrmask = {HADDR_SIZE{1'b1}};
      HSIZE_B16  : hsize2adrmask = {HADDR_SIZE{1'b1}} << 1;
      HSIZE_B32  : hsize2adrmask = {HADDR_SIZE{1'b1}} << 2;
      HSIZE_B64  : hsize2adrmask = {HADDR_SIZE{1'b1}} << 3;
      HSIZE_B128 : hsize2adrmask = {HADDR_SIZE{1'b1}} << 4;
      HSIZE_B256 : hsize2adrmask = {HADDR_SIZE{1'b1}} << 5;
      HSIZE_B512 : hsize2adrmask = {HADDR_SIZE{1'b1}} << 6;
      HSIZE_B1024: hsize2adrmask = {HADDR_SIZE{1'b1}} << 7;
    endcase
  endfunction : hsize2adrmask


  function automatic [HDATA_SIZE-1:0] datamask (
    input [HADDR_SIZE-1:0] haddr,
    input [HSIZE_SIZE-1:0] hsize
  );
    int offset;

    offset = 8 * haddr[0 +: $clog2(HDATA_SIZE/8)];

    datamask = {HDATA_SIZE{1'b0}};
    case (hsize)
      HSIZE_B8   : datamask |= {   8{1'b1}} << offset;
      HSIZE_B16  : datamask |= {  16{1'b1}} << offset;
      HSIZE_B32  : datamask |= {  32{1'b1}} << offset;
      HSIZE_B64  : datamask |= {  64{1'b1}} << offset;
      HSIZE_B128 : datamask |= { 128{1'b1}} << offset;
      HSIZE_B256 : datamask |= { 256{1'b1}} << offset;
      HSIZE_B512 : datamask |= { 512{1'b1}} << offset;
      HSIZE_B1024: datamask |= {1024{1'b1}} << offset;
    endcase
  endfunction : datamask


  //////////////////////////////////////////////////////////////////
  //
  // Tasks
  //
  task welcome_msg();
    $display("\n\n");
    $display ("------------------------------------------------------------");
    $display (" ,------.                    ,--.                ,--.       ");
    $display (" |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---. ");
    $display (" |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--' ");
    $display (" |  |\\  \\ ' '-' '\\ '-'  |    |  '--.' '-' ' '-' ||  |\\ `--. ");
    $display (" `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---' ");
    $display ("- AHB Protocol Checker ------------------  `---'  ----------");
    $display ("- Instance: %m");
    $display ("------------------------------------------------------------");
    $display ("\n");
  endtask


  task goodbye_msg();
    $display("\n\n");
    $display ("------------------------------------------------------------");
    $display (" ,------.                    ,--.                ,--.       ");
    $display (" |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---. ");
    $display (" |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--' ");
    $display (" |  |\\  \\ ' '-' '\\ '-'  |    |  '--.' '-' ' '-' ||  |\\ `--. ");
    $display (" `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---' ");
    $display ("- AHB Protocol Checker ------------------  `---'  ----------");
    $display ("------------------------------------------------------------");
    $display ("  Errors: %0d", errors);
    $display ("------------------------------------------------------------");
  endtask


  task ahb_error;
    input string msg;
    
    $error   ("AHB ERROR   (%m): %s @%0t", msg, $time);
    errors++;
  endtask : ahb_error

  task ahb_warning;
    input string msg;

    $warning ("AHB WARNING (%m): %s @%0t", msg, $time);
    warnings++;
  endtask : ahb_warning


  /*
   * Check HTRANS
   */
  task check_htrans;
    if (curr_htrans == HTRANS_IDLE)
    begin
        //IDLE after BUSY only during a undefined length burst
        if (is_burst && prev_htrans === HTRANS_BUSY && dly_hburst !== HBURST_INCR)
        begin
            ahb_error("Illegal termination of a fixed length burst");
        end

        //IDLE only when non-incrementing burst terminates
        if (is_burst && !last_burst_beat && dly_hburst !== HBURST_INCR)
        begin
            ahb_error ("Expected HTRANS=SEQ or BUSY, seen IDLE instead");
        end
    end

    if (curr_htrans === HTRANS_BUSY)
    begin
        //BUSY only during a burst
        if (!is_burst)
          ahb_error ("HTRANS=BUSY, but not a burst transfer");
    end

    if (curr_htrans === HTRANS_NONSEQ)
    begin
        //NONSEQ after BUSY only during undefined length burst
        if (is_burst && prev_htrans === HTRANS_BUSY && dly_hburst !== HBURST_INCR)
        begin
            ahb_error ("Illegal termination of a fixed length burst");
        end

        //NONSEQ only when burst terminates
        if (is_burst && !last_burst_beat && dly_hburst !== HBURST_INCR)
        begin
            ahb_error ("Expected HTRANS=SEQ or BUSY, seen NONSEQ instead");
        end
    end

    if (curr_htrans == HTRANS_SEQ)
    begin
        //SEQ only during a burst
        if (!is_burst || last_burst_beat)
          ahb_error ("HTRANS=SEQ, but not a burst transfer");
    end


    //HTRANS must remain stable when slave not ready
    if (!dly_hready && curr_htrans !== dly_htrans)
    begin
        ahb_error ("HTRANS must remain stable during wait states");
    end

    //HTRANS may not contain 'x' during transactions
    if (^curr_htrans === 1'bx)
    begin
         ahb_error ("HTRANS undefined");
    end
  endtask : check_htrans



  /*
   * Check HSIZE
   */
  task check_hsize;
    //Check HSIZE versus data bus width
    logic out_of_range;

    case (HSIZE)
       HSIZE_B1024: out_of_range = HDATA_SIZE < 1024 ? 1'b1 : 1'b0;
       HSIZE_B512 : out_of_range = HDATA_SIZE <  512 ? 1'b1 : 1'b0;
       HSIZE_B256 : out_of_range = HDATA_SIZE <  256 ? 1'b1 : 1'b0;
       HSIZE_B128 : out_of_range = HDATA_SIZE <  128 ? 1'b1 : 1'b0;
       HSIZE_DWORD: out_of_range = HDATA_SIZE <   64 ? 1'b1 : 1'b0;
       HSIZE_WORD : out_of_range = HDATA_SIZE <   32 ? 1'b1 : 1'b0;
       HSIZE_HWORD: out_of_range = HDATA_SIZE <   16 ? 1'b1 : 1'b0;
       default    : out_of_range = 1'b0;
    endcase

    if (HSEL && out_of_range)
    begin
        $error ("AHB ERROR (%m): Illegal HSIZE (%0b) @%0t", HSIZE, $time);
    end

    //HSIZE must remain stable during a burst
    if (is_burst && !last_burst_beat && HSIZE !== dly_hsize)
    begin
        ahb_error ("HSIZE must remain stable during burst");
    end

    //HSIZE must remain stable when slave not ready
    if (!dly_hready && HSIZE !== dly_hsize)
    begin
        ahb_error ("HSIZE must remain stable during wait states");
    end

    //HSIZE may not contain 'x' during transactions
    if (HSEL && ^HSIZE === 1'bx)
    begin
         ahb_error ("HSIZE undefined");
    end
  endtask : check_hsize



  /*
   * Check HBURST
   */
  task check_hburst;
    //HBURST must remain stable during a burst
    //1st line checks fixed length burst
    //2nd line checks undefinite (INCR) burst
    if ( (is_burst && !last_burst_beat && dly_hburst !== HBURST_INCR && HBURST !== dly_hburst) ||
         (dly_hburst === HBURST_INCR && HTRANS !== HTRANS_IDLE && HTRANS !== HTRANS_NONSEQ && HBURST !== dly_hburst) )
    begin
        ahb_error ("HBURST must remain stable during burst");
    end

    //HBURST must remain stable when slave not ready
    if (!dly_hready && HBURST !== dly_hburst)
    begin
        ahb_error ("HBURST must remain stable during wait states");
    end

    //HBURST may not contain 'x' during transactions
    if (curr_htrans !== HTRANS_IDLE && ^HBURST === 1'bx)
    begin
         ahb_error ("HBURST undefined");
    end
  endtask : check_hburst


  /*
   * Check HPROT
   */
  task check_hprot;
    //HPROT must remain stable during a burst
    if (is_burst && !last_burst_beat && HPROT !== dly_hprot)
    begin
        ahb_error ("HPROT must remain stable during burst");
    end

    //HPROT must remain stable when slave not ready
    if (!dly_hready && HPROT !== dly_hprot)
    begin
        ahb_error ("HPROT must remain stable during wait states");
    end

    //HPROT may not contain 'x' during transactions
    if (curr_htrans !== HTRANS_IDLE && ^HPROT === 1'bx)
    begin
        ahb_error("HPROT undefined");
    end
  endtask : check_hprot
  

  /*
   * Check HWRITE
   */
  task check_hwrite;
    //HWRITE must remain stable during a burst
    if (is_burst && !last_burst_beat && HWRITE !== dly_hwrite)
    begin
        ahb_error ("HWRITE must remain stable during burst");
    end

    //HWRITE must remain stable when slave not ready
    if (!dly_hready && HWRITE !== dly_hwrite)
    begin
        ahb_error ("HWRITE must remain stable during wait states");
    end

    //HWRITE may not contain 'x' during transactions
    if (curr_htrans !== HTRANS_IDLE && (HWRITE === 1'bx || HWRITE === 1'bz))
    begin
         ahb_error ("HWRITE undefined");
    end
  endtask : check_hwrite



  /*
   * Check HADDR
   */
  task check_haddr;
    //HADDR should increase by HSIZE during bursts (wrap for wrapping-bursts)
    logic incr_haddr;
    logic [HADDR_SIZE-1:0] norm_addr,
                           nxt_addr;

    //HADDR should be an HSIZE aligned address
    if (curr_htrans !== HTRANS_IDLE && |(HADDR & ~hsize2adrmask(HSIZE)))
    begin
        ahb_error ("HADDR not aligned with HSIZE");
    end

    //normalize address
    case (prev_hsize)
       HSIZE_B1024: norm_addr = prev_haddr >> 7;
       HSIZE_B512 : norm_addr = prev_haddr >> 6;
       HSIZE_B256 : norm_addr = prev_haddr >> 5;
       HSIZE_B128 : norm_addr = prev_haddr >> 4;
       HSIZE_DWORD: norm_addr = prev_haddr >> 3;
       HSIZE_WORD : norm_addr = prev_haddr >> 2;
       HSIZE_HWORD: norm_addr = prev_haddr >> 1;
       default    : norm_addr = prev_haddr;
    endcase

    //next address
    nxt_addr = norm_addr +1;

    //handle normalized wrap
    case (prev_hburst)
       HBURST_WRAP4 : nxt_addr = {norm_addr[HADDR_SIZE-1:2],nxt_addr[1:0]};
       HBURST_WRAP8 : nxt_addr = {norm_addr[HADDR_SIZE-1:3],nxt_addr[2:0]}; 
       HBURST_WRAP16: nxt_addr = {norm_addr[HADDR_SIZE-1:4],nxt_addr[3:0]};
    endcase

    //move into correct address range
    case (prev_hsize)
       HSIZE_B1024: nxt_addr = nxt_addr << 7;
       HSIZE_B512 : nxt_addr = nxt_addr << 6;
       HSIZE_B256 : nxt_addr = nxt_addr << 5;
       HSIZE_B128 : nxt_addr = nxt_addr << 4;
       HSIZE_DWORD: nxt_addr = nxt_addr << 3;
       HSIZE_WORD : nxt_addr = nxt_addr << 2;
       HSIZE_HWORD: nxt_addr = nxt_addr << 1;
       default    : ;
    endcase

    if (is_burst && !last_burst_beat && HREADY && HADDR !== nxt_addr)
    begin
        $error ("AHB ERROR (%m): Seen HADDR=%0x, but expected %0x @%0t", HADDR, nxt_addr, $time);
    end

    //HADDR must remain stable when slave not ready
    if (!dly_hready && HADDR !== dly_haddr)
    begin
        ahb_error ("HADDR must remain stable during wait states");
    end

    //HADDR may not contain 'x' during transactions
    if (curr_htrans !== HTRANS_IDLE && ^HADDR === 1'bx)
    begin
         ahb_error ("HADDR undefined");
    end
  endtask : check_haddr



  /*
   * Check HWDATA
   */
  task check_hwdata;
    //HWDATA must remain stable when slave not ready
    if (!dly_hready && dly_hwrite && HWDATA !== dly_hwdata)
    begin
        ahb_error ("HWDATA must remain stable during wait states");
    end

    //HWDATA contains 'x'?
    if (prev_htrans !== HTRANS_IDLE && prev_hwrite &&
        ^(HWDATA & datamask(prev_haddr, prev_hsize) === 1'bx))
    begin
         ahb_warning ("HWDATA contains 'x'");
    end

  endtask : check_hwdata



  /*
   * Check slave response
   */
  task check_slave_response;
    //Always zero-wait-state response to IDLE
    if ( (prev_htrans === HTRANS_IDLE) && (!HREADY || (HRESP !== HRESP_OKAY)) )
    begin
        $error ("AHB ERROR (%m): Slave must provide a zero wait state response to IDLE transfers (HREADY=%0b, HRESP=%0b) @%0t", HREADY, HRESP, $time);
    end

    //HREADY may not contain 'x'
    if (HREADY === 1'bx || HREADY === 1'bz)
    begin
         ahb_error ("HREADY undefined");
    end

    //always zero-wait-state response to BUSY
    //unless slave already inserted wait-states
    if ( (prev_htrans === HTRANS_BUSY) && dly_hready && (!HREADY || (HRESP !== HRESP_OKAY)) )
    begin
        $error ("AHB ERROR (%m): Slave must provide a zero wait state response to BUSY transfers (HREADY=%0b, HRESP=%0b) @%0t", HREADY, HRESP, $time);
    end

    //HRESP may not contain 'x'
    if (HREADY && (HRESP === 1'bx || HRESP === 1'bz))
    begin
         ahb_error ("HRESP undefined");
    end


    //ERROR is a 2 cycle response
    if ( (( HREADY      && HRESP    ) && !(!dly_hready && dly_hresp)) ||
         ((!dly_hready  && dly_hresp) && !( HREADY     && HRESP    )) )
    begin
        $error ("AHB ERROR (%m): Incorrect ERROR sequence @%0t", $time);
    end
  endtask : check_slave_response


  //////////////////////////////////////////////////////////////////
  //
  // Module Body
  //


  /*
   * Check HTRANS
   */
  assign curr_htrans = !HSEL ? HTRANS_IDLE : HTRANS;

  always @(posedge HCLK,negedge HRESETn)
    if (!HRESETn) dly_htrans <= HTRANS_IDLE;
    else          dly_htrans <= curr_htrans;


  always @(posedge HCLK, negedge HRESETn)
    if      (!HRESETn) prev_htrans <= HTRANS_IDLE;
    else if ( HREADY ) prev_htrans <= curr_htrans;


  always @(posedge HCLK,negedge HRESETn)
    if      (!HRESETn) is_burst <= 1'b0;
    else if ( HREADY )
    begin
        if      ( curr_htrans === HTRANS_IDLE   ) is_burst <= 1'b0;
        else if ( HTRANS      === HTRANS_NONSEQ &&
                  HBURST      !== HBURST_SINGLE ) is_burst <= 1'b1;
        else if ( HBURST      === HTRANS_NONSEQ &&
                  HBURST      === HBURST_SINGLE ) is_burst <= 1'b0; //terminated INCR burst
        else if ( last_burst_beat               ) is_burst <= 1'b0; //terminated regular burst
    end

  always @(posedge HCLK) check_htrans();


  /*
   * Check HSIZE
   */
  always @(posedge HCLK)
    if (HREADY) prev_hsize <= HSIZE;

  always @(posedge HCLK) dly_hsize <= HSIZE;
  always @(posedge HCLK) check_hsize();


  /*
   * Check HBURST
   */
  always @(posedge HCLK)
    if (HREADY)
    begin
        if (curr_htrans === HTRANS_NONSEQ)
        begin
            case (HBURST)
               HBURST_WRAP4 : burst_cnt <=  3; // 4
               HBURST_INCR4 : burst_cnt <=  3; // 4
               HBURST_WRAP8 : burst_cnt <=  7; // 8
               HBURST_INCR8 : burst_cnt <=  7; // 8
               HBURST_WRAP16: burst_cnt <= 15; //16
               HBURST_INCR16: burst_cnt <= 15; //16
               default      : burst_cnt <= -1;
            endcase
        end
        else if (curr_htrans === HTRANS_SEQ)
        begin
            burst_cnt <= burst_cnt -1;
        end
    end

  assign last_burst_beat = ~|burst_cnt;


  always @(posedge HCLK)
    if (HREADY) prev_hburst <= HBURST;

  always @(posedge HCLK) dly_hburst <= HBURST;
  always @(posedge HCLK) check_hburst();


  /*
   * Check HPROT
   */
  always @(posedge HCLK) if (CHECK_HPROT) dly_hprot <= HPROT;
  always @(posedge HCLK) if (CHECK_HPROT) check_hprot();
  

  /*
   * Check HADDR
   */
  always @(posedge HCLK)
    if (HREADY) prev_haddr <= HADDR;

  always @(posedge HCLK) dly_haddr <= HADDR;
  always @(posedge HCLK) check_haddr();


  /*
   * Check HWDATA
   */
  always @(posedge HCLK) dly_hwdata <= HWDATA;
  always @(posedge HCLK) check_hwdata();


  /*
   * Check HWRITE
   */
  always @(posedge HCLK)
    if (HREADY) prev_hwrite <= HWRITE;

  always @(posedge HCLK) dly_hwrite <= HWRITE;
  always @(posedge HCLK) check_hwrite();


  /*
   * Check Slave response
   */
  always @(posedge HCLK) dly_hready <= HREADY;
  always @(posedge HCLK) dly_hresp  <= HRESP;
  always @(posedge HCLK) check_slave_response();



   /*
    * Watchdog
    */
  always @(posedge HCLK, negedge HRESETn)
    if      (!HRESETn) watchdog_cnt <= WATCHDOG_TIMEOUT;
    else if ( HREADY ) watchdog_cnt <= WATCHDOG_TIMEOUT;
    else               watchdog_cnt <= watchdog_cnt -1'h1;

  always @(posedge HCLK)
    if (WATCHDOG_TIMEOUT)
      if (~|watchdog_cnt)
      begin
          ahb_error ("Watchdog expired");
          
          if (STOP_ON_WATCHDOG) $stop();
      end
endmodule : ahb_protocol_checker

