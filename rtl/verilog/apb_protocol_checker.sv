/////////////////////////////////////////////////////////////////////
//   ,------.                    ,--.                ,--.          //
//   |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---.    //
//   |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--'    //
//   |  |\  \ ' '-' '\ '-'  |    |  '--.' '-' ' '-' ||  |\ `--.    //
//   `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---'    //
//                                             `---'               //
//   APB Bus Protocol Checker                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//             Copyright (C) 2024 ROA Logic BV                     //
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


module apb_protocol_checker
import ahb3lite_pkg::*;
#(
  parameter PADDR_SIZE       = 32,       //HADDR bus width
  parameter PDATA_SIZE       = 32,       //HDATA bus width

  parameter CHECK_PSTRB      = 1,        //1: check PSTRB signal
                                         //0: do not check PSTRB signal
  parameter CHECK_PPROT      = 1,        //1: check PPROT signal
                                         //0: do not check PPROT signal
  parameter CHECK_PSLVERR    = 1,        //1: check PSLVERR signal
                                         //0: do not check PSLVERR signal
  parameter WATCHDOG_TIMEOUT = 128,      //number of cycles before watchdog triggers
                                         //WATCHDOG_TIMEOUT==0 disables watchdog
  parameter STOP_ON_WATCHDOG = 1         //1: stop simulation if watchdog triggers
                                         //0: do not stop simulation if watchdog triggers
)
(
  //AHB Interface
  input                    PRESETn,
  input                    PCLK,

  input                    PSEL,
  input                    PENABLE,
  input [PADDR_SIZE  -1:0] PADDR,
  input                    PWRITE,
  input [PDATA_SIZE/8-1:0] PSTRB,
  input [             2:0] PPROT,
  input [PDATA_SIZE  -1:0] PWDATA,
  input [PDATA_SIZE  -1:0] PRDATA,
  input                    PREADY,
  input                    PSLVERR
);
  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  logic                    setup_phase,
                           access_phase;
  logic                    dly_psel;
  logic                    dly_penable;
  logic [PADDR_SIZE  -1:0] dly_paddr;
  logic                    dly_pwrite;
  logic [PDATA_SIZE/8-1:0] dly_pstrb;
  logic [HPROT_SIZE  -1:0] dly_pprot;
  logic [PDATA_SIZE  -1:0] dly_pwdata;
  logic                    dly_pready;

  integer                  watchdog_cnt;
  integer                  errors   = 0;
  integer                  warnings = 0;


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
    $display ("- APB Protocol Checker ------------------  `---'  ----------");
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
    $display ("- APB Protocol Checker ------------------  `---'  ----------");
    $display ("------------------------------------------------------------");
    $display ("  Errors: %0d", errors);
    $display ("------------------------------------------------------------");
  endtask


  task apb_error;
    input string msg;
    
    $error   ("APB ERROR   (%m): %s @%0t", msg, $time);
    errors++;
  endtask : apb_error


  task apb_warning;
    input string msg;
    
    $warning ("APB WARNING (%m): %s @%0t", msg, $time);
    warnings++;
  endtask : apb_warning


  /*
   * Check PSEL
   */
  task check_psel;
    //PSEL can only go low if the current transfer ended
    if (!PSEL && dly_psel)
        if (!dly_pready) apb_error ("PSEL must remain high for the entire transfer");

    //PSEL must not be undefined
    if (PSEL === 1'bx)
    begin
         apb_error ("PSEL undefined");
    end
  endtask : check_psel


  /*
   * Check PENABLE
   */
  task check_penable;
    //PENABLE must be low during Setup phase
    if (setup_phase)
      if (PENABLE) apb_error ("PENABLE must be low during Setup phase");

    //PENABLE must be high during Access phase
    if (access_phase)
      if (!PENABLE) apb_error ("PENABLE must be high during Access phase");

    //PENABLE must not be undefined during transfer
    if (PSEL && (PENABLE === 1'bx || PENABLE == 1'bz))
    begin
         apb_error ("PENABLE undefined");
    end
  endtask : check_penable


  /*
   * Check PADDR
   */
  task check_paddr;
    //PADDR must remain stable during entire transfer
    if (PSEL && !dly_pready && PADDR !== dly_paddr)
    begin
        apb_error ("PADDR must remain stable for the entire transfer");
    end

    //PADDR may not be undefined during transfer
    if (PSEL && ^PADDR === 1'bx)
    begin
         apb_error ("PADDR undefined");
    end
  endtask : check_paddr


  /*
   * Check PWRITE
   */
  task check_pwrite;
    //HWRITE must remain stable during a burst
    if (PSEL && !dly_pready && PWRITE !== dly_pwrite)
    begin
        apb_error ("PWRITE must remain stable for the entire transfer");
    end

    //PWRITE may not be undefined during transfer
    if (PSEL && (PWRITE === 1'bx || PWRITE === 1'bz))
    begin
         apb_error ("PWRITE undefined");
    end
  endtask : check_pwrite


  /*
   * Check PSTRB
   */
  task check_pstrb;
    //PSTRB must remain stable during entire transfer
    if (PSEL && !dly_pready && PSTRB !== dly_pstrb)
    begin
        apb_error ("PSTRB must remain stable for the entire transfer");
    end

    //PSTRB may not be undefined during transfer
    if (PSEL && PWRITE && ^PSTRB === 1'bx)
    begin
         apb_error ("PSTRB undefined");
    end
  endtask : check_pstrb


  /*
   * Check PPROT
   */
  task check_pprot;
    //PPROT must remain stable during entire transfer
    if (PSEL && !dly_pready && PPROT !== dly_pprot)
    begin
        apb_error ("PPROT must remain stable for the entire transfer");
    end

    //PPROT may not be undefined during transfer
    if (PSEL && ^PPROT === 1'bx)
    begin
         apb_error ("PPROT undefined");
    end
  endtask : check_pprot
  

  /*
   * Check PWDATA
   */
  task check_pwdata;
    //PWDATA must remain stable during entire transfer
    if (PSEL && !dly_pready && PWDATA !== dly_pwdata)
    begin
        apb_error ("PWDATA must remain stable during wait states");
    end

    //PWDATA undefined?
    if (PSEL && PWRITE && ^PWDATA === 1'bx)
    begin
         apb_warning ("PWDATA contains 'x'");
    end
  endtask : check_pwdata


  /*
   * Check PRDATA
   */
  task check_prdata;
    //PRDATA undefined when transfer completes?
    if (PSEL && PENABLE && PREADY)
      if (^PRDATA === 1'bx) apb_warning ("PRDATA contains 'x'");
  endtask : check_prdata


  /*
   * Check PREADY
   */
  task check_pready;
    //PREADY may not contain 'x' when PENABLE is high
    if (PENABLE && (PREADY === 1'bx || PENABLE === 1'bz))
    begin
         apb_error ("PREADY undefined during Access phase");
    end
  endtask : check_pready


  /*
   * Check PSLVERR
   */
  task check_pslverr;
    //PSLVERR may not be undefined when transfer completes
    if (PSEL && PENABLE && PREADY)
      if (PSLVERR === 1'bx || PSLVERR === 1'bz) apb_error ("PSLVERR undefined");
  endtask : check_pslverr



  //////////////////////////////////////////////////////////////////
  //
  // Module Body
  //

  /*
   * Phase
   */
  assign setup_phase = (PSEL & !dly_psel                ) |
                       (PSEL &  dly_penable & dly_pready);

  always @(posedge PCLK, negedge PRESETn)
    if      (!PRESETn    ) access_phase <= 1'b0;
    else if ( setup_phase) access_phase <= 1'b1;
    else if ( PREADY     ) access_phase <= 1'b0;


  /*
   * Check PSEL
   */
  always @(posedge PCLK, negedge PRESETn)
    if (!PRESETn) dly_psel <= 1'b0;
    else          dly_psel <= PSEL;

  always @(posedge PCLK) check_psel();


  /*
   * Check PENABLE
   */
  always @(posedge PCLK) dly_penable <= PENABLE;
  always @(posedge PCLK) check_penable();


  /*
   * Check PADDR
   */
  always @(posedge PCLK) dly_paddr <= PADDR;
  always @(posedge PCLK) check_paddr();


  /*
   * Check PWRITE
   */
  always @(posedge PCLK) dly_pwrite <= PWRITE;
  always @(posedge PCLK) check_pwrite();


  /*
   * Check PSTRB
   */
  always @(posedge PCLK) if (CHECK_PSTRB) dly_pstrb <= PSTRB;
  always @(posedge PCLK) if (CHECK_PSTRB) check_pstrb();


  /*
   * Check PPROT
   */
  always @(posedge PCLK) if (CHECK_PPROT) dly_pprot <= PPROT;
  always @(posedge PCLK) if (CHECK_PPROT) check_pprot();
  

  /*
   * Check PWDATA
   */
  always @(posedge PCLK) dly_pwdata <= PWDATA;
  always @(posedge PCLK) check_pwdata();


  /*
   * Check PRDATA
   */
  always @(posedge PCLK) check_prdata();


  /*
   * Check PREADY
   */
  always @(posedge PCLK) dly_pready <= PREADY;
  always @(posedge PCLK) check_pready();


  /*
   * Check PSLVERR
   */
  always @(posedge PCLK) if (CHECK_PSLVERR) check_pslverr();


   /*
    * Watchdog
    */
  always @(posedge PCLK, negedge PRESETn)
    if      (!PRESETn           ) watchdog_cnt <= WATCHDOG_TIMEOUT;
    else if (!PSEL              ) watchdog_cnt <= WATCHDOG_TIMEOUT;
    else if ( PENABLE && PREADY ) watchdog_cnt <= WATCHDOG_TIMEOUT;
    else                          watchdog_cnt <= watchdog_cnt -1'h1;


  always @(posedge PCLK)
    if (WATCHDOG_TIMEOUT)
      if (~|watchdog_cnt)
      begin
          apb_error ("Watchdog expired");
          
          if (STOP_ON_WATCHDOG) $stop();
      end
endmodule : apb_protocol_checker

