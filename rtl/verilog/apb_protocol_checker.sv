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
  localparam             PSTRB_SIZE = (PDATA_SIZE+7)/8;

  logic                  setup_phase,
                         access_phase;
  logic                  dly_psel;
  logic                  dly_penable;
  logic [PADDR_SIZE-1:0] dly_paddr;
  logic                  dly_pwrite;
  logic [PSTRB_SIZE-1:0] dly_pstrb;
  logic [HPROT_SIZE-1:0] dly_pprot;
  logic [PDATA_SIZE-1:0] dly_pwdata;
  logic                  dly_pready;

  int                    watchdog_cnt = WATCHDOG_TIMEOUT;
  int                    errors   = 0;
  int                    warnings = 0;
  int                    infos    = 0;


  //////////////////////////////////////////////////////////////////
  //
  // Message Structure
  //
  localparam int MESSAGE_COUNT   = 23;

  typedef enum int {OFF    =0,
                    INFO   =1,
                    WARNING=2,
                    ERROR  =3,
                    FATAL  =4} severity_t;

  typedef struct {
    severity_t severity;
    string     message;
  } message_t;

  message_t _msg[MESSAGE_COUNT];


  //Default values
  initial
  begin
      _msg[ 0] = '{ERROR  , "PSEL must remain high for the entire transfer"};
      _msg[ 1] = '{ERROR  , "PSEL undefined"};
      _msg[ 2] = '{ERROR  , "PENABLE must be low during Setup phase"};
      _msg[ 3] = '{ERROR  , "PENABLE must be high during Access phase"};
      _msg[ 4] = '{ERROR  , "PENABLE undefined"};
      _msg[ 5] = '{ERROR  , "PADDR must remain stable for the entire transfer"};
      _msg[ 6] = '{ERROR  , "PADDR vs PSTRB misaligned"};
      _msg[ 7] = '{ERROR  , "Misaligned PADDR"};
      _msg[ 8] = '{ERROR  , "PADDR undefined"};
      _msg[ 9] = '{ERROR  , "PWRITE must remain stable for the entire transfer"};
      _msg[10] = '{ERROR  , "PWRITE undefined"};
      _msg[11] = '{WARNING, "PSTRB value non byte/word/dword/..."};
      _msg[12] = '{ERROR  , "PSTRB must remain stable for the entire transfer"};
      _msg[13] = '{ERROR  , "PSTRB undefined"};
      _msg[14] = '{ERROR  , "PPROT must remain stable for the entire transfer"};
      _msg[15] = '{ERROR  , "PPROT undefined"};
      _msg[16] = '{ERROR  , "PWDATA must remain stable during wait states"};
      _msg[17] = '{WARNING, "PWDATA contains 'x'"};
      _msg[18] = '{WARNING, "PWDATA contains 'x'"};
      _msg[19] = '{WARNING, "PRDATA contains 'x'"};
      _msg[20] = '{ERROR  , "PREADY undefined during Access phase"};
      _msg[21] = '{ERROR  , "PSLVERR undefined"};
      _msg[22] = '{FATAL  , "Watchdog expired"};
  end


  //Display message
  task automatic message (input int msg_no);
    severity_t msg_severity;
    string     msg;

    msg_severity = _msg[msg_no].severity;
    msg          = $sformatf ("APB-%0d %s (%m): %s @%0t", msg_no, msg_severity.name(), _msg[msg_no].message, $time);

    case (msg_severity)
      OFF    : ;
      INFO   : $info    (msg);
      WARNING: $warning (msg);
      ERROR  : $error   (msg);
      FATAL  : begin $display("%0d", msg_no); $fatal   (msg); end
    endcase

    case (msg_severity)
      OFF    : ;
      INFO   : infos++;
      WARNING: warnings++;
      ERROR  : errors++;
      FATAL  : ;
    endcase
  endtask : message


  //Set severity level of a message/check
  task set_severity(int msg_no, severity_t severity);
    if (msg_no < MESSAGE_COUNT)
      _msg[msg_no].severity = severity;
  endtask : set_severity

  //Get severity level of a message/check
  function severity_t get_severity(int msg_no);
    if (msg_no < MESSAGE_COUNT)
      return _msg[msg_no].severity;
  endfunction : get_severity


  //////////////////////////////////////////////////////////////////
  //
  // Functions
  //

  //Check if PSTRB has a 'logical' structure. Meaning:
  //1 PSTRB for byte access and in the form of 'h8, 'h4, 'h2, 'h1
  //2 PSTRB's for hword access and in the form of 'hc0, 'hc0, 'h0c, 'h03
  //4 PSTRB's for word access and in the form of 'hf000, 'h0f00, 'h00f0, 'h000f
  //etc
  function pstrb_valid(
    input [PSTRB_SIZE-1:0] pstrb
  );
    logic [PSTRB_SIZE-1:0] mask;
 
    pstrb_valid = 0;

    //create all possible valid/logic PSTRB combinations and check against them
    for (int size  =0; size   <= $clog2(PSTRB_SIZE)  ; size++  )
    for (int offset=0; offset <  PSTRB_SIZE/(2**size); offset++)
    begin
        mask         = ((PSTRB_SIZE'(1) << (PSTRB_SIZE'(1) << size)) -1'h1) << (offset << size);
        pstrb_valid |= (pstrb & mask) == pstrb;
    end
  endfunction : pstrb_valid


  //Check if PSTRB is aligned with PADDR (or vice versa)
  function pstrb_misaligned(
    input [PADDR_SIZE-1:0] paddr,
    input [PSTRB_SIZE-1:0] pstrb
  );
    int                    tr_size;
    logic [PSTRB_SIZE-1:0] mask;
    logic [PADDR_SIZE-1:0] addr_mask,
                           masked_addr;

    tr_size = -1;

    //Determine the size of the transaction based on PSTRB value
    //0 = byte  ( 8bits)
    //1 = hword (16bits)
    //2 = word  (32bits)
    //etc
    for (int size  =0; size   <= $clog2(PSTRB_SIZE)  ; size++  )
    for (int offset=0; offset <  PSTRB_SIZE/(2**size); offset++)
    begin
        mask = ((PSTRB_SIZE'(1) << (PSTRB_SIZE'(1) << size)) -1'h1) << (offset << size);
        if (pstrb == mask)
        begin
            tr_size = size;
            break;
        end
    end

    addr_mask   = {PADDR_SIZE{1'b1}} << tr_size;
    masked_addr = paddr & ~addr_mask;

    pstrb_misaligned = |masked_addr;
  endfunction : pstrb_misaligned



  //////////////////////////////////////////////////////////////////
  //
  // Welcome/Goodbye Tasks
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
    $display ("- Instance: %m");
    $display ("------------------------------------------------------------");
    $display ("- Info    : %0d", infos);
    $display ("- Warnings: %0d", warnings);
    $display ("- Errors  : %0d", errors);
    $display ("------------------------------------------------------------");
  endtask

  initial welcome_msg();


  //////////////////////////////////////////////////////////////////
  //
  // Tasks / Checks
  //

  /*
   * Check PSEL
   */
  task check_psel;
    //PSEL can only go low if the current transfer ended
    if (!PSEL && dly_psel)
      if (!dly_pready)
        message(0);

    //PSEL must not be undefined
    if (PSEL === 1'bx)
      message(1);
  endtask : check_psel


  /*
   * Check PENABLE
   */
  task check_penable;
    //PENABLE must be low during Setup phase
    if (setup_phase)
      if (PENABLE)
        message(2);

    //PENABLE must be high during Access phase
    if (access_phase)
      if (!PENABLE)
        message(3);

    //PENABLE must not be undefined during transfer
    if (PSEL && (PENABLE === 1'bx || PENABLE == 1'bz))
      message(4);
  endtask : check_penable


  /*
   * Check PADDR
   */
  task check_paddr;
    logic [PADDR_SIZE-1:0] addr_mask, masked_addr;

    //PADDR must remain stable during entire transfer
    if (!dly_pready && PADDR !== dly_paddr)
      message(5);

    //PADDR must align with PSTRB
    if (CHECK_PSTRB)
      if (PWRITE)
      begin
          if (pstrb_misaligned(PADDR, PSTRB))
            message(6);
      end
      else
      begin
          addr_mask   = {PADDR_SIZE{1'b1}} << $clog2(PSTRB_SIZE);
          masked_addr = PADDR & ~addr_mask;
          if (|masked_addr)
            message(7);
      end

    //PADDR may not be undefined during transfer
    if (^PADDR === 1'bx)
      message(8);
  endtask : check_paddr


  /*
   * Check PWRITE
   */
  task check_pwrite;
    //HWRITE must remain stable during a burst
    if (!dly_pready && PWRITE !== dly_pwrite)
      message(9);

    //PWRITE may not be undefined during transfer
    if (PWRITE === 1'bx || PWRITE === 1'bz)
      message(10);
  endtask : check_pwrite


  /*
   * Check PSTRB
   */
  task check_pstrb;
    //PSTRB valid?
    if (|PSTRB)
      if (~pstrb_valid(PSTRB))
        message(11);


    //PSTRB must remain stable during entire transfer
    if (!dly_pready && PSTRB !== dly_pstrb)
      message(12);

    //PSTRB may not be undefined during transfer
    if (PWRITE && ^PSTRB === 1'bx)
      message(13);
  endtask : check_pstrb


  /*
   * Check PPROT
   */
  task check_pprot;
    //PPROT must remain stable during entire transfer
    if (!dly_pready && PPROT !== dly_pprot)
      message(14);

    //PPROT may not be undefined during transfer
    if (^PPROT === 1'bx)
      message(15);
  endtask : check_pprot
  

  /*
   * Check PWDATA
   */
  task check_pwdata;
    logic is_x;

    //PWDATA must remain stable during entire transfer
    if (!dly_pready && PWDATA !== dly_pwdata)
      message(16);

    //PWDATA undefined?
    if (CHECK_PSTRB)
    begin
        if (PWRITE)
        begin
            is_x = 1'b0;

            foreach (PSTRB[i])
              is_x |= (^PWDATA[i*8 +: 8] & PSTRB[i]) === 1'bx;

            if (is_x)
              message(17);
        end
    end
    else
    begin
        if (PWRITE && ^PWDATA === 1'bx)
          message(18);
    end
  endtask : check_pwdata


  /*
   * Check PRDATA
   */
  task check_prdata;
    //PRDATA undefined when transfer completes?
    if (PENABLE && PREADY)
      if (^PRDATA === 1'bx)
        message(19);
  endtask : check_prdata


  /*
   * Check PREADY
   */
  task check_pready;
    //PREADY may not contain 'x' when PENABLE is high
    if (PENABLE && (PREADY === 1'bx || PREADY === 1'bz))
      message(20);
  endtask : check_pready


  /*
   * Check PSLVERR
   */
  task check_pslverr;
    //PSLVERR may not be undefined when transfer completes
    if (PENABLE && PREADY)
      if (PSLVERR === 1'bx || PSLVERR === 1'bz)
        message(21);
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
  always @(posedge PCLK) if (PSEL) check_paddr();


  /*
   * Check PWRITE
   */
  always @(posedge PCLK) dly_pwrite <= PWRITE;
  always @(posedge PCLK) if (PSEL) check_pwrite();


  /*
   * Check PSTRB
   */
  always @(posedge PCLK) if (CHECK_PSTRB) dly_pstrb <= PSTRB;
  always @(posedge PCLK) if (CHECK_PSTRB) if (PSEL) check_pstrb();


  /*
   * Check PPROT
   */
  always @(posedge PCLK) if (CHECK_PPROT) dly_pprot <= PPROT;
  always @(posedge PCLK) if (CHECK_PPROT) if (PSEL) check_pprot();
  

  /*
   * Check PWDATA
   */
  always @(posedge PCLK) dly_pwdata <= PWDATA;
  always @(posedge PCLK) if (PSEL) check_pwdata();


  /*
   * Check PRDATA
   */
  always @(posedge PCLK) if (PSEL) check_prdata();


  /*
   * Check PREADY
   */
  always @(posedge PCLK) dly_pready <= PREADY;
  always @(posedge PCLK) if (PSEL) check_pready();


  /*
   * Check PSLVERR
   */
  always @(posedge PCLK) if (CHECK_PSLVERR) if (PSEL) check_pslverr();


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
        message(22);
endmodule : apb_protocol_checker

