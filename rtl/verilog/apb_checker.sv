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


/*
 * APB Revisions:                      CommonRef  Status    Changes
 *-------------------------------------------------------------
 * APB Specification Rev E    APB                 obsolete
 * AMBA 2 APB Specification (Issue A)  APB2       active
 * AMBA 3 APB Specification (ISSUE B)  APB3       active    PREADY
 *                                                          PSLVERR
 * AMBA APB Specification (ISSUE C)    APB4       active    PPROT
 *                                                          PSTRB
 * AMBA APB Specification (ISSUE D)    APB5       active    PWAKEUP
 *                                                          PAUSER
 *                                                          PWUSER
 *                                                          PRUSER
 *                                                          PBUSER
 *
 * The checker use the following defines to specify the ABP version
 * APB_VERSION_APB2
 * APB_VERSION_APB3
 * APB_VERSION_APB4
 * APB_VERSION_APB5
 */


/*
 * Relationship between message number and rule number
 * #Rule = message-number +1
 * example msgno=0 results in APB-1
 *         msgno=1 results in APB-2
 */

//APB5 implies APB4 and below
`ifdef APB_VERSION_APB5
    `ifndef APB_VERSION_APB4
        `define APB_VERSION_APB4
    `endif
`endif


//APB4 implies APB3 and below
`ifdef APB_VERSION_APB4
    `ifndef APB_VERSION_APB3
        `define APB_VERSION_APB3
    `endif
`endif


module apb_checker
import ahb3lite_pkg::*;
#(
  parameter int ADDR_WIDTH       = 32,       //PADDR bus width
  parameter int DATA_WIDTH       = 32,       //PRDATA/PWDATA bus width

  parameter int USER_REQ_WIDTH   = 0,
  parameter int USER_DATA_WIDTH  = 0,
  parameter int USER_RESP_WIDTH  = 0,

  parameter int CHECK_PSTRB      = 1,        //1: check PSTRB signal
                                             //0: do not check PSTRB signal
  parameter int CHECK_PPROT      = 1,        //1: check PPROT signal
                                             //0: do not check PPROT signal
  parameter int CHECK_PSLVERR    = 1,        //1: check PSLVERR signal
                                             //0: do not check PSLVERR signal
  parameter int WATCHDOG_TIMEOUT = 128       //number of cycles before watchdog triggers
                                             //WATCHDOG_TIMEOUT==0 disables watchdog
)
(
  //AHB Interface
  input                        PRESETn,
  input                        PCLK,

  input                        PSEL,
  input                        PENABLE,
  input [ADDR_WIDTH      -1:0] PADDR,
  input                        PWRITE,
`ifdef APB_VERSION_APB4
  input [(DATA_WIDTH+7)/8-1:0] PSTRB,
  input [                 2:0] PPROT,
`endif
  input [DATA_WIDTH      -1:0] PWDATA,
  input [DATA_WIDTH      -1:0] PRDATA

`ifdef APB_VERSION_APB3
  ,
  input                        PREADY,
  input                        PSLVERR
`endif

`ifdef APB_VERSION_APB5
  ,
  input                        PWAKEUP,
  input [USER_REQ_WIDTH  -1:0] PAUSER,
  input [USER_DATA_WIDTH -1:0] PRUSER,
                               PWUSER,
  input [USER_RESP_WIDTH -1:0] PBUSER
`endif
);
  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  localparam                  PSTRB_SIZE = (DATA_WIDTH+7)/8;

  logic                       setup_phase,
                              access_phase;
  logic                       dly_psel;
  logic                       dly_penable;
  logic [ADDR_WIDTH     -1:0] dly_paddr;
  logic                       dly_pwrite;
  logic [PSTRB_SIZE     -1:0] dly_pstrb;
  logic [                2:0] dly_pprot;
  logic [DATA_WIDTH     -1:0] dly_pwdata;
  logic                       dly_pready;
  logic                       dly_pwakeup;
  logic [USER_REQ_WIDTH -1:0] dly_pauser;
  logic [USER_DATA_WIDTH-1:0] dly_pwuser,
                              dly_pruser;
  logic [USER_RESP_WIDTH-1:0] dly_pbuser;


  int                         watchdog_cnt = WATCHDOG_TIMEOUT;
  int                         errors   = 0;
  int                         warnings = 0;
  int                         infos    = 0;


  //////////////////////////////////////////////////////////////////
  //
  // Message Structure
  //
  localparam int MESSAGE_COUNT   = 43;

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
      _msg[ 6] = '{ERROR  , "PADDR versus PSTRB misaligned"};
      _msg[38] = '{WARNING, "PADDR should be max 32 bits"};
      _msg[ 7] = '{ERROR  , "PADDR should be aligned to DATA_WIDTH"};
      _msg[ 8] = '{ERROR  , "PADDR undefined"};
      _msg[ 9] = '{ERROR  , "PWRITE must remain stable for the entire transfer"};
      _msg[10] = '{ERROR  , "PWRITE undefined"};
      _msg[11] = '{WARNING, "PSTRB value non byte/word/dword/..."};
      _msg[12] = '{ERROR  , "PSTRB must remain stable for the entire transfer"};
      _msg[13] = '{ERROR  , "PSTRB undefined"};
      _msg[37] = '{ERROR  , "PSTRB must be low during read transfer"};
      _msg[14] = '{ERROR  , "PPROT must remain stable for the entire transfer"};
      _msg[15] = '{ERROR  , "PPROT undefined"};
      _msg[16] = '{ERROR  , "PWDATA must remain stable for the entire transfer"};
      _msg[17] = '{WARNING, "PWDATA contains 'x'"};
      _msg[18] = '{WARNING, "PWDATA contains 'x'"};
      _msg[39] = '{WARNING, "PWDATA should be 8, 16, or 32 bits wide"};
      _msg[19] = '{WARNING, "PRDATA contains 'x'"};
      _msg[40] = '{WARNING, "PRDATA should be 8, 16, or 32 bits wide"};
      _msg[20] = '{ERROR  , "PREADY undefined during Access phase"};
      _msg[21] = '{ERROR  , "PSLVERR undefined"};
      _msg[22] = '{FATAL  , "Watchdog expired"};
      _msg[23] = '{ERROR  , "PWAKEUP must remain high until the end of the transfer"};
      _msg[24] = '{WARNING, "PWAKEUP should be asserted at least one cycle before PSEL"};
      _msg[25] = '{WARNING, "PWAKEUP raised without starting a transfer"};
      _msg[26] = '{ERROR  , "PWAKEUP undefined"};
      _msg[27] = '{ERROR  , "PAUSER must remain stable for the entire transfer"};
      _msg[28] = '{ERROR  , "PAUSER undefined"};
      _msg[29] = '{WARNING, "PAUSER width should be max 128 bits"};
      _msg[30] = '{ERROR  , "PWUSER must remain stable for the entire write transfer"};
      _msg[31] = '{ERROR  , "PWUSER undefined"};
      _msg[32] = '{WARNING, "PWUSER should be max DATA_WIDTH/2 bits"};
      _msg[33] = '{WARNING, "PRUSER contains 'x'"};
      _msg[34] = '{WARNING, "PRUSER should be max DATA_WIDTH/2 bits"};
      _msg[35] = '{ERROR  , "PBUSER undefined"};
      _msg[36] = '{WARNING, "PBUSER should be max 16 bits"};
      //_msg[37] see PSTRB
      //_msg[38] see PADDR
      //_msg[39] see PWDATA
      //_msg[40] see PRDATA
      _msg[41] = '{ERROR  , "PRESETn undefined"};
      _msg[42] = '{ERROR  , "PCLK undefined"};


      /* Initial width checks
       */
      if ($bits(PADDR) > 32)
        message(38);

      if (($bits(PWDATA) !=  8) &&
          ($bits(PWDATA) != 16) &&
          ($bits(PWDATA) != 32) )
        message(39);

      if (($bits(PRDATA) !=  8) &&
          ($bits(PRDATA) != 16) &&
          ($bits(PRDATA) != 32) )
        message(40);

`ifdef APB_VERSION_APB5
      if ($bits(PAUSER) > 128)
        message(29);

      if ($bits(PWUSER) > DATA_WIDTH/2)
        message(32);

      if ($bits(PRUSER) > DATA_WIDTH/2)
        message(34);

      if ($bits(PBUSER) > 16)
        message(36);
`endif
  end


  //Display message
  task automatic message (input int msg_no);
    severity_t msg_severity;
    string     msg;

    msg_severity = _msg[msg_no].severity;
    msg          = $sformatf ("APB-%0d %s (%m): %s @%0t", msg_no +1, msg_severity.name(), _msg[msg_no].message, $time);

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
  task automatic set_severity(int msg_no, severity_t severity);
    if (msg_no < MESSAGE_COUNT && msg_no >= 0)
      _msg[msg_no].severity = severity;
  endtask : set_severity

  //Get severity level of a message/check
  function automatic severity_t get_severity(int msg_no);
    if (msg_no < MESSAGE_COUNT && msg_no >= 0)
      return _msg[msg_no].severity;
  endfunction : get_severity


  //////////////////////////////////////////////////////////////////
  //
  // Functions
  //

  //Return selected APB_VERSION string
  function string apb_version;
`ifdef APB_VERSION_APB5
    return "ABP5";
`elsif APB_VERSION_APB4
    return "APB4";
`elsif APB_VERSION_APB3
    return "APB3";
`else
    return "APB2";
`endif
  endfunction : apb_version


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
    input [ADDR_WIDTH-1:0] paddr,
    input [PSTRB_SIZE-1:0] pstrb
  );
    int                    tr_size;
    logic [PSTRB_SIZE-1:0] mask;
    logic [ADDR_WIDTH-1:0] addr_mask,
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

    addr_mask   = {ADDR_WIDTH{1'b1}} << tr_size;
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
    $display ("- APB Version: %s", apb_version);
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
   * Check PRESETn
   */
  task check_presetn;
    //PRESETn must not be undefined
    if (PRESETn === 1'bx || PRESETn === 1'bz)
      message(41);
  endtask : check_presetn


  /*
   * Check PCLK
   */
  task check_pclk;
    //PCLK must not be undefined
    if (PRESETn)
      if (PCLK === 1'bx || PCLK === 1'bz)
        message(42);
  endtask : check_pclk


  /*
   * Check PSEL
   */
  task check_psel;
    //PSEL can only go low if the current transfer ended
    if (!PSEL && dly_psel)
      if (!dly_pready)
        message(0);

    //PSEL must not be undefined
    if (PSEL === 1'bx || PSEL === 1'bz)
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
    logic [ADDR_WIDTH-1:0] addr_mask, masked_addr;

    //PADDR must remain stable during the entire transfer
    if (!dly_pready && PADDR !== dly_paddr)
      message(5);

    //PADDR must align with PSTRB
`ifdef APB_VERSION_APB4
    if (PWRITE)
    begin
        if (pstrb_misaligned(PADDR, PSTRB))
          message(6);
    end
`endif

    //PADDR should be aligned to DATA_WIDTH
    addr_mask   = {ADDR_WIDTH{1'b1}} << $clog2(PSTRB_SIZE);
    masked_addr = PADDR & ~addr_mask;
    if (|masked_addr)
      message(7);

    //PADDR may not be undefined during transfer
    if (^PADDR === 1'bx)
      message(8);
  endtask : check_paddr


  /*
   * Check PWRITE
   */
  task check_pwrite;
    //PWRITE must remain stable during the entire transfer
    if (!dly_pready && PWRITE !== dly_pwrite)
      message(9);

    //PWRITE may not be undefined during transfer
    if (PWRITE === 1'bx || PWRITE === 1'bz)
      message(10);
  endtask : check_pwrite


  /*
   * Check PSTRB
   */
`ifdef APB_VERSION_APB4
  task check_pstrb;
    //PSTRB valid?
    if (|PSTRB)
      if (~pstrb_valid(PSTRB))
        message(11);

    //PSTRB must remain stable during entire transfer
    if (!dly_pready && PSTRB !== dly_pstrb)
      message(12);

    //PSTRB must be low during read transfer
    if (!PWRITE && |PSTRB)
      message(37);

    //PSTRB may not be undefined during transfer
    if (PWRITE && ^PSTRB === 1'bx)
      message(13);
  endtask : check_pstrb
`endif


  /*
   * Check PPROT
   */
`ifdef APB_VERSION_APB4
  task check_pprot;
    //PPROT must remain stable during entire transfer
    if (!dly_pready && PPROT !== dly_pprot)
      message(14);

    //PPROT may not be undefined during transfer
    if (^PPROT === 1'bx)
      message(15);
  endtask : check_pprot
`endif


  /*
   * Check PWDATA
   */
  task check_pwdata;
    logic is_x;

    //PWDATA must remain stable during entire transfer
    if (!dly_pready && PWDATA !== dly_pwdata)
      message(16);

    //PWDATA undefined?
`ifdef APB_VERSION_APB4
    if (PWRITE)
    begin
        is_x = 1'b0;

        foreach (PSTRB[i])
          is_x |= (^PWDATA[i*8 +: 8] & PSTRB[i]) === 1'bx;

        if (is_x)
          message(17);
    end
`else
    if (PWRITE && ^PWDATA === 1'bx)
      message(18);
`endif
  endtask : check_pwdata


  /*
   * Check PRDATA
   */
  task check_prdata;
    //PRDATA undefined when transfer completes?
`ifdef APB_VERSION_APB3
    if (PENABLE && PREADY && !PWRITE)
`else
    if (PENABLE && !PWRITE)
`endif
      if (^PRDATA === 1'bx)
        message(19);
  endtask : check_prdata


  /*
   * Check PREADY
   */
`ifdef APB_VERSION_APB3
  task check_pready;
    //PREADY may not contain 'x' when PENABLE is high
    if (PENABLE && (PREADY === 1'bx || PREADY === 1'bz))
      message(20);
  endtask : check_pready
`endif


  /*
   * Check PSLVERR
   */
`ifdef APB_VERSION_APB3
  task check_pslverr;
    //PSLVERR may not be undefined when transfer completes
    if (PENABLE && PREADY)
      if (PSLVERR === 1'bx || PSLVERR === 1'bz)
        message(21);
  endtask : check_pslverr
`endif


  /*
   * Check PWAKEUP
   */
`ifdef APB_VERSION_APB5
  task check_pwakeup;
    //PWAKEUP must remain asserted until PREADY asserted if PSEL
    //  and PWAKEUP are high in the same cycle
    if (!dly_pready && dly_psel && dly_pwakeup && !PWAKEUP)
      message(23);

    //Recommendation: assert PWAKEUP 1 cycle before PSEL
    if (PSEL && PWAKEUP && !dly_pwakeup)
      message(24);

    //It is not recommended to assert, then deassert PWAKEUP without
    // starting a transfer
    if (!PWAKEUP && dly_pwakeup && !PSEL && !dly_psel)
      message(25);

    //PWAKEUP should not be undefined ever
    if (PWAKEUP === 1'bx || PWAKEUP === 1'bz)
      message(26);
  endtask : check_pready
`endif


  /*
   * Check PAUSER
   */
`ifdef APB_VERSION_APB5
  task check_pauser;
    //PAUSER must remain valid for the entire transfer
    if (!dly_pready && PAUSER != dly_pauser)
      message(27);

    //PAUSER must be valid when PSEL is asserted
    if (PAUSER === 1'bx || PAUSER === 1'bz)
      message(28);
  endtask : check_pauser
`endif


  /*
   * Check PWUSER
   */
`ifdef APB_VERSION_APB5
  task check_pwuser;
    //PAUSER must remain valid for the entire transfer
    if (!dly_pready && PWUSER != dly_pwuser)
      message(30);

    //PAUSER must be valid when PSEL and PWRITE are asserted
    if (PWRITE && (PWUSER === 1'bx || PWUSER === 1'bz))
      message(31);
  endtask : check_pwuser
`endif


  /*
   * Check PRUSER
   */
`ifdef APB_VERSION_APB5
  task check_pruser;
    //PRUSER undefined when transfer completes?
    if (PENABLE && PREADY && !PWRITE)
      if (^PRUSER === 1'bx)
        message(33);
  endtask : check_pruser
`endif


  /*
   * Check PBUSER
   */
`ifdef APB_VERSION_APB5
  task check_pbuser;
    //PBUSER undefined when transfer completes?
    if (PENABLE && PREADY)
      if (PBUSER === 1'bx || PBUSER === 1'bz)
        message(35);
  endtask : check_pbuser
`endif



  //////////////////////////////////////////////////////////////////
  //
  // Module Body
  //

  /*
   * Phase
   */
  assign setup_phase = (PSEL & !dly_psel                ) |
                       (PSEL &  dly_penable & dly_pready);

  initial access_phase = 1'b0;

  always @(posedge PCLK, negedge PRESETn)
    if      (!PRESETn    ) access_phase <= 1'b0;
    else if ( setup_phase) access_phase <= 1'b1;
`ifdef APB_VERSION_APB3
    else if ( PREADY     ) access_phase <= 1'b0;
`else
    else                   access_phase <= 1'b0;
`endif


  /*
   * Check PRESETn
   */
  initial check_presetn();
  always @(PRESETn) check_presetn();
  

  /*
   * Check PCLK
   */
  initial check_pclk();
  always @(PCLK) check_pclk();


  /*
   * Check PSEL
   */
  initial dly_psel = 0;

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
`ifdef APB_VERSION_APB4
  always @(posedge PCLK) if (CHECK_PSTRB) dly_pstrb <= PSTRB;
  always @(posedge PCLK) if (CHECK_PSTRB) if (PSEL) check_pstrb();
`endif


  /*
   * Check PPROT
   */
`ifdef APB_VERSION_APB4
  always @(posedge PCLK) if (CHECK_PPROT) dly_pprot <= PPROT;
  always @(posedge PCLK) if (CHECK_PPROT) if (PSEL) check_pprot();
`endif


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
`ifdef APB_VERSION_APB3
  always @(posedge PCLK) dly_pready <= PREADY;
  always @(posedge PCLK) if (PSEL) check_pready();
`else
  assign dly_ready = 1'b1;
`endif


  /*
   * Check PSLVERR
   */
`ifdef APB_VERSION_APB3
  always @(posedge PCLK) if (CHECK_PSLVERR) if (PSEL) check_pslverr();
`endif


  /*
   * Check PWAKEUP
   */
`ifdef APB_VERSION_APB5
  always @(posedge PCLK) dly_pwakeup <= PWAKEUP;
  always @(posedge PCLK) if (PSEL) check_pwakeup();
`endif


  /*
   * Check PAUSER
   */
`ifdef APB_VERSION_APB5
  always @(posedge PCLK) if (USER_REQ_WIDTH > 0) dly_pauser <= PAUSER;
  always @(posedge PCLK) if (USER_REQ_WIDTH > 0) check_pauser();
`endif


  /*
   * Check PWUSER
   */
`ifdef APB_VERSION_APB5
  always @(posedge PCLK) if (USER_DATA_WIDTH > 0) dly_pwuser <= PWUSER;
  always @(posedge PCLK) if (USER_DATA_WIDTH > 0) if (PSEL) check_pwuser();
`endif


  /*
   * Check PRUSER
   */
`ifdef APB_VERSION_APB5
  always @(posedge PCLK) if (USER_DATA_WIDTH > 0) dly_pruser <= PRUSER;
  always @(posedge PCLK) if (USER_DATA_WIDTH > 0) if (PSEL) check_pruser();
`endif


  /*
   * Check PBUSER
   */
`ifdef APB_VERSION_APB5
  always @(posedge PCLK) if (USER_RESP_WIDTH > 0) dly_pbuser <= PBUSER;
  always @(posedge PCLK) if (USER_RESP_WIDTH > 0) if (PSEL) check_pbuser();
`endif



   /*
    * Watchdog
    */
  always @(posedge PCLK, negedge PRESETn)
    if      (!PRESETn           ) watchdog_cnt <= WATCHDOG_TIMEOUT;
    else if (!PSEL              ) watchdog_cnt <= WATCHDOG_TIMEOUT;
`ifdef APB_VERSION_APB3
    else if ( PENABLE && PREADY ) watchdog_cnt <= WATCHDOG_TIMEOUT;
`else
    else if ( PENABLE           ) watchdog_cnt <= WATCHDOG_TIMEOUT;
`endif
    else                          watchdog_cnt <= watchdog_cnt -1'h1;


  always @(posedge PCLK)
    if (WATCHDOG_TIMEOUT > 0)
      if (~|watchdog_cnt)
        message(22);
endmodule : apb_checker

