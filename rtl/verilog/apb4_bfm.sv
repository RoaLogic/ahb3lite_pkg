/////////////////////////////////////////////////////////////////////
//   ,------.                    ,--.                ,--.          //
//   |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---.    //
//   |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--'    //
//   |  |\  \ ' '-' '\ '-'  |    |  '--.' '-' ' '-' ||  |\ `--.    //
//   `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---'    //
//                                             `---'               //
//   APB4 Bus Functional Model                                     //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//             Copyright (C) 2023 ROA Logic BV                     //
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
 * Limitations:
 * Does not support Error response/transactions (PSLVERR is ignored)
 */

module apb4_master_bfm
#(
  parameter PADDR_SIZE = 16,
  parameter PDATA_SIZE = 32
)
(
  input                         PRESETn,
                                PCLK,
  output reg                    PSEL,
  output reg                    PENABLE,
  output reg                    PWRITE,
  output reg [PDATA_SIZE/8-1:0] PSTRB,
  output reg [PADDR_SIZE  -1:0] PADDR,
  output reg [             2:0] PPROT,
  output reg [PDATA_SIZE  -1:0] PWDATA,
  input      [PDATA_SIZE  -1:0] PRDATA,
  input                         PREADY,
  input                         PSLVERR
);

  always @(negedge PRESETn) reset();


  /////////////////////////////////////////////////////////
  //
  // Tasks
  //
  task reset();
    //Reset AHB Bus
    PSEL      = 1'b0;
    PENABLE   = 1'bx;
    PWRITE    = 1'bx;
    PSTRB     = 'hx;
    PADDR     = 'hx;
    PPROT     = 'hx;
    PWDATA    = 'hx;

    @(posedge PRESETn);
  endtask


  task idle ();
    //Put AHP Bus in IDLE state
    //Call after write or read sequence
    PSEL <= 1'b0;
  endtask


  /* APB Write
   * 
   */
  task automatic write (
    input [PADDR_SIZE  -1:0] address,
    input [PDATA_SIZE  -1:0] data,
    input [PDATA_SIZE/8-1:0] be
  );
    //Address phase
    PSEL    <= 1'b1;
    PENABLE <= 1'b0;
    PWRITE  <= 1'b1;
    PSTRB   <= be;
    PADDR   <= address;
    PPROT   <= 'hx;
    PWDATA  <= data;
    @(posedge PCLK);

    //Data phase
    PENABLE <= 1'b1;

    do
      @(posedge PCLK);
    while (!PREADY);

    //Transfer done
    PSEL    <= 1'b0;
  endtask


  /* APB Read
   * 
   */
  task automatic read (
    input  [PADDR_SIZE-1:0] address,
    output [PDATA_SIZE-1:0] data
  );
    //Address phase
    PSEL    <= 1'b1;
    PENABLE <= 1'b0;
    PWRITE  <= 1'b0;
    PSTRB   <=  'hx;
    PADDR   <= address;
    PPROT   <= 'hx;
    PWDATA  <= 'hx;
    @(posedge PCLK);

    //Data phase
    PENABLE <= 1'b1;

    do @(posedge PCLK);
    while (!PREADY);
    
    data = PRDATA;

    //Transfer done
    PSEL    <= 1'b0;
  endtask

endmodule : apb4_master_bfm
