;
;  6500 series Debug monitor for use with NOICE02
;
;  This monitor uses only the basic 6502 instructions.
;  No 65C02 extended instructions are used
;
;  Copyright (c) 2001 by John Hartman
;
;  Modification History:
;    6-Feb-94 JLH ported from Mitsubishi 740 version
;   12-Feb-97 JLH wrong target type!  Change from 1 to 7 for 65(C)02
;   21-Jul-00 JLH change FN_MIN from F7 to F0
;   22-Sep-00 JLH add CALL address to TSTG
;   12-Mar-01 JLH V3.0: improve text about paging, formerly called "mapping"
;   18-Feb-09 PO  Updated a few comments for clarity and stored it in SVN
;                 Use DASM to assemble file
;
;============================================================================
;
;  To customize for a given target, you must change code in the
;  hardware equates, the string TSTG, and the routines RESET and REWDT.
;  You may or may not need to change GETCHAR, PUTCHAR, depending on
;  how peculiar your UART is.
;
;  For more information, refer to the NoICE help file monitor.htm
;
;  To add banked or paged memory support:
;  1) Define page latch port PAGELATCH here
;  2) If PAGELATCH is write only, define or import the latch port's RAM
;     image PAGEIMAGE here (The application code must update PAGEIMAGE
;     before outputing to PAGELATCH)
;  3) Search for and modify PAGELATCH, PAGEIMAGE, and REG_PAGE usage below
;  4) In TSTG below edit "LOW AND HIGH LIMIT OF PAGED MEM"
;     to appropriate range (typically 8000H to BFFFH for two-bit MMU)
;
;  For more information, refer to the NoICE help file 2bitmmu.htm
;
;  This code was ported from the Mitsubishi 740 family version.
;  It should work on the 6502 and 65(C)02 with the following assumptions:
;  (These agree with the description of the 6502 given in "6502 Assembly
;  Language Programming" by Lance Leventhal, Osborne/McGraw-Hill, 1979)
;  - Even though BRK is a one byte instruction, when a BRK occurs the PC
;    is incremented by 2 before being pushed.  If this is not true for your
;    processor, change the "SBC #2" instruction after INT_ENTRY to "SBC #1"
;  - When a BRK occurs, the "B" bit is set BEFORE the status register is
;    pushed.  If this is not true for your processor, remove the "PLA/PHA"
;    after .IRQ and .NMI and repalce them with "PHP/PLA"
;  - If an interrupt is pending when a BRK is executed, PC will be loaded
;    with the vector for the INTERRUPT, not for BRK.  Thus, both .IRQ
;    and .NMI check for the break bit.  (This occurs on the 740.  According
;    to the Rockwell databook, it occurs on the 6502, but not the 65C02)
;
;  This file has been assembled with the Avocet 6502/740 assembler.
;
;============================================================================
               processor 6502
;
;  I/O equates for Heng's ROM emulator (set true if used)
ROMEM   EQU     0
;
;============================================================================
;  HARDWARE PLATFORM CUSTOMIZATIONS
;
RAM_START      EQU     $0200           ;START OF MONITOR RAM
ROM_START      EQU     $F800           ;START OF MONITOR CODE
HARD_VECT      EQU     $FFF0           ;START OF HARDWARE VECTORS

;==========================================================================
;
;  Equates for 16450 serial port on ROM emulator board
;
;  The 50747 always reads an address before writing it.  This, of
;  course, is hard on pending inputs when writing to an ACIA.
;  Since our protocol is half duplex, this shouldn't affect us.
;
;S16450        equ     $F800           ;base of 16450 UART
;RXR           equ     0               ;  Receiver buffer register
;TXR           equ     0               ;  Transmitter buffer register
;IER           equ     1               ;  Interrupt enable register
;LCR           equ     3               ;  Line control register
;MCR           equ     4               ;  Modem control register
;LSR           equ     5               ;  Line status register
;
;  Define monitor serial port
;SERIAL_STATUS EQU     S16450+LSR
;SERIAL_RXDATA EQU     S16450+RXR
;SERIAL_TXDATA EQU     S16450+TXR
;RXRDY         EQU     00000001B       ; BIT MASK FOR RX BUFFER FULL
;TXRDY         EQU     00100000B       ; BIT MASK FOR TX BUFFER EMPTY
;
;  Use ACIA (6551) for RS-232
;  The 50747 always reads an address before writing it.  This, of
;  course, is hard on pending inputs when writing to an ACIA.  Mr.
;  Olson has graciously doctored the chip select logic so that
;  A2=0 writes to the chip, A2=1 reads from the chip.  Thus we
;  have Read and Write addresses for each register.
;
;  The hardware connects RS-232 pin 8 (DCD) to 6551 DSR.  This because
;  if 6551 DCD is not true, we cannot receive anything.  Using DSR
;  permits simple cables to be used for TTY applications.
;
;  The hardware connects RS-232 pin 20 (DTR) to 6551 RST.  This because
;  the Hayes modem and direct connect cables use DTR for signaling.
SERIAL_TXDATA  EQU   $7C00             ;ACIA WRITE DATA
SERIAL_RXDATA  EQU   $7C00             ;ACIA READ DATA
SERIAL_RESET   EQU   $7C01             ;WRITE (STATUS) TO RESET ACIA
SERIAL_STATUS  EQU   $7C01             ;ACIA STATUS
SERIAL_WCMD    EQU   $7C02             ;ACIA COMMAND REGISTER
SERIAL_WCTL    EQU   $7C03             ;ACIA CONTROL REGISTER
RXRDY          EQU   $08               ;RECEIVE READY
TXRDY          EQU   $10               ;TRANSMIT READY
;

;============================================================================
;  STACK RAM (PAGE 1)
;  (Calculated use is at most 7 bytes)
INITSTACK      EQU   $13F              ;TOP OF STACK RAM
;
;  RAM definitions
               seg.u RAM
               org   $0200
;
;  RAM interrupt vectors (first in SEG for easy addressing, else move to
;  their own SEG)
;  (RAMVEC + $00)  - NMI Handler
;  (RAMVEC + $04)  - IRQ Handler
RAMVEC          DS      2*3
;
;  Target registers:  order must match that in TRG740.C
TASK_REGS:
REG_STATE       DS      1
REG_PAGE        DS      1
REG_SP          DS      2
REG_Y           DS      1
REG_X           DS      1
REG_A           DS      1
REG_CC          DS      1
REG_PC          DS      2
TASK_REGS_SIZE =     *-TASK_REGS
;
;  In order that we need no page zero RAM, do memory access via an
;  instruction built into RAM.  Build instruction and RTS here
CODEBUF         DS      4       ;ROOM FOR "LDA xxxx, RTS"
;
;  Store a counter for input timeout
RXTIMER         DS      2
;
;  Communications buffer
;  (Must be at least as long as TASK_REG_SZ.  At least 19 bytes recommended.
;  Larger values may improve speed of NoICE memory move commands.)
COMBUF_SIZE     EQU     128             ;DATA SIZE FOR COMM BUFFER
COMBUF          DS      2+COMBUF_SIZE+1 ;BUFFER ALSO HAS FN, LEN, AND CHECK
;
RAM_END        EQU   *                 ;ADDRESS OF TOP+1 OF RAM
;
;===========================================================================
        SEG     CODE
               org   $8000,$ff
               DC    $ff
               org   $fc00
;
;  Power on reset
RESET:
;
;  Set CPU mode to safe state
        NOP                     ;DATA BOOK SHOWS THIS - DON'T KNOW WHY
        SEI                     ;INTERRUPTS OFF
        CLD                     ;USE BINARY MODE
;
;  INITIALIZE TARGET HARDWARE
;
;  INIT STACK
               LDX     #>INITSTACK
        TXS
;
;===========================================================================
#IF ROMEM
;
;  Initialize S16450 UART on ROM emulator
;
;  access baud generator, no parity, 1 stop bit, 8 data bits
               LDA    #%10000011
        STA     S16450+LCR
;
;  fixed baud rate of 19200:  crystal is 3.686400 Mhz.
;  Divisor is 3,686400/(16*baud)
        LDA     #12                     ;fix at 19.2 kbaud
        STA     S16450+RXR              ;lsb
        LDA     #0
        STA     S16450+RXR+1            ;msb=0
;
;  access data registers, no parity, 1 stop bits, 8 data bits
               LDA     #%00000011
        STA     S16450+LCR
;
;  no loopback, OUT2 on, OUT1 on, RTS on, DTR (LED) on
               LDA     #%00001111
        STA     S16450+MCR
;
;  disable all interrupts: modem, receive error, transmit, and receive
               LDA     #%00000000
        STA     S16450+IER
#ELSE

;  INIT MONITOR ACIA: 9600 BAUD, 8 BITS, NO PARITY, 1 STOP BIT
;  ACIA RUNS FROM 3.6864 MHZ CRYSTAL DIVIDED BY 4.
;  BAUD RATES ARE THUS HALF OF THOSE LISTED IN 6551 DATA SHEETS
        LDA     #0
        STA     SERIAL_RESET    ;PROGRAMMED RESET
               LDA     #%00011111      ;1SB, 8 DATA, 9600 BAUD (BOOK'S 19200)
        STA     SERIAL_WCTL
               LDA     #%00001011      ;NO PARITY, RTS, NO TX OR RX INT
        STA     SERIAL_WCMD
               LDA   SERIAL_STATUS     ; this is just to remove any pending
                                       ; interrupt that might exist.
#ENDIF
;===========================================================================
;  INIT RAM INTERRUPT VECTORS TO DUMMY HANDLERS
               LDA     #<.NMIX      ;NMI
        STA     RAMVEC+0
               LDA     #>.NMIX
        STA     RAMVEC+0+1
;
               LDA     #<.IRQX      ;IRQ
               STA     RAMVEC+04
               LDA     #>.IRQX
               STA     RAMVEC+04+1
;
;  Initialize user registers
               LDX     #<INITSTACK
        STX     REG_SP                  ;INIT USER'S STACK POINTER
               LDX     #>INITSTACK
        STX     REG_SP+1
        LDA     #0
        STA     REG_PC
        STA     REG_PC+1
        STA     REG_A
        STA     REG_X
        STA     REG_Y
        STA     REG_CC
        STA     REG_STATE               ;STATE IS 0 = RESET
;
;  Initialize memory paging variables and hardware (if any)
        STA     REG_PAGE                ;NO PAGE YET
;;;     STA     PAGEIMAGE
;;;     STA     PAGELATCH               ;set hardware page
;
;  Set function code for "GO".  Then if we reset after being told to
;  GO, we will come back with registers so user can see the crash
        LDA     #FN_RUN_TARGET
        STA     COMBUF
        JMP     RETURN_REGS             ;DUMP REGS, ENTER MONITOR
;
;===========================================================================
;  Get a character to A
;
;  Return A=char, CY=0 if data received
;         CY=1 if timeout (0.5 seconds)
;
;  Uses 4 bytes of stack including return address
;
GETCHAR:
        LDA     #0              ;LONG TIMEOUT
        STA     RXTIMER
        STA     RXTIMER+1
GC10:   JSR     REWDT           ;PREVENT WATCHDOG TIMEOUT
        DEC     RXTIMER
        BNE     GC20            ;BR IF NOT TIMEOUT
        DEC     RXTIMER+1       ;ELSE DEC HIGH HALF
        BEQ     GC90            ;EXIT IF TIMEOUT
GC20:   LDA     SERIAL_STATUS   ;READ DEVICE STATUS
        AND     #RXRDY
        BEQ     GC10            ;NOT READY YET.
;
;  Data received:  return CY=0. data in A
        CLC                     ;CY=0
        LDA     SERIAL_RXDATA   ;READ DATA
        RTS
;
;  Timeout:  return CY=1
GC90:   SEC                     ;CY=1
        RTS
;
;===========================================================================
;  Output character in A
;
;  Uses 5 bytes of stack including return address
;
PUTCHAR:
        PHA
PC10:   JSR     REWDT           ;PREVENT WATCHDOG TIMEOUT
        LDA     SERIAL_STATUS   ;CHECK TX STATUS
        AND     #TXRDY          ;RX READY ?
        BEQ     PC10
        PLA
        STA     SERIAL_TXDATA   ;TRANSMIT CHAR.
        RTS
;
;======================================================================
;
;  RESET WATCHDOG TIMER.  MUST BE CALLED AT LEAST ONCE EVERY 250 MSEC
;  OR PROCESSOR WILL BE RESET
;
;  Uses 2 bytes of stack including return address
;
REWDT:
        RTS
;
;======================================================================
;  Response string for GET TARGET STATUS request
;  Reply describes target:
TSTG:          DC    7                 ;2: PROCESSOR TYPE = 65(C)02
               DC    COMBUF_SIZE       ;3: SIZE OF COMMUNICATIONS BUFFER
               DC    $80               ;4: has CALL
               DC.W  0,0               ;5-8: LOW AND HIGH LIMIT OF MAPPED MEM (NONE)
               DC    B1-B0             ;9 BREAKPOINT INSTR LENGTH
;
;  Define either the BRK or JSR BRKE instruction for use as breakpoint
B0:     BRK                             ;10+ BREKAPOINT INSTRUCTION
;;;B0:  JSR     BRKE                    ;10+ BREKAPOINT INSTRUCTION
B1:            DC    "6500 monitor V3.0",0  ;DESCRIPTION, ZERO
               DC    0                 ;page of CALL breakpoint
               DC.W  B0                ;address of CALL breakpoint in native order

TSTG_SIZE      EQU   *-TSTG            ;SIZE OF STRING
;
;======================================================================
;  HARDWARE PLATFORM INDEPENDENT EQUATES AND CODE
;
;  Communications function codes.
FN_GET_STATUS  EQU   $FF               ;reply with device info
FN_READ_MEM    EQU   $FE               ;reply with data
FN_WRITE_MEM   EQU   $FD               ;reply with status (+/-)
FN_READ_REGS   EQU   $FC               ;reply with registers
FN_WRITE_REGS  EQU   $FB               ;reply with status
FN_RUN_TARGET  EQU   $FA               ;reply (delayed) with registers
FN_SET_BYTES   EQU   $F9               ;reply with data (truncate if error)
FN_IN          EQU   $F8               ;input from port
FN_OUT         EQU   $F7               ;output to port
;
FN_MIN         EQU   $F0               ;MINIMUM RECOGNIZED FUNCTION CODE
FN_ERROR       EQU   $F0               ;error reply to unknown op-code
;
;  6502 OP-CODE EQUATES
B              EQU   $10               ;BREAK BIT IN CONDITION CODES
LDA_OP         EQU   $AD               ;LDA AAA
STA_OP         EQU   $8D               ;STA AAA
CMP_OP         EQU   $CD               ;CMP AAA
LDAY_OP        EQU   $B9               ;LDA AAA,Y
STAY_OP        EQU   $99               ;STA AAA,Y
CMPY_OP        EQU   $D9               ;CMP AAA,Y
RTS_OP         EQU   $60               ;RTS

;===========================================================================
;  Enter here via JSR for breakpoint:  PC is stacked.
;  Stacked PC points at JSR+1
;;BRKE: STA     REG_A           ;SAVE ACCUM FROM DIRECT ENTRY
;;      PHP                     ;SAVE CC'S AS IF AFTER A BRK INSTRUCTION
;;      SEC
;
;  Common handler for default interrupt handlers
;  Enter with A=interrupt code = processor state
;  PC and CC are stacked.
;  REG_A has pre-interrupt accmulator
;  Stacked PC points at BRK+2 if BRK, else at PC if entry from interrupt
;  A  has state
INT_ENTRY:
;
;  Set CPU mode to safe state
        NOP                     ;DATA BOOK SHOWS THIS - DON'T KNOW WHY
        SEI                     ;INTERRUPTS OFF
        CLD                     ;USE BINARY MODE
;
;  Save registers in reg block for return to master
        STA     REG_STATE       ;SAVE MACHINE STATE
        PLA                     ;GET CONDITION CODES
        STA     REG_CC
        PLA                     ;GET LSB OF PC OF BREAKPOINT
        STA     REG_PC
        PLA                     ;GET MSB OF PC OF BREAKPOINT
        STA     REG_PC+1
;
;  If this is a breakpoint (state = 1), then back up PC to point at BRK
        LDA     REG_STATE       ;SAVED STATUS FOR TESTING
        CMP     #1
        BNE     B99             ;BR IF NOT BREAKPOINT: PC IS OK
;
;  On the 740, BRK leaves PC at break address +2:  back it up by 2
;  If the 6502 leaves PC at break address +1, back up by 1
        SEC
        LDA     REG_PC          ;BACK UP PC TO POINT AT BREAKPOINT
        SBC     #2
        STA     REG_PC
        LDA     REG_PC+1
        SBC     #0
        STA     REG_PC+1
B99:    JMP     ENTER_MON       ;REG_PC POINTS AT BREAKPOINT OPCODE
;
;===========================================================================
;
;  Main loop:  wait for command frame from master
;
;  Uses 4 bytes of stack before jump to functions
;
MAIN:
;
;  Since we have only part of a page for stack, we run on the target's
;  stack.  Thus, reset to target SP, rather than our own.
MAI10:  LDX     REG_SP
        TXS
        LDX     #0                      ;INIT INPUT BYTE COUNT
;
;  First byte is a function code
        JSR     GETCHAR                 ;GET A FUNCTION
        BCS     MAI10                   ;JIF TIMEOUT: RESYNC
        CMP     #FN_MIN
        BCC     MAI10                   ;JIF BELOW MIN: ILLEGAL FUNCTION
        STA     COMBUF,X                ;SAVE FUNCTION CODE
        INX
;
;  Second byte is data byte count (may be zero)
        JSR     GETCHAR                 ;GET A LENGTH BYTE
        BCS     MAI10                   ;JIF TIMEOUT: RESYNC
        CMP     #COMBUF_SIZE+1
        BCS     MAI10                   ;JIF TOO LONG: ILLEGAL LENGTH
        STA     COMBUF,X                ;SAVE LENGTH
        INX
        CMP     #0
        BEQ     MAI80                   ;SKIP DATA LOOP IF LENGTH = 0
;
;  Loop for data
        TAY                             ;SAVE LENGTH FOR LOOP
MAI20:  JSR     GETCHAR                 ;GET A DATA BYTE
        BCS     MAI10                   ;JIF TIMEOUT: RESYNC
        STA     COMBUF,X                ;SAVE DATA BYTE
        INX
        DEY
        BNE     MAI20
;
;  Get the checksum
MAI80:  JSR     GETCHAR                 ;GET THE CHECKSUM
        BCS     MAI10                   ;JIF TIMEOUT: RESYNC
        STA     CODEBUF                 ;SAVE CHECKSUM
;
;  Compare received checksum to that calculated on received buffer
;  (Sum should be 0)
        JSR     CHECKSUM
        CLC
        ADC     CODEBUF
        BNE     MAI10                   ;JIF BAD CHECKSUM
;
;  Process the message.
        LDA     COMBUF+0                ;GET THE FUNCTION CODE
        CMP     #FN_GET_STATUS
        BEQ     TARGET_STATUS
        CMP     #FN_READ_MEM
        BEQ     JREAD_MEM
        CMP     #FN_WRITE_MEM
        BEQ     JWRITE_MEM
        CMP     #FN_READ_REGS
        BEQ     JREAD_REGS
        CMP     #FN_WRITE_REGS
        BEQ     JWRITE_REGS
        CMP     #FN_RUN_TARGET
        BEQ     JRUN_TARGET
        CMP     #FN_SET_BYTES
        BEQ     JSET_BYTES
        CMP     #FN_IN
        BEQ     JIN_PORT
        CMP     #FN_OUT
        BEQ     JOUT_PORT
;
;  Error: unknown function.  Complain
        LDA     #FN_ERROR
        STA     COMBUF          ;SET FUNCTION AS "ERROR"
        LDA     #1
               JMP     SENDSTATUS     ;VALUE IS "ERROR"
;
;  long jumps to handlers
JREAD_MEM:      JMP     READ_MEM
JWRITE_MEM:     JMP     WRITE_MEM
JREAD_REGS:     JMP     READ_REGS
JWRITE_REGS:    JMP     WRITE_REGS
JRUN_TARGET:    JMP     RUN_TARGET
JSET_BYTES:     JMP     SET_BYTES
JIN_PORT:       JMP     IN_PORT
JOUT_PORT:      JMP     OUT_PORT

;===========================================================================
;
;  Target Status:  FN, len
TARGET_STATUS:
        LDX     #0                      ;DATA FOR REPLY
               LDY     #<TSTG_SIZE              ;LENGTH OF REPLY
        STY     COMBUF+1                ;SET SIZE IN REPLY BUFFER
TS10:   LDA     TSTG,X                  ;MOVE REPLY DATA TO BUFFER
        STA     COMBUF+2,X
        INX
        DEY
        BNE     TS10
;
;  Compute checksum on buffer, and send to master, then return
        JMP     SEND

;===========================================================================
;
;  Read Memory:  FN, len, page, Alo, Ahi, Nbytes
;
READ_MEM:
;
;  Set page
;;      LDA     COMBUF+2
;;      STA     PAGEIMAGE
;;      STA     PAGELATCH
;
;  Build "LDA  AAAA,Y" in RAM
        LDA     #LDAY_OP
        STA     CODEBUF+0
;
;  Set address of instruction in RAM
        LDA     COMBUF+3
        STA     CODEBUF+1
        LDA     COMBUF+4
        STA     CODEBUF+2
;
;  Set return after LDA
        LDA     #RTS_OP
        STA     CODEBUF+3
;
;  Prepare return buffer: FN (unchanged), LEN, DATA
        LDX     COMBUF+5                ;NUMBER OF BYTES TO GET
        STX     COMBUF+1                ;RETURN LENGTH = REQUESTED DATA
        BEQ     GLP90                   ;JIF NO BYTES TO GET
;
;  Read the requested bytes from local memory
        LDY     #0                      ;INITIAL OFFSET
GLP:    JSR     CODEBUF                 ;GET BYTE AAAA,Y TO A
        STA     COMBUF+2,Y              ;STORE TO RETURN BUFFER
        INY
        DEX
        BNE     GLP
;
;  Compute checksum on buffer, and send to master, then return
GLP90:  JMP     SEND

;===========================================================================
;
;  Write Memory:  FN, len, page, Alo, Ahi, (len-3 bytes of Data)
;
;  Uses 2 bytes of stack
;
WRITE_MEM:
;
;  Set page
;;      LDA     COMBUF+2
;;      STA     PAGEIMAGE
;;      STA     PAGELATCH
;
;  Build "STA  AAAA,Y" in RAM
        LDA     #STAY_OP
        STA     CODEBUF+0
;
;  Set address into RAM
        LDA     COMBUF+3
        STA     CODEBUF+1
        LDA     COMBUF+4
        STA     CODEBUF+2
;
;  Set return after STA
        LDA     #RTS_OP
        STA     CODEBUF+3
;
;  Prepare return buffer: FN (unchanged), LEN, DATA
        LDX     COMBUF+1                ;NUMBER OF BYTES TO PUT
        DEX                             ;LESS PAGE, ADDRLO, ADDRHI
        DEX
        DEX
        BEQ     WLP50                   ;JIF NO BYTES TO PUT
;
;  Write the specified bytes to local memory
        LDY     #0                      ;INITIAL OFFSET
WLP:    LDA     COMBUF+5,Y              ;GET BYTE TO WRITE
        JSR     CODEBUF                 ;STORE THE BYTE AT AAAA,Y
        INY
        DEX
        BNE     WLP
;
;  Build "CMP  AAAA,Y" in RAM
        LDA     #CMPY_OP
        STA     CODEBUF+0
;
;  Compare to see if the write worked
        LDX     COMBUF+1                ;NUMBER OF BYTES TO PUT
        DEX                             ;LESS PAGE, ADDRLO, ADDRHI
        DEX
        DEX
        LDY     #0                      ;INITIAL OFFSET
WLP20:  LDA     COMBUF+5,Y              ;GET BYTE JUST WRITTEN
        JSR     CODEBUF                 ;COMPARE THE BYTE AT AAAA,Y
        BNE     WLP80                   ;BR IF WRITE FAILED
        INY
        DEX
        BNE     WLP20
;
;  Write succeeded:  return status = 0
WLP50:  LDA     #0                      ;RETURN STATUS = 0
        JMP     WLP90
;
;  Write failed:  return status = 1
WLP80:  LDA     #1
;
;  Return OK status
WLP90:         JMP     SENDSTATUS

;===========================================================================
;
;  Read registers:  FN, len=0
;
READ_REGS:
;
;  Enter here from "RUN" and "STEP" to return task registers
RETURN_REGS:
        LDX     #0                      ;REGISTER LIVE HERE
               LDY     #<TASK_REGS_SIZE         ;NUMBER OF BYTES
        STY     COMBUF+1                ;SAVE RETURN DATA LENGTH
;
;  Copy the registers
GRLP:   LDA     TASK_REGS,X             ;GET BYTE TO A
        STA     COMBUF+2,X              ;STORE TO RETURN BUFFER
        INX
        DEY
        BNE     GRLP
;
;  Compute checksum on buffer, and send to master, then return
        JMP     SEND

;===========================================================================
;
;  Write registers:  FN, len, (register image)
;
WRITE_REGS:
;
        LDX     #0                      ;POINTER TO DATA
        LDY     COMBUF+1                ;NUMBER OF BYTES
        BEQ     WRR80                   ;JIF NO REGISTERS
;
;  Copy the registers
WRRLP:  LDA     COMBUF+2,X              ;GET BYTE TO A
        STA     TASK_REGS,X             ;STORE TO REGISTER RAM
        INX
        DEY
        BNE     WRRLP
;
;  Reload SP, in case it has changed
        LDX     REG_SP
        TXS
;
;  Return OK status
WRR80:  LDA     #0
               JMP     SENDSTATUS

;===========================================================================
;
;  Run Target:  FN, len
;
;  Uses 3 bytes of stack for user PC and CC before RTI
;
RUN_TARGET:
;
;  Restore user's page
;;;     LDA     REG_PAGE                ;USER'S PAGE
;;;     STA     PAGEIMAGE
;;;     STA     PAGELATCH               ;set hardware page
;
;  Switch to user stack, if not already running on it
        LDX     REG_SP                  ;BACK TO USER STACK
        TXS
        LDA     REG_PC+1                ;SAVE MS USER PC FOR RTI
        PHA
        LDA     REG_PC                  ;SAVE LS USER PC FOR RTI
        PHA
        LDA     REG_CC                  ;SAVE USER CONDITION CODES FOR RTI
        PHA
;
;  Restore registers
        LDX     REG_X
        LDY     REG_Y
        LDA     REG_A
;
;  Return to user
        RTI
;
;===========================================================================
;
;  Common continue point for all monitor entrances
;  REG_STATE, REG_A, REG_CC, REG_PC set; X, Y intact; SP = user stack
ENTER_MON:
        STX     REG_X
        STY     REG_Y
        TSX
        STX     REG_SP          ;SAVE USER'S STACK POINTER (LSB)
        LDA     #1              ;STACK PAGE ALWAYS 1
EM10:   STA     REG_SP+1        ;(ASSUME PAGE 1 STACK)
;
;  With only a partial page for the stack, don't switch
;;;        LDX  #MONSTACK       ;AND USE OURS INSTEAD
;;;        TXS
;
;;;     LDA     PAGEIMAGE       ;GET CURRENT USER PAGE
        LDA     #0              ;... OR ZERO IF UNPAGED TARGET
        STA     REG_PAGE        ;SAVE USER'S PAGE
;
;  Return registers to master
        JMP     RETURN_REGS

;===========================================================================
;
;  Set target byte(s):  FN, len { (page, alow, ahigh, data), (...)... }
;
;  Return has FN, len, (data from memory locations)
;
;  If error in insert (memory not writable), abort to return short data
;
;  This function is used primarily to set and clear breakpoints
;
;  Uses 2 bytes of stack
;
SET_BYTES:
        LDY     COMBUF+1                ;LENGTH = 4*NBYTES
        BEQ     SB90                    ;JIF NO BYTES
;
;  Loop on inserting bytes
        LDX     #0                      ;INDEX INTO INPUT BUFFER
        LDY     #0                      ;INDEX INTO OUTPUT BUFFER
SB10:
;
;  Set page
;;      LDA     COMBUF+2,X
;;      STA     PAGEIMAGE
;;      STA     PAGELATCH
;
;  Build "LDA  AAAA" in RAM
        LDA     #LDA_OP
        STA     CODEBUF+0
;
;  Set address
        LDA     COMBUF+3,X
        STA     CODEBUF+1
        LDA     COMBUF+4,X
        STA     CODEBUF+2
;
;  Set return after LDA
        LDA     #RTS_OP
        STA     CODEBUF+3
;
;  Read current data at byte location
        JSR     CODEBUF                 ;GET BYTE AT AAAA
        STA     COMBUF+2,Y              ;SAVE IN RETURN BUFFER
;
;  Insert new data at byte location
;
;  Build "STA  AAAA" in RAM
        LDA     #STA_OP
        STA     CODEBUF+0
        LDA     COMBUF+5,X              ;BYTE TO WRITE
        JSR     CODEBUF
;
;  Verify write
        LDA     #CMP_OP
        STA     CODEBUF+0
        LDA     COMBUF+5,X
        JSR     CODEBUF
        BNE     SB90                    ;BR IF INSERT FAILED: ABORT AT Y BYTES
;
;  Loop for next byte
        INY                             ;COUNT ONE INSERTED BYTE
        INX                             ;STEP TO NEXT BYTE SPECIFIER
        INX
        INX
        INX
        CPX     COMBUF+1
        BNE     SB10                    ;LOOP FOR ALL BYTES
;
;  Return buffer with data from byte locations
SB90:   STY     COMBUF+1                ;SET COUNT OF RETURN BYTES
;
;  Compute checksum on buffer, and send to master, then return
        JMP     SEND

;===========================================================================
;
;  Input from port:  FN, len, PortAddressLo, PAhi (=0)
;
;  While the M740 has no input or output instructions, we retain these
;  to allow write-without-verify
;
IN_PORT:
;
;  Build "LDA  AAAA" in RAM
        LDA     #LDA_OP
        STA     CODEBUF+0
;
;  Set port address
        LDA     COMBUF+2
        STA     CODEBUF+1
        LDA     COMBUF+3
        STA     CODEBUF+2
;
;  Set return after LDA
        LDA     #RTS_OP
        STA     CODEBUF+3
;
;  Read the requested byte from local memory
        JSR     CODEBUF                 ;GET BYTE TO A
;
;  Return byte read as "status"
               JMP     SENDSTATUS

;===========================================================================
;
;  Output to port:  FN, len, PortAddressLo, PAhi (=0), data
;
OUT_PORT:
;
;  Build "STA  AAAA" in RAM
        LDA     #STA_OP
        STA     CODEBUF+0
;
;  Set port address
        LDA     COMBUF+2
        STA     CODEBUF+1
        LDA     COMBUF+3
        STA     CODEBUF+2
;
;  Set return after STA
        LDA     #RTS_OP
        STA     CODEBUF+3
;
;  Get data
        LDA     COMBUF+4
;
;  Write value to port
        JSR     CODEBUF         ;PUT BYTE FROM A
;
;  Do not read port to verify (some I/O devices don't like it)
;
;  Return status of OK
        LDA     #0
               JMP     SENDSTATUS

;===========================================================================
;  Build status return with value from "A"
;

SENDSTATUS:    STA     COMBUF+2                ;SET STATUS
        LDA     #1
        STA     COMBUF+1                ;SET LENGTH
        JMP     SEND

;===========================================================================
;  Append checksum to COMBUF and send to master
;
SEND:   JSR     CHECKSUM                ;GET A=CHECKSUM, X->checksum location
               EOR     #$FF
        CLC
        ADC     #1
        STA     COMBUF,X                ;STORE NEGATIVE OF CHECKSUM
;
;  Send buffer to master
        LDX     #0                      ;POINTER TO DATA
        LDY     COMBUF+1                ;LENGTH OF DATA
        INY                             ;PLUS FUNCTION, LENGTH, CHECKSUM
        INY
        INY
SND10:  LDA     COMBUF,X
        JSR     PUTCHAR                 ;SEND A BYTE
        INX
        DEY
        BNE     SND10
        JMP     MAIN                    ;BACK TO MAIN LOOP

;===========================================================================
;  Compute checksum on COMBUF.  COMBUF+1 has length of data,
;  Also include function byte and length byte
;
;  Returns:
;       A = checksum
;       X = pointer to next byte in buffer (checksum location)
;       Y is scratched
;
CHECKSUM:
        LDX     #0                      ;pointer to buffer
        LDY     COMBUF+1                ;length of message
        INY                             ;plus function, length
        INY
        LDA     #0                      ;init checksum to 0
CHK10:  CLC
        ADC     COMBUF,X
        INX
        DEY
        BNE     CHK10                   ;loop for all
        RTS                             ;return with checksum in A

;**********************************************************************
;
;  VECTORS THROUGH RAM
;
;  ON THE MELPS740 (from which this was ported...)
;  We observe a nasty problem in interrupts during debug with BRK:
;  Suppose 2 breakpoints are set with interrupts disabled (such as within an
;  interrupt handler).  When "GO" is typed to continue from the first,
;  there will most often be another interrupt pending due to the time
;  spent in the monitor.  While this interrupt should be held off because
;  interrupts are disabled, we find that execution of the second BRK fetches
;  the VECTOR FOR THE PENDING INTERRUPT, rather than the BREAK vector.
;
;  To allow breakpoints in interrupt handlers, we have installed code
;  to check the break bit for ALL interrupts.  If clear, control will be
;  passed normally through the RAM vector.  If set, we will enter the
;  breakpoint service routine.
;
;  According to Rockwell, this will also occur on the 6502, but not on
;  the 65C02.
;
;
;  ON THE MELPS740 (from which this was ported...)
;  It seems that "B" gets set on the stack but NOT in the PS in the
;  handler.  Manual claims both get set...
;  Not sure how this works on a 6502 or a 65C02.
;  If your breakpoints or single-step don't work, then change this code
;  to look at PS rather than stack.
;
;  IRQ/BREAK.  VECTOR THROUGH RAM
;  First, check for BRK (breakpoint) interrupt
.IRQ:   CLD
        STA     REG_A           ;SAVE A
;
;  Test B bit in stacked condition codes
        PLA
        PHA                     ;GET COPY OF PRE-INT STATUS REG
;
;  Test B bit in current condition codes
;;      PHP
;;      PLA                     ;GET COPY OF CURRENT STATUS REG
;
        AND     #B
        BNE     GOBREAKP        ;SET: DO BREAKPOINT CODE
        LDA     REG_A           ;ELSE RESTORE ACC.
               JMP     (RAMVEC+04)     ;JUMP THROUGH RAM VECTOR
;
;  IRQ NOT USED (ADDRESS PLACED IN RAMVEC+04H BY INIT)
.IRQX:  LDA     #2              ;TARGET STOP TYPE
        JMP     GOBREAK         ;COMPLAIN, ENTER MONITOR
;
;
;  NMI.  VECTOR THROUGH RAM
;  First, check for BRK (breakpoint) interrupt
.NMI:   CLD
        STA     REG_A           ;SAVE A
;
;  Test B bit in stacked condition codes
        PLA
        PHA                     ;GET COPY OF PRE-INT STATUS REG
;
;  Test B bit in current condition codes
;;      PHP
;;      PLA                     ;GET COPY OF CURRENT STATUS REG
;
        AND     #B
        BNE     GOBREAKP        ;SET: DO BREAKPOINT CODE
        LDA     REG_A           ;ELSE RESTORE ACC.
        JMP     (RAMVEC+0)      ;JUMP THROUGH RAM VECTOR
;
;  NMI NOT USED (ADDRESS PLACED IN RAMVEC+00H BY INIT)
.NMIX:  LDA     #3              ;TARGET STOP TYPE
        JMP     GOBREAK         ;ALWAYS DO BREAKPOINT CODE
;
;
;  BREAK ENTRY.
GOBREAKP:
        LDA     #1              ;STATE IS "BREAKPOINT"
GOBREAK:
        JMP     INT_ENTRY       ;ENTER MONITOR
;
;
;  INTERRUPT VECTORS
        SEG     VECTORS
               org   $fffa
               DC.W  .NMI            ;FFFA NMI
               DC.W  RESET           ;FFFC RESET
               DC.W  .IRQ            ;FFFE IRQ, BRK
;
        END     RESET
