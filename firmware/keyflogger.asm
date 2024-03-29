;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Copyright (C) 2021 Michael Brown <mbrown@fensystems.co.uk>.
;;;
;;; This program is free software; you can redistribute it and/or
;;; modify it under the terms of the GNU General Public License as
;;; published by the Free Software Foundation; either version 2 of the
;;; License, or any later version.
;;;
;;; This program is distributed in the hope that it will be useful, but
;;; WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;;; General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program; if not, write to the Free Software
;;; Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
;;; 02110-1301, USA.
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Internal API
;;;
;;; All code (other than the idle loop itself) executes with
;;; interrupts disabled.
;;;
;;; FSR1 is the data stack pointer
;;;
;;; Functions may modify W, FSR0, and scratch registers com_rN
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	radix	dec
	include	"p16f1454.inc"
	include "usb.inc"

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Macros
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; Align program counter
;;;
#define ALIGN(x) ( $ + ( ( -$ ) & ( (x) - 1 ) ) )

;;; Calculate linear address from banked address
;;;
#define ADR(x) ( 0x2000 + (x) - 0x20 - ( 0x30 * ( (x) / 0x80 ) ) )

;;; Top of general purpose RAM
;;;
#define RAM_TOP 0x064f

;;; Log character to UART
;;;
logch	macro	ch
#if DEBUG
	movlw	ch
	call	uart_tx_char
#endif
	endm

;;; Log register value to UART
;;;
logf	macro	reg
#if DEBUG
	movf	reg, w
	call	uart_tx_hex
#endif
	endm

;;; Drain UART transmit buffer
;;;
logsync	macro
#if DEBUG
	call	uart_tx_sync
#endif
	endm

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Definitions
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; Port A pin assignments
;;;
RA0_USBDP	equ	RA0	; USB D+
RA1_USBDN	equ	RA1	; USB D-
RA3_nMCLR	equ	RA3	; Reset
RA4_NC		equ	RA4	; Not connected
RA5_NC		equ	RA5	; Not connected

;;; Port C pin assignments
;;;
RC0_ICSPDAT	equ	RC0	; In-circuit programming data
RC1_ICSPCLK	equ	RC1	; In-circuit programming clock
RC2_nLED	equ	RC2	; Status LED
RC3_NC		equ	RC3	; Not connected
RC4_nTX		equ	RC4	; UART TX (inverted)
RC5_RX		equ	RC5	; UART RX

;;; USB parameters
;;;
USB_VENDOR	equ	0x1209	; https://pid.codes/
USB_PRODUCT	equ	0xebeb	; https://pid.codes/1209/EBEB/
USB_VERSION	equ	0x0100	; Product version 1.00
USB_MTU_EP0	equ	8	; MTU for control endpoint
USB_MTU_DATA	equ	8	; MTU for keyboard/mouse data endpoints

;;; USB state flags
;;;
USB_SET_ADDR	equ	7

;;; Status LED
;;;
LED_COUNT_SEC	equ	183	; Timer0 overflows at 183Hz
LED_COUNT_FAST	equ	( LED_COUNT_SEC / 20 )
LED_COUNT_SLOW	equ	( LED_COUNT_SEC / 2 )
LED_ACTIVITY	equ	0	; Activity: single fast blink
LED_ERROR	equ	4	; Error: several slow blinks

;;; Communication protocol receive state
;;;
RX_FILL		equ	7	; Bit 7: accumulate hex digits
RX_MS		equ	6	; Bit 6: accumulate into mouse buffer
RX_DISCARD	equ	5	; Bit 5: discard characters
RX_DONE		equ	4	; Bit 4: accumulation complete
RX_MASK		equ	0x0f	; Bits 0-3: accumulated hex digit index

;;; USB polling intervals
;;;
;;; The maximum latency for data received via an interrupt OUT
;;; endpoint to be transmitted via the UART is around 4ms (115200
;;; baud, 19 characters per message, two channels).  With a 10ms
;;; polling interval for the interrupt IN endpoints, this gives a
;;; maximum latency of 14ms from data being received via an interrupt
;;; OUT endpoint until that data has been transmitted via the
;;; corresponding interrupt IN endpoint on the other side.
;;;
;;; Setting the polling interval for the interrupt OUT endpoints to
;;; 20ms ensures that we can never overflow the UART transmit ring,
;;; and that an accumulated interrupt IN endpoint buffer is guaranteed
;;; to have been consumed before the next accumulation begins.
;;;
USB_POLL_IN	equ	10	; 10ms polling
USB_POLL_OUT	equ	20	; 20ms polling

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Device configuration
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; Configuration word 1
;;;
cfg1	set	0x3fff
cfg1	set	cfg1 & _FOSC_INTOSC
cfg1	set	cfg1 & _WDTE_NSLEEP

;;; Configuration word 2
;;;
cfg2	set	0x3fff
cfg2	set	cfg2 & _CPUDIV_NOCLKDIV
cfg2	set	cfg2 & _WRT_ALL

;;; Apply configuration
;;;
	__config	_CONFIG1, cfg1
	__config	_CONFIG2, cfg2

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Memory layout
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; USB buffer descriptors (location fixed by hardware)
;;;

	cblock	0x20

;;; EP0 OUT
;;;
bd_ep0out:		0
bd_ep0out_stat:		1
bd_ep0out_cnt:		1
bd_ep0out_adrl:		1
bd_ep0out_adrh:		1

;;; EP0 IN
;;;
bd_ep0in:		0
bd_ep0in_stat:		1
bd_ep0in_cnt:		1
bd_ep0in_adrl:		1
bd_ep0in_adrh:		1

;;; EP1 OUT
;;;
bd_ep1out:		0
bd_ep1out_stat:		1
bd_ep1out_cnt:		1
bd_ep1out_adrl:		1
bd_ep1out_adrh:		1

;;; EP1 IN
;;;
bd_ep1in:		0
bd_ep1in_stat:		1
bd_ep1in_cnt:		1
bd_ep1in_adrl:		1
bd_ep1in_adrh:		1

;;; EP2 OUT
;;;
bd_ep2out:		0
bd_ep2out_stat:		1
bd_ep2out_cnt:		1
bd_ep2out_adrl:		1
bd_ep2out_adrh:		1

;;; EP2 IN
;;;
bd_ep2in:		0
bd_ep2in_stat:		1
bd_ep2in_cnt:		1
bd_ep2in_adrl:		1
bd_ep2in_adrh:		1

;;; EP3 OUT
;;;
bd_ep3out:		0
bd_ep3out_stat:		1
bd_ep3out_cnt:		1
bd_ep3out_adrl:		1
bd_ep3out_adrh:		1

;;; EP3 IN
;;;
bd_ep3in:		0
bd_ep3in_stat:		1
bd_ep3in_cnt:		1
bd_ep3in_adrl:		1
bd_ep3in_adrh:		1

	endc

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Common data (accessible in any page)
;;;

	cblock	0x70

com_r0:			1	; Scratch register 0
com_r1:			1	; Scratch register 1
com_led_count:		1	; Status LED blink countdown value
com_led_state:		1	; Status LED blink rate and pending flag
com_uart_tx_prod:	1	; UART transmit producer index
com_uart_tx_cons:	1	; UART transmit consumer indexs
com_rx_state:		1	; UART receive state
com_usb_data:		1	; Current descriptor data pointer (low byte)
com_usb_len:		1	; Current descriptor remaining length
com_usb_flags:		1	; USB state flags

	endc

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; USB buffers (must reside within a dual-port RAM bank)
;;;

	cblock	0xa0

ep_buffer:		0

;;; EP0 OUT
;;;
ep0out_buffer:		0
ep0out_bmRequestType:	1
ep0out_bRequest:	1
ep0out_wValue:		0
ep0out_wValueL:		1
ep0out_wValueH:		1
ep0out_wIndex:		0
ep0out_wIndexL:		1
ep0out_wIndexH:		1
ep0out_wLength:		0
ep0out_wLengthL:	1
ep0out_wLengthH:	1

;;; EP0 IN
;;;
ep0in_buffer:		USB_MTU_EP0

;;; EP2 OUT
;;;
ep2out_buffer:		USB_MTU_DATA

;;; EP2 IN
;;;
ep2in_buffer:		USB_MTU_DATA

;;; EP3 OUT
;;;
ep3out_buffer:		USB_MTU_DATA

;;; EP3 IN
;;;
ep3in_buffer:		USB_MTU_DATA

ep_buffer_end:		0

	endc

	if	( high ADR ( ep_buffer_end ) ) != ( high ADR ( ep_buffer ) )
	error	"USB buffers cross a page boundary"
	endif

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; UART transmit ring (accessed via linear address)
;;;

UART_TX_LEN_LOG2	equ	7
UART_TX_LEN		equ	1 << UART_TX_LEN_LOG2
UART_TX_MASK		equ	UART_TX_LEN - 1

	cblock	0x4d0
uart_tx_ring:		UART_TX_LEN
	endc

	if	( low ADR ( uart_tx_ring ) ) != 0
	error	"UART transmit ring is misaligned"
	endif

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Data stack (accessed via FSR1)
;;;

STACK_LEN		equ	48

	cblock	0x620

stack:			STACK_LEN
stack_end:		0

	endc

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Main program
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Reset vector
;;;
	org	0x0000
rst:
	goto	init

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Interrupt vector
;;;
	org	0x0004
irq:
	;; Select peripheral interrupt flag register bank
	banksel	PIR1

	;; Handle USB interrupts
	btfsc	PIR2, USBIF
	goto	usb_irq

	;; Handle Timer0 overflow interrupt
	btfsc	INTCON, TMR0IF
	goto	led_irq

	;; Handle UART receive interrupt
	btfsc	PIR1, RCIF
	goto	uart_rx_irq

	;; Handle UART transmit interrupt
	btfss	PIR1, TXIF
	bra	irq_not_uart_tx
	banksel	PIE1
	btfsc	PIE1, TXIE
	goto	uart_tx_irq
irq_not_uart_tx:

	;; Unexpected interrupt: reset system
	logch	'I'
	logch	'R'
	logch	'Q'
	logch	'?'
	logch	'\n'
	logsync
	reset

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Initialise system
;;;
init:
	;; Configure oscillator: 48MHz via 3xPLL
	banksel	OSCCON
	bsf	OSCCON, IRCF3
	bsf	OSCCON, SPLLMULT

	;; Wait for oscillator to become stable
	call	stabilise

	;; Enable active clock tuning (required for USB)
	banksel	ACTCON
	bsf	ACTCON, ACTSRC
	bsf	ACTCON, ACTEN

	;; Enable weak pull-ups and configure Timer0 to run at Fosc/1024
	banksel	OPTION_REG
	movlw	( 1 << PS2 ) | ( 1 << PS1 ) | ( 1 << PS0 )
	movwf	OPTION_REG

	;; Enable digital I/O
	banksel	ANSELA
	clrf	ANSELA
	clrf	ANSELC

	;; Set all outputs to a known state
	banksel	LATA
	clrf	LATA
	clrf	LATC

	;; Enable digital outputs (including unused pins lacking pull-ups)
	banksel	TRISC
	movlw	( 1 << RC5_RX )
	movwf	TRISC

	;; Clear RAM
	movlw	low ADR ( RAM_TOP )
	movwf	FSR0L
	movlw	high ADR ( RAM_TOP )
	movwf	FSR0H
	clrw
zram:	movwi	FSR0--			; Clear DPR/GPR via linear window
	btfsc	FSR0H, 5		; ...until we fall below 0x2000
	bra	zram
	clrf	FSR0H
zcom:	movwi	FSR0--			; Clear common RAM via bank 1
	btfsc	FSR0L, 4		; ...until we fall below 0x00f0
	bra	zcom

	;; Initialise stack pointer
	movlw	high stack_end
	movwf	FSR1H
	movlw	low stack_end
	movwf	FSR1L

	;; Initialise status LED
	call	led_init

	;; Initialise UART
	call	uart_init
	movlw	'\n'
	call	uart_tx_char

	;; Enable USB
	call	usb_init

	;; Enable interrupts
	banksel	PIE1
	bsf	PIE1, RCIE
	bsf	PIE2, USBIE
	movlw	( 1 << GIE ) | ( 1 << PEIE ) | ( 1 << TMR0IE )
	movwf	INTCON

	;; Idle loop
	;;
	;; The PIC provides no way to sleep the CPU without also
	;; stopping the internal oscillator, so the "idle" loop must
	;; unfortunately be a spin loop.
idle:
	clrwdt
	bra	idle

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Oscillator stabilisation
;;;
stabilise:
	banksel	OSCSTAT
	comf	OSCSTAT, w
	andlw	( 1 << PLLRDY ) | ( 1 << HFIOFR ) | ( 1 << LFIOFR )
	bnz	stabilise
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Status LED
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Initialise LED
;;;
led_init:
	;; Expire software countdown timer on first interrupt
	movlw	1
	movwf	com_led_count
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Handle LED (Timer0) interrupt
;;;
led_irq:
	;; Clear timer interrupt
	bcf	INTCON, TMR0IF

	;; Do nothing else until software countdown timer expires
	decfsz	com_led_count, f
	retfie

	;; Toggle LED state if applicable
	banksel	LATC
	movlw	( 1 << RC2_nLED )
	tstf	com_led_state		; If a blink is pending
	skpnz
	btfsc	LATC, RC2_nLED		; ...or LED is currently off
	xorwf	LATC, f			; ...then toggle LED state

	;; Consume blink state and reload counter
	movlw	LED_COUNT_FAST
	lsrf	com_led_state, f
	skpz
	movlw	LED_COUNT_SLOW
	movwf	com_led_count

	;; Return
	retfie

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; UART
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Initialise UART
;;;
uart_init:
	;; Set baud rate to 115200
	banksel	BAUDCON
	bsf	BAUDCON, SCKP
	bsf	BAUDCON, BRG16
	movlw	0x67
	movwf	SPBRGL
	clrf	SPBRGH
	bsf	TXSTA, BRGH
	bsf	TXSTA, TXEN
	bsf	RCSTA, CREN
	bsf	RCSTA, SPEN
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Transmit next byte from transmit ring
;;;
uart_tx_next:
	;; Get next byte from transmit ring
	movlw	high ADR ( uart_tx_ring )
	movwf	FSR0H
	movf	com_uart_tx_cons, w
	andlw	UART_TX_MASK
	movwf	FSR0L
	moviw	FSR0

	;; Transmit byte
	banksel	TXREG
	movwf	TXREG

	;; Increment consumer counter
	incf	com_uart_tx_cons

	;; Disable transmit interrupt if ring is empty
	movf	com_uart_tx_cons, w
	subwf	com_uart_tx_prod, w
	banksel	PIE1
	skpnz
	bcf	PIE1, TXIE

	;; Return
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Handle UART transmit interrupt
;;;
uart_tx_irq:
	;; Transmit next byte
	call	uart_tx_next

	;; Return
	retfie

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Drain UART transmit ring
;;;
;;; Preserves all registers except STATUS
;;;
uart_tx_sync:
	;; Preserve W, BSR, and FSR0 on stack
	movwi	--FSR1
	movf	BSR, w
	movwi	--FSR1
	movf	FSR0H, w
	movwi	--FSR1
	movf	FSR0L, w
	movwi	--FSR1

	;; Loop until transmit ring is empty
uart_tx_sync_loop:
	banksel	PIE1
	btfss	PIE1, TXIE
	bra	uart_tx_sync_wait_trmt

	;; Wait for TXREG to become empty
uart_tx_sync_wait_txreg:
	banksel	PIR1
	btfss	PIR1, TXIF
	bra	uart_tx_sync_wait_txreg

	;; Transmit next byte
	call	uart_tx_next

	;; Loop until transmit ring is empty
	bra	uart_tx_sync_loop

	;; Wait for transmitter to become idle
uart_tx_sync_wait_trmt:
	banksel	TXSTA
	btfss	TXSTA, TRMT
	bra	uart_tx_sync_wait_trmt

	;; Restore registers and return
	moviw	FSR1++
	movwf	FSR0L
	moviw	FSR1++
	movwf	FSR0H
	moviw	FSR1++
	movwf	BSR
	moviw	FSR1++
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Transmit character in W (with CRLF conversion)
;;;
;;; Preserves all registers except STATUS
;;;
uart_tx_char:
	;; Preserve W on stack
	movwi	--FSR1

	;; Convert LF to CRLF
	sublw	'\n'
	movlw	'\r'
	skpnz
	call	uart_tx_char

	;; Preserve FSR0 on stack
	movf	FSR0H, w
	movwi	--FSR1
	movf	FSR0L, w
	movwi	--FSR1

	;; Check for space in transmit ring
	movf	com_uart_tx_cons, w
	subwf	com_uart_tx_prod, w
	btfsc	WREG, UART_TX_LEN_LOG2
	bsf	com_led_state, LED_ERROR
	btfsc	WREG, UART_TX_LEN_LOG2
	bra	uart_tx_char_done

	;; Add byte to transmit ring
	movlw	high ADR ( uart_tx_ring )
	movwf	FSR0H
	movf	com_uart_tx_prod, w
	andlw	UART_TX_MASK
	movwf	FSR0L
	moviw	2[FSR1]			; Preserved W
	movwi	FSR0

	;; Increment producer counter
	incf	com_uart_tx_prod

	;; Enable transmit interrupt, preserving bank selection register
	movf	BSR, w
	banksel	PIE1
	bsf	PIE1, TXIE
	movwf	BSR

uart_tx_char_done:
	;; Restore registers and return
	moviw	FSR1++
	movwf	FSR0L
	moviw	FSR1++
	movwf	FSR0H
	moviw	FSR1++
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Transmit hex digit in W
;;;
;;; Preserves all registers except STATUS
;;;
uart_tx_hex_digit:
	;; Preserve registers on stack
	movwi	--FSR1

	;; Convert to ASCII character
	andlw	0x0f
	addlw	-10
	skpnc
	addlw	'a' - '0' - 10
	addlw	'0' + 10

	;; Transmit character
	call	uart_tx_char

	;; Restore registers and return
	moviw	FSR1++
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Transmit hex byte in W
;;;
;;; Preserves all registers except STATUS
;;;
uart_tx_hex:
	;; Print high nibble
	swapf	WREG
	call	uart_tx_hex_digit

	;; Print low nibble
	swapf	WREG
	call	uart_tx_hex_digit

	;; Return
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Handle UART receive interrupt
;;;
uart_rx_irq:
	;; Check for errors
	banksel	RCREG
	btfsc	RCSTA, FERR
	bra	uart_rx_irq_error
	btfsc	RCSTA, OERR
	bra	uart_rx_irq_error

	;; Consume received byte
	movf	RCREG, w

	;; Handle received byte
	call	rx

	;; Return
	retfie

uart_rx_irq_error:
	;; Consume and discard byte
	movf	RCREG, w

	;; Disable and reenable receiver
	bcf	RCSTA, CREN
	bsf	RCSTA, CREN

	;; Report error
	bsf	com_led_state, LED_ERROR

	;; Return
	retfie

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Inter-IC communication protocol
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Transmit keyboard/mouse report
;;;
tx_kb:
	;; Transmit 'K'
	movlw	'K'
	call	uart_tx_char

	;; Transmit keyboard report buffer
	movlw	bd_ep2out
	bra	tx

tx_ms:
	;; Transmit 'M'
	movlw	'M'
	call	uart_tx_char

	;; Transmit mouse report buffer
	movlw	bd_ep3out
tx:
	;; Preserve buffer descriptor offset on stack
	movwi	--FSR1

	;; Set FSR0 to address buffer descriptor
	movwf	FSR0L
	movlw	high bd_ep0out
	movwf	FSR0H

	;; Get buffer length into com_r0 and com_r1
	moviw	( bd_ep0out_cnt - bd_ep0out )[FSR0]
	movwf	com_r0
	movwf	com_r1

	;; Set FSR0 to address buffer contents
	moviw	( bd_ep0out_adrl - bd_ep0out )[FSR0]
	movwf	FSR0L
	movlw	high ADR(ep_buffer)
	movwf	FSR0H

	;; Transmit zero padding (if any)
tx_pad_loop:
	btfsc	com_r0, 3
	bra	tx_pad_loop_end
	movlw	0x00
	call	uart_tx_hex
	incf	com_r0
	bra	tx_pad_loop
tx_pad_loop_end:

	;; Transmit buffer contents (if any)
	incf	com_r1
	bra	tx_data_loop_test
tx_data_loop:
	moviw	FSR0++
	call	uart_tx_hex
tx_data_loop_test:
	decfsz	com_r1
	bra	tx_data_loop

	;; Transmit CRLF
	movlw	'\n'
	call	uart_tx_char

	;; Restore buffer descriptor offset, refill endpoint, and return
	moviw	FSR1++
	goto	epintr_refill

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Handle received byte in W
;;;
rx:
	;; Check for CR or LF
	addlw	0 - '\r'
	skpnz
	bra	rx_crlf
	addlw	'\r' - '\n'
	skpnz
	bra	rx_crlf

	;; Discard all other characters if applicable
	btfsc	com_rx_state, RX_DISCARD
	return

	;; Check for excess characters after completing accumulation
	btfsc	com_rx_state, RX_DONE
	bra	rx_error

	;; Accumulate hex digit if accumulating
	btfsc	com_rx_state, RX_FILL
	bra	rx_hex_digit

	;; Check for '#' command
	addlw	'\n' - '#'
	skpnz
	bra	rx_comment

	;; Check for 'K' command
	addlw	'#' - 'K'
	skpnz
	bra	rx_kb

	;; Check for 'M' command
	addlw	'K' - 'M'
	skpnz
	bra	rx_ms

rx_error:
	;; Report error and return
	bsf	com_led_state, LED_ERROR
	clrf	com_rx_state
	bsf	com_rx_state, RX_DISCARD
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Handle UART received comment command
;;;
rx_comment:
	;; Discard remaining characters
	bsf	com_rx_state, RX_DISCARD
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Handle UART received keyboard/mouse command
;;;
rx_ms:
	;; Set accumulation buffer as mouse buffer
	bsf	com_rx_state, RX_MS
rx_kb:
	;; Accumulate hex digits
	bsf	com_rx_state, RX_FILL
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Handle UART received hex digit
;;;
rx_hex_digit:
	;; Check for decimal digit
	addlw	'\n' - '0'
	skpc
	bra	rx_error
	addlw	-10
	skpc
	bra	rx_hex_numeric

	;; Check for uppercase hex digit
	addlw	'0' + 10 - 'A'
	skpc
	bra	rx_error
	addlw	-6
	skpc
	bra	rx_hex_alpha

	;; Check for lowercase hex digit
	addlw	'A' + 6 - 'a'
	skpc
	bra	rx_error
	addlw	-6
	skpnc
	bra	rx_error

	;; Convert to raw digit value
rx_hex_alpha:
	addlw	6
rx_hex_numeric:
	addlw	10

	;; Preserve raw digit value in scratch register
	movwf	com_r0

	;; Set up FSR0 for access to selected buffer
	movlw	high ADR ( ep_buffer )
	movwf	FSR0H
	movlw	low ADR ( ep2in_buffer )
	btfsc	com_rx_state, RX_MS
	movlw	low ADR ( ep3in_buffer )
	movwf	FSR0L
	lsrf	com_rx_state, w
	andlw	( RX_MASK >> 1 )
	addwf	FSR0L, f

	;; Append nibble to buffer
	moviw	FSR0
	swapf	WREG
	andlw	~0x0f
	iorwf	com_r0, w
	movwi	FSR0

	;; Increment offset and mark as complete when buffer is full
	incf	com_rx_state

	;; Return
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Handle UART received CR or LF
;;;
rx_crlf:
	;; Do nothing unless accumulation was in progress
	btfss	com_rx_state, RX_FILL
	bra	rx_crlf_done

	;; Fail if accumulation has not completed
	btfss	com_rx_state, RX_DONE
	bsf	com_led_state, LED_ERROR
	btfss	com_rx_state, RX_DONE
	bra	rx_crlf_done

	;; Refill completed endpoint
	movlw	bd_ep2in
	btfsc	com_rx_state, RX_MS
	movlw	bd_ep3in
	call	epintr_refill

rx_crlf_done:
	;; Reset UART receive state and return
	clrf	com_rx_state
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; USB interface
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; (Re)initialise USB
;;;
usb_init:
	;; Configure endpoint buffer addresses and reset ownership
	banksel	bd_ep0out
	movlw	low ADR(ep0out_buffer)
	movwf	bd_ep0out_adrl
	movlw	low ADR(ep0in_buffer)
	movwf	bd_ep0in_adrl
	movlw	low ADR(ep2out_buffer)
	movwf	bd_ep2out_adrl
	movlw	low ADR(ep2in_buffer)
	movwf	bd_ep2in_adrl
	movlw	low ADR(ep3out_buffer)
	movwf	bd_ep3out_adrl
	movlw	low ADR(ep3in_buffer)
	movwf	bd_ep3in_adrl
	movlw	high ADR(ep_buffer)
	movwf	bd_ep0out_adrh
	movwf	bd_ep0in_adrh
	movwf	bd_ep2out_adrh
	movwf	bd_ep2in_adrh
	movwf	bd_ep3out_adrh
	movwf	bd_ep3in_adrh
	clrf	bd_ep0out_stat
	clrf	bd_ep0in_stat
	clrf	bd_ep2out_stat
	clrf	bd_ep2in_stat
	clrf	bd_ep3out_stat
	clrf	bd_ep3in_stat

	;; Initialise EP0
	call	ep0_reset
	call	ep0out_refill
	call	ep0in_refill

	;; Initialise interrupt endpoints
	movlw	bd_ep2out
	call	epintr_refill
	movlw	bd_ep2in
	call	epintr_refill
	movlw	bd_ep3out
	call	epintr_refill
	movlw	bd_ep3in
	call	epintr_refill

	;; Configure USB registers
	banksel	UCON
	movlw	( 1 << EPHSHK ) | ( 1 << EPOUTEN ) | ( 1 << EPINEN )
	movwf	UEP0
	movlw	( 1 << EPHSHK ) | ( 1 << EPOUTEN ) | ( 1 << EPINEN )
	movwf	UEP2
	movwf	UEP3
	bsf	UCFG, FSEN
	bsf	UCFG, UPUEN
	bsf	UCON, USBEN
	movlw	( 1 << IDLEIE ) | ( 1 << TRNIE ) | ( 1 << URSTIE )
	movwf	UIE
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Clear USB setup transaction state
;;;
ep0_reset:
	clrf	com_usb_data
	clrf	com_usb_len
	clrf	com_usb_flags
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Refill EP0 OUT buffer
;;;
ep0out_refill:
	;; Initialise buffer for SETUP or STATUS
	banksel	bd_ep0out
	movlw	USB_MTU_EP0
	movwf	bd_ep0out_cnt
	movlw	( 1 << DTS ) | ( 1 << DTSEN )
	movwf	bd_ep0out_stat

	;; Pass ownership to USB subsystem
	bsf	bd_ep0out_stat, UOWN
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Refill EP0 IN buffer
;;;
ep0in_refill:
	;; Preserve stack pointer in scratch registers
	movf	FSR1L, w
	movwf	com_r0
	movf	FSR1H, w
	movwf	com_r1

	;; Prepare indirect registers for copying descriptor
	movf	com_usb_data, w
	movwf	FSR0L
	movlw	high usb_desc
	movwf	FSR0H
	movlw	low ep0in_buffer
	movwf	FSR1L
	movlw	high ep0in_buffer
	movwf	FSR1H

	;; Copy up to 8 bytes of descriptor
	banksel	bd_ep0in
	clrf	bd_ep0in_cnt
	tstf	com_usb_len
ep0in_refill_loop:
	skpz				; If remaining length is zero
	btfsc	bd_ep0in_cnt, 3		; ...or copied length is 8
	bra	ep0in_refill_loop_end	; ...then terminate loop
	moviw	FSR0++
	movwi	FSR1++
	incf	bd_ep0in_cnt, f
	incf	com_usb_data, f
	decf	com_usb_len, f
	bra	ep0in_refill_loop
ep0in_refill_loop_end:

	;; Update data toggle
	movf	bd_ep0in_stat, w
	andlw	( 1 << DTS )
	xorlw	( ( 1 << DTS ) | ( 1 << DTSEN ) )
	movwf	bd_ep0in_stat

	;; Pass ownership to USB subsystem
	bsf	bd_ep0in_stat, UOWN

	;; Restore stack pointer and return
	movf	com_r1, w
	movwf	FSR1H
	movf	com_r0, w
	movwf	FSR1L
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Refill interrupt endpoint buffer with descriptor at offset W
;;;
epintr_refill:
	;; Set FSR0 to address buffer descriptor
	movwf	FSR0L
	movlw	high bd_ep0out
	movwf	FSR0H

	;;  Reset buffer length
	movlw	USB_MTU_DATA
	movwi	( bd_ep0out_cnt - bd_ep0out )[FSR0]

	;; Update data toggle
	moviw	( bd_ep0out_stat - bd_ep0out )[FSR0]
	andlw	( 1 << DTS )
	xorlw	( ( 1 << DTS ) | ( 1 << DTSEN ) )
	movwi	( bd_ep0out_stat - bd_ep0out )[FSR0]

	;; Pass ownership to USB subsystem
	bsf	WREG, UOWN
	movwi	( bd_ep0out_stat - bd_ep0out )[FSR0]
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Handle USB interrupts
;;;
usb_irq:
	;; Indicate activity via status LED
	bsf	com_led_state, LED_ACTIVITY

	;; Check for USB reset
	banksel	UIR
	btfsc	UIR, URSTIF
	bra	usb_reset

	;; Check for USB suspend
	btfsc	UIR, IDLEIF
	bra	usb_suspend

	;; Handle according to endpoint and direction
	lsrf	USTAT, w
	lsrf	WREG, w
	brw
	bra	ep0out
	bra	ep0in
	reset
	reset
	bra	ep2out
	bra	ep2in
	bra	ep3out
	bra	ep3in

;;; Handle USB reset
;;;
usb_reset:
	;; Log reset
	logch	'#'
	logch	'U'
	logch	'R'
	logch	'S'
	logch	'T'
	logch	'\n'

	;; Reinitialise USB
	call	usb_init

	;; Clear USB reset interrupt
	banksel	UIR
	bcf	UIR, URSTIF
	bra	usb_done

;;; Handle EP0 IN completion
;;;
ep0in:
	;; Refill EP0 IN buffer
	call	ep0in_refill

	;; Check for pending SET ADDRESS
	btfss	com_usb_flags, USB_SET_ADDR
	bra	epN_done
	bcf	com_usb_flags, USB_SET_ADDR

	;; Set USB address
	movf	com_usb_data, w
	banksel	UADDR
	movwf	UADDR
	bra	epN_done

;;; Handle EP0 OUT completion
;;;
ep0out:
	;; Check for SETUP PID
	banksel	bd_ep0out
	movf	bd_ep0out_stat, w
	andlw	( 1 << PID3 ) | ( 1 << PID2 ) | ( 1 << PID1 ) | ( 1 << PID0 )
	xorlw	( 1 << PID3 ) | ( 1 << PID2 ) | ( 0 << PID1 ) | ( 1 << PID0 )
	bz	ep0setup

;;; Handle EP0 OUT DATA completion
;;;
ep0outdata:
	;; Refill EP0 OUT buffer
	call	ep0out_refill
	bra	epN_done

;;; Handle EP0 SETUP completion
;;;
ep0setup:
	;; Reset any in-progress operations
	call	ep0_reset

	;; Log SETUP request
	banksel	ep0out_buffer
	logch	'#'
	logch	'S'
	logf	ep0out_bmRequestType
	logf	ep0out_bRequest
	logch	':'
	logf	ep0out_wValueH
	logf	ep0out_wValueL
	logch	':'
	logf	ep0out_wIndexH
	logf	ep0out_wIndexL
	logch	':'
	logf	ep0out_wLengthH
	logf	ep0out_wLengthL
	logch	'\n'
	logsync

	;; Check for standard device SET requests
	tstf	ep0out_bmRequestType
	bz	ep0setup_std_dev_set

	;; Check for standard device GET requests
	movlw	0x80
	subwf	ep0out_bmRequestType, w
	bz	ep0setup_std_dev_get

	;; Check for standard interface GET requests
	movlw	0x81
	subwf	ep0out_bmRequestType, w
	bz	ep0setup_std_intf_get

	;; Check for class-specific interface SET requests
	movlw	0x21
	subwf	ep0out_bmRequestType, w
	bz	ep0setup_cls_intf_set

	;; Unrecognised request
	bra	ep0setup_bad

;;; Handle standard device SET requests
;;;
ep0setup_std_dev_set
	;; Check for SET ADDRESS
	movlw	0x05
	subwf	ep0out_bRequest, w
	bz	ep0setup_set_addr

	;; Check for SET CONFIGURATION
	movlw	0x09
	subwf	ep0out_bRequest, w
	bz	ep0setup_set_config

	;; Unrecognised standard device SET
	bra	ep0setup_bad

;;; Handle standard device GET requests
;;;
ep0setup_std_dev_get:
	;; Check for GET DESCRIPTOR
	movlw	0x06
	subwf	ep0out_bRequest, w
	bz	ep0setup_get_desc

	;; Unrecognised standard device GET
	bra	ep0setup_bad

;;; Handle standard interface GET requests
;;;
ep0setup_std_intf_get:
	;; Check for GET REPORT
	movlw	0x06
	subwf	ep0out_bRequest, w
	bz	ep0setup_get_desc_report

	;; Unrecognised standard interface GET
	bra	ep0setup_bad

;;; Handle class-specific interface SET requests
;;;
ep0setup_cls_intf_set:
	;; Accept and ignore
	bra	ep0setup_good

;;; Handle SET ADDRESS
;;;
ep0setup_set_addr:
	;; Record device address to be applied in STATUS stage
	movf	ep0out_wValueL, w
	movwf	com_usb_data
	bsf	com_usb_flags, USB_SET_ADDR
	bra	ep0setup_good

;;; Handle SET CONFIGURATION
;;;
ep0setup_set_config:
	;;  Accept and ignore
	bra ep0setup_good

;;; Handle GET DESCRIPTOR
;;;
ep0setup_get_desc:
	;; Check for DEVICE descriptor type
	movlw	0x01
	subwf	ep0out_wValueH, w
	bz	ep0setup_get_desc_device

	;; Check for CONFIGURATION descriptor type
	movlw	0x02
	subwf	ep0out_wValueH, w
	bz	ep0setup_get_desc_config

	;; Check for STRING descriptor type
	movlw	0x03
	subwf	ep0out_wValueH, w
	bz	ep0setup_get_desc_string

	;; Unrecognised descriptor type
	bra	ep0setup_bad

;;; Handle GET DEVICE DESCRIPTOR
;;;
ep0setup_get_desc_device:
	;; Send device descriptor
	movlw	low usb_desc_device
	bra	ep0setup_good_desc

;;; Handle GET CONFIGURATION DESCRIPTOR
;;;
ep0setup_get_desc_config:
	;; Send configuration descriptor
	movlw	USB_DESC_CONFIG_TLEN
	movwf	com_usb_len
	movlw	low usb_desc_config
	bra	ep0setup_good_desc

;;; Handle GET STRING DESCRIPTOR
;;;
ep0setup_get_desc_string:
	;; Get string index
	movf	ep0out_wValueL, w

	;; Check string index
	addlw	-USB_STRING_COUNT
	bc	ep0setup_bad

	;; Look up string descriptor
	addlw	USB_STRING_COUNT + low usb_stringtab
	movwf	FSR0L
	movlw	high usb_stringtab
	movwf	FSR0H
	moviw	FSR0

	;; Send string descriptor
	bra	ep0setup_good_desc

;;; Handle GET REPORT DESCRIPTOR
;;;
ep0setup_get_desc_report:
	;; Check for HID REPORT descriptor type
	movlw	0x22
	subwf	ep0out_wValueH, w
	bz	ep0setup_get_desc_hid

	;; Unrecognised descriptor type
	bra	ep0setup_bad

;;; Handle GET HID REPORT DESCRIPTOR
;;;
ep0setup_get_desc_hid:
	;; Check interface number
	btfsc	ep0out_wIndexL, 0
	bra	ep0setup_get_desc_hid_ms
ep0setup_get_desc_hid_kb:
	;; Send keyboard HID report descriptor
	movlw	USB_DESC_REPORT_KB_LEN
	movwf	com_usb_len
	movlw	low usb_desc_report_kb
	bra	ep0setup_good_desc
ep0setup_get_desc_hid_ms:
	;; Send mouse HID report descriptor
	movlw	USB_DESC_REPORT_MS_LEN
	movwf	com_usb_len
	movlw	low usb_desc_report_ms
	bra	ep0setup_good_desc

;;; Complete USB interrupt handling
;;;
ep0setup_bad:
	;; Stall EP0 until next SETUP
	banksel	bd_ep0out
	movlw	( 1 << UOWN ) | ( 1 << BSTALL )
	movwf	bd_ep0out_stat
	movwf	bd_ep0in_stat
	bra	ep0setup_done
ep0setup_good_desc:
	;; Store descriptor offset
	movwf	com_usb_data
	;; Look up descriptor length if not already set
	movwf	FSR0L
	movlw	high usb_desc
	movwf	FSR0H
	movf	com_usb_len, w
	skpnz
	moviw	FSR0
	movwf	com_usb_len
	;; Limit to requested length
	banksel	ep0out_buffer
	tstf	ep0out_wLengthH
	bnz	ep0setup_good
	movf	com_usb_len, w
	subwf	ep0out_wLengthL, w
	bc	ep0setup_good
	movf	ep0out_wLengthL, w
	movwf	com_usb_len
ep0setup_good:
	;; Refill EP0 OUT buffer
	call	ep0out_refill
	;; Reset EP0 IN data toggle
	banksel	bd_ep0in
	bcf	bd_ep0in_stat, DTS
	;; Refill EP0 IN buffer
	call	ep0in_refill
ep0setup_done:
	;; Re-enable packet processing
	banksel	UCON
	bcf	UCON, PKTDIS
epN_done:
	;; Clear USB transaction interrupt and permit next USB transaction
	banksel	UIR
	bcf	UIR, TRNIF
usb_done:
	;; Clear USB interrupt
	banksel	PIR2
	bcf	PIR2, USBIF
	retfie

;;; Handle EP2 OUT completion
;;;
ep2out:
	;; Transmit keyboard report
	call	tx_kb

	;; Complete transaction
	bra	epN_done

;;; Handle EP2 IN completion
;;;
ep2in:
	;; Complete transaction
	bra	epN_done

;;; Handle EP3 OUT completion
;;;
ep3out:
	;; Transmit mouse report
	call	tx_ms

	;; Complete transaction
	bra	epN_done

;;; Handle EP3 IN completion
;;;
ep3in:
	;; Complete transaction
	bra	epN_done

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Handle USB suspend
;;;
usb_suspend:
	;; Log suspend
	logch	'#'
	logch	'U'
	logch	'I'
	logch	'D'
	logch	'L'
	logch	'E'
	logch	'\n'
	logsync

	;; Acknowledge idle interrupt and disable USB interrupts
	bcf	UIR, IDLEIF
	clrf	UIE
	banksel	PIR2
	bcf	PIR2, USBIF

	;; Preserve PIE1 on stack and disable UART interrupts
	banksel	PIE1
	movf	PIE1, w
	movwi	--FSR1
	clrf	PIE1

	;; Disable Timer0 and switch off LED
	bcf	INTCON, TMR0IE
	banksel	LATC
	bsf	LATC, RC2_nLED

	;; Suspend USB core
	banksel	UCON
	bsf	UCON, SUSPND

	;; Sleep (with interrupts disabled) until USB activity is detected
	bsf	UIE, ACTVIE
	sleep
	nop

	;; Wait for oscillator to become stable
	call	stabilise

	;; Enable Timer0 (and allow subsequent ISR to restore LED state)
	bsf	INTCON, TMR0IE

	;; Restore PIE1
	moviw	FSR1++
	banksel	PIE1
	movwf	PIE1

	;; Resume USB core
	banksel	UCON
	bcf	UCON, SUSPND
usb_resume_wait:
	bcf	UIR, ACTVIF
	btfsc	UIR, ACTVIF
	bra	usb_resume_wait

	;; Log resume
	logch	'#'
	logch	'U'
	logch	'A'
	logch	'C'
	logch	'T'
	logch	'V'
	logch	'\n'
	logsync

	;; Reenable USB interrupts
	movlw	( 1 << IDLEIE ) | ( 1 << TRNIE ) | ( 1 << URSTIE )
	movwf	UIE

	;; Complete USB activity interrupt
	bra	usb_done

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; USB descriptors
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	;; Align to a page boundary
	org	ALIGN ( 0x100 )
usb_desc:

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Device descriptor
;;;
usb_desc_device:
	;; bLength
	dt	USB_DESC_DEVICE_LEN
	;; bDescriptorType
	dt	0x01			; DEVICE descriptor type
	;; bcdUSB
	dt	0x00, 0x02		; USB specification version 2.00
	;; bDeviceClass
	dt	0x00			; Per interface
	;; bDeviceSubClass
	dt	0x00			; Per interface
	;; bDeviceProtocol
	dt	0x00			; Per interface
	;; bMaxPacketSize0
	dt	USB_MTU_EP0
	;; idVendor
	dt	low USB_VENDOR, high USB_VENDOR
	;; idProduct
	dt	low USB_PRODUCT, high USB_PRODUCT
	;; bcdDevice
	dt	low USB_VERSION, high USB_VERSION
	;; iVendor
	dt	( usb_stringtab_vendor - usb_stringtab )
	;; iProduct
	dt	( usb_stringtab_product - usb_stringtab )
	;; iSerialNumber
	dt	0x00			; No serial number
	;; bNumConfigurations
	dt	0x01			; Single configuration
usb_desc_device_end:
USB_DESC_DEVICE_LEN	equ	usb_desc_device_end - usb_desc_device

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Configuration descriptor
;;;
usb_desc_config:
	;; bLength
	dt	USB_DESC_CONFIG_LEN
	;; bDescriptorType
	dt	0x02			; CONFIGURATION descriptor type
	;; wTotalLength
	dt	USB_DESC_CONFIG_TLEN, 0x00
	;; bNumInterfaces
	dt	0x02			; Keyboard and mouse interfaces
	;; bConfigurationValue
	dt	0x01			; Only configuration value
	;; iConfiguration
	dt	0x00			; No configuration name
	;; bmAttributes
	dt	0x80			; Bus-powered
	;; bMaxPower
	dt	( 100 / 2 )		; 100mA
usb_desc_config_end:
USB_DESC_CONFIG_LEN	equ	usb_desc_config_end - usb_desc_config

;;; Keyboard interface descriptor
;;;
usb_desc_intf_kb:
	;; bLength
	dt	USB_DESC_INTF_KB_LEN
	;; bDescriptorType
	dt	0x04			; INTERFACE descriptor type
	;; bInterfaceNumber
	dt	0x00
	;; bAlternateSetting
	dt	0x00
	;; bNumEndpoints
	dt	0x02
	;; bInterfaceClass
	dt	0x03			; HID class
	;; bInterfaceSubClass
	dt	0x01			; Boot subclass
	;; bInterfaceProtocol
	dt	0x01			; Keyboard protocol
	;; iInterface
	dt	0x00			; No interface name
usb_desc_intf_kb_end:
USB_DESC_INTF_KB_LEN	equ	usb_desc_intf_kb_end - usb_desc_intf_kb

;;; Keyboard HID descriptor
;;;
usb_desc_hid_kb:
	;; bLength
	dt	USB_DESC_HID_KB_LEN
	;; bDescriptorType
	dt	0x21			; HID descriptor
	;; bcdHID
	dt	0x11, 0x01		; HID specification version 1.11
	;; bCountryCode
	dt	0x00
	;; bNumDescriptors
	dt	0x01			; Single report descriptor
	;; bDescriptorType
	dt	0x22			; HID REPORT descriptor type
	;; wDescriptorLength
	dt	USB_DESC_REPORT_KB_LEN, 0x00
usb_desc_hid_kb_end:
USB_DESC_HID_KB_LEN	equ	usb_desc_hid_kb_end - usb_desc_hid_kb

;;; Keyboard interrupt IN endpoint descriptor
;;;
usb_desc_ep_kb_in:
	;; bLength
	dt	USB_DESC_EP_KB_IN_LEN
	;; bDescriptorType
	dt	0x05			; ENDPOINT descriptor type
	;; bEndpointAddress
	dt	0x82			; EP2 IN
	;; bmAttributes
	dt	0x03			; Interrupt endpoint
	;; wMaxPacketSize
	dt	USB_MTU_DATA, 0x00
	;; bInterval
	dt	USB_POLL_IN
usb_desc_ep_kb_in_end:
USB_DESC_EP_KB_IN_LEN	equ	usb_desc_ep_kb_in_end - usb_desc_ep_kb_in

;;; Keyboard interrupt OUT endpoint descriptor
;;;
usb_desc_ep_kb_out:
	;; bLength
	dt	USB_DESC_EP_KB_OUT_LEN
	;; bDescriptorType
	dt	0x05			; ENDPOINT descriptor type
	;; bEndpointAddress
	dt	0x02			; EP2 OUT
	;; bmAttributes
	dt	0x03			; Interrupt endpoint
	;; wMaxPacketSize
	dt	USB_MTU_DATA, 0x00
	;; bInterval
	dt	USB_POLL_OUT
usb_desc_ep_kb_out_end:
USB_DESC_EP_KB_OUT_LEN	equ	usb_desc_ep_kb_out_end - usb_desc_ep_kb_out

;;; Mouse interface descriptor
;;;
usb_desc_intf_ms:
	;; bLength
	dt	USB_DESC_INTF_MS_LEN
	;; bDescriptorType
	dt	0x04			; INTERFACE descriptor type
	;; bInterfaceNumber
	dt	0x01
	;; bAlternateSetting
	dt	0x00
	;; bNumEndpoints
	dt	0x02
	;; bInterfaceClass
	dt	0x03			; HID class
	;; bInterfaceSubClass
	dt	0x00
	;; bInterfaceProtocol
	dt	0x00
	;; iInterface
	dt	0x00			; No interface name
usb_desc_intf_ms_end:
USB_DESC_INTF_MS_LEN	equ	usb_desc_intf_ms_end - usb_desc_intf_ms

;;; Mouse HID descriptor
;;;
usb_desc_hid_ms:
	;; bLength
	dt	USB_DESC_HID_MS_LEN
	;; bDescriptorType
	dt	0x21			; HID descriptor
	;; bcdHID
	dt	0x11, 0x01		; HID specification version 1.11
	;; bCountryCode
	dt	0x00
	;; bNumDescriptors
	dt	0x01			; Single report descriptor
	;; bDescriptorType
	dt	0x22			; HID REPORT descriptor type
	;; wDescriptorLength
	dt	USB_DESC_REPORT_MS_LEN, 0x00
usb_desc_hid_ms_end:
USB_DESC_HID_MS_LEN	equ	usb_desc_hid_ms_end - usb_desc_hid_ms

;;; Mouse interrupt IN endpoint descriptor
;;;
usb_desc_ep_ms_in:
	;; bLength
	dt	USB_DESC_EP_MS_IN_LEN
	;; bDescriptorType
	dt	0x05			; ENDPOINT descriptor type
	;; bEndpointAddress
	dt	0x83			; EP3 IN
	;; bmAttributes
	dt	0x03			; Interrupt endpoint
	;; wMaxPacketSize
	dt	USB_MTU_DATA, 0x00
	;; bInterval
	dt	USB_POLL_IN
usb_desc_ep_ms_in_end:
USB_DESC_EP_MS_IN_LEN	equ	usb_desc_ep_ms_in_end - usb_desc_ep_ms_in

;;; Mouse interrupt OUT endpoint descriptor
;;;
usb_desc_ep_ms_out:
	;; bLength
	dt	USB_DESC_EP_MS_OUT_LEN
	;; bDescriptorType
	dt	0x05			; ENDPOINT descriptor type
	;; bEndpointAddress
	dt	0x03			; EP3 OUT
	;; bmAttributes
	dt	0x03			; Interrupt endpoint
	;; wMaxPacketSize
	dt	USB_MTU_DATA, 0x00
	;; bInterval
	dt	USB_POLL_OUT
usb_desc_ep_ms_out_end:
USB_DESC_EP_MS_OUT_LEN	equ	usb_desc_ep_ms_out_end - usb_desc_ep_ms_out

;;; End of configuration descriptor (including other embedded descriptors)
;;;
usb_desc_config_total_end:
USB_DESC_CONFIG_TLEN	equ	usb_desc_config_total_end - usb_desc_config

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; HID report descriptors
;;;

;;; Keyboard HID report descriptor
;;;
usb_desc_report_kb:
#include "keyboard.xml.inc"
usb_desc_report_kb_end:
USB_DESC_REPORT_KB_LEN	equ	usb_desc_report_kb_end - usb_desc_report_kb

;;; Mouse HID report descriptor
;;;
usb_desc_report_ms:
#include "mouse.xml.inc"
usb_desc_report_ms_end:
USB_DESC_REPORT_MS_LEN	equ	usb_desc_report_ms_end - usb_desc_report_ms

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Supported languages string descriptor
;;;
usb_string_languages:
	;; bLength
	dt	( usb_string_languages_end - usb_string_languages )
	;; bDescriptorType
	dt	0x03			; STRING descriptor type
	;; wLANGID[]
	dt	0x09, 0x04		; English
usb_string_languages_end:

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Vendor name string descriptor
;;;
usb_string_vendor:
	;; bLength
	dt	( usb_string_vendor_end - usb_string_vendor )
	;; bDescriptorType
	dt	0x03			; STRING descriptor type
	;; bString
	dt	'i', 0
	dt	'p', 0
	dt	'x', 0
	dt	'e', 0
	dt	'.', 0
	dt	'o', 0
	dt	'r', 0
	dt	'g', 0
usb_string_vendor_end:

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Product name string descriptor
;;;
usb_string_product:
	;; bLength
	dt	( usb_string_product_end - usb_string_product )
	;; bDescriptorType
	dt	0x03			; STRING descriptor type
	;; bString
	dt	'k', 0
	dt	'e', 0
	dt	'y', 0
	dt	'f', 0
	dt	'l', 0
	dt	'o', 0
	dt	'g', 0
	dt	'g', 0
	dt	'e', 0
	dt	'r', 0
usb_string_product_end:

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Length check
;;;
usb_desc_end:

	if	( high usb_desc_end ) != ( high usb_desc )
	error	"USB descriptors too large"
	endif

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Strings lookup table (not a USB descriptor)
;;;
	;; Align on own size to avoid crossing a page boundary
	org	ALIGN ( 0x04 )
usb_stringtab:
usb_stringtab_languages:
	dt	usb_string_languages
usb_stringtab_vendor:
	dt	usb_string_vendor
usb_stringtab_product:
	dt	usb_string_product
usb_stringtab_end:
USB_STRING_COUNT	equ	usb_stringtab_end - usb_stringtab

	if	( high usb_stringtab_end ) != ( high usb_stringtab )
	error	"USB string table is insufficiently aligned"
	endif

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; End of program
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	end
