/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                     Anna Pfuetzner <anna.pfuetzner@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    Test all RISC-V RV32I commands, exceptions and some Extensions (M, A)

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this 
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

#define PN_HALT  .word 0x0000000B 
        # I-Type: 0000 0000 0000 0000 00000 0000 0000 000B          # halt
#define PN_CINVALIDATE(BASE, REG)   .word (0x100B | ((REG) << 15) | (((BASE) & 0xfff) << 20))
        # I-Type: imm[11:0] rs1 001 00000 100B          # cache writeback
#define PN_CWRITEBACK(BASE, REG)  .word (0x200B | ((REG) << 15) | (((BASE) & 0xfff) << 20))
        # I-Type: imm[11:0] rs1 010 00000 100B            # cache invalidate
#define PN_CFLUSH(BASE, REG)       .word (0x300B | ((REG) << 15) | (((BASE) & 0xfff) << 20))
        # I-Type: imm[11:0] rs1 011 00000 100B            # cache flush

.section ".text.init"
.globl _start

_start:						
				# make sure only CePU proceeds 
				la 		x1, ex_handler
				csrrw   x0, mtvec, x1 
				csrr x1, 0x0D4
				beqz x1, loadstore
				
				# Only CoPUs:
				#.word 0x90F090F0 
				PN_HALT

#### TEST Load and Store ####
loadstore:	    la  x1, result
			    li  x2, 0xCAFEBABE
			   	sw  x2, 0(x1)
			   	PN_CFLUSH(0, 1)
			    lw  x3, 0(x1)
			    bne x2, x3, end

			    la  x15, result_end
				li  x14, 0xDEADBEEF
				lw  x13, 0(x15)
				bne x13, x14, end

				# Test LB 
			    li  x2, 0xffffffBE
			    lb  x3, 0(x1)  # x3 = 0xBE
			    li  x4, 0xffffffBA
			    lb  x5, 1(x1)  # x5 = 0xBA
			    li  x6, 0xffffffFE
			    lb  x7, 2(x1)  # x5 = 0xFE
			    li  x8, 0xffffffCA
			    lb  x9, 3(x1)  # x5 = 0xCA
			    bne x2, x3, end 
			    bne x4, x5, end 
			    bne x6, x7, end 
			    bne x8, x9, end 

			    # Test LH 
			    li  x2, 0xffffBABE
			    lh  x3, 0(x1)  # x3 = 0xBABE
			    li  x4, 0xffffCAFE
			    lh  x5, 2(x1)  # x5 = 0xCAFE
			    bne x2, x3, end 
			    bne x4, x5, end 

				# Test LHU
			    li  x2, 0xBABE
			    lhu x3, 0(x1)  # x3 = 0xBABE
			    li  x4, 0xCAFE
			    lhu x5, 2(x1)  # x5 = 0xCAFE
			    bne x2, x3, end 
			    bne x4, x5, end 

			    
			    # Test LBU
			  	li  x2, 0xBE
			    lbu x3, 0(x1)  # x3 = 0xBE
			    li  x4, 0xBA
			    lbu x5, 1(x1)  # x5 = 0xBA
			    li  x6, 0xFE
			    lbu x7, 2(x1)  # x5 = 0xFE
			    li  x8, 0xCA
			    lbu x9, 3(x1)  # x5 = 0xCA
			    bne x2, x3, end 
			    bne x4, x5, end 
			    bne x6, x7, end 
			    bne x8, x9, end 

			    # Test SH
			    la  x1, temp
			    li  x2, 0xCAFE
			    li  x3, 0xBABE
			    li  x5, 0xCAFEBABE
			    sh  x2, 2(x1)
				PN_CFLUSH(0, 1)
			    sh  x3, 0(x1)
				PN_CFLUSH(0, 1)
			    lhu  x4, 2(x1)
			    bne x2, x4, end  
			    lhu  x4, 0(x1)
			    bne x3, x4, end  
			    lw  x4, 0(x1) # x4 = 0xCAFEBABE
			    bne x5, x4, end
			    

			    # Test SB
			    li  x2, 0xA5
			    li  x5, 0xA2A3A4A5
			    sb  x2, 0(x1)
			    lbu  x4, 0(x1)
			    bne x2, x4, end 
			    addi x2, x2, -1
			    sb  x2, 1(x1)
			    lbu  x4, 1(x1)
			    bne x2, x4, end 
			    addi x2, x2, -1
			    sb  x2, 2(x1)
			    lbu  x4, 2(x1)
			    bne x2, x4, end 
			    addi x2, x2, -1
			    sb  x2, 3(x1)
			    lbu  x4, 3(x1)
			    bne x2, x4, end 
			    
			    lw  x3, 0(x1) # x3 = 0xA2A3A4A5
			    bne x3, x5, end 

			    la  x1, load_ok
			    la  x2, store_ok
			    sw  x0, 0(x1)
			    sw  x0, 0(x2)
				PN_CFLUSH(0, 1)
#### Load and Store passed ####



#### ALU immediate instructions  ####
    			.global alu_imm
alu_imm:		nop
			    la    x31, alu_imm_ok   
			    
			    # addi 
			    addi  x1, x0, 0x123
			    addi  x1, x1, -0x223
			    addi  x1, x1, 0x100
			    sb    x1, 0(x31)      # add ok

			    # ori
			    ori   x0, x0, 0x7ff  # must be 0!
			    ori   x1, x1, 0x123
			    ori   x1, x1, 0x321  # x1 = 0x323
			    addi  x2, x1, -0x323 # x1 = 0     
			    sb    x2, 1(x31)      # or ok
			    
			    # and
			    andi  x1, x1, 0x7f0  # x1 = 0x320
			    andi  x1, x1, 0x7ff  # x1 = 0x320
			    addi  x2, x1, -0x320 # x1 = 0     
			    sb    x2, 2(x31)      # and ok
			    
			    # xor
			    xori  x1, x1, 0x755  # x1 = 0x455
			    xori  x1, x1, 0x475  # x1 = 0x0
			    sb    x1, 3(x31)     # or, and, xor ok

			    # check:
			    lw    x1, 0(x31)
				PN_CFLUSH(0, 31)
			    bne   x0, x1, end
#### ALU immediate passed ####


#### ALU register instructions  ####
    			.global alu_reg
alu_reg: 		nop
			    la    x31, alu_reg_ok
			    
			    li    x1, 0xA5A5A5A5
			    li    x2, 0xCAFEBABE
			    li    x3, 0xCAFE0000
			    li    x4, 0x0000BABE
			    li    x5, 0x0000BEEF
			    li    x5, 0x6F5BA5A5
			    
			    # add
			    ori   x10, x0, 0x123
			    ori   x11, x0, 0x246
			    sub   x12, x10, x11
			    add   x12, x12, x10
			    sb    x12, 0(x31)       # add/sub ok
				
			    # or
			    or    x10, x3, x4
			    sub   x11, x10, x2
			    sb    x11, 1(x31)       # or ok
			    
			    # and
			    and   x11, x10, x3      # x11 = 0xCAFE0000
			    sub   x11, x11, x3
			    sb    x11, 2(x31)       # and ok

			    #xor 
			    xor   x11, x10, x4      # x11 = 0xCAFE0000
			    xor   x11, x11, x1      # x11 = 0x6F5BA5A5
			    xor   x11, x11, x5      # x11 = 0
			    sb    x11, 3(x31)       # xor ok

			    # check:
			    lw    x1, 0(x31)
				PN_CFLUSH(0, 31)
			    bne   x0, x1, end
#### ALU register passed ####


#### ALU shift instructions ####
    			.global alu_shift
alu_shift:		nop
			    la    x31, alu_shift_ok
			    
			    # register
			    li    x1, 1
			    li    x2, 10
			    li    x3, 21
			    li    x4, 0x40401010

			    # Test shift by 0
			    sll   x5, x4, x0
			    srl   x6, x4, x0
		 	    sra   x7, x4, x0
			    xor   x8, x5, x6
			    xor   x9, x4, x7
			    or    x10, x8, x9
			    sb    x10, 0(x31)      # shift register 0 ok 
			    #bnez  x10, end
			    
			    sll   x5, x4, x1      # 0x80802020
			    sll   x6, x1, x2      # 0x00000400
			    sll   x7, x1, x3      # 0x00200000
			    srl   x8, x4, x1      # 0x20200808
			    srl   x9, x5, x2      # 0x00202008
			    srl   x10, x5, x3     # 0x00000404
			    sra   x11, x4, x1     # 0x20200808
			    sra   x12, x5, x2     # 0xffe02008
			    sra   x13, x5, x3     # 0xfffffc04

			    xor   x1, x5, x6      # 0x80802420
			    xor   x2, x7, x8      # 0x20000808
			    xor   x3, x9, x10     # 0x0020240c
			    xor   x4, x11, x12    # 0xdfc02800

			    xor   x1, x1, x13     # 0x7f7fd824
			    xor   x2, x2, x3      # 0x20202c04

			    xor   x1, x1, x4      # 0xa0bff024

			    xor   x1, x1, x2      # 0x809fdc20

			    li    x2, 0x809fdc20
			    xor   x1, x1, x2
			    sb    x1, 1(x31)      # shift register ok
				#bnez  x1, end
				
			    # immediate:
			    li    x1, 1
			    li    x2, 10
			    li    x3, 21
			    li    x4, 0x40401010

			    slli  x5, x4, 1       # 0x80802020
			    slli  x6, x1, 10      # 0x00000400
			    slli  x7, x1, 21      # 0x00200000
			    srli  x8, x4, 1       # 0x20200808
			    srli  x9, x5, 10      # 0x00202008
			    srli  x10, x5, 21     # 0x00000404
			    srai  x11, x4, 1      # 0x20200808
			    srai  x12, x5, 10     # 0xffe02008
			    srai  x13, x5, 21     # 0xfffffc04

			    xor   x1, x5, x6      # 0x80802420
			    xor   x2, x7, x8      # 0x20000808
			    xor   x3, x9, x10     # 0x0020240c
			    xor   x4, x11, x12    # 0xdfc02800

			    xor   x1, x1, x13     # 0x7f7fd824
			    xor   x2, x2, x3      # 0x20202c04

			    xor   x1, x1, x4      # 0xa0bff024

			    xor   x1, x1, x2      # 0x809fdc20

			    li    x2, 0x809fdc20
			    xor   x1, x1, x2
			    sh    x1, 2(x31)      # shift immediate ok
			    #bnez   x1, x1

			    # check:
			    lw    x1, 0(x31)
				PN_CFLUSH(0, 31)
			    bne   x0, x1, end
#### ALU shift passed ####


#### ALU compare instructions ####
    			.global alu_comp
alu_comp:		nop
			    la    x31, alu_comp_ok
			  	li    x1, 50
			 	li    x2, -50
			 	li    x7, 123
			 	li    x8, -123
			 	li    x9, 1
			 	li 	  x10, -1
			 	
			   	# slti
			 	slti  x3, x1, 123    # x3 = 50 < 123 = 1
			  	slti  x4, x1, -123	 # x4 = 50 < -123 = 0
			  	slti  x5, x2, 123    # x5 = -50 < 123 = 1
			  	slti  x6, x2, -123   # x6 = -50 < -123 = 0
			  	xor   x3, x3, x5
			  	xor   x4, x4, x6
			  	or    x11, x3, x4    # x11 = slti ok
				sb 	  x11, 0(x31)	 # slti ok

				# sltiu
			 	sltiu x3, x1, -123   # x3 = 50 < unsigned(-123) = 1
			  	sltiu x4, x1, 1	 	 # x4 = 50 < 1 = 0	
			  	sltiu x5, x2, -1     # x5 = unsigned(-50) < unsigned(-1) = 1
			  	sltiu x6, x2, 123    # x6 = unsigned(-50) < 123 = 0
			  	xor   x3, x3, x5
			  	xor   x4, x4, x6
			  	or    x11, x3, x4    # x11 = sltiu ok
				sb 	  x11, 1(x31)	 # sltiu ok
			  	
				# slt
			  	slt   x3, x1, x7     # x3 = 50 < 123 = 1
			  	slt   x4, x1, x8	 # x4 = 50 < -123 = 0
			  	slt   x5, x2, x7     # x5 = -50 < 123 = 1
			  	slt   x6, x2, x8    # x6 = -50 < -123 = 0
			  	xor   x3, x3, x5
			  	xor   x4, x4, x6
			  	or    x11, x3, x4    # x11 = slt ok
				sb 	  x11, 2(x31)	 # slt ok

				# sltu
			 	sltu  x3, x1, x8     # x3 = 50 < unsigned(-123) = 1
			  	sltu  x4, x1, x9	 # x4 = 50 < 1 = 0	
			  	sltu  x5, x2, x10    # x5 = unsigned(-50) < unsigned(-1) = 1
			  	sltu  x6, x2, x7     # x6 = unsigned(-50) < 123 = 0
			  	xor   x3, x3, x5
			  	xor   x4, x4, x6
			  	or    x11, x3, x4    # x11 = sltu ok
				sb 	  x11, 3(x31)	 # sltu ok

			    # check:
			    lw    x1, 0(x31)
				PN_CFLUSH(0, 31)
			    bne   x0, x1, end

#### ALU compare passed ####


#### Control flow instructions ####

                .global jumps
jumps:          nop   
				la 		x31, jump_ok
				nop
				nop
                # JAL (J pseudo op) ...
                or      x1, x0, x0
                j       cont_06
                ori     x1, x1, 1       # must not be executed: x1 = 1      
cont_05:        xori    x2, x2, 1
                j       cont_01
                nop    
cont_06:        ori     x2, x0, 1
                j       cont_05         # jump backwards
cont_01:        or      x1, x1, x2      # x1 := 0

                # JALR (jr pseudo op) ...
                or      x2, x0, 0
                la      x3, cont_02
                jr      x3
                ori     x2, x2, 2       # must not be executed
cont_02:        or      x1, x1, x2 		# x1 := 0

                # JAL ...
                or      x2, x0, x0
                jal     x9, cont_03
ret_01:         ori     x2, x2, 4       # must not be executed: x2 = 4
cont_03:        la      x3, ret_01
                xor     x3, x3, x9      # Link register contains 'ret_01'?
                or      x1, x1, x2
				or      x1, x1, x3

                # JALR ...
                or      x2, x0, x0
                la      x3, cont_04
                jalr    x9, x3, 0
ret_02:         ori     x2, x2, 8       # must not be executed: x2 = 8
cont_04:        la      x3, ret_02
                xor     x3, x3, x9          # Link register contains 'ret_01'?
                or      x1, x1, x2
				or      x1, x1, x3

				sh 		x1, 0(x31)		# jump ok 
				
				li 	  	x2, 0
                li    	x3, 50
 				li    	x4, -50
				li    	x5, 100
 				li    	x6, -100
 				li    	x7, 1
 				li 	  	x8, -1
 				li		x10, 150

                # BEQ, BNE
                beq    	x2, x7, end  	# something is wrong 
cont_07:		xor     x2, x2, x7      
		        bne     x2, x7, loop_01
                beq 	x2, x7, cont_07
                ori   	x2, x2, 2 		# must not be executed

                # BLT, BGE
loop_01:        sub 	x10, x10, x3
				blt 	x8, x10, loop_01 	# for(;x10 > 0; x10 = x10 - 50)
loop_02: 		add 	x10, x10, x7
				bge 	x8, x10, loop_02  	# for(;x10 <= -1; x10++) -> x10 = 0
				or 		x2, x2, x10		
				
				# BLTU, BGEU
loop_03:		add 	x10, x10, x7
				bltu 	x10, x3, loop_03  	# for(;x10 < x3; x10++) -> x10 = 49
loop_04:		sub		x10, x10, x7
				bgeu 	x10, x8, loop_04 	# should not loop 50 >= 0xffffffff
				addi	x10, x10, 1
				xor 	x10, x10, x3
				or 		x2, x2, x10

				sh 		x2, 2(x31) 			# branch ok

				# check
				lw 		x2, 0(x31)
				PN_CFLUSH(0, 31)
				bne 	x0, x2, end
#### Control flow passed ####

#### SYSTEM instructions ###
				.global system
system:			la 		x31, system_ok	

				# Set mtvec to correct address
				la 		x1, ex_handler
				csrrw   x0, mtvec, x1 

				# EXCEPTION
				li 		x1, 0x0
				li 		x2, 0x3
				addi	x1, x1, 1
				.word 0x90F090F0 				# illegal instruction 1
				addi	x1, x1, 2
				xor 	x1, x1, x2
				sb 		x1, 0(x31)

				# CSSRW
				li 		x1, 0x88
				li 		x2, 0xffffffff
				li 		x3, 0x0					# fixed mvendorid value
				csrrw 	x10, mvendorid, x0 		# write RO Register -> should trigger illegal instruction 2, x10 is read anyway
				csrrw 	x0, mstatus, x1 		# write WARL Register with legal value
				csrr 	x11, mstatus 		
				csrrw 	x0, mstatus, x2 		# write WARL Register with not legal value -> should still be legal on read
				csrr 	x12, mstatus
				xor 	x28, x11, x12
				xor 	x29, x10, x3 
				or 		x30, x0, x28
				or 		x30, x30, x29
				
				# CSSRC
				csrrc 	x10, mvendorid, x0 		# clear RO Register with x0 -> should not write and trigger illegal instruction, x10 is read anyway
				csrrc 	x10, mvendorid, x2 		# clear RO Register with x2 -> should trigger illegal instruction 4, x10 is read anyway
				csrrc 	x0, mstatus, x1 		# clear WARL Register with legal value
				csrr 	x11, mstatus 		
				csrrc 	x0, mstatus, x2 		# clear WARL Register with not legal value -> should still be legal on read
				csrr 	x12, mstatus
				xor 	x28, x11, x12
				xor 	x29, x10, x3 
				or 		x30, x30, x28
				or 		x30, x30, x29
				sb 		x30, 1(x31)				# CSRRW/CSRRC ok

				# CSRRWI
				csrrwi 	x10, mvendorid, 0x0 	# write RO Register -> should trigger illegal instruction 3, x10 is read anyway
				csrrwi 	x0, mstatus, 0x8 		# write WARL Register with legal value
				csrr 	x11, mstatus 		
				csrrwi 	x0, mstatus, 0x1f 		# write WARL Register with not legal value -> should still be legal on read
				csrr 	x12, mstatus
				xor 	x28, x11, x12
				xor 	x29, x10, x3 
				or 		x30, x30, x28
				or 		x30, x30, x29
				
				# CSRRCI
				csrrci 	x10, mvendorid, 0x0 	# clear RO Register with 0 -> should not write and trigger illegal instruction, x10 is read anyway
				csrrci 	x10, mvendorid, 0x1f	# clear RO Register with value -> should trigger illegal instruction 5, x10 is read anyway
				csrrci 	x0, mstatus, 0x8 		# clear WARL Register with legal value
				csrr 	x11, mstatus 		
				csrrci 	x0, mstatus, 0x1f 		# clear WARL Register with not legal value -> should still be legal on read
				csrr 	x12, mstatus
				xor 	x28, x11, x12
				xor 	x29, x10, x3 
				or 		x30, x30, x28
				or 		x30, x30, x29
				sb 		x30, 2(x31)				# CSRRWI/CSRRCI ok				

				# CSRRS
				csrrs 	x10, mvendorid, x0 		# set RO Register with x0 -> should not write and trigger illegal instruction, x10 is read anyway
				csrrs 	x10, mvendorid, x2 		# set RO Register with x2 -> should trigger illegal instruction 6, x10 is read anyway
				csrrs 	x0, mstatus, x1 		# set WARL Register with legal value
				csrr 	x11, mstatus 		
				csrrs 	x0, mstatus, x2 		# set WARL Register with not legal value -> should still be legal on read
				csrr 	x12, mstatus
				xor 	x28, x11, x12
				xor 	x29, x10, x3 
				or 		x30, x30, x28
				or 		x30, x30, x29
				
				# CSRRSI
				csrrsi 	x10, mvendorid, 0x0 	# set RO Register with 0 -> should not write and trigger illegal instruction, x10 is read anyway
				csrrsi 	x10, mvendorid, 0x1f	# set RO Register with value -> should trigger illegal instruction 7, x10 is read anyway
				csrrsi 	x0, mstatus, 0x8 		# set WARL Register with legal value
				csrr 	x11, mstatus 		
				csrrsi 	x0, mstatus, 0x1f 		# set WARL Register with not legal value -> should still be legal on read
				csrr 	x12, mstatus
				xor 	x28, x11, x12
				xor 	x29, x10, x3 
				or 		x30, x30, x28
				or 		x30, x30, x29
				sb 		x30, 3(x31)				# CSRRS/CSRRSI ok				


				# check
				li 		x1, 7
				la 		x2, illinst
				lw 		x2, 0(x2)
				bne 	x1, x2, end 		# check for correct number of illegal instructions (7) 
				
				lw 		x3, 0(x31)
				PN_CFLUSH(0, 31)
				bne 	x3, x0, end 		
#### SYSTEM passed ####

#### EXCEPTIONS ####
				.global exceptions
exceptions:		la 		x31, exceptions_ok		
 				
 				# check load address misaligned
 				la 		x30, lam
 				lw		x1, 1(x31)
 				lw		x1, 2(x31)
 				lw		x1, 3(x31)
 				lh 		x1, 1(x31)
 				lh 		x1, 3(x31)

 				lw		x2, 0(x30)
 				xori	x1, x2, 0x5 	# lam should be 5
 				sb		x1, 0(x31)		# store lam ok

 				# check store address misaligned
 				la 		x30, sam
 				li 		x1, 0xcafebabe 	# this should never be written to exeptions_ok
 				sw		x1, 1(x31)
 				sw		x1, 2(x31)
 				sw		x1, 3(x31)
 				sh 		x1, 1(x31)
 				sh 		x1, 3(x31)

 				lw		x2, 0(x30)
 				xori	x1, x2, 0x5 	# sam should be 5
 				sb		x1, 1(x31)		# store sam ok

 				# check instruction address misaligned
 				la 		x30, iam
 				la 		x1, end
 				.word 	0x0020006F  	# jal offset 2
 				jalr 	x0, x1, 2
 				jalr 	x0, x1, 0x1ff
 				.word 	0x00000163 		#beq 	x0, x0, 0x2   #assembler substitutes this
 				.word 	0x00101FE3 		#bneq 	x0, x1, 0x81e  #assembler substitutes this
 				.word 	0x00104FE3		#blt    x0, x1, 0x81e #assembler substitutes this
 				# check no exception for not taken branches
 				.word   0x00100463		#beq 	x0, x1, 2 #assembler substitutes this
												
				lw		x2, 0(x30)
 				xori	x1, x2, 0x6 	# iam should be 6
 				sb		x1, 2(x31)		# store iam ok
 				sb 		x1, 3(x31)		# store again so exeptions_ok gets 0

				# check
 				lw		x3, 0(x31)
				PN_CFLUSH(0, 31)
				bne 	x3, x0, end 
#### EXCEPTIONS passed ####

#### EXTENSIONS #####
				la 		x31, extensions_ok
				# read misa CSR
				csrr 	x2, misa
				
				# M-Extension (Bit 12 of misa)
				li		x3, 0x1000				
				and	 	x3, x2, x3 	
				beqz	x3, no_m
				
				li  	x1, 2
				li  	x2, 5
				li  	x3, 10
				li  	x4, 0x99999999
				li		x5, -3
				li		x6, -1
						
				# 32 bit signed only positive 
				mul 	x7, x1, x2
				mul 	x7, x7, x3
				mul 	x7, x7, x1
				xori 	x7, x7, 200 	# x7 == 200
				bne		x0,	x7, end
				
				mul 	x7, x1, x6		# rs2 negativ
				mul		x7, x7, x3		# rs1 negativ
				mul 	x7, x7, x5		# rs1 and rs2 negative
				xori	x7, x7, 60		# x7 == 60
				bne		x0,	x7, end

				# 64 bit 
				mul 	x7, x1, x4
				mulh 	x7, x1, x4
				

				# 64 bit
				li 		x1, 1
				mulhsu  x7, x6, x1
				
				# div and rem
				li  	x1, 2
				li  	x2, 5
				li  	x3, 10
				li  	x4, 0x99999999
				li		x5, -3
				li		x6, -1

				div 	x7, x3, x2
				div 	x7, x2, x3
				rem 	x7, x3, x2
				rem 	x7, x2, x3
				
				sb		x0, 0(x31)
				j		ext_fin

no_m:			li 		x1, 0xaa
				sb		x1, 0(x31)

ext_fin:		# fill space for further extension checks
				li 		x1, 0xbb
				sb		x1, 1(x31)
				li 		x1, 0xcc
				sb		x1, 2(x31)
				li 		x1, 0xdd
				sb		x1, 3(x31)

				

end:			la   	x1, result
				la 		x2, result_end
				PN_CFLUSH(0, 1);
				PN_HALT
  

# exception handler
				
				#.section .vectors
				.org 0x1000
				.global ex_handler
ex_handler:		
				csrr	x29, mcause			# read mcause
				xori 	x28, x29, 0x0 		# instruction address misaligned?
				beq 	x0, x28, iam_handler 		
				xori 	x28, x29, 0x2 		# illegal instruction?
				beq 	x0, x28, illinst_handler
				xori 	x28, x29, 0x4 		# load address misaligned?
				beq 	x0, x28, lam_handler 		
				xori 	x28, x29, 0x6 		# store address misaligned?
				beq 	x0, x28, sam_handler
				xori 	x28, x29, 0x10 		# copu exception misaligned?
				beq 	x0, x28, copu_handler  		
				

				.global iam_handler	
iam_handler:
				# load, increment, save iam
				la 		x29, iam
				lw 		x28, 0(x29)
				addi	x28, x28, 1
				sw 		x28, 0(x29)
				csrr	x28, mepc			# read mepc
				addi 	x28, x28, 4
				csrw    mepc, x28
				mret

				.global illinst_handler
illinst_handler:
				# load, increment, save illinst
				la 		x29, illinst
				lw 		x28, 0(x29)
				addi	x28, x28, 1
				sw 		x28, 0(x29)
				csrr	x28, mepc			# read mepc				
				addi 	x28, x28, 4
				csrw    mepc, x28
				mret


				.global lam_handler
lam_handler:
				# load, increment, save lam
				la 		x29, lam
				lw 		x28, 0(x29)
				addi	x28, x28, 1
				sw 		x28, 0(x29)
				csrr	x28, mepc			# read mepc
				addi 	x28, x28, 4
				csrw    mepc, x28
				mret

				
				.global sam_handler	
sam_handler:
				# load, increment, save sam
				la 		x29, sam
				lw 		x28, 0(x29)
				addi	x28, x28, 1
				sw 		x28, 0(x29)
				csrr	x28, mepc			# read mepc
				addi 	x28, x28, 4
				csrw    mepc, x28
				mret

				.global copu_handler	
copu_handler:
				# load, increment, save copu
				la 		x29, copu
				lw 		x28, 0(x29)
				addi	x28, x28, 1
				sw 		x28, 0(x29)
				csrr	x29, 0xFC2 			# read mpnx				
				li		x28, 1				# prepare value
loop:			srli	x29, x29, 1
				andi 	x25, x29, 1
				slli 	x28, x28, 1				
				beqz	x25, loop	
				csrrs	x0, 0x7C1, x28		# enable the CoPU (exception is "handled")
				
				csrr  	x28, 0x7C2			# read pnlm
				beqz	x28, return			# if not linked we need to return to mepc else mepc+4
				csrr	x28, mepc			# read mepc
				addi 	x28, x28, 4
				csrw    mepc, x28
return: 		mret

.section .data
.global result
.global load_ok
.global store_ok
.global alu_imm_ok
.global alu_reg_ok
.global alu_shift_ok
.global alu_comp_ok
.global jump_ok
.global system_ok
.global exceptions_ok
.global extensions_ok
.global result_end
.global temp

# Result array...
                        .org 0x0 
result:                 .byte 0xff, 0xff, 0xff, 0xff        # must become 0xcafebabe

load_ok:                .byte 0xff, 0xff, 0xff, 0xff        # must become 0
store_ok:               .byte 0xff, 0xff, 0xff, 0xff        # must become 0
alu_imm_ok:             .byte 0xff, 0xff, 0xff, 0xff        # must become 0
alu_reg_ok:             .byte 0xff, 0xff, 0xff, 0xff        # must become 0
alu_shift_ok:           .byte 0xff, 0xff, 0xff, 0xff        # must become 0
alu_comp_ok:            .byte 0xff, 0xff, 0xff, 0xff        # must become 0
jump_ok:                .byte 0xff, 0xff, 0xff, 0xff        # must become 0
system_ok:              .byte 0xff, 0xff, 0xff, 0xff        # must become 0
exceptions_ok:			.byte 0xff, 0xff, 0xff, 0xff        # must become 0
extensions_ok: 			.byte 0xff, 0xff, 0xff, 0xff		# no extension: must become 0xaabbccdd, extension: must become 0x00bbccdd
result_end:             .word 0xDEADBEEF			        # must remain 0xdeadbeef

illinst:				.byte 0x00, 0x00, 0x00, 0x00        # illegal instruction counter, must become 7
lam:					.byte 0x00, 0x00, 0x00, 0x00        # laod address misaligned counter, must become 5
sam:					.byte 0x00, 0x00, 0x00, 0x00        # store address misaligned counter, must become 5
iam:					.byte 0x00, 0x00, 0x00, 0x00        # instruction address misaligned counter, must become 6
copu:					.byte 0x00, 0x00, 0x00, 0x00 		# copu exception counter, must become CFG_NUT_NUM_CPUS-1

temp:                   .byte 0x00, 0x00, 0x00, 0x00        # temp storage
