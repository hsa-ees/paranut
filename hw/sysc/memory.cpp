/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

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


#include "memory.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <elf.h>

#include "base.h"

#define BUSRT_ADR_OFF (CFG_MEMU_BUSIF_WIDTH/32 + 2)
#define BURST_ADR_ADD (CFG_MEMU_BUSIF_WIDTH/8)

class CMemory *mainMemory = NULL;


// ********************* CLabel ***************************

CLabel::CLabel (TWord _adr, const char *_name) {
    adr = _adr;
    strncpy (name, _name, LABEL_LEN);
    name[LABEL_LEN] = '\0';
}


// ********************* CMemory **************************

CMemory::~CMemory() {
    delete data_;
}

void CMemory::Init (TWord base, TWord size) {
    base_ = base;
    size_ = size;
    data_ = new TByte[size];
    memset (data_, 0, size);

    label_list_.clear();

    sig_found_ = false;

    if (!mainMemory) mainMemory = this;
}


// ***** ELF file reading *****

#define PRINTF fprintf
#define PRIx32 "x"


/* Helpers to deal with arbitrary endianess in the fields of ELF files.
 *
 * NOTE: This code assumes that it is running on a little endian machine (e.g. x86/amd64).
 */


static uint16_t ElfGetHalf (unsigned char eiData, uint16_t fieldVal) {
  switch (eiData) {
    case ELFDATA2LSB:
      return fieldVal;
    case ELFDATA2MSB:
      return (fieldVal >> 8) | (fieldVal << 8);
    default:
      PN_ASSERT (false);
      return 0;
  }
}


static uint32_t ElfGetWord (unsigned char eiData, uint32_t fieldVal) {
  switch (eiData) {
    case ELFDATA2LSB:
      return fieldVal;
    case ELFDATA2MSB:
      return (fieldVal >> 24) |
             ((fieldVal >> 8) & 0x0000ff00) |
             ((fieldVal << 8) & 0x00ff0000) |
             (fieldVal << 24);
    default:
      PN_ASSERT (false);
      return 0;
  }
}


bool CMemory::ReadFile (const char *filename, const bool dumpVHDL) {
    FILE *inputfs;
    Elf32_Ehdr elfhdr;
    unsigned char eiData;
    Elf32_Phdr *elf_phdata = NULL;
    Elf32_Shdr *elf_spnt, *elf_shdata;
    Elf32_Sym *sym_tbl = (Elf32_Sym *)0;
    unsigned syms = 0;
    char *str_tbl = (char *)0;
    char *s_str = (char *)0;
    unsigned inputbuf;
    unsigned padd;
    unsigned insn;
    int i, j, sectsize, len;
    TWord adr;

    // Reset :
    tdata_adr = 0;
    tohost_adr = 0;

    // PN_INFO  ("### sizeof (elfhdr) = %i", sizeof (elfhdr));

    if (!label_list_.empty ()) {
        label_list_.clear () ;
    }

    if (!(inputfs = fopen (filename, "r"))) return false;

    if (fread (&elfhdr, sizeof (elfhdr), 1, inputfs) != 1) return false;
    eiData = elfhdr.e_ident[EI_DATA];   // get endianess

    if ((elf_shdata = (Elf32_Shdr *)malloc (ElfGetHalf (eiData, elfhdr.e_shentsize) *
                                                   ElfGetHalf (eiData, elfhdr.e_shnum))) == NULL)
        return false;

    if (fseek (inputfs, ElfGetWord (eiData, elfhdr.e_shoff), SEEK_SET) != 0) return false;

    // PN_INFOF (("# elfhdr.e_shentsize = %i, elfhdr.e_shnum = %i\n", elfhdr.e_shentsize, elfhdr.e_shnum));
    if (fread (elf_shdata, ElfGetHalf (eiData, elfhdr.e_shentsize) * ElfGetHalf (eiData, elfhdr.e_shnum), 1, inputfs) != 1)
        return false;

    if (ElfGetWord (eiData, elfhdr.e_phoff)) {
        if ((elf_phdata = (Elf32_Phdr *)malloc (ElfGetHalf (eiData, elfhdr.e_phnum) *
                                                       ElfGetHalf (eiData, elfhdr.e_phentsize))) == NULL)
            return false;

        if (fseek (inputfs, ElfGetWord (eiData, elfhdr.e_phoff), SEEK_SET) != 0) return false;

        if (fread (elf_phdata, ElfGetHalf (eiData, elfhdr.e_phnum) * ElfGetHalf (eiData, elfhdr.e_phentsize), 1, inputfs) != 1)
            return false;
    }

    for (i = 0, elf_spnt = elf_shdata; i < ElfGetHalf (eiData, elfhdr.e_shnum); i++, elf_spnt++) {
        if (ElfGetWord (eiData, elf_spnt->sh_type) == SHT_STRTAB) {
            if (NULL != str_tbl)
                continue; // Continue with first string table (second one is section header string table)

            if ((str_tbl = (char *)malloc (ElfGetWord (eiData, elf_spnt->sh_size))) == NULL) return false;

            if (fseek (inputfs, ElfGetWord (eiData, elf_spnt->sh_offset), SEEK_SET) != 0) return false;

            if (fread (str_tbl, ElfGetWord (eiData, elf_spnt->sh_size), 1, inputfs) != 1) return false;
        } else if (ElfGetWord (eiData, elf_spnt->sh_type) == SHT_SYMTAB) {
            if (NULL != sym_tbl) free (sym_tbl);

            if ((sym_tbl = (Elf32_Sym *)malloc (ElfGetWord (eiData, elf_spnt->sh_size))) == NULL)
                return false;

            if (fseek (inputfs, ElfGetWord (eiData, elf_spnt->sh_offset), SEEK_SET) != 0) return false;

            if (fread (sym_tbl, ElfGetWord (eiData, elf_spnt->sh_size), 1, inputfs) != 1) return false;

            syms = ElfGetWord (eiData, elf_spnt->sh_size) / ElfGetWord (eiData, elf_spnt->sh_entsize);
        }
    }

    if (ElfGetHalf (eiData, elfhdr.e_shstrndx) != SHN_UNDEF) {
        elf_spnt = &elf_shdata[ElfGetHalf (eiData, elfhdr.e_shstrndx)];

        if ((s_str = (char *)malloc (ElfGetWord (eiData, elf_spnt->sh_size))) == NULL) return false;

        if (fseek (inputfs, ElfGetWord (eiData, elf_spnt->sh_offset), SEEK_SET) != 0) return false;

        if (fread (s_str, ElfGetWord (eiData, elf_spnt->sh_size), 1, inputfs) != 1) return false;
    }

    for (i = 0, elf_spnt = elf_shdata; i < ElfGetHalf (eiData, elfhdr.e_shnum); i++, elf_spnt++) {
        if ((ElfGetWord (eiData, elf_spnt->sh_type) & SHT_PROGBITS) && (ElfGetWord (eiData, elf_spnt->sh_flags) & SHF_ALLOC)) {

            padd = ElfGetWord (eiData, elf_spnt->sh_addr);
            for (j = 0; j < ElfGetHalf (eiData, elfhdr.e_phnum); j++) {
                if (ElfGetWord (eiData, elf_phdata[j].p_offset) &&
                    ElfGetWord (eiData, elf_phdata[j].p_offset) <= ElfGetWord (eiData, elf_spnt->sh_offset) &&
                    (ElfGetWord (eiData, elf_phdata[j].p_offset) + ElfGetWord (eiData, elf_phdata[j].p_memsz)) >
                    ElfGetWord (eiData, elf_spnt->sh_offset))
                    padd = ElfGetWord (eiData, elf_phdata[j].p_paddr) + ElfGetWord (eiData, elf_spnt->sh_offset) -
                           ElfGetWord (eiData, elf_phdata[j].p_offset);
            }

            char noname[] = "noname";
            char *snamep = noname;
            if (ElfGetWord (eiData, elf_spnt->sh_name) && s_str)
                snamep = &s_str[ElfGetWord (eiData, elf_spnt->sh_name)];
            PN_INFOF (("Section: %-15s  vaddr: 0x%.8lx, paddr: 0x%.8lx, offset: 0x%.8lx, size: 0x%.8lx", snamep, ElfGetWord (eiData, elf_spnt->sh_addr),
                                padd, ElfGetWord (eiData, elf_spnt->sh_offset),  ElfGetWord (eiData, elf_spnt->sh_size)));

            adr = padd;
            sectsize = ElfGetWord (eiData, elf_spnt->sh_size);

            if (fseek (inputfs, ElfGetWord (eiData, elf_spnt->sh_offset), SEEK_SET) != 0) {
                free (elf_phdata);
                return false;
            }

            while (sectsize > 0 && (len = fread (&inputbuf, sizeof (inputbuf), 1, inputfs))) {
                insn = ElfGetWord (eiData, inputbuf);
                WriteWord (adr, insn); // addprogram (freemem, insn, &breakpoint);
                // PN_INFOF (("### Write (%08x, %08x)", freemem, insn));
                adr += 4;
                sectsize -= 4;
            }
        }
    }

    //  labelList = new CLabel [syms];
    //  labels = 0;

    if (str_tbl) {
        i = 0;
        while (syms--) {
            if (ElfGetWord (eiData, sym_tbl[i].st_name) && sym_tbl[i].st_info &&
                ElfGetHalf (eiData, sym_tbl[i].st_shndx) < 0x8000) {
                std::string label_name = std::string (&str_tbl[ElfGetWord (eiData, sym_tbl[i].st_name)]);
                label_list_.push_back(CLabel(ElfGetWord (eiData, sym_tbl[i].st_value), label_name.c_str()));
                // PN_INFOF (("%s", label_name.c_str()));
                // Check for _thread_data symbol (indicates use of libc)
                if (label_name.compare ("_thread_data") == 0) {
                    PN_INFOF (("Symbols: _thread_data found (0x%08x)", ElfGetWord (eiData, sym_tbl[i].st_value)));
                    tdata_adr = ElfGetWord (eiData, sym_tbl[i].st_value);
                    /* Write 'S' to _thread_data to indicate simulation to the setup_cache function
                     *  It is not necessary to invalidate the cache during simulation since the
                     *  MTagRam in the MEMU will be clean on a new startup and reseted if the
                     *  reset signal is asserted
                     */
                    WriteByte (tdata_adr, (TByte)'S');
                }
                // Check for tohost/fromhost symbols in file (used for UART communication)
                if (label_name.compare ("tohost") == 0) {
                    PN_INFOF (("Symbols: tohost found (0x%08x)", ElfGetWord (eiData, sym_tbl[i].st_value)));
                    tohost_adr = ElfGetWord (eiData, sym_tbl[i].st_value);
                }
                // Check for signature symbols in file (used in RISC-V compliance tests)
                if (label_name.compare ("begin_signature") == 0) {
                    PN_INFOF (("Symbols: Compliance signature start found (0x%08x)",
                            ElfGetWord (eiData, sym_tbl[i].st_value)));
                    sig_found_ = true;
                    sig_adr_ = ElfGetWord (eiData, sym_tbl[i].st_value);
                }
                if (label_name.compare ("end_signature") == 0) {
                    PN_INFOF (("Symbols: Compliance signature end found (0x%08x)",
                            ElfGetWord (eiData, sym_tbl[i].st_value)));
                    sig_adr_end_ = ElfGetWord (eiData, sym_tbl[i].st_value);
                }
            }
            i++;
        }
    }

    if (dumpVHDL) DumpVHDL (filename, adr - base_);

    if (NULL != str_tbl) free (str_tbl);
    if (NULL != sym_tbl) free (sym_tbl);
    free (s_str);
    free (elf_phdata);
    free (elf_shdata);

    return true;
}


void CMemory::DumpVHDL (const char *filename, unsigned size) {
    TWord adr;
    FILE *outputfs;

    std::string fname = std::string (filename);
    std::string ofname (fname, fname.find_last_of ('/') + 1);
    if (!(outputfs = fopen ((ofname + "_mem_dump.vhd").c_str (), "w"))) {
        PN_ERRORF (("Unable to open file '%s_mem_dump.vhd' for writing.", ofname.c_str ()));
        return;
    }

    size += 4;
    PRINTF (outputfs, "library ieee;\n");
    PRINTF (outputfs, "use ieee.std_logic_1164.all;\n\n");
    PRINTF (outputfs, "library paranut;\n");
    PRINTF (outputfs, "use paranut.types.all;\n\n");
    PRINTF (outputfs, "package prog_mem is\n\n");
    PRINTF (outputfs, "\tconstant PROG_SIZE : integer := %u;\n\n", size);
    PRINTF (outputfs, "\tconstant PROG_DATA : mem_type(0 to PROG_SIZE/4-1) := (\n");
    for (adr = base_; adr < base_ + size; adr += 4) {
        PRINTF (outputfs, "\t\t%s\n", GetDumpStrVHDL (adr));
    }
    PRINTF (outputfs, "\t\tothers => X\"00000000\"\n");
    PRINTF (outputfs, "\t);\n\n");
    PRINTF (outputfs, "end package;");

    fclose (outputfs);
    PN_INFOF (("Dumped program memory content to '%s_mem_dump.vhd'", ofname.c_str ()));
}


// ***** Labels *****

int CMemory::FindLabel(TWord adr) {
    for (int i = 0; i < label_list_.size(); i++)
        if (label_list_.at(i).adr == adr) return i;
    return -1;
}


// ***** Dump *****

static char Printable (TByte byte) { return isprint (byte) ? byte : '.'; }


char *CMemory::GetDumpStr (TWord adr) {
    static char ret[200];

    int label;
    TByte *bytes;
    if (adr - base_ >= 0 && adr - base_ < size_) {
        bytes = &data_[adr - base_];
        label = FindLabel (adr);
        sprintf (ret, "%-15s %08x: %02x %02x %02x %02x   %c%c%c%c   %s", label != -1 ? label_list_.at(label).name : "", adr,
                 bytes[0], bytes[1], bytes[2], bytes[3], Printable (bytes[0]), Printable (bytes[1]),
                 Printable (bytes[2]), Printable (bytes[3]), pn_DisAss (ReadWord (adr)));
    } else
        sprintf (ret, "Address outside of memory address space: 0x%08x", adr);

    return ret;
}

char *CMemory::GetDumpStrVHDL (TWord adr) {
    static char ret[200];

    TByte *bytes;
    bytes = &data_[adr - base_];
    sprintf (ret, "16#%04x# => X\"%02x%02x%02x%02x\",", (adr - base_) / 4, bytes[0], bytes[1],
             bytes[2], bytes[3]);
    return ret;
}

void CMemory::Dump (TWord adr0, TWord adr1) {
    TWord val, lastVal, adr;
    bool printPeriod;

    lastVal = 0;
    printPeriod = false;
    for (adr = adr0 < base_ ? base_ + adr0 : adr0;
         adr < (adr1 < base_ ? MIN (base_ + size_, base_ + adr1) : MIN (base_ + size_, adr1)); adr += 4) {
        val = ReadWord (adr);
        if (val != lastVal /*/ val != 0 */) {
            if (printPeriod) PRINTF (stderr, "%21s...\n", "");
            PRINTF (stderr, "%s\n", GetDumpStr (adr));
            printPeriod = false;
        } else
            printPeriod = true;
        lastVal = val;
    }
    if (printPeriod) PRINTF (stderr, "%21s...\n", "");
}

void CMemory::DumpSignature (const char *filename) {
    FILE *outputfs;

    if (sig_found_) {
        std::string fname = std::string (filename);
        std::string ofname (fname, 0, fname.find_last_of ('.'));
        if (!(outputfs = fopen ((ofname + ".signature.output").c_str (), "w"))) {
            PN_ERRORF (("Unable to open file '%s.signature.output' for writing.\n", ofname.c_str ()));
            return;
        }

        // Write signature
        for (TWord adr = sig_adr_; adr < sig_adr_end_; adr += 4) {
            PRINTF (outputfs, "%08x\n", *(TWord *)&data_[adr - base_]);
        }

        fclose (outputfs);
        PN_INFOF (("Dumped signature memory content to '%s.signature.output'", ofname.c_str ()));
    } else {
        PN_WARNING ("No Signature section found! Dumping is cancelled!");
    }
}

// **************** MWBMemory ********************

void MWBMemory::SetDelays (uint rd_setup, uint rd_delay, uint wr_setup, uint wr_delay) {
    // Set setup and delay times in cycles
    this->rd_setup = rd_setup;
    this->rd_delay = rd_delay;
    this->wr_setup = wr_setup;
    this->wr_delay = wr_delay;
}

TDWord MWBMemory::ReadMemory(TWord adr, TWord sel)
{
    TDWord val = 0;

    if (!IsAdressed (adr))
      PN_ERRORF (("Memory read address bound check failure: 0x%08x", adr));

    if (sel == 0xff)
        val = ReadDWord (adr);
    else if (sel == 0xf0)
        val = (TDWord)ReadWord (adr+4) << 32;
    else if (sel == 0xf)
        val = ReadWord (adr);
    else
        for (int n = 0; n < CFG_MEMU_BUSIF_WIDTH/8; n++)
#if PN_BIG_ENDIAN == 1
            if (sel & (1 << n)) val |= (TDWord)ReadByte (adr + n) << (24 - 8 * n);
#else
            if (sel & (1 << n)) val |= (TDWord)ReadByte (adr + n) << (8 * n);
#endif
    return val;
}

void MWBMemory::WriteMemory(TWord adr, TWord sel, TDWord val)
{
    if (!IsAdressed (adr))
      PN_ERRORF (("Memory write address bound check failure: 0x%08x", adr));

    if (sel == 0xff)
        WriteDWord (adr, val);
    else if (sel == 0xf)
        WriteWord (adr, (TWord)val);
    else
        for (int n = 0; n < CFG_MEMU_BUSIF_WIDTH/8; n++)
#if PN_BIG_ENDIAN == 1
            if (sel & (1 << n)) WriteByte (adr + n, (val >> (24 - 8 * n)) & 0xff);
#else
            if (sel & (1 << n)) WriteByte (adr + n, (val >> (8 * n)) & 0xff);
#endif
}

void MWBMemory::MainThread () {
    // TBD: Read and write delay are currently ingored!
    TWord sel;
    sc_uint<32> adr;
#if CFG_MEMU_BUSIF_WIDTH == 64
    TDWord val;
#else
    TWord val;
#endif

    // Reset...
    wb_dat_o = 0;
    wb_err_o = 0;
    wb_rty_o = 0;
    wb_ack_o = 0;
    wait ();

    while (true) {
        wb_ack_o = 0;
        wait ();
        if (wb_stb_i == 1 && wb_cyc_i == 1) {
            if (wb_we_i == 1) {
                // Write transfer
                // Write setup delay...
                if (wr_setup > 1) wait (wr_setup-1);

                // Read inputs...
                adr = wb_adr_i.read ();
                val = wb_dat_i.read ();
                sel = wb_sel_i.read ();

                // Check cti_i for type of access..
                if (wb_cti_i.read () == 0b000) {
//                    PN_INFOF (("MEMORY: Write (%08x, %016llx) [%x]", adr.value (), val, sel));
                    WriteMemory (adr, sel, val);
                    wb_ack_o = 1;
                    wait ();
                } else if (wb_cti_i.read () == 0b010) {
                    // Incrementing burst cycle...

                    // Send acknowledge before writing first data
                    wb_ack_o = 1;
                    // Delay next result
//                    if (wr_delay > 0) {
//                        wait (); // Acknowledge first
//                        ack_o = 0;
//                        for (int n = 0; n < (int)(wr_delay-2); ++n)
//                            wait ();
//                    }

                    while (wb_cti_i.read () != 0b111) { // Wait for End-of-Burst signal
                        wait ();

                        // Read inputs for next write cycle...
                        val = wb_dat_i.read ();
                        sel = wb_sel_i.read ();

//                        PN_INFOF (("MEMORY: Write (%08x, %016llx) [%x]", adr.value (), val, sel));
                        WriteMemory (adr, sel, val);

                        // Set outputs...
                        wb_ack_o = 1;

                        // Calculate next address based on bte_i...
                        if (wb_bte_i.read () == 0b01)
                            adr = (adr(31, 1+BUSRT_ADR_OFF), sc_uint<1+BUSRT_ADR_OFF>(adr(BUSRT_ADR_OFF, 0) + BURST_ADR_ADD));
                        else if (wb_bte_i.read () == 0b10)
                            adr = (adr(31, 2+BUSRT_ADR_OFF), sc_uint<2+BUSRT_ADR_OFF>(adr(1+BUSRT_ADR_OFF, 0) + BURST_ADR_ADD));
                        else if (wb_bte_i.read () == 0b11)
                            adr = (adr(31, 3+BUSRT_ADR_OFF), sc_uint<3+BUSRT_ADR_OFF>(adr(2+BUSRT_ADR_OFF, 0) + BURST_ADR_ADD));
                        else
                            adr += BURST_ADR_ADD;

                        // Delay next result
//                        if (wr_delay > 0) {
//                            wait (); // Acknowledge first
//                            ack_o = 0;
//                            for (int n = 0; n < wr_delay-1 && cti_i.read () != 0b111; ++n) // No need to wait outside of burst
//                                wait ();
//                        }
                    }
                }
            } else {
                // Read transfer
                // Read setup delay...
                if (rd_setup > 1) wait (rd_setup-1);

                // Read inputs...
                adr = wb_adr_i.read ();
                sel = wb_sel_i.read ();

                // Check cti_i for type of access..
                if (wb_cti_i.read () == 0b000) {
                    // Classic cylce...
                    val = ReadMemory (adr, sel);
//                    PN_INFOF (("MEMORY: Read (%08x, %016llx) [%x]", adr, val, sel));

                    // Set outputs...
                    wb_dat_o = val;
                    wb_ack_o = 1;

                    wait ();
                } else if (wb_cti_i.read () == 0b010) {
                    // Incrementing burst cycle...
                    while (wb_cti_i.read () != 0b111) { // Wait for End-of-Burst signal
//                        PN_INFOF (("MEMORY: Read (%08x, %016llx) [%x]", adr.value (), val, sel));
                        val = ReadMemory (adr, sel);

                        // Set outputs...
                        wb_dat_o = val;
                        wb_ack_o = 1;

                        // Read inputs for next read...
                        sel = wb_sel_i.read ();
                        // Calculate next address based on bte_i...
                        if (wb_bte_i.read () == 0b01)
                            adr = (adr(31, 1+BUSRT_ADR_OFF), sc_uint<1+BUSRT_ADR_OFF>(adr(BUSRT_ADR_OFF, 0) + BURST_ADR_ADD));
                        else if (wb_bte_i.read () == 0b10)
                            adr = (adr(31, 2+BUSRT_ADR_OFF), sc_uint<2+BUSRT_ADR_OFF>(adr(1+BUSRT_ADR_OFF, 0) + BURST_ADR_ADD));
                        else if (wb_bte_i.read () == 0b11)
                            adr = (adr(31, 3+BUSRT_ADR_OFF), sc_uint<3+BUSRT_ADR_OFF>(adr(2+BUSRT_ADR_OFF, 0) + BURST_ADR_ADD));
                        else
                            adr += BURST_ADR_ADD;

                        wait ();

                        // Delay next result
//                        ack_o = 0;
//                        if (rd_delay > 0) {
//                            for (int n = 0; n < rd_delay && cti_i.read () != 0b111; ++n) // No need to wait outside of burst
//                                wait ();
//                        }
                    } // while (cti_i.read () != 0b111)
                }
            }
        } // if (stb_i == 1 && cyc_i == 1) {
    } // while (true)
}

