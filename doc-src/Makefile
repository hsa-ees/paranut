#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
# 
#  Copyright (C) 2010-2018 Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without modification, 
#  are permitted provided that the following conditions are met:
# 
#  1. Redistributions of source code must retain the above copyright notice, this 
#     list of conditions and the following disclaimer.
# 
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



################################################################################
#                                                                              #
#     Settings                                                                 #
#                                                                              #
################################################################################

DOCNAME=paranut-manual

# Use the PDF viewer of your choice (e.g. 'evince' on Debian, 'atril' on MINT):
PDFVIEWER=evince
FIG_DPI=150



################################################################################
#                                                                              #
#     Help and Default Target                                                  #
#                                                                              #
################################################################################

.PHONY: build
build: pdf

.PHONY: help
help:
	@echo -e 'Usage: make [command] [<PARAMETER>=<VALUE>]'
	@echo -e 'Commands:'
	@echo -e '\t build \t\t builds the .tex Document defined by DOCNAME'
	@echo -e '\t commit \t executes build and copies the created document to /doc'
	@echo -e '\t install \t installs the documentation contained in /doc to PREFIX'
	@echo -e '\t clean \t\t deletes files created by build'
	@echo -e '\t view \t\t used to view the document defined by DOCNAME with the viewer defined by PDFVIEWER'
	@echo -e '\t pdf \t builds the .tex Document defined by DOCNAME'
	@echo -e '\t %.pdf \t builds the .tex Document defined by the makecall'



################################################################################
#                                                                              #
#     Version file                                                             #
#                                                                              #
################################################################################

VERSION_FILE ?= ../version.env

$(VERSION_FILE): update-version

.PHONY: update-version
update-version:
	$(MAKE) -C ../ update-version



################################################################################
#                                                                              #
#     Installing                                                               #
#                                                                              #
################################################################################

INSTALLATION_FILES = ../doc/paranut-manual.pdf \
         ../doc/2013-master_thesis-michael_seider.pdf \
         ../doc/2020-master_thesis-alexander_bahle.pdf \
         ../doc/2020-master_thesis-alexander_bahle-presentation.pdf \
         ../doc/paper-ew2015.pdf \
         ../doc/paper-ew2020.pdf \
         ../doc/libparanut_manual.pdf \
         ../doc/libparanut_manual_internal.pdf

.PHONY: install
install:
	@echo
	@echo "### Installing documentation to $(PREFIX) ###"
	@if [ $(PREFIX) ]; then \
	  # Install FILES \
	  for FILE in $(INSTALLATION_FILES); do \
	    install -Dp -m 644 -t $(PREFIX)/doc $$FILE; \
	  done; \
	  # Install libparanut documentation: \
	  make -C ../sw/libparanut install-doc SUFFIX=libparanut; \
	  # Install libparanutsim documentation: \
	  make -C ../hw/sysc install-doc SUFFIX=libparanutsim; \
	fi



################################################################################
#                                                                              #
#     Building and Helpers                                                     #
#                                                                              #
################################################################################

.PHONY: pdf
pdf: update-version ${DOCNAME}.pdf 

%.pdf:	%.tex $(VERSION_FILE)
	$(eval GITVERSIONESCAPED= $(shell grep PN_GITVERSION $(VERSION_FILE) | sed 's#^.*=##' | sed 's/_/\\\\_/g'))
	echo "VERSION=$(GITVERSIONESCAPED)"
	pdflatex \\newcommand\\gitversion{$(GITVERSIONESCAPED)}\\input{$<} # Pass 1
	pdflatex \\newcommand\\gitversion{$(GITVERSIONESCAPED)}\\input{$<} # Pass 2
	pdflatex \\newcommand\\gitversion{$(GITVERSIONESCAPED)}\\input{$<} # Pass 3

%.png:	%.svg
ifeq ($(shell inkscape -V 2> /dev/null | cut -c 10),1)
	inkscape $< -D -d $(FIG_DPI) -o $@
else
	inkscape $< -D -d $(FIG_DPI) -e $@
endif

%.tex:	%.def
	./def2tex $< $@

${DOCNAME}.pdf: instructions.tex figs/cc-by-sa.png figs/hsa_logo.png
${DOCNAME}.pdf: figs/paranut-0-4_cores.png
#${DOCNAME}.pdf: figs/paranut-1-linked_mode.png
${DOCNAME}.pdf: figs/paranut-2-all_modes.png
${DOCNAME}.pdf: figs/paranut-3-example-capabilities.png
${DOCNAME}.pdf: figs/pn_logo.png
#${DOCNAME}.pdf: figs/paranut-4-example-1_thread.png
#${DOCNAME}.pdf: figs/paranut-5-example-2_threads.png
#${DOCNAME}.pdf: figs/paranut-6-example-simd_4.png



################################################################################
#                                                                              #
#     Commit                                                                   #
#                                                                              #
###############################################################################

# Create all documentations (paranut-manual, Doxygen libparanutsim, Doxygen liparanut) 
# and commit them to the ../doc directory.  
.PHONY: commit
commit: pdf
	cp ${DOCNAME}.pdf ../doc
	+$(MAKE) -C ../hw/sysc install-doc DOC=$(abspath ../doc)/libparanutsim
	+$(MAKE) -C ../sw/libparanut install-doc DOC=$(abspath ../doc)/libparanut



################################################################################
#                                                                              #
#     Others                                                                   #
#                                                                              #
###############################################################################

.PHONY: view
view: ${DOCNAME}.pdf
	$(PDFVIEWER) ${DOCNAME}.pdf &



################################################################################
#                                                                              #
#     Clean                                                                    #
#                                                                              #
################################################################################

clean:
	rm -f instructions.tex figs/*.png ${DOCNAME}.pdf *.aux *.bbl *.blg *.dvi *.log *.idx *.out *.toc *.synctex.gz
