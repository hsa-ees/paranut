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

PARANUT_HOME ?= $(abspath $(CURDIR)/..)
include $(PARANUT_HOME)/directory-base.mk
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



.PHONY: help
help:
	@echo -e 'Usage: make [command] [<PARAMETER>=<VALUE>]'
	@echo -e 'Commands:'
	@echo -e '\t build \t\t builds the .tex Document defined by DOCNAME'
	@echo -e '\t build-pdf \t builds paranut-manual.pdf in doc-src'
	@echo -e '\t install \t installs the documentation contained in /doc to INSTALL_DIR'
	@echo -e '\t clean \t\t deletes files created by build'
	@echo -e '\t view \t\t used to view the document defined by DOCNAME with the viewer defined by PDFVIEWER'
	@echo -e '\t pdf \t builds the .tex Document defined by DOCNAME'
	@echo -e '\t %.pdf \t builds the .tex Document defined by the makecall'



################################################################################
#                                                                              #
#     Version file                                                             #
#                                                                              #
################################################################################
$(VERSION_ENV): 
	$(MAKE) -C $(PARANUT_HOME) update-version



################################################################################
#                                                                              #
#     Installing                                                               #
#                                                                              #
################################################################################

INSTALLATION_FILES = ../doc/paranut-manual.pdf \
         ../doc/technical_reports/2013-master_thesis-michael_seider.pdf \
         ../doc/technical_reports/2020-master_thesis-alexander_bahle.pdf \
         ../doc/technical_reports/2020-master_thesis-alexander_bahle-presentation.pdf \
         ../doc/papers/paper-ew2015.pdf \
         ../doc/papers/paper-ew2020.pdf \
         ../doc/libparanut_manual.pdf \
         ../doc/libparanut_manual_internal.pdf \
		 

.PHONY: install
install:
	@echo
	@echo "### Installing documentation to $(INSTALL_DIR) ###"
	@if [ $(INSTALL_DIR) ]; then \
	  # Install FILES \
	  for FILE in $(INSTALLATION_FILES); do \
	    install -Dp -m 644 -t $(INSTALL_DIR)/doc $$FILE; \
	  done; \
	fi
	@cp -r ../doc/libparanut $(INSTALL_DIR)/doc/libparanut 
	@cp -r ../doc/libparanutsim $(INSTALL_DIR)/doc/libparanutsim


################################################################################
#                                                                              #
#     Building and Helpers                                                     #
#                                                                              #
################################################################################

.PHONY: build-pdf
build-pdf: $(VERSION_ENV) ${DOCNAME}.pdf 

%.pdf:	%.tex $(VERSION_ENV)
	$(eval GITVERSIONESCAPED= $(shell grep PN_GITVERSION $(VERSION_ENV) | sed 's#^.*=##' | sed 's/_/\\\\_/g'))
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
.PHONY: build
build: build-pdf
	cp ${DOCNAME}.pdf ../doc
	+$(MAKE) -C $(HAL_DIR)/libparanut install-doc INSTALL_DIR=$(abspath ../doc)/libparanut



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
	@echo
	@echo "### All doc directories cleaned ###"
	@rm -f instructions.tex figs/*.png ${DOCNAME}.pdf *.aux *.bbl *.blg *.dvi *.log *.idx *.out *.toc *.synctex.gz
