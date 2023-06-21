# This file is part of the ParaNut project.
# 
#  Copyright (C) 2021-2022 Felix Wagner <felix.wagner1@hs-augsburg.de>
#     Hochschule Augsburg, University of Applied Sciences
#
# Redistribution and use in source and binary forms, with or without modification, 
# are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this 
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation and/or
#    other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# Date: 30.01.2022

# Documentation install path
DOC ?= $(INSTALL_DIR)/doc/$(SUFFIX)
DOC_INDEX_HTML = $(DOC)/index.html

# Main Documentation file
INDEX_HTML = doc-src/html/index.html

# Doxygen target (dependent on libparanut source files)
.PHONY: doxygen
doxygen: $(INDEX_HTML)
$(INDEX_HTML): $(patsubst %.o, %.[cS], $(OBJECT_LIST)) pn_pthread.h pn_pthread_types.h
	doxygen doc-src/Doxyfile

# Documentation install target
.PHONY: install-doc
install-doc: $(DOC)
$(DOC): $(DOC_INDEX_HTML)
$(DOC_INDEX_HTML): $(INDEX_HTML)
	@echo "### Install ParaNut Pthread Documentation to: $(INSTALL_DIR)";
	mkdir -p $(DOC)
	cp -r doc-src/html/* $(DOC)
	rm -rf doc-src/html
	rm -rf doc-src/latex

