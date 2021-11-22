# This file is part of the ParaNut project.
# 
#  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
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



# Main Documentation file
BUILD_HTML_DIR = Documentation/html/
BUILD_HTML_INDEX = $(BUILD_HTML_DIR)index.html

LOG=doxygen.log

INSTALL_DIR = $(PREFIX)/doc/$(SUFFIX)


# Doxygen target (dependent on libparanutsim source files)
.PHONY: doxygen
doxygen: $(BUILD_HTML_INDEX)
$(BUILD_HTML_INDEX): $(PARANUT_SYSTEM_SRC) $(PARANUT_SYSTEM_HDR) 
	doxygen Documentation/Doxyfile > $(LOG) 2>&1

# Documentation install target
.PHONY: install-doc
install-doc : $(BUILD_HTML_INDEX)
	@test "$(PREFIX)" != "" || ( echo "ERROR: Make variable PREFIX must be set for the 'install-doc' target."; exit 3; )
	@echo "### Install libparanutsim documentation to: $(INSTALL_DIR)";
	mkdir -p $(INSTALL_DIR)
	cp -r $(BUILD_HTML_DIR)* $(INSTALL_DIR)

# Documentation clean target (double colons for composition)
.PHONY: clean
clean::
	rm -rf $(LOG) Documentation/html Documentation/latex
