# This file is part of the ParaNut project.

#   Copyright (C) 2022 Marco Milenkovic <marco.milenkovic@hs-augsburg.de>
#			      2023 Elias Schuler <elias.schuler@hs-augsburg.de>
#					   Lukas Bauer <lukas.bauer@hs-aubsburg.de>
#       Hochschule Augsburg, University of Applied Sciences

#   Redistribution and use in source and binary forms, with or without modification,
#   are permitted provided that the following conditions are met:

#   1. Redistributions of source code must retain the above copyright notice, this
#      list of conditions and the following disclaimer.

#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation and/or
#      other materials provided with the distribution.

#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.18)

# Design template
### --- Hier den Projektnamen festlegen ---
project(uart_test)
find_package(SVC REQUIRED)

set(PROC_TECH fpga)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fcommon")
enable_testing()

### --- Wenn du dein Design debuggen musst ---
add_compile_options(-Wall -ggdb -O0)
# Add alle the nessesery files here
### --- Syntax ist hier: Name executable und dann ALLE cpp files die im Projekt referenziert oder verwendet werden. Reihenfolge egal
add_executable(UARTModule wb8_uart_16750.cpp uart_transmitter.cpp ../base.cpp uart_receiver.cpp uart_interrupt.cpp uart_baudgen.cpp slib_mv_filter.cpp slib_input_sync.cpp slib_input_filter.cpp slib_input_filter_2.cpp slib_fifo.cpp slib_fifo_11.cpp slib_edge_detect.cpp slib_counter.cpp slib_clock_div.cpp slib_clock_div_1.cpp ${CMAKE_CURRENT_LIST_DIR}/../tb/uart/wb8_uart_16750/wb8_uart_16750_tb.cpp)
### --- Das muss der Pfad zu den header files sein. Am besten alles in einen Ordner packen, damit hatte ich dann keine Probleme
target_include_directories(UARTModule PUBLIC "${PROJECT_SOURCE_DIR}")
target_include_directories(UARTModule PUBLIC "${PROJECT_SOURCE_DIR}/..")

# use cmake -DSYN=ON to enable the synthese flag or OFF to disable it
OPTION(SYN "set the __SYNTHESIS__ Flag" ON) # Enabled by default
IF(SYN)
	target_compile_definitions(UARTModule PUBLIC -D__SYNTHESIS__)
ENDIF(SYN)

### --- Diese Zeile ruft das ICSC Tool auf und linkt die gepachte SystemC Bibliothek
svc_target(UARTModule INIT_LOCAL_VARS ELAB_TOP tb.wb8_uart_16750)
### ---                                         ^ das muss das TOP Modul in der hierachie sein


