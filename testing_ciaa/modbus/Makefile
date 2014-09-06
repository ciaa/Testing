###############################################################################
#
# Copyright 2014, Fernando Beunza
#
# This file is part of CIAA Firmware.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

###############################################################################
# Project name
#
PROJECT := modbus

###############################################################################
# Definitions
#
INCDIR := -I"inc/"
SRCDIR := src/
BINDIR := bin/
LIBDIR := 
CC := gcc
AS := as
CFLAGS := -g -Wall -c -fPIC $(INCDIR)
LIBS = -lpthread
OBJECTS := $(patsubst %.c,%.o,$(wildcard $(SRCDIR)*.c))

###############################################################################
# make all: compile all
#
all: $(PROJECT).so

###############################################################################
# Generate library
#
$(PROJECT).so : $(OBJECTS)
	$(CC) -shared -Wl,-soname,lib$(PROJECT).so $(LIBS) -o $(BINDIR)lib$(PROJECT).so $(OBJECTS)

###############################################################################
# Generate object files
#
%.o : %.c
	$(CC) $(CFLAGS) $< -o $@

# Procesamiento de los archivos .asm
%.rel : %.asm
	$(AS) $<

###############################################################################
# make clean: delete binaries
#
clean:
	@rm -fv $(SRCDIR)*.o
	@rm -fv $(BINDIR)lib$(PROJECT).so

###############################################################################
# make rebuild: clean & all
#
rebuild: clean all

