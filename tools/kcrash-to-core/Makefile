# Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
# Use of this source code is governed by the GNU General Public License v2.

PROG = kcrash-to-core
CFLAGS ?= -g -O0
CFLAGS += -Wall -Werror
LDLIBS=-lfl

$(PROG): fixed.lex.yy.c Makefile
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $< -o $@ $(LDLIBS)

# Restore original file name changed by workaround.
fixed.lex.yy.c: lex.yy.c Makefile
	sed 's/fixed-$(PROG).l/$(PROG).l/' < $< > $@

# Workaround for off-by-one error in lex line numbers.
fixed-$(PROG).l: $(PROG).l Makefile
	awk '/resetline/ \
	    { printf("#line %d \"$(PROG).l\"\n", NR+1); next; } \
	    // {print;}' < $< > $@

lex.yy.c: fixed-$(PROG).l Makefile
	flex $<

clean:
	rm -f $(PROG) lex.yy.c fixed.lex.yy.c
