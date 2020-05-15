#
# Copyright (C) 2011-2020 Canonical, Ltd.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#

VERSION=0.02.23

CFLAGS += -Wall -Wextra -DVERSION='"$(VERSION)"'

#
# Pedantic flags
#
ifeq ($(PEDANTIC),1)
CFLAGS += -Wabi -Wcast-qual -Wfloat-equal -Wmissing-declarations \
	-Wmissing-format-attribute -Wno-long-long -Wpacked \
	-Wredundant-decls -Wshadow -Wno-missing-field-initializers \
	-Wno-missing-braces -Wno-sign-compare -Wno-multichar
endif

prefix=/usr
BINDIR=$(prefix)/bin
MANDIR=$(prefix)/share/man/man8
BASHDIR=$(prefix)/share/bash-completion/completions

powerstat: powerstat.o
	$(CC) $(CFLAGS) $< -lm -o $@ $(LDFLAGS)

powerstat.8.gz: powerstat.8
	gzip -c $< > $@

dist:
	rm -rf powerstat-$(VERSION)
	mkdir powerstat-$(VERSION)
	cp -rp Makefile mascot powerstat.c powerstat.8 COPYING snap \
		.travis.yml bash-completion powerstat-$(VERSION)
	tar -zcf powerstat-$(VERSION).tar.gz powerstat-$(VERSION)
	rm -rf powerstat-$(VERSION)

clean:
	rm -f powerstat powerstat.o powerstat.8.gz
	rm -f powerstat-$(VERSION).tar.gz

install: powerstat powerstat.8.gz
	mkdir -p ${DESTDIR}${BINDIR}
	cp powerstat ${DESTDIR}${BINDIR}
	mkdir -p ${DESTDIR}${MANDIR}
	cp powerstat.8.gz ${DESTDIR}${MANDIR}
	mkdir -p ${DESTDIR}${BASHDIR}
	cp bash-completion/powerstat ${DESTDIR}${BASHDIR}
