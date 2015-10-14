#
# Copyright (C) 2011-2015 Canonical, Ltd.
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

VERSION=0.02.06

CFLAGS += -Wall -Wextra -DVERSION='"$(VERSION)"'

BINDIR=/usr/bin
MANDIR=/usr/share/man/man8

powerstat: powerstat.o
	$(CC) $(CFLAGS) $< -lm -o $@ $(LDFLAGS)

powerstat.8.gz: powerstat.8
	gzip -c $< > $@

dist:
	rm -rf powerstat-$(VERSION)
	mkdir powerstat-$(VERSION)
	cp -rp Makefile powerstat.c powerstat.8 COPYING powerstat-$(VERSION)
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
