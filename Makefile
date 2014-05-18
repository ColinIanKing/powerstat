VERSION=0.01.31

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
