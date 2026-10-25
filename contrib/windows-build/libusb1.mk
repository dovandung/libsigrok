# This file is part of MXE. See LICENSE.md for licensing information.
#
# sigrok: stock libusb 1.0.30 instead of the 1.0.20-rc3 "event-abstraction"
# fork. Replaces src/libusb1.mk as a whole file, so it works whatever MXE
# commit or sigrok mxe_fixes.patch was applied before. RAW_IO policy:
# src/libusb1-1-fixes.patch (copy it next to this file).

PKG             := libusb1
$(PKG)_WEBSITE  := https://libusb.info/
$(PKG)_DESCR    := LibUsb-1.0
$(PKG)_IGNORE   :=
$(PKG)_VERSION  := 1.0.30
$(PKG)_CHECKSUM := fea36f34f9156400209595e300840767ab1a385ede1dc7ee893015aea9c6dbaf
$(PKG)_SUBDIR   := libusb-$($(PKG)_VERSION)
$(PKG)_FILE     := libusb-$($(PKG)_VERSION).tar.bz2
$(PKG)_URL      := https://github.com/libusb/libusb/releases/download/v$($(PKG)_VERSION)/$($(PKG)_FILE)
$(PKG)_DEPS     := cc

define $(PKG)_UPDATE
    $(WGET) -q -O- 'https://github.com/libusb/libusb/releases' | \
    $(SED) -n 's,.*/releases/tag/v\([0-9][0-9.]*\)".*,\1,p' | \
    head -1
endef

define $(PKG)_BUILD
    cd '$(1)' && ./configure \
        $(MXE_CONFIGURE_OPTS)
    $(MAKE) -C '$(1)' -j '$(JOBS)' install
endef
