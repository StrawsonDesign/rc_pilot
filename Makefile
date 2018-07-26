# This is a general use makefile for robotics cape projects written in C.
# Just change the target name to match your main source code filename.

SRCDIR		:= src
BINDIR		:= bin
BUILDDIR	:= build
INCLUDEDIR	:= include
TARGET		:= $(BINDIR)/rc_pilot

# file definitions for rules
SOURCES		:= $(shell find $(SRCDIR) -type f -name *.c)
OBJECTS		:= $(SOURCES:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
INCLUDES	:= $(shell find $(INCLUDEDIR) -name '*.h')

CC		:= gcc
LINKER		:= gcc
WFLAGS		:= -Wall -Wextra
CFLAGS		:= -I $(INCLUDEDIR)
OPT_FLAGS	:= -O1
LDFLAGS		:= -lm -lrt -pthread -lrobotcontrol -ljson-c

RM		:= rm -rf
INSTALL		:= install -m 4755
INSTALLDIR	:= install -d -m 755

LINK		:= ln -s -f
LINKDIR		:= /etc/robotcontrol
LINKNAME	:= link_to_startup_program

prefix		?= /usr


# linking Objects
$(TARGET): $(OBJECTS)
	@mkdir -p $(BINDIR)
	@$(LINKER) -o $(@) $(OBJECTS) $(LDFLAGS)
	@echo "made: $(@)"

# rule for all other objects
$(BUILDDIR)/%.o : $(SRCDIR)/%.c $(INCLUDES)
	@mkdir -p $(dir $(@))
	@$(CC) -c $(CFLAGS) $(OPT_FLAGS) $(DEBUGFLAG) $< -o $(@)
	@echo "made: $(@)"

all: $(TARGET)

debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo "$(TARGET) Make Debug Complete"

install:
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/bin
	@echo "$(TARGET) Install Complete"

clean:
	@$(RM) $(BINDIR)
	@$(RM) $(BUILDDIR)
	@$(RM) docs/html
	@echo "Library Clean Complete"

uninstall:
	@$(RM) $(DESTDIR)$(prefix)/$(TARGET)
	@echo "$(TARGET) Uninstall Complete"

runonboot:
	@$(MAKE) install
	@$(LINK) $(DESTDIR)$(prefix)/bin/$(TARGET) $(LINKDIR)/$(LINKNAME)
	@echo "$(TARGET) Set to Run on Boot"

