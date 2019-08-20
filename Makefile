# Directory and file variables
SRCDIR		:= src
BINDIR		:= bin
BUILDDIR	:= build
INCLUDEDIR	:= include
TARGET		:= $(BINDIR)/rc_pilot

# File definitions for rules
SOURCES		:= $(shell find $(SRCDIR) -type f -name *.c)
OBJECTS		:= $(SOURCES:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
INCLUDES	:= $(shell find $(INCLUDEDIR) -name '*.h')

CC			:= gcc
LINKER		:= gcc
WFLAGS		:= -Wall -Wextra -Werror
CFLAGS		:= -I $(INCLUDEDIR)
OPT_FLAGS	:= -O1
LDFLAGS		:= -lm -lrt -pthread -lrobotcontrol -ljson-c

RM			:= rm -rf
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
	@$(CC) -c $(CFLAGS) $(OPT_FLAGS) $(DEBUGFLAG) $(WFLAGS) $< -o $(@)
	@echo "made: $(@)"

all: $(TARGET)

debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo "$(TARGET) Make Debug Complete"

docs:
	@cd docs; doxygen Doxyfile

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

.PHONY: all debug docs clean install uninstall runonboot