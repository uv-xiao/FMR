CC := g++
SRCDIR := src
BUILDDIR := build
BINDIR := bin
TESTDIR := test

SOURCES := $(shell find $(SRCDIR) -type f -name *.cpp)
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.cpp=.o))


$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	$(CC) -c -o $@ $< -g

lib: flute
	mkdir -p $(BUILDDIR)
	cd flute; gcc -g -c *.c; mv *.o ../build

clean:
	rm -rf $(BUILDDIR)
	rm -rf $(BINDIR)

all: $(OBJECTS) $(MAIN)
	mkdir -p $(BINDIR)
	g++ build/*.o -o $(BINDIR)/a.out -g
.PHONY: clean all lib
