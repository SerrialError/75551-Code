# Makefile — build all src/*.cpp into a binary in bin/
CXX       := g++
CXXFLAGS  := -std=c++17 -O2 -Wall -Wextra -Iinclude $(EIGEN_FLAGS)
LDFLAGS   :=
TARGET    := main
BINDIR    := bin
OBJDIR    := build

# User can override EIGEN_FLAGS when invoking make, e.g.
# make EIGEN_FLAGS="-I/path/to/eigen/include/eigen3"
# On NixOS you usually run inside nix-shell where Eigen is already visible.
SRCS  := $(wildcard src/*.cpp)
OBJS  := $(patsubst src/%.cpp,$(OBJDIR)/%.o,$(SRCS))

.PHONY: all clean debug run

all: $(BINDIR)/$(TARGET)

# Link
$(BINDIR)/$(TARGET): $(OBJS) | $(BINDIR)
	$(CXX) $(LDFLAGS) -o $@ $(OBJS)

# Compile rule for each source -> object
$(OBJDIR)/%.o: src/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# ensure dirs exist
$(BINDIR) $(OBJDIR):
	mkdir -p $@

clean:
	rm -rf $(OBJDIR) $(BINDIR)

# debug build (rebuilds)
debug: CXXFLAGS += -g -O0 -DDEBUG
debug: clean all

# convenience: build then run
run: all
	./$(BINDIR)/$(TARGET)
