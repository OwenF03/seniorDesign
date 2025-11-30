CXX=g++
#flags for importing matlab values
MATFLAGS= -I${MATLAB_ROOT}/extern/include -L${MATLAB_ROOT}/bin/glnxa64 -Wl,-rpath,${MATLAB_ROOT}/bin/glnxa64 -lmat -lmx -leng
LDFLAGS=
BASE_CFLAGS= -g -Wall -Wextra

BUILD ?= release

ifeq ($(BUILD),debug)
	MUSIC_SRC = MUSIC.cpp
	CFLAGS = $(BASE_CFLAGS) -O0 -DDEBUG
	OUTPUT = music_debug.out
else ifeq ($(BUILD), debugS)
	MUSIC_SRC = MUSIC_single.cpp
	CFLAGS = $(BASE_CFLAGS)  -O0 -DDEBUG
	OUTPUT = music_single_debug.out
else ifeq ($(BUILD), releaseS)
	MUSIC_SRC = MUSIC_single.cpp
	CFLAGS = $(BASE_CFLAGS) -O2 
	OUTPUT = music_single.out
else
	MUSIC_SRC = MUSIC.cpp
	CFLAGS = $(BASE_CFLAGS) -O2
	OUTPUT = music.out

endif

SRCS= musTest.cpp readMatFile.cpp $(MUSIC_SRC)
OBJS = $(SRCS:.cpp=.o)

all: $(OUTPUT)

debugSingle: 
	$(MAKE) BUILD=debugS

single: 
	$(MAKE) BUILD=releaseS

music: 
	$(MAKE) BUILD=music

debug:
	$(MAKE) BUILD=debug

$(OUTPUT): $(OBJS)
	$(CXX) $(OBJS) -o $(OUTPUT) $(MATFLAGS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -c $< -o $@ $(MATFLAGS)

clean:
	rm -f $(OUTPUT) $(OBJS) *.d
	-rm music.o
	-rm MUSIC_single.o
	-rm music_single.o
	-rm music_single_debug.o
	-rm music_debug.o



