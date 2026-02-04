CXX=g++
#flags for importing matlab values
MATFLAGS= -I${MATLAB_ROOT}/extern/include -L${MATLAB_ROOT}/bin/glnxa64 -Wl,-rpath,${MATLAB_ROOT}/bin/glnxa64 -lmat -lmx -leng
LDFLAGS=
BASE_CFLAGS= -g -Wall -Wextra
SRC = src

BUILD ?= release

ifeq ($(BUILD),debug)
	MUSIC_SRC = MUSIC.cpp
	CFLAGS = $(BASE_CFLAGS) -O0 -DDEBUG_PRINT -DTEST
	OUTPUT = music_debug.out
else ifeq ($(BUILD),test)
	MUSIC_SRC = MUSIC.cpp
	CFLAGS = $(BASE_CFLAGS) -O0 -DTEST
	OUTPUT = music_test.out
else
	MUSIC_SRC = MUSIC.cpp
	CFLAGS = $(BASE_CFLAGS) -O2
	OUTPUT = music.out

endif

SRCS= $(SRC)/test.cpp $(SRC)/readMatFile.cpp $(SRC)/$(MUSIC_SRC)
OBJS = $(SRCS:.cpp=.o)

all: $(OUTPUT)

music: 
	$(MAKE) BUILD=music
debug:
	$(MAKE) BUILD=debug
test: 
	$(MAKE) BUILD=test

$(OUTPUT): $(OBJS)
	$(CXX) $(OBJS) -o $(OUTPUT) $(MATFLAGS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -c $< -o $@ $(MATFLAGS)

clean:
	rm -f $(SRC)/*.d
	rm -f *.out
	rm -f $(SRC)/*.o
	



