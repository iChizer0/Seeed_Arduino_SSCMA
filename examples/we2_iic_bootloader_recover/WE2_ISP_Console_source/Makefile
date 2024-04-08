OUTPUTDIR = output
SRCDIR = src
OBJDIR = debug
CC = gcc
CXX = g++


ISP_SRC := $(wildcard $(SRCDIR)/*.c) 
ISP_OBJ := $(subst $(SRCDIR),$(OBJDIR),$(ISP_SRC:%.c=%.o))
ISP_CPP_SRC := $(wildcard $(SRCDIR)/*.cpp) 
ISP_CPP_OBJ := $(subst $(SRCDIR),$(OBJDIR),$(ISP_CPP_SRC:%.cpp=%.o))
ISP_LIB = ./library/hmx_ft4222_lib.dll ./${OUTPUTDIR}/hmx_ft4222_lib.dll

ISP_EXE:=$(OUTPUTDIR)/ISP$(E)

EXE=$(ISP_EXE)

CFLAGS+= -Wall -g3 -O2
CXXFLAGS+= -Wall -g3 -O2
INCLUDE= -I ./include
LDFLAGS+= -g3 -O2
ISPLIBLDFLAGS+= -g3 -O2 -s -shared -Wl,--subsystem,windows
LIB+= -L ./library -lhmx_ft4222_lib -lftd2xx -lLibFT4222
ISP_LINK_LIB+= -L ./library -lftd2xx -lLibFT4222

all: compile

compile: $(EXE)

# $(ISP_EXE): ${ISP_OBJ} ${ISP_LIB}
# 	$(CC) ${LDFLAGS} ${ISP_OBJ} ${LIB} ${INCLUDE} -o $@

${ISP_OBJ}: ${OBJDIR}/%.o: ${SRCDIR}/%.c 		
	$(CC) -c ${CFLAGS} ${CFLAGS} ${INCLUDE} $< -o $@
	
# ${ISP_LIB}: ${ISP_CPP_OBJ}
# 	$(CXX) ${ISPLIBLDFLAGS} ${ISP_CPP_OBJ} ${ISP_LINK_LIB} ${INCLUDE} -o $@
		
${ISP_CPP_OBJ}: ${OBJDIR}/%.o: ${SRCDIR}/%.cpp 		
	$(CXX) -c ${CFLAGS} ${CXXFLAGS} ${INCLUDE} $< -o $@	
	
.PHONY: clean
clean:
	rm -f  *.o *.stackdump *.bak $(ISP_EXE).exe $(ISP_OBJ) $(ISP_CPP_OBJ) $(ISP_LIB)


