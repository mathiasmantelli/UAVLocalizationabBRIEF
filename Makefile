CXX = g++
CFLAGS = -g -O2 -DDEBUG -fPIC -std=c++0x #-Wall

ARIA_INCLUDE=-I/usr/local/Aria/include
ARIA_LINK=-L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt

LFLAGS = $(ARIA_LINK) -lglut -lGL -lfreeimage

OBJS = Utils.o Grid.o  GlutClass.o Robot.o PioneerRobot.o main.o

EXEC = programa

%.o: %.cpp $(DEPS)
	@echo "Compilando $@"
	@$(CXX) $(CFLAGS) $(ARIA_INCLUDE) -c $< -o $@

$(EXEC): $(OBJS)
	@echo "\nLinkando $(EXEC)\n"
	@$(CXX) -o $(EXEC) $(OBJS) $(LFLAGS)

all: teste

clean:
	@echo "Limpando..."
	@rm -f $(OBJS) $(EXEC) *~

