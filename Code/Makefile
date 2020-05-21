all: myrtes

VAR= myrtes 

CC = g++


CPPFILES = main.cpp motor.cpp object_detection.cpp 
LDFLAGS = -lpthread -lrt -lm -lwiringPi -L/usr/lib -lopencv_core -lopencv_video -lopencv_flann -lraspicam 
CPPLIBS= -I/usr/local/include/ 
FLAG= -D$(VAR)
OBJS = $(CPPFILES:.cpp=.o)

myrtes:	$(OBJS)
	$(CC) -g $(FLAG) $(OBJS) -o $@ `pkg-config --libs opencv` $(CPPLIBS) $(LDFLAGS)

	
clean:
	rm -f $(VAR) *.o *.d
	-rm -f myrtes

