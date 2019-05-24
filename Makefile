CXXFLAGS =	-O2 -g -Wall -fmessage-length=0

OBJS =		yandex_mitap.o

LIBS =

TARGET =	bin/yandex_mitap

$(TARGET):	$(OBJS)
	@mkdir -p bin
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
