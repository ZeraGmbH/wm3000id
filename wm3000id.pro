TEMPLATE	= app
LANGUAGE	= C++

QMAKE_CXXFLAGS += -O0

CONFIG	+= qt warn_on release debug

HEADERS	+= cmdinterpret.h \
	crcutils.h \
	gaussmatrix.h \
	i2ceeprom.h \
	i2cutils.h \
	intelhexfileio.h \
	justdata.h \
	parse.h \
	scpi.h \
	wm3000id.h \
	wmiglobal.h \
	wmjustdata.h \
	wmscpi.h \
	zeraglobal.h \
	zhserver.h \
        wmjustdatabase.h \
        wmjustdatav208.h \
    justdatabase.h \
    justdatav208.h

SOURCES	+= cmdinterpret.cpp \
	cmds.cpp \
	crcutils.cpp \
	gaussmatrix.cpp \
	i2ceeprom.cpp \
	i2cutils.cpp \
	intelhexfileio.cpp \
	justdata.cpp \
	main.cpp \
	parse.cpp \
	scpi.cpp \
	wm3000id.cpp \
	wmjustdata.cpp \
	zhserver.cpp \
        wmjustdatav208.cpp \
    justdatabase.cpp \
    justdatav208.cpp

# Input
#The following line was inserted by qt3to4
QT += xml  qt3support 

target.path = /usr/bin
INSTALLS += target
