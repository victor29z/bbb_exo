arm-linux-gnueabihf-gcc can_comm.cpp -o can_comm -I. -L/usr/lib/libsocketcan -lsocketcan -lstdc++ -lpthread

arm-linux-gnueabihf-gcc bbb_exo.cpp -o bbb_exo -I. -L/usr/lib/libsocketcan -lsocketcan -lstdc++ -lpthread
