cmd_Release/obj.target/shm.node := g++ -shared -pthread -rdynamic -m64  -Wl,-soname=shm.node -o Release/obj.target/shm.node -Wl,--start-group Release/obj.target/shm/src/node_shm.o -Wl,--end-group 
