# Fixed-size message for zero-copy shared memory communication
# Format: [8 bytes timestamp][4 bytes counter][1024 bytes payload] = 1036 bytes total
# Fixed-size arrays are required for Iceoryx zero-copy to work
uint8[1036] data
