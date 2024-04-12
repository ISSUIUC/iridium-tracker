#include <stdio.h>
#include <stdint.h>

#define GET_BOTTOM_BITS(n) ((1 << n) - 1)

typedef struct small_packet {
  uint8_t id; // compressed to 2 bits
  uint8_t seconds; // compressed to 6 bits
  uint8_t minutes; // compressed to 6 bits
  int32_t latitude; // compressed to 26 bits
  int32_t longitude; // compressed to 26 bits
  int32_t altitude; // stays at 32 bits
} small_packet;

small_packet small_packet_buffer[4];

int main() {
    uint8_t outBufferBinary[50] = {0x24, 0xf8, 0xe8, 0x17, 0x06, 0x66, 0x69, 0xcb, 0xcc, 0x01, 0x0c, 0x13, 0x9d, 0x19, 0xe9, 0x17, 0x78, 0x5b, 0x69, 0xcb, 0xd3, 0x00, 0x13, 0x13, 0x3c, 0x1f, 0xe9, 0x17, 0x78, 0x59, 0x69, 0xcb, 0xa7, 0x00, 0x1a, 0x13, 0x36, 0x1a, 0xe9, 0x17, 0x0b, 0x5c, 0x69, 0xcb, 0xd6, 0x00, 0x21, 0x13, 0x00}; 
    size_t outBufferPtr = 0; // outBuffer pointer
    
    small_packet p0 = {1, 5, 12, -1707483649, 65, 1};
    small_packet p1 = {0, 0, 0, 0, 0, 0};
    small_packet p2 = {1, 5, 12, 412, 65, -1};
    small_packet p3 = {0, 0, 0, 0, 0, 0};

    small_packet_buffer[0] = p0;
    small_packet_buffer[1] = p1;
    small_packet_buffer[2] = p2;
    small_packet_buffer[3] = p3;
    
    // for (int i = 0; i < 4; i++ ) {
    //     int sp = i * 12;
    //     small_packet cur = small_packet_buffer[i];

    //     outBufferBinary[sp + 0] = cur.latitude & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 1] = (cur.latitude >> 8) & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 2] = (cur.latitude >> 16) & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 3] = (cur.latitude >> 24) & GET_BOTTOM_BITS(8);

    //     outBufferBinary[sp + 4] = cur.longitude & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 5] = (cur.longitude >> 8) & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 6] = (cur.longitude >> 16) & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 7] = (cur.longitude >> 24) & GET_BOTTOM_BITS(8);
        
    //     outBufferBinary[sp + 8] = (cur.altitude) & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 9] = (cur.altitude >> 8) & GET_BOTTOM_BITS(8);
        
    //     outBufferBinary[sp + 10] = (cur.seconds) & GET_BOTTOM_BITS(6);
    //     outBufferBinary[sp + 10] |= ((cur.id) & GET_BOTTOM_BITS(2)) << 6;
    //     outBufferBinary[sp + 11] = (cur.minutes & GET_BOTTOM_BITS(6));
    //     outBufferPtr += 12;
    // }
    
    for (int i = 0; i < outBufferPtr; i++) {
        printf("%02X ", outBufferBinary[i]);
    }
    printf("\n");

    for (int i = 0; i < 4; i++) {
        uint8_t id_0 = (outBufferBinary[12 * i + 10] >> 6) & GET_BOTTOM_BITS(2);
        uint8_t seconds_0 = (outBufferBinary[12 * i + 10]) & GET_BOTTOM_BITS(6);
        uint8_t minutes_0 = (outBufferBinary[12 * i + 11] & GET_BOTTOM_BITS(6));
        int32_t latitude_0 = (outBufferBinary[12 * i + 0] | (outBufferBinary[12 * i + 1] << 8) | (outBufferBinary[12 * i + 2] << 16) | (outBufferBinary[12 * i + 3] << 24));
        int32_t longitude_0 = (outBufferBinary[12 * i + 4] | (outBufferBinary[12 * i + 5] << 8) | (outBufferBinary[12 * i + 6] << 16) | (outBufferBinary[12 * i + 7] << 24));
        int32_t altitude_0 = (outBufferBinary[12 * i + 8] | (outBufferBinary[12 * i + 9] << 8));

        printf("ID: %02X, SEC: %d, MIN:  %d, LAT:  %d, LONG: %d, ALT: %d \n", id_0, seconds_0, minutes_0, latitude_0, longitude_0, altitude_0);
    }


    printf("\n");
    return 0;
}