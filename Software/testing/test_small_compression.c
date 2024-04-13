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
    uint8_t outBufferBinary[50] = {0xf8, 0x06, 0xab, 0xfb, 0x17, 0x7a, 0x29, 0x50, 0xcb, 0xe7, 0x00, 0x03, 0x04, 0x3f, 0xe6, 0xfb, 0x17, 0xac, 0x29, 0x50, 0xcb, 0xe2, 0x00, 0x04, 0x0c, 0x49, 0x1d, 0xfc, 0x17, 0x95, 0x29, 0x50, 0xcb, 0xe0, 0x00, 0x04, 0x14, 0x3a, 0x54, 0xfc, 0x17, 0x47, 0x29, 0x50, 0xcb, 0xdf, 0x00, 0x04}; 
    size_t outBufferPtr = 0; // outBuffer pointer
    
    // small_packet p0 = {1, 5, 12, -1707483649, 65, 1};
    // small_packet p1 = {0, 0, 0, 0, 0, 0};
    // small_packet p2 = {1, 5, 12, 412, 65, -1};
    // small_packet p3 = {0, 0, 0, 0, 0, 0};

    // small_packet_buffer[0] = p0;
    // small_packet_buffer[1] = p1;
    // small_packet_buffer[2] = p2;
    // small_packet_buffer[3] = p3;
    
    // for (int i = 0; i < 4; i++ ) {
    //     int sp = i * 12;
    //     small_packet cur = small_packet_buffer[i];

    //     outBufferBinary[sp + 0] = ((cur.id) & GET_BOTTOM_BITS(2));
    //     outBufferBinary[sp + 0] |= (cur.seconds) & GET_BOTTOM_BITS(6) << 2;

    //     outBufferBinary[sp + 1] = cur.latitude & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 2] = (cur.latitude >> 8) & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 3] = (cur.latitude >> 16) & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 4] = (cur.latitude >> 24) & GET_BOTTOM_BITS(8);

    //     outBufferBinary[sp + 5] = cur.longitude & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 6] = (cur.longitude >> 8) & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 7] = (cur.longitude >> 16) & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 8] = (cur.longitude >> 24) & GET_BOTTOM_BITS(8);
        
    //     outBufferBinary[sp + 9] = (cur.altitude) & GET_BOTTOM_BITS(8);
    //     outBufferBinary[sp + 10] = (cur.altitude >> 8) & GET_BOTTOM_BITS(8);
        
    //     outBufferBinary[sp + 11] = (cur.minutes & GET_BOTTOM_BITS(6));
    //     outBufferPtr+=12;
    // }
    
    for (int i = 0; i < outBufferPtr; i++) {
        printf("%02X ", outBufferBinary[i]);
    }
    printf("\n");

    for (int i = 0; i < 4; i++) {
        uint8_t id_0 = (outBufferBinary[12 * i + 0]) & GET_BOTTOM_BITS(2);
        uint8_t seconds_0 = (outBufferBinary[12 * i + 0] >> 2) & GET_BOTTOM_BITS(6);
        uint8_t minutes_0 = (outBufferBinary[12 * i + 11] & GET_BOTTOM_BITS(6));
        int32_t latitude_0 = (outBufferBinary[12 * i + 1] | (outBufferBinary[12 * i + 2] << 8) | (outBufferBinary[12 * i + 3] << 16) | (outBufferBinary[12 * i + 4] << 24));
        int32_t longitude_0 = (outBufferBinary[12 * i + 5] | (outBufferBinary[12 * i + 6] << 8) | (outBufferBinary[12 * i + 7] << 16) | (outBufferBinary[12 * i + 8] << 24));
        int16_t altitude_0 = (outBufferBinary[12 * i + 9] | (outBufferBinary[12 * i + 10] << 8));

        printf("ID: %02X, SEC: %d, MIN: %d, LAT (d): %f, LONG (d): %f, ALT (m): %d \n", id_0, seconds_0, minutes_0, ((float)latitude_0) * 0.0000001, ((float)longitude_0) * 0.0000001, altitude_0);
    }


    printf("\n");
    return 0;
}