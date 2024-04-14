#include <stdio.h>
#include <stdint.h>

#define GET_BOTTOM_BITS(n) ((1 << n) - 1)

typedef struct small_packet {
  uint8_t id : 2; // compressed to 2 bits
  int seconds : 6; // compressed to 6 bits
  int minutes : 6; // compressed to 6 bits
  int32_t latitude; 
  int32_t longitude; 
  int32_t altitude; // compressed to 16 bits
  uint16_t vbatt;
  uint8_t sats;
  uint16_t pressure;
} small_packet;

small_packet small_packet_buffer[4];

int main() {
    uint8_t outBufferBinary[50]; 
    size_t outBufferPtr = 0; // outBuffer pointer
    
    small_packet p0 = {1, 5, 12, -1707483649, 65, 1, 432, 123, 32};
    small_packet p1 = {0, 0, 0, 0, 0, 0};
    small_packet p2 = {1, 5, 12, -1707483649, -8812312, 1, 312, 123, 32};
    small_packet p3 = {0, 0, 0, 0, 0, 0};

    small_packet_buffer[0] = p0;
    small_packet_buffer[1] = p1;
    small_packet_buffer[2] = p2;
    small_packet_buffer[3] = p3;
    
    outBufferBinary[0] = small_packet_buffer[0].id & GET_BOTTOM_BITS(2);

    outBufferBinary[0] |= (small_packet_buffer[0].minutes & GET_BOTTOM_BITS(6)) << 2;

    outBufferBinary[1] = small_packet_buffer[0].seconds & GET_BOTTOM_BITS(6);

    outBufferBinary[1] |= (small_packet_buffer[1].seconds & GET_BOTTOM_BITS(2)) << 6;
    outBufferBinary[2] = (small_packet_buffer[1].seconds >> 2) & GET_BOTTOM_BITS(4);

    outBufferBinary[2] |= (small_packet_buffer[2].seconds & GET_BOTTOM_BITS(4)) << 4;
    outBufferBinary[3] = (small_packet_buffer[2].seconds >> 4) & GET_BOTTOM_BITS(2);

    outBufferBinary[3] |= (small_packet_buffer[3].seconds & GET_BOTTOM_BITS(6)) << 6;

    outBufferBinary[4] = (small_packet_buffer[0].pressure) & GET_BOTTOM_BITS(8);
    outBufferBinary[5] = (small_packet_buffer[0].pressure >> 8) & GET_BOTTOM_BITS(8);

    outBufferBinary[6] = (small_packet_buffer[3].pressure) & GET_BOTTOM_BITS(8);
    outBufferBinary[7] = (small_packet_buffer[3].pressure >> 8) & GET_BOTTOM_BITS(8);

    outBufferBinary[8] = ((small_packet_buffer[3].vbatt >> 1) & GET_BOTTOM_BITS(8));
    outBufferBinary[9] = (small_packet_buffer[3].sats & GET_BOTTOM_BITS(8));
    
    // cant loop this cause packets are 12.25 bytes (AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH)
    for (int i = 1; i < 5; i++ ) {
        int sp = i * 10;
        small_packet cur = small_packet_buffer[i - 1];

        outBufferBinary[sp + 0] = cur.latitude & GET_BOTTOM_BITS(8);
        outBufferBinary[sp + 1] = (cur.latitude >> 8) & GET_BOTTOM_BITS(8);
        outBufferBinary[sp + 2] = (cur.latitude >> 16) & GET_BOTTOM_BITS(8);
        outBufferBinary[sp + 3] = (cur.latitude >> 24) & GET_BOTTOM_BITS(8);

        outBufferBinary[sp + 4] = cur.longitude & GET_BOTTOM_BITS(8);
        outBufferBinary[sp + 5] = (cur.longitude >> 8) & GET_BOTTOM_BITS(8);
        outBufferBinary[sp + 6] = (cur.longitude >> 16) & GET_BOTTOM_BITS(8);
        outBufferBinary[sp + 7] = (cur.longitude >> 24) & GET_BOTTOM_BITS(8);
        
        outBufferBinary[sp + 8] = (cur.altitude) & GET_BOTTOM_BITS(8);
        outBufferBinary[sp + 9] = (cur.altitude >> 8) & GET_BOTTOM_BITS(8);
    }
    outBufferPtr = 50;
    
    for (int i = 0; i < outBufferPtr; i++) {
        printf("%02X ", outBufferBinary[i]);
    }
    printf("\n");

    small_packet decoded[4];

    decoded[0].id = outBufferBinary[0] & GET_BOTTOM_BITS(2);
    decoded[0].minutes = (outBufferBinary[0] >> 2) & GET_BOTTOM_BITS(6);
    decoded[0].seconds = outBufferBinary[1] & GET_BOTTOM_BITS(6);

    decoded[1].seconds = ((outBufferBinary[1] >> 6) & GET_BOTTOM_BITS(2)) | ((outBufferBinary[2] & GET_BOTTOM_BITS(4)) << 2);
    decoded[2].seconds = ((outBufferBinary[2] >> 4) & GET_BOTTOM_BITS(4)) | ((outBufferBinary[3] & GET_BOTTOM_BITS(2)) << 4);
    decoded[3].seconds = (outBufferBinary[3] >> 2) & GET_BOTTOM_BITS(6);

    decoded[0].pressure = (outBufferBinary[4] & GET_BOTTOM_BITS(8)) | ((outBufferBinary[5] & GET_BOTTOM_BITS(8)) << 8);
    decoded[3].pressure = (outBufferBinary[6] & GET_BOTTOM_BITS(8)) | ((outBufferBinary[7] & GET_BOTTOM_BITS(8)) << 8);

    decoded[3].vbatt = (outBufferBinary[8] << 1);
    decoded[3].sats = outBufferBinary[9];

    for (int i = 1; i < 5; i++) {
        small_packet cur = small_packet_buffer[i-1];
        
        decoded[i - 1].latitude = (outBufferBinary[10 * i + 0]) | (outBufferBinary[10 * i + 1] << 8) | (outBufferBinary[10 * i + 2] << 16) | (outBufferBinary[10 * i + 3] << 24);
        decoded[i - 1].longitude = (outBufferBinary[10 * i + 0]) | (outBufferBinary[10 * i + 5] << 8) | (outBufferBinary[10 * i + 6] << 16) | (outBufferBinary[10 * i + 7] << 24);
        decoded[i - 1].altitude = (outBufferBinary[10 * i + 8]) | (outBufferBinary[10 * i + 9] << 8);
    }

    for (int i = 0; i < 4; i++) {
        small_packet cur = small_packet_buffer[i];

        printf("ID: %d, SEC: %d, MIN: %d, LAT (d): %f, LONG (d): %f, ALT (m): %d, VBAT (V): %f, PRESSURE (mbar): %d\n", cur.id, cur.seconds, cur.minutes, cur.latitude * 0.0000001, cur.longitude * 0.0000001, cur.altitude, cur.vbatt * 0.01, cur.pressure);
    }


    printf("\n");
    return 0;
}