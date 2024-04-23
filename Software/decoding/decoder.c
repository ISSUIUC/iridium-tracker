#include <stdio.h>
#include <stdint.h>

#define GET_BOTTOM_BITS(n) ((1 << n) - 1)

typedef struct small_packet {
  uint8_t id : 2; // compressed to 2 bits
  uint8_t seconds : 6; // compressed to 6 bits
  uint8_t minutes : 6; // compressed to 6 bits
  int32_t latitude; 
  int32_t longitude; 
  int32_t altitude; // compressed to 16 bits
  uint16_t vbatt;
  uint16_t pressure;
  uint8_t sats;
} small_packet;

small_packet small_packet_buffer[4];

int main() {
    while (1) {
        char input_string[100];

        printf("Enter the encoded text:\n");
        fgets(input_string, sizeof(input_string), stdin);

        int numBytes = sizeof(input_string) / sizeof(input_string[0]) - 1;

        uint8_t outBufferBinary[50];

        for (int i = 0, j = 0; i < numBytes; i += 2, j++) {
            sscanf(input_string + i, "%2hhx", &outBufferBinary[j]);
        }

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
            decoded[i - 1].latitude = (outBufferBinary[10 * i + 0]) | (outBufferBinary[10 * i + 1] << 8) | (outBufferBinary[10 * i + 2] << 16) | (outBufferBinary[10 * i + 3] << 24);
            decoded[i - 1].longitude = (outBufferBinary[10 * i + 4]) | (outBufferBinary[10 * i + 5] << 8) | (outBufferBinary[10 * i + 6] << 16) | (outBufferBinary[10 * i + 7] << 24);
            decoded[i - 1].altitude = (outBufferBinary[10 * i + 8]) | (outBufferBinary[10 * i + 9] << 8);
        }

        for (int i = 0; i < 4; i++) {
            small_packet cur = decoded[i];
            printf("Decoded\t ID: %01d, SEC: %02d, MIN: %02d, LAT (d): %f, LONG (d): %f, ALT (m): %d, VBAT (V): %f, PRESSURE (mbar): %d, SATS: %d\n", cur.id, cur.seconds, cur.minutes, cur.latitude * 0.0000001, cur.longitude * 0.0000001, cur.altitude, cur.vbatt * 0.01, cur.pressure, cur.sats);
        }
        fgets(input_string, sizeof(input_string), stdin);
        printf("\n");
    }


    return 0;
}