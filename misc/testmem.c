#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

// BRAM register addresses
#define BASE_ADDR      0x40000000
#define ENABLE_OFFSET  0x00
#define COUNTER_OFFSET 0x04

uint32_t data_array[1000];

int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Usage: %s <number of reads>\n", argv[0]);
        return 1;
    }

    int num_reads = atoi(argv[1]);
    if (num_reads <= 0) {
        printf("Invalid number of reads.\n");
        return 1;
    }

    if (num_reads > 999) {
        printf("Capped at 1000.\n");
        num_reads = 1000;
    }

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    void *map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, BASE_ADDR & ~MAP_MASK);
    if (map_base == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }

    volatile uint32_t *enable_ptr  = (volatile uint32_t *)((char *)map_base + ENABLE_OFFSET);
    volatile uint32_t *counter_ptr = (volatile uint32_t *)((char *)map_base + COUNTER_OFFSET);

    // Enable the counter
    *enable_ptr = 1;

    // Read counter value N times
    for (int i = 0; i < num_reads; i++) {
        data_array[i] = *counter_ptr;
    }

    // Optionally disable the counter
    *enable_ptr = 0;

    munmap((void *)map_base, MAP_SIZE);
    close(fd);

    // print counter values
    for (int i = 0; i < num_reads; i++) {
        printf("Read %d: %u\n", i, data_array[i]);
    }

    return 0;
}
