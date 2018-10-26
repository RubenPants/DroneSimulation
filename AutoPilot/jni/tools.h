/**
 * General tools.
 *
 * @version Team Safier
 * @version 1.0
 */

//
// DEBUGGING
//

/*
 * Print out the bits of the given unsigned integer.
 */
void showbits(unsigned int num) {
    unsigned int size = sizeof(unsigned int);
    unsigned int maxPow = 1<<(size*8-1);
    for(int i=0 ; i<size*8 ; ++i) {
        printf("%u ",num&maxPow ? 1 : 0); // Print last bit and shift left.
        num = num<<1;
    }
}