#ifndef _OPENSOURCE_ADPCM_H_
#define _OPENSOURCE_ADPCM_H_

#include <stdint.h>
#include <stdbool.h>

struct adpcm_state
{
    short valprev; /* Previous output value */
    char index;    /* Index into stepsize table */
} __attribute__((packed));

typedef struct adpcm_state adpcm_state_t;

void adpcm_coder(short s[], char d[], int s_bytes, adpcm_state_t *);
void adpcm_decoder(char s[], short d[], int s_bytes, adpcm_state_t *);
#endif
