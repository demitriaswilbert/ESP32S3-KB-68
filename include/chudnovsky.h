/**
 * @file chudnovsky.h
 * @author Demitrias Wilbert (demitriaswilbert@gmail.com)
 * @brief Chudnovsky Algorithm using gmp-ino
 * 
 * based on https://en.wikipedia.org/wiki/Chudnovsky_algorithm#Python_code
 *      
 * @version 0.1
 * @date 2024-10-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <gmp-ino.h>

typedef struct {
    mpz_t P, Q, R;
} PQR;

typedef struct {
    mpz_t c3_24;
    mpz_t a3;
    mpz_t _545140134;
    size_t nres;
    PQR* res[2];
} BS_t;

typedef struct {
    char* buf;
    size_t size;
} string_t;

#define mpzinit(v) mpz_init(v)
#define mpzclear(v) mpz_clear(v)

/**
 * @brief 
 * 
 * @param digits 
 * @return mpz_t 
 */
void chudnovsky(mpz_t result, int digits);