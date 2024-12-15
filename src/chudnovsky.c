#include "chudnovsky.h"

static void initPQR(PQR* pqr) {
    mpz_init(pqr->P);
    mpz_init(pqr->Q);
    mpz_init(pqr->R);
} 

/**
 * @brief initalize
 * 
 * @param pqr 
 */
static void clearPQR(PQR* pqr) {
    mpz_clear(pqr->P);
    mpz_clear(pqr->Q);
    mpz_clear(pqr->R);
}

/**
 * @brief initalize binary split data struct
 * 
 * @param bs struct to initialize
 * @param depth max recursive depth
 */
static void BS_init(BS_t* bs, size_t depth) {
    mpz_init(bs->c3_24);
    mpz_init(bs->a3);
    mpz_init(bs->_545140134);
    mpz_set_str(bs->c3_24, "10939058860032000", 10);
    bs->nres = depth;
    bs->res[0] = (PQR*) malloc(sizeof(PQR) * depth);
    bs->res[1] = (PQR*) malloc(sizeof(PQR) * depth);
    for (int i = 0; i < depth; i++) {
        initPQR(&bs->res[0][i]);
        initPQR(&bs->res[1][i]);
    }
}

/**
 * @brief deinitialize binary split data struct
 * 
 * @param bs struct to deinit
 */
static void BS_deinit(BS_t* bs) {
    mpz_clear(bs->a3);
    mpz_clear(bs->_545140134);
    mpz_clear(bs->c3_24);
    for (int i = 0; i < bs->nres; i++) {
        clearPQR(&bs->res[0][i]);
        clearPQR(&bs->res[1][i]);
    }
    bs->nres = 0;
    free(bs->res[0]);
    free(bs->res[1]);
}

/**
 * @brief recursive binary splitting method
 * 
 * slightly modified version of
 * https://en.wikipedia.org/wiki/Chudnovsky_algorithm#Python_code
 * 
 * @code{.py}
 * def binary_split(a, b):
 *     if b == a + 1:
 *         Pab = -(6*a - 5)*(2*a - 1)*(6*a - 1)
 *         Qab = 10939058860032000 * a**3
 *         Rab = Pab * (545140134*a + 13591409)
 *     else:
 *         m = (a + b) // 2
 *         Pam, Qam, Ram = binary_split(a, m)
 *         Pmb, Qmb, Rmb = binary_split(m, b)
 *         
 *         Pab = Pam * Pmb
 *         Qab = Qam * Qmb
 *         Rab = Qmb * Ram + Pam * Rmb
 *     return Pab, Qab, Rab
 * @endcode
 *
 * @param a start
 * @param b end
 * @param data binary split data struct
 * @param depth current recursive depth
 * @param pos first half (a to m) or second half (m to b)
 * @return PQR* pointer result
 */
static PQR* binary_split(int a, int b, BS_t* data, int depth, int pos) {
    PQR* res = &data->res[pos][depth];
    if (b == a + 1) {
        mpz_set_si(res->P, -(6*a - 5));
        mpz_mul_si(res->P, res->P, 2 * a - 1);
        mpz_mul_si(res->P, res->P, 6 * a - 1);
        
        mpz_set_si(data->a3, a); mpz_pow_ui(data->a3, data->a3, 3);
        mpz_mul(res->Q, data->c3_24, data->a3);

        mpz_set_ui(data->_545140134, 545140134);
        mpz_mul_ui(data->_545140134, data->_545140134, a);
        mpz_add_ui(data->_545140134, data->_545140134, 13591409);
        mpz_mul(res->R, res->P, data->_545140134);
    } else {
        int m = (a + b) / 2;

        PQR *pqr_am = binary_split(a, m, data, depth + 1, 0);
        PQR *pqr_mb = binary_split(m, b, data, depth + 1, 1);

        mpz_mul(res->P, pqr_am->P, pqr_mb->P); 
        mpz_mul(res->Q, pqr_am->Q, pqr_mb->Q); 

        mpz_mul(pqr_am->P, pqr_am->P, pqr_mb->R);
        mpz_mul(res->R, pqr_mb->Q, pqr_am->R);
        mpz_add(res->R, res->R, pqr_am->P);

    }
    return res;
}

/**
 * @brief 
 * 
 * @code{.py}
 * def chudnovsky(n):
 *     """Chudnovsky algorithm."""
 *     n_iter = n / 14 + 1;
 *     P1n, Q1n, R1n = binary_split(1, n_iter)
 *     return (426880 * decimal.Decimal(10005).sqrt() * Q1n) / (13591409*Q1n + R1n)
 * @endcode
 * 
 * @param n 
 * @return string_t 
 */
void chudnovsky(mpz_t result, int digits) {
    
    // binary split data
    BS_t data;

    // calculate iterations and depth

    // precision per iteration = log10((640320**3)/(24*6*2*6))
    // or log10(53660**3) = 14.1889520050077605971726493174695353
    //iter = int((n+1) / (14.1889520050077605971726493174695353)) + 1
    int n_iter = (digits + 1) / 14.18 + 1;
    int depth = 31 - __builtin_clz(n_iter) + 2;

    // initialize mpz variables inside BS_t
    BS_init(&data, depth);

    // magic
    PQR* _1n = binary_split(1, n_iter, &data, 0, 0);

    // numerator = Q * 426880 * sqrt(10005)
    // or isqrt(10005 * 10^2n) * Q * 426880
    // mpz_t num; mpz_init(num);
    mpz_set_ui(result, 10);
    mpz_pow_ui(result, result, digits * 2);
    mpz_mul_ui(result, result, 10005);
    mpz_sqrt(result, result);
    mpz_mul(result, result, _1n->Q); 
    mpz_mul_ui(result, result, 426880);

    // denominator = Q * 13591409 + R
    mpz_t denom; mpz_init(denom);
    mpz_mul_ui(denom, _1n->Q, 13591409); // Multiply by 13591409
    mpz_add(denom, denom, _1n->R); // Add R1n to denominator

    // no need for BS data anymore
    BS_deinit(&data);

    // divide numerator and denominator
    if (mpz_sgn(denom) != 0) 
        mpz_tdiv_q(result, result, denom);
    
    // discard denominator
    mpz_clear(denom);

    // mpz_set(result, result);
    // mpz_clear(result);
}
