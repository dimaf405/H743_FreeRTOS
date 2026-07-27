#include "bootutil/sign_key.h"

extern const unsigned char ecdsa_pub_key[];
extern unsigned int ecdsa_pub_key_len;

const struct bootutil_key bootutil_keys[] = {
    {
        .key = ecdsa_pub_key,
        .len = &ecdsa_pub_key_len,
    },
};

const int bootutil_key_cnt = 1;
