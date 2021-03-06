// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license.

#ifndef RIOT_BASE64_H
#define RIOT_BASE64_H

#ifdef __cplusplus
extern "C" {
#endif

//#define Base64Length(l) ((l == 0) ? (1) : (((((l - 1) / 3) + 1) * 4) + 2))

int
Base64Encode(
    const unsigned char *Input,
    uint32_t             Length,
    char                *Output,
    uint32_t            OutLen
);

int
Base64Decode(
    const char      *Input,
    unsigned char   *Output,
    uint32_t        OutLen
);

#ifdef __cplusplus
}
#endif

#endif
