/*
 * Copyright (c) 2022, Nuvoton Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ECP_HELPER_H
#define ECP_HELPER_H

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include "mbedtls/bignum.h"

/**
 * \brief           Check if MPI has been normalized
 *
 * \param N         Input MPI which is to check
 * \param P         Prime modulus
 *
 * \return          0 if not normalized,
 *                  1 if normalized
 */
#define ECP_HELPER_MPI_IS_NORM(N, P)                          \
    ((mbedtls_mpi_cmp_int(&N, 0) >= 0) && (mbedtls_mpi_cmp_mpi(&N, &P) < 0))

/**
 * \brief           Normalize MPI if it is not normalized yet
 *
 * \param R         Holds pointer to normalized MPI (N1 or N2)
 * \param N1        Input MPI which is to normalize
 * \param N2        Output MPI which holds normalized N1 if N1 is not normalized yet
 * \param P         Prime modulus
 */
#define ECP_HELPER_MPI_NORM(R, N1, N2, P)                     \
    do {                                                            \
        if (ECP_HELPER_MPI_IS_NORM(N1, P)) {                  \
            *R = &N1;                                               \
        } else {                                                    \
            MBEDTLS_MPI_CHK(mbedtls_mpi_mod_mpi(&N2, &N1, &P));     \
            *R = &N2;                                               \
        }                                                           \
    } while(0)

#endif /* ECP_HELPER_H */
