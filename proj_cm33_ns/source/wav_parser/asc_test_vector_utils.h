/******************************************************************************
 * File Name:   asc_test_vector_utils.h
 *
 * Description: Header for Audio codec MW helper.
 *
 ******************************************************************************
 * (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 ******************************************************************************/

/*******************************************************************************
 * Include guard
 ******************************************************************************/
#ifndef ASC_TEST_VECTOR_UTILS_H
#define ASC_TEST_VECTOR_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

/*******************************************************************************
* Function prototypes
*******************************************************************************/

int asc_byte_stream_read(unsigned char *bitstream, unsigned char *bytes, unsigned int size,unsigned int *current_read_pos, unsigned int bitstream_max );
int asc_byte_stream_write(unsigned char *bytes, unsigned int size);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ASC_TEST_VECTOR_UTILS_H */

/* [] END OF FILE */
