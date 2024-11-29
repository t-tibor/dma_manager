/**
 * Zero-copy DMA driver
 * Copyright (C) 2023-2024 Tibor Tusori
 * 
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef USERMEM_DMA_H
#define USERMEM_DMA_H

#include <linux/kernel.h>


/**
 * @brief Direction of the dma periphery.
 * 
 */
enum zcdma_dma_dir
{
    ZCDMA_DIR_READ = 0,
    ZCDMA_DIR_WRITE
};


/**
 * @brief Structure describing the dma hw channel.
 * 
 */
struct dma_hw_channel_info
{
    enum zcdma_dma_dir  direction;
    struct dma_chan*    dma_chan;
};


/**
 * @brief Forward declaration of the zerocopy-dma structure.
 * Only pointers to this type are used in the API,
 * so the implementation details can be hidden.
 * 
 */
struct zcdma;


/**
 * @brief Create a new zerocopy-dma session.
 * 
 * @param hw_info   Structure describing the dma HW channel to be used.
 *                  This structure is copied, so it shall not be preserved by the caller.
 * @return struct zcdma*    Pointer to a resulting structure,
 *                          or NULL when an error occurred.
 */
struct zcdma* zcdma_alloc(const struct dma_hw_channel_info* const hw_info);


/**
 * @brief Perform a DMA read operation and store the recevied data
 * into the given userspace memory buffer.
 * 
 * @param session Session object to use for the read operation.
 * @param userbuf Userspace memory buffer to store the received data into.
 * @param len Number of bytes to be transferred by the DMA.
 * @return ssize_t Number of received bytes.
 */
ssize_t zcdma_read( struct zcdma* session, 
                    char __user* userbuf, 
                    long unsigned int len );


/**
 * @brief Perform a DMA write operation that sends out the data pointed
 * by the \ref userbuf buffer.
 * 
 * @param session Session object to use for the write operation.
 * @param userbuf Userspace memory buffer to write the data from.
 * @param len Number of bytes to by transferred by the DMA.
 * @return ssize_t Number of bytes successfully written.
 */
ssize_t zcdma_write( struct zcdma* session,
                     const char __user* userbuf, 
                     long unsigned int len );


/**
 * @brief Deinit the given zcdma session structure
 *                  and free its memory.
 * 
 * @param session Zerocopy dma session to delete.
 */
void zcdma_free(struct zcdma* session);

#endif /* USERMEM_DMA_H */