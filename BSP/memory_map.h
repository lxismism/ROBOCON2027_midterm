/**
 * @file memory_map.h
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */
 #pragma once

// dtcmram为dma不可达区域，因此需要额外定义dma缓冲区
#if defined(__GNUC__)
#define DMA_BUFFER_ATTR __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define DMA_BUFFER_ATTR
#endif