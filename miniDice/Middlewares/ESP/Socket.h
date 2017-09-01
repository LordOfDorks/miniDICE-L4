#ifndef __SOCKET_H
#define __SOCKET_H
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
    
extern const char* SocketUDP;
extern const char* SocketTCP;

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef SWAP_UINT16
#define SWAP_UINT16(x) (((x) >> 8) | ((x) << 8))
#endif
#ifndef SWAP_UINT32
#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))
#endif
    
HAL_StatusTypeDef  InitializeESP8266(
    uint32_t* ip,
    uint8_t* mac,
    uint32_t timeout,
    char* apsid,
    char* appwd,
    bool autoconn);

HAL_StatusTypeDef SocketOpen(
    uint32_t handle,
    const char* socketType,
    const char* hostname,
    uint32_t destinationPort,
    uint32_t timeout);
    
HAL_StatusTypeDef SocketClose(
    uint32_t handle,
    uint32_t timeout);
    
HAL_StatusTypeDef SocketSend(
    uint32_t handle,
    uint8_t* data,
    uint32_t dataSize,
    uint32_t timeout);
    
HAL_StatusTypeDef  SocketReceive(
    uint32_t handle,
    uint8_t* data,
    uint32_t dataSize,
    uint32_t timeout);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
