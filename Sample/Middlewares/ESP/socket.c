#include <string.h>
#include "main.h"
#ifdef STM32L0
#include "stm32l0xx_hal.h"
#endif

//#ifdef STM32L4
#include "stm32l4xx_hal.h"
//#endif
#include "socket.h"
#include "errno.h"

#define TIMEOUTLEFT(__timeout, __starttime) ((__timeout == HAL_MAX_DELAY) ? HAL_MAX_DELAY : (__timeout - (HAL_GetTick() - __starttime)))
extern FILE *__wifiRd;
extern FILE *__wifiWr;

const char* SocketUDP = "UDP";
const char* SocketTCP = "TCP";
const char* ResponseOK = "OK\r\n";
const char* ResponseSENDOK = "SEND OK\r\n";
const char* ResponseERROR = "ERROR\r\n";
const char* ResponseALREADYCONNECTED = "ALREADY CONNECTED\r\n\r\n";

//#define printfwifi(...) fprintf(__wifiWr, ##__VA_ARGS__)
//#define fwritewifi(data, block, size) fwrite(data, block, size, __wifiWr);

/*
#ifdef STM32L4

char wifiLine[256] = {0};
// Until we can get either printf working, or va_list
#define WifiSendString(...)  \
    { memset(&wifiLine, 0, 256); \
    sprintf( wifiLine, ##__VA_ARGS__ ); \
    WifiSendBytes( wifiLine, strlen(wifiLine), HAL_MAX_DELAY); }\

#elif defined(STM32L0)

#define __wifi stdout
#define WifiSendString(...)  fprintf(__wifi, ##__VA_ARGS__)
#define WifiSendBytes(data, size, timeout) fwrite(data, 1, size, __wifi);
#endif
*/

char* respspy = NULL;
static HAL_StatusTypeDef ESP8266GetResponse(
    char* responseIn,
    uint32_t respLenIn,
    uint32_t* usedLen,
    uint32_t timeout)
{
    HAL_StatusTypeDef retVal = HAL_OK;
    uint32_t startTime = HAL_GetTick();
    char resp[256] = {0};
    char* response = (responseIn && respLenIn) ? responseIn : resp;
    uint32_t responseLen = (responseIn && respLenIn) ? respLenIn : sizeof(resp);
    uint32_t cursor = 0;
    uint32_t newCursor = 0;
    respspy = response;
    do
    {
        if((timeout != HAL_MAX_DELAY) && (startTime + timeout < HAL_GetTick())) return HAL_TIMEOUT;
        cursor = newCursor;
        fgets(&response[cursor], responseLen - cursor - 1, __wifiRd);
        if((!strcmp(&response[cursor], ResponseOK)) || (!strcmp(&response[cursor], ResponseSENDOK)))
        {
            break;
        }
        else if(!strcmp(&response[cursor], ResponseERROR))
        {
            retVal = HAL_ERROR;
            break;
        }
        newCursor = strlen(response);
    }
    while((newCursor + 1) < responseLen);
    response[cursor] = '\0';
    if(usedLen) *usedLen = cursor;
    return retVal;
}

HAL_StatusTypeDef GetNet(
    uint32_t* ip,
    uint8_t* mac,
    uint32_t timeout)
{
    HAL_StatusTypeDef retVal = HAL_OK;
    uint32_t startTime = HAL_GetTick();
    char response[100];
    uint32_t responseLen = 0;
    uint32_t extracted[10] = {0};

    fprintf( __wifiWr, "AT+CIFSR\r\n");
    if((retVal = ESP8266GetResponse(response, sizeof(response), &responseLen, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        return retVal;
    }
    if(sscanf(response, "+CIFSR:STAIP,\"%lu.%lu.%lu.%lu\"\r\n+CIFSR:STAMAC,\"%lx:%lx:%lx:%lx:%lx:%lx:\"\r\n",
        &extracted[0],
        &extracted[1],
        &extracted[2],
        &extracted[3],
        &extracted[4],
        &extracted[5],
        &extracted[6],
        &extracted[7],
        &extracted[8],
        &extracted[9]) != 10)
    {
        return HAL_ERROR;
    }
    if(ip) *ip = (extracted[0] << 24) | (extracted[1] << 16) | (extracted[2] << 8) | extracted[3];
    if(mac) for(uint32_t n = 0; n < 6; n++) mac[n] = (uint8_t)(extracted[n + 4] & 0x000000ff);

    return retVal;
}

HAL_StatusTypeDef InitializeESP8266(
    uint32_t* ip,
    uint8_t* mac,
    uint32_t timeout,
    char* apsid,
    char* appwd,
    bool autoconn)
{
    HAL_StatusTypeDef retVal = HAL_OK;
    uint32_t startTime = HAL_GetTick();
    //char sidout[100] = {0};
    //uint32_t responseLen = 0;

    if (autoconn)
    {
        if((retVal = GetNet(ip, mac, 10000)) == HAL_OK)
        {
            // Autoconnect succeeded.
            return retVal;
        }
    }

    fprintf( __wifiWr, "ATE0\r\n");
    if((retVal = ESP8266GetResponse(NULL, 0, NULL, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        return retVal;
    }

    fprintf( __wifiWr, "AT+CWMODE_DEF=1\r\n");
    if((retVal = ESP8266GetResponse(NULL, 0, NULL, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        return retVal;
    }

    fprintf( __wifiWr, "AT+CWJAP_DEF=\"%s\",\"%s\"\r\n", apsid, (appwd == NULL) ? "" : appwd);
    if((retVal = ESP8266GetResponse(NULL, 0, NULL, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        return retVal;
    }
/*  // Query connect SID
    fprintf( __wifiWr, "AT+CWJAP?\r\n");
    if((retVal = ESP8266GetResponse(sizeof(sidout), 256, &responseLen, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        return retVal;
    }*/

    fprintf( __wifiWr, "AT+PING=\"pool.ntp.org\"\r\n");
    if((retVal = ESP8266GetResponse(NULL, 0, NULL, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        return retVal;
    }

    fprintf( __wifiWr, "AT+CIPMUX=1\r\n");
    if((retVal = ESP8266GetResponse(NULL, 0, NULL, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        return retVal;
    }
    
    if((ip) || (mac))
    {
        if((retVal = GetNet(ip, mac, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
        {
            return retVal;
        }
    }

    fprintf( __wifiWr, "AT+CWAUTOCONN=%s\r\n", autoconn ? "1" : "0");
    if((retVal = ESP8266GetResponse(NULL, 0, NULL, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        return retVal;
    }

    return retVal;
}

HAL_StatusTypeDef SocketOpen(
    uint32_t handle,
    const char* socketType,
    const char* hostname,
    uint32_t destinationPort,
    uint32_t timeout)
{
    HAL_StatusTypeDef retVal = HAL_BUSY;
    uint32_t startTime = HAL_GetTick();

    do
    {
        char response[128];
        uint32_t responseLen;
        fprintf( __wifiWr, "AT+CIPSTART=%lu,\"%s\",\"%s\",%lu\r\n", handle, socketType, hostname, destinationPort);
        if((retVal = ESP8266GetResponse(response, sizeof(response), &responseLen, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
        {
            if((!strcmp(response, ResponseALREADYCONNECTED)) &&
               ((retVal = SocketClose(handle, TIMEOUTLEFT(timeout, startTime))) == HAL_OK))
            {
                retVal = HAL_BUSY;
                continue; // Let's try again
            }
            goto Cleanup;
        }
    }
    while(retVal != HAL_OK);

Cleanup:
    return retVal;
}

HAL_StatusTypeDef SocketClose(
    uint32_t handle,
    uint32_t timeout)
{
    HAL_StatusTypeDef retVal = HAL_OK;
    uint32_t startTime = HAL_GetTick();

    fprintf( __wifiWr, "AT+CIPCLOSE=%lu\r\n", handle);
    if((retVal = ESP8266GetResponse(NULL, 0, NULL, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        retVal = HAL_ERROR;
        goto Cleanup;
    }

Cleanup:
    return retVal;
}

HAL_StatusTypeDef SocketSend(
    uint32_t handle,
    uint8_t* data,
    uint32_t dataSize,
    uint32_t timeout)
{
    HAL_StatusTypeDef retVal = HAL_OK;
    uint32_t startTime = HAL_GetTick();
    uint32_t sent = 0;
    
    fprintf( __wifiWr, "AT+CIPSEND=%lu,%lu\r\n", handle, dataSize);
    if((retVal = ESP8266GetResponse(NULL, 0, NULL, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        retVal = HAL_ERROR;
        goto Cleanup;
    }

    if((fgetc(__wifiRd) != '>') || (fgetc(__wifiRd) != ' '))
    {
        retVal = HAL_ERROR;
        goto Cleanup;
    }

    if (dataSize != (sent = fwrite(data, 1, dataSize, __wifiWr)))
    {
        retVal = HAL_ERROR;
        goto Cleanup;
    }

    if((retVal = ESP8266GetResponse(NULL, 0, NULL, TIMEOUTLEFT(timeout, startTime))) != HAL_OK)
    {
        retVal = HAL_ERROR;
        goto Cleanup;
    }
Cleanup:
    return retVal;
}

HAL_StatusTypeDef  SocketReceive(
    uint32_t handle,
    uint8_t* data,
    uint32_t dataSize,
    uint32_t timeout)
{
    HAL_StatusTypeDef retVal = HAL_OK;
    uint32_t startTime = HAL_GetTick();
    static uint32_t lastRec = 0;
    uint32_t cursor = 0;

    while(cursor < dataSize)
    {
        uint32_t recLen = 0;

        if((timeout != HAL_MAX_DELAY) && (startTime + timeout < HAL_GetTick())) return HAL_TIMEOUT;
        if (lastRec == 0)
        {
            char hdr[20] = {0};
            uint32_t index = 0;
            uint32_t hdl = 0;

            while((hdr[index++] = (char)fgetc(__wifiRd)) != ':');
            hdr[index] = '\0';
            if((sscanf(&hdr[2], "+IPD,%lu,%lu:", &hdl, &recLen) != 2) || (hdl != handle))
            {
                retVal = HAL_ERROR;
                goto Cleanup;
            }
            lastRec = recLen;
        }
        else
        {
            recLen = lastRec;
        }
        lastRec -= MIN(recLen, dataSize - cursor);
        cursor += fread(data+cursor, 1, MIN(recLen, dataSize - cursor), __wifiRd);
    }

Cleanup:
    return retVal;
}

