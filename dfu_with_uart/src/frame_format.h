#ifndef FRAME_FORMAT_H
#define FRAME_FORMAT_H

// include the necessary headers
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define FF_ESC_CHAR 0x7D
#define FF_SOF_CHAR 0xAA
#define FF_EOF_CHAR 0x55
#define FF_XOR_MASK 0x20

    typedef enum
    {
        FF_ERR_OK = 0,
        FF_ERR_INVALID_INPUT,
        FF_ERR_NO_MEMORY,
        FF_ERR_SOF_MISMATCH,
        FF_ERR_EOF_MISMATCH,
        FF_ERR_CRC_MISMATCH,
        FF_ERR_INVALID_CMD,
        FF_ERR_INVALID_SSOH,
        FF_ERR_INVALID_VER,
    } ff_err_t;

    typedef enum
    {
        FF_VER_1 = 1, /* Version 1 */
    } ff_ver_t;

    typedef enum
    {
        FF_SSOH_1 = 1, /* SSOH 1 -> tx msg in ff format */
        FF_SSOH_2,     /* SSOH 2 -> rx msg in ff format */
        FF_SSOH_3,     /* SSOH 3 -> error msg in ff format */
        FF_SSOH_4,     /* SSOH 4 -> ack msg in ff format */
    } ff_ssoh_t;

    typedef struct
    {
        uint8_t soh;
        uint8_t ver;
        uint16_t cmd;
        uint8_t ssoh;
        uint16_t len;
        void *buff;
        uint32_t crc;
        uint8_t id;
        uint8_t eot;
    } frame_frm_t;

#define FF_FRAME_POPULATE_V1_TX(cmd_val, len_val, buff_ptr)                                                            \
    {                                                                                                                  \
        .soh = 0xAA, .ver = 1, .cmd = (cmd_val), .ssoh = (FF_SSOH_1), .len = (len_val), .buff = (buff_ptr), .crc = 0,  \
        .id = 0, .eot = 0x55                                                                                           \
    }

#define FF_FRAME_POPULATE_V1_RX(cmd_val, len_val, buff_ptr, id)                                                        \
    {                                                                                                                  \
        .soh = 0xAA, .ver = 1, .cmd = (cmd_val), .ssoh = (FF_SSOH_2), .len = (len_val), .buff = (buff_ptr), .crc = 0,  \
        .id = (id), .eot = 0x55                                                                                        \
    }

#define FF_FRAME_POPULATE_V1_ERR(cmd_val, len_val, buff_ptr, id)                                                       \
    {                                                                                                                  \
        .soh = 0xAA, .ver = 1, .cmd = (cmd_val), .ssoh = (FF_SSOH_3), .len = (len_val), .buff = (buff_ptr), .crc = 0,  \
        .id = (id), .eot = 0x55                                                                                        \
    }

#define FF_FRAME_POPULATE_V1_ACK(cmd_val, frame_id)                                                                    \
    {                                                                                                                  \
        .soh = 0xAA, .ver = 1, .cmd = (cmd_val), .ssoh = (FF_SSOH_4), .len = 0, .buff = NULL, .crc = 0,                \
        .id = (frame_id), .eot = 0x55                                                                                  \
    }

    typedef struct
    {
        uint8_t *buff;
        uint16_t len;
    } ff_buff_t;

    ff_err_t ff_process_frame(frame_frm_t *frame, uint8_t *buff, uint16_t len);
    ff_buff_t *ff_create_frame(frame_frm_t *frame);
    void ff_free_frame(ff_buff_t *frame);

#ifdef __cplusplus
}
#endif

#endif // FRAME_FORMAT_H
