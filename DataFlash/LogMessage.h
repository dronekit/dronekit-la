#ifndef _LOG_MESSAGE_H
#define _LOG_MESSAGE_H

#ifndef PACKED
#define PACKED
#endif


// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

/*
  unfortunately these need to be macros because of a limitation of
  named member structure initialisation in g++
 */
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id


/*
  log structures common to all vehicle types
 */
struct PACKED log_Format {
    LOG_PACKET_HEADER
    uint8_t type;
    uint8_t length;
    char name[4];
    char format[16];
    char labels[64];
};


#endif
