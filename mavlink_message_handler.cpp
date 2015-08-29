#include "mavlink_message_handler.h"

#include "la-log.h"
#include <errno.h>

void MAVLink_Message_Handler::send_message_to_telem_forwarder(mavlink_message_t &msg)
{
  uint8_t sendbuf[512];

  uint16_t messageLen = mavlink_msg_to_send_buffer((uint8_t*)sendbuf,&msg);

  if (sendto(_fd_telem_forwarder, sendbuf, messageLen, 0, (struct sockaddr *)_sa_telemetry_forwarder, sizeof(struct sockaddr)) == -1) {
    la_log(LOG_INFO, "Failed sendto: %s", strerror(errno));
  }
}
