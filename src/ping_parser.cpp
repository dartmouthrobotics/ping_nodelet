#include "ping_nodelet/ping_parser.h"

namespace ping_nodelet
{

int PingParser::parse_byte(char msg_byte)
{
  // convert char to int
  unsigned int msg = msg_byte;

  if (state == WAIT_START)
  {
    buf.clear();
    if (msg == int('B'))
    {
      buf.push_back(msg);
      state++;
    }
  }
  else if (state == WAIT_HEADER)
  {
    if (msg == int('R'))
    {
      buf.push_back(msg);
      state++;
    }
    else
    {
      state = WAIT_START;
    }
  }
  else if (state == WAIT_LENGTH_L)
  {
    payload_length = msg;
    buf.push_back(msg);
    state++;
  }
  else if (state == WAIT_LENGTH_H)
  {
    payload_length = (msg << 8) | payload_length;
    buf.push_back(msg);
    state++;
  }
  else if (state == WAIT_MSG_ID_L)
  {
    message_id = msg;
    buf.push_back(msg);
    state++;
  }
  else if (state == WAIT_MSG_ID_H)
  {
    message_id = (msg << 8) | message_id;
    buf.push_back(msg);
    state++;
  }
  else if (state == WAIT_SRC_ID)
  {
    buf.push_back(msg);
    state++;
  }
  else if (state == WAIT_DST_ID)
  {
    buf.push_back(msg);
    state++;
    if (payload_length == 0)  // no payload bytes
    {
      state++;
    }
  }
  else if (state == WAIT_PAYLOAD)
  {
    buf.push_back(msg);
    payload_length--;
    if (payload_length == 0)
    {
      state++;
    }
  }
  else if (state == WAIT_CHECKSUM_L)
  {
    buf.push_back(msg);
    state++;
  }
  else if (state == WAIT_CHECKSUM_H)
  {
    buf.push_back(msg);
    m->unpack_msg_data(buf);

    state = WAIT_START;
    payload_length = 0;
    message_id = 0;

    /*!
      TO DO more work in parsing the message
    */
    // if (rx_msg->verify_checksum())
    // {
    //   parsed++;
    //   return NEW_MESSAGE;
    // }
    // else
    // {
    //   errors++;
    // }
  }
  return state;
}

}
