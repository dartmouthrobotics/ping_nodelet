#include "ping_nodelet/ping_message.h"

namespace ping_nodelet
{

/*!
  \param new_request_id for packing messages
*/
PingMessage::PingMessage(unsigned int new_request_id)
{
  request_id = new_request_id;
}

/*!
  Packs an empty message with the request id.
  Usually called before sensing a message via UDP socket.
  TO DO:
    Substitute hard code values with variables
*/
void PingMessage::pack_msg_data()
{
  msg_data[0] = start_1;                      /**< 'B' 1 byte */
  msg_data[1] = start_2;                      /**< 'R' 1 byte */
  msg_data[2] = '\0';                         /**< TO DO payload length (must be empty) 1/2 bytes */
  msg_data[3] = '\0';                         /**< TO DO payload length (must be empty) 2/2 bytes */
  msg_data[4] = (request_id) & 0xff;          /**< request id 1/2 bytes */
  msg_data[5] = ((request_id >> 8) & 0xff);   /**< request id 2/2 bytes */
  msg_data[6] = '\0';                         /**< TO DO src device id (must be empty) 1 byte */
  msg_data[7] = '\0';                         /**< dst device id 1 byte */
  msg_data[8] = 'S';                          /**< TO DO checksum (not clear how to get these values) 1/2 bytes */
  msg_data[9] = 0x01;                         /**< TO DO checksum (not clear how to get these values) 2/2 bytes */
}


/*
  Unpacks messages received from the UDP socket aka ping echo sounder.
  Usually called when parsing the incoming UDP message.
  Only works with reading in PING1D_DISTANCE_SIMPLE messages.
  @param buf vector of bytes read from the UDP socket.
  TO DO:
    Format reading data to be applicable to other message types
*/
void PingMessage::unpack_msg_data(std::vector<unsigned char> buf)
{
  unsigned char data[4]; /**< temp holder for distance */
  confidence = 0;

  unsigned int raw_dist;
  for(int i = 0; i < buf.size(); i++)
  {
    if ( (8 <= i)  and (i < 12))
    {
      data[i-8] = buf.at(i);
    }
    if (12 == i)
    {
      confidence = 0xff & buf.at(i);
    }
  }

  raw_dist = (unsigned int)(data[3] << 24 |
              data[2] << 16 |
              data[1] << 8 |
              data[0]);
  distance = raw_dist / 100.0;
}

}
