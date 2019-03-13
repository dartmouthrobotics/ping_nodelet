#ifndef PINGPARSER_H
#define PINGPARSER_H

#include <string>
#include <iostream>
#include <vector>

#include "ping_nodelet/ping_message.h"

namespace ping_nodelet
{

/*!
  Class that handles parsing received messages from the UDP socket
*/
class PingParser
{
  public:

    PingMessage* m;   /**< message container for the data received */

    /*!
      Constructor that initializes the start of parsing new data
    */
    PingParser()
    {
      m = new PingMessage();
      state = WAIT_START;
      payload_length = 0;
      message_id = 0;
      errors = 0;
      parsed = 0;
      rx_msg = "";
    }
    ~PingParser(){};

    int parse_byte(char msg_byte);  /**< parses received message byte by byte */

  private:

    std::vector<unsigned char> buf; /**< the data received from the ping echo sounder in a vector of bytes */
    unsigned int state;             /**< curent state of the parser */
    unsigned int payload_length;    /**< payload length of the received message */
    unsigned int message_id;        /**< message id of the received message */
    unsigned int errors;            /**< TO DO errors from the received message (should be used in parsing) */
    unsigned int parsed;            /**< TO DO (should be used in the parser) */
    std::string rx_msg;             /**< TO DO (should be use) in the parser) */

    /*!
      Different state values during parsing
    */
    static const int NEW_MESSAGE = 0;
    static const int WAIT_START = 1;
    static const int WAIT_HEADER = 2;
    static const int WAIT_LENGTH_L = 3;
    static const int WAIT_LENGTH_H = 4;
    static const int WAIT_MSG_ID_L = 5;
    static const int WAIT_MSG_ID_H = 6;
    static const int WAIT_SRC_ID = 7;
    static const int WAIT_DST_ID = 8;
    static const int WAIT_PAYLOAD = 9;
    static const int WAIT_CHECKSUM_L = 10;
    static const int WAIT_CHECKSUM_H = 11;
};

}

#endif
