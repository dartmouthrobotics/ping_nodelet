#ifndef PINGMESSAGE_H
#define PINGMESSAGE_H

#include <iostream>
#include <vector>

namespace ping_nodelet
{

/*! Class that handles packing and unpacking messages from the ping echo sounder */

class PingMessage
{
  public:
    char msg_data[10];          /**< packed message sent to UDP socket */
    float distance;      /**< distance recorded by ping echo sounder */
    unsigned int confidence;   /**< confidence recorded by ping echo sounder */

    PingMessage(){};
    PingMessage(unsigned int new_request_id);
    ~PingMessage(){};

    void pack_msg_data();                                 /**< pack messages sent to UDP socket */
    void unpack_msg_data(std::vector<unsigned char> buf); /**< unpack messages received from UDP socket */

  private:
    static const unsigned char start_1 = 'B';   /**< first byte of messages */
    static const unsigned char start_2 = 'R';   /**< second byte of messages */
    unsigned int message_id;                    /**< TO DO message id (not used) */
    unsigned int request_id;                    /**< request id; used in packing messages */
    short int payload_length;                   /**< TO DO payload length (should be used in packing messages) */
    unsigned char dst_device_id;                /**< TO DO dst device id (not used) */
    unsigned char src_device_id;                /**< TO DO src device id (not used) */
    short int checksum;                         /**< TO DO checksum (should be used in packing messages) */
};

}


#endif
