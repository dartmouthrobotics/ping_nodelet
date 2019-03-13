#ifndef PINGDEPTH_H
#define PINGDEPTH_H

#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ping_nodelet/ping_parser.h"
#include "ping_nodelet/ping_message.h"
#include "ping_nodelet/Ping.h"

#define HOST "192.168.2.2"
#define PORT 9090

namespace ping_nodelet
{

/*!
  Class that handles sending and receiving messages from the ping echo sounder.
  Publishes distance and confidence data.
*/
class PingDepth
{
  public:
    /*!
      Receives the nodehandler and nodelet name from the nodelet initializer
    */
    PingDepth(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name) {};
    ~PingDepth(){};

    /*!
      Called from nodelet for initialization.
      TO DO:
        Divide function into smaller subfunctions.
    */
    bool init()
    {
      /*! Initialize parameters needed for socket */
      int sock;
      struct sockaddr_in raspi_addr;
      memset(&raspi_addr, '0', sizeof(raspi_addr));
      raspi_addr.sin_family = AF_INET;
      raspi_addr.sin_port = htons(9090);
      raspi_addr.sin_addr.s_addr = inet_addr(HOST);

      /*! Create UDP socket */
      if( (sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0 )
      {
        perror("Cannot create socket");
      }

      /*! Connect to the BlueROV2 Raspberry Pi */
      if( connect(sock, (struct sockaddr *)&raspi_addr, sizeof(raspi_addr)) < 0 )
      {
        perror("Connection failed");
      }

      /*! Create rostopic to publish distance and confidence */
      ros::Publisher pub = nh_.advertise<Ping>("/ping_nodelet/ping", 1000);
      Ping msg;

      /*! Initialize a parser to handle incoming messages */
      PingParser* parser = new PingParser();
      /*!
        Initialize message to handle packing and unpacking of data
        TO DO: initialize what kind of mesage to request
      */
      PingMessage* m = new PingMessage(PING1D_DISTANCE_SIMPLE);
      /*! pack message to be sent via UDP socket */
      m->pack_msg_data();

      while(ros::ok())
      {
        /*! send message to ping echo sounder via raspberry pi */
        send(sock, m->msg_data, (int)( sizeof(m->msg_data) / sizeof(m->msg_data[0]) ), 0);

        struct sockaddr_in address;
        char buffer[1024];
        socklen_t len = sizeof(address);

        /*! receive message from ping echo sounder via raspberry pi */
        int rec_bytes = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr*)&address, &len);

        if(rec_bytes > 0)
        {
          for(int i = 0; i < sizeof(buffer); i++)
          {
            /*! parse data */
            parser->parse_byte(buffer[i]);
          }

          /*!
              publish data
              TO DO: set frame_id
          */
          msg.header.stamp = ros::Time::now();
          msg.pinger_link = '0';
          msg.distance = parser->m->distance;
          msg.confidence = parser->m->confidence;
          pub.publish(msg);

        }
      }

      return true;
    }

  private:
    ros::NodeHandle nh_;  /**< node handler used for publishing ping data */
    std::string name_;    /**< name of the nodelet (not used) */

    /*!
      Different message ids
    */
    static const unsigned int PING1D_ACK = 1;
    static const unsigned int PING1D_ASCII_TEXT = 3;
    static const unsigned int PING1D_CONTINUOUS_START = 1400;
    static const unsigned int PING1D_CONTINUOUS_STOP = 1401;
    static const unsigned int PING1D_DEVICE_ID = 1201;
    static const unsigned int PING1D_DISTANCE = 1212;
    static const unsigned int PING1D_DISTANCE_SIMPLE = 1211;
    static const unsigned int PING1D_FIRMWARE_VERSION = 1200;
    static const unsigned int PING1D_GAIN_INDEX = 1207;
    static const unsigned int PING1D_GENERAL_INFO = 1210;
    static const unsigned int PING1D_GOTO_BOOTLOADER = 1100;
    static const unsigned int PING1D_MODE_AUTO = 1205;
    static const unsigned int PING1D_NACK = 2;
    static const unsigned int PING1D_PCB_TEMPERATURE = 1214;
    static const unsigned int PING1D_PING_ENABLE = 1215;
    static const unsigned int PING1D_PING_INTERVAL = 1206;
    static const unsigned int PING1D_PROCESSOR_TEMPERATURE = 1213;
    static const unsigned int PING1D_PROFILE = 1300;
    static const unsigned int PING1D_PROTOCOL_VERSION = 5;
    static const unsigned int PING1D_PULSE_DURATION = 1208;
    static const unsigned int PING1D_RANGE = 1204;
    static const unsigned int PING1D_SET_DEVICE_ID = 1000;
    static const unsigned int PING1D_SET_GAIN_INDEX = 1005;
    static const unsigned int PING1D_SET_MODE_AUTO = 1003;
    static const unsigned int PING1D_SET_PING_ENABLE = 1006;
    static const unsigned int PING1D_SET_PING_INTERVAL = 1004;
    static const unsigned int PING1D_SET_RANGE = 1001;
    static const unsigned int PING1D_SET_SPEED_OF_SOUND = 1002;
    static const unsigned int PING1D_SPEED_OF_SOUND = 1203;
    static const unsigned int PING1D_UNDEFINED = 0;
    static const unsigned int PING1D_VOLTAGE_5 = 1202;

};

}

#endif
