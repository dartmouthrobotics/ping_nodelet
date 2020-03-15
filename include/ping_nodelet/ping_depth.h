// Copyright (c) 2020 Monika Roznere, RLab @ Dartmouth College

#ifndef PING_NODELET_PING_DEPTH_H
#define PING_NODELET_PING_DEPTH_H

#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>

#include "ping_nodelet/ping-parser.h"
#include "ping_nodelet/ping-message.h"
#include "ping_nodelet/ping-message-all.h"
#include "ping_nodelet/ping-message-ping1d.h"
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
    /*! Receives the nodehandler and nodelet name from the nodelet initializer. */
    PingDepth(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name) {}
    ~PingDepth() {}

    /*! Called from nodelet for initialization. */
    bool init()
    {
      /*! Initialize parameters needed for socket */
      int sock;
      struct sockaddr_in raspi_addr;
      memset(&raspi_addr, '0', sizeof(raspi_addr));
      raspi_addr.sin_family = AF_INET;
      raspi_addr.sin_port = htons(9090);
      raspi_addr.sin_addr.s_addr = inet_addr(HOST);

      struct sockaddr_in address;
      char buffer[512];
      socklen_t len = sizeof(address);

      /*! Create UDP socket */
      if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        perror("Cannot create socket");

      /*! Connect to the BlueROV2 Raspberry Pi */
      if (connect(sock, (struct sockaddr *)&raspi_addr, sizeof(raspi_addr)) < 0)
        perror("Connection failed");

      /*! Create rostopic to publish distance and confidence */
      ros::Publisher pubPing = nh_.advertise<Ping>("/ping_nodelet/ping", 1000);
      Ping rosMessage;

      /*!
        Initialize message to send as a request
        MESSAGE: change message type here
      */
      ping1d_distance_simple txMessage;
      txMessage.updateChecksum();

      /*! Initialize a parser to handle incoming messages */
      PingParser parser(512);

      while (ros::ok())
      {
        send(sock, txMessage.msgData, txMessage.msgDataLength(), 0);

        /*! Receive message from ping echosounder via raspberry pi */
        int rec_bytes = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr*)&address, &len);

        if (rec_bytes > 0)
        {
          /*! Parse data */
          for (uint32_t i = 0; i < profile_msg_length; i++)
              parser.parseByte(buffer[i]);

          /*! MESSAGE: change message type here */
          ping1d_distance_simple rxMessage(parser.rxMessage);

          /*!
            Publish data.
            MESSAGE: if message type changes, then change ROS message format. 
          */
          rosMessage.header.stamp = ros::Time::now();
          rosMessage.header.frame_id = "ping1d";
          rosMessage.pinger_link = '0';
          rosMessage.distance = rxMessage.distance() / 1000.0;
          rosMessage.confidence = rxMessage.confidence();
          pubPing.publish(rosMessage);
        }

        parser.reset();
      }
      return true;
    }

  private:
    ros::NodeHandle nh_;  /**< node handler used for publishing ping data */
    std::string name_;    /**< name of the nodelet (not used) */

    static const uint8_t header_length = 8;
    static const uint8_t checksum_length = 2;

    static const uint16_t profile_static_payload_length = 4+2+2+4+4+4+4+2;
    static const uint16_t profile_points = 10;
    static const uint16_t profile_payload_length = profile_static_payload_length + profile_points;
    static const uint32_t profile_msg_length = header_length + profile_payload_length + checksum_length;
};

}  // namespace ping_nodelet

#endif  // PING_NODELET_PING_DEPTH_H
