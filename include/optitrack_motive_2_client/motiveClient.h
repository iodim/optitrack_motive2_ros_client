#ifndef MOTIONCAPTURECLIENTFRAMEWORK_H
#define MOTIONCAPTURECLIENTFRAMEWORK_H

#include <vector>
#include <iostream>
#include <linux/limits.h>
#include <boost/optional.hpp>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <cstring>

#include "NatNetDataTypes.h"

#define MULTICAST_ADDRESS       "239.255.42.99"
#define PORT_COMMAND            1510      // NatNet Command channel
#define PORT_DATA               1511      // NatNet Data channel

class motiveClient {

public:
    motiveClient(const std::string &, const std::string &);

    // Starts connection to mocap and initializes settings.

    bool isOK();

    uint64_t getServerFrequency();

    uint64_t getTimestamp();

    void requestDataDescriptions();

    void registerOnFrameCallback(const std::function<void(const sFrameOfMocapData&)> &);

    void registerOnDataDescriptionsCallback(const std::function<void(const sDataDescriptions&)> &);

private:
    // Sockets
    int commandSocket;
    int frameSocket;
    in_addr ServerAddress;
    sockaddr_in hostAddr;
    const char *my_address;
    const char *server_address;
    std::string multicast_address;

    // Versioning
    int NatNetVersion[4] = {3, 0, 0, 0};
    int ServerVersion[4] = {0, 0, 0, 0};

    // Command mode global variables
    int gCommandResponse = 0;
    int gCommandResponseSize = 0;
    unsigned char gCommandResponseString[PATH_MAX];

    // Instance vars
    uint64_t server_frequency = 0;
    bool ok_ = false;

    // Threads
    std::thread frameListenerThread;
    std::thread commandResponseThread;

    // Callbacks
    std::function<void(const sFrameOfMocapData&)> onFrameCallback;
    std::function<void(const sDataDescriptions&)> onDataDescriptionsCallback;

    // Thread listening to the data sockets.
    void dataListener();

    // Thread listening to the command responses. Commands are issued by sendCommand().
    // Currently only listening to data description and other misc packets (not frame packets)
    void commandResponseListener();

    bool initConnection();

    void handlePacket(char *);

    void setMulticastAddress (const std::string &);

    bool IPAddress_StringToAddr(char *, struct in_addr *) const;

    int createCommandSocket(in_addr_t, unsigned short);

    void decodeMarkerID(int, int *, int *);

    bool decodeTimecode(unsigned int, unsigned int, int *, int *, int *, int *, int *);

    // Takes timecode and assigns it to a string
    bool timecodeStringify(unsigned int, unsigned int, char *, size_t);

    void unpackFrameOfMocapData(char *, sFrameOfMocapData &);

    void unpackDataDescriptions(char *, sDataDescriptions &);

    int sendCommand(char *);

};


#endif // MOTIONCAPTURECLIENTFRAMEWORK_H
