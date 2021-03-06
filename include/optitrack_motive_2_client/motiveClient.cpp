#include "motiveClient.h"
#include "NatNetDataTypes.h"

motiveClient::motiveClient() = default;

motiveClient::motiveClient(const std::string &szMyIPAddress, const std::string &szServerIPAddress) {
    // Convert address std::string to c_str.
    my_address = szMyIPAddress.c_str();
    server_address = szServerIPAddress.c_str();

    // init connection
    ok_ = initConnection();
    if (ok_) {
        commandResponseThread = std::thread(&motiveClient::commandResponseListener, this);
        frameListenerThread = std::thread(&motiveClient::dataListener, this);
    }

}

void motiveClient::registerOnFrameCallback(const std::function<void(const sFrameOfMocapData &)> &callback) {
    onFrameCallback = callback;
}

void motiveClient::registerOnDataDescriptionsCallback(const std::function<void(const sDataDescriptions&)> &callback) {
    onDataDescriptionsCallback = callback;
}

bool motiveClient::initConnection() {
    const int optval = 0x100000;
    socklen_t optval_size = 4;

    in_addr myAddress{};
    in_addr multicastAddress{};

    int retval;

    multicastAddress.s_addr = inet_addr(MULTICAST_ADDRESS);
    std::cout << "Client: " << my_address << std::endl;
    std::cout << "Server: " << server_address << std::endl;
    std::cout <<"Multicast Group: " << MULTICAST_ADDRESS << std::endl;

    // Create "Command" socket
    unsigned short port = 8000;
    commandSocket = createCommandSocket(inet_addr(my_address), port);
    if (commandSocket == -1) { // error
        std::cerr << "Command socket creation error\n";
        return false;
    }
    else {
        // [optional] set to non-blocking
        //u_long iMode=1;
        //ioctlsocket(commandSocket,FIONBIO,&iMode);
        // set buffer
        setsockopt(commandSocket, SOL_SOCKET, SO_RCVBUF, (char *) &optval, 4);
        getsockopt(commandSocket, SOL_SOCKET, SO_RCVBUF, (char *) &optval, &optval_size);
        if (optval != 0x100000) {
          // err - actual size...
          DEBUG_MSG("[commandSocket] ReceiveBuffer size =" << optval);
        }
    }

    // Create a "Frame" socket
    frameSocket = socket(AF_INET, SOCK_DGRAM, 0);

    // allow multiple clients on same machine to use address/port
    int value = 1;
    retval = setsockopt(frameSocket, SOL_SOCKET, SO_REUSEADDR, (char *) &value, sizeof(value));

    if (retval == -1) {
        close(frameSocket);
        std::cerr << "Error while setting frameSocket options\n";
        return false;
    }

    struct sockaddr_in mySocketAddr{};
    memset(&mySocketAddr, 0, sizeof(mySocketAddr));
    mySocketAddr.sin_family = AF_INET;
    mySocketAddr.sin_port = htons(PORT_DATA);
    mySocketAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(frameSocket, (struct sockaddr *) &mySocketAddr, sizeof(struct sockaddr)) == -1) {
        std::cerr << "[PacketClient] bind failed\n";
        return false;
    }

    // join multicast group
    struct ip_mreq mreq{};
    mreq.imr_multiaddr = multicastAddress;
    mreq.imr_interface = myAddress;
    retval = setsockopt(frameSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq));

    if (retval == -1) {
        std::cerr << "[PacketClient] join failed\n";
        return false;
    }

    // create a 1MB buffer
    setsockopt(frameSocket, SOL_SOCKET, SO_RCVBUF, (char *) &optval, 4);
    getsockopt(frameSocket, SOL_SOCKET, SO_RCVBUF, (char *) &optval, &optval_size);
    if (optval != 0x100000) {
        DEBUG_MSG("[commandSocket] ReceiveBuffer size =" << optval);
    }

    // Server address for commands
    memset(&hostAddr, 0, sizeof(hostAddr));
    hostAddr.sin_family = AF_INET;
    hostAddr.sin_port = htons(PORT_COMMAND);
    hostAddr.sin_addr.s_addr = inet_addr(server_address);

    // send initial connect request
    sPacket packetOut{};
    packetOut.iMessage = NAT_CONNECT;
    packetOut.nDataBytes = 0;
    int nTries = 3;
    while (nTries--) {
        ssize_t iRet = sendto(commandSocket,
                              (char *) &packetOut,
                              4 + packetOut.nDataBytes,
                              0,
                              (sockaddr *) &hostAddr,
                              sizeof(hostAddr));

        if (iRet != -1){
            std::cout << "Connected to server!\n";
        }

        // Wait for server response.
        // This will contain the server tick frequency.
        char ip_as_str[INET_ADDRSTRLEN];
        ssize_t nDataBytesReceived;
        sockaddr_in srvSocketAddress{};
        sPacket packetIn{};
        socklen_t addr_len = sizeof(struct sockaddr);
        nDataBytesReceived = recvfrom(commandSocket,
                                      (char *) &packetIn,
                                      sizeof(sPacket),
                                      0,
                                      (struct sockaddr *) &srvSocketAddress,
                                      &addr_len);

        if ((nDataBytesReceived != 0) && (nDataBytesReceived != -1)) {
            // debug - print message
            inet_ntop(AF_INET, &(srvSocketAddress.sin_addr), ip_as_str, INET_ADDRSTRLEN);

            DEBUG_MSG("[Client] Received command from " << ip_as_str << ": Command="
                << (int) packetIn.iMessage << ", nDataBytes=" << (int) packetIn.nDataBytes);

            auto ptr = (unsigned char *) &packetIn;
            auto server_info = (sSender_Server *) (ptr + 4);

            DEBUG_MSG("server tick frequency: " << server_info->HighResClockFrequency);
            server_frequency = server_info->HighResClockFrequency;
            // Done processing server response.

            return true;
        }
    }
    std::cerr << "Number of tries exceeded -- shutting down.\n";
    return false;
}

void motiveClient::dataListener() {
    char szData[MAX_PACKETSIZE];
    socklen_t addr_len = sizeof(sockaddr);
    sockaddr_in TheirAddress{};

    while (isOK()) {
        recvfrom(frameSocket, szData, sizeof(szData), 0, (sockaddr *) &TheirAddress, &addr_len);
        handlePacket(szData);
    }
}

void motiveClient::handlePacket(char *ptr) {
    auto packet = (sPacket *) ptr;

    switch (packet->iMessage) {
        case NAT_FRAMEOFDATA: {
            sFrameOfMocapData frame;
            unpackFrameOfMocapData(ptr + 4, frame);
            bool models_changed = (frame.params & 0x02) != 0;
            if (models_changed){
                requestDataDescriptions();
                DEBUG_MSG("Models changed!");
            }
            if (onFrameCallback) {
                onFrameCallback(frame);
            }
            break;
        }
        case NAT_MODELDEF: {
            sDataDescriptions descriptions;
            unpackDataDescriptions(ptr + 4, descriptions);
            if (onDataDescriptionsCallback) {
                onDataDescriptionsCallback(descriptions);
            }
            break;
        }
        case NAT_SERVERINFO: {
            auto *server_info = (sSender_Server *) (ptr + 4);

            // Streaming app's name, e.g., Motive
            std::cout << server_info->Common.szName << ' ';

            // Streaming app's version, e.g., 2.0.0.0
            for (const auto &digit: server_info->Common.Version) {
                std::cout << static_cast<int>(digit) << '.';
            }

            std::cout << '\b' << std::endl;

            // Streaming app's NatNet version, e.g., 3.0.0.0
            std::cout << "NatNet ";
            for (const auto &digit: server_info->Common.NatNetVersion) {
                std::cout << static_cast<int>(digit) << '.';
            }
            std::cout << '\b' << std::endl;

            // Save versions in instance variables
            for (int i = 0; i < 4; i++) {
                NatNetVersion[i] = server_info->Common.NatNetVersion[i];
                ServerVersion[i] = server_info->Common.Version[i];
            }
            break;
        }
        case NAT_RESPONSE: {
            gCommandResponseSize = packet->nDataBytes;

            if (gCommandResponseSize == 4) {
                memcpy(&gCommandResponse, &packet->Data.lData[0], gCommandResponseSize);
            } else {
                memcpy(&gCommandResponseString[0], &packet->Data.cData[0], gCommandResponseSize);
                DEBUG_MSG("Response : " << gCommandResponseString);
                gCommandResponse = 0;   // ok
            }
            break;
        }
        case NAT_UNRECOGNIZED_REQUEST: {
            DEBUG_MSG("[Client] received 'unrecognized request'");
            gCommandResponseSize = 0;
            gCommandResponse = 1;       // err
            break;
        }
        case NAT_MESSAGESTRING: {
            DEBUG_MSG("[Client] Received message: " << packet->Data.szData << std::endl);
            break;
        }
        default: {
            break;
        }
    }
}



// ============================== Data mode ================================ //
// Funtion that assigns a time code values to 5 variables passed as arguments
// Requires an integer from the packet as the timecode and timecodeSubframe
bool motiveClient::decodeTimecode(
            unsigned int inTimecode,
            unsigned int inTimecodeSubframe,
            int *hour,
            int *minute,
            int *second,
            int *frame,
            int *subframe) {
    bool bValid = true;

    *hour = (inTimecode >> 24) & 255;
    *minute = (inTimecode >> 16) & 255;
    *second = (inTimecode >> 8) & 255;
    *frame = inTimecode & 255;
    *subframe = inTimecodeSubframe;

    return bValid;
}

// Takes timecode and assigns it to a string
bool motiveClient::timecodeStringify(
            unsigned int inTimecode,
            unsigned int inTimecodeSubframe,
            char *Buffer,
            size_t BufferSize) {
    bool bValid;
    int hour, minute, second, frame, subframe;
    bValid = decodeTimecode(inTimecode,
                            inTimecodeSubframe,
                            &hour,
                            &minute,
                            &second,
                            &frame,
                            &subframe);

    snprintf(Buffer, BufferSize, "%2d:%2d:%2d:%2d.%d",
             hour, minute, second, frame, subframe);
    for (unsigned int i = 0; i < strlen(Buffer); i++) {
        if (Buffer[i] == ' ') {
            Buffer[i] = '0';
        }
    }

    return bValid;
}

void motiveClient::decodeMarkerID(int sourceID, int *pOutEntityID, int *pOutMemberID) {
    if (pOutEntityID)
        *pOutEntityID = sourceID >> 16;

    if (pOutMemberID)
        *pOutMemberID = sourceID & 0x0000ffff;
}

// ============================= Command mode ============================== //
// Send a command to Motive.
int motiveClient::sendCommand(char *szCommand) {
    // reset global result
    gCommandResponse = -1;

    // format command packet
    sPacket commandPacket{};
    strcpy(commandPacket.Data.szData, szCommand);
    commandPacket.iMessage = NAT_REQUEST;
    commandPacket.nDataBytes = (unsigned short) (strlen(commandPacket.Data.szData) + 1);

    // send command and wait (a bit)
    // for command response to set global response var in CommandListenThread
    ssize_t iRet = sendto(commandSocket,
                        (char *) &commandPacket,
                        4 + commandPacket.nDataBytes,
                        0,
                        (sockaddr *) &hostAddr,
                        sizeof(hostAddr));
    if (iRet == -1) {
        printf("Socket error sending command");
    }
    else {
        int waitTries = 5;
        while (waitTries--) {
            if (gCommandResponse != -1)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        if (gCommandResponse == -1)
            printf("Command response not received (timeout)");
        else if (gCommandResponse == 0)
            printf("Command response received with success");
        else if (gCommandResponse > 0)
            printf("Command response received with errors");
    }
    return gCommandResponse;
}

void motiveClient::unpackFrameOfMocapData(char *ptr, sFrameOfMocapData &frame) {
    // Checks for NatNet Version number. Used later in function.
    // Packets may be different depending on NatNet version.
    int major = NatNetVersion[0];
    int minor = NatNetVersion[1];

    // Next 4 Bytes is the frame number
    memcpy(&frame.iFrame, ptr, 4);
    ptr += 4;

    // Next 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
    int nMarkerSets = 0;
    memcpy(&frame.nMarkerSets, ptr, 4);
    ptr += 4;

    // Loop through number of marker sets and get name and data
    for (int i = 0; i < frame.nMarkerSets; i++) {
        // Markerset name
        strcpy(frame.MocapData[i].szName, ptr);
        size_t nDataBytes = strlen(frame.MocapData[i].szName) + 1;
        ptr += nDataBytes;


        // sMarker data
        memcpy(&frame.MocapData[i].nMarkers, ptr, 4);
        ptr += 4;
        frame.MocapData[i].Markers = new MarkerData[frame.MocapData[i].nMarkers];

        // Loop through every Marker(x, y, z) contained in the current MarketSet
        for (int j = 0; j < frame.MocapData[i].nMarkers; j++) {
            memcpy(&frame.MocapData[i].Markers[j][0], ptr, 4);
            ptr += 4;
            memcpy(&frame.MocapData[i].Markers[j][1], ptr, 4);
            ptr += 4;
            memcpy(&frame.MocapData[i].Markers[j][2], ptr, 4);
            ptr += 4;
        }
    }

    // Loop through unlabeled markers
    memcpy(&frame.nOtherMarkers, ptr, 4);
    ptr += 4;

    // OtherMarker list is Deprecated
    for (int i = 0; i < frame.nOtherMarkers; i++) {
        memcpy(&frame.OtherMarkers[i][0], ptr, 4);
        ptr += 4;
        memcpy(&frame.OtherMarkers[i][1], ptr, 4);
        ptr += 4;
        memcpy(&frame.OtherMarkers[i][2], ptr, 4);
        ptr += 4;
    }

    // Loop through rigidbodies
    memcpy(&frame.nRigidBodies, ptr, 4);
    ptr += 4;

    for (int i = 0; i < frame.nRigidBodies; i++) {
        // Rigid body position and orientation
        memcpy(&frame.RigidBodies[i].ID, ptr, 4);
        ptr += 4;
        memcpy(&frame.RigidBodies[i].x, ptr, 4);
        ptr += 4;
        memcpy(&frame.RigidBodies[i].y, ptr, 4);
        ptr += 4;
        memcpy(&frame.RigidBodies[i].z, ptr, 4);
        ptr += 4;
        memcpy(&frame.RigidBodies[i].qx, ptr, 4);
        ptr += 4;
        memcpy(&frame.RigidBodies[i].qy, ptr, 4);
        ptr += 4;
        memcpy(&frame.RigidBodies[i].qz, ptr, 4);
        ptr += 4;
        memcpy(&frame.RigidBodies[i].qw, ptr, 4);
        ptr += 4;

        // NatNet version 2.0 and later
        if (major >= 2) {
            // Mean marker error
            memcpy(&frame.RigidBodies[i].MeanError, ptr, 4);
            ptr += 4;
        }

        // NatNet version 2.6 and later
        if (((major == 2) && (minor >= 6)) || (major > 2)) {
            // params
            memcpy(&frame.RigidBodies[i].params, ptr, 2);
            ptr += 2;
            // 0x01 : rigid body was successfully tracked in this frame
            bool bTrackingValid = frame.RigidBodies[i].params & 0x01;
        }
    } // Go to next rigid body

    // Skeletons (NatNet version 2.1 and later)
    if (((major == 2) && (minor > 0)) || (major > 2)) {
        memcpy(&frame.nSkeletons, ptr, 4);
        ptr += 4;
        // printf("Skeleton Count : %d\n", nSkeletons);

        // Loop through skeletons
        for (int i = 0; i < frame.nSkeletons; i++) {
            // Skeleton ID
            memcpy(&frame.Skeletons[i].skeletonID, ptr, 4);
            ptr += 4;

            // Number of rigid bodies (bones) in skeleton
            memcpy(&frame.Skeletons[i].nRigidBodies, ptr, 4);
            ptr += 4;
            // printf("Rigid Body Count : %d\n", nRigidBodies);

            // Loop through rigid bodies (bones) in skeleton
            for (int j = 0; j < frame.Skeletons[i].nRigidBodies; j++) {
                // Rigid body position and orientation
                memcpy(&frame.Skeletons[i].RigidBodyData[j].ID, ptr, 4);
                ptr += 4;
                memcpy(&frame.Skeletons[i].RigidBodyData[j].x, ptr, 4);
                ptr += 4;
                memcpy(&frame.Skeletons[i].RigidBodyData[j].y, ptr, 4);
                ptr += 4;
                memcpy(&frame.Skeletons[i].RigidBodyData[j].z, ptr, 4);
                ptr += 4;
                memcpy(&frame.Skeletons[i].RigidBodyData[j].qx, ptr, 4);
                ptr += 4;
                memcpy(&frame.Skeletons[i].RigidBodyData[j].qy, ptr, 4);
                ptr += 4;
                memcpy(&frame.Skeletons[i].RigidBodyData[j].qz, ptr, 4);
                ptr += 4;
                memcpy(&frame.Skeletons[i].RigidBodyData[j].qw, ptr, 4);
                ptr += 4;

                // Mean marker error (NatNet version 2.0 and later)
                if (major >= 2) {
                    memcpy(&frame.Skeletons[i].RigidBodyData[j].MeanError, ptr, 4);
                    ptr += 4;
                }

                // Tracking flags (NatNet version 2.6 and later)
                if (((major == 2) && (minor >= 6)) || (major > 2)) {
                    // params
                    memcpy(&frame.Skeletons[i].RigidBodyData[j].params, ptr, 2);
                    ptr += 2;
                    // 0x01 : rigid body was successfully tracked in this frame
                    bool bTrackingValid = frame.Skeletons[i].RigidBodyData[j].params & 0x01;
                }
            } // Next rigid body
        } // Next skeleton
    }

    // labeled markers (NatNet version 2.3 and later)
    if (((major == 2) && (minor >= 3)) || (major > 2)) {
        memcpy(&frame.nLabeledMarkers, ptr, 4);
        ptr += 4;
//            printf("Labeled sMarker Count : %d\n", nLabeledMarkers);

        // Loop through labeled markers
        for (int i = 0; i < frame.nLabeledMarkers; i++) {

            memcpy(&frame.LabeledMarkers[i].ID, ptr, 4);
            ptr += 4;
            int modelID, markerID;
            decodeMarkerID(frame.LabeledMarkers[i].ID, &modelID, &markerID);

            // x
            float x = 0.0f;
            memcpy(&frame.LabeledMarkers[i].x, ptr, 4);
            ptr += 4;
            // y
            float y = 0.0f;
            memcpy(&frame.LabeledMarkers[i].y, ptr, 4);
            ptr += 4;
            // z
            float z = 0.0f;
            memcpy(&frame.LabeledMarkers[i].z, ptr, 4);
            ptr += 4;
            // size
            float size = 0.0f;
            memcpy(&frame.LabeledMarkers[i].size, ptr, 4);
            ptr += 4;

            // NatNet version 2.6 and later
            if (((major == 2) && (minor >= 6)) || (major > 2)) {
                // marker params
                memcpy(&frame.LabeledMarkers[i].params, ptr, 2);
                ptr += 2;

                // marker was not visible (occluded) in this frame
                bool bOccluded = (frame.LabeledMarkers[i].params & 0x01) != 0;

                // position provided by point cloud solve
                bool bPCSolved = (frame.LabeledMarkers[i].params & 0x02) != 0;

                // position provided by model solve
                bool bModelSolved = (frame.LabeledMarkers[i].params & 0x04) != 0;

                if (major >= 3) {
                    // marker has an associated model
                    bool bHasModel = (frame.LabeledMarkers[i].params & 0x08) != 0;

                    // marker is an unlabeled marker
                    bool bUnlabeled = (frame.LabeledMarkers[i].params & 0x10) != 0;

                    // marker is an active marker
                    bool bActiveMarker = (frame.LabeledMarkers[i].params & 0x20) != 0;
                }
            }

            // NatNet version 3.0 and later
            float residual = 0.0f;
            if (major >= 3) {
                // sMarker residual
                memcpy(&frame.LabeledMarkers[i].residual, ptr, 4);
                ptr += 4;
            }
        }
    }

    // Force Plate data (NatNet version 2.9 and later)
    if (((major == 2) && (minor >= 9)) || (major > 2)) {
        memcpy(&frame.nForcePlates, ptr, 4);
        ptr += 4;
        for (int i = 0; i < frame.nForcePlates; i++) {
            // ID
            memcpy(&frame.ForcePlates[i].ID, ptr, 4);
            ptr += 4;
            // printf("Force Plate : %d\n", ID);

            // Channel Count
            memcpy(&frame.ForcePlates[i].nChannels, ptr, 4);
            ptr += 4;

            // Channel Data
            for (int j = 0; j < frame.ForcePlates[i].nChannels; j++) {
                // printf(" Channel %d : ", i);
                memcpy(&frame.ForcePlates[i].ChannelData[j].nFrames, ptr, 4);
                ptr += 4;

                for (int k = 0; k < frame.ForcePlates[i].ChannelData[j].nFrames; k++) {
                    memcpy(&frame.ForcePlates[i].ChannelData[j].Values[k], ptr, 4);
                    ptr += 4;
                }
            }
        }
    }

    // Device data (NatNet version 3.0 and later)
    if (((major == 2) && (minor >= 11)) || (major > 2)) {
        int nDevices;
        memcpy(&frame.nDevices, ptr, 4);
        ptr += 4;
        for (int i = 0; i < frame.nDevices; i++) {
            // ID
            memcpy(&frame.Devices[i].ID, ptr, 4);
            ptr += 4;
            // printf("Device : %d\n", ID);

            // Channel Count
            memcpy(&frame.Devices[i].nChannels, ptr, 4);
            ptr += 4;

            // Channel Data
            for (int j = 0; j < frame.Devices[i].nChannels; j++) {
                // printf(" Channel %d : ", i);
                memcpy(&frame.Devices[i].ChannelData[j].nFrames, ptr, 4);
                ptr += 4;
                for (int k = 0; k < frame.Devices[i].ChannelData[j].nFrames; k++) {
                    memcpy(&frame.Devices[i].ChannelData[j].Values[k], ptr, 4);
                    ptr += 4;
                }
            }
        }
    }

    // software latency (removed in version 3.0)
    if (major < 3) {
        float softwareLatency = 0.0f;
        memcpy(&softwareLatency, ptr, 4);
        ptr += 4;
        printf("software latency : %3.3f\n", softwareLatency);
    }

    // timecode
    unsigned int timecode = 0;
    memcpy(&frame.Timecode, ptr, 4);
    ptr += 4;
    unsigned int timecodeSub = 0;
    memcpy(&frame.TimecodeSubframe, ptr, 4);
    ptr += 4;
    char szTimecode[128] = "";
    timecodeStringify(timecode, timecodeSub, szTimecode, 128);

    // NatNet version 2.7 and later - increased from single to double precision
    if (((major == 2) && (minor >= 7)) || (major > 2)) {
        memcpy(&frame.fTimestamp, ptr, 8);
        ptr += 8;
    }
    else {
        float fTemp = 0.0f;
        memcpy(&fTemp, ptr, 4);
        ptr += 4;
        frame.fTimestamp = (double) fTemp;
    }

    // high res timestamps (version 3.0 and later)
    if (major >= 3) {
        memcpy(&frame.CameraMidExposureTimestamp, ptr, 8);
        ptr += 8;

        memcpy(&frame.CameraDataReceivedTimestamp, ptr, 8);
        ptr += 8;

        uint64_t transmitTimestamp = 0;
        memcpy(&frame.TransmitTimestamp, ptr, 8);
        ptr += 8;
    }

    // frame params
    memcpy(&frame.params, ptr, 2);
    ptr += 2;

    // 0x01 Motive is recording
    bool bIsRecording = (frame.params & 0x01) != 0;

    // 0x02 Actively tracked model list has changed
    bool bTrackedModelsChanged = (frame.params & 0x02) != 0;

    // end of data tag
    int eod = 0;
    memcpy(&eod, ptr, 4);
    ptr += 4;
}

void motiveClient::requestDataDescriptions() {
    sPacket packet;
    packet.iMessage = NAT_REQUEST_MODELDEF;
    packet.nDataBytes = 0;

    int tries = 3;
    while (tries--) {
        ssize_t nDataBytesReceived = sendto(commandSocket, (char*) &packet, 4 + packet.nDataBytes, 0, (sockaddr *) &hostAddr, sizeof(hostAddr));
        if ((nDataBytesReceived != 0) && (nDataBytesReceived != -1)) {

            break;
        }
    }
}

void motiveClient::unpackDataDescriptions(char *ptr, sDataDescriptions &descriptions) {
    // Checks for NatNet Version number. Used later in function.
    // Packets may be different depending on NatNet version.
    int major = NatNetVersion[0];
    int minor = NatNetVersion[1];

    DEBUG_MSG("\nBegin descriptions packet");
    DEBUG_MSG("==========================");

    memcpy(&descriptions.nDataDescriptions, ptr, 4);
    ptr += 4;
    DEBUG_MSG("Dataset Count: " << descriptions.nDataDescriptions);

    for (int i = 0; i < descriptions.nDataDescriptions; i++) {
        DEBUG_MSG("Dataset: " << i);

        memcpy(&descriptions.arrDataDescriptions[i].type, ptr, 4);
        ptr += 4;
        DEBUG_MSG("Type: " << descriptions.arrDataDescriptions[i].type);
        descriptions.arrDataDescriptions[i].initData();

        if (descriptions.arrDataDescriptions[i].type == Descriptor_MarkerSet) { // markerset
            // name
            strcpy(descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->szName, ptr);
            size_t nDataBytes = strlen(descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->szName) + 1;
            ptr += nDataBytes;
            DEBUG_MSG("Markerset Name: " << descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->szName);

            // marker data
            memcpy(&descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->nMarkers, ptr, 4);
            ptr += 4;
            DEBUG_MSG("sMarker Count: " << descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->nMarkers);

            descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->szMarkerNames =
                    new char*[descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->nMarkers];
            for (int j = 0; j < descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->nMarkers; j++) {
                char str[MAX_NAMELENGTH];
                strcpy(str, ptr);
                size_t tess = strlen(str) + 1;
                descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->szMarkerNames[j] = new char[tess];
                strcpy(descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->szMarkerNames[j], ptr);
                ptr += tess;
                DEBUG_MSG("sMarker Name: " << descriptions.arrDataDescriptions[i].Data.MarkerSetDescription->szMarkerNames[j]);
            }
        }
        else if (descriptions.arrDataDescriptions[i].type == Descriptor_RigidBody) { // rigid body
            if (major >= 2) {
                // name
                strcpy(descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->szName, ptr);
                ptr += strlen(descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->szName) + 1;
                DEBUG_MSG("Name:: " << descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->szName);
            }

            memcpy(&descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->ID, ptr, 4);
            ptr += 4;
            DEBUG_MSG("ID: " << descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->ID);

            memcpy(&descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->parentID, ptr, 4);
            ptr += 4;
            DEBUG_MSG("Parent ID: " << descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->parentID);

            memcpy(&descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->offsetx, ptr, 4);
            ptr += 4;
            DEBUG_MSG("X Offset: " << descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->offsetx);

            memcpy(&descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->offsety, ptr, 4);
            ptr += 4;
            DEBUG_MSG("Y Offset: " << descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->offsety);

            memcpy(&descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->offsetz, ptr, 4);
            ptr += 4;
            DEBUG_MSG("Z Offset: " << descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->offsetz);

            // Per-marker data (NatNet 3.0 and later)
            if (major >= 3) {
                int nMarkers = 0;
                memcpy(&nMarkers, ptr, 4);
                ptr += 4;
                descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->nMarkers = nMarkers;

                // sMarker positions
                int nBytes = nMarkers * 3 * sizeof(float);
                descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->MarkerPositions = (MarkerData *) malloc(nBytes);
                memcpy(descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->MarkerPositions, ptr, nBytes);
                ptr += nBytes;

                // sMarker required active labels
                nBytes = nMarkers * sizeof(int);
                descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->MarkerRequiredLabels = (int *) malloc(nBytes);
                memcpy(descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->MarkerRequiredLabels, ptr, nBytes);
                ptr += nBytes;

                for (int j = 0; j < nMarkers; j++) {
                    DEBUG_MSG("\tsMarker #" << j);
                    DEBUG_MSG("\t\tPosition: " <<
                           descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->MarkerPositions[j][0] <<
                           descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->MarkerPositions[j][1] <<
                           descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->MarkerPositions[j][2]
                    );
                    if (descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->MarkerRequiredLabels[j] != 0) {
                        DEBUG_MSG("\t\tRequired active label:: " <<
                               descriptions.arrDataDescriptions[i].Data.RigidBodyDescription->MarkerRequiredLabels[j]
                        );
                    }
                }
            }
        }
        else if (descriptions.arrDataDescriptions[i].type == Descriptor_Skeleton) { // skeleton
            strcpy(descriptions.arrDataDescriptions[i].Data.SkeletonDescription->szName, ptr);
            ptr += strlen(descriptions.arrDataDescriptions[i].Data.SkeletonDescription->szName) + 1;
            DEBUG_MSG("Name: " << descriptions.arrDataDescriptions[i].Data.SkeletonDescription->szName);

            memcpy(&descriptions.arrDataDescriptions[i].Data.SkeletonDescription->skeletonID, ptr, 4);
            ptr += 4;
            DEBUG_MSG("ID: " << descriptions.arrDataDescriptions[i].Data.SkeletonDescription->skeletonID);

            memcpy(&descriptions.arrDataDescriptions[i].Data.SkeletonDescription->nRigidBodies, ptr, 4);
            ptr += 4;
            DEBUG_MSG("RigidBody (Bone) Count: " <<
                   descriptions.arrDataDescriptions[i].Data.SkeletonDescription->nRigidBodies
            );

            for (int j = 0; j < descriptions.arrDataDescriptions[i].Data.SkeletonDescription->nRigidBodies; j++) {
                if (major >= 2) {
                    // RB name
                    strcpy(descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].szName, ptr);
                    ptr += strlen(descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].szName) + 1;
                    DEBUG_MSG("Rigid Body Name: " <<
                           descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].szName
                    );
                }

                memcpy(&descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].ID, ptr, 4);
                ptr += 4;
                DEBUG_MSG("RigidBody ID: " <<
                       descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].ID
                );

                memcpy(&descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].parentID, ptr, 4);
                ptr += 4;
                DEBUG_MSG("Parent ID: " <<
                       descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].parentID
                );

                memcpy(&descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].offsetx, ptr, 4);
                ptr += 4;
                DEBUG_MSG("X Offset: " <<
                       descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].offsetx
                );

                memcpy(&descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].offsety, ptr, 4);
                ptr += 4;
                DEBUG_MSG("Y Offset: " <<
                       descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].offsety
                );

                memcpy(&descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].offsetz, ptr, 4);
                ptr += 4;
                DEBUG_MSG("Z Offset: " <<
                       descriptions.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].offsetz
                );
            } // next rigid body
        } // next skeleton
    }  // next dataset
    DEBUG_MSG("==========================");
    DEBUG_MSG("End descriptions packet\n");
}

int motiveClient::createCommandSocket(in_addr_t IP_Address, unsigned short uPort) {
    struct sockaddr_in my_addr{};
    static unsigned long ivalue;
    static unsigned long bFlag;
    int nlengthofsztemp = 64;
    int sockfd;

    // Create a blocking, datagram socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        return -1;
    }

    // bind socket
    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(uPort);
    my_addr.sin_addr.s_addr = IP_Address;
    if (bind(sockfd, (struct sockaddr *) &my_addr, sizeof(struct sockaddr)) == -1) {
        close(sockfd);
        return -1;
    }

    // set to broadcast mode
    ivalue = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (char *) &ivalue, sizeof(ivalue)) == -1) {
        close(sockfd);
        return -1;
    }

    return sockfd;
}

// Command response listener thread
void motiveClient::commandResponseListener() {
    char ip_as_str[INET_ADDRSTRLEN];
    ssize_t nDataBytesReceived;
    sockaddr_in srvSocketAddress{};
    sPacket packetIn{};
    socklen_t addr_len = sizeof(struct sockaddr);

    while (isOK()) {
        // blocking
        nDataBytesReceived = recvfrom(commandSocket, (char *) &packetIn, sizeof(sPacket), 0, (sockaddr *) &srvSocketAddress, &addr_len);

        if ((nDataBytesReceived == 0) || (nDataBytesReceived == -1))
          continue;

        // debug - print message
        inet_ntop(AF_INET, &(srvSocketAddress.sin_addr), ip_as_str, INET_ADDRSTRLEN);
        DEBUG_MSG("[Client] Received command from " << ip_as_str << ": Command="
             << (int) packetIn.iMessage <<  ", nDataBytes=" << (int) packetIn.nDataBytes);

        auto ptr = (char *) &packetIn;
        handlePacket(ptr);
    }
}

// Convert IP address string to address
bool motiveClient::IPAddress_StringToAddr(
        char *szNameOrAddress,
        struct in_addr *Address) const {
    struct sockaddr_in saGNI{};
    char hostName[256];
    char servInfo[256];
    u_short port;
    port = 0;

    // Set up sockaddr_in structure which is passed to the getnameinfo function
    saGNI.sin_family = AF_INET;
    saGNI.sin_addr.s_addr = inet_addr(szNameOrAddress);
    saGNI.sin_port = htons(port);

    // getnameinfo in WS2tcpip is protocol independent
    // and resolves address to ANSI host name
    int retVal = getnameinfo((sockaddr *) &saGNI, sizeof(sockaddr), hostName, 256, servInfo, 256, NI_NUMERICSERV);
    if (retVal != 0) {
        // Returns error if getnameinfo failed
        printf("[PacketClient] GetHostByAddr failed\n");
        return false;
    }

    Address->s_addr = saGNI.sin_addr.s_addr;
    return true;
}

bool motiveClient::isOK() {
    return ok_;
}

uint64_t motiveClient::getServerFrequency() {
    return server_frequency;
}

void motiveClient::setMulticastAddress (const std::string &address) {
    multicast_address = address;
}
