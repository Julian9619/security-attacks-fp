package inet_ext.linklayer;

import inet.linklayer.base.MacProtocolBase;
import inet.queueing.contract.IPacketQueue;

module CanController extends MacProtocolBase
{
    parameters:
        int identifier;
        @class(CanController);
    submodules:
        queue: <default("PacketQueue")> like IPacketQueue {
            parameters:
                @display("p=100,100;q=l2queue");
        }
}