package inet_ext.linklayer;

import inet_ext.applications.RosApp;
import inet_ext.linklayer.CanController;
import inet_ext.physicallayer.CanTransciever;
import inet.networklayer.common.InterfaceEntry;

module CanInterface extends InterfaceEntry
{
    parameters:
        @class(CanInterface);
        string interfaceTableModule;
        *.interfaceTableModule = default(absPath(interfaceTableModule));
        @display("i=block/ifcard");
    gates:
        input upperLayerIn;
        output upperLayerOut;
        inout phys;
    submodules:
        controller: CanController {
            parameters:
                @display("p=150,100");
        }
        transciever: CanTransciever {
            parameters:
                @display("p=150,200,row,150;q=txQueue");
        }
    connections:
        upperLayerIn --> controller.upperLayerIn;
        controller.upperLayerOut --> upperLayerOut;

        controller.lowerLayerOut --> transciever.upperLayerIn;
        transciever.upperLayerOut --> controller.lowerLayerIn;

        transciever.phys <--> { @display("m=s"); } <--> phys;
}

