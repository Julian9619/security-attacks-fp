package inet_ext;

import inet_ext.applications.RosApp;
import inet_ext.linklayer.CanInterface;
import inet.node.base.LinkLayerNodeBase;

module CanHost extends LinkLayerNodeBase
{
    parameters:
    gates:
        inout cang @labels(CanFrame-conn) @allowUnconnected;
    submodules:
        app: RosApp {
            parameters:
                @display("p=600,425");
        }
        can: CanInterface {
            parameters:
                @display("p=600,525");
        }
    connections:
        app.lowerLayerOut --> can.upperLayerIn;
        can.upperLayerOut --> app.lowerLayerIn;

        can.phys <--> { @display("m=s"); } <--> cang;
}

