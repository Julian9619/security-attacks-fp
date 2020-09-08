package inet_ext;

import inet.node.base.LinkLayerNodeBase;
import inet_ext.applications.AppInterface;
import inet_ext.linklayer.CanInterface;


module CanHost extends LinkLayerNodeBase
{
    parameters:
    gates:
        inout cang @labels(CanFrame-conn) @allowUnconnected;
    submodules:
        app: <> like AppInterface {
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

