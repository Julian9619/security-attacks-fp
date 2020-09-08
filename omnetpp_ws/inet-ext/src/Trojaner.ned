package inet_ext;

import inet_ext.applications.AppInterface;
import inet_ext.physicallayer.CanTransciever;


module Trojaner
{
    parameters:
    gates:
        inout cang @labels(CanFrame-conn) @allowUnconnected;
    submodules:
        app: <> like AppInterface {
            parameters:
                @display("p=600,425");
        }
        can: CanTransciever {
            parameters:
                @display("p=600,525");
        }
    connections:
        app.lowerLayerOut --> can.upperLayerIn;
        can.upperLayerOut --> app.lowerLayerIn;

        can.phys <--> { @display("m=s"); } <--> cang;
}

