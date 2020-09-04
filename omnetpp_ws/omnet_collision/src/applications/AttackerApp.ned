package omnet_collision.applications;

import inet.applications.contract.IApp;


simple AtackerApp
{
    parameters:
        string rosModule = default("ros");
        @display("i=block/app");
        @class(RosApp);
    gates:
        input rosIn @directIn;
        input lowerLayerIn;
        output lowerLayerOut;
}
