package inet_ext.applications;

simple RosApp like AppInterface
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
