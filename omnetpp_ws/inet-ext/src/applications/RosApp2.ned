package inet_ext.applications;

simple RosApp2 like AppInterface
{
    parameters:
        string rosModule = default("ros");
        @display("i=block/app");
        @class(RosApp2);
    gates:
        input rosIn @directIn;
        input lowerLayerIn;
        output lowerLayerOut;
}
