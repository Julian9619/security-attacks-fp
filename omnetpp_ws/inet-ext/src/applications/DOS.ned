package inet_ext.applications;

simple DOS like AppInterface
{
    parameters:
        double delay @unit(s) = default(1us);
        string rosModule = default("ros");
        @display("i=block/app");
        @class(DOS);
    gates:
        input rosIn @directIn;
        input lowerLayerIn;
        output lowerLayerOut;
}
