package inet_ext.applications;

simple BurstApp like AppInterface
{
    parameters:
        double delay @unit(s) = default(10us);
        string rosModule = default("ros");
        @display("i=block/app");
        @class(BurstApp);
    gates:
        input rosIn @directIn;
        input lowerLayerIn;
        output lowerLayerOut;
}
