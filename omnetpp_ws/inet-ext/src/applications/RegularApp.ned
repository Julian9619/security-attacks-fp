package inet_ext.applications;

simple RegularApp like AppInterface
{
    parameters:
        double delay @unit(s) = default(10us);
        string rosModule = default("ros");
        @display("i=block/app");
        @class(RegularApp);
    gates:
        input rosIn @directIn;
        input lowerLayerIn;
        output lowerLayerOut;
}
