package inet_ext.applications;

simple NoApp like AppInterface
{
    gates:
        input rosIn @directIn;
        input lowerLayerIn;
        output lowerLayerOut;
}
