package inet_ext.physicallayer;

simple CanBus 
{
    parameters:
        double delay @unit(s) = default(1ns);
        double interFrameSpace @unit(s) = default(10us);
    gates:
        inout cang[];
}