package inet_ext.physicallayer;

simple CanBus 
{
    parameters:
        double delay @unit(s) = default(1us);
        double interFrameSpace @unit(s) = default(100us);
    gates:
        inout cang[];
}