package omnet_collision.simulations;

import ned.DatarateChannel;
import omnet_collision.CanHost;
import omnet_collision.CanHost2;
import omnet_collision.physicallayer.CanBus;
import omnet_collision.ros.RosInterface;


network CanNet
{
    types:
        channel C extends DatarateChannel
        {
            delay = 0s;
        }
    submodules:
        ros: RosInterface {
            parameters:
                @display("p=50,50;i=misc/cloud");
        }
        hostA: CanHost2 {
            parameters:
                @display("p=100,200");
        }
        reciever: CanHost {
            parameters:
                @display("p=250,200");
        }
        motor: CanHost {
            parameters:
                @display("p=400,200");
        }
        bus: CanBus {
            parameters:
                @display("p=325,100;b=500,6;o=#408060");
        }
    connections:
        bus.cang++ <--> C <--> hostA.cang;
        bus.cang++ <--> C <--> reciever.cang;
        bus.cang++ <--> C <--> motor.cang;
}
