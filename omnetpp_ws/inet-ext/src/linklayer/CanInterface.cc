#ifndef __CANINTERFACE_H
#define __CANINTERFACE_H

#include "inet/common/INETDefs.h"
#include "inet/networklayer/common/InterfaceEntry.h"

using namespace inet;

class CanInterface : public InterfaceEntry
{
};

#endif // ifndef __CANINTERFACE_H

//#include "CanTransciever.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(CanInterface);

/// TODO ///
