import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;

cplusplus{{
#include "std_msgs/Bool.h"
}}

namespace inet;

class RosMsg extends FieldsChunk
{
    int id = -1;
    int data = -1;
};
