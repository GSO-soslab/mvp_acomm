
#pragma once

#include <iostream>

#include <google/protobuf/message.h>

namespace google
{
namespace protobuf
{
std::ostream& operator<<(std::ostream& os, const Message& msg)
{
    return os << msg.DebugString() << std::endl;
}
} // namespace protobuf
} 