#ifndef LIME_OPSTATUS_H
#define LIME_OPSTATUS_H

namespace lime {

enum class OpStatus
{
    SUCCESS = 0,
    ERROR = -1,
    NOT_IMPLEMENTED = -2,
    IO_FAILURE = -3,
    INVALID_VALUE = -4,
    FILE_NOT_FOUND = -5,
    OUT_OF_RANGE = -6,
    NOT_SUPPORTED = -7,
    TIMEOUT = -8,
    BUSY = -9,
};

const char* ToCString(OpStatus value);

} // namespace lime

#endif
