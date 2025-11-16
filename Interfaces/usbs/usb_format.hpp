#pragma once
#include <cstddef>
#include <cstdarg>

namespace usb {

std::size_t format(char* dst, std::size_t cap, const char* fmt, ...);
std::size_t vformat(char* dst, std::size_t cap, const char* fmt, va_list ap);

}
