#pragma once

#include <cstdarg>
#include <string>
#include <cstdint>

#ifdef _WIN32
#define ATTRIBUTE_FORMAT_PRINTF(a, b)
#else
#define ATTRIBUTE_FORMAT_PRINTF(a, b) __attribute__((format(printf, a, b)))
#endif

std::string StrFormat(const char* format, ...) ATTRIBUTE_FORMAT_PRINTF(1, 2);
std::string StrFormatV(const char* format, va_list va);

namespace {

const char g_aHexDigits[16] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
};

// No buffer overflow check or null termination. Use with caution.
inline char* StrBufferAppendByteAsHex(char* cp, uint8_t n)
{
	*cp++ = g_aHexDigits[(n >> 4) & 0x0f];
	*cp++ = g_aHexDigits[(n >> 0) & 0x0f];
	return cp;
}

// No buffer overflow check or null termination. Use with caution.
inline char* StrBufferAppendWordAsHex(char* cp, uint16_t n)
{
	*cp++ = g_aHexDigits[(n >> 12) & 0x0f];
	*cp++ = g_aHexDigits[(n >>  8) & 0x0f];
	*cp++ = g_aHexDigits[(n >>  4) & 0x0f];
	*cp++ = g_aHexDigits[(n >>  0) & 0x0f];
	return cp;
}

inline std::string& StrAppendByteAsHex(std::string& s, uint8_t n)
{
	const char hex[2] = { g_aHexDigits[(n >> 4) & 0x0f],
						  g_aHexDigits[(n >> 0) & 0x0f] };
	return s.append(hex, 2);
}

inline std::string& StrAppendWordAsHex(std::string& s, uint16_t n)
{
	const char hex[4] = { g_aHexDigits[(n >> 12) & 0x0f],
						  g_aHexDigits[(n >>  8) & 0x0f],
						  g_aHexDigits[(n >>  4) & 0x0f],
						  g_aHexDigits[(n >>  0) & 0x0f] };
	return s.append(hex, 4);
}

inline std::string ByteToHexStr(uint8_t n)
{
	std::string s;
	StrAppendByteAsHex(s, n);
	return s;
}

inline std::string WordToHexStr(uint16_t n)
{
	std::string s;
	StrAppendWordAsHex(s, n);
	return s;
}

inline std::string DWordToHexStr(uint32_t n)
{
	std::string s;
	StrAppendWordAsHex(s, n >> 16);
	StrAppendWordAsHex(s, n);
	return s;
}

} // namespace
