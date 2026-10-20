#pragma once
// NightRaven device protocol v2 (ESP32-S3 -> PC over USB serial), little-endian, packed.
// Shared file: keep the PC copy (src/Net/) and the firmware copy (include/) identical.
// Bump kVersion whenever a struct changes so old/new sides ignore each other.
//
// Wire frame:  [Header 13 bytes][payload `len` bytes][crc8 of everything before it]
// A serial port is a byte stream, so the receiver scans for the magic bytes 'N' 'R'
// and only accepts a frame whose CRC matches. Garbage (boot log, half frames) is skipped.

#include <stddef.h>
#include <stdint.h>

namespace nr
{
	constexpr uint16_t kMagic = 0x524E; // bytes on the wire: 0x4E 'N', 0x52 'R'
	constexpr uint8_t kVersion = 2;
	constexpr uint32_t kBaud = 115200;
	constexpr uint8_t kMaxPayload = 32;
	constexpr int16_t kAxisMax = 1000; // joystick axes are sent as -1000..1000

	enum class PacketType : uint8_t
	{
		Hello = 1,    // "I'm here and I have these modules", about once a second
		Joystick = 2, // stream, ~100 Hz
		Sonar = 3,    // one sweep sample per packet
	};

	enum Module : uint8_t
	{
		ModuleJoystick = 1 << 0,
		ModuleSonar = 1 << 1,
	};

#pragma pack(push, 1)
	struct Header
	{
		uint16_t magic;
		uint8_t version;
		uint8_t type;
		uint8_t len;       // payload size in bytes
		uint32_t seq;      // increments per frame (all types), used for loss counting
		uint32_t deviceMs; // millis() on the ESP32
	};

	struct HelloPayload { uint8_t modules; };
	struct JoystickPayload { int16_t x; int16_t y; uint8_t button; }; // button: 1 = pressed
	struct SonarPayload { uint16_t angleDeg10; uint16_t distanceMm; }; // angle in 0.1 degree, distance 0 = no echo
#pragma pack(pop)

	static_assert(sizeof(Header) == 13, "protocol header must be 13 bytes");
	static_assert(sizeof(JoystickPayload) == 5, "joystick payload must be 5 bytes");

	// CRC-8, polynomial 0x07, init 0.
	inline uint8_t Crc8(const uint8_t* data, size_t len)
	{
		uint8_t crc = 0;
		for (size_t i = 0; i < len; ++i)
		{
			crc ^= data[i];
			for (int b = 0; b < 8; ++b)
				crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
		}
		return crc;
	}
}
