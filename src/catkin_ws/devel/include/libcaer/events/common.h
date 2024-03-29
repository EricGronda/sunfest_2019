/**
 * @file common.h
 *
 * Common EventPacket header format definition and handling functions.
 * Every EventPacket, of any type, has as a first member a common header,
 * which describes various properties of the contained events. This allows
 * easy parsing of events. See the 'struct caer_event_packet_header'
 * documentation for more details.
 */

#ifndef LIBCAER_EVENTS_COMMON_H_
#define LIBCAER_EVENTS_COMMON_H_

#include "../libcaer.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CAER_EVENTS_HEADER_ONLY
#define caerLogEHO caerLog
#else
static inline void caerLogEHO(enum caer_log_level logLevel, const char *subSystem, const char *format, ...) {
	// Ignore logLevel, all event packet messages are critical.
	(void) (logLevel);

	printf("%s: ", subSystem);

	va_list argumentList;
	va_start(argumentList, format);
	vprintf(format, argumentList);
	va_end(argumentList);

	printf("\n");
}
#endif

/**
 * Generic validity mark:
 * this bit is used to mark whether an event is still
 * valid or not, and can be used to efficiently filter
 * out events from a packet.
 * The caerXXXEventValidate() and caerXXXEventInvalidate()
 * functions should be used to toggle this!
 * 0 in the 0th bit of the first byte means invalid, 1 means valid.
 * This way zeroing-out an event packet sets all its events to invalid.
 * Care must be taken to put the field containing the validity
 * mark always as the first member of an event.
 */
//@{
#define VALID_MARK_SHIFT 0
#define VALID_MARK_MASK 0x00000001
//@}

/**
 * 64bit timestamp support:
 * since timestamps wrap around after some time, being only 31 bit (32 bit signed int),
 * another timestamp at the packet level provides another 31 bit (32 bit signed int),
 * to enable the generation of a 62 bit (64 bit signed int) microsecond timestamp which
 * is guaranteed to never wrap around (in the next 146'138 years at least).
 * The TSOverflow needs to be shifted by 31 thus when constructing such a timestamp.
 */
#define TS_OVERFLOW_SHIFT 31

/**
 * List of supported event types.
 * Each event type has its own integer representation.
 * All event types below 100 are reserved for use
 * by libcaer and cAER.
 * DO NOT USE THEM FOR YOUR OWN EVENT TYPES!
 */
enum caer_default_event_types {
	SPECIAL_EVENT   = 0,  //!< Special events.
	POLARITY_EVENT  = 1,  //!< Polarity (change, DVS) events.
	FRAME_EVENT     = 2,  //!< Frame (intensity, APS) events.
	IMU6_EVENT      = 3,  //!< 6 axes IMU events.
	IMU9_EVENT      = 4,  //!< 9 axes IMU events.
	SAMPLE_EVENT    = 5,  //!< ADC sample events (deprecated).
	EAR_EVENT       = 6,  //!< Ear (cochlea) events (deprecated).
	CONFIG_EVENT    = 7,  //!< Device configuration events (deprecated).
	POINT1D_EVENT   = 8,  //!< 1D measurement events (deprecated).
	POINT2D_EVENT   = 9,  //!< 2D measurement events (deprecated).
	POINT3D_EVENT   = 10, //!< 3D measurement events (deprecated).
	POINT4D_EVENT   = 11, //!< 4D measurement events (deprecated).
	SPIKE_EVENT     = 12, //!< Spike events.
	MATRIX4x4_EVENT = 13, //!< 4D matrix events (deprecated).
};

/**
 * Number of default event types that are part of libcaer.
 * Corresponds to the count of definitions inside the
 * 'enum caer_default_event_types' enumeration.
 */
#define CAER_DEFAULT_EVENT_TYPES_COUNT 14

/**
 * Size of the EventPacket header.
 * This is constant across all supported systems.
 */
#define CAER_EVENT_PACKET_HEADER_SIZE 28

/**
 * EventPacket header data structure definition.
 * The size, also defined in CAER_EVENT_PACKET_HEADER_SIZE,
 * must always be constant. The header is common to all
 * types of event packets and is always the very first
 * member of an event packet data structure.
 * Signed integers are used for compatibility with languages that
 * do not have unsigned ones, such as Java.
 */
PACKED_STRUCT(struct caer_event_packet_header {
	/// Numerical type ID, unique to each event type (see 'enum caer_default_event_types').
	int16_t eventType;
	/// Numerical source ID, unique inside a process, identifies who generated the events.
	int16_t eventSource;
	/// Size of one event in bytes.
	int32_t eventSize;
	/// Offset from the start of an event, in bytes, at which the main 32 bit time-stamp can be found.
	int32_t eventTSOffset;
	/// Overflow counter for the standard 32bit event time-stamp. Used to generate the 64 bit time-stamp.
	int32_t eventTSOverflow;
	/// Maximum number of events this packet can store.
	int32_t eventCapacity;
	/// Total number of events present in this packet (valid + invalid).
	int32_t eventNumber;
	/// Total number of valid events present in this packet.
	int32_t eventValid;
});

/**
 * Type for pointer to EventPacket header data structure.
 */
typedef struct caer_event_packet_header *caerEventPacketHeader;
typedef const struct caer_event_packet_header *caerEventPacketHeaderConst;

/**
 * Return the numerical event type ID, representing the event type this
 * EventPacket is containing.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the numerical event type (see 'enum caer_default_event_types').
 */
static inline int16_t caerEventPacketHeaderGetEventType(caerEventPacketHeaderConst header) {
	return (I16T(le16toh(U16T(header->eventType))));
}

/**
 * Set the numerical event type ID, representing the event type this
 * EventPacket will contain.
 * All event types below 100 are reserved for use by libcaer and cAER.
 * DO NOT USE THEM FOR YOUR OWN EVENT TYPES!
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 * @param eventType the numerical event type (see 'enum caer_default_event_types').
 */
static inline void caerEventPacketHeaderSetEventType(caerEventPacketHeader header, int16_t eventType) {
	if (eventType < 0) {
		// Negative numbers (bit 31 set) are not allowed!
		caerLogEHO(
			CAER_LOG_CRITICAL, "EventPacket Header", "Called caerEventPacketHeaderSetEventType() with negative value!");
		return;
	}

	header->eventType = I16T(htole16(U16T(eventType)));
}

/**
 * Get the numerical event source ID, representing the event source
 * that generated all the events present in this packet.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the numerical event source ID.
 */
static inline int16_t caerEventPacketHeaderGetEventSource(caerEventPacketHeaderConst header) {
	return (I16T(le16toh(U16T(header->eventSource))));
}

/**
 * Set the numerical event source ID, representing the event source
 * that generated all the events present in this packet.
 * This ID should be unique at least within a process, if not within
 * the whole system, to guarantee correct identification of who
 * generated an event later on.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 * @param eventSource the numerical event source ID.
 */
static inline void caerEventPacketHeaderSetEventSource(caerEventPacketHeader header, int16_t eventSource) {
	if (eventSource < 0) {
		// Negative numbers (bit 31 set) are not allowed!
		caerLogEHO(CAER_LOG_CRITICAL, "EventPacket Header",
			"Called caerEventPacketHeaderSetEventSource() with negative value!");
		return;
	}

	header->eventSource = I16T(htole16(U16T(eventSource)));
}

/**
 * Get the size of a single event, in bytes.
 * All events inside an event packet always have the same size.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the event size in bytes.
 */
static inline int32_t caerEventPacketHeaderGetEventSize(caerEventPacketHeaderConst header) {
	return (I32T(le32toh(U32T(header->eventSize))));
}

/**
 * Set the size of a single event, in bytes.
 * All events inside an event packet always have the same size.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 * @param eventSize the event size in bytes.
 */
static inline void caerEventPacketHeaderSetEventSize(caerEventPacketHeader header, int32_t eventSize) {
	if (eventSize < 0) {
		// Negative numbers (bit 31 set) are not allowed!
		caerLogEHO(
			CAER_LOG_CRITICAL, "EventPacket Header", "Called caerEventPacketHeaderSetEventSize() with negative value!");
		return;
	}

	header->eventSize = I32T(htole32(U32T(eventSize)));
}

/**
 * Get the offset, in bytes, to where the field with the main
 * 32 bit timestamp is stored. This is useful for generic access
 * to the timestamp field, given that different event types might
 * have it at different offsets or might even have multiple
 * timestamps, in which case this offset references the 'main'
 * timestamp, the most representative one.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the event timestamp offset in bytes.
 */
static inline int32_t caerEventPacketHeaderGetEventTSOffset(caerEventPacketHeaderConst header) {
	return (I32T(le32toh(U32T(header->eventTSOffset))));
}

/**
 * Set the offset, in bytes, to where the field with the main
 * 32 bit timestamp is stored. This is useful for generic access
 * to the timestamp field, given that different event types might
 * have it at different offsets or might even have multiple
 * timestamps, in which case this offset references the 'main'
 * timestamp, the most representative one.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 * @param eventTSOffset the event timestamp offset in bytes.
 */
static inline void caerEventPacketHeaderSetEventTSOffset(caerEventPacketHeader header, int32_t eventTSOffset) {
	if (eventTSOffset < 0) {
		// Negative numbers (bit 31 set) are not allowed!
		caerLogEHO(CAER_LOG_CRITICAL, "EventPacket Header",
			"Called caerEventPacketHeaderSetEventTSOffset() with negative value!");
		return;
	}

	header->eventTSOffset = I32T(htole32(U32T(eventTSOffset)));
}

/**
 * Get the 32 bit timestamp overflow counter (in microseconds). This is per-packet
 * and is used to generate a 64 bit timestamp that never wraps around.
 * Since timestamps wrap around after some time, being only 31 bit (32 bit signed int),
 * another timestamp at the packet level provides another 31 bit (32 bit signed int),
 * to enable the generation of a 62 bit (64 bit signed int) microsecond timestamp which
 * is guaranteed to never wrap around (in the next 146'138 years at least).
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the packet-level timestamp overflow counter, in microseconds.
 */
static inline int32_t caerEventPacketHeaderGetEventTSOverflow(caerEventPacketHeaderConst header) {
	return (I32T(le32toh(U32T(header->eventTSOverflow))));
}

/**
 * Set the 32 bit timestamp overflow counter (in microseconds). This is per-packet
 * and is used to generate a 64 bit timestamp that never wraps around.
 * Since timestamps wrap around after some time, being only 31 bit (32 bit signed int),
 * another timestamp at the packet level provides another 31 bit (32 bit signed int),
 * to enable the generation of a 62 bit (64 bit signed int) microsecond timestamp which
 * is guaranteed to never wrap around (in the next 146'138 years at least).
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 * @param eventTSOverflow the packet-level timestamp overflow counter, in microseconds.
 */
static inline void caerEventPacketHeaderSetEventTSOverflow(caerEventPacketHeader header, int32_t eventTSOverflow) {
	if (eventTSOverflow < 0) {
		// Negative numbers (bit 31 set) are not allowed!
		caerLogEHO(CAER_LOG_CRITICAL, "EventPacket Header",
			"Called caerEventPacketHeaderSetEventTSOverflow() with negative value!");
		return;
	}

	header->eventTSOverflow = I32T(htole32(U32T(eventTSOverflow)));
}

/**
 * Get the maximum number of events this packet can store.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the number of events this packet can hold.
 */
static inline int32_t caerEventPacketHeaderGetEventCapacity(caerEventPacketHeaderConst header) {
	return (I32T(le32toh(U32T(header->eventCapacity))));
}

/**
 * Set the maximum number of events this packet can store.
 * This is determined at packet allocation time and should
 * not be changed during the life-time of the packet.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 * @param eventsCapacity the number of events this packet can hold.
 */
static inline void caerEventPacketHeaderSetEventCapacity(caerEventPacketHeader header, int32_t eventsCapacity) {
	if (eventsCapacity < 0) {
		// Negative numbers (bit 31 set) are not allowed!
		caerLogEHO(CAER_LOG_CRITICAL, "EventPacket Header",
			"Called caerEventPacketHeaderSetEventCapacity() with negative value!");
		return;
	}

	header->eventCapacity = I32T(htole32(U32T(eventsCapacity)));
}

/**
 * Get the number of events currently stored in this packet,
 * considering both valid and invalid events.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the number of events in this packet.
 */
static inline int32_t caerEventPacketHeaderGetEventNumber(caerEventPacketHeaderConst header) {
	return (I32T(le32toh(U32T(header->eventNumber))));
}

/**
 * Set the number of events currently stored in this packet,
 * considering both valid and invalid events.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 * @param eventsNumber the number of events in this packet.
 */
static inline void caerEventPacketHeaderSetEventNumber(caerEventPacketHeader header, int32_t eventsNumber) {
	if (eventsNumber < 0) {
		// Negative numbers (bit 31 set) are not allowed!
		caerLogEHO(CAER_LOG_CRITICAL, "EventPacket Header",
			"Called caerEventPacketHeaderSetEventNumber() with negative value!");
		return;
	}

	header->eventNumber = I32T(htole32(U32T(eventsNumber)));
}

/**
 * Get the number of valid events in this packet, disregarding
 * invalid ones (where the invalid mark is set).
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the number of valid events in this packet.
 */
static inline int32_t caerEventPacketHeaderGetEventValid(caerEventPacketHeaderConst header) {
	return (I32T(le32toh(U32T(header->eventValid))));
}

/**
 * Set the number of valid events in this packet, disregarding
 * invalid ones (where the invalid mark is set).
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 * @param eventsValid the number of valid events in this packet.
 */
static inline void caerEventPacketHeaderSetEventValid(caerEventPacketHeader header, int32_t eventsValid) {
	if (eventsValid < 0) {
		// Negative numbers (bit 31 set) are not allowed!
		caerLogEHO(CAER_LOG_CRITICAL, "EventPacket Header",
			"Called caerEventPacketHeaderSetEventValid() with negative value!");
		return;
	}

	header->eventValid = I32T(htole32(U32T(eventsValid)));
}

/**
 * Get a generic pointer to an event, without having to know what event
 * type the packet is containing.
 *
 * @param headerPtr a valid EventPacket header pointer. Cannot be NULL.
 * @param n the index of the returned event. Must be within [0,eventNumber[ bounds.
 *
 * @return a generic pointer to the requested event. NULL on error.
 *         This points to unmodifiable memory, as it should never be used for anything
 *         other than read operations, such as caerGenericEventGetTimestamp(). Don't
 *         modify the memory, you have no idea what it is! If you do know, just use the
 *         proper typed packet functions.
 */
static inline const void *caerGenericEventGetEvent(caerEventPacketHeaderConst headerPtr, int32_t n) {
	// Check that we're not out of bounds.
	// Accessing elements after EventNumber() but before EventCapacity() doesn't
	// make any sense here for the Generic Event getter, as we only support
	// reading/querying data from those events, and that would always fail for
	// those empty events, as they are all zeroed out.
	if (n < 0 || n >= caerEventPacketHeaderGetEventNumber(headerPtr)) {
		caerLogEHO(CAER_LOG_CRITICAL, "Generic Event",
			"Called caerGenericEventGetEvent() with invalid event offset %" PRIi32
			", while maximum allowed value is %" PRIi32 ". Negative values are not allowed!",
			n, caerEventPacketHeaderGetEventNumber(headerPtr) - 1);
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (((const uint8_t *) headerPtr)
			+ (CAER_EVENT_PACKET_HEADER_SIZE + U64T(n * caerEventPacketHeaderGetEventSize(headerPtr))));
}

/**
 * Get the main 32 bit timestamp for a generic event, without having to
 * know what event type the packet is containing.
 *
 * @param eventPtr a generic pointer to an event. Cannot be NULL.
 * @param headerPtr a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the main 32 bit timestamp of this event.
 */
static inline int32_t caerGenericEventGetTimestamp(const void *eventPtr, caerEventPacketHeaderConst headerPtr) {
	return (I32T(le32toh(U32T(*(
		(const int32_t *) (((const uint8_t *) eventPtr) + U64T(caerEventPacketHeaderGetEventTSOffset(headerPtr))))))));
}

/**
 * Get the main 64 bit timestamp for a generic event, without having to
 * know what event type the packet is containing. This takes the
 * per-packet timestamp into account too, generating a timestamp
 * that doesn't suffer from overflow problems.
 *
 * @param eventPtr a generic pointer to an event. Cannot be NULL.
 * @param headerPtr a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the main 64 bit timestamp of this event.
 */
static inline int64_t caerGenericEventGetTimestamp64(const void *eventPtr, caerEventPacketHeaderConst headerPtr) {
	return (I64T((U64T(caerEventPacketHeaderGetEventTSOverflow(headerPtr)) << TS_OVERFLOW_SHIFT)
				 | U64T(caerGenericEventGetTimestamp(eventPtr, headerPtr))));
}

/**
 * Check if the given generic event is valid or not.
 *
 * @param eventPtr a generic pointer to an event. Cannot be NULL.
 *
 * @return true if the event is valid, false otherwise.
 */
static inline bool caerGenericEventIsValid(const void *eventPtr) {
	// Look at first byte of event memory's lowest bit.
	// This should always work since first event member must contain the valid mark
	// and memory is little-endian, so lowest bit must be in first byte of memory.
	return (*((const uint8_t *) eventPtr) & VALID_MARK_MASK);
}

/**
 * Copy a given event's content to another location in memory.
 *
 * @param eventPtrDestination a generic pointer to an event to copy to. Cannot be NULL.
 * @param eventPtrSource a generic pointer to an event to copy from. Cannot be NULL.
 * @param headerPtrDestination a valid EventPacket header pointer from the destination packet. Cannot be NULL.
 * @param headerPtrSource a valid EventPacket header pointer from the source packet. Cannot be NULL.
 *
 * @return true on successful copy, false otherwise.
 */
static inline bool caerGenericEventCopy(void *eventPtrDestination, const void *eventPtrSource,
	caerEventPacketHeaderConst headerPtrDestination, caerEventPacketHeaderConst headerPtrSource) {
	if ((caerEventPacketHeaderGetEventType(headerPtrDestination) != caerEventPacketHeaderGetEventType(headerPtrSource))
		|| (caerEventPacketHeaderGetEventSize(headerPtrDestination)
			   != caerEventPacketHeaderGetEventSize(headerPtrSource))
		|| (caerEventPacketHeaderGetEventTSOverflow(headerPtrDestination)
			   != caerEventPacketHeaderGetEventTSOverflow(headerPtrSource))) {
		return (false);
	}

	memcpy(eventPtrDestination, eventPtrSource, (size_t) caerEventPacketHeaderGetEventSize(headerPtrDestination));
	return (true);
}

/**
 * Generic iterator over all events in a packet.
 * Returns the current index in the 'caerIteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerIteratorElement' variable
 * of type EVENT_TYPE.
 *
 * PACKET_HEADER: a valid EventPacket header pointer. Cannot be NULL.
 * EVENT_TYPE: the event pointer type for this EventPacket (ie. caerPolarityEvent or caerFrameEvent).
 */
#define CAER_ITERATOR_ALL_START(PACKET_HEADER, EVENT_TYPE)                                                          \
	for (int32_t caerIteratorCounter = 0; caerIteratorCounter < caerEventPacketHeaderGetEventNumber(PACKET_HEADER); \
		 caerIteratorCounter++) {                                                                                   \
		EVENT_TYPE caerIteratorElement = (EVENT_TYPE) caerGenericEventGetEvent(PACKET_HEADER, caerIteratorCounter);

/**
 * Generic iterator close statement.
 */
#define CAER_ITERATOR_ALL_END }

/**
 * Generic iterator over only the valid events in a packet.
 * Returns the current index in the 'caerIteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerIteratorElement' variable
 * of type EVENT_TYPE.
 *
 * PACKET_HEADER: a valid EventPacket header pointer. Cannot be NULL.
 * EVENT_TYPE: the event pointer type for this EventPacket (ie. caerPolarityEvent or caerFrameEvent).
 */
#define CAER_ITERATOR_VALID_START(PACKET_HEADER, EVENT_TYPE)                                                        \
	for (int32_t caerIteratorCounter = 0; caerIteratorCounter < caerEventPacketHeaderGetEventNumber(PACKET_HEADER); \
		 caerIteratorCounter++) {                                                                                   \
		EVENT_TYPE caerIteratorElement = (EVENT_TYPE) caerGenericEventGetEvent(PACKET_HEADER, caerIteratorCounter); \
		if (!caerGenericEventIsValid(caerIteratorElement)) {                                                        \
			continue;                                                                                               \
		} // Skip invalid events.

/**
 * Generic iterator close statement.
 */
#define CAER_ITERATOR_VALID_END }

/**
 * Get the data size of an event packet, in bytes.
 * This is only the size of the data portion, excluding the header.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the event packet data size in bytes.
 */
static inline int64_t caerEventPacketGetDataSize(caerEventPacketHeaderConst header) {
	return (I64T(caerEventPacketHeaderGetEventSize(header)) * I64T(caerEventPacketHeaderGetEventCapacity(header)));
}

/**
 * Get the full size of an event packet, in bytes.
 * This includes both the header and the data portion.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the event packet size in bytes.
 */
static inline int64_t caerEventPacketGetSize(caerEventPacketHeaderConst header) {
	return (CAER_EVENT_PACKET_HEADER_SIZE + caerEventPacketGetDataSize(header));
}

/**
 * Get the data size of an event packet, in bytes, up to its last actual
 * event. This means only up to EventNumber, not up to EventCapacity.
 * This is only the size of the data portion, excluding the header.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the event packet data size in bytes (up to event number).
 */
static inline int64_t caerEventPacketGetDataSizeEvents(caerEventPacketHeaderConst header) {
	return (I64T(caerEventPacketHeaderGetEventSize(header)) * I64T(caerEventPacketHeaderGetEventNumber(header)));
}

/**
 * Get the full size of an event packet, in bytes, up to its last actual
 * event. This means only up to EventNumber, not up to EventCapacity.
 * This includes both the header and the data portion.
 *
 * @param header a valid EventPacket header pointer. Cannot be NULL.
 *
 * @return the event packet size in bytes (up to event number).
 */
static inline int64_t caerEventPacketGetSizeEvents(caerEventPacketHeaderConst header) {
	return (CAER_EVENT_PACKET_HEADER_SIZE + caerEventPacketGetDataSizeEvents(header));
}

/**
 * Verify if two event packets are equal. This means that the
 * header and all events are equal.
 *
 * @param firstPacket an event packet to be compared.
 * @param secondPacket the other event packet to compare against.
 *
 * @return true if both are the same, false otherwise.
 */
static inline bool caerEventPacketEquals(
	caerEventPacketHeaderConst firstPacket, caerEventPacketHeaderConst secondPacket) {
	// If any of the packets is NULL, they can't be equal.
	if (firstPacket == NULL || secondPacket == NULL) {
		return (false);
	}

	// If both packets are the same packet (pointer equal),
	// they are of course indeed equal packets.
	if (firstPacket == secondPacket) {
		return (true);
	}

	// Actually compare memory now. We compare header equality, and
	// all events up to eventNumber. The remaining events up to
	// eventCapacity are by definition all zeroed out, so must be
	// equal, if the capacity is the same, which it is, as we check
	// for that when ensuring header equality.
	if (memcmp(firstPacket, secondPacket, CAER_EVENT_PACKET_HEADER_SIZE) != 0) {
		return (false);
	}

	size_t memCmpSize
		= (size_t)(caerEventPacketHeaderGetEventNumber(firstPacket) * caerEventPacketHeaderGetEventSize(firstPacket));
	if (memcmp(((const uint8_t *) firstPacket) + CAER_EVENT_PACKET_HEADER_SIZE,
			((const uint8_t *) secondPacket) + CAER_EVENT_PACKET_HEADER_SIZE, memCmpSize)
		!= 0) {
		return (false);
	}

	return (true);
}

/**
 * Clear a packet by zeroing out all events.
 * Capacity doesn't change, event number is set to zero.
 *
 * @param packet an event packet to clear out.
 */
static inline void caerEventPacketClear(caerEventPacketHeader packet) {
	// Handle empty event packets.
	if (packet == NULL) {
		return;
	}

	// Set events up to eventNumber to zero. The remaining events up to
	// eventCapacity are by definition all zeroed out, so nothing to do
	// there. Also reset the eventValid and eventNumber header fields.
	size_t memZeroSize
		= (size_t)(caerEventPacketHeaderGetEventNumber(packet) * caerEventPacketHeaderGetEventSize(packet));
	memset(((uint8_t *) packet) + CAER_EVENT_PACKET_HEADER_SIZE, 0, memZeroSize);

	caerEventPacketHeaderSetEventValid(packet, 0);
	caerEventPacketHeaderSetEventNumber(packet, 0);
}

/**
 * Clean a packet by removing all invalid events, so that
 * the total number of events is the number of valid events.
 * The packet's capacity doesn't change.
 *
 * @param packet an event packet to clean.
 */
static inline void caerEventPacketClean(caerEventPacketHeader packet) {
	// Handle empty event packets.
	if (packet == NULL) {
		return;
	}

	// Calculate needed memory for new event packet.
	int32_t eventValid  = caerEventPacketHeaderGetEventValid(packet);
	int32_t eventNumber = caerEventPacketHeaderGetEventNumber(packet);

	// If we have no invalid events, we're already done.
	if (eventValid == eventNumber) {
		return;
	}

	int32_t eventSize     = caerEventPacketHeaderGetEventSize(packet);
	int32_t eventCapacity = caerEventPacketHeaderGetEventCapacity(packet);

	// Move all valid events close together. Must check every event for validity!
	size_t offset = CAER_EVENT_PACKET_HEADER_SIZE;

	CAER_ITERATOR_VALID_START(packet, const void *)
	void *dest = ((uint8_t *) packet) + offset;

	if (dest != caerIteratorElement) {
		memcpy(dest, caerIteratorElement, (size_t) eventSize);
		offset += (size_t) eventSize;
	}
}

// Reset remaining memory, up to capacity, to zero (all events invalid).
memset(((uint8_t *) packet) + offset, 0, (size_t)((eventCapacity - eventValid) * eventSize));

// Event capacity remains unchanged, event number shrunk to event valid number.
caerEventPacketHeaderSetEventNumber(packet, eventValid);
}

/**
 * Resize an event packet.
 * First, the packet is cleaned (all invalid events removed), then:
 * - If the old and new event capacity are equal, nothing else changes.
 * - If the new capacity is bigger, the packet is enlarged and the new events
 *   are initialized to all zeros (invalid).
 * - If the new capacity is smaller, the packet is truncated at the given point.
 * Use free() to reclaim this memory afterwards.
 *
 * @param packet the current event packet.
 * @param newEventCapacity the new maximum number of events this packet can hold.
 *                         Cannot be zero.
 *
 * @return a valid event packet handle or NULL on error.
 * On success, the old packet handle is to be considered invalid and not to be
 * used anymore. On failure, the old packet handle is still valid, but will
 * have been cleaned of all invalid events!
 */
static inline caerEventPacketHeader caerEventPacketResize(caerEventPacketHeader packet, int32_t newEventCapacity) {
	if (packet == NULL || newEventCapacity <= 0) {
		return (NULL);
	}

	// Always clean for consistency with shrink case (side-effects guarantee).
	caerEventPacketClean(packet);

	int32_t oldEventCapacity = caerEventPacketHeaderGetEventCapacity(packet);

	if (oldEventCapacity == newEventCapacity) {
		// Nothing to do in this case.
		return (packet);
	}

	int32_t eventSize         = caerEventPacketHeaderGetEventSize(packet);
	size_t newEventPacketSize = CAER_EVENT_PACKET_HEADER_SIZE + (size_t)(newEventCapacity * eventSize);

	// Reallocate memory used to hold events.
	packet = (caerEventPacketHeader) realloc(packet, newEventPacketSize);
	if (packet == NULL) {
		caerLogEHO(CAER_LOG_CRITICAL, "Event Packet",
			"Failed to reallocate %zu bytes of memory for resizing Event Packet of capacity %" PRIi32
			" to new capacity of %" PRIi32 ". Error: %d.",
			newEventPacketSize, oldEventCapacity, newEventCapacity, errno);
		return (NULL);
	}

	if (newEventCapacity > oldEventCapacity) {
		// Capacity increased: we simply zero out the newly added events.
		size_t oldEventPacketSize = CAER_EVENT_PACKET_HEADER_SIZE + (size_t)(oldEventCapacity * eventSize);

		memset(
			((uint8_t *) packet) + oldEventPacketSize, 0, (size_t)((newEventCapacity - oldEventCapacity) * eventSize));
	}
	else {
		// Capacity decreased: the events were cleaned, so eventValid == eventNumber.
		// They also are all together in memory. Thus we can simply keep the current
		// eventValid/eventNumber counts if the capacity is still bigger or equal to
		// them, or, if new capacity is smaller, we reset them to that value.
		int32_t oldEventNumber = caerEventPacketHeaderGetEventNumber(packet);

		if (newEventCapacity < oldEventNumber) {
			caerEventPacketHeaderSetEventValid(packet, newEventCapacity);
			caerEventPacketHeaderSetEventNumber(packet, newEventCapacity);
		}
	}

	// Update capacity header field.
	caerEventPacketHeaderSetEventCapacity(packet, newEventCapacity);

	return (packet);
}

/**
 * Grows an event packet.
 * This only supports strictly increasing the size of a packet.
 * For a more flexible resize operation, see caerEventPacketResize().
 * Use free() to reclaim this memory afterwards.
 *
 * @param packet the current event packet.
 * @param newEventCapacity the new maximum number of events this packet can hold.
 *                         Cannot be zero.
 *
 * @return a valid event packet handle or NULL on error.
 * On success, the old packet handle is to be considered invalid and not to be
 * used anymore. On failure, the old packet handle is not touched in any way.
 */
static inline caerEventPacketHeader caerEventPacketGrow(caerEventPacketHeader packet, int32_t newEventCapacity) {
	if (packet == NULL || newEventCapacity <= 0) {
		return (NULL);
	}

	int32_t oldEventCapacity = caerEventPacketHeaderGetEventCapacity(packet);

	if (newEventCapacity <= oldEventCapacity) {
		caerLogEHO(CAER_LOG_CRITICAL, "Event Packet", "Called caerEventPacketGrow() with a new capacity value (%" PRIi32
													  ") that is equal or smaller than the old one (%" PRIi32 "). "
													  "Only strictly growing an event packet is supported!",
			newEventCapacity, oldEventCapacity);
		return (NULL);
	}

	int32_t eventSize         = caerEventPacketHeaderGetEventSize(packet);
	size_t newEventPacketSize = CAER_EVENT_PACKET_HEADER_SIZE + (size_t)(newEventCapacity * eventSize);

	// Grow memory used to hold events.
	packet = (caerEventPacketHeader) realloc(packet, newEventPacketSize);
	if (packet == NULL) {
		caerLogEHO(CAER_LOG_CRITICAL, "Event Packet",
			"Failed to reallocate %zu bytes of memory for growing Event Packet of capacity %" PRIi32
			" to new capacity of %" PRIi32 ". Error: %d.",
			newEventPacketSize, oldEventCapacity, newEventCapacity, errno);
		return (NULL);
	}

	// Zero out new event memory (all events invalid).
	size_t oldEventPacketSize = CAER_EVENT_PACKET_HEADER_SIZE + (size_t)(oldEventCapacity * eventSize);

	memset(((uint8_t *) packet) + oldEventPacketSize, 0, (size_t)((newEventCapacity - oldEventCapacity) * eventSize));

	// Update capacity header field.
	caerEventPacketHeaderSetEventCapacity(packet, newEventCapacity);

	return (packet);
}

/**
 * Appends an event packet to another.
 * This is a simple append operation, no timestamp reordering is done.
 * Please ensure time is monotonically increasing over the two packets!
 * Use free() to reclaim this memory afterwards.
 *
 * @param packet the main events packet.
 * @param appendPacket the events packet to append on the main one.
 *
 * @return a valid event packet handle or NULL on error.
 * On success, the old packet handle is to be considered invalid and not to be
 * used anymore. On failure, the old packet handle is not touched in any way.
 * The appendPacket handle is never touched in any way.
 */
static inline caerEventPacketHeader caerEventPacketAppend(
	caerEventPacketHeader packet, caerEventPacketHeader appendPacket) {
	if (packet == NULL) {
		return (NULL);
	}

	// Support appending nothing, the result is the unmodified input.
	if (appendPacket == NULL) {
		return (packet);
	}

	// Check that the two packets are of the same type and size, and have the same TSOverflow epoch.
	if ((caerEventPacketHeaderGetEventType(packet) != caerEventPacketHeaderGetEventType(appendPacket))
		|| (caerEventPacketHeaderGetEventSize(packet) != caerEventPacketHeaderGetEventSize(appendPacket))
		|| (caerEventPacketHeaderGetEventTSOverflow(packet) != caerEventPacketHeaderGetEventTSOverflow(appendPacket))) {
		return (NULL);
	}

	int32_t packetEventValid    = caerEventPacketHeaderGetEventValid(packet);
	int32_t packetEventNumber   = caerEventPacketHeaderGetEventNumber(packet);
	int32_t packetEventCapacity = caerEventPacketHeaderGetEventCapacity(packet);

	int32_t appendPacketEventValid    = caerEventPacketHeaderGetEventValid(appendPacket);
	int32_t appendPacketEventNumber   = caerEventPacketHeaderGetEventNumber(appendPacket);
	int32_t appendPacketEventCapacity = caerEventPacketHeaderGetEventCapacity(appendPacket);

	int32_t eventSize = caerEventPacketHeaderGetEventSize(packet); // Is the same! Checked above.
	size_t newEventPacketSize
		= CAER_EVENT_PACKET_HEADER_SIZE + (size_t)((packetEventCapacity + appendPacketEventCapacity) * eventSize);

	// Grow memory used to hold events.
	packet = (caerEventPacketHeader) realloc(packet, newEventPacketSize);
	if (packet == NULL) {
		caerLogEHO(CAER_LOG_CRITICAL, "Event Packet",
			"Failed to reallocate %zu bytes of memory for appending Event Packet of capacity %" PRIi32
			" to Event Packet of capacity %" PRIi32 ". Error: %d.",
			newEventPacketSize, appendPacketEventCapacity, packetEventCapacity, errno);
		return (NULL);
	}

	// Copy appendPacket event memory at start of free space in packet.
	memcpy(((uint8_t *) packet) + CAER_EVENT_PACKET_HEADER_SIZE + (packetEventNumber * eventSize),
		((uint8_t *) appendPacket) + CAER_EVENT_PACKET_HEADER_SIZE, (size_t)(appendPacketEventNumber * eventSize));

	// Zero out remaining event memory (all events invalid).
	memset(((uint8_t *) packet) + CAER_EVENT_PACKET_HEADER_SIZE
			   + ((packetEventNumber + appendPacketEventNumber) * eventSize),
		0, (size_t)(((packetEventCapacity + appendPacketEventCapacity) - (packetEventNumber + appendPacketEventNumber))
					* eventSize));

	// Update header fields.
	caerEventPacketHeaderSetEventValid(packet, (packetEventValid + appendPacketEventValid));
	caerEventPacketHeaderSetEventNumber(packet, (packetEventNumber + appendPacketEventNumber));
	caerEventPacketHeaderSetEventCapacity(packet, (packetEventCapacity + appendPacketEventCapacity));

	return (packet);
}

/**
 * Make a full copy of an event packet (up to eventCapacity).
 *
 * @param packet an event packet to copy.
 *
 * @return a full copy of an event packet.
 */
static inline caerEventPacketHeader caerEventPacketCopy(caerEventPacketHeaderConst packet) {
	// Handle empty event packets.
	if (packet == NULL) {
		return (NULL);
	}

	// Calculate needed memory for new event packet.
	int32_t eventSize     = caerEventPacketHeaderGetEventSize(packet);
	int32_t eventNumber   = caerEventPacketHeaderGetEventNumber(packet);
	int32_t eventCapacity = caerEventPacketHeaderGetEventCapacity(packet);
	size_t packetMem      = CAER_EVENT_PACKET_HEADER_SIZE + (size_t)(eventSize * eventCapacity);
	size_t dataMem        = CAER_EVENT_PACKET_HEADER_SIZE + (size_t)(eventSize * eventNumber);

	// Allocate memory for new event packet.
	caerEventPacketHeader packetCopy = (caerEventPacketHeader) malloc(packetMem);
	if (packetCopy == NULL) {
		// Failed to allocate memory.
		return (NULL);
	}

	// Copy the data over.
	memcpy(packetCopy, packet, dataMem);

	// Zero out the rest of the packet.
	memset(((uint8_t *) packetCopy) + dataMem, 0, packetMem - dataMem);

	return (packetCopy);
}

/**
 * Make a copy of an event packet, sized down to only include the
 * currently present events (eventNumber, valid+invalid), and not
 * including the possible extra unused events (up to eventCapacity).
 *
 * @param packet an event packet to copy.
 *
 * @return a sized down copy of an event packet.
 */
static inline caerEventPacketHeader caerEventPacketCopyOnlyEvents(caerEventPacketHeaderConst packet) {
	// Handle empty event packets.
	if (packet == NULL) {
		return (NULL);
	}

	// Calculate needed memory for new event packet.
	int32_t eventSize   = caerEventPacketHeaderGetEventSize(packet);
	int32_t eventNumber = caerEventPacketHeaderGetEventNumber(packet);

	if (eventNumber == 0) {
		// No copy possible if result is empty (capacity=0).
		return (NULL);
	}

	size_t packetMem = CAER_EVENT_PACKET_HEADER_SIZE + (size_t)(eventSize * eventNumber);

	// Allocate memory for new event packet.
	caerEventPacketHeader packetCopy = (caerEventPacketHeader) malloc(packetMem);
	if (packetCopy == NULL) {
		// Failed to allocate memory.
		return (NULL);
	}

	// Copy the data over.
	memcpy(packetCopy, packet, packetMem);

	// Set the event capacity to the event number, since we only allocated
	// memory for that many events.
	caerEventPacketHeaderSetEventCapacity(packetCopy, eventNumber);

	return (packetCopy);
}

/**
 * Make a copy of an event packet, sized down to only include the
 * currently valid events (eventValid), and discarding everything else.
 *
 * @param packet an event packet to copy.
 *
 * @return a copy of an event packet, containing only valid events.
 */
static inline caerEventPacketHeader caerEventPacketCopyOnlyValidEvents(caerEventPacketHeaderConst packet) {
	// Handle empty event packets.
	if (packet == NULL) {
		return (NULL);
	}

	// Calculate needed memory for new event packet.
	int32_t eventSize  = caerEventPacketHeaderGetEventSize(packet);
	int32_t eventValid = caerEventPacketHeaderGetEventValid(packet);

	if (eventValid == 0) {
		// No copy possible if result is empty (capacity=0).
		return (NULL);
	}

	size_t packetMem = CAER_EVENT_PACKET_HEADER_SIZE + (size_t)(eventSize * eventValid);

	// Allocate memory for new event packet.
	caerEventPacketHeader packetCopy = (caerEventPacketHeader) malloc(packetMem);
	if (packetCopy == NULL) {
		// Failed to allocate memory.
		return (NULL);
	}

	// First copy over the header.
	memcpy(packetCopy, packet, CAER_EVENT_PACKET_HEADER_SIZE);

	// Copy the data over. Must check every event for validity!
	size_t offset = CAER_EVENT_PACKET_HEADER_SIZE;

	CAER_ITERATOR_VALID_START(packet, const void *)
	memcpy(((uint8_t *) packetCopy) + offset, caerIteratorElement, (size_t) eventSize);
	offset += (size_t) eventSize;
}

// Set the event capacity and the event number to the number of
// valid events, since we only copied those.
caerEventPacketHeaderSetEventCapacity(packetCopy, eventValid);
caerEventPacketHeaderSetEventNumber(packetCopy, eventValid);

return (packetCopy);
}

/**
 * Allocate memory for an event packet and fill its header with the
 * proper information.
 * THIS FUNCTION IS INTENDED FOR INTERNAL USE ONLY BY THE VARIOUS
 * EVENT PACKET TYPES FOR MEMORY ALLOCATION.
 *
 * @return memory for an event packet, NULL on error.
 */
static inline caerEventPacketHeader caerEventPacketAllocate(int32_t eventCapacity, int16_t eventSource,
	int32_t tsOverflow, int16_t eventType, int32_t eventSize, int32_t eventTSOffset) {
	if ((eventCapacity <= 0) || (eventSource < 0) || (tsOverflow < 0) || (eventType < 0) || (eventSize <= 0)
		|| (eventTSOffset < 0)) {
		return (NULL);
	}

	size_t eventPacketSize = CAER_EVENT_PACKET_HEADER_SIZE + ((size_t) eventCapacity * (size_t) eventSize);

	// Zero out event memory (all events invalid).
	caerEventPacketHeader packet = (caerEventPacketHeader) calloc(1, eventPacketSize);
	if (packet == NULL) {
		caerLogEHO(CAER_LOG_CRITICAL, "Event Packet",
			"Failed to allocate %zu bytes of memory for Event Packet of type %" PRIi16 ", capacity %" PRIi32
			" from source %" PRIi16 ". Error: %d.",
			eventPacketSize, eventType, eventCapacity, eventSource, errno);
		return (NULL);
	}

	// Fill in header fields.
	caerEventPacketHeaderSetEventType(packet, eventType);
	caerEventPacketHeaderSetEventSource(packet, eventSource);
	caerEventPacketHeaderSetEventSize(packet, eventSize);
	caerEventPacketHeaderSetEventTSOffset(packet, eventTSOffset);
	caerEventPacketHeaderSetEventTSOverflow(packet, tsOverflow);
	caerEventPacketHeaderSetEventCapacity(packet, eventCapacity);

	return (packet);
}

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_EVENTS_COMMON_H_ */
