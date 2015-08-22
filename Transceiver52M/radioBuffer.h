#include <stdlib.h>
#include <stddef.h>
#include <vector>

class RadioBuffer {
public:
	RadioBuffer(size_t numSegments, size_t segmentLen,
		    size_t hLen, bool outDirection);

	~RadioBuffer();

	const size_t getSegmentLen() { return segmentLen; };
	const size_t getNumSegments() { return numSegments; };
	const size_t getAvailSamples() { return availSamples; };
	const size_t getAvailSegments() { return availSamples / segmentLen; };

	const size_t getFreeSamples()
	{
		return bufferLen - availSamples;
	}

	const size_t getFreeSegments()
	{
		return getFreeSamples() / segmentLen;
	}

	void reset();

	/* Output direction */
	const float *getReadSegment();
	bool write(const float *wr, size_t len);
	bool zero(size_t len);

	/* Input direction */
	float *getWriteSegment();
	bool zeroWriteSegment();
	bool read(float *rd, size_t len);

private:
	size_t writeIndex, readIndex, availSamples;
	size_t bufferLen, numSegments, segmentLen, hLen;
	float *buffer;
	std::vector<float *> segments;
	bool outDirection;
};
