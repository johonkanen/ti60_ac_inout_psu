/*
 * memcpy that stores whole words whatever the alignment of source and destination.
 *
 * Picolibc copies words only when both pointers have the same alignment within a word;
 * otherwise it copies bytes. On a Sapphire SoC running from HyperRAM every store goes
 * through the write-through data cache to the memory, about 28 CPU clocks each whether it
 * stores a byte or a word, so a byte copy is four times slower than it needs to be. The
 * network stack copies at a 2 byte misalignment all the time (TCP payload starts 54
 * bytes into a frame).
 *
 * The destination is aligned first, as stores are what costs. A source at another
 * alignment is read in aligned words, each output word put together from two of them
 * with shifts (little endian). The last of those reads may take up to 3 bytes past the
 * end of the source, within its last aligned word: no memory protection here.
 *
 * memmove uses the same copy when the destination is below the source (net_pkt_pull
 * shifts data down this way): every read is of memory above anything written so far.
 * A destination above an overlapping source is copied backwards a byte at a time.
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static inline void copy_forward(void *dst, const void *src, size_t n)
{
	uint8_t *d = dst;
	const uint8_t *s = src;
	uint32_t *dw;
	size_t words;
	unsigned int offset;

	if (n < 16) {
		while (n--) {
			*d++ = *s++;
		}
		return;
	}

	while ((uintptr_t)d & 3) {
		*d++ = *s++;
		n--;
	}
	dw = (uint32_t *)d;
	words = n / 4;
	offset = (uintptr_t)s & 3;

	if (offset == 0) {
		const uint32_t *sw = (const uint32_t *)s;
		size_t w = words;

		for (; w >= 4; w -= 4) {
			uint32_t w0 = sw[0], w1 = sw[1], w2 = sw[2], w3 = sw[3];

			dw[0] = w0;
			dw[1] = w1;
			dw[2] = w2;
			dw[3] = w3;
			sw += 4;
			dw += 4;
		}
		while (w--) {
			*dw++ = *sw++;
		}
	} else {
		const uint32_t *sw = (const uint32_t *)(s - offset);
		unsigned int right = 8 * offset, left = 32 - right;
		uint32_t w0 = *sw++;

		for (size_t w = words; w; w--) {
			uint32_t w1 = *sw++;

			*dw++ = (w0 >> right) | (w1 << left);
			w0 = w1;
		}
	}

	d = (uint8_t *)dw;
	s += 4 * words;
	n &= 3;
	while (n--) {
		*d++ = *s++;
	}
}

void *memcpy(void *__restrict dst, const void *__restrict src, size_t n)
{
	copy_forward(dst, src, n);
	return dst;
}

void *memmove(void *dst, const void *src, size_t n)
{
	uint8_t *d = dst;
	const uint8_t *s = src;

	if (d <= s || d >= s + n) {
		copy_forward(dst, src, n);
	} else {
		while (n--) {
			d[n] = s[n];
		}
	}
	return dst;
}
