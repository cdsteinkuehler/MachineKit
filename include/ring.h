/********************************************************************
* Description:  ring.h
*
*               This file, 'ring.h', implements generic ringbuffer
*               functions for byte- and record-oriented queues.
*
*               All functions are defined as inline functions and do not
*               depend on any other code. They can in fact used standalone
*               provided a ringheader of ring_memsize() is allocated.
*
* Terminology:
*    record:
*
* ring buffers support non-blocking, variable-sized record queue
* operations between cooperating entitiess. The underlying model is a
* lock-free, single-reader, single-writer queue which does not require
* any operating system support and is extremely fast.
*
* ringbuffers are intended to replace a variety of special-purpose
* messaging schemes like the ones used between task and motion,
* in halstreamer, halsampler and halscope, at the same time making
* those amenable to a networked messaging integration.
*
* For an introduction on lock-free programming, see:
*   http://preshing.com/20120612/an-introduction-to-lock-free-programming
*   http://julien.benoist.name/lockfree.html
*
* The definitive reference on the subject is:
*   Herlihy/Shavit:  The Art of Multiprocessor Programming, Morgan Kaufmann 2012.
*
* The original record-oriented ring code is by Pavel Shramov, 1/2013,
* The multiframe ring code is by Pavel Shramov, 3/2014.
* http://psha.org.ru/cgit/psha/ring/
* License: MIT
*
* LinuxCNC integration by Michael Haberler, 2/2013.
*
* The stream-oriented ring code is adapted from:
* Portable Audio I/O Library Ring Buffer utility.
* Author: Phil Burk, http://www.softsynth.com
* License:
* PortAudio Portable Real-Time Audio Library 
* Copyright (c) 1999-2011 Ross Bencina and Phil Burk

* Permission is hereby granted, free of charge, to any person obtaining a copy of this
* software and associated documentation files (the "Software"), to deal in the Software
* without restriction, including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software, and to permit
* persons to whom the Software is furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE AND NONINFRINGEMENT. 
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
********************************************************************/

#ifndef RING_H
#define RING_H

#include "rtapi_bitops.h"
#include "rtapi_atomics.h"
#include "rtapi_string.h"
#include "rtapi_int.h"


#ifndef MAXIMUM // MAX conflicts with definition in hal/drivers/pci_8255.c
#define MAXIMUM(x, y) (((x) > (y))?(x):(y))
#endif

// use 32bit uints for ring indices and size fields
// so we can use 32bit atomics throughout (size_t on amd64 is 64bits)
typedef __u32 ringsize_t;

// size of a record in a record ring - in record mode,
// negative numbers are needed for skips.
typedef __s32 rrecsize_t;

typedef struct {
    ringsize_t tail __attribute__((aligned(RTAPI_CACHELINE)));
    char __tailpad[RTAPI_CACHELINE - sizeof(ringsize_t)];
    // offset 64:
    __u8 scratchpad_buf[0];  // actual scratchpad storage
} ringtrailer_t;


// the ringbuffer shared data
// defaults: record mode, no rmutex/wmutex use
// refcount mirrors the number of hal_ring_attach() operations but is held
// here since ringbuffers do not rely on HAL and may be used without HAL
// an example thereof is the error ring buffer in the global data segment

typedef enum {
    RINGTYPE_RECORD = 0,
    RINGTYPE_MULTIPART = RTAPI_BIT(0),
    RINGTYPE_STREAM = RTAPI_BIT(1),
    RINGTYPE_ANY  = (RTAPI_BIT(0)|RTAPI_BIT(1)),
    RINGTYPE_MASK = (RTAPI_BIT(0)|RTAPI_BIT(1))
} ring_type_t;

// mode flags passed in by ring_new
// exposed in ringheader_t.mode
typedef enum {
    USE_RMUTEX = RTAPI_BIT(2),
    USE_WMUTEX = RTAPI_BIT(3),
    ALLOC_HALMEM = RTAPI_BIT(4),
} ring_mode_flags_t;

typedef struct {
    __u8    type       : 2;  // RINGTYPE_*
    __u8    use_rmutex : 1;  // hint to using code - use ringheader_t.rmutex
    __u8    use_wmutex : 1;  // hint to using code - use ringheader_t.wmutex

    // allocate this ring in HAL shared memory rather than a separate shm segment
    // this is needed if pins/signals etc are allocated in the scratchpad area
    // because those objects must reside in HAL mem.
    // technically this option is a directive to the using HAL layer, not the
    // ringbuffer code per se.
    __u8    alloc_halmem : 1;

    __u32   userflags : 27;  // not interpreted by ringbuffer code
    // offset 4:
    __s32   refcount;        // number of referencing entities (modules, threads..)
    // offset 8:
    __s32   reader, writer;  // HAL comp or instance id's - informational
    // offset 16:
    __s32   reader_instance, writer_instance; // RTAPI instance id's
    // offset 24:
    rtapi_atomic_type rmutex, wmutex; // optional use - if used by multiple readers/writers
    // offset 32:
    ringsize_t trailer_size;   // sizeof(ringtrailer_t) + scratchpad size
    // offset 36:
    ringsize_t size_mask;      // stream mode only
    // offset 40:
    // this is the size of the actual ring buffer. There might be
    // padding between the ring storage and the ringtrailer_t due to the alignment
    // of the trailer (64) so the tail pointer is cache-aligned.
    ringsize_t size;           // common to stream and record mode
    // offset 44 - 4 bytes left:
    __u32 __unused1;
    // offset 48:
    __u64   generation;
    // offset 56:
    ringsize_t  head __attribute__((aligned(RTAPI_CACHELINE)));
    char __headpad[RTAPI_CACHELINE - sizeof(ringsize_t)];
    // offset 128:
    __u8    buf[0];         // actual ring storage without scratchpad
} ringheader_t;


// the ringbuffer shared data as made accessible
// to the using code by ringbuffer_init(), and hal_ring_attach()
// this structure lives in per-user memory, and refers to
// the ringheader structure, and buffers therein

#define RINGBUFFER_MAGIC 0x74737769

typedef struct {
    __s32           magic;
    ringheader_t   *header;
    ringtrailer_t  *trailer;
    __u8           *buf;
    void           *scratchpad;
} ringbuffer_t;

static inline int ringbuffer_attached(const ringbuffer_t *rb)
{
    return (rb->magic == RINGBUFFER_MAGIC);
}

// using layer data structures
typedef struct {
    const ringbuffer_t *ring;
    ringsize_t offset;
    __u64   generation;
} ringiter_t;

typedef struct {
    const void *rv_base;
    __u32       rv_flags; // meaningful only for multiframe ringvec_t's
    ringsize_t  rv_len;
} ringvec_t;


// record ring buffer size must be aligned on dword boundaries
// macro variant of size_aligned() for allocating at compile time
#define RB_ALIGN 8

// Round up X to closest upper alignment boundary
static inline ringsize_t size_aligned(const ringsize_t x)
{
    return RTAPI_ALIGN(x, RB_ALIGN);
}

// compute the next highest power of 2 of 32-bit v
// http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
static unsigned inline next_power_of_two(unsigned v) {
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}

// always cache-align the ring storage so ringtrailer_t lies on a
// cacheline boundary
static inline ringsize_t ring_storage_alloc(const int flags,
					    const ringsize_t size)
{
    if  ((flags & RINGTYPE_MASK) == RINGTYPE_STREAM) {
	// stream mode buffers need to be a power of two sized
	return RTAPI_CACHE_ALIGN(next_power_of_two(size));
    } else {
	// default to /* RINGTYPE_RECORD */
	// round up buffer size to closest upper alignment boundary
	return  RTAPI_CACHE_ALIGN(size_aligned(size));
    }
}

static inline ringsize_t ring_trailer_alloc(const ringsize_t sp_size)
{
    return size_aligned(sizeof(ringtrailer_t) + sp_size);
}

// the total size of the ringbuffer header plus storage for the ring
// and scratchpad
static inline ringsize_t ring_memsize(const int flags,
				      const ringsize_t size,
				      const ringsize_t  sp_size)
{
    return (ringsize_t) (sizeof(ringheader_t) +
			 ring_storage_alloc(flags,  size) +
			 ring_trailer_alloc(sp_size));
}

static inline int ring_refcount(ringheader_t *ringheader)
{
    return ringheader->refcount;
}

static inline ringtrailer_t *_trailer_from_header(const ringheader_t *ringheader)
{
    return (ringtrailer_t *) ((char *)ringheader->buf +
			      RTAPI_CACHE_ALIGN(ringheader->size));
}

static inline ringsize_t ring_scratchpad_size(const ringbuffer_t *ring)
{
    return ring->header->trailer_size - (ringsize_t) sizeof(ringtrailer_t);
}

// initialize a ringbuffer header and storage as already allocated
// with a size of ring_memsize(flags, size, sp_size)
// this will not clear the storage allocated.
static inline void ringheader_init(ringheader_t *ringheader,
				   const int flags,
				   const ringsize_t size,
				   const ringsize_t sp_size)
{
    ringtrailer_t *t;

    // layout the ring in memory:
    // first the ringheader_t struct
    // followed by the aligned ring buffer
    // followed by the ringtrailer_t
    // the latter includes scratchpad storage
    ringheader->size = ring_storage_alloc(flags, size);
    ringheader->trailer_size = ring_trailer_alloc(sp_size);

    // init the ringheader - mode independent part
    ringheader->rmutex = ringheader->wmutex = 0;
    ringheader->reader = ringheader->writer = 0;
    ringheader->reader_instance = ringheader->writer_instance = 0;
    ringheader->head = 0;
    t = _trailer_from_header(ringheader);
    t->tail = 0;
    ringheader->type = (flags & RINGTYPE_MASK);
    ringheader->use_rmutex = (flags & USE_RMUTEX) != 0;
    ringheader->use_wmutex = (flags & USE_WMUTEX) != 0;
    ringheader->alloc_halmem = (flags & ALLOC_HALMEM) != 0;

    // mode-dependent initialisation
    if (flags &  RINGTYPE_STREAM) {
	ringheader->size_mask = ringheader->size -1;
    } else {
	ringheader->generation = 0;
    }
    ringheader->refcount = 1;
}

// given a ringheader_t *, initialize a ringbuffer_t
// the latter is the per-user access structure and filled
// in e.g. in ring_attach()
// ringbuffer_t must point to allocated memory
static inline void ringbuffer_init(ringheader_t *ringheader,
				   ringbuffer_t *ring)
{
    // pass address of ringheader to caller
    ring->header = ringheader;
    ring->trailer = _trailer_from_header(ringheader);
    ring->buf = &ringheader->buf[0];

    // set pointer to scratchpad if used - stored
    // right behind buffer area
    if (ring_scratchpad_size(ring))
	ring->scratchpad = ring->trailer->scratchpad_buf;
    else
	ring->scratchpad = NULL;
    ring->magic = RINGBUFFER_MAGIC;
}

// memory layout of record rings:
//
// an RB_ALIGN aligned sequence of records:
// struct record {
//    rrecsize_t size;  // a int32_t
//    char data[0];
// }
// zero-length records are ok.
//
// when writing near the end of the buffer, and the remaining space
// is too small, a zero-length record with size -1 is added.
// this marks the current end of buffer, and tells the reader to retry
// the read at the beginning of the buffer.


// returns pointer to size field at a given offset
static inline rrecsize_t * _size_at(const ringbuffer_t *ring,
				    const ringsize_t off)
{
    return (rrecsize_t *) (ring->buf + off);
}

// for a record of a given size, return space used in a record ring buffer.
// used to dimension record rings
static inline ringsize_t record_usage(const ringsize_t record_size)
{
    return size_aligned(record_size + sizeof(rrecsize_t));
}

/* record_write_begin():
 *
 * begin a zero-copy write operation for up to sz bytes. This povides a buffer
 * to write to within free ringbuffer space.
 *
 * return 0 if there is sufficient space for the requested size
 * return EAGAIN if there is currently insufficient space
 * return ERANGE if the write size exceeds the ringbuffer size.
 *
 * The write needs to be committed with a corresponding record_write_end()
 * operation whose size argument must be less or equal to the size requested in
 * record_write_begin().
 *
 * This operation does not modifiy head or tail; record_write_end() does.
 * It also does not record the tentative write size in the ring; this is
 * only done in record_write_end() once the write is committed.
 *
 * 'sz' is used here to decide if an actual write of the size would wrap, and
 * return 'data' accordingly. The decision to wrap is committed though, even if
 * record_write_end() applies a smaller size which would have fit without
 * wrapping during record_write_begin().
 *
 * record_write_end() uses the 'data' field to decide if the decision was to
 * wrap or not.
 */
static inline int record_write_begin(ringbuffer_t *ring,
				     void ** data,
				     const ringsize_t sz)
{
    ringsize_t free;
    ringheader_t *h = ring->header;
    ringtrailer_t *t = ring->trailer;
    ringsize_t a = size_aligned(sz + sizeof(rrecsize_t));

    // record too large for ring?
    if (a > h->size)
	return ERANGE;

     // -1 + 1 is needed for head==tail
    free = (h->size +  rtapi_load_u32(&h->head) - t->tail - 1) % h->size + 1;

    //printf("Free space: %d; Need %zd\n", free, a);

    if (free <= a) return EAGAIN; // currently not enough space

    // would the write wrap around the end of ring?
    if (t->tail + a > h->size) {

	// would record fit at the start of the ring?
	if (rtapi_load_u32(&h->head) <= a)
	    // currently not
	    return EAGAIN;

	// yes, return address of tentative write at start of ring
	*data = _size_at(ring, 0) + 1;
	return 0;
    }
    // size would fit, return address post tail + size field
    *data = _size_at(ring, t->tail) + 1;
    return 0;
}

/* record_write_end()
 *
 * commit a zero-copy write to the ring which was started by record_write_begin().
 * 'sz' on record_write_end() must be less equal to the size requested in
 * the corresponding record_write_begin().
 * 'data' must be the pointer returned by record_write_begin().
 */
static inline int record_write_end(ringbuffer_t *ring,
				   const void * data,
				   const ringsize_t sz)
{
    ringheader_t *h = ring->header;
    ringtrailer_t *t = ring->trailer;

    ringsize_t a = size_aligned(sz + sizeof(rrecsize_t));

    // was the write at the beginning of the buffer?
    if (data == _size_at(ring, 0) + 1) {
	// Wrap case
	// invalidate the tail record
	rtapi_store_u32((__u32 *)_size_at(ring, t->tail), -1);
	rtapi_smp_wmb();
	rtapi_store_u32(&t->tail, 0);
    }
    // record comitted write size
    rtapi_store_u32((__u32 *)_size_at(ring, t->tail), sz);

    /* ensure that previous writes are seen before we update the write index
       (write after write)
    */
    rtapi_smp_wmb();

    rtapi_store_u32(&t->tail, (t->tail + a) % h->size);
    //printf("New head/tail: %zd/%zd\n", h->head, t->tail);
    return 0;
}

/* record_write()
 *
 * copying write operation from existing buffer/length
 * record boundaries will be preserved.
 *
 * return 0 on success
 * return EAGAIN if there is currently insufficient space
 * return ERANGE if the write size exceeds the ringbuffer size.
 */
static inline int record_write(ringbuffer_t *ring, void * data,
			       ringsize_t sz)
{
    void * ptr;
    int r = record_write_begin(ring, &ptr, sz);
    if (r) return r;
    memmove(ptr, data, sz);
    return record_write_end(ring, ptr, sz);
}

/* internal use function
 *
 * returns size and data address of the record at 'offset',
 * or EAGAIN if empty at 'offset'.
 */
static inline int _ring_read_at(const ringbuffer_t *ring,
				const ringsize_t offset,
				const void **data,
				ringsize_t *size)
{
    rrecsize_t *sz;
    ringtrailer_t *t = ring->trailer;

    if (offset == rtapi_load_u32(&t->tail))
	// nothing available
	return EAGAIN;

    // Make sure to serialize with respect to our snapshot
    // of the tail index
    rtapi_smp_rmb();

    sz = _size_at(ring, offset);
    if (*sz < 0)  // wrap mark
	// return size/data of record at start of ring
        return _ring_read_at(ring, 0, data, size);

    // non-wrapping case, sz >= 0:
    *size = (ringsize_t)*sz;
    *data = sz + 1;

    // Make sure copy is completed with respect to head
    // update.
    rtapi_smp_wmb();
    return 0;
}

/* record_read()
 *
 * non-copying read operation.
 *
 * Fail with EAGAIN if no data available.
 * Otherwise set data to point to the current record, and size to the size of the
 * current record.
 *
 * Note this is a 'peek' operation, it does not advance the read pointer to the next
 * record. To actually consume the record, call record_shift() after processing.
 */
static inline int record_read(const ringbuffer_t *ring,
			      const void **data,
			      ringsize_t *size)
{
    return _ring_read_at(ring, ring->header->head, data, size);
}

/* record_next()
 *
 * test for data available. Return zero if none, else pointer to data
 * of the next available record.
 */
static inline const void *record_next(const ringbuffer_t *ring)
{
    const void *data;
    ringsize_t size;
    if (record_read(ring, &data, &size)) return 0;
    return data;
}

/* record_next_size()
 *
 * retrieve the size of the next available record.
 * Return -1 if no data available.
 *
 * Note that zero-length records are supported and valid.
 */
static inline rrecsize_t record_next_size(const ringbuffer_t *ring)
{
    const void *data;
    ringsize_t size;
    if (record_read(ring, &data, &size)) return -1;
    return size;
}

/* record_write_space()
 *
 * returns the largest contiguous block available
 * such that record_write(h, data, record_write_space(h))
 * will succeed
 *
 * Note this does not retrieve the amount of free space in the buffer which
 * might be larger than the value returned; however, this space maye not be written
 * by records larger than returned by record_write_space() (i.e. many small
 * writes may be possible which are in sum larger than the value returned here).
 */
static inline ringsize_t record_write_space(const ringheader_t *h)
{
    int avail = 0;
    ringtrailer_t *t =  _trailer_from_header(h);

    ringsize_t head = rtapi_load_u32(&h->head);

    if (t->tail < head)
        avail = head - t->tail;
    else
        avail = MAXIMUM(head, h->size - t->tail);
    return MAXIMUM(0, avail - (2 * RB_ALIGN));
}

/* helper for sizing a record mode ringbuffer
 * given the size of an record, it will return the space used
 * in the ring buffer, including any overhead and
 * alignment requirements
 *
 * example: to hold 1000 records of struct foo a ringbuffer will need
 * a size of at least record_space(sizeof(struct foo)) * 1000
 *
 * (to be on the safe side, add some headroom to that)
 */
static inline ringsize_t record_space(ringsize_t element)
{
    return size_aligned(element + sizeof(rrecsize_t));
}


/* internal function */
static inline rrecsize_t _ring_shift_offset(const ringbuffer_t *ring,
					    const ringsize_t offset)
{
    rrecsize_t size;
    ringheader_t *h = ring->header;
    ringtrailer_t *t = ring->trailer;

    if (h->head == rtapi_load_u32(&t->tail))
	return -1;

    // ensure that previous reads (copies out of the ring buffer) are always completed 
    // before updating (writing) the read index. 
    // (write-after-read) => full barrier
    rtapi_smp_rmb();

    size = *_size_at(ring, offset);
    if (size < 0)
	return _ring_shift_offset(ring, 0);
    size = size_aligned(size + sizeof(rrecsize_t));
    return (offset + size) % h->size;
}

/* record_shift()
 *
 * consume a record in the ring buffer.
 * used with a corresponding record_read().
 *
 * return 0 on success
 * return EAGAIN if nothing to consume.
 *
 * example processing loop:
 *
 * void *data;
 * ringsize_t size;
 *
 * while (record_read(ring, &data, &size) == 0) {
 *    // process(data,size)
 *    record_shift(ring);
 * }
 *
 * NB: without record_shift(), record_next/record_next_size
 * are in effect a peek operation
 */
static inline int record_shift(ringbuffer_t *ring)
{
    rrecsize_t off = _ring_shift_offset(ring,  // does the barrier
					 ring->header->head);
    if (off < 0) return EAGAIN;

    rtapi_inc_u64((uint64_t *)&ring->header->generation);
    rtapi_store_u32(&ring->header->head, off);
    return 0;
}

/* record_flush_reader()
 *
 * clear the buffer, and return the number of records flushed.
 * call from reader only (!)
 */
static inline int record_flush_reader(ringbuffer_t *ring)
{
    int count = 0;

    while (!record_shift(ring))
	count++;
    return count;
}

/* record_flush()
 *
 * clear the buffer
 * should work from reader or writer
 */
static inline void record_flush(ringbuffer_t *ring)
{
    ringtrailer_t *t =  _trailer_from_header(ring->header);

    // set head to match tail with a CAS loop
    do {
	rtapi_inc_u64((uint64_t *)&ring->header->generation);
    } while (!rtapi_cas_u32(&ring->header->head,
			   rtapi_load_u32(&ring->header->head),
			   rtapi_load_u32(&t->tail)));
}

/* rings by default behave like queues:
 * - record_write() to add
 * - record_read()/record_shift() to remove.
 *
 * to make a ring behave as a circular buffer, use this scheme:
 *
 * write side: gain space by consuming records if needed:
 *
 * while (record_write(...) == EAGAIN)
 *    record_shift();
 *
 * the read side uses iterators like so:
 *
 * ringiter_t ri;
 * record_iter_init(ringbuffer, &ri);
 * while (1) {
 *    while ((result = record_iter_read(&ri,&data, &size)) == EINVAL)
 *       record_iter_init(ringbuffer, &ri); // renew
 *    if (result == EAGAIN)
 *       sleep(duration);
 *       continue;
 *    copy data to local buffer;
 *    if (record_iter_shift(&ri)) == EINVAL)
 *       continue;
 *    consume_data(localbuffer);
 */

// iterator functions

static inline int record_iter_init(const ringbuffer_t *ring,
				   ringiter_t *iter)
{
    iter->ring = ring;
    iter->generation = rtapi_load_u64((uint64_t *)&ring->header->generation);
    iter->offset = rtapi_load_u32(&ring->header->head);
    if (rtapi_load_u64((uint64_t *)&ring->header->generation) != iter->generation)
        return EAGAIN;
    return 0;
}

static inline int record_iter_invalid(const ringiter_t *iter)
{
    if (rtapi_load_u64((uint64_t *)&iter->ring->header->generation) >
	iter->generation)
        return EINVAL;
    return 0;
}

static inline int record_iter_shift(ringiter_t *iter)
{
    rrecsize_t off;

    if (record_iter_invalid(iter)) return EINVAL;
    off = _ring_shift_offset(iter->ring, iter->offset);
    if (off < 0) return EAGAIN;

    rtapi_inc_u64((uint64_t *)&iter->generation);
    iter->offset = off;
    return 0;
}

static inline int record_iter_read(const ringiter_t *iter,
				   const void **data,
				   ringsize_t *size)
{
    if (record_iter_invalid(iter)) return EINVAL;

//    printf("Read at %zd\n", iter->offset);
    return _ring_read_at(iter->ring, iter->offset, data, size);
}

// observer accessors:

static inline int ring_isstream(const ringbuffer_t *ring)
{
    return (ring->header->type == RINGTYPE_STREAM);
}

static inline int ring_ismultipart(const ringbuffer_t *ring)
{
    return (ring->header->type == RINGTYPE_MULTIPART);
}


static inline int ring_use_wmutex(const ringbuffer_t *ring)
{
    return ring->header->use_wmutex ;
}

static inline int ring_use_rmutex(const ringbuffer_t *ring)
{
    return ring->header->use_rmutex;
}


//  SMP barriers adapted from:
/*
 * $Id: pa_ringbuffer.c 1738 2011-08-18 11:47:28Z rossb $
 * Portable Audio I/O Library
 * Ring Buffer utility.
 *
 * Author: Phil Burk, http://www.softsynth.com
 * modified for SMP safety on Mac OS X by Bjorn Roche
 * modified for SMP safety on Linux by Leland Lucius
 * also, allowed for const where possible
 * modified for multiple-byte-sized data elements by Sven Fischer
 *
 * Note that this is safe only for a single-thread reader and a
 * single-thread writer.
 *
 * This program uses the PortAudio Portable Audio Library.
 * For more information see: http://www.portaudio.com
 * Copyright (c) 1999-2000 Ross Bencina and Phil Burk
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * The text above constitutes the entire PortAudio license; however,
 * the PortAudio community also makes the following non-binding requests:
 *
 * Any person wishing to distribute modifications to the Software is
 * requested to send the modifications to the original developer so that
 * they can be incorporated into the canonical version. It is also
 * requested that these non-binding requests be included along with the
 * license above.
 */


//  MODE_STREAM ring operations: adapted from the jack2
//  https://github.com/jackaudio/jack2/blob/master/common/ringbuffer.c
//  XXX: I understand this to be compatible with the RTAPI license


/*
  Copyright (C) 2000 Paul Davis
  Copyright (C) 2003 Rohan Drape

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 2.1 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

  ISO/POSIX C version of Paul Davis's lock free ringbuffer C++ code.
  This is safe for the case of one read thread and one write thread.
*/

// the stream 'methods'
/* void   stream_get_read_vector(const ringbuffer_t *ring, */
/* 				  ringvec_t *vec); */
/* void   stream_get_write_vector(const ringbuffer_t *ring, */
/* 				   ringvec_t *vec); */
/* ringsize_t stream_read(ringbuffer_t *ring, char *dest, ringsize_t cnt); */
/* ringsize_t stream_peek(ringbuffer_t *ring, char *dest, ringsize_t cnt); */
/* void   stream_read_advance(ringbuffer_t *ring, ringsize_t cnt); */
/* ringsize_t stream_read_space(const ringheader_t *h); */
/* void   stream_flush(ringbuffer_t *ring); */
/* ringsize_t stream_write(ringbuffer_t *ring, const char *src, */
/* 			ringsize_t cnt); */
/* void   stream_write_advance(ringbuffer_t *ring, ringsize_t cnt); */
/* ringsize_t stream_write_space(const ringheader_t *h); */

/* The non-copying data reader.  `vec' is an array of two places.  Set
 * the values at `vec' to hold the current readable data at `ring'.  If
 * the readable data is in one segment the second segment has zero
 * length.
 */
static inline ringsize_t stream_get_read_vector(const ringbuffer_t *ring,
						ringvec_t *vec)
{
    ringsize_t free_cnt;
    ringsize_t cnt2;
    ringsize_t w, r;
    ringheader_t *h = ring->header;
    ringtrailer_t *t = ring->trailer;

    w = rtapi_load_u32(&t->tail);
    r = h->head;

    if (w > r) {
	free_cnt = w - r;
    } else {
	free_cnt = (w - r + h->size) & h->size_mask;
    }
    cnt2 = r + free_cnt;
    if (cnt2 > h->size) {
	/* Two part vector: the rest of the buffer after the current write
	   ptr, plus some from the start of the buffer. */
	vec[0].rv_base = (void *) &(ring->buf[r]);
	vec[0].rv_len = h->size - r;
	vec[1].rv_base = (void *) ring->buf;
	vec[1].rv_len = cnt2 & h->size_mask;

    } else {
	/* Single part vector: just the rest of the buffer */
	vec[0].rv_base = (void *) &(ring->buf[r]);
	vec[0].rv_len = free_cnt;
	vec[1].rv_len = 0;
	vec[1].rv_base = NULL;
    }
    return vec[0].rv_len + vec[1].rv_len;
}

/* The non-copying data writer.  `vec' is an array of two places.  Set
 * the values at `vec' to hold the current writeable data at `ring'.  If
 * the writeable data is in one segment the second segment has zero
 * length.
 */
static inline void stream_get_write_vector(const ringbuffer_t *ring,
					   ringvec_t *vec)
{
    ringsize_t free_cnt;
    ringsize_t cnt2;
    ringsize_t w, r;
    ringheader_t *h = ring->header;
    ringtrailer_t *t = ring->trailer;

    w = t->tail;
    r = rtapi_load_u32(&h->head);

    if (w > r) {
	free_cnt = ((r - w + h->size) & h->size_mask) - 1;
    } else if (w < r) {
	free_cnt = (r - w) - 1;
    } else {
	free_cnt = h->size - 1;
    }
    cnt2 = w + free_cnt;
    if (cnt2 > h->size) {
	// Two part vector: the rest of the buffer after the current write
	// ptr, plus some from the start of the buffer.
	vec[0].rv_base = (void *) &(ring->buf[w]);
	vec[0].rv_len = h->size - w;
	vec[1].rv_base = (void *) ring->buf;
	vec[1].rv_len = cnt2 & h->size_mask;
    } else {
	vec[0].rv_base = (void *) &(ring->buf[w]);
	vec[0].rv_len = free_cnt;
	vec[1].rv_len = 0;
    }
    if (free_cnt)
	rtapi_smp_mb();
}


/* Return the number of bytes available for reading.  This is the
 * number of bytes in front of the read pointer and behind the write
 * pointer.
 */
static inline ringsize_t stream_read_space(const ringheader_t *h)
{
    ringsize_t w, r;
    ringtrailer_t *t =  _trailer_from_header(h);

    w = rtapi_load_u32(&t->tail);
    r = h->head;

    if (w > r) {
	return w - r;
    } else {
	return (w - r + h->size) & h->size_mask;
    }
}

/* The copying data reader.  Copy at most `cnt' bytes from `ring' to
 * `dest'.  Returns the actual number of bytes copied.
 */
static inline ringsize_t stream_read(ringbuffer_t *ring,
				     char *dest,
				     const ringsize_t cnt)
{
    ringsize_t free_cnt;
    ringsize_t cnt2;
    ringsize_t to_read;
    ringsize_t n1, n2;
    ringheader_t *h = ring->header;

    if ((free_cnt = stream_read_space (h)) == 0) {
	return 0;
    }
    to_read = cnt > free_cnt ? free_cnt : cnt;
    cnt2 = h->head + to_read;
    if (cnt2 > h->size) {
	n1 = h->size - h->head;
	n2 = cnt2 & h->size_mask;
    } else {
	n1 = to_read;
	n2 = 0;
    }
    /*
     * Make sure to serialize with respect to our snapshot
     * of the tail counter acquired in stream_read_space
     */
    rtapi_smp_rmb();

    memcpy (dest, &(ring->buf[h->head]), n1);

    if (n2) {
	memcpy (dest + n1, &(ring->buf[h->head + n1]), n2);
	rtapi_smp_wmb();
	rtapi_store_u32(&h->head, (h->head + n1 + n2) & h->size_mask);
    } else {
	rtapi_smp_wmb();
	rtapi_store_u32(&h->head, (h->head + n1) & h->size_mask);
    }
    return to_read;
}


/* The copying data reader w/o read pointer advance.  Copy at most
 * `cnt' bytes from `ring' to `dest'.  Returns the actual number of bytes
 * copied.
 */
static inline ringsize_t stream_peek(ringbuffer_t *ring,
				     char *dest,
				     const ringsize_t cnt)
{
    ringsize_t free_cnt;
    ringsize_t cnt2;
    ringsize_t to_read;
    ringsize_t n1, n2;
    ringsize_t tmp_head;
    ringheader_t *h = ring->header;

    tmp_head = rtapi_load_u32(&h->head);
    if ((free_cnt = stream_read_space (h)) == 0) {
	return 0;
    }
    to_read = cnt > free_cnt ? free_cnt : cnt;
    cnt2 = tmp_head + to_read;
    if (cnt2 > h->size) {
	n1 = h->size - tmp_head;
	n2 = cnt2 & h->size_mask;
    } else {
	n1 = to_read;
	n2 = 0;
    }
    memcpy (dest, &(ring->buf[tmp_head]), n1);
    tmp_head = (tmp_head + n1) & h->size_mask;

    if (n2) {
	memcpy (dest + n1, &(ring->buf[tmp_head]), n2);
    }
    return to_read;
}


/* Advance the read pointer `cnt' places. */
static inline void stream_read_advance(ringbuffer_t *ring,
				       const ringsize_t cnt)
{
    ringheader_t *h = ring->header;

    /* ensure that previous reads (copies out of the ring buffer) are always
       completed before updating (writing) the read index.
       (write-after-read) => full barrier
    */
    rtapi_smp_wmb();
    rtapi_store_u32(&h->head, (h->head + cnt) & h->size_mask);
}


/* Flush any pending data from ring.
 *
 * return number of bytes flushed.
 */
static inline ringsize_t stream_flush(ringbuffer_t *ring)
{
    ringsize_t left = stream_read_space(ring->header);
    if (left > 0)
	stream_read_advance(ring, left);
    return left;
}

/* Return the number of bytes available for writing.  This is the
 * number of bytes in front of the write pointer and behind the read
 * pointer.
 */
static inline ringsize_t stream_write_space(const ringheader_t *h)
{
    ringsize_t w, r;
    ringtrailer_t *t =  _trailer_from_header(h);

    w = t->tail;
    r = rtapi_load_u32(&h->head);

    if (w > r) {
	return ((r - w + h->size) & h->size_mask) - 1;
    } else if (w < r) {
	return (r - w) - 1;
    } else {
	return h->size - 1;
    }
}

/* The copying data writer.  Copy at most `cnt' bytes to `ring' from
 * `src'.  Returns the actual number of bytes copied.
 */
static inline ringsize_t stream_write(ringbuffer_t *ring,
				      const char *src,
				      const ringsize_t cnt)
{
    ringsize_t free_cnt;
    ringsize_t cnt2;
    ringsize_t to_write;
    ringsize_t n1, n2;
    ringheader_t *h = ring->header;
    ringtrailer_t *t = ring->trailer;

    if ((free_cnt = stream_write_space (h)) == 0) {
	return 0;
    }
    to_write = cnt > free_cnt ? free_cnt : cnt;
    cnt2 = t->tail + to_write;
    if (cnt2 > h->size) {
	n1 = h->size - t->tail;
	n2 = cnt2 & h->size_mask;
    } else {
	n1 = to_write;
	n2 = 0;
    }
    memcpy (&(ring->buf[t->tail]), src, n1);

    if (n2) {
	memcpy (&(ring->buf[t->tail + n1]), src + n1, n2);
	rtapi_smp_wmb();
	rtapi_store_u32(&t->tail, (t->tail + n1 + n2) & h->size_mask);
    } else {
	rtapi_smp_wmb();
	rtapi_store_u32(&t->tail,(t->tail + n1) & h->size_mask);
    }
    return to_write;
}

/* Advance the write pointer `cnt' places. */
static inline void stream_write_advance(ringbuffer_t *ring,
					const ringsize_t cnt)
{
    ringheader_t *h = ring->header;
    ringtrailer_t *t = ring->trailer;

    /* ensure that previous writes are seen before we update the write index
       (write after write)
    */
    rtapi_smp_wmb();
    rtapi_store_u32(&t->tail, (t->tail + cnt) & h->size_mask);
}

#endif // RING_H
