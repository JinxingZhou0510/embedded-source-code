#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ring buffer */
struct rt_ringbuffer
{
    unsigned char *buffer_ptr;
    /* use the msb of the {read,write}_index as mirror bit. You can see this as
     * if the buffer adds a virtual mirror and the pointers point either to the
     * normal or to the mirrored buffer. If the write_index has the same value
     * with the read_index, but in a different mirror, the buffer is full.
     * While if the write_index and the read_index are the same and within the
     * same mirror, the buffer is empty. The ASCII art of the ringbuffer is:
     *
     *          mirror = 0                    mirror = 1
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 | Full
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     *  read_idx-^                   write_idx-^
     *
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 | Empty
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * read_idx-^ ^-write_idx
     *
     * The tradeoff is we could only use 32KiB of buffer for 16 bit of index.
     * But it should be enough for most of the cases.
     *
     * Ref: http://en.wikipedia.org/wiki/Circular_buffer#Mirroring */
    unsigned short read_mirror : 1;
    unsigned short read_index : 15;
    unsigned short write_mirror : 1;
    unsigned short write_index : 15;
    /* as we use msb of index as mirror bit, the size should be signed and
     * could only be positive. */
    signed short buffer_size;
};
/**
 * RingBuffer for DeviceDriver
 *
 * Please note that the ring buffer implementation of RT-Thread
 * has no thread wait or resume feature.
 */
void rt_ringbuffer_init(struct rt_ringbuffer *rb,
                        unsigned char           *pool,
                        signed short            size);
unsigned short rt_ringbuffer_put(struct rt_ringbuffer *rb,
                            const unsigned char     *ptr,
                            unsigned short           length);
unsigned short rt_ringbuffer_put_force(struct rt_ringbuffer *rb,
                                  const unsigned char     *ptr,
                                  unsigned short           length);
unsigned short rt_ringbuffer_putchar(struct rt_ringbuffer *rb,
                                const unsigned char      ch);
unsigned short rt_ringbuffer_putchar_force(struct rt_ringbuffer *rb,
                                      const unsigned char      ch);
unsigned short rt_ringbuffer_get(struct rt_ringbuffer *rb,
                            unsigned char           *ptr,
                            unsigned short           length);
unsigned short rt_ringbuffer_peek(struct rt_ringbuffer *rb,
                            unsigned char           *ptr,
                            unsigned short           length);
unsigned short rt_ringbuffer_getchar(struct rt_ringbuffer *rb, unsigned char *ch);

enum rt_ringbuffer_state
{
    RT_RINGBUFFER_EMPTY,
    RT_RINGBUFFER_FULL,
    /* half full is neither full nor empty */
    RT_RINGBUFFER_HALFFULL,
};

static __inline unsigned short rt_ringbuffer_get_size(struct rt_ringbuffer *rb)
{
    return rb->buffer_size;
}

static __inline enum rt_ringbuffer_state
rt_ringbuffer_status(struct rt_ringbuffer *rb)
{
    if (rb->read_index == rb->write_index)
    {
        if (rb->read_mirror == rb->write_mirror)
            return RT_RINGBUFFER_EMPTY;
        else
            return RT_RINGBUFFER_FULL;
    }
    return RT_RINGBUFFER_HALFFULL;
}

/** return the size of data in rb */
static __inline unsigned short rt_ringbuffer_data_len(struct rt_ringbuffer *rb)
{
    switch (rt_ringbuffer_status(rb))
    {
    case RT_RINGBUFFER_EMPTY:
        return 0;
    case RT_RINGBUFFER_FULL:
        return rb->buffer_size;
    case RT_RINGBUFFER_HALFFULL:
    default:
        if (rb->write_index > rb->read_index)
            return rb->write_index - rb->read_index;
        else
            return rb->buffer_size - (rb->read_index - rb->write_index);
    };
}

/** return the size of empty space in rb */
#define rt_ringbuffer_space_len(rb) ((rb)->buffer_size - rt_ringbuffer_data_len(rb))

#ifdef __cplusplus
}
#endif

#endif
