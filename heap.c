/*
 * File: heap.c
 *
 *   Implementation of library functions manipulating a min priority queue.
 *     Feel free to add, remove, or modify these library functions to serve
 *     your algorithm.
 *
 * Jose @ ShanghaiTech University
 *
 */

#include <stdlib.h>     /* malloc, free */
#include <limits.h>     /* INT_MAX */

#include "heap.h"

/*
 * Initialize a min heap. The heap is constructed using array-based
 *   implementation. Returns the pointer to the new heap.
 *
 */
heap_t *
heap_init(int channel, LESS_THAN less)
{
    heap_t *h = malloc(sizeof(heap_t));
    h->channel = channel;
    h->size = 0;
    h->capacity = INIT_CAPACITY;    /* Initial capacity = 1000. */
    h->nodes = malloc(INIT_CAPACITY * sizeof(node_t *));
    h->nodes[0] = malloc(sizeof(node_t));
    h->less = less;
    return h;
}

/*
 * Delete the memory occupied by the min heap H.
 *
 */
void
heap_destroy (heap_t *h)
{
    free(h->nodes[0]);
    free(h->nodes);
    free(h);
}

/*
 * Insert a node N into the min heap H.
 *
 */
void
heap_insert (heap_t *h, node_t *n)
{
    int cur = ++h->size;    /* Index 0 lays dummy node, so increment first. */

    /* If will exceed current capacity, doubles the capacity. */
    if (h->size == h->capacity) {
        h->capacity *= 2;
        h->nodes = realloc(h->nodes, h->capacity * sizeof(node_t *));
    }

    h->nodes[h->size] = n;
    while (h->less(n, h->nodes[cur / 2], h->channel) && cur >= 2) {
        h->nodes[cur] = h->nodes[cur / 2];
        h->nodes[cur]->heap_id[h->channel] = cur;
        cur /= 2;
    }
    h->nodes[cur] = n;
    n->heap_id[h->channel] = cur;
}

/*
 * Extract the root (i.e. the minimum node) in min heap H. Returns the pointer
 *   to the extracted node.
 *
 */
node_t *
heap_extract (heap_t *h)
{
    node_t *ret = h->nodes[1];
    node_t *last = h->nodes[h->size--];
    int cur, child;
    for (cur = 1; 2 * cur <= h->size; cur = child) {
        child = 2 * cur;
        if (child < h->size && h->less(h->nodes[child + 1], h->nodes[child], h->channel))
            child++;
        if (h->less(h->nodes[child], last, h->channel)) {
            h->nodes[cur] = h->nodes[child];
            h->nodes[cur]->heap_id[h->channel] = cur;
        } else
            break;
    }
    h->nodes[cur] = last;
    last->heap_id[h->channel] = cur;
    return ret;
}

/*
 * Extract the root (i.e. the minimum node) in min heap H.
 * This doesn't remove the root item from the heap
 */
node_t *heap_peek(heap_t *h)
{
    return h->nodes[1];
}

/*
 * Update the min heap H in case that node N has changed its f-score.
 *
 */
void
heap_update (heap_t *h, node_t *n)
{
    int cur = n->heap_id[h->channel];
    while (h->less(n, h->nodes[cur / 2], h->channel) && cur >= 2) {
        h->nodes[cur] = h->nodes[cur / 2];
        h->nodes[cur]->heap_id[h->channel] = cur;
        cur /= 2;
    }
    h->nodes[cur] = n;
    n->heap_id[h->channel] = cur;
}
