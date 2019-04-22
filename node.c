/*
 * File: node.c
 *
 *   Implementation of library functions manipulating a block cell node. Feel
 *     free to add, remove, or modify these library functions to serve your
 *     algorithm.
 *
 * Jose @ ShanghaiTech University
 *
 */

#include <stdlib.h>     /* NULL, malloc, free */
#include <limits.h>     /* INT_MAX */
#include "node.h"

/* Extension to add operator, considering operation between infinity */
int add_extension(int X, int Y)
{
    X = _MAX_(X, Y), Y = _MIN_(X, Y);
    if(X == INT_MAX && Y == -INT_MAX){
        return 0;
    }else if(X == INT_MAX){
        return INT_MAX;
    }else if(Y == -INT_MAX){
        return -INT_MAX;
    }else{
        return X + Y;
    }
}

/*
 * Initialize a node whose position is recorded at (X, Y) with type MARK.
 *   Returns the pointer to the new node.
 *
 */
node_t *
node_init (int x, int y, mark_t mark)
{
    node_t *n = malloc(sizeof(node_t));
    n->x = x;
    n->y = y;

    n->gs[0] = INT_MAX;
    n->gs[1] = INT_MAX;

    n->fs[0] = INT_MAX;
    n->fs[1] = INT_MAX;
    
    n->mark = mark;

    n->heap_id[0] = 0;
    n->heap_id[1] = 0;

    n->opened = false;
    n->closed = false;

    n->parent[0] = NULL;
    n->parent[1] = NULL;

    return n;
}

/*
 * Delete the memory occupied by the node N.
 *
 */
void
node_destroy (node_t *n)
{
    free(n);
}

/*
 * Defines the comparison method between nodes, that is, comparing their
 * A* f-scores.
 *
 */
bool node_cost_less(node_t *n1, node_t *n2, int channel)
{
    return n1->fs[channel] < n2->fs[channel];
}

/*
 * Defines the comparison method between nodes, that is, comparing their
 * coordinates. The primary order is x, and secondary order is y.
 *
 */
bool node_coord_less(node_t *n1, node_t *n2, int channel)
{
    UNUSED(channel);
    if(n1->x < n2->x){
        return true;
    }else if(n1->x == n2->x){
        if(n1->y < n2->y) return true;
    }
    return false;
}
