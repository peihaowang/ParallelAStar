/*
 * File: node.h
 *
 *   Declaration of a block cell node data structure and library functions
 *     manipulating the node. Feel free to add, remove, or modify these
 *     declarations to serve your algorithm.
 *
 * Jose @ ShanghaiTech University
 *
 */

#ifndef _NODE_H_
#define _NODE_H_

/*
 * Introduce the "bool" type. Though I really love C/C++, I would still say
 *   it is stupid for not even defining a standard "bool" type in ANSI C
 *   standard.
 *
 */
typedef int bool;
#define true  (1)
#define false (0)

/* This macro are useful to suppress the unsued variable warnings */
#define UNUSED(VAR) (void)(VAR)

/* Definition of max and min macro */
#define _MAX_(X, Y) (X > Y ? X : Y)
#define _MIN_(X, Y) (X < Y ? X : Y)


/* Extension to add operator, considering operation between infinity */
int add_extension(int X, int Y);

/* Add operator considering infinity as operand */
#define _ADD_(X, Y) add_extension(X, Y)

/* Subtraction operator considering infinity as operand */
#define _SUB_(X, Y) add_extension(X, -Y)

/* Define the channel number of each node, i.e. how many heap will
 * hold such nodes.
 */
#define CHANNEL_NUMBERS     3
#define CHANNEL_THREAD_ONE  0
#define CHANNEL_THREAD_TWO  1
#define CHANNEL_SORTING     2

/*
 * Explicitly defines the four different node types.
 *   START: start point
 *   GOAL:  goal point
 *   WALL:  wall block, cannot step on / move across
 *   NONE:  normal blank point
 *
 */
typedef enum {START, GOAL, WALL, NONE} mark_t;

/*
 * Structure of a block cell node.
 *
 */
typedef struct node_t
{
    int x;          /* X coordinate, starting from 0. */
    int y;          /* Y coordinate, starting from 0. */
    int gs[2];      /* A* g-score. */
    int fs[2];      /* A* f-score. */
    mark_t mark;    /* Node type. */
    /* Position on min heap, used by updating. */
    int heap_id[CHANNEL_NUMBERS];
    bool opened;    /* Has been discovered? */
    bool closed;    /* Has been closed? */
      /* Parent node along the path. */
    struct node_t *parent[2];
} node_t;

/* Function prototypes. */
node_t *node_init (int x, int y, mark_t mark);
void node_destroy (node_t *n);

bool node_cost_less (node_t *n1, node_t *n2, int channel);
bool node_coord_less (node_t *n1, node_t *n2, int channel);

#endif
