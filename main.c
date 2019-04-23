/*
 * File: main.c
 *
 *   Main body of A* path searching algorithm on a block maze. The maze is
 *     given in a source file, whose name is put as a command-line argument.
 *     Then it should perform an A* search on the maze and prints the steps
 *     along the computed shortest path back to the file.
 *
 *     * Heuristic function chosen as the "Manhattan Distance":
 *
 *         heuristic(n1, n2) = |n1.x - n2.x| + |n1.y - n2.y|
 *
 *     * The whole procedure, including I/O and computing, will be time-
 *         ticked. So you are allowed to modify and optimize everything in
 *         this file and in all the libraries, as long as it satisfies:
 *
 *         1. It performs an A* path searching algorithm on the given maze.
 *
 *         2. It computes one SHORTEST (exactly optimal) path and prints the
 *              steps along the shortest path back to file, just as the
 *              original version.
 *
 *         3. Compiles with the given "Makefile". That means we are using
 *              (and only manually using) "pthread" for parallelization.
 *              Further parallelization techniques, such as OpenMP and SSE,
 *              are not required (and not allowed).
 *
 *         4. If there are multiple shortest paths, any one of them will be
 *              accepted. Please make sure you only print exactly one valid
 *              path to the file.
 *
 * Jose @ ShanghaiTech University
 *
 */

#ifndef __APPLE__

/* With which C standard (C89, C99, C11, etc.) you are compiling your code
 * decides which features are available. pthread_barrier is just an example
 * not present under C89. The following macro selects the version of POSIX, 
 * or the X/Open specification. Using 200112L for POSIX 2001, which introduced
 * pthread_barrier in.
 */
#define _POSIX_C_SOURCE 200112L

#endif

#include <stdlib.h>     /* NULL */
#include <assert.h>     /* assert */
#include <time.h>       /* clock */
#include <limits.h>     /* INT_MAX */

#include <pthread.h>    /* Multithreading */
#ifdef __APPLE__
#include "pthread_barrier.h"
#endif

#include "heap.h"
#include "node.h"
#include "maze.h"
#include "compass.h"    /* The heuristic. */

/*****************************************************************************/

#ifdef __PERFORMANCE_METRIC__

#define SW_MAX_COUNT    5
#define SW_0            0
#define SW_1            1
#define SW_2            2        
#define SW_3            3    
#define SW_4            4

typedef struct stopwatch_t{
    time_t first;
    time_t last;
} stopwatch_t;

static stopwatch_t g_stopwatch[SW_MAX_COUNT];

void __stopwatch_reset(stopwatch_t *sw)
{
    if(sw != NULL) sw->first = sw->last = clock();
}

float __stopwatch_tick(stopwatch_t* sw, const char* message)
{
    float secs = -1.0f;
    if(sw != NULL){
        secs = (float)(clock() - sw->last) / (float)CLOCKS_PER_SEC;
        if(message != NULL) printf("%s: %f s\n", message, secs);
        else printf("%f s\n", secs);
        sw->last = clock();
    }
    return secs;
}

float __stopwatch_escape(stopwatch_t* sw, const char* message)
{
    float secs = -1.0f;
    if(sw != NULL){
        secs = (float)(clock() - sw->first) / (float)CLOCKS_PER_SEC;
#ifdef __PERFORMANCE_METRIC__
        if(message != NULL) printf("%s: %f s\n", message, secs);
        else printf("%f s\n", secs);
#else
        UNUSED(message);
#endif
    }
    return secs;
}

#define SW_RESET(N)         __stopwatch_reset(&g_stopwatch[N])
#define SW_TICK(N, S)       __stopwatch_tick(&g_stopwatch[N], S)
#define SW_ESCAPE(N, S)     __stopwatch_escape(&g_stopwatch[N], S)

#else

#define SW_RESET(N)
#define SW_TICK(N, S)
#define SW_ESCAPE(N, S)

#endif

/*****************************************************************************/

/* Local helper functions. */
static node_t *fetch_neighbour (maze_t *m, node_t *n, int direction);

/*
 * Arguments and settings for each thread
 */
typedef struct pnba_arguments_t{
    int id;
    node_t* start;
    node_t *goal;
} pnba_arguments_t;

/*
 * Multithread context, including shared information for
 * interaction and parallelism
 */
static struct pnba_context_t{
    bool finished;

    maze_t* maze;

    int F[2];
    int L;
    node_t* joint;

    /* Threads. Thread objects should be put on static data */
    pthread_t threads[2];
    /* Shared mutex lock for updating Lambda */
    pthread_mutex_t mutexL;
    /* Shared barrier lock to synchronize the initialization */
    pthread_barrier_t barrierInit;
    /* Shared barrier lock to synchronize each step(loop) in pnba */
    pthread_barrier_t barrierStep;

} g_cxt;

/* Output the cost of path and the joint point of two search process */
#ifdef __PERFORMANCE_METRIC__

void __print_pathinfo()
{
    printf("===== Path Info =====\n");
    printf("Joint: %d, %d\n", g_cxt.joint->x, g_cxt.joint->y);
    printf("Cost: %d \n", g_cxt.L);
    printf("=====================\n");
}

#define PRT_PATHINFO()      __print_pathinfo()

#else

#define PRT_PATHINFO()

#endif

/*****************************************************************************/

void* search_thread(void *arguments)
{
    heap_t *openset;
    pnba_arguments_t *pnba_args = (pnba_arguments_t*)arguments;
    /* Note that, here pid means partner id */
    int id = pnba_args->id, pid = 1 - id;

    openset = heap_init(id, node_cost_less);
    pnba_args->start->gs[id] = 0;
    pnba_args->start->fs[id] = heuristic(pnba_args->start, pnba_args->goal);
    g_cxt.F[id] = pnba_args->start->fs[id];
    heap_insert(openset, pnba_args->start);

    pthread_barrier_wait(&g_cxt.barrierInit);

    /* Loop and repeatedly extracts the node with the highest f-score to
       process on. */
    while (!g_cxt.finished) {
        if(openset->size > 0) {
            int direction;
            node_t *cur = heap_extract(openset);

            if (!cur->closed){
                int minL = cur->gs[id] + g_cxt.F[pid] - heuristic(cur, pnba_args->start);
                if (cur->fs[id] < g_cxt.L && minL < g_cxt.L) {
                    /* Check all the neighbours. Since we are using a block maze, at most
                    four neighbours on the four directions. */
                    for (direction = 0; direction < 4; ++direction) {
                        node_t *n = fetch_neighbour(g_cxt.maze, cur, direction);
                        if(n == NULL || n->mark == WALL || n->closed) continue;
                        if (!n->opened[id] || n->gs[id] > cur->gs[id] + 1) {
                            n->gs[id] = cur->gs[id] + 1;
                            n->fs[id] = n->gs[id] + heuristic(n, pnba_args->goal);
                            n->parent[id] = cur;

                            if (!n->opened[id]) {
                                /* New node discovered, add into heap. */
                                n->opened[id] = true;
                                heap_insert(openset, n);
                            } else {
                                /* Updated old node. */
                                heap_update(openset, n);
                            }

                            /* There's a design to prevent block due to issues
                            * on synchronization. Check before mutex lock can
                            * prevent a large probability on waiting for checking
                            */
                            if (n->gs[pid] != INT_MAX){
                                int l = n->gs[id] + n->gs[pid];
                                if (l < g_cxt.L) {
                                    pthread_mutex_lock(&g_cxt.mutexL);
                                    if (l < g_cxt.L) {
                                        g_cxt.L = l;
                                        g_cxt.joint = n;
                                    }
                                    pthread_mutex_unlock(&g_cxt.mutexL);
                                }
                            }
                        }
                    }
                }

                cur->closed = true;
            }
        }

        if(openset->size > 0){
            g_cxt.F[id] = heap_peek(openset)->fs[id];
        }else{
            g_cxt.finished = true;
        }
    }

    pthread_exit((void*)0);
}

/*
 * Entrance point. Time ticking will be performed on the whole procedure,
 *   including I/O. Parallelize and optimize as much as you can.
 *
 */
int main(int argc, char *argv[])
{
    int i;

    node_t *n;
    maze_t *maze;
    heap_t *pathset;

    /* Attributes of threads */
    pthread_attr_t attr;

    /* Arguments for each thread, make sure these arguments have
     * through-program life time.
     */
    pnba_arguments_t arguments[2];

    assert(argc == 2);  /* Must have given the source file name. */

    /* Stopwatch for running time calculation */
    SW_RESET(SW_0);

    /* Initializations. */
    maze = maze_init(argv[1]);

    g_cxt.finished = false;
    g_cxt.maze = maze;
    g_cxt.L = INT_MAX;
    g_cxt.joint = NULL;

    /* Fill out arguments, reverse start and goal for two threads */

    arguments[0].id = 0;
    arguments[0].start = maze->start;
    arguments[0].goal = maze->goal;
    
    arguments[1].id = 1;
    arguments[1].start = maze->goal;
    arguments[1].goal = maze->start;

    /* Initialize mutex for multithreading */
    pthread_mutex_init(&g_cxt.mutexL, NULL);
    /* Initialize barrier for multithreading */
    pthread_barrier_init(&g_cxt.barrierInit, NULL, 2);
    pthread_barrier_init(&g_cxt.barrierStep, NULL, 2);

    SW_TICK(SW_0, "Initialize maze");

    /* Create threads to perform the A* pathfinding  */
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    for(i = 0; i < 2; i++) {
        /* 
        Each thread works on different end, and do a bidirectional pathfinding.
        */
        pthread_create(&g_cxt.threads[i], &attr, search_thread, (void *)&arguments[i]);
    }

    pthread_attr_destroy(&attr);

    /* Wait on the other threads */
    for(i = 0; i < 2; i++) {
        pthread_join(g_cxt.threads[i], NULL);
    }
    pthread_mutex_destroy(&g_cxt.mutexL);
    pthread_barrier_destroy(&g_cxt.barrierInit);
    pthread_barrier_destroy(&g_cxt.barrierStep);

    SW_TICK(SW_0, "Pathfinding");

    /* Output path information */
    PRT_PATHINFO();

    /* Print the steps back. */

    pathset = heap_init(CHANNEL_SORTING, node_coord_less);

    n = g_cxt.joint;
    while (n != NULL && n->mark != START) {
        /*maze_print_step(maze, n);*/
        heap_insert(pathset, n);
        n = n->parent[CHANNEL_THREAD_ONE];
    }
    n = g_cxt.joint;
    while (n != NULL && n->mark != GOAL) {
        /*maze_print_step(maze, n);*/
        heap_insert(pathset, n);
        n = n->parent[CHANNEL_THREAD_TWO];
    }
    maze_print_steps(maze, pathset);

    heap_destroy(pathset);

    SW_TICK(SW_0, "Output path");

    /* Free resources and return. */
    maze_destroy(maze);

    SW_ESCAPE(SW_0, "All time");

    return 0;
}

/*
 * Fetch the neighbour located at direction DIRECTION of node N, in the
 *   maze M. Returns pointer to the neighbour node.
 *
 */
static node_t *
fetch_neighbour (maze_t *m, node_t *n, int direction)
{
    switch (direction) {
        case 0: return maze_get_cell(m, n->x, n->y - 1);
        case 1: return maze_get_cell(m, n->x + 1, n->y);
        case 2: return maze_get_cell(m, n->x, n->y + 1);
        case 3: return maze_get_cell(m, n->x - 1, n->y);
    }
    return NULL;
}
