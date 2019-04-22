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

#include <stdlib.h>     /* NULL */
#include <assert.h>     /* assert */
#include <time.h>       /* clock */
#include <limits.h>     /* INT_MAX */

#include <pthread.h>    /* Multithreading */

#include "heap.h"
#include "node.h"
#include "maze.h"
#include "compass.h"    /* The heuristic. */

/*****************************************************************************/
typedef struct stopwatch_t{
    time_t first;
    time_t last;
} stopwatch_t;

void stopwatch_init(stopwatch_t* sw)
{
    if(sw != NULL) sw->first = sw->last = clock();
}

float stopwatch_tick(stopwatch_t* sw, const char* message)
{
    float secs = -1.0f;
    if(sw != NULL){
        secs = (float)(clock() - sw->last) / (float)CLOCKS_PER_SEC;
#ifdef __PERFORMANCE_METRIC__
        if(message != NULL) printf("%s: %f s\n", message, secs);
        else printf("%f s\n", secs);
        sw->last = clock();
#else
        UNUSED(message);
#endif
    }
    return secs;
}

float stopwatch_escape(stopwatch_t* sw, const char* message)
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
struct pnba_context_t{
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

} g_cxt;

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
    while (openset->size > 0)
    {
        int direction;
        node_t *cur = heap_extract(openset);

        if (!cur->closed){
            if (cur->fs[id] < g_cxt.L
                && _SUB_(_ADD_(cur->gs[id], g_cxt.F[pid]), heuristic(cur, pnba_args->start)) < g_cxt.L
            ){
                /* Check all the neighbours. Since we are using a block maze, at most
                four neighbours on the four directions. */
                for (direction = 0; direction < 4; ++direction) {
                    node_t *n = fetch_neighbour(g_cxt.maze, cur, direction);
                    if(n == NULL) continue;
                    if(n->mark == WALL) continue;
                    if(!n->closed && n->gs[id] > _ADD_(cur->gs[id], 1)){
                        n->gs[id] = _ADD_(cur->gs[id], 1);
                        n->fs[id] = _ADD_(cur->gs[id], heuristic(cur, pnba_args->goal));
                        n->parent[id] = cur;

                        if (!n->opened) {
                            /* New node discovered, add into heap. */
                            n->opened = true;
                            heap_insert(openset, n);
                        } else {
                            /* Updated old node. */
                            heap_update(openset, n);
                        }

                        /* There's a design to prevent block due to issues
                         * on synchronization. Check before mutex lock can
                         * prevent a large probability on waiting for checking
                         */
                        if(_ADD_(n->gs[id], n->gs[pid]) < g_cxt.L) {
                            pthread_mutex_lock(&g_cxt.mutexL);
                            if (_ADD_(n->gs[id], n->gs[pid]) < g_cxt.L){
                                g_cxt.L = _ADD_(n->gs[id], n->gs[pid]);
                                g_cxt.joint = n;
                            }
                            pthread_mutex_unlock(&g_cxt.mutexL);
                        }
                    }
                }
            }

            cur->closed = true;
        }

        if(openset->size > 0){
            g_cxt.F[id] = heap_peek(openset)->fs[id];
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

    /* Stopwatch for running time calculation */
    stopwatch_t sw;

    assert(argc == 2);  /* Must have given the source file name. */

    stopwatch_init(&sw);

    /* Initializations. */
    maze = maze_init(argv[1]);

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

    stopwatch_tick(&sw, "Initialize maze");

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
    /* pthread_barrier_destory(&g_cxt.barrierInit); */

    stopwatch_tick(&sw, "Pathfinding");

    printf("Path length: %d\n", g_cxt.L);

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

    printf("%d, %d\n", g_cxt.joint->x, g_cxt.joint->y);

    heap_destroy(pathset);

    #if 0
    n = maze->goal->parent;
    while (n != NULL && n->mark != START) {
        /*maze_print_step(maze, n);*/
        heap_insert(pathset, n);
        n = n->parent;
    }
    maze_print_steps(maze, pathset);
    #endif

    stopwatch_tick(&sw, "Output path");

    /* Free resources and return. */
    maze_destroy(maze);

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
