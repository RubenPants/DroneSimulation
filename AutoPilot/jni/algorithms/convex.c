/**
 * C implementation of Javin's march.
 *
 * @author  Team Safier
 * @version 1.0
 */

#pragma mark Heading

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "convex.h"

#define PRINT_CONVEX_HULL 1
#define COLINEARITY_TRESHOLD 1.5

/**
 * Determine the orientation of 3 points.
 * 
 * @param   A positive integer if the orientation is clockwise, a negative integer otherwise.
 *          Zero is returned if the points are (nearly?) colinear.
 * @note    To determine whether or not points are colinear, a colinearity treshold is applied.
 *          Points close enough to each other are considered colinear.
 */
int orientation(point_2d *p, point_2d *q, point_2d *r);

#pragma mark Implementation

point_2d *convex_hull(vector *points) {
    
    // Check if there are enough points
    int n = vector_count(points);
    if (n < 3)
        return NULL;
    
    // Initialization
    point_2d *list = (point_2d *)vector_get(points, 0), *current;
    list->next = NULL;
    
    // Start from leftmost point (index 0), keep moving counterclockwise
    //  until the start point is reached again
    int l=0, p=l, q;
    do {
        
        // Search for a point 'q' such that orientation(p, i, q) is
        //  counterclockwise for all points 'i'
        q = (p + 1) % n;
        for (int i=0 ; i<n ; i++)
            if (orientation(vector_get(points, p),
                            vector_get(points, i),
                            vector_get(points, q)) == 2)
                q = i;
        
        current = (point_2d *)vector_get(points, q); // Add q to result as a next point of p
        current->next = list->next;
        list->next = current;
        p = q; // Set p as q for next iteration
        
    }
    while (p != l);
    
    // Print Result?
#if PRINT_CONVEX_HULL
    current = list;
    while (current != NULL) {
        printf("(%i,%i)\n", current->x, current->y);
        current = current->next;
    }
#endif
    
    return list;
    
}

int orientation(point_2d *p, point_2d *q, point_2d *r) {
    
    int val = (q->y - p->y) * (r->x - q->x) - (q->x - p->x) * (r->y - q->y);
    if (abs(val) < COLINEARITY_TRESHOLD)
        return 0; // colinear
    
    return (val > 0) ? 1 : 2; // clock or counterclock wise
    
}