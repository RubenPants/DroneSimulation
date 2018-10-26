/**
 * C implementation of Javin's march.
 *
 * @author  Team Safier
 * @version 1.0
 */

#include "../data/vector.h"
#include "geom.h"

/**
 * Construct the convex hull of the given vector of points.
 * The first point is assumed to be the left-most one.
 */
point_2d *convex_hull(vector *points);