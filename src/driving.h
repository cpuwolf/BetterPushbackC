/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file COPYING.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#ifndef	_DRIVING_H_
#define	_DRIVING_H_

#include <acfutils/list.h>
#include <acfutils/geom.h>

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum {
	SEG_TYPE_STRAIGHT,
	SEG_TYPE_TURN
} seg_type_t;

typedef struct {
	seg_type_t	type;

	/*
	 * The vector positions are in local OpenGL coordinates space and
	 * are used by the steering algorithm. The geographic coordinates
	 * are used by the persistence algorithms to store the points in
	 * an absolute sense. Before passing such a segment to the driving
	 * algorithm, they must be converted to local coordinates using
	 * seg_world2local.
	 */
	bool_t	have_local_coords;
	bool_t	have_world_coords;

	vect2_t		start_pos;
	geo_pos2_t	start_pos_geo;
	double		start_hdg;
	vect2_t		end_pos;
	geo_pos2_t	end_pos_geo;
	double		end_hdg;

	/*
	 * A backward pushback segment looks like this:
	 *         ^^  (start_hdg)
	 *      ---++  (start_pos)
	 *      ^  ||
	 *      |  ||
	 * (s1) |  ||
	 *      |  ||
	 *      v  || (r)
	 *      ---||-----+
	 *          \\    | (r)    (end_hdg)
	 *            \\  |            |
	 *              ``=============<+ (end_pos)
	 *                |             |
	 *                |<----------->|
	 *                      (s2)
	 *
	 * A towing segment is similar, but the positions of the respective
	 * segments is reversed.
	 */
	bool_t		backward;
	union {
		double		len;	/* straight segment length (meters) */
		struct {
			double	r;	/* turn radius (meters) */
			bool_t	right;	/* turn center is right or left */
		} turn;
	};

	/*
	 * Flag indicating if the user placed this segment. Non-user-placed
	 * segments (up to the last user-placed segment) are deleted from
	 * the segment list.
	 */
	bool_t		user_placed;

	list_node_t	node;
} seg_t;

/*
 * Current position, orientation & velocity of vehicle.
 */
typedef struct {
	vect2_t	pos;		/* centerpoint position, world coords, meters */
	double	hdg;		/* true heading, world coords, degrees */
	double	spd;		/* forward speed, m/s, neg when reversing */
} vehicle_pos_t;

/*
 * Vehicle capability description. Used when determining steering commands.
 */
typedef struct {
	double	wheelbase;	/* distance from front to rear axle, meters */
	double	fixed_z_off;	/* long offset of rear axle from pos, meters */
	double	max_steer;	/* max steer angle, degrees */
	double	max_fwd_spd;	/* max forward speed, m/s */
	double	max_rev_spd;	/* max rev speed, m/s */
	double	max_ang_vel;	/* max turn angular velocity, deg/s */
	double	max_accel;	/* max acceleration, m/s^2 */
	double	max_decel;	/* max deceleration, m/s^2 */
	bool_t	xp10_bug_ign;	/* ignore X-Plane 10 stickiness bug */
} vehicle_t;

int compute_segs(const vehicle_t *veh, vect2_t start_pos, double start_hdg,
    vect2_t end_pos, double end_hdg, list_t *segs);
bool_t drive_segs(const vehicle_pos_t *pos, const vehicle_t *veh, list_t *segs,
    double *last_mis_hdg, double d_t, double *out_steer, double *out_speed);

void seg_world2local(seg_t *seg);
void seg_local2world(seg_t *seg);

void route_save(const list_t *segs);
void route_load(geo_pos2_t start_pos, double start_hdg, list_t *segs);

#ifdef	__cplusplus
}
#endif

#endif	/* _DRIVING_H_ */
