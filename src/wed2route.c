/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
*/
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#include <stddef.h>

#include <libxml/parser.h>
#include <libxml/xpath.h>

#include <acfutils/helpers.h>

#include "driving.h"
#include "wed2route.h"

#define	MIN_NODE_DIST	3	/* meters */

typedef struct {
	uint64_t	id;
	const xmlNode	*xml_node;
	avl_node_t	avl_node;
} objmap_t;

/* The virtual vehicle spec we use for computing path segments. */
const static vehicle_t veh = {
	.wheelbase = 1, .fixed_z_off = -0.5, .max_steer = 60,
};

static int
objmap_compar(const void *a, const void *b)
{
	const objmap_t *oa = a, *ob = b;

	if (oa->id < ob->id)
		return (-1);
	if (oa->id == ob->id)
		return (0);
	return (1);
}

void
wed2route_init(void)
{
	xmlInitParser();
}

void
wed2route_fini(void)
{
	xmlCleanupParser();
}

/*
 * Read the route start heading based on the orientation of the ramp start.
 */
static double
route_read_start_hdg(const xmlNode *point_node, xmlXPathContext *xpath_ctx)
{
	char xpath_query[128];
	xmlXPathObject *xpath_obj = NULL;
	xmlChar *lat_prop = xmlGetProp(point_node, (xmlChar *)"latitude");
	xmlChar *lon_prop = xmlGetProp(point_node, (xmlChar *)"longitude");
	xmlChar *hdg_prop;
	double hdg;

	ASSERT(lat_prop != NULL);
	ASSERT(lon_prop != NULL);

	snprintf(xpath_query, sizeof (xpath_query),
	    "//object[@class='WED_RampPosition']"
	    "[point[@latitude='%s'][@longitude='%s']]/point",
	    (char *)lat_prop, (char *)lon_prop);
	xmlFree(lat_prop);
	xmlFree(lon_prop);

	xpath_obj = xmlXPathEvalExpression((xmlChar *)xpath_query, xpath_ctx);
	if (xpath_obj == NULL || xpath_obj->nodesetval == NULL ||
	    xpath_obj->nodesetval->nodeNr == 0) {
		if (xpath_obj != NULL)
			xmlXPathFreeObject(xpath_obj);
		return (NAN);
	}
	hdg_prop = xmlGetProp(xpath_obj->nodesetval->nodeTab[0], "heading");
	if (hdg_prop == NULL || sscanf(hdg_prop, "%lf", &hdg) != 1 ||
	    !is_valid_hdg(hdg)) {
		if (hdg_prop != NULL)
			xmlFree(hdg_prop);
		xmlXPathFreeObject(xpath_obj);
		return (NAN);
	}
	xmlXPathFreeObject(xpath_obj);

	return (hdg);
}

static route_t *
cons_route(const xmlNode *node, const avl_tree_t *objmap)
{
	char xpath_query[128];
	xmlXPathContext *xpath_ctx = xmlXPathNewContext(node->doc);
	xmlXPathObject *xpath_obj = NULL;
	xmlChar *id_prop = xmlGetProp(node, (xmlChar *)"id");
	route_t *route = NULL;
	fpp_t fpp;
	geo_pos2_t start_pos_geo = NULL_GEO_POS2;
	vect2_t start_pos = NULL_VECT2;
	double start_hdg = NAN;

	vect2_t prev_pos = NULL_VECT2;
	double prev_hdg = NAN;

	if (id_prop == NULL)
		goto out;
	snprintf(xpath_query, sizeof (xpath_query),
	    "//object[@class='WED_StringPlacement'][@id='%s'][string_placement"
		"[@resource='pushback']]/children/child", id_prop);
	xmlFree(id_prop);

	xpath_obj = xmlXPathEvalExpression((xmlChar *)xpath_query, xpath_ctx);
	if (xpath_obj == NULL)
		goto out;

	route = route_alloc(NULL, NULL);
	for (int i = 0; i < xpath_obj->nodesetval->nodeNr; i++) {
		const xmlNode *string_node = xpath_obj->nodesetval->nodeTab[i];
		const xmlNode *point_node = NULL;
		const objmap_t *vertex, srch;
		double lat, lon, lat_hi, lon_hi, lat_lo, lon_lo;

#define	READ_PROP(var, fmt, node, propname) \
	do { \
		xmlChar *prop = xmlGetProp(node, (xmlChar *)propname); \
		if (prop == NULL || sscanf((char *)prop, fmt, var) != 1) { \
			if (prop != NULL) \
				xmlFree(prop); \
			route_free(route); \
			route = NULL; \
			goto out; \
		} \
	} while (0)

		READ_PROP(&srch.id, "%llu", node, "id");
		vertex = avl_find(objmap, &srch, NULL);
		if (vertex == NULL) {
			route_free(route);
			route = NULL;
			goto out;
		}

		for (const xmlNode *node = string_node->children;
		    node != NULL && node != string_node->last;
		    node = node->next) {
			if (strcmp((char *)node->name, "point") == 0) {
				point_node = node;
				break;
			}
		}
		if (point_node == NULL) {
			route_free(route);
			route = NULL;
			goto out;
		}

		READ_PROP(&lat, "%lf", point_node, "latitude");
		READ_PROP(&lon, "%lf", point_node, "longitude");
		READ_PROP(&lat_hi, "%lf", point_node, "ctrl_latitude_hi");
		READ_PROP(&lon_hi, "%lf", point_node, "ctrl_longitude_hi");
		READ_PROP(&lat_lo, "%lf", point_node, "ctrl_latitude_lo");
		READ_PROP(&lon_lo, "%lf", point_node, "ctrl_longitude_lo");

#undef	READ_PROP

		if (IS_NULL_GEO_POS(start_pos_geo)) {
			start_pos_geo = GEO_POS2(lat, lon);
			start_pos = ZERO_VECT2;
			start_hdg = route_read_start_hdg(point_node);
			if (isnan(start_hdg)) {
				route_free(route);
				route = NULL;
				goto out;
			}
			fpp = stereo_fpp_init(start_pos_geo, 0, &wgs84, B_TRUE);
		} else {
			geo_pos2_t end_pos_geo = GEO_POS2(lat, lon);
			vect2_t end_pos = geo2fpp(end_pos, &fpp);
			double end_hdg;
			geo_pos2_t node_pos_geo = GEO_POS2(
			    end_pos_geo.lat + lat_lo, end_pos_geo.lon + lon_lo);
			vect2_t node_pos = geo2fpp(node_pos_geo, &fpp);

			/* skip doubled nodes */
			if (vect2_dist(end_pos, start_pos) < MIN_NODE_DIST)
				continue;

			if (vect2_dist(node_pos, end_pos) < MIN_NODE_DIST / 2) {
				end_hdg = dir2hdg(vect2_sub(start_pos,
				    end_pos));
			} else {
				end_hdg = dir2hdg(vect2_sub(node_pos, end_pos));
			}
		}
	}

out:
	if (route != NULL && avl_numnodes(&route->segs) == 0) {
		route_free(route);
		route = NULL;
		goto out;
	}
	if (xpath_obj != NULL)
		xmlXPathFreeObject(xpath_obj);
	if (xpath_ctx != NULL)
		xmlXPathFreeContext(xpath_ctx);

	return (route);
}

bool_t
wed2dat(const char *earthwedxml)
{
	avl_tree_t *objmap = NULL;
	xmlDoc *doc = NULL;
	xmlXPathContext *xpath_ctx = NULL;
	xmlXPathObject *xpath_obj = NULL;

	doc = xmlParseFile(earthwedxml);
	if (doc == NULL)
		goto errout;
	xpath_ctx = xmlXPathNewContext(doc);
	if (xpath_ctx == NULL)
		goto errout;

	/* construct an ID-to-object mapping for faster lookup */
	objmap = calloc(1, sizeof (*objmap));
	avl_create(objmap, objmap_compar, sizeof (objmap_t),
	    offsetof(objmap_t, avl_node));

	xpath_obj = xmlXPathEvalExpression((xmlChar *)"//object", xpath_ctx);
	if (xpath_obj == NULL || xpath_obj->nodesetval == NULL)
		goto errout;
	for (int i = 0; i < xpath_obj->nodesetval->nodeNr; i++) {
		objmap_t *e = calloc(1, sizeof (*e));
		const xmlNode *node = xpath_obj->nodesetval->nodeTab[i];
		xmlChar *prop = xmlGetProp(node, (xmlChar *)"id");

		e->xml_node = node;
		if (prop == NULL || sscanf((char *)prop, "%llu", &e->id) != 1 ||
		    avl_find(objmap, e, NULL) != NULL) {
			if (prop != NULL)
				xmlFree(prop);
			free(e);
			goto errout;
		}
	}
	xmlXPathFreeObject(xpath_obj);
	xpath_obj = NULL;

	/* Enumerate all pushback routes and construct them in memory */
	xpath_obj = xmlXPathEvalExpression((xmlChar *)
	    "//object[@class='WED_StringPlacement'][string_placement"
		"[@resource='pushback']]", xpath_ctx);
	for (int i = 0; i < xpath_obj->nodesetval->nodeNr; i++) {
		const xmlNode *node = xpath_obj->nodesetval->nodeTab[i];
		route_t *route = cons_route(node, objmap);

		if (route == NULL)
			goto errout;
	}
	xmlXPathFreeObject(xpath_obj);
	xpath_obj = NULL;

	if (objmap != NULL) {
		void *cookie = NULL;
		objmap_t *e;
		while ((e = avl_destroy_nodes(objmap, &cookie)) != NULL)
			free(e);
		avl_destroy(objmap);
		free(objmap);
	}
	xmlXPathFreeObject(xpath_obj);
	xmlXPathFreeContext(xpath_ctx);
	xmlFreeDoc(doc);

	return (B_TRUE);
errout:
	if (objmap != NULL) {
		void *cookie = NULL;
		objmap_t *e;
		while ((e = avl_destroy_nodes(objmap, &cookie)) != NULL)
			free(e);
		avl_destroy(objmap);
		free(objmap);
	}

	if (xpath_obj != NULL)
		xmlXPathFreeObject(xpath_obj);
	if (xpath_ctx != NULL)
		xmlXPathFreeContext(xpath_ctx);
	if (doc != NULL)
		xmlFreeDoc(doc);
	return (B_FALSE);
}
