#ifndef TRACK_PROFILE_H
#define TRACK_PROFILE_H
/* Populate only after verifying the actual element order and topology.
 * Codes 0..7 identify distinct Y junctions; 128 identifies an island.
 * Empty by default: generic line/cross training works, unclassified split stops.
 */
#ifndef APP_ROUTE_LENGTH
#define APP_ROUTE_LENGTH 0u
#endif
#ifndef APP_ROUTE_SEQUENCE
#define APP_ROUTE_SEQUENCE {0u}
#endif
#ifndef APP_FINISH_DISTANCE_M
#define APP_FINISH_DISTANCE_M 0.0f /* measured: last element rear-clear to finish */
#endif
#define APP_ISLAND_ENTER_BRANCH 0u /* 0=left candidate, 1=right candidate */
#endif
