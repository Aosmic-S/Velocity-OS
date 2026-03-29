/**
 * ============================================================
 *  AOSMIC HYPERKERNEL OS — VelocityOS v1.0
 *  Area Scanning Service
 *
 *  Drives the robot in a systematic spiral/grid pattern,
 *  builds an occupancy map from ultrasonic readings, then
 *  exports the map to the AI training buffer.
 *
 *  Apache License 2.0 — Aosmic Studio × Absolute Tech
 * ============================================================
 */

#pragma once
#include "../kernel/hyperkernel.h"
#include "math.h"

// ─── Map Configuration ────────────────────────────────────
#define SCAN_GRID_W          40      // cells wide
#define SCAN_GRID_H          40      // cells tall
#define SCAN_CELL_SIZE_CM    20.0f   // each cell = 20cm real-world
#define SCAN_MAX_WAYPOINTS   256     // spiral waypoints
#define SCAN_OBSTACLE_THRESH 25.0f   // cm — cell marked occupied
#define SCAN_FREE_THRESH     60.0f   // cm — cell marked free
#define SCAN_PATROL_PTS      64      // patrol route points extracted from map

// ─── Cell States ──────────────────────────────────────────
typedef enum {
    CELL_UNKNOWN   = 0,
    CELL_FREE      = 1,
    CELL_OCCUPIED  = 2,
    CELL_FRONTIER  = 3,   // boundary of explored area
    CELL_PATROL    = 4,   // selected for patrol route
} cell_state_t;

// ─── Occupancy Map ────────────────────────────────────────
typedef struct {
    uint8_t  state;         // cell_state_t
    uint8_t  visit_count;
    float    min_dist_cm;   // min obstacle reading from this cell
    float    confidence;    // 0.0 - 1.0 (multiple readings)
} map_cell_t;

// ─── Robot Pose ───────────────────────────────────────────
typedef struct {
    float x_cm;       // real-world X (cm)
    float y_cm;       // real-world Y (cm)
    float heading_deg;// heading 0=north, 90=east
    float vx, vy;     // velocity
} robot_pose_t;

// ─── Waypoint ─────────────────────────────────────────────
typedef struct {
    float x_cm;
    float y_cm;
    float heading_deg;
    bool  completed;
} scan_waypoint_t;

// ─── Patrol Point ─────────────────────────────────────────
typedef struct {
    float x_cm;
    float y_cm;
    float dwell_ms;   // time to spend at this point
} patrol_point_t;

// ─── Scan Phase ───────────────────────────────────────────
typedef enum {
    SCAN_PHASE_IDLE      = 0,
    SCAN_PHASE_INIT      = 1,
    SCAN_PHASE_SPIRALING = 2,
    SCAN_PHASE_SWEEP     = 3,
    SCAN_PHASE_ANALYSIS  = 4,
    SCAN_PHASE_TRAINING  = 5,
    SCAN_PHASE_COMPLETE  = 6,
    SCAN_PHASE_ERROR     = 7,
} scan_phase_t;

// ─── Scan Service State ───────────────────────────────────
typedef struct {
    scan_phase_t    phase;
    map_cell_t      grid[SCAN_GRID_H][SCAN_GRID_W];
    robot_pose_t    pose;
    scan_waypoint_t waypoints[SCAN_MAX_WAYPOINTS];
    int             waypoint_count;
    int             waypoint_idx;
    patrol_point_t  patrol_route[SCAN_PATROL_PTS];
    int             patrol_count;
    float           coverage_pct;
    int             cells_explored;
    int             cells_total;
    bool            scan_active;
    bool            map_ready;
    int64_t         scan_start_time;
    int64_t         scan_duration_ms;
    QueueHandle_t   sensor_sub;
    QueueHandle_t   motor_status_sub;
    SemaphoreHandle_t map_lock;
    // Training data
    uint32_t        training_samples;
    float           avg_open_space_pct;
    float           obstacle_density;
} scan_service_t;

extern scan_service_t g_scan;

// ─── Scan Service API ─────────────────────────────────────
hk_status_t scan_service_init(void);
hk_status_t scan_service_start_scan(void);
hk_status_t scan_service_stop_scan(void);
void        scan_get_map_json(char* buf, size_t maxlen);
void        scan_get_patrol_json(char* buf, size_t maxlen);
bool        scan_is_complete(void);
float       scan_get_coverage(void);
const char* scan_phase_name(scan_phase_t p);
