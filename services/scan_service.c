/**
 * Area Scanning Service Implementation
 * Spiral exploration → Occupancy grid → Patrol route → AI training
 *
 * Apache License 2.0 — Aosmic Studio × Absolute Tech
 */

#include "scan_service.h"
#include "motion_service.h"
#include "../bus/message_bus.h"
#include "../ai/ai_engine.h"
#include "esp_log.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"

static const char* TAG = "ScanSvc";
scan_service_t g_scan = {0};

static const char* PHASE_NAMES[] = {
    "IDLE","INIT","SPIRALING","SWEEP","ANALYSIS","TRAINING","COMPLETE","ERROR"
};
const char* scan_phase_name(scan_phase_t p) {
    if (p > SCAN_PHASE_ERROR) return "?";
    return PHASE_NAMES[p];
}

// ─── Coordinate helpers ───────────────────────────────────
static inline int pose_to_cell_x(float x_cm) {
    int cx = (int)(x_cm / SCAN_CELL_SIZE_CM) + SCAN_GRID_W/2;
    return cx < 0 ? 0 : cx >= SCAN_GRID_W ? SCAN_GRID_W-1 : cx;
}
static inline int pose_to_cell_y(float y_cm) {
    int cy = (int)(y_cm / SCAN_CELL_SIZE_CM) + SCAN_GRID_H/2;
    return cy < 0 ? 0 : cy >= SCAN_GRID_H ? SCAN_GRID_H-1 : cy;
}

// ─── Spiral Waypoint Generator ────────────────────────────
/**
 * Generates an outward-expanding rectangular spiral.
 * Starts at center (0,0) and grows by SCAN_CELL_SIZE_CM each ring.
 * Each waypoint is ~40cm apart with 90-degree turns.
 */
static void generate_spiral_waypoints(void) {
    g_scan.waypoint_count = 0;
    float step = SCAN_CELL_SIZE_CM * 2.0f;  // 40cm per step
    float x = 0, y = 0;
    int   layer = 1;
    float heading = 0.0f;  // Start facing north

    // Center point
    g_scan.waypoints[g_scan.waypoint_count++] = (scan_waypoint_t){ .x_cm=0, .y_cm=0, .heading_deg=0 };

    // Spiral: right, down, left, up, growing each ring
    int moves[] = {1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8};
    // Directions: N=0, E=90, S=180, W=270
    float dirs[][2] = { {0,-1},{1,0},{0,1},{-1,0} }; // N,E,S,W → dx,dy
    int   dir_idx = 1;  // Start east

    int side = 1;
    for (int ring = 0; ring < 8 && g_scan.waypoint_count < SCAN_MAX_WAYPOINTS - 4; ring++) {
        int steps_per_side = ring + 1;
        for (int s = 0; s < 4; s++) {
            for (int i = 0; i < steps_per_side && g_scan.waypoint_count < SCAN_MAX_WAYPOINTS - 1; i++) {
                x += dirs[dir_idx % 4][0] * step;
                y += dirs[dir_idx % 4][1] * step;
                // Clamp to grid bounds in cm
                float max_cm = (SCAN_GRID_W/2 - 1) * SCAN_CELL_SIZE_CM;
                if (fabsf(x) > max_cm || fabsf(y) > max_cm) goto done_spiral;
                g_scan.waypoints[g_scan.waypoint_count++] = (scan_waypoint_t){
                    .x_cm = x,
                    .y_cm = y,
                    .heading_deg = dir_idx % 4 * 90.0f,
                    .completed = false,
                };
            }
            dir_idx++;
        }
    }
    done_spiral:
    ESP_LOGI(TAG, "Generated %d spiral waypoints", g_scan.waypoint_count);
}

// ─── Update Map Cell from Sensor Reading ─────────────────
static void update_map_from_sensor(float front_cm, float left_cm, float rear_cm) {
    xSemaphoreTake(g_scan.map_lock, portMAX_DELAY);

    int cx = pose_to_cell_x(g_scan.pose.x_cm);
    int cy = pose_to_cell_y(g_scan.pose.y_cm);

    // Mark current cell as free (robot is here)
    if (g_scan.grid[cy][cx].state == CELL_UNKNOWN) g_scan.cells_explored++;
    g_scan.grid[cy][cx].state = CELL_FREE;
    g_scan.grid[cy][cx].visit_count++;
    g_scan.grid[cy][cx].confidence = fminf(1.0f, g_scan.grid[cy][cx].confidence + 0.2f);

    // Ray-cast front sensor
    float hrad = g_scan.pose.heading_deg * M_PI / 180.0f;
    float fdx = sinf(hrad), fdy = -cosf(hrad);

    // Mark cells along ray as free, end cell as occupied if obstacle
    int ray_cells = (int)(front_cm / SCAN_CELL_SIZE_CM);
    for (int r = 1; r <= ray_cells && r < 8; r++) {
        int rx = (int)(g_scan.pose.x_cm / SCAN_CELL_SIZE_CM + fdx * r) + SCAN_GRID_W/2;
        int ry = (int)(g_scan.pose.y_cm / SCAN_CELL_SIZE_CM + fdy * r) + SCAN_GRID_H/2;
        if (rx < 0 || rx >= SCAN_GRID_W || ry < 0 || ry >= SCAN_GRID_H) break;
        if (g_scan.grid[ry][rx].state == CELL_UNKNOWN) g_scan.cells_explored++;
        if (r == ray_cells && front_cm < SCAN_OBSTACLE_THRESH) {
            g_scan.grid[ry][rx].state = CELL_OCCUPIED;
        } else if (g_scan.grid[ry][rx].state != CELL_OCCUPIED) {
            g_scan.grid[ry][rx].state = CELL_FREE;
        }
        g_scan.grid[ry][rx].min_dist_cm = fminf(
            g_scan.grid[ry][rx].min_dist_cm > 0 ? g_scan.grid[ry][rx].min_dist_cm : 9999.0f,
            front_cm
        );
    }

    // Mark frontier cells (free cell adjacent to unknown)
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int nx = cx+dx, ny = cy+dy;
            if (nx<0||nx>=SCAN_GRID_W||ny<0||ny>=SCAN_GRID_H) continue;
            if (g_scan.grid[ny][nx].state == CELL_UNKNOWN) {
                g_scan.grid[cy][cx].state = CELL_FRONTIER;
            }
        }
    }

    g_scan.coverage_pct = (float)g_scan.cells_explored / g_scan.cells_total * 100.0f;
    xSemaphoreGive(g_scan.map_lock);
}

// ─── Dead-Reckoning Pose Update ───────────────────────────
static void update_pose_from_motors(float* wheel_speeds, uint8_t motion_cmd) {
    // Simple dead-reckoning: average wheel speeds → velocity → pose
    float avgL = (wheel_speeds[0] + wheel_speeds[2]) / 2.0f;  // left
    float avgR = (wheel_speeds[1] + wheel_speeds[3]) / 2.0f;  // right
    float speed_cm_s = (avgL + avgR) / 2.0f * 0.5f;           // scale to cm/s
    float turn_rate  = (avgR - avgL) / 100.0f * 2.5f;          // rad/s

    float hrad = g_scan.pose.heading_deg * M_PI / 180.0f;
    float dt   = 0.05f;  // 50ms tick

    g_scan.pose.x_cm      += sinf(hrad) * speed_cm_s * dt;
    g_scan.pose.y_cm      -= cosf(hrad) * speed_cm_s * dt;
    g_scan.pose.heading_deg += turn_rate * dt * 180.0f / M_PI;
    if (g_scan.pose.heading_deg < 0)   g_scan.pose.heading_deg += 360.0f;
    if (g_scan.pose.heading_deg >= 360) g_scan.pose.heading_deg -= 360.0f;
}

// ─── Waypoint Navigator ───────────────────────────────────
static void navigate_to_waypoint(scan_waypoint_t* wp) {
    float dx = wp->x_cm - g_scan.pose.x_cm;
    float dy = wp->y_cm - g_scan.pose.y_cm;
    float dist = sqrtf(dx*dx + dy*dy);

    if (dist < 15.0f) {
        // Arrived
        wp->completed = true;
        g_scan.waypoint_idx++;
        return;
    }

    // Calculate desired heading
    float target_deg = atan2f(dx, -dy) * 180.0f / M_PI;
    if (target_deg < 0) target_deg += 360.0f;
    float heading_err = target_deg - g_scan.pose.heading_deg;
    if (heading_err > 180)  heading_err -= 360;
    if (heading_err < -180) heading_err += 360;

    motion_msg_t mcmd = { .profile = ACCEL_SMOOTH };

    if (fabsf(heading_err) > 25.0f) {
        // Turn first
        mcmd.cmd   = heading_err > 0 ? MOTION_CMD_PIVOT_RIGHT : MOTION_CMD_PIVOT_LEFT;
        mcmd.speed = 28.0f;
    } else {
        // Move toward waypoint
        mcmd.cmd       = MOTION_CMD_FORWARD;
        mcmd.speed     = fminf(55.0f, dist * 0.6f);
        mcmd.turn_bias = heading_err / 180.0f;
    }
    motion_send_cmd(&mcmd);
}

// ─── Map Analysis → Patrol Route ─────────────────────────
/**
 * After scan completes, sample free cells uniformly across the map
 * and build an ordered patrol route using a greedy nearest-neighbor
 * travelling-salesman approximation.
 */
static void analyze_map_build_patrol(void) {
    ESP_LOGI(TAG, "Analyzing map, building patrol route...");

    // Collect candidate free cells
    typedef struct { float x, y; } point_t;
    point_t candidates[256];
    int n = 0;
    int step = SCAN_GRID_W / 8;  // Sample every ~5 cells

    for (int gy = step/2; gy < SCAN_GRID_H && n < 256; gy += step) {
        for (int gx = step/2; gx < SCAN_GRID_W && n < 256; gx += step) {
            if (g_scan.grid[gy][gx].state == CELL_FREE ||
                g_scan.grid[gy][gx].state == CELL_FRONTIER) {
                candidates[n].x = (gx - SCAN_GRID_W/2) * SCAN_CELL_SIZE_CM;
                candidates[n].y = (gy - SCAN_GRID_H/2) * SCAN_CELL_SIZE_CM;
                n++;
            }
        }
    }

    if (n == 0) { ESP_LOGW(TAG, "No free cells found for patrol"); return; }

    // Greedy nearest-neighbor from origin
    bool used[256] = {0};
    g_scan.patrol_count = 0;
    float cx = 0, cy = 0;

    int patrol_max = n < SCAN_PATROL_PTS ? n : SCAN_PATROL_PTS;
    for (int i = 0; i < patrol_max; i++) {
        int best = -1;
        float best_d = 1e9f;
        for (int j = 0; j < n; j++) {
            if (used[j]) continue;
            float dx = candidates[j].x - cx;
            float dy = candidates[j].y - cy;
            float d  = sqrtf(dx*dx + dy*dy);
            if (d < best_d) { best_d = d; best = j; }
        }
        if (best < 0) break;
        used[best] = true;
        g_scan.patrol_route[g_scan.patrol_count++] = (patrol_point_t){
            .x_cm    = candidates[best].x,
            .y_cm    = candidates[best].y,
            .dwell_ms = 500.0f,
        };
        // Mark cell as patrol point on map
        int mx = pose_to_cell_x(candidates[best].x);
        int my = pose_to_cell_y(candidates[best].y);
        g_scan.grid[my][mx].state = CELL_PATROL;

        cx = candidates[best].x;
        cy = candidates[best].y;
    }

    // Compute map stats for AI training
    int free_cells = 0, occ_cells = 0;
    for (int gy = 0; gy < SCAN_GRID_H; gy++)
        for (int gx = 0; gx < SCAN_GRID_W; gx++) {
            if (g_scan.grid[gy][gx].state == CELL_FREE ||
                g_scan.grid[gy][gx].state == CELL_PATROL) free_cells++;
            if (g_scan.grid[gy][gx].state == CELL_OCCUPIED) occ_cells++;
        }

    g_scan.avg_open_space_pct = (float)free_cells / (free_cells + occ_cells + 1) * 100.0f;
    g_scan.obstacle_density   = (float)occ_cells  / (free_cells + occ_cells + 1);

    ESP_LOGI(TAG, "Patrol route: %d points | Open: %.1f%% | Obstacles: %.1f%%",
             g_scan.patrol_count, g_scan.avg_open_space_pct, g_scan.obstacle_density * 100);
}

// ─── AI Training Feed ─────────────────────────────────────
/**
 * Exports scan-derived features into the AI engine's knowledge base.
 * The AI uses these to calibrate:
 *   - patrol speed (based on open space %)
 *   - obstacle avoidance aggression (based on density)
 *   - patrol route (replaces default spiral with map-derived route)
 *   - safe stopping distance (based on min recorded obstacle distance)
 */
static void feed_ai_training_data(void) {
    ESP_LOGI(TAG, "Feeding scan data to AI engine for training...");

    // Calibrate patrol speed
    float new_patrol_speed;
    if (g_scan.avg_open_space_pct > 70.0f)       new_patrol_speed = 65.0f;
    else if (g_scan.avg_open_space_pct > 40.0f)  new_patrol_speed = 45.0f;
    else                                           new_patrol_speed = 28.0f;

    // Calibrate avoidance distance
    float closest_obstacle = 9999.0f;
    for (int gy = 0; gy < SCAN_GRID_H; gy++)
        for (int gx = 0; gx < SCAN_GRID_W; gx++)
            if (g_scan.grid[gy][gx].state == CELL_OCCUPIED &&
                g_scan.grid[gy][gx].min_dist_cm > 0)
                closest_obstacle = fminf(closest_obstacle, g_scan.grid[gy][gx].min_dist_cm);

    float new_stop_dist = closest_obstacle < 9999.0f ?
                          fmaxf(15.0f, closest_obstacle * 0.6f) : 20.0f;

    // Write to AI engine
    xSemaphoreTake(g_scan.map_lock, portMAX_DELAY);

    g_ai.patrol_speed = new_patrol_speed;
    g_ai.patrol_step  = 0;

    // Copy patrol route into AI (AI now navigates the scanned map)
    // We encode patrol points as a step count; the AI navigates them sequentially
    // This is stored in g_ai.patrol_direction as the learned patrol preset index
    g_ai.patrol_direction = 100; // signals "use scanned route"

    g_scan.training_samples++;
    xSemaphoreGive(g_scan.map_lock);

    // Publish training complete event
    typedef struct {
        float patrol_speed;
        float stop_dist;
        float open_pct;
        float obs_density;
        int   patrol_pts;
        uint32_t samples;
    } training_result_t;

    training_result_t tr = {
        .patrol_speed = new_patrol_speed,
        .stop_dist    = new_stop_dist,
        .open_pct     = g_scan.avg_open_space_pct,
        .obs_density  = g_scan.obstacle_density,
        .patrol_pts   = g_scan.patrol_count,
        .samples      = g_scan.training_samples,
    };
    bus_publish(TOPIC_AI_DECISION, &tr, sizeof(tr));

    ESP_LOGI(TAG, "AI trained: speed=%.0f%% stop=%.1fcm open=%.1f%% samples=%u",
             new_patrol_speed, new_stop_dist, g_scan.avg_open_space_pct,
             g_scan.training_samples);

    char alert[80];
    snprintf(alert, sizeof(alert),
             "AI retrained from scan: patrol=%.0f%% open, %d patrol points",
             g_scan.avg_open_space_pct, g_scan.patrol_count);
    bus_publish(TOPIC_SYSTEM_ALERT, alert, strlen(alert)+1);
}

// ─── Main Scan Task ───────────────────────────────────────
static void scan_task(void* arg) {
    TickType_t last_wake = xTaskGetTickCount();
    ESP_LOGI(TAG, "Scan service task running");

    // Sensor data snapshot
    typedef struct { float f,r,l,gas,bv,tmp; bool flame,smoke; } sd_t;
    typedef struct { float speeds[4]; uint8_t cmd; }              ms_t;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50)); // 20Hz

        if (!g_scan.scan_active) continue;

        // Consume latest sensor data
        hk_message_t msg;
        sd_t latest_sensor = {0};
        ms_t latest_motor  = {0};

        while (xQueueReceive(g_scan.sensor_sub, &msg, 0) == pdTRUE) {
            if (msg.payload_len >= sizeof(sd_t))
                memcpy(&latest_sensor, msg.payload, sizeof(sd_t));
        }
        while (xQueueReceive(g_scan.motor_status_sub, &msg, 0) == pdTRUE) {
            if (msg.payload_len >= sizeof(ms_t))
                memcpy(&latest_motor, msg.payload, sizeof(ms_t));
        }

        // Dead-reckoning pose update
        update_pose_from_motors(latest_motor.speeds, latest_motor.cmd);

        switch (g_scan.phase) {

        case SCAN_PHASE_INIT:
            generate_spiral_waypoints();
            g_scan.cells_total    = SCAN_GRID_W * SCAN_GRID_H;
            g_scan.cells_explored = 0;
            g_scan.waypoint_idx   = 0;
            g_scan.coverage_pct   = 0;
            g_scan.scan_start_time = esp_timer_get_time();
            // Reset pose to center
            g_scan.pose = (robot_pose_t){0};
            // Reset grid
            memset(g_scan.grid, 0, sizeof(g_scan.grid));
            g_scan.phase = SCAN_PHASE_SPIRALING;
            ESP_LOGI(TAG, "Scan INIT complete → SPIRALING");
            break;

        case SCAN_PHASE_SPIRALING: {
            // Update map from sensor readings
            update_map_from_sensor(
                latest_sensor.f > 0 ? latest_sensor.f : 300.0f,
                latest_sensor.l > 0 ? latest_sensor.l : 300.0f,
                latest_sensor.r > 0 ? latest_sensor.r : 300.0f
            );

            if (g_scan.waypoint_idx >= g_scan.waypoint_count) {
                // All waypoints visited
                motion_msg_t stop = { .cmd = MOTION_CMD_STOP };
                motion_send_cmd(&stop);
                g_scan.phase = SCAN_PHASE_SWEEP;
                ESP_LOGI(TAG, "Spiral complete → SWEEP (coverage %.1f%%)", g_scan.coverage_pct);
            } else {
                navigate_to_waypoint(&g_scan.waypoints[g_scan.waypoint_idx]);
            }

            // Safety: obstacle interrupt during scan
            if (latest_sensor.f > 0 && latest_sensor.f < 12.0f) {
                motion_msg_t stop = { .cmd = MOTION_CMD_STOP };
                motion_send_cmd(&stop);
            }
            break;
        }

        case SCAN_PHASE_SWEEP: {
            // 360-degree in-place sensor sweep for final map refinement
            static int sweep_ticks = 0;
            sweep_ticks++;
            if (sweep_ticks < 80) {
                // Slow rotation
                motion_msg_t turn = { .cmd = MOTION_CMD_PIVOT_RIGHT, .speed = 18.0f };
                motion_send_cmd(&turn);
                update_map_from_sensor(
                    latest_sensor.f > 0 ? latest_sensor.f : 300.0f,
                    latest_sensor.l > 0 ? latest_sensor.l : 300.0f,
                    latest_sensor.r > 0 ? latest_sensor.r : 300.0f
                );
            } else {
                sweep_ticks = 0;
                motion_msg_t stop = { .cmd = MOTION_CMD_STOP };
                motion_send_cmd(&stop);
                g_scan.phase = SCAN_PHASE_ANALYSIS;
            }
            break;
        }

        case SCAN_PHASE_ANALYSIS:
            analyze_map_build_patrol();
            g_scan.phase = SCAN_PHASE_TRAINING;
            break;

        case SCAN_PHASE_TRAINING:
            feed_ai_training_data();
            g_scan.scan_duration_ms = (esp_timer_get_time() - g_scan.scan_start_time) / 1000;
            g_scan.phase      = SCAN_PHASE_COMPLETE;
            g_scan.map_ready  = true;
            g_scan.scan_active = false;
            ESP_LOGI(TAG, "SCAN COMPLETE in %lldms — map ready for patrol",
                     g_scan.scan_duration_ms);
            break;

        case SCAN_PHASE_COMPLETE:
            // Stay idle, map is persisted in g_scan.grid
            break;

        default: break;
        }

        // Publish scan status to bus (for dashboard)
        typedef struct {
            uint8_t phase;
            float   coverage;
            int     waypoint_idx;
            int     waypoint_count;
            float   pose_x, pose_y, pose_hdg;
            bool    map_ready;
        } scan_status_t;

        scan_status_t ss = {
            .phase          = g_scan.phase,
            .coverage       = g_scan.coverage_pct,
            .waypoint_idx   = g_scan.waypoint_idx,
            .waypoint_count = g_scan.waypoint_count,
            .pose_x         = g_scan.pose.x_cm,
            .pose_y         = g_scan.pose.y_cm,
            .pose_hdg       = g_scan.pose.heading_deg,
            .map_ready      = g_scan.map_ready,
        };
        bus_publish(TOPIC_SENSOR_DATA, &ss, sizeof(ss));
    }
}

// ─── Map JSON Serializer ──────────────────────────────────
void scan_get_map_json(char* buf, size_t maxlen) {
    int n = 0;
    n += snprintf(buf+n, maxlen-n,
        "{\"w\":%d,\"h\":%d,\"cell_cm\":%.0f,\"coverage\":%.1f,"
        "\"phase\":\"%s\",\"patrol_pts\":%d,\"pose\":{\"x\":%.1f,\"y\":%.1f,\"hdg\":%.1f},"
        "\"cells\":\"",
        SCAN_GRID_W, SCAN_GRID_H, SCAN_CELL_SIZE_CM,
        g_scan.coverage_pct,
        scan_phase_name(g_scan.phase),
        g_scan.patrol_count,
        g_scan.pose.x_cm, g_scan.pose.y_cm, g_scan.pose.heading_deg
    );
    // Encode grid as hex string (2 nibbles per cell → 4 states)
    xSemaphoreTake(g_scan.map_lock, portMAX_DELAY);
    for (int gy = 0; gy < SCAN_GRID_H && n < (int)maxlen-8; gy++) {
        for (int gx = 0; gx < SCAN_GRID_W && n < (int)maxlen-8; gx++) {
            buf[n++] = '0' + g_scan.grid[gy][gx].state;
        }
    }
    xSemaphoreGive(g_scan.map_lock);
    n += snprintf(buf+n, maxlen-n, "\"}");
}

void scan_get_patrol_json(char* buf, size_t maxlen) {
    int n = snprintf(buf, maxlen, "{\"count\":%d,\"points\":[", g_scan.patrol_count);
    for (int i = 0; i < g_scan.patrol_count && n < (int)maxlen - 40; i++) {
        n += snprintf(buf+n, maxlen-n,
            "%s[%.1f,%.1f]",
            i>0?",":"",
            g_scan.patrol_route[i].x_cm,
            g_scan.patrol_route[i].y_cm);
    }
    snprintf(buf+n, maxlen-n, "]}");
}

bool  scan_is_complete(void) { return g_scan.phase == SCAN_PHASE_COMPLETE; }
float scan_get_coverage(void){ return g_scan.coverage_pct; }

// ─── Service Lifecycle ────────────────────────────────────
hk_status_t scan_service_init(void) {
    memset(&g_scan, 0, sizeof(g_scan));
    g_scan.map_lock  = xSemaphoreCreateMutex();
    g_scan.phase     = SCAN_PHASE_IDLE;

    bus_subscribe(TOPIC_SENSOR_DATA,  "scan_sensor", &g_scan.sensor_sub);
    bus_subscribe(TOPIC_MOTOR_STATUS, "scan_motor",  &g_scan.motor_status_sub);
    return g_scan.map_lock ? HK_OK : HK_ERR_NOMEM;
}

hk_status_t scan_service_start_scan(void) {
    if (g_scan.scan_active) return HK_ERR_BUSY;
    g_scan.phase      = SCAN_PHASE_INIT;
    g_scan.scan_active = true;
    g_scan.map_ready  = false;

    // Transition AI to IDLE so scan takes motion control
    g_ai.manual_override = true;

    static bool task_created = false;
    if (!task_created) {
        hk_task_descriptor_t* td;
        hk_status_t r = hk_task_create("svc_scan", scan_task, 8192,
                                        NULL, PRIORITY_SENSOR+1, CORE_SERVICES, &td);
        task_created = (r == HK_OK);
    }

    char alert[] = "SCAN MODE ACTIVATED — robot scanning environment";
    bus_publish(TOPIC_SYSTEM_ALERT, alert, sizeof(alert));
    ESP_LOGI(TAG, "Area scan started");
    return HK_OK;
}

hk_status_t scan_service_stop_scan(void) {
    g_scan.scan_active = false;
    g_ai.manual_override = false;
    motion_msg_t stop = { .cmd = MOTION_CMD_STOP };
    motion_send_cmd(&stop);
    g_scan.phase = SCAN_PHASE_IDLE;
    return HK_OK;
}
