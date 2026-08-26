/**
 * R2B4 Unified State Store
 * Single Source of Truth for the GUI layer.
 * Decouples WebSocket updates from UI rendering.
 */
class StateStore {
    constructor() {
        this.state = {
            // System identity & auth
            version: '6.0',
            snapshot_schema_version: 2,
            authority: 'STANDBY', // ADAPTIVE | GUI | AI | MANUAL

            // Real-time telemetry
            pose: { x: 0, y: 0, theta: 0, v: 0, omega: 0, lastUpdate: 0 },
            motion: {
                v_target: 0,
                omega_target: 0,
                v_l: 0,
                v_r: 0,
                pwm_l: 0,
                pwm_r: 0,
                curr_l: 0,
                curr_r: 0,
                pid_state: {},
                limits: {}
            },
            motionReadiness: {
                quality: {},
                semantics: {},
                encoder_reliability: {},
                encoder_canonical: {},
                heading_controller: {},
                command_overlap: { active: false, details: {} },
                estimator_confidence: 1.0,
                tuning: {}
            },
            robotMotion: {
                intent_x: 0,
                intent_y: 0,
                target_left: 0,
                target_right: 0,
                actual_left: 0,
                actual_right: 0,
                stale: true,
                last_update_ms: 0
            },

            // Navigation & EKF
            ekf: {
                mode: 'OFFLINE',
                live: {
                    yaw: 0,
                    position: [0, 0],
                    velocity: 0,
                    cov_diag: [0, 0, 0, 0, 0],
                    innovation: [0, 0]
                },
                shadow: {
                    yaw: 0,
                    position: [0, 0],
                    velocity: 0,
                    cov_diag: [0, 0, 0, 0, 0],
                    innovation: [0, 0]
                },
                yaw_diff: 0,
                validation_status: 'initializing',
                shadow_params: {
                    Q_yaw: 0,
                    Q_velocity: 0,
                    R_gyro: 0,
                    R_encoder: 0,
                    ZUPT_threshold: 0
                },
                validation_metrics: {
                    mean_innovation: 0,
                    covariance_growth_rate: 0,
                    yaw_stability: 0,
                    stable_frames: 0,
                    error_code: ''
                }
            },

            // Safety & Health
            safety: {
                allow: true,
                reason: 'OK',
                watchdog_hz: 0,
                watchdog_jitter: 0,
                lidar_state: 'OK',
                emergency_stop: false
            },

            // System Stats
            stats: {
                cpu: 0,
                mem: 0,
                temp: 0,
                volt: 12.1,
                uptime: 0,
                latency_ms: 0,
                ws_clients: 0
            },
            connection: {
                transport: 'INIT',
                status: 'CONNECTING',
                ws_connected: false,
                fallback_active: false,
                reconnect_attempts: 0,
                latency_ms: null,
                last_message_ms: 0
            },

            // Environment/Sensors
            sensors: {
                imu: { acc: [0, 0, 0], gyr: [0, 0, 0], mag: 0, health: 'N/A' },
                encoder: {},
                lidar: {},
                camera_enabled: false,
                encoder_enabled: true,
                peripherals: {},
                hardware: {},
                startup: {},
                encoder_dist_left: 0,
                encoder_dist_right: 0,
                tof: [],
                lidar_min: 0
            }
        };

        this.listeners = new Set();
        this.updatePending = false;
    }

    /**
     * Update state with diff/delta.
     * @param {Object} delta Partial state update
     */
    update(delta) {
        let changed = false;

        // Deep merge logic (simplified for top-level keys)
        for (const key in delta) {
            if (typeof delta[key] === 'object' && delta[key] !== null && !Array.isArray(delta[key])) {
                if (!this.state[key]) this.state[key] = {};
                for (const subKey in delta[key]) {
                    if (this.state[key][subKey] !== delta[key][subKey]) {
                        this.state[key][subKey] = delta[key][subKey];
                        changed = true;
                    }
                }
            } else {
                if (this.state[key] !== delta[key]) {
                    this.state[key] = delta[key];
                    changed = true;
                }
            }
        }

        if (changed && !this.updatePending) {
            this.updatePending = true;
            requestAnimationFrame(() => {
                this.notify();
                this.updatePending = false;
            });
        }
    }

    /**
     * Subscribe to state changes.
     * @param {Function} callback 
     */
    subscribe(callback) {
        this.listeners.add(callback);
        return () => this.listeners.delete(callback);
    }

    notify() {
        this.listeners.forEach(cb => cb(this.state));
    }

    getSnapshot() {
        return JSON.parse(JSON.stringify(this.state));
    }
}

window.R2B4_Store = new StateStore();
