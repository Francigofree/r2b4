/**
 * R2B4 WebSocket Manager
 * Bridges the gap between backend updates and the State Store.
 */
class WebSocketManager {
    constructor(store) {
        this.store = store;
        this.ws = null;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 15;
        this.reconnectDelay = 1000;
        this.maxReconnectDelay = 15000;
        this.connected = false;
        this.reconnectTimer = null;
        this.connecting = false;
        this.lastMessageAt = 0;
        this.lastPongAt = 0;
        this.pingTimer = null;
        this.staleTimer = null;
        this.fallbackPollTimer = null;
        this.fallbackPollInFlight = false;
        this.pingIntervalMs = 10000;
        this.staleTimeoutMs = 3000;
        this.fallbackPollIntervalMs = 1000;
        this.connState = {
            transport: 'INIT',
            status: 'CONNECTING',
            ws_connected: false,
            fallback_active: false,
            reconnect_attempts: 0,
            latency_ms: null,
            last_message_ms: 0
        };

        this.pushConnectionState({ status: 'CONNECTING', transport: 'INIT' });
        this.init();
    }

    pushConnectionState(patch = {}) {
        this.connState = {
            ...this.connState,
            ...patch,
            ws_connected: !!this.connected,
            fallback_active: !!this.fallbackPollTimer,
            reconnect_attempts: this.reconnectAttempts,
            last_message_ms: this.lastMessageAt || 0
        };
        this.store.update({ connection: { ...this.connState } });
    }

    init() {
        if (this.connected || this.connecting) return;
        if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
            return;
        }
        try {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/ws/control`;

            this.connecting = true;
            this.pushConnectionState({ status: 'CONNECTING', transport: 'WS' });
            this.ws = new WebSocket(wsUrl);
        } catch (err) {
            console.error('[WS] Init failed:', err);
            this.connected = false;
            this.connecting = false;
            this.pushConnectionState({ status: 'OFFLINE', transport: 'WS' });
            this.handleReconnect();
            this.startFallbackPolling();
            return;
        }

        this.ws.onopen = () => {
            console.log('[WS] Connected to R2B4 Core');
            this.connected = true;
            this.connecting = false;
            this.reconnectAttempts = 0;
            this.lastMessageAt = Date.now();
            if (this.reconnectTimer) {
                clearTimeout(this.reconnectTimer);
                this.reconnectTimer = null;
            }
            this.stopFallbackPolling();
            this.startSocketHealthLoops();
            this.pushConnectionState({ status: 'ONLINE', transport: 'WS' });
            this.dispatchEvent('connected', true);
        };

        this.ws.onmessage = (event) => {
            this.lastMessageAt = Date.now();
            this.pushConnectionState({ status: this.connected ? 'ONLINE' : this.connState.status });
            try {
                const message = JSON.parse(event.data);
                this.processMessage(message);
            } catch (e) {
                console.error('[WS] Metadata parse error:', e);
            }
        };

        this.ws.onclose = () => {
            this.connected = false;
            this.connecting = false;
            this.stopSocketHealthLoops();
            this.pushConnectionState({ status: 'DEGRADED', transport: 'WS' });
            this.dispatchEvent('connected', false);
            this.startFallbackPolling();
            this.handleReconnect();
        };

        this.ws.onerror = (err) => {
            console.error('[WS] Error:', err);
            this.connected = false;
            this.connecting = false;
            this.pushConnectionState({ status: 'DEGRADED', transport: 'WS' });
        };
    }

    startSocketHealthLoops() {
        this.stopSocketHealthLoops();
        this.pingTimer = setInterval(() => {
            if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
            try {
                this.ws.send(JSON.stringify({ type: 'ping', client_ts: Date.now() }));
            } catch (_) {}
        }, this.pingIntervalMs);

        this.staleTimer = setInterval(() => {
            if (!this.connected) return;
            if (!this.lastMessageAt) return;
            const ageMs = Date.now() - this.lastMessageAt;
            if (ageMs <= this.staleTimeoutMs) return;
            this.pushConnectionState({ status: 'DEGRADED', transport: 'WS' });
            try {
                if (this.ws) this.ws.close();
            } catch (_) {}
        }, 500);
    }

    stopSocketHealthLoops() {
        if (this.pingTimer) {
            clearInterval(this.pingTimer);
            this.pingTimer = null;
        }
        if (this.staleTimer) {
            clearInterval(this.staleTimer);
            this.staleTimer = null;
        }
    }

    startFallbackPolling() {
        if (this.fallbackPollTimer) return;
        this.fallbackPollTimer = setInterval(() => {
            this.pollSnapshotOnce();
        }, this.fallbackPollIntervalMs);
        this.pushConnectionState({ transport: 'POLL', status: 'DEGRADED' });
        this.pollSnapshotOnce();
    }

    stopFallbackPolling() {
        if (this.fallbackPollTimer) {
            clearInterval(this.fallbackPollTimer);
            this.fallbackPollTimer = null;
        }
        this.pushConnectionState({ fallback_active: false });
    }

    async pollSnapshotOnce() {
        if (this.connected) return;
        if (this.fallbackPollInFlight) return;
        this.fallbackPollInFlight = true;
        let timeoutId = null;
        let controller = null;
        try {
            controller = new AbortController();
            timeoutId = setTimeout(() => controller.abort(), 900);
            const res = await fetch('/api/realtime/snapshot', {
                cache: 'no-store',
                signal: controller.signal
            });
            if (!res.ok) throw new Error('snapshot-http-' + res.status);
            const payload = await res.json();
            const snap = payload && payload.snapshot ? payload.snapshot : null;
            if (!snap) throw new Error('snapshot-missing');
            this.applySnapshotPayload(snap);
            this.lastMessageAt = Date.now();
            this.pushConnectionState({ transport: 'POLL', status: 'DEGRADED' });
        } catch (_) {
            this.pushConnectionState({ transport: 'POLL', status: 'OFFLINE' });
        } finally {
            if (timeoutId) clearTimeout(timeoutId);
            if (controller) {
                try { controller.abort(); } catch (_) {}
            }
            this.fallbackPollInFlight = false;
        }
    }

    handleReconnect() {
        if (this.reconnectTimer || this.connected || this.connecting) return;
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            const baseDelay = this.reconnectDelay * Math.pow(1.7, this.reconnectAttempts - 1);
            const jitter = 0.85 + Math.random() * 0.3;
            const delay = Math.min(this.maxReconnectDelay, Math.round(baseDelay * jitter));
            console.warn(`[WS] Reconnecting in ${delay}ms...`);
            this.pushConnectionState({ status: 'CONNECTING', transport: 'WS' });
            this.reconnectTimer = setTimeout(() => {
                this.reconnectTimer = null;
                this.init();
            }, delay);
        }
    }

    applyRuntimePayload(payload) {
        const data = payload || {};
        const ekfPayload = data.ekf || {};
        const posePayload = data.pose || {};
        const lidarPayload = data.lidar || {};
        const sensorsDelta = {};
        const ekfUpdate = {
            mode: data.ekf_mode || ekfPayload.mode || 'N/A',
            yaw_diff: Number(ekfPayload.yaw_diff || 0),
            validation_status: ekfPayload.validation_status || 'initializing'
        };
        if (ekfPayload.live) ekfUpdate.live = ekfPayload.live;
        if (ekfPayload.shadow) ekfUpdate.shadow = ekfPayload.shadow;
        if (ekfPayload.shadow_params) ekfUpdate.shadow_params = ekfPayload.shadow_params;
        if (ekfPayload.validation_metrics) ekfUpdate.validation_metrics = ekfPayload.validation_metrics;

        if (Object.prototype.hasOwnProperty.call(data, 'imu')) {
            const imuPayload = data.imu || {};
            sensorsDelta.imu = {
                acc: Array.isArray(imuPayload.accel) ? imuPayload.accel : [0, 0, 0],
                gyr: Array.isArray(imuPayload.gyro) ? imuPayload.gyro : [0, 0, 0],
                mag: imuPayload.mag ?? 0,
                health: imuPayload.health || 'N/A'
            };
        }
        if (Object.prototype.hasOwnProperty.call(data, 'encoder')) {
            sensorsDelta.encoder = data.encoder || {};
        }
        if (Object.prototype.hasOwnProperty.call(data, 'lidar') || Object.prototype.hasOwnProperty.call(data, 'lidar_health') || Object.prototype.hasOwnProperty.call(data, 'lidar_enabled')) {
            const lidarEnabled = (typeof data.lidar_enabled === 'boolean')
                ? data.lidar_enabled
                : (typeof lidarPayload.enabled === 'boolean' ? lidarPayload.enabled : null);
            const lidarHealth = data.lidar_health || lidarPayload.health || 'N/A';
            sensorsDelta.lidar = {
                ...lidarPayload,
                enabled: lidarEnabled,
                health: lidarHealth
            };
            sensorsDelta.lidar_min = lidarPayload.min_dist ?? 0;
        }
        if (Object.prototype.hasOwnProperty.call(data, 'camera_enabled')) {
            sensorsDelta.camera_enabled = data.camera_enabled;
        }
        if (Object.prototype.hasOwnProperty.call(data, 'encoder_enabled')) {
            sensorsDelta.encoder_enabled = data.encoder_enabled;
        }
        if (Object.prototype.hasOwnProperty.call(data, 'peripherals')) {
            sensorsDelta.peripherals = data.peripherals || {};
        }
        if (Object.prototype.hasOwnProperty.call(data, 'hardware')) {
            sensorsDelta.hardware = data.hardware || {};
        }
        if (Object.prototype.hasOwnProperty.call(data, 'startup')) {
            sensorsDelta.startup = data.startup || {};
        }
        if (Object.prototype.hasOwnProperty.call(data, 'encoder_dist_left')) {
            sensorsDelta.encoder_dist_left = data.encoder_dist_left ?? null;
        }
        if (Object.prototype.hasOwnProperty.call(data, 'encoder_dist_right')) {
            sensorsDelta.encoder_dist_right = data.encoder_dist_right ?? null;
        }

        const delta = {
            snapshot_schema_version: data.snapshot_schema_version || 2,
            pose: {
                x: data.x ?? posePayload.x ?? 0,
                y: data.y ?? posePayload.y ?? 0,
                theta: data.theta_deg ?? data.theta ?? posePayload.theta_deg ?? posePayload.theta ?? 0,
                v: data.v ?? posePayload.v ?? 0,
                lastUpdate: Date.now()
            },
            motion: {
                v_target: data.v_target,
                omega_target: data.omega_target,
                v_l: data.v_l,
                v_r: data.v_r,
                pwm_l: data.pwm?.left,
                pwm_r: data.pwm?.right
            },
            safety: {
                allow: data.safety_allow,
                reason: data.safety_reason,
                watchdog_hz: data.loop_hz,
                watchdog_jitter: data.jitter_sec
            },
            motionReadiness: {
                quality: data.motion_quality || {},
                semantics: data.motion_semantics || {},
                public_semantics: data.motion_public || {},
                encoder_reliability: data.encoder_reliability || {},
                encoder_canonical: data.encoder_canonical || {},
                heading_controller: data.heading_controller || {},
                command_overlap: data.command_overlap || { active: false, details: {} },
                estimator_confidence: data.estimator_confidence,
                tuning: data.tuning || {}
            },
            ekf: { ...ekfUpdate }
        };

        if (Object.keys(sensorsDelta).length > 0) {
            delta.sensors = sensorsDelta;
        }
        this.store.update(delta);
    }

    applySnapshotPayload(snapshot) {
        const snap = snapshot || {};
        const status = snap.status || {};
        const pose = status.pose || {};
        const safety = snap.safety || status.safety || {};
        const watchdog = snap.watchdog || status.watchdog || {};
        const runtimePayload = {
            snapshot_schema_version: snap.snapshot_schema_version || 2,
            pose: pose,
            v_target: status.v_target ?? status.v_cmd ?? 0,
            omega_target: status.omega_target ?? 0,
            v_l: status.v_l_raw ?? 0,
            v_r: status.v_r_raw ?? 0,
            pwm: status.pwm || { left: 0, right: 0 },
            safety_allow: safety.allow,
            safety_reason: safety.reason || 'OK',
            loop_hz: watchdog.freq_hz ?? 0,
            jitter_sec: (watchdog.period_sec ?? 0) - 0.02,
            ekf_mode: pose.EKF_mode,
            ekf: status.ekf || {},
            imu: status.imu,
            encoder: status.encoder,
            lidar: status.lidar,
            lidar_enabled: snap.lidar_enabled ?? status.lidar_enabled,
            camera_enabled: snap.camera_enabled ?? status.camera_enabled,
            encoder_enabled: snap.encoder_enabled ?? status.encoder_enabled,
            peripherals: snap.peripherals ?? status.peripherals ?? {},
            lidar_health: snap.lidar_health ?? status.lidar_health,
            hardware: status.hardware,
            startup: status.startup,
            encoder_dist_left: status.encoder_dist_left,
            encoder_dist_right: status.encoder_dist_right,
            motion_quality: status.motion_quality || {},
            motion_semantics: status.motion_semantics || {},
            motion_public: status.motion_public || {},
            encoder_reliability: status.encoder_reliability || {},
            encoder_canonical: status.encoder_canonical || {},
            heading_controller: status.heading_controller || {},
            command_overlap: status.command_overlap || {},
            estimator_confidence: status.estimator_confidence,
            tuning: status.tuning || {}
        };
        this.applyRuntimePayload(runtimePayload);

        const systemStats = snap.system_stats || {};
        if (Object.keys(systemStats).length > 0) {
            this.store.update({
                stats: {
                    cpu: systemStats.cpu_percent,
                    mem: systemStats.memory_percent,
                    temp: systemStats.cpu_temp,
                    volt: systemStats.battery_voltage,
                    ws_clients: systemStats.websocket_clients,
                    uptime: systemStats.uptime_sec
                }
            });
        }
    }

    processMessage(msg) {
        // Map backend message types to State Store updates
        switch (msg.type) {
            case 'pose_update':
                this.applyRuntimePayload(msg.data || {});
                break;

            case 'system_stats':
                this.store.update({
                    stats: {
                        cpu: msg.data.cpu_usage,
                        mem: msg.data.memory_usage,
                        temp: msg.data.cpu_temp,
                        volt: msg.data.battery_voltage,
                        ws_clients: msg.data.websocket_clients,
                        uptime: msg.data.timestamp // Simplified
                    },
                    safety: {
                        emergency_stop: msg.data.emergency_state
                    }
                });
                break;

            case 'realtime_snapshot':
                this.applySnapshotPayload(msg.data || {});
                break;

            case 'pong': {
                this.lastPongAt = Date.now();
                const clientTs = Number(msg.client_ts);
                const latencyMs = Number.isFinite(clientTs) ? Math.max(0, this.lastPongAt - clientTs) : null;
                this.pushConnectionState({
                    status: 'ONLINE',
                    transport: 'WS',
                    latency_ms: latencyMs
                });
                break;
            }

            case 'emergency_activated':
                this.store.update({
                    safety: { emergency_stop: true }
                });
                break;
        }
    }

    sendCommand(type, data = {}) {
        if (this.connected && this.ws) {
            this.ws.send(JSON.stringify({ type, ...data }));
        } else {
            console.error('[WS] Cannot send command: Disconnected');
        }
    }

    dispatchEvent(name, data) {
        window.dispatchEvent(new CustomEvent(`r2b4:${name}`, { detail: data }));
    }
}

window.R2B4_WS = new WebSocketManager(window.R2B4_Store);
