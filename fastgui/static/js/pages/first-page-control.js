(function () {
    const ui = {
        // Input
        joyModeStatus: document.getElementById('joy-mode-status'),
        joyModeStatusLabel: document.getElementById('joy-mode-status-label'),
        joyTurnLeft90Btn: document.getElementById('joy-turn-left-90-btn'),
        joyPad: document.getElementById('joystick-drive-pad'),
        joyKnob: document.getElementById('joystick-knob'),
        joyXReadout: document.getElementById('joy-x-readout'),
        joyYReadout: document.getElementById('joy-y-readout'),
        sliderX: document.getElementById('intent-slider-x'),
        sliderY: document.getElementById('intent-slider-y'),
        sliderXValue: document.getElementById('intent-slider-x-value'),
        sliderYValue: document.getElementById('intent-slider-y-value'),
        simInputMode: document.getElementById('sim-input-mode'),
        simJoyState: document.getElementById('sim-joy-state'),
        simSysState: document.getElementById('sim-sys-state'),
        simStreamState: document.getElementById('sim-stream-state'),
        simTelemetryAge: document.getElementById('sim-telemetry-age'),

        simTargetLeftLabel: document.getElementById('sim-target-left'),
        simTargetRightLabel: document.getElementById('sim-target-right'),
        simLeftTargetReadout: document.getElementById('sim-left-target-readout'),
        simRightTargetReadout: document.getElementById('sim-right-target-readout'),
        simLeftActualReadout: document.getElementById('sim-left-actual-readout'),
        simRightActualReadout: document.getElementById('sim-right-actual-readout'),
        simPoseInline: document.getElementById('sim-pose-inline'),
        simYawInline: document.getElementById('sim-yaw-inline'),
        simSpeedInline: document.getElementById('sim-speed-inline'),
        simAutoTurnQa: document.getElementById('sim-auto-turn-qa'),
        simAutoTurnMetrics: document.getElementById('sim-auto-turn-metrics'),
        simCommandInput: document.getElementById('sim-command-input'),
        simCommandExec: document.getElementById('sim-command-exec'),
        cameraLidarMap: document.getElementById('camera-lidar-map'),

        // Simulator status
        simSteering: document.getElementById('sim-steering-state'),
        simFailsafe: document.getElementById('sim-failsafe-state'),
        simStale: document.getElementById('sim-stale-state'),

        // Peripheral toggle buttons
        periphBtnLid: document.getElementById('periph-btn-lid'),
        periphBtnEnc: document.getElementById('periph-btn-enc'),

        // Telemetry bars
        targetLeftBar: document.getElementById('left-track-pwm'),
        targetRightBar: document.getElementById('right-track-pwm'),
        actualLeftBar: document.getElementById('left-track-speed'),
        actualRightBar: document.getElementById('right-track-speed'),
        leftTop: document.getElementById('left-track-stats'),
        leftBottom: document.getElementById('left-track-stats-bottom'),
        rightTop: document.getElementById('right-track-stats'),
        rightBottom: document.getElementById('right-track-stats-bottom'),

        // Header
        hdrConnection: document.getElementById('hdr-robot-connection'),
        hdrIntent: document.getElementById('hdr-intent-vector'),
        hdrTarget: document.getElementById('hdr-motor-targets'),
        hdrActual: document.getElementById('hdr-motor-actual'),
        hdrFailsafe: document.getElementById('hdr-failsafe-status'),
        hdrEncoderToggle: document.getElementById('hdr-encoder-toggle'),
        hdrEncoderState: document.getElementById('hdr-encoder-state'),

    };

    const state = {
        keys: { up: false, left: false, down: false, right: false },
        resetHotkey: {
            lastAtMs: 0,
            minIntervalMs: 700
        },
        autoTurn: {
            active: false,
            stopping: false,
            startHeadingDeg: 0,
            startedAtMs: 0,
            startPoseX: 0,
            startPoseY: 0,
            timer: null,
            targetDeltaDeg: -90.0,
            stopToleranceDeg: 1.5,
            timeoutMs: 22000,
            lockUntilMs: 0,
            debounceMs: 650,
            telemetryFreshMs: 900,
            minStartFreshMs: 400,
            commandLeft: -0.18,
            commandRight: 0.18,
            commandFastAbs: 0.18,
            commandSlowAbs: 0.13,
            commandFineAbs: 0.09,
            slowZoneDeg: 34,
            fineZoneDeg: 12,
            commandRefreshMs: 220,
            lastCommandSentAtMs: 0,
            vTargetLeakEps: 0.015,
            signEps: 0.02,
            maxYawErrorDeg: 1.5,
            maxDriftM: 0.02,
            maxLeakRatio: 0.25,
            minCmdOppRatio: 0.70,
            quality: null,
            lastResult: null
        },
        currentHeadingDeg: 0,
        currentPoseX: 0,
        currentPoseY: 0,
        latestRuntime: {
            vTarget: 0,
            pwmL: 0,
            pwmR: 0,
            vLRaw: 0,
            vRRaw: 0,
            watchdogHz: 0
        },
        joyPointerId: null,
        sse: null,
        sseReconnectTimer: null,
        sseReconnectDelayMs: 1200,
        sseReconnectMaxDelayMs: 8000,
        lastTelemetryAt: 0,
        watchdogTimer: null,
        lastRenderedIntent: { x: 0, y: 0 },
        controlMode: 'UNIFIED',
        encoderEnabled: true,
        encoderBusy: false,
        lidarOverlay: {
            timer: null,
            inFlight: false,
            intervalMs: 320,
            rangeM: 4.0
        },
    };

    function clamp(v, minV, maxV) {
        return Math.max(minV, Math.min(maxV, v));
    }

    function num(v, fallback) {
        const n = Number(v);
        return Number.isFinite(n) ? n : fallback;
    }

    function normalizeAngleDeg(a) {
        const n = num(a, 0);
        return ((n + 180) % 360 + 360) % 360 - 180;
    }

    function angleDeltaDeg(startDeg, currentDeg) {
        return normalizeAngleDeg(num(currentDeg, 0) - num(startDeg, 0));
    }

    function setText(node, value) {
        if (!node) return;
        const txt = String(value ?? '');
        if (node.textContent !== txt) {
            node.textContent = txt;
        }
    }

    function isTelemetryFresh(maxAgeMs) {
        if (!state.lastTelemetryAt) return false;
        return (Date.now() - state.lastTelemetryAt) <= Math.max(100, num(maxAgeMs, 900));
    }

    function hasOppositeSign(a, b, eps) {
        const x = num(a, 0);
        const y = num(b, 0);
        const e = Math.max(0.0001, num(eps, 0.02));
        if (Math.abs(x) < e || Math.abs(y) < e) return false;
        return x * y < 0;
    }

    function hasSameSign(a, b, eps) {
        const x = num(a, 0);
        const y = num(b, 0);
        const e = Math.max(0.0001, num(eps, 0.02));
        if (Math.abs(x) < e || Math.abs(y) < e) return false;
        return x * y > 0;
    }

    function createAutoTurnQualityState() {
        return {
            samples: 0,
            vTargetLeakCount: 0,
            cmdOppositeCount: 0,
            cmdSameCount: 0,
            rawOppositeCount: 0,
            rawSameCount: 0,
            staleSampleCount: 0
        };
    }

    function setAutoTurnQa(statusText, metricsText) {
        if (ui.simAutoTurnQa) setText(ui.simAutoTurnQa, statusText || 'IDLE');
        if (ui.simAutoTurnMetrics) setText(ui.simAutoTurnMetrics, metricsText || 'n/a');
    }

    function sampleAutoTurnQuality() {
        if (!state.autoTurn.active || !state.autoTurn.quality) return;
        const q = state.autoTurn.quality;
        const rt = state.latestRuntime;
        q.samples += 1;
        if (Math.abs(num(rt.vTarget, 0)) > num(state.autoTurn.vTargetLeakEps, 0.015)) {
            q.vTargetLeakCount += 1;
        }
        if (hasOppositeSign(rt.pwmL, rt.pwmR, state.autoTurn.signEps)) {
            q.cmdOppositeCount += 1;
        } else if (hasSameSign(rt.pwmL, rt.pwmR, state.autoTurn.signEps)) {
            q.cmdSameCount += 1;
        }
        if (hasOppositeSign(rt.vLRaw, rt.vRRaw, state.autoTurn.signEps)) {
            q.rawOppositeCount += 1;
        } else if (hasSameSign(rt.vLRaw, rt.vRRaw, state.autoTurn.signEps)) {
            q.rawSameCount += 1;
        }
        if (!isTelemetryFresh(state.autoTurn.telemetryFreshMs)) {
            q.staleSampleCount += 1;
        }
    }

    function buildAutoTurnResult(flags) {
        const opts = flags || {};
        const q = state.autoTurn.quality || createAutoTurnQualityState();
        const nowMs = Date.now();
        const deltaDeg = angleDeltaDeg(state.autoTurn.startHeadingDeg, state.currentHeadingDeg);
        const durationMs = Math.max(0, nowMs - num(state.autoTurn.startedAtMs, nowMs));
        const dx = num(state.currentPoseX, 0) - num(state.autoTurn.startPoseX, 0);
        const dy = num(state.currentPoseY, 0) - num(state.autoTurn.startPoseY, 0);
        const driftM = Math.hypot(dx, dy);
        const yawErrorDeg = Math.abs(deltaDeg - num(state.autoTurn.targetDeltaDeg, -90.0));
        const cmdTotal = q.cmdOppositeCount + q.cmdSameCount;
        const rawTotal = q.rawOppositeCount + q.rawSameCount;
        const cmdOppRatio = cmdTotal > 0 ? (q.cmdOppositeCount / cmdTotal) : 0;
        const rawOppRatio = rawTotal > 0 ? (q.rawOppositeCount / rawTotal) : 0;
        const leakRatio = q.samples > 0 ? (q.vTargetLeakCount / q.samples) : 0;
        const timedOut = !!opts.timedOut;
        const staleAbort = !!opts.staleAbort;
        const pass = (
            !timedOut &&
            !staleAbort &&
            yawErrorDeg <= num(state.autoTurn.maxYawErrorDeg, 1.5) &&
            driftM <= num(state.autoTurn.maxDriftM, 0.02) &&
            leakRatio <= num(state.autoTurn.maxLeakRatio, 0.25) &&
            cmdOppRatio >= num(state.autoTurn.minCmdOppRatio, 0.70) &&
            q.staleSampleCount === 0
        );

        return {
            pass: !!pass,
            timed_out: timedOut,
            stale_abort: staleAbort,
            delta_deg: Number(deltaDeg.toFixed(3)),
            yaw_error_deg: Number(yawErrorDeg.toFixed(3)),
            duration_ms: Math.round(durationMs),
            drift_m: Number(driftM.toFixed(4)),
            drift_cm: Number((driftM * 100).toFixed(2)),
            v_target_leak_ratio: Number(leakRatio.toFixed(3)),
            cmd_opposite_ratio: Number(cmdOppRatio.toFixed(3)),
            raw_opposite_ratio: Number(rawOppRatio.toFixed(3)),
            sample_count: q.samples,
            stale_sample_count: q.staleSampleCount
        };
    }

    function renderAutoTurnResult(result) {
        if (!result) {
            setAutoTurnQa('IDLE', 'n/a');
            return;
        }
        const tag = result.pass ? 'PASS' : 'WARN';
        const qa = [
            tag,
            'cmdOpp ' + (result.cmd_opposite_ratio * 100).toFixed(0) + '%',
            'leak ' + (result.v_target_leak_ratio * 100).toFixed(0) + '%'
        ].join(' | ');
        const met = [
            'Δ=' + num(result.delta_deg, 0).toFixed(1) + '°',
            't=' + (num(result.duration_ms, 0) / 1000).toFixed(2) + 's',
            'drift=' + num(result.drift_cm, 0).toFixed(1) + 'cm'
        ].join(' | ');
        setAutoTurnQa(qa, met);
    }

    function normalizeControlMode(mode) {
        return 'UNIFIED';
    }

    function controlModeLabel(mode) {
        return 'UNIFIED';
    }

    function renderControlMode(mode) {
        const normalized = normalizeControlMode(mode) || 'UNIFIED';
        state.controlMode = normalized;
        const label = controlModeLabel(normalized);
        if (ui.joyModeStatus) {
            ui.joyModeStatus.title = 'Mozgásvezérlés: ' + label;
        }
        setText(ui.joyModeStatusLabel, label);
    }

    function renderHeaderEncoder(enabled) {
        const on = !!enabled;
        state.encoderEnabled = on;
        setText(ui.hdrEncoderState, on ? 'BE' : 'KI');
        if (ui.hdrEncoderToggle) {
            ui.hdrEncoderToggle.classList.toggle('nominal', on);
            ui.hdrEncoderToggle.classList.toggle('warning', !on);
            ui.hdrEncoderToggle.disabled = !!state.encoderBusy;
            ui.hdrEncoderToggle.setAttribute('aria-pressed', on ? 'true' : 'false');
            ui.hdrEncoderToggle.title = on ? 'Encoder adatfelhasználás: BE' : 'Encoder adatfelhasználás: KI';
        }
    }

    async function refreshHeaderEncoder() {
        try {
            const response = await fetch('/api/status', { method: 'GET' });
            if (!response.ok) return;
            const payload = await response.json().catch(function () { return null; });
            const peripherals = (payload && payload.peripherals) || {};
            if (payload && typeof payload.encoder_enabled === 'boolean') {
                renderHeaderEncoder(payload.encoder_enabled);
            } else if (typeof peripherals.encoder === 'boolean') {
                renderHeaderEncoder(peripherals.encoder);
            }
        } catch (_) {
            // Halk hibatűrés.
        }
    }

    async function toggleHeaderEncoder() {
        if (!ui.hdrEncoderToggle || state.encoderBusy) return;
        const target = !state.encoderEnabled;
        state.encoderBusy = true;
        renderHeaderEncoder(state.encoderEnabled);
        try {
            const response = await fetch('/api/encoder-toggle', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ enabled: target })
            });
            if (!response.ok) throw new Error('encoder-toggle-failed');
            const payload = await response.json().catch(function () { return {}; });
            const peripherals = (payload && payload.peripherals) || {};
            const confirmed = (payload && typeof payload.enabled === 'boolean')
                ? payload.enabled
                : (typeof peripherals.encoder === 'boolean' ? peripherals.encoder : target);
            renderHeaderEncoder(confirmed);
        } catch (_) {
            await refreshHeaderEncoder();
        } finally {
            state.encoderBusy = false;
            renderHeaderEncoder(state.encoderEnabled);
        }
    }

    async function refreshControlMode() {
        try {
            const response = await fetch('/api/control-mode', { method: 'GET' });
            if (!response.ok) return;
            const payload = await response.json();
            const mode = normalizeControlMode(payload && payload.control_mode);
            if (mode) renderControlMode(mode);
        } catch (err) {
            // A mozgásvezérlést ne blokkoljuk hálózati hiba miatt.
        }
    }

    function writeIntentReadouts(x, y) {
        setText(ui.joyXReadout, (x >= 0 ? '+' : '') + x.toFixed(2));
        setText(ui.joyYReadout, (y >= 0 ? '+' : '') + y.toFixed(2));
        setText(ui.sliderXValue, x.toFixed(2));
        setText(ui.sliderYValue, y.toFixed(2));
        if (ui.sliderX) ui.sliderX.value = String(Math.round(x * 100));
        if (ui.sliderY) ui.sliderY.value = String(Math.round(y * 100));
    }

    function renderKnob(x, y) {
        if (!ui.joyKnob) return;
        const tx = x * 34;
        const ty = -y * 34;
        ui.joyKnob.style.transform = 'translate(' + tx.toFixed(0) + 'px,' + ty.toFixed(0) + 'px)';
        if (ui.joyPad) {
            var strength = Math.min(1, Math.hypot(x, y));
            ui.joyPad.style.boxShadow = 'inset 0 0 24px rgba(92,245,149,0.14), 0 0 ' +
                (8 + strength * 20).toFixed(0) + 'px rgba(109,255,165,' + (0.15 + strength * 0.35).toFixed(2) + ')';
        }
    }

    function publishIntent(x, y, source) {
        const motion = window.R2B4_MotionController;
        if (!motion) return;
        const intent = motion.setIntent(x, y, normalizeIntentSource(source));
        state.lastRenderedIntent = intent;
        renderKnob(intent.x, intent.y);
        writeIntentReadouts(intent.x, intent.y);
    }

    function normalizeIntentSource(source) {
        const raw = String(source || '').toUpperCase();
        if (raw === 'GUI_JOYSTICK') return raw;
        // A GUI al-forrásokat közös joystick forrásként kezeljük az arbiterhez.
        return 'GUI_JOYSTICK';
    }

    function stopIntent(source) {
        const motion = window.R2B4_MotionController;
        if (!motion) return;
        const intent = motion.stop(normalizeIntentSource(source || 'HYBRID'));
        state.lastRenderedIntent = intent || { x: 0, y: 0 };
        renderKnob(state.lastRenderedIntent.x, state.lastRenderedIntent.y);
        writeIntentReadouts(state.lastRenderedIntent.x, state.lastRenderedIntent.y);
    }

    function directionFromIntent(x, y) {
        if (Math.abs(x) < 0.05 && Math.abs(y) < 0.05) return 'ÁLL';
        if (Math.abs(y) >= Math.abs(x)) return y > 0 ? 'ELŐRE' : 'HÁTRA';
        return x > 0 ? 'JOBBRA FORDUL' : 'BALRA FORDUL';
    }

    function buildReadinessSummary(readiness, safety, stale) {
        const q = (readiness && readiness.quality) || {};
        const enc = (readiness && readiness.encoder_reliability) || {};
        const overlap = (readiness && readiness.command_overlap) || {};
        const sem = (readiness && readiness.semantics) || {};
        const tuning = (readiness && readiness.tuning) || {};
        const tuningEkf = (tuning && tuning.ekf) || {};
        const tuningPid = (tuning && tuning.pid) || {};
        const reasons = Array.isArray(q.degradation_reasons) ? q.degradation_reasons : [];
        const qualityState = String(q.quality_state || '').toUpperCase();
        const estFromQ = num((q.estimator_consistency || {}).confidence, 1.0);
        const estConf = num((readiness && readiness.estimator_confidence), estFromQ);
        const safetyAllow = !(safety && safety.allow === false);
        const encUsageMode = String((enc && enc.ekf_usage_mode) || 'NORMAL').toUpperCase();
        const encAsym = num((enc && enc.asymmetry_score), num((enc && enc.side_asymmetry), 0));
        const encTrust = num((enc && enc.combined_trust), 1.0);
        const encTimingDegraded = !!(enc && enc.degraded_timing);
        const encStale = !!(enc && enc.snapshot_stale);
        const hasTuneData = (
            (tuning && typeof tuning === 'object' && Object.keys(tuning).length > 0) ||
            typeof (readiness && readiness.ekf_tune_ready) === 'boolean' ||
            typeof (readiness && readiness.pid_tune_ready) === 'boolean'
        );
        const ekfTuneReady = (typeof tuningEkf.ready === 'boolean')
            ? tuningEkf.ready
            : (typeof (readiness && readiness.ekf_tune_ready) === 'boolean' ? readiness.ekf_tune_ready : true);
        const pidTuneReady = (typeof tuningPid.ready === 'boolean')
            ? tuningPid.ready
            : (typeof (readiness && readiness.pid_tune_ready) === 'boolean' ? readiness.pid_tune_ready : true);

        const tags = [];
        if (stale) tags.push('INTENT');
        if (!safetyAllow) tags.push('SAFE');
        if (overlap && overlap.active) tags.push('CMD');
        if (enc && (enc.anomaly_active || (Array.isArray(enc.flags) && enc.flags.length > 0))) tags.push('ENC');
        if (encAsym >= 0.35) tags.push('ASYM');
        if (encTrust < 0.65) tags.push('TRUST');
        if (encUsageMode === 'REJECT') tags.push('EKF_REJ');
        if (encTimingDegraded) tags.push('TIMING');
        if (encStale) tags.push('STALE');
        if (estConf < 0.65) tags.push('EST');
        if ((sem && sem.semantic_state === 'CURVED') || reasons.indexOf('ESTIMATOR_GATE_REJECT') >= 0) tags.push('DRIFT');
        if (hasTuneData && !ekfTuneReady) tags.push('EKF');
        if (hasTuneData && !pidTuneReady) tags.push('PID');

        let sysState = 'OK';
        if (!safetyAllow || qualityState === 'CRITICAL' || estConf < 0.35) {
            sysState = 'CRIT';
        } else if (qualityState === 'DEGRADED' || tags.length > 0) {
            sysState = 'DEG';
        }

        let hdrText = 'OK';
        if (stale) {
            hdrText = 'INTENT TIMEOUT';
        } else if (!safetyAllow) {
            const reason = String((safety && safety.reason) || 'BLOCK');
            hdrText = 'SAFETY ' + reason;
        } else if (overlap && overlap.active) {
            hdrText = 'CMD CONFLICT';
        } else if (encUsageMode === 'REJECT') {
            hdrText = 'EKF ENCODER REJECT';
        } else if (encStale) {
            hdrText = 'ENCODER STALE';
        } else if (encAsym >= 0.65) {
            hdrText = 'ENCODER ASYMMETRY';
        } else if (encTimingDegraded) {
            hdrText = 'ENCODER TIMING DEGRADED';
        } else if (enc && (enc.anomaly_active || (Array.isArray(enc.flags) && enc.flags.length > 0))) {
            hdrText = 'ENCODER ANOM';
        } else if (estConf < 0.65) {
            hdrText = 'EST TRUST ' + Math.round(estConf * 100) + '%';
        } else if (sem && sem.semantic_state === 'CURVED') {
            hdrText = 'CURVED MOTION';
        } else if (hasTuneData && (!ekfTuneReady || !pidTuneReady)) {
            hdrText = (!ekfTuneReady && !pidTuneReady) ? 'EKF+PID TUNE WAIT' : (!ekfTuneReady ? 'EKF TUNE WAIT' : 'PID TUNE WAIT');
        }
        const tuneText = hasTuneData
            ? ('EKF ' + (ekfTuneReady ? 'READY' : 'WAIT') + ' | PID ' + (pidTuneReady ? 'READY' : 'WAIT'))
            : 'EKF/PID N/A';

        return {
            sysText: tags.length > 0 ? (sysState + ' ' + tags.join('+')) : sysState,
            hdrText: hdrText,
            critical: sysState === 'CRIT',
            degraded: sysState === 'DEG',
            tuneText: tuneText
        };
    }

    function renderMotionTelemetry(robotMotion, readiness, safety) {
        const ix = num(robotMotion.intent_x, 0);
        const iy = num(robotMotion.intent_y, 0);
        const tl = num(robotMotion.target_left, 0);
        const tr = num(robotMotion.target_right, 0);
        const al = num(robotMotion.actual_left, 0);
        const ar = num(robotMotion.actual_right, 0);
        const stale = !!robotMotion.stale;
        const readinessSummary = buildReadinessSummary(readiness || {}, safety || {}, stale);

        setText(ui.hdrIntent, 'x=' + ix.toFixed(2) + ' | y=' + iy.toFixed(2));
        setText(ui.hdrTarget, 'L=' + tl.toFixed(2) + ' | R=' + tr.toFixed(2));
        setText(ui.hdrActual, 'L=' + al.toFixed(2) + ' | R=' + ar.toFixed(2));
        setText(ui.hdrFailsafe, readinessSummary.hdrText);
        if (ui.hdrFailsafe) {
            ui.hdrFailsafe.className = readinessSummary.critical ? 'critical' : (readinessSummary.degraded ? 'warning' : 'nominal');
        }

        setText(ui.simFailsafe, readinessSummary.hdrText === 'OK' ? 'NORMÁL' : readinessSummary.hdrText);
        setText(ui.simStale, stale ? 'IGEN' : 'NEM');
        setText(ui.simSysState, readinessSummary.sysText + ' | ' + readinessSummary.tuneText);

        setText(ui.leftTop, 'T: ' + tl.toFixed(2) + ' | A: ' + al.toFixed(2));
        setText(ui.leftBottom, 'TARGET / ACTUAL');
        setText(ui.rightTop, 'T: ' + tr.toFixed(2) + ' | A: ' + ar.toFixed(2));
        setText(ui.rightBottom, 'TARGET / ACTUAL');
        setText(ui.simTargetRightLabel, 'R cél: ' + tr.toFixed(2));
        setText(ui.simLeftTargetReadout, tl.toFixed(2));
        setText(ui.simRightTargetReadout, tr.toFixed(2));
        setText(ui.simLeftActualReadout, al.toFixed(2));
        setText(ui.simRightActualReadout, ar.toFixed(2));

        if (ui.targetLeftBar) ui.targetLeftBar.style.height = Math.round(clamp(Math.abs(tl), 0, 1) * 100) + '%';
        if (ui.targetRightBar) ui.targetRightBar.style.height = Math.round(clamp(Math.abs(tr), 0, 1) * 100) + '%';
        if (ui.actualLeftBar) ui.actualLeftBar.style.height = Math.round(clamp(Math.abs(al), 0, 1) * 100) + '%';
        if (ui.actualRightBar) ui.actualRightBar.style.height = Math.round(clamp(Math.abs(ar), 0, 1) * 100) + '%';
    }

    function renderVisionInline(pose, motion) {
        const p = pose || {};
        const m = motion || {};
        const x = num(p.x, 0);
        const y = num(p.y, 0);
        const yaw = num(p.theta, 0);
        const measuredV = num(p.v, (num(m.v_l, 0) + num(m.v_r, 0)) * 0.5);
        setText(ui.simPoseInline, x.toFixed(2) + ' / ' + y.toFixed(2) + ' m');
        setText(ui.simYawInline, yaw.toFixed(1) + '°');
        setText(ui.simSpeedInline, measuredV.toFixed(2) + ' m/s');
    }

    function lidarPointDistanceM(point) {
        if (!point) return 0;
        let raw = NaN;
        if (Array.isArray(point)) {
            raw = num(point[1], NaN);
        } else if (typeof point === 'object') {
            if (Number.isFinite(Number(point.dist_m))) return Number(point.dist_m);
            if (Number.isFinite(Number(point.distance_m))) return Number(point.distance_m);
            if (Number.isFinite(Number(point.dist_mm))) return Number(point.dist_mm) / 1000;
            if (Number.isFinite(Number(point.distance_mm))) return Number(point.distance_mm) / 1000;
            raw = num(point.dist, num(point.distance, NaN));
        }
        if (!Number.isFinite(raw)) return 0;
        if (Math.abs(raw) > 25) return raw / 1000;
        return raw;
    }

    function lidarPointAngleDeg(point) {
        if (!point) return 0;
        if (Array.isArray(point)) return num(point[0], 0);
        return num(point.angle, num(point.angle_deg, 0));
    }

    function drawLidarOverlayBase(ctx, width, height) {
        const cx = width / 2;
        const cy = height / 2;
        const maxR = Math.max(8, Math.min(cx, cy) - 8);
        ctx.clearRect(0, 0, width, height);
        ctx.strokeStyle = 'rgba(88, 186, 136, 0.35)';
        ctx.lineWidth = 1;
        for (let i = 1; i <= 4; i += 1) {
            const r = (maxR / 4) * i;
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, Math.PI * 2);
            ctx.stroke();
        }
        ctx.beginPath();
        ctx.moveTo(cx, 4);
        ctx.lineTo(cx, height - 4);
        ctx.moveTo(4, cy);
        ctx.lineTo(width - 4, cy);
        ctx.stroke();
        ctx.fillStyle = '#8afeb6';
        ctx.beginPath();
        ctx.arc(cx, cy, 2.3, 0, Math.PI * 2);
        ctx.fill();
        return { cx: cx, cy: cy, maxR: maxR };
    }

    function initLidarOverlay() {
        if (!ui.cameraLidarMap) return;
        const canvas = ui.cameraLidarMap;
        const ctx = canvas.getContext ? canvas.getContext('2d') : null;
        if (!ctx) return;
        const rangeM = Math.max(0.5, num(state.lidarOverlay.rangeM, 4.0));

        async function tick() {
            if (state.lidarOverlay.inFlight) return;
            state.lidarOverlay.inFlight = true;
            const base = drawLidarOverlayBase(ctx, canvas.width, canvas.height);
            try {
                const res = await fetch('/api/lidar-scan', { cache: 'no-store' });
                if (!res.ok) throw new Error('lidar-scan-http-' + res.status);
                const payload = await res.json().catch(function () { return {}; });
                const scan = Array.isArray(payload.scan)
                    ? payload.scan
                    : (Array.isArray(payload.points) ? payload.points : []);
                if (scan.length > 0) {
                    const stride = scan.length > 420 ? 2 : 1;
                    ctx.fillStyle = 'rgba(120, 255, 170, 0.92)';
                    for (let i = 0; i < scan.length; i += stride) {
                        const p = scan[i];
                        const dM = clamp(lidarPointDistanceM(p), 0, rangeM);
                        if (dM <= 0) continue;
                        const angleRad = (lidarPointAngleDeg(p) - 90) * (Math.PI / 180);
                        const r = (dM / rangeM) * base.maxR;
                        const x = base.cx + r * Math.cos(angleRad);
                        const y = base.cy + r * Math.sin(angleRad);
                        ctx.fillRect(x, y, 1.4, 1.4);
                    }
                } else {
                    ctx.fillStyle = 'rgba(255, 200, 120, 0.9)';
                    ctx.fillRect(base.cx - 1.5, base.cy - 1.5, 3, 3);
                }
            } catch (_) {
                ctx.fillStyle = 'rgba(255, 120, 120, 0.9)';
                ctx.fillRect(base.cx - 2, base.cy - 2, 4, 4);
            } finally {
                state.lidarOverlay.inFlight = false;
            }
        }

        tick();
        if (state.lidarOverlay.timer) clearInterval(state.lidarOverlay.timer);
        state.lidarOverlay.timer = setInterval(function () {
            tick();
        }, Math.max(120, num(state.lidarOverlay.intervalMs, 320)));
    }

    function connectMotionSSE() {
        if (state.sseReconnectTimer) {
            clearTimeout(state.sseReconnectTimer);
            state.sseReconnectTimer = null;
        }
        if (state.sse) {
            try { state.sse.close(); } catch (e) { }
        }
        let es = null;
        try {
            es = new EventSource('/api/runtime/motion_state/stream');
        } catch (_) {
            scheduleMotionSSEReconnect();
            return;
        }
        es.onopen = function () {
            state.sseReconnectDelayMs = 1200;
            setText(ui.simStreamState, 'ONLINE');
        };
        es.onmessage = function (event) {
            try {
                const payload = JSON.parse(event.data || '{}');
                const mt = payload.motion_telemetry || {};
                const readiness = {
                    quality: payload.motion_quality || {},
                    semantics: payload.motion_semantics || {},
                    encoder_reliability: payload.encoder_reliability || {},
                    encoder_canonical: payload.encoder_canonical || {},
                    heading_controller: payload.heading_controller || {},
                    command_overlap: payload.command_overlap || { active: false, details: {} },
                    estimator_confidence: payload.estimator_confidence,
                    tuning: payload.tuning || {},
                    ekf_tune_ready: payload.ekf_tune_ready,
                    pid_tune_ready: payload.pid_tune_ready
                };
                const modeFromStream = normalizeControlMode(payload.control_mode || mt.control_mode);
                const robotMotion = {
                    intent_x: num(mt.intent_x, 0),
                    intent_y: num(mt.intent_y, 0),
                    target_left: num(mt.target_left, 0),
                    target_right: num(mt.target_right, 0),
                    actual_left: num(mt.actual_left, 0),
                    actual_right: num(mt.actual_right, 0),
                    stale: !!mt.stale,
                    last_update_ms: Date.now()
                };
                state.lastTelemetryAt = robotMotion.last_update_ms;
                if (modeFromStream) {
                    renderControlMode(modeFromStream);
                }
                if (window.R2B4_Store && window.R2B4_Store.update) {
                    const delta = {
                        robotMotion: robotMotion,
                        motionReadiness: readiness
                    };
                    if (payload.peripherals && typeof payload.peripherals === 'object') {
                        delta.sensors = { peripherals: payload.peripherals };
                        if (typeof payload.peripherals.encoder === 'boolean') {
                            delta.sensors.encoder_enabled = payload.peripherals.encoder;
                        }
                    }
                    window.R2B4_Store.update(delta);
                }
            } catch (err) {
                // Hibás csomag esetén csendes kihagyás.
            }
        };
        es.onerror = function () {
            try { es.close(); } catch (e) { }
            state.sse = null;
            setText(ui.simStreamState, 'RECONNECT');
            scheduleMotionSSEReconnect();
        };
        setText(ui.simStreamState, 'CONNECTING');
        state.sse = es;
    }

    function scheduleMotionSSEReconnect() {
        if (state.sseReconnectTimer) return;
        const base = Math.max(400, num(state.sseReconnectDelayMs, 1200));
        const jitter = 0.85 + Math.random() * 0.3;
        const waitMs = Math.round(base * jitter);
        state.sseReconnectTimer = setTimeout(function () {
            state.sseReconnectTimer = null;
            connectMotionSSE();
        }, waitMs);
        state.sseReconnectDelayMs = Math.min(
            num(state.sseReconnectMaxDelayMs, 8000),
            Math.round(base * 1.5)
        );
    }

    function startConnectionWatchdog() {
        if (state.watchdogTimer) clearInterval(state.watchdogTimer);
        state.watchdogTimer = setInterval(function () {
            const ageMs = Date.now() - state.lastTelemetryAt;
            const alive = ageMs < 1200;
            setText(ui.hdrConnection, alive ? 'ONLINE' : 'OFFLINE');
            setText(ui.simTelemetryAge, Math.max(0, Math.round(ageMs)) + 'ms');
            setText(ui.simStreamState, alive ? 'ONLINE' : 'OFFLINE');
            if (ui.hdrConnection) {
                ui.hdrConnection.className = alive ? 'nominal' : 'critical';
            }
        }, 300);
    }

    function shapeAnalogAxis(v, expo, deadzone) {
        const dz = Number(deadzone || 0.06);
        const absV = Math.abs(v);
        if (absV < dz) return 0;
        const normalized = (absV - dz) / (1 - dz);
        const shaped = Math.pow(clamp(normalized, 0, 1), Number(expo || 1.0));
        return Math.sign(v) * shaped;
    }

    function analogFromPoint(clientX, clientY) {
        if (!ui.joyPad) return { x: 0, y: 0 };
        const rect = ui.joyPad.getBoundingClientRect();
        const cx = rect.left + rect.width / 2;
        const cy = rect.top + rect.height / 2;
        // 0.41: nagyobb hasznos analóg tér, stabilabb kis kitérés.
        const radius = Math.min(rect.width, rect.height) * 0.41;
        let xRaw = (clientX - cx) / radius;
        let yRaw = (cy - clientY) / radius;
        let x = xRaw;
        let y = yRaw;
        const mag = Math.hypot(x, y);
        if (mag > 1) {
            x = x / mag;
            y = y / mag;
        }
        // R2B4 karakterisztika:
        // - Thrust (Y): lágy indulás, pontos lassú haladás
        // - Steer (X): enyhén agresszívebb középtartomány, de nagy thrust mellett csillapított kormányzás
        const yShaped = shapeAnalogAxis(y, 1.55, 0.06);
        const xBase = shapeAnalogAxis(x, 1.25, 0.05);
        const thrustAbs = Math.abs(yShaped);
        const steerMix = 1.0 - (0.38 * thrustAbs); // gyors előremenetben kisebb túlfordulás
        const xShaped = clamp(xBase * steerMix, -1, 1);

        // Kis zajzóna: nagyon pici minták nullázása (simább IDLE tartás).
        const xOut = Math.abs(xShaped) < 0.015 ? 0 : xShaped;
        const yOut = Math.abs(yShaped) < 0.015 ? 0 : yShaped;
        return { x: xOut, y: yOut };
    }

    function vectorFromKeys() {
        const x = (state.keys.right ? 1 : 0) + (state.keys.left ? -1 : 0);
        const y = (state.keys.up ? 1 : 0) + (state.keys.down ? -1 : 0);
        return {
            x: x > 0 ? 1 : (x < 0 ? -1 : 0),
            y: y > 0 ? 1 : (y < 0 ? -1 : 0)
        };
    }

    function mapDirectionalKey(event) {
        const k = String(event.key || '').toLowerCase();
        if (k === 'arrowup') return 'up';
        if (k === 'arrowdown') return 'down';
        if (k === 'arrowleft') return 'left';
        if (k === 'arrowright') return 'right';
        return '';
    }

    function bindKeyboard() {
        document.addEventListener('keydown', function (e) {
            if (e.defaultPrevented) return;
            if (isEditableTarget(e.target)) return;
            if (String(e.key || '').toLowerCase() === 'r') {
                const nowMs = Date.now();
                if ((nowMs - num(state.resetHotkey.lastAtMs, 0)) >= num(state.resetHotkey.minIntervalMs, 700)) {
                    state.resetHotkey.lastAtMs = nowMs;
                    postRuntimeCommand({ type: 'reset_pos' });
                    setText(ui.simJoyState, 'POZÍCIÓ RESET (R)');
                }
                e.preventDefault();
                return;
            }
            const dir = mapDirectionalKey(e);
            if (dir) {
                // JOY érintés közben a fizikai/analóg irányítás élvez prioritást.
                if (state.joyPointerId !== null) return;
                state.keys[dir] = true;
                e.preventDefault();
                const v = vectorFromKeys();
                publishIntent(v.x, v.y, 'KEYBOARD');
                setText(ui.simInputMode, 'HYBRID (ARROW)');
            }
            if (e.code === 'Space') {
                e.preventDefault();
                state.keys = { up: false, left: false, down: false, right: false };
                stopIntent('KEYBOARD');
            }
        });

        document.addEventListener('keyup', function (e) {
            if (e.defaultPrevented) return;
            if (isEditableTarget(e.target)) return;
            const dir = mapDirectionalKey(e);
            if (dir) {
                state.keys[dir] = false;
                e.preventDefault();
                if (state.joyPointerId !== null) return;
                const v = vectorFromKeys();
                publishIntent(v.x, v.y, 'KEYBOARD');
            }
        });

        window.addEventListener('blur', function () {
            state.keys = { up: false, left: false, down: false, right: false };
            stopIntent('KEYBOARD');
        });
    }

    function isEditableTarget(target) {
        if (!target) return false;
        const tag = String(target.tagName || '').toLowerCase();
        if (tag === 'input' || tag === 'textarea' || tag === 'select') return true;
        if (target.isContentEditable) return true;
        return !!(target.closest && target.closest('[contenteditable="true"]'));
    }

    function bindJoystick() {
        if (!ui.joyPad) return;
        ui.joyPad.addEventListener('pointerdown', function (e) {
            if (e.cancelable) e.preventDefault();
            state.joyPointerId = e.pointerId;
            ui.joyPad.classList.add('active');
            ui.joyPad.setPointerCapture(e.pointerId);
            const analog = analogFromPoint(e.clientX, e.clientY);
            publishIntent(analog.x, analog.y, 'JOYSTICK');
            setText(ui.simInputMode, 'HYBRID (JOY)');
            setText(ui.simJoyState, 'ANALOG');
        });

        ui.joyPad.addEventListener('pointermove', function (e) {
            if (state.joyPointerId !== e.pointerId) return;
            const analog = analogFromPoint(e.clientX, e.clientY);
            publishIntent(analog.x, analog.y, 'JOYSTICK');
        });

        function releasePointer(e) {
            if (state.joyPointerId === null) return;
            if (e && e.pointerId != null && e.pointerId !== state.joyPointerId) return;
            state.joyPointerId = null;
            ui.joyPad.classList.remove('active');
            stopIntent('JOYSTICK');
            setText(ui.simJoyState, 'CENTER');
        }

        ui.joyPad.addEventListener('pointerup', releasePointer);
        ui.joyPad.addEventListener('pointercancel', releasePointer);
        ui.joyPad.addEventListener('lostpointercapture', releasePointer);
    }

    function bindSliders() {
        if (!ui.sliderX || !ui.sliderY) return;
        function updateFromSliders() {
            const x = clamp(num(ui.sliderX.value, 0) / 100, -1, 1);
            const y = clamp(num(ui.sliderY.value, 0) / 100, -1, 1);
            state.keys = { up: false, left: false, down: false, right: false };
            publishIntent(x, y, 'SLIDER');
            setText(ui.simJoyState, 'SLIDER');
        }
        ui.sliderX.addEventListener('input', updateFromSliders);
        ui.sliderY.addEventListener('input', updateFromSliders);
        ui.sliderX.addEventListener('change', updateFromSliders);
        ui.sliderY.addEventListener('change', updateFromSliders);
    }

    function bindHeaderEncoderToggle() {
        if (!ui.hdrEncoderToggle) return;
        ui.hdrEncoderToggle.addEventListener('click', function () {
            toggleHeaderEncoder();
        });
    }

    /* ---- Peripheral toggle buttons (Perifériák card) ---- */
    var PERIPH_MAP = [
        { key: 'lidar', btn: 'periphBtnLid', api: '/api/lidar-toggle', field: 'lidar' },
        { key: 'encoder', btn: 'periphBtnEnc', api: '/api/encoder-toggle', field: 'encoder' }
    ];

    var periphState = {
        lidar: false, encoder: false,
        busy: {}
    };

    function renderPeriphBtn(entry) {
        var el = ui[entry.btn];
        if (!el) return;
        var on = !!periphState[entry.key];
        el.classList.toggle('nominal', on);
        el.classList.toggle('warning', !on);
        el.setAttribute('aria-pressed', on ? 'true' : 'false');
        el.title = entry.key.toUpperCase() + (on ? ' BE' : ' KI');
        el.disabled = !!periphState.busy[entry.key];
    }

    function renderAllPeriphBtns() {
        for (var i = 0; i < PERIPH_MAP.length; i++) renderPeriphBtn(PERIPH_MAP[i]);
    }

    async function togglePeriphBtn(entry) {
        if (periphState.busy[entry.key]) return;
        var target = !periphState[entry.key];
        periphState.busy[entry.key] = true;
        renderPeriphBtn(entry);
        try {
            var response = await fetch(entry.api, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ enabled: target })
            });
            if (!response.ok) throw new Error('toggle-failed');
            var payload = await response.json().catch(function () { return {}; });
            var peripherals = (payload && payload.peripherals) || {};
            if (typeof payload.enabled === 'boolean') {
                periphState[entry.key] = payload.enabled;
            } else if (typeof peripherals[entry.field] === 'boolean') {
                periphState[entry.key] = peripherals[entry.field];
            } else {
                periphState[entry.key] = target;
            }
            // Sync header encoder button if encoder toggled from periph card
            if (entry.key === 'encoder') {
                renderHeaderEncoder(periphState.encoder);
            }
            // LIDAR overlay: stop polling when disabled, restart when enabled
            if (entry.key === 'lidar') {
                if (!periphState.lidar) {
                    // Stop overlay polling and clear canvas
                    if (state.lidarOverlay.timer) {
                        clearInterval(state.lidarOverlay.timer);
                        state.lidarOverlay.timer = null;
                    }
                    if (ui.cameraLidarMap) {
                        var ctx = ui.cameraLidarMap.getContext('2d');
                        if (ctx) ctx.clearRect(0, 0, ui.cameraLidarMap.width, ui.cameraLidarMap.height);
                    }
                } else {
                    // Re-start overlay polling
                    initLidarOverlay();
                }
            }
        } catch (_) {
            await refreshAllPeriphBtns();
        } finally {
            periphState.busy[entry.key] = false;
            renderPeriphBtn(entry);
        }
    }

    async function refreshAllPeriphBtns() {
        try {
            var response = await fetch('/api/status', { method: 'GET' });
            if (!response.ok) return;
            var payload = await response.json().catch(function () { return null; });
            var p = (payload && payload.peripherals) || {};
            for (var i = 0; i < PERIPH_MAP.length; i++) {
                var e = PERIPH_MAP[i];
                if (typeof p[e.field] === 'boolean') periphState[e.key] = p[e.field];
            }
        } catch (_) { }
        renderAllPeriphBtns();
    }

    function bindPeriphButtons() {
        for (var i = 0; i < PERIPH_MAP.length; i++) {
            (function (entry) {
                var el = ui[entry.btn];
                if (!el) return;
                el.addEventListener('click', function () {
                    togglePeriphBtn(entry);
                });
            })(PERIPH_MAP[i]);
        }
    }

    function setTurnLeft90Button(active, label) {
        if (!ui.joyTurnLeft90Btn) return;
        ui.joyTurnLeft90Btn.disabled = !!active;
        ui.joyTurnLeft90Btn.classList.toggle('running', !!active);
        setText(ui.joyTurnLeft90Btn, label || (active ? 'BALRA 90° FUT...' : 'BALRA 90° (HEADING)'));
    }

    async function postRuntimeCommand(payload) {
        try {
            const response = await fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload || {})
            });
            if (!response.ok) return { ok: false, status: response.status };
            const data = await response.json().catch(function () { return {}; });
            return { ok: !!data.ok, data: data };
        } catch (_) {
            return { ok: false };
        }
    }

    function clearAutoTurnTimer() {
        if (state.autoTurn.timer) {
            clearInterval(state.autoTurn.timer);
            state.autoTurn.timer = null;
        }
    }

    function autoTurnCommandPayload(left, right) {
        return {
            type: 'set_twist',
            v: 0,
            omega: Number((num(right, 0) - num(left, 0)).toFixed(3)),
            motion_source: 'GUI_JOYSTICK'
        };
    }

    function autoTurnDirectionSign() {
        return num(state.autoTurn.targetDeltaDeg, -90) < 0 ? -1 : 1;
    }

    function commandMagnitudeForRemainingDeg(remainingDeg) {
        const remain = Math.max(0, num(remainingDeg, 0));
        const fast = Math.max(0.05, num(state.autoTurn.commandFastAbs, 0.18));
        const slow = Math.max(0.05, num(state.autoTurn.commandSlowAbs, 0.13));
        const fine = Math.max(0.04, num(state.autoTurn.commandFineAbs, 0.09));
        const slowZone = Math.max(1, num(state.autoTurn.slowZoneDeg, 34));
        const fineZone = Math.max(1, Math.min(slowZone - 0.5, num(state.autoTurn.fineZoneDeg, 12)));

        if (remain <= fineZone) return fine;
        if (remain >= slowZone) return fast;
        const t = (remain - fineZone) / Math.max(0.001, slowZone - fineZone);
        return slow + (fast - slow) * clamp(t, 0, 1);
    }

    function profileAutoTurnCommand(diffDeg) {
        const targetDeg = num(state.autoTurn.targetDeltaDeg, -90);
        const currentDiffDeg = num(diffDeg, 0);
        const remainingDeg = Math.abs(targetDeg - currentDiffDeg);
        const magnitude = commandMagnitudeForRemainingDeg(remainingDeg);
        const sign = autoTurnDirectionSign();
        state.autoTurn.commandLeft = sign * magnitude;
        state.autoTurn.commandRight = -sign * magnitude;
        return {
            left: state.autoTurn.commandLeft,
            right: state.autoTurn.commandRight,
            magnitude: magnitude,
            remainingDeg: remainingDeg
        };
    }

    function sendAutoTurnKeepalive(diffDeg) {
        if (!state.autoTurn.active || state.autoTurn.stopping) return null;
        const command = profileAutoTurnCommand(diffDeg);
        const nowMs = Date.now();
        if ((nowMs - num(state.autoTurn.lastCommandSentAtMs, 0)) < num(state.autoTurn.commandRefreshMs, 240)) {
            return command;
        }
        state.autoTurn.lastCommandSentAtMs = nowMs;
        postRuntimeCommand(autoTurnCommandPayload(state.autoTurn.commandLeft, state.autoTurn.commandRight));
        return command;
    }

    async function stopAutoTurnMotion() {
        await postRuntimeCommand(autoTurnCommandPayload(0, 0));
        await postRuntimeCommand({ type: 'turn', direction: 0, motion_source: 'KEYBOARD' });
        await postRuntimeCommand({ type: 'set_speed', level: 0, motion_source: 'KEYBOARD' });
    }

    async function completeAutoTurn(label, flags) {
        if (!state.autoTurn.active || state.autoTurn.stopping) return;
        state.autoTurn.stopping = true;
        clearAutoTurnTimer();
        const result = buildAutoTurnResult(flags);
        await stopAutoTurnMotion();
        state.autoTurn.active = false;
        state.autoTurn.quality = null;
        state.autoTurn.lastResult = result;
        state.autoTurn.lockUntilMs = Date.now() + Math.max(150, num(state.autoTurn.debounceMs, 650));
        state.autoTurn.stopping = false;
        renderAutoTurnResult(result);
        try {
            console.info('[AUTO_TURN_L90]', result);
        } catch (_) { }
        const effectiveLabel = label || (result.pass ? 'BALRA 90° KÉSZ' : 'BALRA 90° WARN');
        setTurnLeft90Button(false, effectiveLabel);
        if (label) {
            setTimeout(function () {
                setTurnLeft90Button(false, 'BALRA 90° (HEADING)');
            }, 1000);
        }
    }

    async function startAutoTurnLeft90() {
        if (state.autoTurn.active || state.autoTurn.stopping) return;
        const nowMs = Date.now();
        if (nowMs < num(state.autoTurn.lockUntilMs, 0)) return;
        if (!isTelemetryFresh(state.autoTurn.minStartFreshMs)) {
            setTurnLeft90Button(false, 'BALRA 90° STALE');
            setAutoTurnQa('STALE', 'Nincs friss telemetria');
            setTimeout(function () {
                setTurnLeft90Button(false, 'BALRA 90° (HEADING)');
            }, 1000);
            return;
        }
        state.autoTurn.active = true;
        state.autoTurn.stopping = false;
        state.autoTurn.startHeadingDeg = num(state.currentHeadingDeg, 0);
        state.autoTurn.startPoseX = num(state.currentPoseX, 0);
        state.autoTurn.startPoseY = num(state.currentPoseY, 0);
        state.autoTurn.startedAtMs = nowMs;
        state.autoTurn.lastCommandSentAtMs = 0;
        state.autoTurn.quality = createAutoTurnQualityState();
        setText(ui.simJoyState, 'AUTO TURN L90');
        setTurnLeft90Button(true, 'BALRA 90° FUT...');
        setAutoTurnQa('RUN', 'Δ=0.0° | cél=-90°');
        stopIntent('AUTO_TURN');
        const startCmd = profileAutoTurnCommand(0);
        const turnStart = await postRuntimeCommand(autoTurnCommandPayload(startCmd.left, startCmd.right));
        state.autoTurn.lastCommandSentAtMs = Date.now();
        if (!turnStart.ok || !turnStart.data || !turnStart.data.ok) {
            state.autoTurn.active = false;
            state.autoTurn.quality = null;
            await stopAutoTurnMotion();
            setTurnLeft90Button(false, 'BALRA 90° HIBA');
            setAutoTurnQa('ERROR', 'Start parancs sikertelen');
            setTimeout(function () {
                setTurnLeft90Button(false, 'BALRA 90° (HEADING)');
            }, 1200);
            return;
        }

        clearAutoTurnTimer();
        state.autoTurn.timer = setInterval(function () {
            if (!state.autoTurn.active || state.autoTurn.stopping) return;
            if (!isTelemetryFresh(state.autoTurn.telemetryFreshMs)) {
                completeAutoTurn('BALRA 90° TELEMETRIA', { staleAbort: true });
                return;
            }
            const diff = angleDeltaDeg(state.autoTurn.startHeadingDeg, state.currentHeadingDeg);
            const cmd = sendAutoTurnKeepalive(diff) || { magnitude: 0 };
            sampleAutoTurnQuality();
            const elapsed = Date.now() - state.autoTurn.startedAtMs;
            setAutoTurnQa(
                'RUN',
                'Δ=' + diff.toFixed(1) + '° | cél=' + num(state.autoTurn.targetDeltaDeg, -90).toFixed(1) + '° | cmd=' + num(cmd.magnitude, 0).toFixed(2)
            );
            if (diff <= (state.autoTurn.targetDeltaDeg + state.autoTurn.stopToleranceDeg)) {
                completeAutoTurn('BALRA 90° KÉSZ');
                return;
            }
            if (elapsed >= state.autoTurn.timeoutMs) {
                completeAutoTurn('BALRA 90° TIMEOUT', { timedOut: true });
            }
        }, 40);
    }

    function bindQuickTurnButton() {
        if (!ui.joyTurnLeft90Btn) return;
        ui.joyTurnLeft90Btn.addEventListener('click', function () {
            startAutoTurnLeft90();
        });
    }

    async function sendTextCommand() {
        const input = ui.simCommandInput;
        if (!input) return;
        const command = String(input.value || '').trim();
        if (!command) return;
        try {
            const res = await fetch('/api/terminal-command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command: command })
            });
            if (res.ok) {
                input.value = '';
            }
        } catch (e) {
            // Nem blokkoljuk a fő vezérlést.
        }
    }

    function bindCommandInput() {
        if (ui.simCommandExec) {
            ui.simCommandExec.addEventListener('click', sendTextCommand);
        }
        if (ui.simCommandInput) {
            ui.simCommandInput.addEventListener('keydown', function (e) {
                if (e.key === 'Enter') {
                    e.preventDefault();
                    sendTextCommand();
                }
            });
        }
    }

    function bindRouteButtons() {
        var btnSquare = document.getElementById('btn-route-square');
        var btnCircle = document.getElementById('btn-route-circle');

        async function sendRoute(type, btn) {
            if (!btn || btn.disabled) return;
            btn.disabled = true;
            btn.classList.add('running');
            try {
                var res = await fetch('/api/command', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ type: type })
                });
                var data = await res.json();
                if (data && data.cmd_id) {
                    // Parancs kiadva, életciklus követés
                    var t0 = Date.now();
                    var poll = setInterval(async function () {
                        if (Date.now() - t0 > 30000) {
                            clearInterval(poll);
                            btn.disabled = false;
                            btn.classList.remove('running');
                            return;
                        }
                        try {
                            var sr = await fetch('/api/command-status/' + encodeURIComponent(data.cmd_id), { cache: 'no-store' });
                            var sd = await sr.json();
                            if (!sd.pending) {
                                clearInterval(poll);
                                btn.disabled = false;
                                btn.classList.remove('running');
                            }
                        } catch (e) { }
                    }, 500);
                } else {
                    btn.disabled = false;
                    btn.classList.remove('running');
                }
            } catch (e) {
                btn.disabled = false;
                btn.classList.remove('running');
            }
        }

        if (btnSquare) {
            btnSquare.addEventListener('click', function () { sendRoute('square', btnSquare); });
        }
        if (btnCircle) {
            btnCircle.addEventListener('click', function () { sendRoute('circle', btnCircle); });
        }
    }

    function bindStoreRender() {
        if (!window.R2B4_Store || !window.R2B4_Store.subscribe) return;
        window.R2B4_Store.subscribe(function (store) {
            const robotMotion = (store && store.robotMotion) || {};
            const readiness = (store && store.motionReadiness) || {};
            const pose = (store && store.pose) || {};
            const motion = (store && store.motion) || {};
            const safety = (store && store.safety) || {};
            const sensors = (store && store.sensors) || {};
            const peripherals = (sensors && sensors.peripherals) || {};
            state.currentHeadingDeg = num(pose.theta, state.currentHeadingDeg);
            state.currentPoseX = num(pose.x, state.currentPoseX);
            state.currentPoseY = num(pose.y, state.currentPoseY);
            state.latestRuntime.vTarget = num(motion.v_target, state.latestRuntime.vTarget);
            state.latestRuntime.pwmL = num(motion.pwm_l, state.latestRuntime.pwmL);
            state.latestRuntime.pwmR = num(motion.pwm_r, state.latestRuntime.pwmR);
            state.latestRuntime.vLRaw = num(motion.v_l, state.latestRuntime.vLRaw);
            state.latestRuntime.vRRaw = num(motion.v_r, state.latestRuntime.vRRaw);
            state.latestRuntime.watchdogHz = num(safety.watchdog_hz, state.latestRuntime.watchdogHz);
            renderVisionInline(pose, motion);
            if (typeof sensors.encoder_enabled === 'boolean') {
                renderHeaderEncoder(sensors.encoder_enabled);
            } else if (typeof peripherals.encoder === 'boolean') {
                renderHeaderEncoder(peripherals.encoder);
            }
            // Sync peripheral toggle buttons from SSE stream
            if (peripherals && typeof peripherals === 'object') {
                for (var pi = 0; pi < PERIPH_MAP.length; pi++) {
                    var pe = PERIPH_MAP[pi];
                    if (typeof peripherals[pe.field] === 'boolean') {
                        periphState[pe.key] = peripherals[pe.field];
                    }
                }
                renderAllPeriphBtns();
            }
            renderMotionTelemetry(robotMotion, readiness, safety);
        });
    }

    function boot() {
        if (!document.getElementById('page-motion-console')) return;
        bindHeaderEncoderToggle();
        bindPeriphButtons();
        bindQuickTurnButton();
        bindJoystick();
        bindKeyboard();
        bindSliders();
        bindCommandInput();
        bindRouteButtons();
        bindStoreRender();
        initLidarOverlay();
        connectMotionSSE();
        startConnectionWatchdog();
        refreshControlMode();
        refreshHeaderEncoder();
        refreshAllPeriphBtns();
        renderControlMode(state.controlMode);
        renderHeaderEncoder(state.encoderEnabled);
        setTurnLeft90Button(false, 'BALRA 90° (HEADING)');
        renderAutoTurnResult(null);
        setText(ui.simInputMode, 'HYBRID (JOY+KEY)');
        setText(ui.simJoyState, 'ACTIVE');
        stopIntent('INIT');
        window.addEventListener('online', function () {
            if (!state.sse) connectMotionSSE();
        });
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', boot);
    } else {
        boot();
    }

    window.addEventListener('beforeunload', function () {
        if (state.sse) {
            try { state.sse.close(); } catch (e) { }
            state.sse = null;
        }
        if (state.watchdogTimer) {
            clearInterval(state.watchdogTimer);
            state.watchdogTimer = null;
        }
        if (state.sseReconnectTimer) {
            clearTimeout(state.sseReconnectTimer);
            state.sseReconnectTimer = null;
        }
        clearAutoTurnTimer();
        state.autoTurn.active = false;
        if (state.lidarOverlay.timer) {
            clearInterval(state.lidarOverlay.timer);
            state.lidarOverlay.timer = null;
        }
    });
})();
