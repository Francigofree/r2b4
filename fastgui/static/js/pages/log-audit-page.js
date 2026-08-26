(function () {
    const API = {
        catalog: '/api/test/hub/catalog',
        toolsLiveCatalog: '/api/test/hub/tools-live/catalog',
        overview: '/api/test/hub/overview',
        job: '/api/test/hub/job',
        run: '/api/test/hub/run',
        runToolLive: '/api/test/hub/tools-live/run',
        runSequence: '/api/test/hub/run-sequence',
        archive: '/api/test/hub/archive-logs',
        report: '/api/test/hub/report',
        cameraToggle: '/api/camera-toggle',
    };

    const TOOLS_LIVE_PROFILE_FALLBACK = 'follow_forward_home_toggle_live';
    const HUMAN_FOLLOW_PROFILE_FALLBACK = 'person_target_direction_live';
    const HUMAN_FOLLOW_QUALITY_PROFILE_FALLBACK = 'M3_emberkovetes_mozgasminoseg';
    const M3_UNIFIED_PROFILE_FALLBACK = 'M3_room_cruise_unified_validator';
    const ROOM_CRUISE_QUALITY_PROFILE_FALLBACK = 'M4_1_room_cruise_quality_validator';
    const M0_MEASUREMENT_PROFILE = 'M0_measurement_trust_live';
    const M1_MOTION_PROFILE = 'M1_motion_baseline_live';

    const state = {
        catalog: [],
        toolsLiveCatalog: [],
        toolsLiveDefaultProfile: TOOLS_LIVE_PROFILE_FALLBACK,
        humanFollowProfile: HUMAN_FOLLOW_PROFILE_FALLBACK,
        humanFollowQualityProfile: HUMAN_FOLLOW_QUALITY_PROFILE_FALLBACK,
        m3UnifiedProfile: M3_UNIFIED_PROFILE_FALLBACK,
        roomCruiseQualityProfile: ROOM_CRUISE_QUALITY_PROFILE_FALLBACK,
        refreshTimer: null,
        refreshing: false,
    };

    function el(id) {
        return document.getElementById(id);
    }

    function escapeHtml(value) {
        return String(value ?? '')
            .replaceAll('&', '&amp;')
            .replaceAll('<', '&lt;')
            .replaceAll('>', '&gt;')
            .replaceAll('"', '&quot;')
            .replaceAll("'", '&#39;');
    }

    function safeText(value, fallback = '-') {
        const text = String(value ?? '').trim();
        return text || fallback;
    }

    function parseNumInput(id, fallback, min, max) {
        const raw = Number(el(id)?.value ?? fallback);
        if (!Number.isFinite(raw)) return fallback;
        return Math.max(min, Math.min(max, raw));
    }

    function setStatus(text) {
        const message = String(text || '');
        const node = el('hub-status-text');
        if (node) node.textContent = message;
        const motionNode = el('motion-live-test-status');
        if (motionNode) motionNode.textContent = message;
    }

    function setChipText(id, text) {
        const node = el(id);
        if (node) node.textContent = String(text || '');
    }

    function setLiveState(running) {
        const chip = el('hub-live-status-chip');
        if (!chip) return;
        chip.classList.toggle('running', !!running);
        chip.classList.toggle('idle', !running);
        chip.textContent = running ? 'Allapot: FUT' : 'Allapot: KESZENLET';
    }

    async function getJson(url) {
        const res = await fetch(url, { cache: 'no-store' });
        const data = await res.json().catch(() => ({}));
        if (!res.ok) {
            throw new Error(data.error || data.detail || ('HTTP ' + res.status));
        }
        return data;
    }

    async function postJson(url, body) {
        const res = await fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body || {}),
        });
        const data = await res.json().catch(() => ({}));
        if (!res.ok) {
            throw new Error(data.error || data.detail || ('HTTP ' + res.status));
        }
        return data;
    }

    function formatTs(value) {
        if (!value) return '-';
        if (typeof value === 'number' && Number.isFinite(value)) {
            return new Date(value * 1000).toLocaleString('hu-HU');
        }
        const date = new Date(value);
        if (!Number.isNaN(date.getTime())) {
            return date.toLocaleString('hu-HU');
        }
        return String(value);
    }

    function formatDuration(value) {
        const num = Number(value);
        if (!Number.isFinite(num)) return '-';
        return num.toFixed(2) + ' s';
    }

    function profileLabel(profile) {
        const name = safeText(profile?.name, '?');
        const family = safeText(profile?.family, '?');
        return `${name} (${family})`;
    }

    function renderCatalog(catalogPayload) {
        const profiles = Array.isArray(catalogPayload?.profiles) ? catalogPayload.profiles : [];
        state.catalog = profiles;
        const select = el('hub-profile-select');
        if (!select) return;
        const previous = select.value;
        select.innerHTML = '';
        profiles.forEach((profile) => {
            const option = document.createElement('option');
            option.value = String(profile.name || '');
            option.textContent = profileLabel(profile);
            select.appendChild(option);
        });
        const motionM0Button = el('motion-btn-run-m0-live');
        if (motionM0Button) {
            motionM0Button.disabled = !profiles.some(
                (item) => String(item?.name || '') === M0_MEASUREMENT_PROFILE
            );
        }
        if (previous && profiles.some((item) => String(item.name) === previous)) {
            select.value = previous;
        }
    }

    function toolLiveLabel(profile) {
        const name = safeText(profile?.name, '?');
        const family = safeText(profile?.family, '?');
        const script = safeText(profile?.script, '?');
        return `${name} (${family}) - ${script}`;
    }

    function renderToolsLiveCatalog(payload) {
        const profiles = Array.isArray(payload?.profiles) ? payload.profiles : [];
        const defaultProfile = String(payload?.default_profile || TOOLS_LIVE_PROFILE_FALLBACK);
        const humanFollowProfile = String(payload?.human_follow_profile || HUMAN_FOLLOW_PROFILE_FALLBACK);
        const humanFollowQualityProfile = String(
            payload?.human_follow_quality_profile || HUMAN_FOLLOW_QUALITY_PROFILE_FALLBACK
        );
        const m3UnifiedProfile = String(payload?.m3_unified_profile || M3_UNIFIED_PROFILE_FALLBACK);
        const roomCruiseQualityProfile = String(
            payload?.room_cruise_quality_profile || ROOM_CRUISE_QUALITY_PROFILE_FALLBACK
        );
        state.toolsLiveDefaultProfile = profiles.some((item) => String(item.name || '') === defaultProfile)
            ? defaultProfile
            : '';
        state.humanFollowProfile = profiles.some((item) => String(item.name || '') === humanFollowProfile)
            ? humanFollowProfile
            : '';
        state.humanFollowQualityProfile = profiles.some(
            (item) => String(item.name || '') === humanFollowQualityProfile
        ) ? humanFollowQualityProfile : '';
        state.m3UnifiedProfile = profiles.some((item) => String(item.name || '') === m3UnifiedProfile)
            ? m3UnifiedProfile
            : '';
        state.roomCruiseQualityProfile = profiles.some(
            (item) => String(item.name || '') === roomCruiseQualityProfile
        ) ? roomCruiseQualityProfile : '';
        state.toolsLiveCatalog = profiles;
        const motionM3Button = el('motion-btn-run-m3-unified-live');
        if (motionM3Button) motionM3Button.disabled = !state.m3UnifiedProfile;

        const select = el('hub-tools-live-select');
        if (select) {
            const previous = select.value;
            select.innerHTML = '';
            profiles.forEach((profile) => {
                const option = document.createElement('option');
                option.value = String(profile.name || '');
                option.textContent = toolLiveLabel(profile);
                select.appendChild(option);
            });
            if (previous && profiles.some((item) => String(item.name) === previous)) {
                select.value = previous;
            } else if (state.toolsLiveDefaultProfile) {
                select.value = state.toolsLiveDefaultProfile;
            }
        }

        const list = el('hub-tools-live-list');
        if (!list) return;
        if (profiles.length === 0) {
            list.innerHTML = '<div class="hub-empty">Nincs kompatibilis /tools live teszt.</div>';
            return;
        }
        list.innerHTML = profiles.map((item) => {
            const isDefault = state.toolsLiveDefaultProfile && String(item.name || '') === state.toolsLiveDefaultProfile;
            const isHumanFollow = state.humanFollowProfile && String(item.name || '') === state.humanFollowProfile;
            const isHumanFollowQuality = state.humanFollowQualityProfile
                && String(item.name || '') === state.humanFollowQualityProfile;
            const isM3Unified = state.m3UnifiedProfile
                && String(item.name || '') === state.m3UnifiedProfile;
            const isRoomCruiseQuality = state.roomCruiseQualityProfile
                && String(item.name || '') === state.roomCruiseQualityProfile;
            return (
            '<div class="hub-list-row">' +
            '<div class="hub-list-main"><strong>' + escapeHtml(safeText(item.name)) + '</strong> ' +
            '<span class="hub-list-status">[' + escapeHtml(safeText(item.family)) + ']</span> ' +
            (isDefault ? '<span class="hub-list-status">[default]</span> ' : '') +
            (isHumanFollow ? '<span class="hub-list-status">[emberkovetes]</span>' : '') +
            (isHumanFollowQuality ? '<span class="hub-list-status">[M3 mozgasminoseg]</span>' : '') +
            (isM3Unified ? '<span class="hub-list-status">[M3 unified]</span>' : '') +
            (isRoomCruiseQuality ? '<span class="hub-list-status">[M3 room cruise]</span>' : '') +
            '</div>' +
            '<div class="hub-list-sub">script=' + escapeHtml(safeText(item.script)) + '</div>' +
            '<div class="hub-list-sub">' + escapeHtml(safeText(item.description, '')) + '</div>' +
            '</div>'
            );
        }).join('');
    }

    function renderMeta(containerId, rows) {
        const box = el(containerId);
        if (!box) return;
        const html = rows.map((row) => (
            '<div class="hub-meta-item">' +
            '<span class="hub-meta-key">' + escapeHtml(row.label) + '</span>' +
            '<span class="hub-meta-val">' + escapeHtml(row.value) + '</span>' +
            '</div>'
        )).join('');
        box.innerHTML = html || '<div class="hub-empty">Nincs adat.</div>';
    }

    function renderLatest(latest) {
        const compact = (latest && typeof latest.compact === 'object') ? latest.compact : {};
        const summaryNode = el('hub-latest-summary');
        if (summaryNode) {
            summaryNode.textContent = safeText(compact.summary_hu, 'Nincs elerheto hub eredmeny.');
            summaryNode.classList.toggle('fail', safeText(compact.status, '') === 'FAIL');
            summaryNode.classList.toggle('pass', safeText(compact.status, '') === 'PASS');
        }
        renderMeta('hub-latest-meta', [
            { label: 'Status', value: safeText(compact.status) },
            { label: 'Profil', value: safeText(compact.profile) },
            { label: 'Primary', value: safeText(compact.primary) },
            { label: 'Reason', value: safeText(compact.reason) },
            { label: 'Idotartam', value: formatDuration(compact.duration_s) },
            { label: 'Preflight', value: latest?.summary?.preflight_ok === false ? 'FAIL' : 'OK' },
            { label: 'Measurement gate', value: compact.measurement_truth_gate_ok ? 'OK' : 'FAIL' },
            { label: 'EKF gate', value: compact.ekf_truth_gate_ok ? 'OK' : 'FAIL' },
        ]);

        const incidentBox = el('hub-incident-view');
        if (incidentBox) {
            const incident = (latest && typeof latest.incident === 'object') ? latest.incident : {};
            const compactIncident = {
                status: safeText(incident.status, ''),
                reason: safeText(incident.reason, ''),
                primary_failure: incident.primary_failure || {},
                preflight: incident.preflight || {},
                command: incident.command || {},
            };
            incidentBox.textContent = JSON.stringify(compactIncident, null, 2);
        }
    }

    function renderSequence(latest) {
        const sequenceSummary = (latest && typeof latest.sequence_summary === 'object') ? latest.sequence_summary : {};
        const verdict = (sequenceSummary && typeof sequenceSummary.verdict === 'object') ? sequenceSummary.verdict : {};
        const summaryNode = el('hub-sequence-summary');
        if (summaryNode) {
            if (Object.keys(sequenceSummary).length === 0) {
                summaryNode.textContent = 'Nincs elerheto szekvencia eredmeny.';
            } else if (String(sequenceSummary.status || '').toUpperCase() === 'PASS') {
                summaryNode.textContent = 'A legutobbi szekvencia PASS lett.';
            } else {
                summaryNode.textContent = 'A legutobbi szekvencia FAIL lett.';
            }
        }
        renderMeta('hub-sequence-meta', [
            { label: 'Sequence', value: safeText(sequenceSummary.sequence) },
            { label: 'Status', value: safeText(sequenceSummary.status) },
            { label: 'Primary', value: safeText(verdict.primary) },
            { label: 'Reason', value: safeText(verdict.reason) },
            { label: 'Lepesek', value: safeText(sequenceSummary.step_count_executed, '-') + '/' + safeText(sequenceSummary.step_count_requested, '-') },
            { label: 'Idotartam', value: formatDuration(sequenceSummary.duration_s) },
        ]);
    }

    function runRowHtml(item) {
        const status = safeText(item.status).toUpperCase();
        const tone = status === 'PASS' ? 'pass' : (status === 'FAIL' ? 'fail' : '');
        return (
            '<div class="hub-list-row ' + tone + '">' +
            '<div class="hub-list-main">' +
            '<strong>' + escapeHtml(safeText(item.profile || item.sequence, '?')) + '</strong> ' +
            '<span class="hub-list-status">[' + escapeHtml(status || '?') + ']</span>' +
            '</div>' +
            '<div class="hub-list-sub">' + escapeHtml(safeText(item.summary_hu, '')) + '</div>' +
            '<div class="hub-list-sub">' +
            'primary=' + escapeHtml(safeText(item.primary)) +
            ' | duration=' + escapeHtml(formatDuration(item.duration_s)) +
            ' | start=' + escapeHtml(formatTs(item.started_at_utc)) +
            '</div>' +
            '</div>'
        );
    }

    function renderRuns(runs) {
        const box = el('hub-runs-list');
        if (!box) return;
        if (!Array.isArray(runs) || runs.length === 0) {
            box.innerHTML = '<div class="hub-empty">Nincs futasi elozmeny.</div>';
            return;
        }
        box.innerHTML = runs.map(runRowHtml).join('');
    }

    function sessionRowHtml(item) {
        return (
            '<div class="hub-list-row">' +
            '<div class="hub-list-main"><strong>' + escapeHtml(safeText(item.folder, '?')) + '</strong></div>' +
            '<div class="hub-list-sub">' + escapeHtml(safeText(item.status_hu, 'Nincs status.')) + '</div>' +
            '<div class="hub-list-sub">' +
            'duration=' + escapeHtml(formatDuration(item.duration_s)) +
            ' | drop=' + escapeHtml(safeText(item.dropped_messages, '0')) +
            ' | write=' + escapeHtml(safeText(item.write_errors, '0')) +
            ' | start=' + escapeHtml(formatTs(item.start_wall)) +
            '</div>' +
            '</div>'
        );
    }

    function renderSessions(sessions) {
        const box = el('hub-sessions-list');
        if (!box) return;
        if (!Array.isArray(sessions) || sessions.length === 0) {
            box.innerHTML = '<div class="hub-empty">Nincs session adat.</div>';
            return;
        }
        box.innerHTML = sessions.map(sessionRowHtml).join('');
    }

    function renderJob(job) {
        const summary = el('hub-job-summary');
        const detail = el('hub-job-detail');
        const running = !!(job && job.running);
        setLiveState(running);
        if (summary) {
            if (running) {
                summary.textContent = `Futas alatt: ${safeText(job.operation, 'ismeretlen')} (${formatDuration(job.elapsed_s)})`;
            } else if (job && job.return_code !== null && job.return_code !== undefined) {
                const ok = !!job.ok;
                summary.textContent = ok
                    ? `Utolso muvelet sikeres (${safeText(job.operation, 'ismeretlen')}).`
                    : `Utolso muvelet sikertelen (${safeText(job.operation, 'ismeretlen')}).`;
            } else {
                summary.textContent = 'Nincs aktiv futas.';
            }
        }
        if (detail) {
            const payload = (job && typeof job.payload === 'object') ? job.payload : {};
            detail.textContent = JSON.stringify({
                operation: safeText(job?.operation, ''),
                running: running,
                ok: !!job?.ok,
                return_code: job?.return_code,
                error: safeText(job?.error, ''),
                elapsed_s: job?.elapsed_s,
                payload: payload,
            }, null, 2);
        }
    }

    function collectRunOptions() {
        return {
            stop_runtime_after: !!el('hub-opt-stop-runtime')?.checked,
        };
    }

    async function loadCatalog() {
        const response = await getJson(API.catalog);
        renderCatalog(response.catalog || {});
    }

    async function loadToolsLiveCatalog() {
        const response = await getJson(API.toolsLiveCatalog);
        renderToolsLiveCatalog(response || {});
    }

    async function refreshOverview() {
        if (state.refreshing) return;
        state.refreshing = true;
        try {
            const sessionLimit = parseNumInput('hub-session-limit', 8, 1, 30);
            const runLimit = parseNumInput('hub-run-limit', 10, 1, 40);
            const qs = new URLSearchParams({
                session_limit: String(sessionLimit),
                run_limit: String(runLimit),
            });
            const payload = await getJson(API.overview + '?' + qs.toString());
            renderLatest(payload.latest || {});
            renderSequence(payload.latest || {});
            renderRuns(payload.recent_runs || []);
            renderSessions(payload.sessions || []);
            renderJob(payload.job || {});
            setChipText('hub-last-update-chip', 'Frissites: ' + formatTs(payload.timestamp));
            setStatus('A teszt oldal frissitve.');
        } catch (err) {
            setStatus('Frissitesi hiba: ' + err.message);
        } finally {
            state.refreshing = false;
        }
    }

    async function requestReport() {
        try {
            await postJson(API.report, {});
            await refreshOverview();
            setStatus('Jelentes frissitve.');
        } catch (err) {
            setStatus('Jelentes hiba: ' + err.message);
        }
    }

    async function runProfile() {
        const select = el('hub-profile-select');
        const profile = String(select?.value || '').trim();
        if (!profile) {
            setStatus('Valassz profilt.');
            return;
        }
        try {
            const payload = await postJson(API.run, {
                profile: profile,
                ...collectRunOptions(),
            });
            renderJob(payload.job || {});
            setStatus('Profil futas elinditva: ' + profile);
            await refreshOverview();
        } catch (err) {
            setStatus('Profil futtatas hiba: ' + err.message);
        }
    }

    async function runCanonicalMotionLevel(profile) {
        const canonicalProfile = String(profile || '').trim();
        const exists = state.catalog.some((item) => String(item?.name || '') === canonicalProfile);
        if (!exists) {
            setStatus('A canonical Hub profil nem elerheto: ' + canonicalProfile);
            return;
        }
        const select = el('hub-profile-select');
        if (select) select.value = canonicalProfile;
        await runProfile();
    }

    async function runSequence() {
        const sequence = String(el('hub-sequence-select')?.value || 'motion_levels_M0_M4_1');
        try {
            const payload = await postJson(API.runSequence, {
                sequence: sequence,
                ...collectRunOptions(),
            });
            renderJob(payload.job || {});
            setStatus('Szekvencia futas elinditva: ' + sequence);
            await refreshOverview();
        } catch (err) {
            setStatus('Szekvencia futtatas hiba: ' + err.message);
        }
    }

    function refreshCameraStreamView() {
        const feed = el('ts-camera-feed-tac');
        if (feed) {
            feed.src = '/api/camera-stream?ts=' + Date.now();
            feed.style.opacity = '1';
            feed.style.background = 'transparent';
        }
        const wrap = el('camera-feed-container');
        if (wrap) wrap.classList.remove('stream-off');
        const badge = el('camera-toggle-badge');
        if (badge) {
            badge.textContent = 'STREAM KI';
            badge.style.color = '#9affc1';
            badge.style.borderColor = 'rgba(105, 255, 170, 0.55)';
        }
        const header = el('hdr-camera-toggle');
        if (header) header.textContent = 'Kamera BE';
    }

    async function ensureCameraStreamVisible() {
        try {
            await postJson(API.cameraToggle, { enabled: true });
        } catch (err) {
            setStatus('Kamera stream inditas hiba: ' + err.message);
        }
        refreshCameraStreamView();
    }

    async function ensureCameraStreamHidden() {
        try {
            await postJson(API.cameraToggle, { enabled: false });
        } catch (err) {
            setStatus('Kamera stream kikapcsolas hiba: ' + err.message);
        }
        refreshCameraStreamView();
    }

    function selectHumanFollowToolLive() {
        const profile = state.humanFollowProfile || HUMAN_FOLLOW_PROFILE_FALLBACK;
        const select = el('hub-tools-live-select');
        if (!select || !profile) return false;
        const exists = Array.from(select.options || []).some((option) => String(option.value || '') === profile);
        if (!exists) return false;
        select.value = profile;
        return true;
    }

    function selectHumanFollowQualityToolLive() {
        const profile = state.humanFollowQualityProfile || HUMAN_FOLLOW_QUALITY_PROFILE_FALLBACK;
        const select = el('hub-tools-live-select');
        if (!select || !profile) return false;
        const exists = Array.from(select.options || []).some((option) => String(option.value || '') === profile);
        if (!exists) return false;
        select.value = profile;
        return true;
    }

    function selectRoomCruiseQualityToolLive() {
        const profile = state.roomCruiseQualityProfile || ROOM_CRUISE_QUALITY_PROFILE_FALLBACK;
        const select = el('hub-tools-live-select');
        if (!select || !profile) return false;
        const exists = Array.from(select.options || []).some((option) => String(option.value || '') === profile);
        if (!exists) return false;
        select.value = profile;
        return true;
    }

    function selectM3UnifiedToolLive() {
        const profile = state.m3UnifiedProfile || M3_UNIFIED_PROFILE_FALLBACK;
        const select = el('hub-tools-live-select');
        if (!select || !profile) return false;
        const exists = Array.from(select.options || []).some((option) => String(option.value || '') === profile);
        if (!exists) return false;
        select.value = profile;
        return true;
    }

    async function runToolLive(options) {
        const opts = options || {};
        const select = el('hub-tools-live-select');
        const profile = String(select?.value || '').trim();
        if (!profile) {
            setStatus('Valassz kompatibilis /tools live tesztet.');
            return;
        }
        const isHumanFollow = profile === (state.humanFollowProfile || HUMAN_FOLLOW_PROFILE_FALLBACK);
        const isHumanFollowQuality = profile === (
            state.humanFollowQualityProfile || HUMAN_FOLLOW_QUALITY_PROFILE_FALLBACK
        );
        const isDefaultFollowTool = profile === (state.toolsLiveDefaultProfile || TOOLS_LIVE_PROFILE_FALLBACK);
        const isRoomCruiseQuality = profile === (
            state.roomCruiseQualityProfile || ROOM_CRUISE_QUALITY_PROFILE_FALLBACK
        );
        const isM3Unified = profile === (state.m3UnifiedProfile || M3_UNIFIED_PROFILE_FALLBACK);
        try {
            if (opts.ensureCameraStream || isHumanFollow || isHumanFollowQuality || isDefaultFollowTool) {
                await ensureCameraStreamVisible();
            }
            if (opts.ensureCameraOff || isRoomCruiseQuality || isM3Unified) {
                await ensureCameraStreamHidden();
            }
            const payload = await postJson(API.runToolLive, {
                profile: profile,
                ...collectRunOptions(),
            });
            renderJob(payload.job || {});
            const profileSelect = el('hub-profile-select');
            if (profileSelect) profileSelect.value = profile;
            setStatus('Tools live teszt inditva: ' + profile);
            await refreshOverview();
        } catch (err) {
            setStatus('Tools live futtatas hiba: ' + err.message);
        }
    }

    async function runHumanFollowToolLive() {
        if (!selectHumanFollowToolLive()) {
            setStatus('Az emberkovetes live profil nem elerheto a Tools live katalogusban.');
            return;
        }
        await runToolLive({ ensureCameraStream: true });
    }

    async function runHumanFollowQualityToolLive() {
        if (!selectHumanFollowQualityToolLive()) {
            setStatus('Az M3 emberkovetesi mozgasminoseg profil nem elerheto a Tools live katalogusban.');
            return;
        }
        await runToolLive({ ensureCameraStream: true });
    }

    async function runRoomCruiseQualityToolLive() {
        if (!selectRoomCruiseQualityToolLive()) {
            setStatus('Az M4.1 Room Cruise mozgasminoseg profil nem elerheto a Tools live katalogusban.');
            return;
        }
        await runToolLive({ ensureCameraOff: true });
    }

    async function runM3UnifiedToolLive() {
        if (!selectM3UnifiedToolLive()) {
            setStatus('Az M3 unified live profil nem elerheto a Tools live katalogusban.');
            return;
        }
        await runToolLive({ ensureCameraOff: true });
    }

    async function archiveLogs() {
        try {
            const payload = await postJson(API.archive, {
                max_file_mb: 12,
                keep_latest_sessions: 20,
            });
            renderJob(payload.job || {});
            setStatus('Naplo karbantartas elinditva.');
            await refreshOverview();
        } catch (err) {
            setStatus('Archivasi hiba: ' + err.message);
        }
    }

    function setAutoRefresh(enabled) {
        if (state.refreshTimer) {
            clearInterval(state.refreshTimer);
            state.refreshTimer = null;
        }
        if (enabled) {
            state.refreshTimer = setInterval(() => {
                refreshOverview();
            }, 2500);
        }
    }

    function bind() {
        el('hub-btn-refresh')?.addEventListener('click', refreshOverview);
        el('hub-btn-report')?.addEventListener('click', requestReport);
        el('hub-btn-run-profile')?.addEventListener('click', runProfile);
        el('hub-btn-run-m0')?.addEventListener('click', () => runCanonicalMotionLevel(M0_MEASUREMENT_PROFILE));
        el('hub-btn-run-m1')?.addEventListener('click', () => runCanonicalMotionLevel(M1_MOTION_PROFILE));
        el('motion-btn-run-m0-live')?.addEventListener('click', () => runCanonicalMotionLevel(M0_MEASUREMENT_PROFILE));
        el('motion-btn-run-m3-unified-live')?.addEventListener('click', runM3UnifiedToolLive);
        el('hub-btn-run-tool-live')?.addEventListener('click', runToolLive);
        el('hub-btn-run-human-follow-live')?.addEventListener('click', runHumanFollowToolLive);
        el('hub-btn-run-human-follow-quality-live')?.addEventListener('click', runHumanFollowQualityToolLive);
        el('hub-btn-run-room-cruise-quality-live')?.addEventListener('click', runRoomCruiseQualityToolLive);
        el('hub-btn-run-sequence')?.addEventListener('click', runSequence);
        el('hub-btn-archive')?.addEventListener('click', archiveLogs);
        el('hub-opt-auto-refresh')?.addEventListener('change', () => {
            setAutoRefresh(!!el('hub-opt-auto-refresh')?.checked);
        });
    }

    async function init() {
        bind();
        setLiveState(false);
        setStatus('Teszt oldal inicializalas...');
        try {
            await loadCatalog();
            await loadToolsLiveCatalog();
            await refreshOverview();
            setAutoRefresh(!!el('hub-opt-auto-refresh')?.checked);
            setStatus('Keszenlet.');
        } catch (err) {
            setStatus('Inicializalasi hiba: ' + err.message);
        }
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
