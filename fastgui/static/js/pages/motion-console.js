/**
 * Motion Console page extensions:
 * - Shadow EKF tuning panel
 * - Live/Shadow diagnostics
 * Safety:
 * - Apply-to-live disabled until validation_status === "stable"
 */
class MotionConsolePage {
    constructor() {
        this.pageId = 'page-motion-console';
        this.active = false;
        this.shadowDraft = {
            Q_yaw: null,
            Q_velocity: null,
            R_gyro: null,
            R_encoder: null,
            ZUPT_threshold: null
        };
        this.shadowServer = { ...this.shadowDraft };
        this.shadowDirty = {};
        this.init();
    }

    init() {
        this.ensureUi();
        this.setupStateBinding();
        this.bindActions();
        const page = document.getElementById(this.pageId);
        if (page && page.classList.contains('active')) this.active = true;
    }

    ensureUi() {
        const page = document.getElementById(this.pageId);
        if (!page || document.getElementById('ekf-shadow-panel')) return;

        const panel = document.createElement('article');
        panel.id = 'ekf-shadow-panel';
        panel.className = 'r2-panel';
        panel.style.marginTop = '12px';
        panel.innerHTML = `
            <details open>
                <summary style="cursor:pointer; font-weight:700;">EKF SHADOW TUNING</summary>
                <div style="display:grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 10px; margin-top: 10px;">
                    ${this.paramRow('Q_yaw', 'Q_yaw')}
                    ${this.paramRow('Q_velocity', 'Q_velocity')}
                    ${this.paramRow('R_gyro', 'R_gyro')}
                    ${this.paramRow('R_encoder', 'R_encoder')}
                    ${this.paramRow('ZUPT_threshold', 'ZUPT_threshold')}
                </div>
                <div style="display:flex; justify-content:flex-end; margin-top: 10px;">
                    <button id="ekf-apply-live-btn" class="r2-button" disabled>Apply Shadow to Live</button>
                </div>
                <hr style="margin: 12px 0; border-color: rgba(255,255,255,0.08);" />
                <div style="display:grid; grid-template-columns: repeat(auto-fit, minmax(190px, 1fr)); gap: 8px;">
                    <div>Live yaw: <span id="ekf-live-yaw">0.00</span></div>
                    <div>Shadow yaw: <span id="ekf-shadow-yaw">0.00</span></div>
                    <div>Yaw diff: <span id="ekf-yaw-diff">0.00</span></div>
                    <div>Innovation L/S: <span id="ekf-inno-live">0.00,0.00</span> / <span id="ekf-inno-shadow">0.00,0.00</span></div>
                    <div>Cov L/S: <span id="ekf-cov-live">0.00</span> / <span id="ekf-cov-shadow">0.00</span></div>
                    <div>Validation: <span id="ekf-validation" style="font-weight:700;">initializing</span></div>
                </div>
            </details>
        `;
        page.appendChild(panel);
    }

    paramRow(id, label) {
        return `
            <div style="border:1px solid rgba(255,255,255,0.08); border-radius: 6px; padding: 8px;">
                <div style="display:flex; justify-content:space-between;">
                    <strong>${label}</strong>
                    <span id="dirty-${id}" style="color:#fbbf24; visibility:hidden;">DIRTY</span>
                </div>
                <input id="inp-${id}" type="number" step="0.0001" style="width:100%; margin-top:6px;" />
                <div style="display:flex; justify-content:flex-end; margin-top:6px;">
                    <button id="btn-${id}" class="r2-button">Apply</button>
                </div>
            </div>
        `;
    }

    setupStateBinding() {
        window.R2B4_Store.subscribe((state) => {
            if (!this.active) return;
            this.updateDiagnostics(state);
            this.syncParamsFromState(state);
        });
    }

    syncParamsFromState(state) {
        const sp = (state.ekf && state.ekf.shadow_params) || {};
        const values = {
            Q_yaw: this.safeNum(sp.Q_yaw),
            Q_velocity: this.safeNum(sp.Q_velocity),
            R_gyro: this.safeNum(sp.R_gyro),
            R_encoder: this.safeNum(sp.R_encoder),
            ZUPT_threshold: this.safeNum(sp.ZUPT_threshold)
        };
        Object.keys(values).forEach((k) => {
            if (this.shadowServer[k] === null || this.shadowServer[k] === undefined) {
                this.shadowServer[k] = values[k];
            }
            if (this.shadowDraft[k] === null || this.shadowDraft[k] === undefined) {
                this.shadowDraft[k] = this.shadowServer[k];
                const inp = document.getElementById(`inp-${k}`);
                if (inp) inp.value = String(this.shadowDraft[k]);
            }
        });
    }

    updateDiagnostics(state) {
        const ekf = state.ekf || {};
        const live = ekf.live || {};
        const shadow = ekf.shadow || {};
        this.setText('ekf-live-yaw', this.safeNum(live.yaw).toFixed(2));
        this.setText('ekf-shadow-yaw', this.safeNum(shadow.yaw).toFixed(2));
        this.setText('ekf-yaw-diff', this.safeNum(ekf.yaw_diff).toFixed(2));
        this.setText('ekf-inno-live', this.formatPair(live.innovation));
        this.setText('ekf-inno-shadow', this.formatPair(shadow.innovation));
        this.setText('ekf-cov-live', this.sumArray(live.cov_diag).toFixed(4));
        this.setText('ekf-cov-shadow', this.sumArray(shadow.cov_diag).toFixed(4));

        const status = String(ekf.validation_status || 'initializing').toLowerCase();
        const validation = document.getElementById('ekf-validation');
        if (validation) {
            validation.textContent = status;
            validation.style.color = status === 'stable' ? '#22c55e' : '#ef4444';
        }
        const applyBtn = document.getElementById('ekf-apply-live-btn');
        if (applyBtn) applyBtn.disabled = status !== 'stable';
    }

    bindActions() {
        Object.keys(this.shadowDraft).forEach((k) => {
            const input = document.getElementById(`inp-${k}`);
            const button = document.getElementById(`btn-${k}`);
            if (input) {
                input.addEventListener('input', () => {
                    const next = this.safeNum(parseFloat(input.value));
                    this.shadowDraft[k] = next;
                    this.shadowDirty[k] = this.shadowServer[k] !== next;
                    this.renderDirty(k);
                });
            }
            if (button) {
                button.addEventListener('click', async () => {
                    await this.applyShadowParam(k);
                });
            }
        });

        const applyLiveBtn = document.getElementById('ekf-apply-live-btn');
        if (applyLiveBtn) {
            applyLiveBtn.addEventListener('click', async () => {
                await this.applyShadowToLive();
            });
        }
    }

    async applyShadowParam(key) {
        if (!window.R2B4_API || !window.R2B4_API.updateEkfShadow) return;
        const value = this.safeNum(this.shadowDraft[key]);
        const res = await window.R2B4_API.updateEkfShadow({ [key]: value });
        const json = await res.json().catch(() => ({}));
        if (res.ok && json.status !== 'failed') {
            this.shadowServer[key] = value;
            this.shadowDirty[key] = false;
            this.renderDirty(key);
        }
    }

    async applyShadowToLive() {
        if (!window.R2B4_API || !window.R2B4_API.applyEkfShadowToLive) return;
        const res = await window.R2B4_API.applyEkfShadowToLive();
        const json = await res.json().catch(() => ({}));
        if (!res.ok && json && json.error_code) {
            console.warn('[EKF] Shadow apply failed:', json.error_code);
        }
    }

    renderDirty(key) {
        const el = document.getElementById(`dirty-${key}`);
        if (!el) return;
        el.style.visibility = this.shadowDirty[key] ? 'visible' : 'hidden';
    }

    activate() {
        this.active = true;
        const page = document.getElementById(this.pageId);
        if (page) page.classList.add('active');
    }

    deactivate() {
        this.active = false;
        const page = document.getElementById(this.pageId);
        if (page) page.classList.remove('active');
    }

    safeNum(v) {
        const n = Number(v);
        return Number.isFinite(n) ? n : 0;
    }

    sumArray(arr) {
        if (!Array.isArray(arr)) return 0;
        return arr.reduce((s, x) => s + this.safeNum(x), 0);
    }

    formatPair(arr) {
        if (!Array.isArray(arr)) return '0.00,0.00';
        return `${this.safeNum(arr[0]).toFixed(2)},${this.safeNum(arr[1]).toFixed(2)}`;
    }

    setText(id, value) {
        const el = document.getElementById(id);
        if (el && el.textContent !== value) el.textContent = value;
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.MotionConsole = new MotionConsolePage();
});
