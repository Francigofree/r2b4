(function () {
    function clamp(value, minValue, maxValue) {
        return Math.max(minValue, Math.min(maxValue, value));
    }

    function toNumber(value, fallback) {
        var n = Number(value);
        return Number.isFinite(n) ? n : fallback;
    }

    function sameIntent(a, b, epsilon) {
        return Math.abs(a.x - b.x) < epsilon && Math.abs(a.y - b.y) < epsilon;
    }

    function MotionController() {
        this.intent = { x: 0, y: 0 };
        this.lastSentIntent = { x: 999, y: 999 };
        this.lastSentAt = 0;
        this.sendInFlight = false;
        this.pendingTimer = null;
        // Analóg vezérléshez sűrűbb, finomabb intent stream.
        this.rateLimitMs = 35;
        this.epsilon = 0.004;
        this.defaultSource = 'GUI_JOYSTICK';
        this.intentSeq = 0;
        this.requestTimeoutMs = 900;
        this.maxRetries = 1;
    }

    MotionController.prototype.getIntent = function () {
        return { x: this.intent.x, y: this.intent.y };
    };

    MotionController.prototype.stop = function (source) {
        return this.setIntent(0, 0, source || this.defaultSource);
    };

    MotionController.prototype.setIntent = function (x, y, source) {
        var next = {
            x: clamp(toNumber(x, 0), -1, 1),
            y: clamp(toNumber(y, 0), -1, 1)
        };

        this.intent = next;
        this._scheduleSend(source || this.defaultSource);
        return next;
    };

    MotionController.prototype._scheduleSend = function (source) {
        var now = Date.now();
        var elapsed = now - this.lastSentAt;

        if (this.sendInFlight) {
            this._armPending(source, this.rateLimitMs);
            return;
        }

        if (elapsed < this.rateLimitMs) {
            this._armPending(source, this.rateLimitMs - elapsed);
            return;
        }

        if (sameIntent(this.intent, this.lastSentIntent, this.epsilon)) {
            return;
        }

        this._sendNow(source);
    };

    MotionController.prototype._armPending = function (source, delayMs) {
        var self = this;
        if (self.pendingTimer) return;
        self.pendingTimer = setTimeout(function () {
            self.pendingTimer = null;
            self._scheduleSend(source || self.defaultSource);
        }, Math.max(10, delayMs || self.rateLimitMs));
    };

    MotionController.prototype._postIntent = async function (payload) {
        var attempt = 0;
        var self = this;
        while (attempt <= self.maxRetries) {
            var controller = null;
            var timeoutId = null;
            try {
                if (typeof AbortController !== 'undefined') {
                    controller = new AbortController();
                    timeoutId = setTimeout(function () {
                        try { controller.abort(); } catch (_) {}
                    }, self.requestTimeoutMs);
                }
                var response = await fetch('/api/motion_intent', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(payload),
                    signal: controller ? controller.signal : undefined
                });
                if (!response.ok) {
                    throw new Error('http-' + response.status);
                }
                return true;
            } catch (_) {
                attempt += 1;
                if (attempt > self.maxRetries) return false;
                await new Promise(function (resolve) { setTimeout(resolve, 60); });
            } finally {
                if (timeoutId) clearTimeout(timeoutId);
            }
        }
        return false;
    };

    MotionController.prototype._sendNow = async function (source) {
        this.sendInFlight = true;
        this.lastSentAt = Date.now();
        var payload = {
            x: this.intent.x,
            y: this.intent.y,
            source: source || this.defaultSource,
            seq: ++this.intentSeq,
            client_ts: Date.now()
        };
        try {
            var ok = await this._postIntent(payload);
            if (ok) {
                this.lastSentIntent = { x: this.intent.x, y: this.intent.y };
            }
        } catch (err) {
            // Halk hibatűrés: következő intent újraküldi.
        } finally {
            this.sendInFlight = false;
            if (!sameIntent(this.intent, this.lastSentIntent, this.epsilon)) {
                this._armPending(source, this.rateLimitMs);
            }
        }
    };

    window.R2B4_MotionController = new MotionController();
})();
