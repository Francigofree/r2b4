/**
 * API hívások a FastGUI saját backendjéhez (/api/*).
 */
(function () {
    var API_BASE = '';
    var TIMEOUT_MS = 15000;

    function fetchApi(path, options) {
        options = options || {};
        var url = path.startsWith('http') ? path : API_BASE + path;
        var init = {
            method: options.method || 'GET',
            headers: options.headers || {},
            signal: options.signal || null
        };
        if (options.body !== undefined) {
            init.body = typeof options.body === 'string' ? options.body : JSON.stringify(options.body);
            if (typeof options.body !== 'string') {
                init.headers['Content-Type'] = 'application/json';
            }
        }
        var controller = null;
        if (!init.signal && typeof AbortController !== 'undefined') {
            controller = new AbortController();
            init.signal = controller.signal;
            if (options.timeout !== false) {
                setTimeout(function () { if (controller) controller.abort(); }, options.timeout || TIMEOUT_MS);
            }
        }
        return fetch(url, init);
    }

    function get(path, opts) {
        return fetchApi(path, Object.assign({}, opts, { method: 'GET' }));
    }

    function post(path, body, opts) {
        return fetchApi(path, Object.assign({}, opts, { method: 'POST', body: body }));
    }

    function updateEkfShadow(params) {
        return post('/api/ekf/shadow/update', { params: params || {} });
    }

    function applyEkfShadowToLive() {
        return post('/api/ekf/shadow/apply', {});
    }

    window.R2B4_API = {
        API_BASE: API_BASE,
        get: get,
        post: post,
        fetchApi: fetchApi,
        updateEkfShadow: updateEkfShadow,
        applyEkfShadowToLive: applyEkfShadowToLive
    };
})();
