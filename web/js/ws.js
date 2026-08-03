// Transport: connect/reconnect to the device's WebSocket, parse frames into
// typed events. No DOM knowledge - app.js wires this to ui.js.

const RECONNECT_MS = 1000;

export class ArmSocket {
    constructor() {
        this._ws = null;
        this._connected = false;
        this._nextId = 0;
        this._handlers = { open: [], close: [], hello: [], state: [], ack: [], err: [] };
    }

    on(type, cb) {
        this._handlers[type].push(cb);
        return this;
    }

    connect() {
        const ws = new WebSocket(`ws://${location.host}/ws`);
        this._ws = ws;
        ws.onopen = () => {
            this._connected = true;
            this._emit('open');
        };
        ws.onclose = () => {
            this._connected = false;
            this._emit('close');
            setTimeout(() => this.connect(), RECONNECT_MS);
        };
        ws.onerror = () => ws.close();
        ws.onmessage = (ev) => this._onMessage(ev.data);
    }

    // Queue-less: silently dropped if not connected (docs/architecture.md -
    // WS disconnect must never queue up stale commands to fire on reconnect).
    // Every outgoing message gets a fresh id so the caller can match a later
    // ack/err back to this specific call. Returns the id, or null if dropped.
    send(cmd, args) {
        if (!this._connected || !this._ws) return null;
        const id = ++this._nextId;
        this._ws.send(JSON.stringify({ cmd, id, ...args }));
        return id;
    }

    _onMessage(data) {
        let msg;
        try {
            msg = JSON.parse(data);
        } catch {
            return;
        }
        if (msg && typeof msg.type === 'string' && this._handlers[msg.type]) {
            this._emit(msg.type, msg);
        }
    }

    _emit(type, msg) {
        for (const cb of this._handlers[type]) cb(msg);
    }
}
