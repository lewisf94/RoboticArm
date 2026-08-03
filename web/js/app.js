// Entry point: wires ws.js transport events to ui.js rendering, and ui.js
// user-action callbacks back to ws.js sends. Owns the small bits of state
// that don't belong in either: the joint-slider throttle and the
// request-id -> joint index map used to flash the right row on an
// out_of_range error.

import { ArmSocket } from './ws.js';
import * as ui from './ui.js';

const THROTTLE_MS = 100; // 10 Hz cap on set_joint/grip while dragging (task spec)

const sock = new ArmSocket();

// req id -> joint index, for correlating a later ack/err back to the joint
// slider that caused it (get_profile's request has no entry - id -> undefined
// is treated as "not joint-specific" throughout).
const pending = new Map();

// Per-joint trailing-edge throttle: a burst of `input` events during a drag
// collapses to at most one send every THROTTLE_MS, always carrying the most
// recent value rather than an intermediate one.
const lastSentAt = new Map();
const pendingTimers = new Map();

function throttled(key, fn) {
    const now = performance.now();
    const elapsed = now - (lastSentAt.get(key) || 0);
    if (elapsed >= THROTTLE_MS) {
        lastSentAt.set(key, now);
        fn();
        return;
    }
    clearTimeout(pendingTimers.get(key));
    pendingTimers.set(
        key,
        setTimeout(() => {
            pendingTimers.delete(key);
            lastSentAt.set(key, performance.now());
            fn();
        }, THROTTLE_MS - elapsed)
    );
}

function sendTracked(cmd, args, joint) {
    const id = sock.send(cmd, args);
    if (id != null) pending.set(id, joint);
}

sock.on('open', () => ui.setConnected(true));
sock.on('close', () => ui.setConnected(false));

sock.on('hello', (msg) => {
    ui.setProfileName(msg.profile);
    sock.send('get_profile');
});

sock.on('ack', (msg) => {
    if (msg.id != null) pending.delete(msg.id);
    if (msg.cmd === 'get_profile') {
        ui.buildJoints(
            msg.joints,
            (j, deg) => throttled(j, () => sendTracked('set_joint', { j, deg }, j)),
            (j, pct) => throttled(j, () => sendTracked('grip', { pct }, j))
        );
        ui.buildTrims(msg.joints, (j, deg) => sendTracked('set_trim', { j, deg }, j));
    }
});

sock.on('err', (msg) => {
    ui.toast(`${msg.cmd}: ${msg.msg}`);
    const joint = msg.id != null ? pending.get(msg.id) : undefined;
    if (msg.id != null) pending.delete(msg.id);
    if (msg.code === 'out_of_range' && joint != null) ui.flashJoint(joint);
});

sock.on('state', (msg) => {
    ui.updateState(msg);
    ui.setWifi(msg.wifi);
});

ui.init({
    onEnableToggle: (on) => sock.send('enable', { on }),
    onEstop: () => sock.send('estop'),
});

sock.connect();
