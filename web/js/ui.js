// DOM layer: builds/updates the page. No WebSocket knowledge - app.js
// supplies plain callbacks for user actions and calls these functions when
// data arrives.

const dot = document.getElementById('dot');
const profileNameEl = document.getElementById('profile-name');
const wifiEl = document.getElementById('wifi');
const enableToggle = document.getElementById('enable-toggle');
const estopBtn = document.getElementById('estop');
const jointsEl = document.getElementById('joints');
const jointsPlaceholder = document.getElementById('joints-placeholder');
const trimRowsEl = document.getElementById('trim-rows');
const toastsEl = document.getElementById('toasts');

// Row state, indexed by joint index j. Rebuilt by buildJoints().
let rows = [];

function fmtDeg(v) {
    return `${v.toFixed(1)}°`;
}

function pctOfRange(v, min, max) {
    return ((v - min) / (max - min)) * 100;
}

function degToPct(deg, jc) {
    return pctOfRange(deg, jc.min, jc.max);
}

export function init({ onEnableToggle, onEstop }) {
    enableToggle.addEventListener('change', () => onEnableToggle(enableToggle.checked));
    estopBtn.addEventListener('click', () => onEstop());
}

export function setConnected(connected) {
    dot.classList.toggle('connected', connected);
    dot.title = connected ? 'connected' : 'disconnected';
    if (!connected) {
        // A dead link means whatever rows exist are stale and no more state
        // updates are coming - showing them as live would be misleading.
        // replaceChildren(jointsPlaceholder) both clears the old rows and
        // puts the (possibly already-detached) placeholder node back in one
        // step, rather than clearing everything and then mutating a node
        // that clearing just removed.
        jointsEl.replaceChildren(jointsPlaceholder);
        trimRowsEl.replaceChildren();
        jointsPlaceholder.hidden = false;
        jointsPlaceholder.textContent = 'disconnected — retrying…';
        rows = [];
        enableToggle.checked = false;
        enableToggle.disabled = true;
        wifiEl.textContent = '–';
    }
}

export function setProfileName(name) {
    profileNameEl.textContent = name;
}

export function setWifi(wifi) {
    if (!wifi) {
        wifiEl.textContent = '–';
        return;
    }
    if (wifi.mode === 'sta') {
        wifiEl.textContent = `${wifi.rssi} dBm`;
    } else {
        wifiEl.textContent = wifi.mode;
    }
}

// joints: the `joints` array from a get_profile ack ({name,min,max,home,vmax,gripper}).
// Rebuilds from scratch and is safe to call more than once per page load
// (e.g. a reconnect mid-session) - nothing here assumes it only runs once.
export function buildJoints(joints, onJointInput, onGripInput) {
    jointsPlaceholder.hidden = true;
    enableToggle.disabled = false;
    rows = joints.map((jc, j) => {
        const row = document.createElement('div');
        row.className = 'joint-row';

        const head = document.createElement('div');
        head.className = 'joint-head';
        const nameEl = document.createElement('span');
        nameEl.className = 'joint-name';
        nameEl.textContent = jc.gripper ? `${jc.name} (grip)` : jc.name;
        const readoutEl = document.createElement('span');
        readoutEl.className = 'joint-readout';
        head.append(nameEl, readoutEl);

        const wrap = document.createElement('div');
        wrap.className = 'slider-wrap';
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.step = jc.gripper ? '1' : '0.5';
        slider.min = jc.gripper ? '0' : String(jc.min);
        slider.max = jc.gripper ? '100' : String(jc.max);
        const marker = document.createElement('div');
        marker.className = 'target-marker';
        wrap.append(slider, marker);

        row.append(head, wrap);

        const state = { row, slider, readoutEl, marker, jc, dragging: false };

        slider.addEventListener('pointerdown', () => {
            state.dragging = true;
        });
        const endDrag = () => {
            state.dragging = false;
        };
        slider.addEventListener('pointerup', endDrag);
        slider.addEventListener('pointercancel', endDrag);

        slider.addEventListener('input', () => {
            const raw = Number(slider.value);
            marker.style.left = `${pctOfRange(raw, Number(slider.min), Number(slider.max))}%`;
            if (jc.gripper) {
                onGripInput(j, raw);
            } else {
                onJointInput(j, raw);
            }
        });

        return state;
    });
    jointsEl.replaceChildren(jointsPlaceholder, ...rows.map((r) => r.row));
}

// trimJoints: same shape as buildJoints' `joints`. There's no protocol
// getter for the persisted trim (docs/protocol.md), so every row starts at
// 0 each page load; only a slider the user actually touches sends anything.
export function buildTrims(joints, onTrimChange) {
    trimRowsEl.replaceChildren();
    joints.forEach((jc, j) => {
        const row = document.createElement('div');
        row.className = 'trim-row';

        const head = document.createElement('div');
        head.className = 'joint-head';
        const nameEl = document.createElement('span');
        nameEl.className = 'joint-name';
        nameEl.textContent = jc.name;
        const readoutEl = document.createElement('span');
        readoutEl.className = 'joint-readout';
        readoutEl.textContent = fmtDeg(0);
        head.append(nameEl, readoutEl);

        const slider = document.createElement('input');
        slider.type = 'range';
        slider.className = 'trim-slider';
        slider.min = '-10';
        slider.max = '10';
        slider.step = '0.5';
        slider.value = '0';

        row.append(head, slider);
        trimRowsEl.appendChild(row);

        slider.addEventListener('input', () => {
            readoutEl.textContent = fmtDeg(Number(slider.value));
        });
        slider.addEventListener('change', () => onTrimChange(j, Number(slider.value)));
    });
}

// state: a `state` message ({t,en,j,tgt,heap,wifi,...}).
export function updateState(state) {
    enableToggle.checked = state.en;
    for (let j = 0; j < rows.length; ++j) {
        const r = rows[j];
        const cur = state.j[j];
        const tgt = state.tgt[j];
        if (r.jc.gripper) {
            r.readoutEl.textContent = `${Math.round(degToPct(cur, r.jc))}%`;
            if (!r.dragging) r.slider.value = String(degToPct(cur, r.jc));
            r.marker.style.left = `${degToPct(tgt, r.jc)}%`;
        } else {
            r.readoutEl.textContent = fmtDeg(cur);
            if (!r.dragging) r.slider.value = String(cur);
            r.marker.style.left = `${pctOfRange(tgt, r.jc.min, r.jc.max)}%`;
        }
        r.slider.disabled = !state.en;
    }
    const trimSliders = trimRowsEl.querySelectorAll('.trim-slider');
    trimSliders.forEach((s) => {
        s.disabled = !state.en;
    });
}

export function flashJoint(j) {
    const r = rows[j];
    if (!r) return;
    r.row.classList.remove('flash');
    // Force reflow so re-adding the class restarts the animation even if
    // two out_of_range errors land on the same joint in quick succession.
    void r.row.offsetWidth;
    r.row.classList.add('flash');
    r.row.addEventListener('animationend', () => r.row.classList.remove('flash'), { once: true });
}

export function toast(message) {
    const el = document.createElement('div');
    el.className = 'toast';
    el.textContent = message;
    toastsEl.appendChild(el);
    setTimeout(() => el.remove(), 3000);
}
