// Gamepad input for the Control tab (browser Gamepad API, Xbox "standard" mapping).
//
// Thin on purpose: this module only reads the pad. control.js decides what the sticks and
// buttons mean and sends the commands.
//
//   initGamepad({ onTick, onButton, onConnection })
//     onTick({ lx, ly, rx })   every POLL_MS while a pad is connected; sticks after deadzone,
//                              -1..1, ly up = +1 (browser axes are y-down, so flipped here)
//     onButton(name)           on press (edge), name in BUTTONS
//     onConnection(pad|null)   on connect / disconnect
//
// setInterval, not requestAnimationFrame: rAF stops dead in a background tab, the interval only
// slows down. Either way the stream to the server stalls and its deadman zeroes the command.

const POLL_MS = 50;
const DEADZONE = 0.12;

// Standard-mapping button indices.
const BUTTONS = { 0: 'A', 1: 'B', 2: 'X', 3: 'Y', 8: 'View' };

let prevPressed = {};
let padIndex = null;

// Radial deadzone, rescaled so the output still starts at 0 and reaches 1.
function deadzone2(x, y) {
  const m = Math.hypot(x, y);
  if (m < DEADZONE) return [0, 0];
  const k = Math.min(1, (m - DEADZONE) / (1 - DEADZONE)) / m;
  return [x * k, y * k];
}

function deadzone1(v) {
  const m = Math.abs(v);
  if (m < DEADZONE) return 0;
  return Math.sign(v) * Math.min(1, (m - DEADZONE) / (1 - DEADZONE));
}

function currentPad() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  if (padIndex !== null && pads[padIndex]) return pads[padIndex];
  // Prefer a standard-mapped pad (axes/buttons at the indices above).
  const list = Array.from(pads).filter(Boolean);
  return list.find(p => p.mapping === 'standard') || list[0] || null;
}

export function gamepadSupported() {
  return typeof navigator.getGamepads === 'function';
}

export function initGamepad({ onTick, onButton, onConnection }) {
  if (!gamepadSupported()) { onConnection(null); return; }

  window.addEventListener('gamepadconnected', e => {
    padIndex = e.gamepad.index;
    prevPressed = {};
    onConnection(e.gamepad);
  });
  window.addEventListener('gamepaddisconnected', e => {
    if (padIndex === e.gamepad.index) padIndex = null;
    prevPressed = {};
    onConnection(currentPad());
  });

  setInterval(() => {
    const pad = currentPad();
    if (!pad || !pad.connected) return;
    padIndex = pad.index;

    for (const [i, name] of Object.entries(BUTTONS)) {
      const pressed = !!(pad.buttons[i] && pad.buttons[i].pressed);
      if (pressed && !prevPressed[i]) onButton(name);
      prevPressed[i] = pressed;
    }

    const [lx, ly] = deadzone2(pad.axes[0] || 0, -(pad.axes[1] || 0));
    const rx = deadzone1(pad.axes[2] || 0);
    onTick({ lx, ly, rx });
  }, POLL_MS);
}
