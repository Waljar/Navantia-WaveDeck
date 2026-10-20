#!/usr/bin/env node
/* Browser test of docs/control.html against a simulated serial port: paced key sending, STOP, live keystrokes,
 * held keys, sliders, Setup pads, recovery of "back to menu".
 *   npm i playwright && node tools/test-control.js        (CHROMIUM=/path/to/chromium to use an installed browser) */
const path = require('path');
let chromium;
try { ({ chromium } = require('playwright')); } catch (e) { ({ chromium } = require('/opt/npm-tools/node_modules/playwright')); }
(async () => {
  const exe = process.env.CHROMIUM || (require('fs').existsSync('/opt/pw-browsers/chromium') ? '/opt/pw-browsers/chromium' : undefined);
  const b = await chromium.launch({ executablePath: exe, args: ['--no-sandbox'] });
  const ctx = await b.newContext({ viewport: { width: 1400, height: 900 } });
  const p = await ctx.newPage();
  const errs = [];
  p.on('console', (m) => { if (m.type() === 'error') errs.push(m.text()); });
  p.on('pageerror', (e) => errs.push('PAGEERROR ' + e.message));
  let ok = true; const check = (c, msg) => { console.log((c ? 'PASS ' : 'FAIL ') + msg); if (!c) ok = false; };

  await p.addInitScript(() => {
    const enc = new TextEncoder();
    const M = window.__mock = { log: [], mode: 0, qNeeded: 1, qSeen: 0, teleOn: false, ctrl: null };
    M.emit = (s) => M.ctrl.enqueue(enc.encode(s));
    M.menu = () => M.emit('\r\nStatus: HOMED\r\n\r\n=========== SEA PLATFORM ===========\r\n 1  Setup\r\nSelect: ');
    const port = {
      open: async () => {}, close: async () => {},
      readable: new ReadableStream({ start(c) { M.ctrl = c; } }),
      writable: new WritableStream({ write(chunk) {
        const s = new TextDecoder().decode(chunk);
        for (const ch of s) {
          M.log.push({ t: performance.now(), ch });
          if (ch === '\r' && M.mode === 0) M.menu();
          else if (ch === 'q' && M.mode !== 0) { if (++M.qSeen >= M.qNeeded) { M.mode = 0; M.qSeen = 0; setTimeout(M.menu, 50); } }
          else if (ch >= '1' && ch <= '6' && M.mode === 0) { M.mode = +ch; M.emit('\r\n=== MODE ' + ch + ' ===\r\n'); if (ch === '1') M.emit('\r\n=== SETUP ===\r\nSelect: '); }
        }
      } }),
    };
    Object.defineProperty(navigator, 'serial', { value: { requestPort: async () => port }, configurable: true });
  });
  const chars = () => p.evaluate(() => window.__mock.log.map((e) => e.ch).join(''));
  const clear = () => p.evaluate(() => { window.__mock.log.length = 0; });
  const gaps = () => p.evaluate(() => { const l = window.__mock.log; return l.slice(1).map((e, i) => e.t - l[i].t); });

  await p.goto('file://' + path.resolve(__dirname, '../docs/control.html'));
  await p.waitForTimeout(400);
  await p.click('#conn'); await p.waitForTimeout(2200);
  check(/Connected|telemetry/i.test(await p.textContent('#state')), 'connects (' + await p.textContent('#state') + ')');

  /* 1. a burst typed in the box goes out one character at a time */
  await clear();
  await p.fill('#raw', 'h12.5'); await p.press('#raw', 'Enter');
  await p.waitForTimeout(400);
  check((await chars()) === 'h12.5\r', 'typed text arrives complete and in order: ' + JSON.stringify(await chars()));
  const g1 = await gaps();
  check(g1.length === 5 && Math.min(...g1) >= 14, 'characters are spaced (min gap ' + Math.min(...g1).toFixed(1) + ' ms)');

  /* 2. mode buttons wait for the menu, ask again until it is back */
  await p.click('[data-mode="4"]'); await p.waitForTimeout(1500);
  await p.evaluate(() => { window.__mock.qNeeded = 3; window.__mock.qSeen = 0; });
  await clear();
  await p.click('[data-mode="2"]'); await p.waitForTimeout(9000);
  const c2 = await chars();
  check(/^q[\s\S]*2$/.test(c2) && (c2.match(/q/g) || []).length >= 3, 'goMenu repeats q until the menu is back, then starts the mode: ' + JSON.stringify(c2));
  /* panel believes it is in a mode but the board is already at the menu: Enter reprints it, no 20 s wait */
  await p.evaluate(() => { window.__mock.mode = 0; window.__mock.qNeeded = 1; });
  await clear();
  await p.click('[data-mode="5"]'); await p.waitForTimeout(4500);
  const c2b = await chars();
  check(/^q\r5$/.test(c2b), 'out-of-step panel recovers with Enter: ' + JSON.stringify(c2b));

  /* 3. live keys: a held key does not repeat (mode 2) */
  await p.check('#kbd'); await clear();
  await p.keyboard.down('p'); await p.keyboard.down('p'); await p.keyboard.down('p'); await p.keyboard.down('p'); await p.keyboard.up('p');
  await p.waitForTimeout(300);
  check((await chars()) === 'p', 'held key sends once outside jog modes: ' + JSON.stringify(await chars()));

  /* 4. focus on the checkbox: Space goes to the board, does not untick the box */
  await p.focus('#kbd'); await clear();
  await p.keyboard.press(' ');
  await p.waitForTimeout(200);
  check((await chars()) === ' ' && (await p.isChecked('#kbd')), 'Space with the checkbox focused is sent and does not untick it');

  /* 5. Enter with a pad button focused only sends Enter, never also clicks the button */
  await p.evaluate(() => { const b = document.querySelector('#pad .btn'); if (b) b.focus(); });
  await clear();
  await p.keyboard.press('Enter'); await p.waitForTimeout(200);
  check((await chars()) === '\r', 'Enter with a button focused sends only Enter: ' + JSON.stringify(await chars()));

  /* 6. typing into the text box while live: goes to the board, not into the box */
  await p.evaluate(() => { window.__mock.mode = 0; });
  check(await p.isDisabled('#raw'), 'text box is switched off while live keys are on');
  await p.uncheck('#kbd');

  /* 7. manual mode: held jog key is rate-limited and never queues */
  await p.click('[data-mode="6"]'); await p.waitForTimeout(3500);
  await p.check('#kbd'); await p.evaluate(() => document.activeElement && document.activeElement.blur());
  await clear();
  await p.keyboard.down('u');
  for (let i = 0; i < 40; i++) { await p.keyboard.down('u'); await p.waitForTimeout(25); }       /* ~1 s of auto-repeat at 40 Hz */
  await p.keyboard.up('u');
  await p.waitForTimeout(400);
  const n7 = (await chars()).length;
  check(n7 >= 4 && n7 <= 10, 'held jog key: ' + n7 + ' sent in about 1 s (limit 8 per second)');
  const q7 = await p.evaluate(() => window.__mock.log.length);

  /* 8. STOP jumps the queue and clears what is waiting */
  await p.uncheck('#kbd'); await clear();
  await p.evaluate(() => { document.getElementById('raw').value = 'abcdefghijklmnopqrst'; document.getElementById('send').click(); });
  await p.waitForTimeout(60);
  await p.click('#stop'); await p.waitForTimeout(400);
  const c8 = await chars();
  check(c8.includes('x') && c8.length < 12, 'STOP is sent at once and the waiting keys are dropped: ' + JSON.stringify(c8));

  /* 9. refusing a flood */
  await clear();
  await p.evaluate(() => { document.getElementById('raw').removeAttribute('maxlength'); document.getElementById('raw').value = 'a'.repeat(200); document.getElementById('send').click(); });
  await p.waitForTimeout(300);
  check((await chars()).length === 0, 'a flood of 200 characters is refused, not queued');
  check(/too many keys/.test(await p.textContent('#log')), 'the refusal is shown in the messages');

  /* 10. sliders: only the newest pose waits */
  await p.waitForTimeout(500); await clear();
  await p.evaluate(() => { const s = document.getElementById('sh'); [10, 11, 12, 13, 14].forEach((v) => { s.value = v; s.dispatchEvent(new Event('change')); }); });
  await p.waitForTimeout(1500);
  const c10 = await chars();
  check(c10 === 'h10\rh14\r' || c10 === 'h14\r', 'older slider values are replaced: ' + JSON.stringify(c10));

  /* 11. Setup screens follow what the board prints */
  await p.click('[data-mode="1"]'); await p.waitForTimeout(4500);
  check(/leg difference/.test(await p.textContent('#pad')) && /tilt limits/.test(await p.textContent('#pad')), 'Setup menu pad lists options 4 and 5');
  await p.evaluate(() => window.__mock.emit('\r\n--- Find the TILT limits by hand ---\r\nw / s = pitch\r\n'));
  await p.waitForTimeout(400);
  check(/mark PITCH limit here/.test(await p.textContent('#pad')), 'tilt screen pad appears');
  await p.evaluate(() => window.__mock.emit('\r\n--- Set the largest DIFFERENCE between two legs ---\r\n'));
  await p.waitForTimeout(400);
  check(/mark this difference/.test(await p.textContent('#pad')), 'difference screen pad appears');
  await p.evaluate(() => window.__mock.emit('\r\n=== SETUP ===\r\nSelect: '));
  await p.waitForTimeout(400);
  check(/save to flash/.test(await p.textContent('#pad')), 'back to the Setup menu pad');
  /* a held d in the Setup menu must NOT repeat (it would restore defaults) */
  await p.check('#kbd'); await p.evaluate(() => document.activeElement && document.activeElement.blur()); await clear();
  await p.keyboard.down('d'); await p.keyboard.down('d'); await p.keyboard.down('d'); await p.keyboard.up('d'); await p.waitForTimeout(200);
  check((await chars()) === 'd', 'held d in the Setup menu is sent once');

  /* 12. disconnect switches live keys off */
  await p.click('#conn'); await p.waitForTimeout(500);
  check(!(await p.isChecked('#kbd')), 'live keys switch off when the port closes');

    check(errs.length === 0, 'no console errors' + (errs.length ? ': ' + errs.join(' | ') : ''));
  await b.close();
  process.exit(ok ? 0 : 1);
})();
