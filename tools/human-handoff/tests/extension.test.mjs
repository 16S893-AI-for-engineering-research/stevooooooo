import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtemp, realpath, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join, dirname, resolve } from 'node:path';
import { createRequire } from 'node:module';
import { execFile } from 'node:child_process';
import { promisify } from 'node:util';
import { Handoffs, backlog } from '../core.mjs';

async function extensionFactory() {
  const { stdout } = await promisify(execFile)('which', ['pi']);
  const piRoot = resolve(dirname(await realpath(stdout.trim())), '../..');
  const req = createRequire(join(piRoot, 'package.json'));
  const { createJiti } = req('jiti');
  const jiti = createJiti(import.meta.url, { alias: {
    typebox: req.resolve('typebox'),
    '@earendil-works/pi-ai': join(piRoot, 'node_modules/@earendil-works/pi-ai/dist/index.js'),
  } });
  return (await jiti.import(resolve('.pi/extensions/human-handoff/index.ts'))).default;
}
function harness(factory) {
  const handlers = new Map(), commands = new Map(), tools = new Map(), messages = [], entries = [];
  const pi = { on: (n, fn) => handlers.set(n, fn), registerCommand: (n, cmd) => commands.set(n, cmd), registerTool: tool => tools.set(tool.name, tool), sendMessage: (m, options) => messages.push({ ...m, options }), appendEntry: (customType, data) => entries.push({ type: 'custom', customType, data }), setActiveTools() {} };
  factory(pi);
  return { handlers, commands, tools, messages, entries };
}

test('real extension: noninteractive/synthetic acceptance blocked; user confirmation required', { timeout: 90000 }, async t => {
  const repo = await mkdtemp(join(tmpdir(), 'handoff-ui-test-'));
  const home = join(repo, 'private');
  const previous = process.env.PI_HUMAN_STATE_HOME;
  process.env.PI_HUMAN_STATE_HOME = home;
  t.after(async () => { if (previous === undefined) delete process.env.PI_HUMAN_STATE_HOME; else process.env.PI_HUMAN_STATE_HOME = previous; await rm(repo, { recursive: true, force: true }); });
  await backlog(repo, ['init', 'UI test', '--defaults', '--no-git', '--integration-mode', 'none', '--task-prefix', 'UT']);
  await backlog(repo, ['task', 'create', 'Parent', '--plain']);
  const parent = JSON.parse(await backlog(repo, ['task', 'list', '--json'])).tasks[0].id;
  const h = harness(await extensionFactory());
  const confirmations = [], notifications = [];
  const answers = [];
  const ctx = { cwd: repo, mode: 'tui', hasUI: true, ui: { confirm: async (...args) => { confirmations.push(args); return answers.shift() ?? false; }, notify: (...args) => notifications.push(args), setStatus() {} }, sessionManager: { getSessionId: () => 'test-origin', getEntries: () => h.entries } };
  assert.ok(h.tools.has('human_request'));
  assert.ok(!h.tools.has('human_accept'));
  assert.ok(!h.tools.has('human_checkpoint'));
  await h.tools.get('human_request').execute('test', { title: 'Test observation', parent, category: 'physical-access', reason: 'Only physical access can observe the label.', alternatives: 'No camera is accessible.', outcome: 'Read test label', preparation: 'No hardware; fixture only', minutes: 1, steps: [{ action: 'Observe label', expected: 'TEST', verification: 'Compare text', stop: 'Not a test fixture' }] }, null, null, ctx);
  const store = new Handoffs(home);
  const row = (await store.list(repo))[0];
  assert.equal((await h.handlers.get('input')({ text: `accept ${row.task.id}`, source: 'extension' }, ctx)).action, 'continue');
  assert.equal(confirmations.length, 0);
  await h.commands.get('human').handler(`accept ${row.task.id}`, { ...ctx, mode: 'rpc' });
  assert.equal(confirmations.length, 0);
  assert.equal((await store.item(repo, row.task.id)).state.phase, 'proposed');
  answers.push(false);
  await h.handlers.get('input')({ text: `accept ${row.task.id}`, source: 'interactive' }, ctx);
  assert.equal(confirmations.length, 1);
  assert.equal((await store.item(repo, row.task.id)).state.phase, 'proposed');
  answers.push(true, false); // Accept the procedure; cancel terminal setup (no actual launch in unit test).
  await h.handlers.get('input')({ text: `accept ${row.task.id}`, source: 'interactive' }, ctx);
  assert.equal((await store.item(repo, row.task.id)).state.phase, 'accepted');
  assert.equal(confirmations.length, 3);
  assert.ok(notifications.some(n => n[0].includes('Task stays accepted')));
  await store.startNext();
  await store.checkpoint(1, 'TEST observed', 'self-reported', '', true);
  const accepted = await store.item(repo, row.task.id);
  await store.finish('Fixture ready for verification', accepted.state.acceptedHash);
  await h.handlers.get('session_start')({}, ctx);
  try {
    const deadline = Date.now() + 8000;
    while (!h.messages.some(m => m.customType === 'human-result') && Date.now() < deadline) await new Promise(r => setTimeout(r, 30));
    const result = h.messages.find(m => m.customType === 'human-result');
    assert.ok(result, 'Ready result is delivered back to orchestrator');
    assert.equal(result.options.deliverAs, 'followUp');
    assert.equal(result.options.triggerTurn, true);
    assert.ok(h.entries.some(e => e.customType === 'human-result-seen'));
  } finally { h.handlers.get('session_shutdown')(); }
});

test('guide role exposes no proposal/verification tools and blocks unrelated tools', async t => {
  const previous = process.env.PI_HUMAN_GUIDE;
  process.env.PI_HUMAN_GUIDE = '1';
  t.after(() => { if (previous === undefined) delete process.env.PI_HUMAN_GUIDE; else process.env.PI_HUMAN_GUIDE = previous; });
  const h = harness(await extensionFactory());
  assert.deepEqual([...h.tools.keys()].sort(), ['human_checkpoint', 'human_queue']);
  assert.equal(h.handlers.get('tool_call')({ toolName: 'bash' }).block, true);
  assert.equal(h.handlers.get('tool_call')({ toolName: 'human_request' }).block, true);
  assert.equal(h.handlers.get('tool_call')({ toolName: 'read' }), undefined);
});
