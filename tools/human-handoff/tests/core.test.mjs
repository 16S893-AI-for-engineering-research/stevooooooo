import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { spawn } from 'node:child_process';
import { Handoffs, validateProposal, fingerprint, assertApproved, stateOf, procedure, updatedNotes, parseAcceptance, locked, saveJSON } from '../core.mjs';
import { terminalArgs, guideRunning } from '../launcher.mjs';

export const example = (outcome = 'Read the serial label') => ({ title: 'Photograph controller label', parent: 'AA-1', category: 'physical-access', reason: 'Only the user can see the physical label.', alternatives: 'The supplied CAD does not include the label; no camera is accessible.', outcome, preparation: 'Powered-off controller and camera; no wiring changes.', minutes: 2, steps: [{ action: 'Photograph the serial label without moving wires.', expected: 'The complete label is legible.', verification: 'Read the label in the photograph.', stop: 'The label requires removing covers or moving wiring.' }] });

function fakeCLI() {
  const tasks = new Map([['AA-1', { id: 'AA-1', title: 'Parent', labels: [], description: '', acceptanceCriteria: [], implementationNotes: '' }]]);
  let seq = 1;
  const run = async (_repo, args) => {
    const val = flag => args[args.indexOf(flag) + 1];
    if (args[1] === 'create') {
      const id = `AA-${++seq}`;
      tasks.set(id, { id, title: args[2], description: val('-d'), labels: val('-l').split(','), status: 'To Do', acceptanceCriteria: [{ text: val('--ac'), checked: false }], implementationNotes: '' });
      return '';
    }
    if (args[1] === 'list') return JSON.stringify({ schemaVersion: 1, tasks: [...tasks.values()].filter(t => t.labels.includes(val('--labels'))).map(t => ({ id: t.id })) });
    const task = tasks.get(args[2]);
    if (!task) throw new Error('unknown task ID');
    if (args[1] === 'view') return JSON.stringify({ schemaVersion: 1, task });
    if (args[1] === 'edit') {
      task.implementationNotes = val('--notes');
      if (args.includes('-s')) task.status = val('-s');
      if (args.includes('--check-ac')) task.acceptanceCriteria[0].checked = true;
      return '';
    }
    throw new Error(`Unexpected command: ${args}`);
  };
  return { run, tasks };
}
async function fixture(t) {
  const root = await mkdtemp(join(tmpdir(), 'human-handoff-test-'));
  t.after(() => rm(root, { recursive: true, force: true }));
  const cli = fakeCLI();
  return { store: new Handoffs(join(root, 'state'), cli.run), repo: root, ...cli };
}

test('proposal validation requires human-only justification and verifiable minimal steps', () => {
  assert.equal(validateProposal(example()).steps.length, 1);
  assert.throws(() => validateProposal({ ...example(), category: 'agent-is-busy' }));
  assert.throws(() => validateProposal({ ...example(), alternatives: '' }));
  assert.throws(() => validateProposal({ ...example(), steps: [{ action: 'do it' }] }));
  assert.throws(() => validateProposal({ ...example(), parent: '--dangerous' }));
});

test('acceptance parser only recognizes explicit interactive user input', () => {
  assert.equal(parseAcceptance('accept aa-12', 'interactive'), 'AA-12');
  for (const source of ['extension', 'rpc', undefined]) assert.equal(parseAcceptance('accept AA-12', source), null);
  for (const input of ['accept it', 'I might accept AA-12', 'accept AA-12 and AA-13']) assert.equal(parseAcceptance(input, 'interactive'), null);
});

test('proposals are deduplicated, including declined/deferred requests', async t => {
  const { store, repo } = await fixture(t);
  const first = await store.propose(repo, example(), 'origin');
  assert.equal(first.state.phase, 'proposed');
  assert.equal(await store.startNext(), null);
  await store.decide(repo, first.task.id, 'decline', fingerprint(first.task));
  const again = await store.propose(repo, example(), 'origin');
  assert.equal(again.task.id, first.task.id);
  assert.equal(again.duplicate, true);
  assert.equal(again.state.phase, 'declined');
});

test('accepted tasks progress one step, pause/resume, then await verification', async t => {
  const { store, repo } = await fixture(t);
  const first = await store.propose(repo, example(), 'origin');
  await store.decide(repo, first.task.id, 'accept', fingerprint(first.task));
  const active = await store.startNext();
  assert.equal(active.state.phase, 'active');
  await assert.rejects(store.finish('done', fingerprint(active.task)), /Complete each step/);
  await assert.rejects(store.checkpoint(1, 'Readable', 'photo', '', true), /reference is required/);
  await store.pause();
  await assert.rejects(store.checkpoint(1, 'Done', 'self-reported', '', true), /No active/);
  assert.equal((await store.startNext()).state.step, 0);
  await store.checkpoint(1, 'User reports label is readable', 'self-reported', '', true);
  await assert.rejects(store.checkpoint(1, 'duplicate', 'self-reported', '', true), /Stale step/);
  const ready = await store.finish('Label photographed, awaiting review', fingerprint(active.task));
  assert.equal(ready.state.phase, 'ready');
  assert.notEqual((await store.get(repo, ready.task.id)).status, 'Done');
  const verified = await store.verify(repo, ready.task.id, 'Reviewed the label photo against the manufacturer specification.');
  assert.equal(verified.state.phase, 'verified');
  assert.equal((await store.get(repo, ready.task.id)).status, 'Done');
  assert.equal((await store.get(repo, 'AA-1')).acceptanceCriteria.length, 0);
});

test('changed scope invalidates review and prior acceptance; matching prose is required', async t => {
  const { store, repo, tasks } = await fixture(t);
  const first = await store.propose(repo, example(), 'origin');
  const hash = fingerprint(first.task);
  tasks.get(first.task.id).description += '\nExtra instruction not in approved scope.';
  await assert.rejects(store.decide(repo, first.task.id, 'accept', hash), /changed/);
  assert.throws(() => procedure(tasks.get(first.task.id)), /differ/);
  tasks.get(first.task.id).description = first.task.description;
  await store.decide(repo, first.task.id, 'accept', hash);
  tasks.get(first.task.id).acceptanceCriteria[0].text = 'Also rewire the controller';
  assert.throws(() => assertApproved({ task: tasks.get(first.task.id), state: { acceptedHash: hash } }), /changed/);
  await assert.rejects(store.startNext(), /changed/);
});

test('new accepted work does not interrupt active task; next advances in same queue', async t => {
  const { store, repo } = await fixture(t);
  const a = await store.propose(repo, example('Read label A'), 'origin');
  await store.decide(repo, a.task.id, 'accept', fingerprint(a.task));
  await store.startNext();
  const b = await store.propose(repo, example('Read label B'), 'origin');
  await store.decide(repo, b.task.id, 'accept', fingerprint(b.task));
  assert.equal((await store.startNext()).task.id, a.task.id);
  await store.checkpoint(1, 'User reports A complete', 'self-reported', '', true);
  await store.finish('A reported complete', fingerprint(a.task));
  assert.equal((await store.startNext()).task.id, b.task.id);
  // New engine object represents a restarted guide process.
  const restarted = new Handoffs(store.home, store.run);
  assert.equal((await restarted.current()).task.id, b.task.id);
});

test('notes outside owned block are preserved and malformed records fail closed', () => {
  const task = { id: 'AA-1', implementationNotes: 'Existing human notes' };
  const one = updatedNotes(task, { version: 1, phase: 'proposed' });
  const two = updatedNotes({ ...task, implementationNotes: one + '\nManual comment' }, { version: 1, phase: 'accepted' });
  assert.ok(two.startsWith('Existing human notes'));
  assert.ok(two.endsWith('Manual comment'));
  assert.equal(stateOf({ ...task, implementationNotes: two }).phase, 'accepted');
  assert.throws(() => stateOf(task), /missing/);
});

test('cross-process lock serializes concurrent proposals; singleton lock released on exit', async t => {
  const { store, repo } = await fixture(t);
  const [a, b] = await Promise.all([store.propose(repo, example(), 'origin'), store.propose(repo, example(), 'origin')]);
  assert.equal(a.task.id, b.task.id);
  assert.equal((await store.list(repo)).length, 1);
  assert.equal(await guideRunning(store.home), false);
  const child = spawn('flock', [join(store.home, 'guide.lock'), process.execPath, '-e', "console.log('ready');process.stdin.resume()"], { stdio: ['pipe', 'pipe', 'pipe'] });
  await new Promise((resolve, reject) => { child.stdout.once('data', resolve); child.once('error', reject); });
  assert.equal(await guideRunning(store.home), true);
  const ended = new Promise(resolve => child.once('close', resolve));
  child.stdin.end();
  await ended;
  assert.equal(await guideRunning(store.home), false);
  await locked(store.home, async () => saveJSON(join(store.home, 'test.json'), { ok: true }));
});

test('terminal arguments target an explicit window, preserve spaces, and use no shell interpolation', () => {
  const args = terminalArgs({ window: '0', distro: 'Ubuntu' }, '/home/name with spaces/state', '/usr/bin/node', '/project with spaces/runner.mjs');
  assert.deepEqual(args.slice(0, 8), ['-w', '0', 'new-tab', '--title', 'Pi Walkthrough', 'wsl.exe', '-d', 'Ubuntu']);
  assert.ok(args.includes('/home/name with spaces/state/guide.lock'));
  assert.ok(args.includes('/project with spaces/runner.mjs'));
  assert.ok(args.includes('--exec'));
  assert.ok(!args.includes('-c'));
});
