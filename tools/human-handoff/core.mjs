import { execFile, spawn } from 'node:child_process';
import { promisify } from 'node:util';
import { mkdir, readFile, writeFile, rename, realpath } from 'node:fs/promises';
import { homedir } from 'node:os';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash, randomUUID } from 'node:crypto';

const exec = promisify(execFile);
const HERE = dirname(fileURLToPath(import.meta.url));
export const STATE_HOME = join(homedir(), '.pi', 'agent', 'human-handoff');
const BEGIN = '<!-- human-handoff-state:v1 -->';
const END = '<!-- /human-handoff-state -->';
const PROC = /```human-procedure\n([\s\S]*?)\n```/;
export const idPattern = /^[A-Z][A-Z0-9]*-\d+(?:\.\d+)*$/;
export const now = () => new Date().toISOString();

export async function readJSON(path, fallback) {
  try { return JSON.parse(await readFile(path, 'utf8')); }
  catch (e) { if (e.code === 'ENOENT') return fallback; throw e; }
}
export async function saveJSON(path, value) {
  await mkdir(dirname(path), { recursive: true, mode: 0o700 });
  const tmp = `${path}.${randomUUID()}.tmp`;
  await writeFile(tmp, JSON.stringify(value, null, 2) + '\n', { mode: 0o600 });
  await rename(tmp, path);
}

// A kernel-owned lock: crashes release it; there is no stale PID file to delete.
export async function locked(home, fn) {
  await mkdir(home, { recursive: true, mode: 0o700 });
  const child = spawn('flock', ['-w', '20', join(home, 'operations.lock'), process.execPath, join(HERE, 'lock-holder.mjs')], { stdio: ['pipe', 'pipe', 'pipe'] });
  let stderr = '';
  child.stderr.on('data', b => { stderr += b; });
  const exited = new Promise(resolve => child.once('close', resolve));
  try {
    await new Promise((resolve, reject) => {
      child.once('error', reject);
      child.stdout.once('data', () => resolve());
      child.once('close', code => reject(new Error(`Handoff lock failed (${code}): ${stderr}`)));
    });
    return await fn();
  } finally {
    child.stdin.end();
    await exited;
  }
}

export async function backlog(repo, args) {
  const env = { ...process.env };
  delete env.BACKLOG_CWD; // cwd is explicit; do not accidentally mutate another project.
  const { stdout } = await exec('backlog', args, { cwd: repo, env, timeout: 30000, maxBuffer: 4 * 1024 * 1024 });
  return stdout;
}
export function fingerprint(task) {
  return createHash('sha256').update(JSON.stringify([task.title, task.description, task.acceptanceCriteria?.map(a => a.text)])).digest('hex');
}
export function procedure(task) {
  const match = task.description?.match(PROC);
  if (!match) throw new Error(`${task.id}: no human procedure. Do not infer steps.`);
  const p = validateProposal(JSON.parse(match[1]));
  if (task.description.trim() !== describe(p).trim() || task.title !== p.title) throw new Error(`${task.id}: displayed procedure and structured steps differ. Ask the orchestrator to repair both before review.`);
  return p;
}
export function stateOf(task) {
  const notes = task.implementationNotes || '';
  const start = notes.indexOf(BEGIN), end = notes.indexOf(END);
  if (start < 0 || end < start) throw new Error(`${task.id}: missing handoff state; repair explicitly.`);
  const text = notes.slice(start + BEGIN.length, end).trim();
  const state = JSON.parse(text.replace(/^```json\n/, '').replace(/\n```$/, ''));
  if (state.version !== 1) throw new Error('Unsupported handoff state version');
  return state;
}
export function updatedNotes(task, state) {
  const notes = task.implementationNotes || '';
  const block = `${BEGIN}\n\`\`\`json\n${JSON.stringify(state, null, 2)}\n\`\`\`\n${END}`;
  if (!notes.includes(BEGIN)) return `${notes}\n\n${block}`.trim();
  return notes.slice(0, notes.indexOf(BEGIN)) + block + notes.slice(notes.indexOf(END) + END.length);
}
function text(value, label, max = 3000) {
  if (typeof value !== 'string' || !value.trim() || value.length > max || /[\x00-\x08\x0b-\x1f]/.test(value) || value.includes('```') || value.includes(BEGIN)) throw new Error(`Invalid ${label}`);
  return value.trim();
}
export function validateProposal(p) {
  const result = {};
  for (const field of ['title', 'parent', 'reason', 'alternatives', 'outcome', 'preparation', 'category']) result[field] = text(p[field], field, field === 'title' ? 160 : 3000);
  if (!idPattern.test(result.parent)) throw new Error('A valid originating Backlog task ID is required');
  if (!['physical-access', 'identity', 'permission', 'personal-judgment'].includes(p.category)) throw new Error('Not a human-only category');
  if (!Number.isInteger(p.minutes) || p.minutes < 1 || p.minutes > 480) throw new Error('Estimate must be 1–480 minutes');
  result.minutes = p.minutes;
  if (!Array.isArray(p.steps) || p.steps.length < 1 || p.steps.length > 20) throw new Error('Provide 1–20 minimal steps');
  result.steps = p.steps.map(s => Object.fromEntries(['action', 'expected', 'verification', 'stop'].map(k => [k, text(s[k], `step ${k}`)])));
  if (JSON.stringify(result).length > 18000) throw new Error('Procedure exceeds 18,000 characters; reduce the human intervention');
  return result;
}
export function describe(p) {
  return `Originating task: ${p.parent}\n\nWhy only you: ${p.reason}\n\nAlternatives considered: ${p.alternatives}\n\nUnlocks: ${p.outcome}\n\nEstimated active time: ${p.minutes} minutes\n\nPreparation: ${p.preparation}\n\n## Procedure\n\n` + p.steps.map((s, i) => `${i + 1}. ${s.action}\n   - Expected: ${s.expected}\n   - Verify: ${s.verification}\n   - Stop if: ${s.stop}`).join('\n\n') + `\n\n## Machine-readable procedure (keep in sync when revising)\n\n\`\`\`human-procedure\n${JSON.stringify(p, null, 2)}\n\`\`\``;
}
export function assertApproved(item) {
  if (item.state.acceptedHash !== fingerprint(item.task)) throw new Error(`${item.task.id}: procedure changed since acceptance. Review and accept again; do not continue.`);
}
export function parseAcceptance(input, source) {
  if (source !== 'interactive') return null;
  return input.trim().match(/^accept\s+([A-Z][A-Z0-9]*-\d+(?:\.\d+)*)[.!]?$/i)?.[1].toUpperCase() || null;
}

export class Handoffs {
  constructor(home = STATE_HOME, run = backlog) { this.home = home; this.run = run; }
  async register(repo) {
    repo = await realpath(repo);
    const file = join(this.home, 'projects.json');
    const projects = await readJSON(file, []);
    if (!projects.includes(repo)) await saveJSON(file, [...projects, repo]);
    return repo;
  }
  async get(repo, id) {
    if (!idPattern.test(id)) throw new Error('Invalid task ID');
    const data = JSON.parse(await this.run(repo, ['task', 'view', id, '--json']));
    const task = data.task;
    if (!task || data.schemaVersion !== 1) throw new Error('Unsupported Backlog JSON schema');
    return task;
  }
  async item(repo, id) {
    const task = await this.get(repo, id);
    if (!task.labels.includes('human-action')) throw new Error('Not a human-action task');
    return { repo, task, state: stateOf(task) };
  }
  async list(repo) {
    const data = JSON.parse(await this.run(repo, ['task', 'list', '--labels', 'human-action', '--exclude-status', 'Done', '--json']));
    if (data.schemaVersion !== 1 || !Array.isArray(data.tasks)) throw new Error('Unsupported Backlog list JSON');
    const result = [];
    for (const row of data.tasks) result.push(await this.item(repo, row.id));
    return result;
  }
  async all() {
    const rows = [];
    for (const repo of await readJSON(join(this.home, 'projects.json'), [])) rows.push(...await this.list(repo));
    return rows.sort((a, b) => (a.state.acceptedAt || a.state.createdAt).localeCompare(b.state.acceptedAt || b.state.createdAt));
  }
  async put(item, state, extra = []) {
    // No direct Backlog markdown writes. Serialize our writers across Pi processes.
    await this.run(item.repo, ['task', 'edit', item.task.id, '--notes', updatedNotes(item.task, state), ...extra]);
    return { ...item, state };
  }
  async propose(repo, raw, origin) {
    const p = validateProposal(raw);
    return locked(this.home, async () => {
      repo = await this.register(repo);
      await this.get(repo, p.parent); // Must be an existing task, not a fabricated relationship.
      const key = createHash('sha256').update(`${repo}\n${p.parent}\n${p.outcome.toLowerCase().replace(/\s+/g, ' ').trim()}`).digest('hex');
      const existing = (await this.list(repo)).find(i => i.state.key === key && i.state.phase !== 'verified');
      if (existing) return { ...existing, duplicate: true }; // Includes deferred/declined requests: do not nag.
      const tag = `human-key-${key.slice(0, 16)}`;
      await this.run(repo, ['task', 'create', p.title, '-d', describe(p), '-l', `human-action,${tag}`, '-s', 'To Do', '-a', '', '--ref', p.parent, '--ac', p.outcome, '--plain']);
      const rows = JSON.parse(await this.run(repo, ['task', 'list', '--labels', tag, '--json'])).tasks;
      if (rows.length !== 1) throw new Error('Could not identify created handoff task; inspect Backlog before retrying');
      const task = await this.get(repo, rows[0].id);
      const state = { version: 1, key, phase: 'proposed', createdAt: now(), origin, step: 0, evidence: [], history: [] };
      return this.put({ repo, task }, state);
    });
  }
  async decide(repo, id, decision, expectedHash) {
    return locked(this.home, async () => {
      const item = await this.item(repo, id), s = item.state;
      if (fingerprint(item.task) !== expectedHash) throw new Error('Task changed while you reviewed it. Please review again.');
      if (!['accept', 'defer', 'decline'].includes(decision)) throw new Error('Invalid decision');
      if (['ready', 'verified'].includes(s.phase)) throw new Error('Task is already awaiting verification or verified');
      if (s.phase === 'active' && decision !== 'accept') throw new Error('Pause the active walkthrough before deferring/declining');
      procedure(item.task); // Reject corrupt procedures even if a human clicked confirm.
      const same = s.acceptedHash === expectedHash;
      const phase = decision === 'accept' ? (same && s.phase === 'active' ? 'active' : 'accepted') : decision === 'defer' ? 'deferred' : 'declined';
      const updated = { ...s, phase, history: [...s.history, { action: decision, at: now(), hash: expectedHash }] };
      if (decision === 'accept') Object.assign(updated, { acceptedHash: expectedHash, acceptedAt: s.acceptedAt && same ? s.acceptedAt : now(), step: same ? s.step : 0, evidence: same ? s.evidence : [] });
      const saved = await this.put(item, updated, ['-a', decision === 'accept' ? '@human' : '']);
      await this.register(repo);
      return saved;
    });
  }
  async current() {
    const ref = await readJSON(join(this.home, 'current.json'), null);
    if (!ref) return null;
    const item = await this.item(ref.repo, ref.id);
    return ['active', 'paused'].includes(item.state.phase) ? item : null;
  }
  async startNext() {
    return locked(this.home, async () => {
      const current = await this.current();
      if (current) { assertApproved(current); return this.put(current, { ...current.state, phase: 'active' }); }
      const next = (await this.all()).find(i => i.state.phase === 'accepted');
      if (!next) return null;
      assertApproved(next);
      // Persist selection first: if interrupted, next call can recover from the accepted record.
      await saveJSON(join(this.home, 'current.json'), { repo: next.repo, id: next.task.id });
      return this.put(next, { ...next.state, phase: 'active' }, ['-s', 'In Progress', '--plan', 'Follow the user-accepted procedure one step at a time, record evidence and stop on deviations. Submit the outcome for orchestrator verification; do not complete the originating project task.']);
    });
  }
  async checkpoint(expectedStep, note, evidenceType, reference = '', advance = false) {
    return locked(this.home, async () => {
      const item = await this.current();
      if (!item || item.state.phase !== 'active') throw new Error('No active task. Use /walk next.');
      assertApproved(item);
      const p = procedure(item.task), s = item.state;
      if (expectedStep !== s.step + 1 || s.step >= p.steps.length) throw new Error('Stale step number or procedure already finished');
      if (!['self-reported', 'photo', 'measurement', 'automated'].includes(evidenceType)) throw new Error('Unknown evidence type');
      if (evidenceType !== 'self-reported' && !reference.trim()) throw new Error('Evidence reference is required; use self-reported if not independently observed');
      const record = { step: expectedStep, note: text(note, 'observation'), type: evidenceType, reference: reference.slice(0, 2000), advanced: advance, at: now() };
      if (s.evidence.length >= 200) throw new Error('Evidence limit reached; summarize with the orchestrator rather than expanding this task');
      return this.put(item, { ...s, step: s.step + (advance ? 1 : 0), evidence: [...s.evidence, record] });
    });
  }
  async pause() {
    return locked(this.home, async () => {
      const item = await this.current();
      if (!item) return null;
      return this.put(item, { ...item.state, phase: 'paused' });
    });
  }
  async finish(summary, expectedHash) {
    return locked(this.home, async () => {
      const item = await this.current();
      if (!item || item.state.phase !== 'active') throw new Error('No active task');
      assertApproved(item);
      if (expectedHash !== fingerprint(item.task)) throw new Error('Procedure changed during confirmation');
      if (item.state.step !== procedure(item.task).steps.length) throw new Error('Complete each step or pause; do not skip verification');
      return this.put(item, { ...item.state, phase: 'ready', summary: text(summary, 'completion summary'), completedAt: now() });
    });
  }
  async verify(repo, id, summary) {
    return locked(this.home, async () => {
      const item = await this.item(repo, id);
      if (item.state.phase !== 'ready') throw new Error('Not ready for verification');
      assertApproved(item);
      return this.put(item, { ...item.state, phase: 'verified', verifiedAt: now(), verification: text(summary, 'verification evidence') }, ['--check-ac', '1', '--final-summary', summary, '-s', 'Done']);
    });
  }
}
